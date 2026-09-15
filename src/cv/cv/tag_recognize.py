import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2


class TagRecognize(Node):
    def __init__(self):
        super().__init__('tag_recognize')
        self.get_logger().info('Tag 识别节点启动')

        # 发布者
        self.pub_ = self.create_publisher(String, '/cv/tag_recognize_topic', 10)

        # 打开摄像头
        self.cap_ = cv2.VideoCapture(2)
        if not self.cap_.isOpened():
            self.get_logger().error("无法打开tag摄像头，请检查连接。")
            self.timer_ = None
            return

        # 二维码检测器 + CLAHE 对比度增强对象
        self.detector_ = cv2.QRCodeDetector()
        self.clahe_ = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # 记录上一次发布的结果，避免重复刷屏
        self.last_data_ = None

        # 定时器：30 Hz 抓帧识别
        self.timer_ = self.create_timer(1.0 / 30.0, self.timer_callback)

    def detect_qrcode(self, frame, detector, clahe):
        """
        检测一帧图像中的二维码，返回解码文本。
        预处理：灰度化 + CLAHE 局部自适应增强，提高恶劣光照下的识别稳定性。
        """
        try:
            # 转为灰度图
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # 应用 CLAHE 增强局部对比度
            enhanced = clahe.apply(gray)
            # 使用增强后的图像进行二维码检测
            data, bbox, _ = detector.detectAndDecode(enhanced)
            if bbox is not None and data:
                data = data.replace("+", "")
                return data.strip()
        except cv2.error:
            # 忽略 OpenCV 内部错误（如无效轮廓）
            pass
        return None

    def timer_callback(self):
        ret, frame = self.cap_.read()
        if not ret or frame is None:
            self.get_logger().warn("读取摄像头帧失败", throttle_duration_sec=2.0)
            return

        data = self.detect_qrcode(frame, self.detector_, self.clahe_)
        if data is None:
            return

        # 去重：只有和上次结果不同才发布
        if data == self.last_data_:
            return

        self.last_data_ = data
        msg = String()
        msg.data = data
        self.pub_.publish(msg)
        self.get_logger().info(f'识别到 Tag: {data}')
        self.destroy_node()  # 识别到后立即退出节点, 因为只需要识别一次tag

        # 可选：显示画面便于调试
        # cv2.imshow("tag", frame)
        # cv2.waitKey(1)

    def destroy_node(self):
        if getattr(self, 'cap_', None) is not None:
            self.cap_.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TagRecognize()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
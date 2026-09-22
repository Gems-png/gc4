#!/usr/bin/env python3
"""tag_recognize — tag 相机认**普通 tag (二维码)**

认到一次就发一次 /cv/tag_recognize_topic (内容 = 那串数字), 然后自己退出节点。

相机: 板卡名含 'Integrated' 的那只 tag 相机 (设备名/分辨率/帧率都是 Minit.open_cap
的形参, 要换就在这儿传, 别写死 /dev/video2 —— 设备号会随插拔漂)。
算法和参数: 全在 qrcode 模块里 (灰度 + CLAHE + QRCodeDetector), 本文件不重复实现,
原来那个 detect_qrcode 已删 —— 同一份算法两处放就是两个地方能改。
参数要调就在这儿传覆盖值, 比如 QrReader(clahe_clip=3.0)。
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2

from Minit import open_cap
from qrcode import QrReader


class TagRecognize(Node):
    def __init__(self):
        super().__init__('tag_recognize')
        self.get_logger().info('Tag 识别节点启动')

        # 发布者
        self.pub_ = self.create_publisher(String, '/cv/tag_recognize_topic', 10)

        # 打开 tag 相机 (板卡名片段; 分辨率/帧率用 Minit 里的默认值)
        self.cap_ = open_cap('Integrated')
        if self.cap_ is None:
            self.get_logger().error('无法打开 tag 摄像头，请检查连接。')
            self.timer_ = None
            return

        # 二维码识别器 (detector + CLAHE 建一次反复用, 别每帧重建)
        self.reader_ = QrReader()

        # 定时器：30 Hz 抓帧识别
        self.timer_ = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap_.read()
        if not ret or frame is None:
            self.get_logger().warn('读取摄像头帧失败', throttle_duration_sec=2.0)
            return

        # read() 认到就发, 不挑位数 —— 跟原来的行为一致。
        # 要是哪天必须卡死 12 位 (下位机那条 0x03 帧是定长), 把下面这行换成
        # self.reader_.read_ok(frame) 就完了, 位数不对的自动当没认到。
        reading = self.reader_.read(frame)
        if reading is None:
            return

        msg = String()
        msg.data = reading.digits          # 发数字那串 (原来是把 '+' 去掉再发)
        self.pub_.publish(msg)
        self.get_logger().info(f'识别到 Tag: {reading.describe()}')
        self.destroy_node()  # 识别到后立即退出节点, 因为只需要识别一次tag

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

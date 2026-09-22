#!/usr/bin/env python3

# 相似三角形法测距: 只靠内参 fx/fy/cx/cy 和 tag 真实边长, 不用 solvePnP。
import os
import json
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy

import tkinter as tk


class CamPoseNode(Node):
    """
    订阅原始图像 -> 检测 AprilTag (36h11) -> 相似三角形法估计 tag 中心在相机系下的位置
    发布 /goal_position (mm, 相机系) 以及带调试信息的 image_result
    """

    def __init__(self):
        super().__init__('cam_pos_node')

        # ---------- 参数 ----------
        self.declare_parameter(
            'calib_file',
            os.path.join(get_package_share_directory('cv'), 'config', 'gc1080p.json'))
        self.declare_parameter('tag_size_mm', 40.0)
        self.declare_parameter('tag_id', 1)
        self.declare_parameter('sub_topic', '/camera/image_raw')
        self.declare_parameter('pub_topic', '/camera/image_result')
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('publish_rate', 10.0)

        calib_file = self.get_parameter('calib_file').value
        self.tag_size = self.get_parameter('tag_size_mm').value
        self.tag_id = self.get_parameter('tag_id').value
        if self.tag_id < 0:
            self.tag_id = None
        sub_topic = self.get_parameter('sub_topic').value
        pub_topic = self.get_parameter('pub_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # ---------- 加载内参 (只需 fx/fy/cx/cy) ----------
        self.inmtx, _ = self._load_intrinsics(calib_file)

        # ---------- 初始化工具 ----------
        self.detector = self._make_detector()

        # ---------- 状态变量 ----------
        self.last_publish_time = 0.0

        # ---------- 发布器和订阅器 ----------
        qos_sub = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(Image, sub_topic, self.image_callback, qos_sub)
        self.pub_image = self.create_publisher(Image, pub_topic, 10)
        self.goal_pub = self.create_publisher(PointStamped, '/goal_position', 10)

        self.get_logger().info(
            f'CamPoseNode(相似三角形) 启动: 订阅 {sub_topic}, 发布 {pub_topic}, '
            f'tag 边长 {self.tag_size} mm, ID={self.tag_id}'
        )

        # ---------- 创建 tkinter 文本显示窗口 ----------
        self.root = tk.Tk()
        self.root.title("CamPos Debug Info")
        self.root.geometry("600x300")

        self.info_label = tk.Label(self.root, text="等待数据...",
                                   justify=tk.LEFT, font=("Courier", 11),
                                   bg="black", fg="white")
        self.info_label.pack(padx=10, pady=10)

        # 窗口关闭时，需要优雅地退出 ROS 节点
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # fx = self.inmtx[0, 0]
        # fy = self.inmtx[1, 1]
        # cx = self.inmtx[0, 2]
        # cy = self.inmtx[1, 2]
        self.fx = 500
        self.fy = 500
        self.cx = 320
        self.cy = 240

        self.x_offset = 0
        self.y_offset = 0

    def on_closing(self):
        self.get_logger().info("关闭 tkinter 窗口，退出节点")
        self.destroy_node()
        rclpy.shutdown()
        self.root.quit()

    # ------------------ 内参加载 ------------------
    def _load_intrinsics(self, path):
        if not os.path.exists(path):
            raise FileNotFoundError(f'内参文件 {path} 不存在，请先标定生成。')
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        if 'camera_matrix' not in data:
            raise ValueError(f'{path} 中缺少 camera_matrix，不是有效内参文件。')
        inmtx = np.asarray(data['camera_matrix'], dtype=np.float64)
        distortion = np.asarray(data.get('distortion_coefficients', [0, 0, 0, 0, 0]), dtype=np.float64).ravel()
        self.get_logger().info(f'加载内参: fx={inmtx[0,0]:.2f}, fy={inmtx[1,1]:.2f}, '
                               f'cx={inmtx[0,2]:.2f}, cy={inmtx[1,2]:.2f}')
        return inmtx, distortion

    # ---------- 图像转换 ----------
    def _imgmsg_to_bgr(self, msg):
        """sensor_msgs/Image -> numpy BGR 图像, 支持 bgr8/rgb8/mono8 与 step 填充。"""
        data = np.frombuffer(msg.data, dtype=np.uint8)
        channels = 3 if msg.encoding in ('bgr8', 'rgb8') else 1
        row_bytes = msg.step if msg.step else msg.width * channels
        rows = [data[y * row_bytes: y * row_bytes + msg.width * channels]
                for y in range(msg.height)]
        img = np.stack(rows).reshape(msg.height, msg.width, channels)
        if channels == 1:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        elif msg.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    def _bgr_to_imgmsg(self, img, header=None):
        """numpy BGR 图像 -> sensor_msgs/Image。"""
        msg = Image()
        if header is not None:
            msg.header = header
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
        msg.height = int(img.shape[0])
        msg.width = int(img.shape[1])
        msg.encoding = 'bgr8'
        msg.step = int(img.shape[1] * 3)
        msg.data = np.ascontiguousarray(img, dtype=np.uint8).tobytes()
        return msg

    # ------------------ AprilTag 检测器 ------------------
    def _make_detector(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        params = cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        return cv2.aruco.ArucoDetector(dictionary, params)

    # ------------------ 核心处理 ------------------
    def image_callback(self, msg: Image):
        try:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_publish_time < 1.0 / self.publish_rate:
                return
            self.last_publish_time = now

            cv_image = self._imgmsg_to_bgr(msg)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray)

            selected = self._pick_tag(corners, ids)
            if selected is None:
                self.get_logger().debug('未检测到目标 tag')
                return

            tid, c = selected

            # 相似三角形法: tag 中心在相机系下的位置 [X, Y, Z] (mm)
            tag_in_cam = self._estimate_position_similar_triangle(c)
            if tag_in_cam is None:
                return
            X_mm, Y_mm, Z_mm = tag_in_cam

            # ---------- 发布目标位置 (mm, 相机系) ----------
            goal_msg = PointStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = self.frame_id
            goal_msg.point.x = float(X_mm)
            goal_msg.point.y = float(Y_mm)
            goal_msg.point.z = float(Z_mm)
            self.goal_pub.publish(goal_msg)

            # ---------- 调试显示 ----------
            info_text = (
                f"Tag ID: {tid}\n"
                f"方法: 相似三角形\n"
                f"相机 中心在tag系 (mm): [{X_mm:.1f}, {Y_mm:.1f}, {Z_mm:.1f}]\n"
                f"距离 (mm): {Z_mm:.1f}\n"
                f"中心点的像素距离：[{self.x_offset}, {self.y_offset}]\n"
                f"内参: fx={self.fx:.1f} fy={self.fy:.1f} \n"
                f"cx={self.cx:.1f} cy={self.cy:.1f}\n"
            )
            self.root.after(0, lambda: self.info_label.config(text=info_text))

            # ---------- 绘制部分 ----------
            cv_image = self._draw_debug(cv_image, c, Z_mm)

            img_msg = self._bgr_to_imgmsg(cv_image, msg.header)
            self.pub_image.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"🚨 回调崩溃: {e}")
            import traceback
            traceback.print_exc()

    # ------------------ 相似三角形法 ------------------
    def _estimate_position_similar_triangle(self, corners):
        """
        相似三角形法: 由 tag 的像素尺寸/质心估计 tag 中心在相机系下的位置 (mm)。

        针孔模型:
            Z = f * S / s          (f: 焦距 px, S: tag 真实边长 mm, s: tag 像素边长 px)
            X = (u - cx) / fx * Z  (质心偏离光轴的横向偏移)
            Y = (v - cy) / fy * Z  (纵向偏移)

        只用 fx/fy/cx/cy, 不依赖 solvePnP 的旋转解算, 距离更稳。
        """
        c = np.asarray(corners, dtype=np.float64).reshape(-1, 2)

        # 质心 (像素)
        uc, vc = c.mean(axis=0)

        # 平均边长 (像素): 用周长/4, 比单条边抗噪声
        perimeter = 0.0
        for i in range(4):
            perimeter += np.linalg.norm(c[(i + 1) % 4] - c[i])

        # 平均边长
        avg_edge_px = perimeter / 4.0

        if avg_edge_px < 1e-6:
            self.get_logger().error(f'相似三角形法: tag 像素边长异常 ({avg_edge_px:.3f} px)')
            return None



        # 沿光轴距离 (mm), 焦距取 fx/fy 平均
        f_avg = 0.5 * (self.fx + self.fy)
        Z_mm = f_avg * self.tag_size / avg_edge_px

        # 横向/纵向偏移 (mm), 这里处理世界坐标系和图像系不一致的问题
        Y_mm = (uc - self.cx) / self.fx * Z_mm
        X_mm = (vc - self.cy) / self.fy * Z_mm

        # X_mm, Y_mm的正负号已经是对的
        # 但是相机不方便调前后左右，所以提高变大系数

        k = 5
        X_mm *= k
        Y_mm *= k

        # 记录中心点的偏移量
        self.x_offset = uc - self.cx
        self.y_offset = vc - self.cy

        return np.array([X_mm, Y_mm, Z_mm])

    # ------------------ 绘制 ------------------
    def _draw_debug(self, img, corners, distance_mm):
        """画检测框、质心和距离文字。"""
        c = corners.reshape(-1, 2).astype(int)
        cv2.polylines(img, [c], True, (0, 255, 0), 2)
        centroid = c.mean(axis=0).astype(int)
        cv2.circle(img, tuple(centroid), 5, (0, 255, 255), -1)
        cv2.putText(img, f"{distance_mm:.0f} mm", (c[0][0] + 10, c[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        return img

    # ------------------ 辅助函数 ------------------
    @staticmethod
    def _polygon_area(c):
        p = c.reshape(-1, 2)
        x, y = p[:, 0], p[:, 1]
        return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))

    def _pick_tag(self, corners, ids):
        if ids is None or len(ids) == 0:
            return None
        best = None
        best_area = -1.0
        for i in range(len(ids)):
            tid = int(np.asarray(ids[i]).reshape(-1)[0])
            a = self._polygon_area(corners[i])
            if self.tag_id is not None and tid == self.tag_id:
                return tid, corners[i].reshape(-1, 2)
            if a > best_area:
                best_area = a
                best = (tid, corners[i].reshape(-1, 2))
        return best


def main(args=None):
    rclpy.init(args=args)
    node = CamPoseNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.root.update()   # 处理 tkinter 事件

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

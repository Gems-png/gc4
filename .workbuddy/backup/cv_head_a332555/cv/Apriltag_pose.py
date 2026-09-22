#!/usr/bin/env python3

import os
import json
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy

import tkinter as tk


class TagPoseNode(Node):
    """
    订阅原始图像 -> 检测 AprilTag (36h11) -> 计算相机在世界系下的坐标
    发布 /goal_position (目标坐标, mm) 给 IK 节点, 以及 image_result, pose, TF
    """

    def __init__(self):
        super().__init__('tag_pose_node')

        # ---------- 参数 ----------
        self.declare_parameter(
            'calib_file',
            os.path.join(get_package_share_directory('cv'), 'config', 'gc480p.json'))
        self.declare_parameter('tag_size_mm', 40.0)
        self.declare_parameter('tag_id', 1)
        self.declare_parameter('sub_topic', '/camera/image_raw')
        self.declare_parameter('pub_topic', '/camera/image_result')
        self.declare_parameter('frame_id', 'tag_world')
        self.declare_parameter('broadcast_tf', True)
        self.declare_parameter('publish_rate', 10.0)

        calib_file = self.get_parameter('calib_file').value
        self.tag_size = self.get_parameter('tag_size_mm').value
        self.tag_id = self.get_parameter('tag_id').value
        if self.tag_id < 0:
            self.tag_id = None
        sub_topic = self.get_parameter('sub_topic').value
        pub_topic = self.get_parameter('pub_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.broadcast_tf = self.get_parameter('broadcast_tf').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # ---------- 加载内参 ----------
        self.inmtx, self.distortion = self._load_intrinsics(calib_file)

        # ---------- 初始化工具 ----------
        self.detector = self._make_detector()

        # ---------- 状态变量 ----------
        self.R_TW = None          # 世界系 -> tag 系的旋转矩阵
        self.last_publish_time = 0.0
        self._corner_shift = None

        # ---------- 卡尔曼滤波器（使用 OpenCV 实现，针对 X,Y 滤波） ----------
        self.kalman = None          # 延迟初始化，等首次检测到有效坐标再创建
        self.filter_initialized = False

        # ---------- 发布器和订阅器 ----------
        qos_sub = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.sub = self.create_subscription(Image, sub_topic, self.image_callback, qos_sub)
        self.pub_image = self.create_publisher(Image, pub_topic, 10)

        # pose_Pub 有更详细的朝向信息
        self.pose_pub = self.create_publisher(PoseStamped, '/camera/pose', 10)
        self.goal_pub = self.create_publisher(PointStamped, '/goal_position', 10)

        if self.broadcast_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'TagPoseNode 启动: 订阅 {sub_topic}, 发布 {pub_topic}, '
            f'tag 边长 {self.tag_size} mm, ID={self.tag_id}'
        )

        # ---------- 创建 tkinter 文本显示窗口 ----------
        self.root = tk.Tk()
        self.root.title("TagPose Debug Info")
        self.root.geometry("600x350")

        self.info_label = tk.Label(self.root, text="等待数据...", 
                                justify=tk.LEFT, font=("Courier", 11), 
                                bg="black", fg="white")
        self.info_label.pack(padx=10, pady=10)

        # 窗口关闭时，需要优雅地退出 ROS 节点（可选）
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

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
        distortion = np.asarray(data.get('distortion_coefficients', [0,0,0,0,0]), dtype=np.float64).ravel()
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

    def _draw_world_axes(self, img, inmtx, distortion, R_TW, R, tvec, header=None, length=40.0, thickness=3):
        """在 tag 中心画世界系坐标轴: X红 Y绿 Z蓝，并返回绘制后的图像。"""
        origin_cam = tvec.reshape(3)      
        dirs_cam = R @ R_TW
        pts = [origin_cam]
        for k in range(3):
            pts.append(origin_cam + dirs_cam[:, k] * length)
        pts = np.float32(pts).reshape(-1, 1, 3)
        imgpts, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), inmtx, distortion)
        imgpts = imgpts.reshape(-1, 2).astype(int)
        o = tuple(imgpts[0])
        cv2.line(img, o, tuple(imgpts[1]), (0, 0, 255), thickness)   # X 红
        cv2.line(img, o, tuple(imgpts[2]), (0, 255, 0), thickness)   # Y 绿
        cv2.line(img, o, tuple(imgpts[3]), (255, 0, 0), thickness)   # Z 蓝
        return img

    # ------------------ 初始化卡尔曼滤波器 ------------------
    def _init_kalman_filter(self, x, y):
        """使用初始位置 x,y 初始化卡尔曼滤波器"""
        kalman = cv2.KalmanFilter(4, 4, 0)   # 状态维度4，测量维度4
        # 状态转移矩阵 F
        kalman.transitionMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)
        # 测量矩阵 H
        kalman.measurementMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)
        # 过程噪声协方差 Q（调大一点，响应更快）
        kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.05
        # 测量噪声协方差 R（根据抖动幅度调整）
        kalman.measurementNoiseCov = np.eye(4, dtype=np.float32) * 0.5
        # 后验误差协方差 P（初始较大）
        kalman.errorCovPre = np.eye(4, dtype=np.float32) * 100.0

        # 初始化状态向量 [x, y, vx, vy]
        state = np.array([[float(x)], [float(y)], [0.0], [0.0]], dtype=np.float32)
        kalman.statePre = state.copy()
        kalman.statePost = state.copy()

        # 保存上一次测量值用于计算速度测量
        self.last_measurement = state.copy()

        return kalman

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
            R_CT, tvec = self._solve_pose(c, self.inmtx, self.distortion, self.tag_size, self._corner_shift)

            if self.R_TW is None:
                self.R_TW = self._build_world_frame(R_CT)
                self.get_logger().info(f'首次检测到 tag ID={tid}, 世界坐标系已锁定。')

            # 原始位姿（mm）
            Camera_w = self.R_TW.T @ (-R_CT.T @ tvec)
            R_WC = self.R_TW.T @ R_CT.T
            quat = self._rot_to_quat(R_WC)

            # 提取坐标
            raw_x, raw_y, raw_z = Camera_w[0], Camera_w[1], Camera_w[2]

            # ======== 对 X、Y 进行卡尔曼滤波 ========
            if not self.filter_initialized:
                # 首次检测到有效位置（z 不为0），初始化滤波器
                if abs(raw_z) > 1.0:   # 确保有有效深度
                    self.kalman = self._init_kalman_filter(raw_x, raw_y)
                    self.filter_initialized = True
                    filtered_x, filtered_y = raw_x, raw_y   # 第一次用原始值
                    self.get_logger().info(f"卡尔曼滤波器已初始化，位置({raw_x:.1f}, {raw_y:.1f})")
                else:
                    # 如果 z 太小，可能位姿解算不可靠，暂不使用
                    filtered_x, filtered_y = raw_x, raw_y
            else:
                # 先预测
                pred = self.kalman.predict().copy()
                # 构建测量向量 [x, y, vx, vy]
                # 计算速度测量（上一次位置到本次位置的差值）
                last_pos = self.last_measurement[:2].flatten()
                dt = now - self.last_publish_time  # 近似，更准确可用实际时间戳
                if dt > 0:
                    vx_meas = (raw_x - last_pos[0]) / dt
                    vy_meas = (raw_y - last_pos[1]) / dt
                else:
                    vx_meas, vy_meas = 0.0, 0.0
                measurement = np.array([[raw_x], [raw_y], [vx_meas], [vy_meas]], dtype=np.float32)
                # 更新滤波器
                self.kalman.correct(measurement)
                # 获取滤波后的状态
                state = self.kalman.statePost
                filtered_x = state[0, 0]
                filtered_y = state[1, 0]
                # 保存本次位置用于下次速度计算
                self.last_measurement = measurement.copy()

            # ======== Z 轴不滤波，直接使用原始值 ========
            filtered_z = raw_z

            # 组合滤波后坐标（单位：mm）
            filtered_pos = np.array([filtered_x, filtered_y, filtered_z])

            # ---------- 发布目标位置 ----------
            goal_msg = PointStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = self.frame_id
            goal_msg.point.x = filtered_pos[0] * 2
            goal_msg.point.y = filtered_pos[1] * 2
            goal_msg.point.z = filtered_pos[2]
            self.goal_pub.publish(goal_msg)

            pose_msg = PoseStamped()
            pose_msg.header = goal_msg.header
            pose_msg.pose.position.x = filtered_pos[0] / 1000.0
            pose_msg.pose.position.y = filtered_pos[1] / 1000.0
            pose_msg.pose.position.z = filtered_pos[2] / 1000.0
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
            self.pose_pub.publish(pose_msg)

            if self.broadcast_tf:
                t = TransformStamped()
                t.header = goal_msg.header
                t.child_frame_id = 'camera_link'
                t.transform.translation.x = filtered_pos[0] / 1000.0
                t.transform.translation.y = filtered_pos[1] / 1000.0
                t.transform.translation.z = filtered_pos[2] / 1000.0
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]
                self.tf_broadcaster.sendTransform(t)

            # 计算各轴向量用于显示
            x_world = np.array([1.0, 0.0, 0.0])
            x_tag = self.R_TW @ x_world
            x_camera = R_CT @ x_tag

            info_text = (
                f"shift: {self._corner_shift}\n"
                f"X_tag:    [{x_tag[0]:.2f}, {x_tag[1]:.2f}, {x_tag[2]:.2f}]\n"
                f"X_world:  [{x_world[0]:.2f}, {x_world[1]:.2f}, {x_world[2]:.2f}]\n"
                f"X_camera: [{x_camera[0]:.2f}, {x_camera[1]:.2f}, {x_camera[2]:.2f}]\n"
                f"Tag ID: {tid}\n"
                f"Raw pos (mm):   [{Camera_w[0]:.1f}, {Camera_w[1]:.1f}, {Camera_w[2]:.1f}]\n"
                f"Filtered pos(mm):[{filtered_pos[0]:.1f}, {filtered_pos[1]:.1f}, {filtered_pos[2]:.1f}]\n"
                f"tvec (tag->cam, mm): [{tvec[0]:.1f}, {tvec[1]:.1f}, {tvec[2]:.1f}]"
            )
            self.root.after(0, lambda: self.info_label.config(text=info_text))

            # 绘制世界轴和标签等
            cv_image = self._draw_world_axes(cv_image, self.inmtx, self.distortion,
                                             self.R_TW, R_CT, tvec, header=msg.header)
            top_left = c[self._corner_shift].astype(int)
            cv2.circle(cv_image, tuple(top_left), 8, (0, 255, 255), -1)
            cv2.putText(cv_image, "左上", (top_left[0] + 15, top_left[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            img_msg = self._bgr_to_imgmsg(cv_image, msg.header)
            self.pub_image.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"🚨 回调崩溃: {e}")
            import traceback
            traceback.print_exc()

    # ------------------ 辅助函数（保持不变） ------------------
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

    def _solve_pose(self, corners, inmtx, distortion, tag_size_mm=None, shift=None):
        if tag_size_mm is None:
            tag_size_mm = self.tag_size
        if tag_size_mm is None:
            tag_size_mm = 50.0
            self.get_logger().warning("tag_size_mm 为 None，使用默认值 50.0 mm")
        try:
            tag_size_mm = float(tag_size_mm)
        except (TypeError, ValueError):
            self.get_logger().error(f"tag_size_mm 类型错误: {type(tag_size_mm)}，使用默认值 50.0")
            tag_size_mm = 50.0

        obj_pts = np.array([[-0.5,  0.5, 0.0],
                            [ 0.5,  0.5, 0.0],
                            [ 0.5, -0.5, 0.0],
                            [-0.5, -0.5, 0.0]], dtype=np.float64) * tag_size_mm

        img_pts = corners.reshape(-1, 2).astype(np.float64)

        if shift is None:
            best_error = float('inf')
            best_shift = 0
            for s in range(4):
                shifted = np.roll(img_pts, -s, axis=0)
                error = -(shifted[1, 0] - shifted[0, 0])
                if error < best_error:
                    best_error = error
                    best_shift = s
            shifted = np.roll(img_pts, -best_shift, axis=0)
            ok, rvec, tvec = cv2.solvePnP(obj_pts, shifted, inmtx, distortion, flags=0)
            if not ok:
                raise RuntimeError('solvePnP 失败')
            R, _ = cv2.Rodrigues(rvec)
            self._corner_shift = best_shift
            self.get_logger().info(f'锁定角点移位为 {best_shift}')
            return R, tvec.reshape(3)
        else:
            shifted = np.roll(img_pts, -shift, axis=0)
            ok, rvec, tvec = cv2.solvePnP(obj_pts, shifted, inmtx, distortion, flags=0)
            if not ok:
                raise RuntimeError('solvePnP 失败')
            R, _ = cv2.Rodrigues(rvec)
            return R, tvec.reshape(3)

    @staticmethod
    def _build_world_frame(R_CT):
        X_cam_init = np.array([0, -1, 0], dtype=np.float64)
        n_proj = np.linalg.norm(X_cam_init)
        X_tag_Init = R_CT.T @ X_cam_init
        X_tag_Init /= n_proj
        candidates = np.array([[1.0, 0.0, 0.0],
                               [0.0, 1.0, 0.0],
                               [0.0, -1.0, 0.0],
                               [-1.0, 0.0, 0.0]])
        best_idx = 0
        best_dot = -1.0
        for i, axis in enumerate(candidates):
            dot = np.dot(axis, X_tag_Init)
            if dot > best_dot:
                best_dot = dot
                best_idx = i
        x_world = candidates[best_idx]
        z_world = np.array([0.0, 0.0, 1.0])
        y_world = np.cross(z_world, x_world)
        norm_y = np.linalg.norm(y_world)
        if norm_y < 1e-9:
            raise RuntimeError("X 轴与 Z 轴平行，无法构建 Y 轴。")
        y_world /= norm_y
        R_TW = np.column_stack([x_world, y_world, z_world])
        return R_TW

    @staticmethod
    def _rot_to_quat(R):
        R = np.asarray(R, dtype=np.float64)
        tr = np.trace(R)
        if tr > 0:
            s = np.sqrt(tr + 1.0) * 2.0
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return np.array([x, y, z, w])


def main(args=None):
    rclpy.init(args=args)
    node = TagPoseNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.root.update()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
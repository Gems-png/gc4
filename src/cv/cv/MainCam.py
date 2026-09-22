#!/usr/bin/env python3
"""MainCam — 主相机节点 (缩编合并版)

原先 4 个独立节点 (raw_image_pub / cam_pos / Apriltag_pose / Apriltag_image_pub)
全部收编进本文件: 取图发布 + AprilTag 位姿解算 (PnP / 相似三角形两种方法),
位姿解算的具体方法以 self 方法形式挂在节点上, 通用算法通过 import 引入。

发布:
    /camera/image_raw    原始 (或合成) 图像
    /camera/image_result 带调试标注的图像
    /goal_position       tag/物料 目标位置 (mm, 相机系)
    /camera/pose         相机在 tag 世界系下的位姿 (仅 PnP 法)
    TF camera_link <- tag_world (仅 PnP 法, 可关)

参数:
    use_sim: true=不开摄像头, 用合成 AprilTag 图像 (原 Apriltag_image_pub 功能)
    pose_method: 'pnp'=solvePnP 全位姿 (原 Apriltag_pose), 'similar'=相似三角形测距 (原 cam_pos)
"""

import os
import json
import time
import threading

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy

# ---- 外部算法模块: 通过 import 引入, 函数以节点实例(self)为入参 ----
import circle
from cv import material


class MainCam(Node):
    from Minit import open_cap


    def __init__(self):
        super().__init__('main_cam')

        # ---------- 相机参数 ----------
        self.declare_parameter('use_sim', False)
        self.declare_parameter('camera_id', 4)
        self.declare_parameter('freq', 15.0)
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)

        # ---------- tag / 位姿参数 ----------
        self.declare_parameter(
            'calib_file',
            os.path.join(get_package_share_directory('cv'), 'config', 'gc480p.json'))
        self.declare_parameter('pose_method', 'pnp')   # 'pnp' 或 'similar'
        self.declare_parameter('tag_size_mm', 40.0)
        self.declare_parameter('tag_id', 1)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('broadcast_tf', True)

        # ---------- 相似三角形法内参 (0 = 用 calib_file 的值) ----------
        self.declare_parameter('fx', 0.0)
        self.declare_parameter('fy', 0.0)
        self.declare_parameter('cx', 0.0)
        self.declare_parameter('cy', 0.0)

        self.use_sim = self.get_parameter('use_sim').value
        camera_id = self.get_parameter('camera_id').value
        self.freq = self.get_parameter('freq').value
        self.frame_id = self.get_parameter('frame_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value

        calib_file = self.get_parameter('calib_file').value
        self.pose_method = self.get_parameter('pose_method').value
        self.tag_size = self.get_parameter('tag_size_mm').value
        self.tag_id = self.get_parameter('tag_id').value
        if self.tag_id < 0:
            self.tag_id = None
        self.publish_rate = self.get_parameter('publish_rate').value
        self.broadcast_tf = self.get_parameter('broadcast_tf').value

        # ---------- 内参与检测器 ----------
        self.inmtx, self.distortion = self._load_intrinsics(calib_file)
        self.fx = self.get_parameter('fx').value or self.inmtx[0, 0]
        self.fy = self.get_parameter('fy').value or self.inmtx[1, 1]
        self.cx = self.get_parameter('cx').value or self.inmtx[0, 2]
        self.cy = self.get_parameter('cy').value or self.inmtx[1, 2]
        self.detector = self._make_detector()

        # ---------- PnP 位姿状态 ----------
        self.R_TW = None              # 世界系 -> tag 系旋转
        self._corner_shift = None
        self.kalman = None
        self.filter_initialized = False

        # ---------- 相似三角形状态 ----------
        self.x_offset = 0
        self.y_offset = 0

        # 供 circle / material 等外部模块通过 self 使用的上下文
        self.hsv = None                      # HSV 分割参数 (由配置或调试节点写入)
        self._material_detector = None

        # ---------- 发布器 ----------
        self.pub_raw = self.create_publisher(Image, '/camera/image_raw', 10)
        self.pub_image = self.create_publisher(Image, '/camera/image_result', 10)
        self.color_sub_ = self.create_subscription(Int32, '/target_color', self._color_callback, 10)
        self.goal_pub = self.create_publisher(PointStamped, '/goal_position', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/camera/pose', 10)
        if self.broadcast_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        self.last_process_time = 0.0

        # ---------- 相机 / 合成图 ----------
        self._sim_frame = None
        self.cap = None
        if self.use_sim:
            self._sim_frame = self._make_sim_frame()
            # 暂时不生成物料的合成图, 仅 AprilTag，因为ai不给我搞
            self.get_logger().info('MainCam 启动 (仿真模式): 发布合成 AprilTag 图像')
        else:
            self.cap = self.open_cap('2M')
            self.get_logger().info(f'MainCam 启动: 摄像头 {camera_id}, '
                                   f'位姿方法={self.pose_method}')

        # ---------- 取图线程 ----------
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    # ================= 相机 =================

    def _make_sim_frame(self):
        """合成一帧 AprilTag 图像 (原 Apriltag_image_pub 核心逻辑缩编)。"""
        tag_id = self.tag_id if self.tag_id is not None else 1
        marker_px = 320
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        marker = cv2.aruco.generateImageMarker(dictionary, tag_id, marker_px)
        frame = np.full((self.height, self.width, 3), 200, dtype=np.uint8)
        x0 = self.width // 2 - marker_px // 2
        y0 = self.height // 2 - marker_px // 2
        frame[y0:y0 + marker_px, x0:x0 + marker_px] = \
            cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        return frame

    def _capture_loop(self):
        interval = 1.0 / self.freq if self.freq > 0 else 0.0
        while self._running and rclpy.ok():
            start = time.time()

            if self.use_sim:
                frame = self._sim_frame
            else:
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    self.get_logger().warn('读取帧失败', throttle_duration_sec=2.0)
                    frame = None

            if frame is not None:
                self.pub_raw.publish(self._bgr_to_imgmsg(frame))

                # 位姿处理独立限频, 不拖累取图帧率
                now = time.time()
                if now - self.last_process_time >= 1.0 / max(self.publish_rate, 0.1):
                    self.last_process_time = now
                    try:
                        self.process_frame(frame)
                    except Exception as e:
                        self.get_logger().error(f'处理帧异常: {e}')

            if interval > 0:
                sleep_time = interval - (time.time() - start)
                if sleep_time > 0:
                    time.sleep(sleep_time)

    # ================= 核心处理 =================
    def process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        selected = self._pick_tag(corners, ids)

        # 物料/同心圆识别: 通过 import 引入的模块方法, 以 self 传入节点上下文
        if selected is None:
            circle.detect_circle_by_alt(self, frame)
            return

        tid, c = selected

        if self.pose_method == 'similar':
            self._process_similar_triangle(tid, c, frame)
        else:
            self._process_pnp(tid, c, frame)

    def detect_material(self, frame, color=None):
        """物料识别: 调用 import 引入的 material 模块, 供 circle 等方法复用。"""
        if self._material_detector is None:
            self._material_detector = material.MaterialDetector(
                material.MaterialSpec(), material.DepthCalib())
        reading, mask = self._material_detector.detect(frame, color)
        if reading is not None:
            self.get_logger().info(
                f"物料: {reading.color} 视角: {reading.view_mode} "
                f"距离: {reading.distance_mm}")
        return reading, mask

    # ------------------ 相似三角形法 (原 cam_pos) ------------------
    def _process_similar_triangle(self, tid, c, frame):
        tag_in_cam = self._estimate_position_similar_triangle(c)
        if tag_in_cam is None:
            return
        X_mm, Y_mm, Z_mm = tag_in_cam

        goal_msg = PointStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = self.frame_id
        goal_msg.point.x = float(X_mm)
        goal_msg.point.y = float(Y_mm)
        goal_msg.point.z = float(Z_mm)
        self.goal_pub.publish(goal_msg)

        cv_image = self._draw_debug(frame.copy(), c, Z_mm)
        self.pub_image.publish(self._bgr_to_imgmsg(cv_image))

    def _estimate_position_similar_triangle(self, corners):
        """
        相似三角形法: 由 tag 的像素尺寸/质心估计 tag 中心在相机系下的位置 (mm)。
            Z = f * S / s ;  X/Y 由质心偏离光轴的偏移反算
        只用 fx/fy/cx/cy, 不依赖 solvePnP 的旋转解算, 距离更稳。
        """
        c = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
        uc, vc = c.mean(axis=0)

        perimeter = 0.0
        for i in range(4):
            perimeter += np.linalg.norm(c[(i + 1) % 4] - c[i])
        avg_edge_px = perimeter / 4.0
        if avg_edge_px < 1e-6:
            self.get_logger().error(f'相似三角形法: tag 像素边长异常 ({avg_edge_px:.3f} px)')
            return None

        f_avg = 0.5 * (self.fx + self.fy)
        Z_mm = f_avg * self.tag_size / avg_edge_px

        # 图像系与世界系方向不一致的补偿
        Y_mm = (uc - self.cx) / self.fx * Z_mm
        X_mm = (vc - self.cy) / self.fy * Z_mm

        k = 5     # 相机不方便调前后左右, 用放大系数提高响应
        X_mm *= k
        Y_mm *= k

        self.x_offset = uc - self.cx
        self.y_offset = vc - self.cy
        return np.array([X_mm, Y_mm, Z_mm])

    # ------------------ PnP 全位姿 (原 Apriltag_pose) ------------------
    def _process_pnp(self, tid, c, frame):
        now = self.get_clock().now()
        R_CT, tvec = self._solve_pose(c, self.inmtx, self.distortion,
                                      self.tag_size, self._corner_shift)

        if self.R_TW is None:
            self.R_TW = self._build_world_frame(R_CT)
            self.get_logger().info(f'首次检测到 tag ID={tid}, 世界坐标系已锁定。')

        Camera_w = self.R_TW.T @ (-R_CT.T @ tvec)
        R_WC = self.R_TW.T @ R_CT.T
        quat = self._rot_to_quat(R_WC)

        raw_x, raw_y, raw_z = Camera_w[0], Camera_w[1], Camera_w[2]

        # 对 X/Y 卡尔曼滤波, Z 直接用原始值
        if not self.filter_initialized:
            if abs(raw_z) > 1.0:
                self.kalman = self._init_kalman_filter(raw_x, raw_y)
                self.filter_initialized = True
                self.get_logger().info(f'卡尔曼滤波器已初始化，位置({raw_x:.1f}, {raw_y:.1f})')
            filtered_x, filtered_y = raw_x, raw_y
        else:
            pred = self.kalman.predict().copy()
            _ = pred
            last_pos = self.last_measurement[:2].flatten()
            dt = now.nanoseconds / 1e9 - self.last_process_time
            if dt > 0:
                vx_meas = (raw_x - last_pos[0]) / dt
                vy_meas = (raw_y - last_pos[1]) / dt
            else:
                vx_meas, vy_meas = 0.0, 0.0
            measurement = np.array([[raw_x], [raw_y], [vx_meas], [vy_meas]],
                                   dtype=np.float32)
            self.kalman.correct(measurement)
            state = self.kalman.statePost
            filtered_x = state[0, 0]
            filtered_y = state[1, 0]
            self.last_measurement = measurement.copy()
        filtered_z = raw_z
        filtered_pos = np.array([filtered_x, filtered_y, filtered_z])

        # ---------- 发布目标位置 ----------
        goal_msg = PointStamped()
        goal_msg.header.stamp = now.to_msg()
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

        # ---------- 调试绘制 ----------
        cv_image = self._draw_world_axes(frame.copy(), self.inmtx, self.distortion,
                                         self.R_TW, R_CT, tvec)
        top_left = c[self._corner_shift].astype(int) if self._corner_shift is not None \
            else c[0].astype(int)
        cv2.circle(cv_image, tuple(top_left), 8, (0, 255, 255), -1)
        cv2.putText(cv_image, "top-left", (top_left[0] + 15, top_left[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        self.pub_image.publish(self._bgr_to_imgmsg(cv_image))

    def _color_callback(self, msg):
        self.get_logger().info(f"接收到目标颜色: {msg.data}")
        self.target_color = msg.data

    def _solve_pose(self, corners, inmtx, distortion, tag_size_mm=None, shift=None):
        """solvePnP 解算 tag 位姿; 首次自动锁定角点顺序 (shift)。"""
        if tag_size_mm is None:
            tag_size_mm = self.tag_size
        try:
            tag_size_mm = float(tag_size_mm)
        except (TypeError, ValueError):
            tag_size_mm = 50.0
            self.get_logger().warning(f'tag_size_mm 类型错误, 使用默认值 50.0')

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

        shifted = np.roll(img_pts, -shift, axis=0)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, shifted, inmtx, distortion, flags=0)
        if not ok:
            raise RuntimeError('solvePnP 失败')
        R, _ = cv2.Rodrigues(rvec)
        return R, tvec.reshape(3)

    def _init_kalman_filter(self, x, y):
        kalman = cv2.KalmanFilter(4, 4, 0)
        kalman.transitionMatrix = np.array([
            [1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float32)
        kalman.measurementMatrix = np.array([
            [1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float32)
        kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.05
        kalman.measurementNoiseCov = np.eye(4, dtype=np.float32) * 0.5
        kalman.errorCovPre = np.eye(4, dtype=np.float32) * 100.0
        state = np.array([[float(x)], [float(y)], [0.0], [0.0]], dtype=np.float32)
        kalman.statePre = state.copy()
        kalman.statePost = state.copy()
        self.last_measurement = state.copy()
        return kalman

    # ================= 工具方法 =================
    def _load_intrinsics(self, path):
        if not os.path.exists(path):
            raise FileNotFoundError(f'内参文件 {path} 不存在，请先标定生成。')
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        if 'camera_matrix' not in data:
            raise ValueError(f'{path} 中缺少 camera_matrix，不是有效内参文件。')
        inmtx = np.asarray(data['camera_matrix'], dtype=np.float64)
        distortion = np.asarray(data.get('distortion_coefficients', [0, 0, 0, 0, 0]),
                                dtype=np.float64).ravel()
        self.get_logger().info(f'加载内参: fx={inmtx[0, 0]:.2f}, fy={inmtx[1, 1]:.2f}, '
                               f'cx={inmtx[0, 2]:.2f}, cy={inmtx[1, 2]:.2f}')
        return inmtx, distortion

    def _make_detector(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        params = cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        return cv2.aruco.ArucoDetector(dictionary, params)

    def _bgr_to_imgmsg(self, img, header=None):
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

    @staticmethod
    def _polygon_area(c):
        p = c.reshape(-1, 2)
        x, y = p[:, 0], p[:, 1]
        return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))

    def _pick_tag(self, corners, ids):
        if ids is None or len(ids) == 0:
            return None
        best, best_area = None, -1.0
        for i in range(len(ids)):
            tid = int(np.asarray(ids[i]).reshape(-1)[0])
            a = self._polygon_area(corners[i])
            if self.tag_id is not None and tid == self.tag_id:
                return tid, corners[i].reshape(-1, 2)
            if a > best_area:
                best_area = a
                best = (tid, corners[i].reshape(-1, 2))
        return best

    def _draw_debug(self, img, corners, distance_mm):
        c = corners.reshape(-1, 2).astype(int)
        cv2.polylines(img, [c], True, (0, 255, 0), 2)
        centroid = c.mean(axis=0).astype(int)
        cv2.circle(img, tuple(centroid), 5, (0, 255, 255), -1)
        cv2.putText(img, f"{distance_mm:.0f} mm", (c[0][0] + 10, c[0][1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        return img

    def _draw_world_axes(self, img, inmtx, distortion, R_TW, R, tvec,
                         length=40.0, thickness=3):
        """在 tag 中心画世界系坐标轴: X红 Y绿 Z蓝。"""
        origin_cam = tvec.reshape(3)
        dirs_cam = R @ R_TW
        pts = [origin_cam]
        for k in range(3):
            pts.append(origin_cam + dirs_cam[:, k] * length)
        pts = np.float32(pts).reshape(-1, 1, 3)
        imgpts, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), inmtx, distortion)
        imgpts = imgpts.reshape(-1, 2).astype(int)
        o = tuple(imgpts[0])
        cv2.line(img, o, tuple(imgpts[1]), (0, 0, 255), thickness)
        cv2.line(img, o, tuple(imgpts[2]), (0, 255, 0), thickness)
        cv2.line(img, o, tuple(imgpts[3]), (255, 0, 0), thickness)
        return img

    @staticmethod
    def _build_world_frame(R_CT):
        X_cam_init = np.array([0, -1, 0], dtype=np.float64)
        X_tag_Init = R_CT.T @ X_cam_init
        X_tag_Init /= np.linalg.norm(X_cam_init)
        candidates = np.array([[1.0, 0.0, 0.0],
                               [0.0, 1.0, 0.0],
                               [0.0, -1.0, 0.0],
                               [-1.0, 0.0, 0.0]])
        best_idx = int(np.argmax(candidates @ X_tag_Init))
        x_world = candidates[best_idx]
        z_world = np.array([0.0, 0.0, 1.0])
        y_world = np.cross(z_world, x_world)
        norm_y = np.linalg.norm(y_world)
        if norm_y < 1e-9:
            raise RuntimeError("X 轴与 Z 轴平行，无法构建 Y 轴。")
        y_world /= norm_y
        return np.column_stack([x_world, y_world, z_world])

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

    # ================= 生命周期 =================
    def destroy_node(self):
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MainCam()
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

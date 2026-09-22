#!/usr/bin/env python3
"""MainCam — 主相机节点 (缩编合并版)

原先 4 个独立节点 (raw_image_pub / cam_pos / Apriltag_pose / Apriltag_image_pub)
全部收编进本文件, 现在再往下分: 本文件只管**取图 + 把各算法模块的结果发出去**,
算法和它们的参数都在各自的功能模块里 (见下)。

发布:
    /camera/image_raw    原始 (或合成) 图像
    /camera/image_result 带调试标注的图像
    /goal_position       tag 目标位置 (mm, 相机系)
    /camera/pose         相机在 tag 世界系下的位姿 (仅 pnp 法)
    TF camera_link <- tag_world (仅 pnp 法, 可关)

参数 (只留本节点自己说了算的; 功能模块的旋钮在各自的文件里):
    use_sim: true=不开摄像头, 用合成 AprilTag 图像
    frame_id: 发布消息的 frame_id
    pose_method: AprilTag 位姿算法 ''=用 apriltag 模块的默认, 或 'pnp' / 'similar'
    publish_rate / broadcast_tf: 处理限频 / 发不发 TF

参数都在哪儿 (同一个旋钮只在一个地方能改):
    相机   (设备名/分辨率/帧率/格式)  Minit.py     —— open_cap 的形参, 默认值 DEFAULT_SIZE/DEFAULT_FPS
    圆     (Hough 方法/半径/合并...)  circle.py    —— CircleParams
    AprilTag (方法/tag 边长/id/内参文件...)  apriltag.py —— AprilTagParams, 标定文件由它自己找
    普通 tag (二维码)                 qrcode.py    —— QrParams (tag_recognize 节点用, 不在这儿)
    内参   (fx/fy/cx/cy/畸变)         config/gc480p.json —— apriltag 模块自己解析, 节点不碰
"""

import time
import threading

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped
from tf2_ros import TransformBroadcaster

# ---- 外部算法模块: 通过 import 引入 ----
# 都是纯函数模块 (参数封在模块里, 不 declare 到节点上), 直接 import 成方法用。
# material 暂时留空 (material.py 里只有 docstring), 什么都没 import —— 要用时再加回来。
import circle
import apriltag
from Minit import open_cap, DEFAULT_SIZE, DEFAULT_FPS
from circle import find_circles
from apriltag import TagTracker


class MainCam(Node):
    # 这两个是纯函数模块, 参数都封在模块自己里面, 这里 import 成方法用。
    # **必须 staticmethod**: 普通函数挂到类上会变成"绑定方法", 调用时 self 会被当成
    # 第一个位置参数传进去 —— self.open_cap('2M') 会变成 name_fragment=self、width='2M',
    # 一启动就在 find_dev 里炸 (self 没有 .lower)。
    open_cap = staticmethod(open_cap)                 # 相机 (参数见 Minit)
    find_circles = staticmethod(find_circles)         # 地面同心圆 (参数见 circle)

    def __init__(self):
        super().__init__('main_cam')

        # ---------- 本节点自己的参数 ----------
        # 相机/圆/AprilTag/内参的旋钮都**不在这儿** —— 在各自的模块文件里 (见上面的表),
        # 节点上只留本文件自己说了算的, 免得同一个旋钮两处都能改。
        self.declare_parameter('use_sim', False)
        self.declare_parameter('frame_id', 'camera_frame')
        # '' = 用 apriltag 模块的默认方法; 要按 launch 选 pnp/similar 就在这儿覆盖。
        # 默认值写 ''(而不是 'pnp')是有意的: 默认值只该有一个出处 —— apriltag 模块里。
        self.declare_parameter('pose_method', '')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('broadcast_tf', True)

        self.use_sim = self.get_parameter('use_sim').value
        self.frame_id = self.get_parameter('frame_id').value
        self.pose_method = self.get_parameter('pose_method').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.broadcast_tf = self.get_parameter('broadcast_tf').value

        # ---------- AprilTag: 检测 + 位姿全在 apriltag 模块里 ----------
        # 内参、tag 边长/id、世界系锁定、卡尔曼滤波、角点顺序都在 tracker 里, 节点不存。
        self.tracker = TagTracker(method=self.pose_method)
        self.get_logger().info(f'AprilTag 参数: {self.tracker.params.describe()}')
        self._world_locked_logged = False

        # 同心圆的参数打在日志里一次就够 (circle 模块自己算, 节点不存它的上下文)
        self._circle_params_logged = False

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
        # 设备名是 open_cap 唯一的必给参数 (板卡名片段, 不像 /dev/videoN 会随插拔漂移);
        # 分辨率/帧率/格式走 Minit 里的默认值 DEFAULT_SIZE / DEFAULT_FPS。
        self._sim_frame = None
        self.cap = None
        if self.use_sim:
            tag_id = self.tracker.params.tag_id
            if self.tracker.params.wants_largest:
                tag_id = apriltag.DEFAULT_TAG_ID
            self._sim_frame = apriltag.make_tag_image(*DEFAULT_SIZE, tag_id=tag_id)
            # 暂时不生成物料的合成图, 仅 AprilTag，因为ai不给我搞
            self.get_logger().info('MainCam 启动 (仿真模式): 发布合成 AprilTag 图像')
        else:
            self.cap = self.open_cap('2M')
            self.get_logger().info(f'MainCam 启动: 摄像头 2M, 位姿方法={self.tracker.params.method}')

        # ---------- 取图线程 ----------
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    # ================= 取图 =================

    def _capture_loop(self):
        # 节拍按**相机实际协商到的帧率**走 (原来有个 freq 参数, 跟 open_cap 的 fps 是同一个
        # 旋钮; 现在直接问相机, 驱动自己降帧/换分辨率都不影响)。仿真模式没相机, 用 Minit
        # 里的默认帧率。
        fps = DEFAULT_FPS
        if self.cap is not None:
            got = float(self.cap.get(cv2.CAP_PROP_FPS))
            if got > 0.1:
                fps = got
        interval = 1.0 / fps
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
        # 认 tag 加算位姿都在 apriltag 模块里, 一句就够 (参数也归它)
        pose = self.tracker.track(frame)

        # 没认到 tag 就找地面的同心圆 (circle 模块, 参数封在模块里)
        if pose is None:
            self._process_circle(frame)
            return

        if self.tracker.world_locked and not self._world_locked_logged:
            self.get_logger().info(f'首次检测到 tag ID={pose.tag_id}, 世界坐标系已锁定。')
            self._world_locked_logged = True

        self._publish_goal(pose)
        if pose.method == 'pnp':        # 只有 pnp 法有姿态, 才有 /camera/pose 和 TF
            self._publish_pose_and_tf(pose)

        self.get_logger().info(f'AprilTag: {pose.describe()}', throttle_duration_sec=2.0)
        self.pub_image.publish(self._bgr_to_imgmsg(self.tracker.draw_debug(frame, pose)))

    # ------------------ 目标位置 ------------------

    def _publish_goal(self, pose):
        goal_msg = PointStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = self.frame_id
        # goal 的口径(要不要放大 x/y)归 apriltag 模块说了算 —— pose.goal_mm 已经算好了,
        # 节点别再自己乘一个系数, 那就是同一个旋钮两处能改。
        goal_msg.point.x = float(pose.goal_mm[0])
        goal_msg.point.y = float(pose.goal_mm[1])
        goal_msg.point.z = float(pose.goal_mm[2])
        self.goal_pub.publish(goal_msg)

    def _publish_pose_and_tf(self, pose):
        # 两条消息共用一个时间戳 —— 下游做 TF 的时候时间是同一拍
        stamp = self.get_clock().now().to_msg()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.frame_id
        # mm -> m (ROS 的规矩)
        pose_msg.pose.position.x = pose.x / 1000.0
        pose_msg.pose.position.y = pose.y / 1000.0
        pose_msg.pose.position.z = pose.z / 1000.0
        q = pose.quat
        pose_msg.pose.orientation.x = float(q[0])
        pose_msg.pose.orientation.y = float(q[1])
        pose_msg.pose.orientation.z = float(q[2])
        pose_msg.pose.orientation.w = float(q[3])
        self.pose_pub.publish(pose_msg)

        if self.broadcast_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.frame_id
            t.child_frame_id = 'camera_link'
            t.transform.translation.x = pose.x / 1000.0
            t.transform.translation.y = pose.y / 1000.0
            t.transform.translation.z = pose.z / 1000.0
            t.transform.rotation.x = float(q[0])
            t.transform.rotation.y = float(q[1])
            t.transform.rotation.z = float(q[2])
            t.transform.rotation.w = float(q[3])
            self.tf_broadcaster.sendTransform(t)

    # ------------------ 地面同心圆 (原 circle.detect_circle_by_alt) ------------------

    def _process_circle(self, frame):
        """找地面上的同心圆靶心。

        参数全在 circle 模块里 (default_params), 要调就在这儿传覆盖值, 比如
        self.find_circles(frame, param2=0.9) —— 不 declare 到节点上, 免得参数
        散得到处都是。
        """
        circles = self.find_circles(frame)
        if not circles:
            return

        if not self._circle_params_logged:      # 实际用的参数打一次, 方便对着调
            p = circle.default_params().resolved(*frame.shape[:2])
            self.get_logger().info(f'同心圆识别参数: {p.describe()}')
            self._circle_params_logged = True

        self.get_logger().info(f'同心圆 {len(circles)} 个, 最大: {circles[0].describe()}',
                               throttle_duration_sec=2.0)
        self.pub_image.publish(self._bgr_to_imgmsg(circle.draw_debug(frame, circles)))

    # 物料识别 (self.detect_material) 暂时空着 —— material.py 里只有 docstring。
    # 要做的时候按 circle 这个路子来: 纯函数 + 参数封在模块里, 这里 import 成方法调用,
    # 别再往节点上 declare 一批参数。

    def _color_callback(self, msg):
        self.get_logger().info(f"接收到目标颜色: {msg.data}")
        self.target_color = msg.data

    # ================= 工具方法 =================

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

#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class TagImagePublisher(Node):
    """
    发布合成 AprilTag (36h11) 图像到 /camera/image_raw，
    用于无实物 tag 时测试整条「tag->IK->串口」流水线。
    """

    def __init__(self):
        super().__init__('tag_image_pub')

        self.declare_parameter('tag_id', 1)
        self.declare_parameter('marker_px', 320)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('freq', 10.0)
        self.declare_parameter('frame_id', 'camera_frame')
        self.declare_parameter('center', [0.5, 0.5])   # tag 中心在图像中的归一化位置

        tag_id = self.get_parameter('tag_id').value
        marker_px = self.get_parameter('marker_px').value
        w = self.get_parameter('width').value
        h = self.get_parameter('height').value
        freq = self.get_parameter('freq').value
        self.frame_id = self.get_parameter('frame_id').value
        cx, cy = self.get_parameter('center').value

        # 预生成一帧: 灰底 + 一个 AprilTag
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        marker = cv2.aruco.generateImageMarker(dictionary, tag_id, marker_px)
        frame = np.full((h, w, 3), 200, dtype=np.uint8)
        x0 = int(w * cx) - marker_px // 2
        y0 = int(h * cy) - marker_px // 2
        x0 = max(0, min(x0, w - marker_px))
        y0 = max(0, min(y0, h - marker_px))
        frame[y0:y0 + marker_px, x0:x0 + marker_px] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
        self.frame = frame

        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(1.0 / freq, self.timer_callback)
        self.get_logger().info(
            f'合成 tag 图像发布节点启动: tag_id={tag_id}, 图像 {w}x{h}, '
            f'marker {marker_px}px, 位置 ({cx},{cy}), 频率 {freq} Hz'
        )

    def timer_callback(self):
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = self.frame.shape[0]
        msg.width = self.frame.shape[1]
        msg.encoding = 'bgr8'
        msg.step = self.frame.shape[1] * 3
        msg.data = self.frame.tobytes()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TagImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

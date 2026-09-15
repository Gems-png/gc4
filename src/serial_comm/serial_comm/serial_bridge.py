#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import JointState
import struct
import crcmod
import time
import math

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.sub_joint = self.create_subscription(
            Float32, '/joint', self.joint_callback, 10)
        self.sub_plate = self.create_subscription(
            Bool, '/plate', self.plate_callback, 10)
        self.sub_gripper = self.create_subscription(
            Bool, '/gripper', self.gripper_callback, 10)
        self.sub_tag = self.create_subscription(
            String, '/tag', self.tag_callback, 10)
        self.sub_rx = self.create_subscription(
            String, '/rx', self.rx_callback, 10)
        self.pub_tx = self.create_publisher(String, '/tx', 10)
        self.pub_joint_states = self.create_publisher(JointState, '/real_joint_states', 10)
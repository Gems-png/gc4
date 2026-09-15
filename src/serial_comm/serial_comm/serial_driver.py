#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32
import serial
import struct
import crcmod
import time
import math

# 协议常量
FRAME_HEADER = b'\xAA\x55'
FRAME_TAIL   = b'\x0D\x0A'
BAUDRATE = 115200

PORT = '/dev/ttyUSB0' # Linux 示例, 树梅派上好像是一样的


class SerialDriver(Node):
    def __init__(self):
        super().__init__('serial_driver')

        # 串口配置 (参数化)
        self.declare_parameter('simulate', False)
        self.simulate = self.get_parameter('simulate').value
        try:
            self.ser = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=1)
            self.get_logger().info(f'串口 {PORT} 已打开')
        except Exception as e:
            self.get_logger().error(
                f'串口 {PORT} 打开失败: {e}，进入 SIM 模式（仅打印帧，不真实发送）')
            self.simulate = True
            self.ser = None

        self.sub_ = self.create_subscription(String, '/tx', self.send_serial_data, 10)

    def send_serial_data(self, msg: String):
        """
        通用发送函数：打包并发送数据
        values: 浮点数列表
        帧格式：帧头 + 数据（浮点数小端打包） + CRC16（Modbus，小端） + 帧尾
        """
        # 限速
        now = time.time()
        if now - self.last_send_time < self.send_interval:
            return
        self.last_send_time = now

        try:
            # 打包所有浮点数（每个4字节，小端）
            data_pack = b''.join(struct.pack('<f', v) for v in msg.values)

            # CRC计算（对数据部分）
            crc = self.crc_func(data_pack)
            crc_bytes = struct.pack('<H', crc)

            # 组装完整帧
            frame = FRAME_HEADER + data_pack + crc_bytes + FRAME_TAIL


            if self.simulate or self.ser is None:
                # SIM 模式: 只打印帧, 不真实发送
                self.get_logger().info(
                    f'[SIM] 已发送帧，长度={len(frame)}字节, Hex: {frame.hex()}'
                )
            else:
                # 真实发送
                self.ser.write(frame)
                self.get_logger().info(
                    f'已发送帧，长度={len(frame)}字节, Hex: {frame.hex()}'
                )

        except Exception as e:
            self.get_logger().error(f'发送失败: {e}')

    def read_serial(self):
        """定时读取串口数据，解析完整帧并发布的通用读取串口函数"""
        if self.ser is None or not self.ser.is_open:
            return

        # 读取所有待处理数据
        if self.ser.in_waiting > 0:
            data = self.ser.read(self.ser.in_waiting)
            self.rx_buffer.extend(data)

        # 循环处理缓冲区中的完整帧
        while True:
            # 查找帧头位置
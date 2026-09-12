#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
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

class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')

        # 串口配置 (参数化)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('simulate', False)
        # 临时: 关节1 物理硬限位 (度), 变换后 clamp
        self.declare_parameter('joint1_limit_deg', 90.0)
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.simulate = self.get_parameter('simulate').value
        self.joint1_limit_deg = self.get_parameter('joint1_limit_deg').value
        try:
            self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
            self.get_logger().info(f'串口 {port} 已打开')
        except Exception as e:
            self.get_logger().error(
                f'串口 {port} 打开失败: {e}，进入 SIM 模式（仅打印帧，不真实发送）')
            self.simulate = True
            self.ser = None

        # --- 订阅 /joint_states ---
        self.sub_joint = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # --- 订阅 /gripper_status ---
        self.sub_gripper = self.create_subscription(
            Bool,
            '/gripper_cmd',
            self.gripper_callback,
            10
        )

        # --- 订阅 /plate_status ---
        self.sub_plate = self.create_subscription(
            Float32,
            '/plate_cmd',
            self.plate_callback,
            10
        )

        # CRC 函数
        self.crc_func = crcmod.predefined.mkCrcFun('modbus')

        # 限速
        self.last_send_time = 0.0
        self.send_interval = 0.05  # 50ms

        # 缓存最新的夹爪状态（默认为张开，即1.0）和物料盘状态（默认初始位置0）
        self.latest_gripper_value = 1.0
        self.latest_plate_value = 0.0
        self.latest_joint_positions = [0, 0, 0, 0]

        # 在 __init__ 中
        self.rx_buffer = bytearray()          # 接收缓冲区
        self.create_timer(0.01, self.read_serial)  # 每10ms轮询一次串口读取

        # 例如发布真实关节状态
        self.pub_real_joint = self.create_publisher(JointState, '/real_joint_states', 10)
        self.pub_real_gripper = self.create_publisher(Bool, '/real_gripper_status', 10)
        self.pub_real_plate = self.create_publisher(Float32, '/real_plate_status', 10)

    def joint_callback(self, msg):
        """处理关节状态：将关节数据 + 最新夹爪状态合并发送"""
        if not msg.position or len(msg.position) < 4:
            return

        # 取前4个关节位置
        self.latest_joint_positions = msg.position[:4]

        # 转换：第一个关节偏移并取反

        # 直接从tag得到的goal没有偏执，没有反向
        # self.latest_joint_positions[0] += 0.855
        # self.latest_joint_positions[0] *= -1

        # 临时硬限位: 关节1 只允许 ±joint1_limit_deg, 超限 clamp 并告警
        # limit = math.radians(self.joint1_limit_deg)
        # orig = self.latest_joint_positions[0]
        # self.latest_joint_positions[0] = max(-limit, min(limit, orig))
        # self.latest_joint_positions[0] = max(0, min(limit, orig))

        self.latest_joint_positions[1] = min(1.5708, self.latest_joint_positions[1])

        # if abs(orig) > limit + 1e-9:
        #     self.get_logger().warn(
        #         f'关节1 目标 {math.degrees(orig):.1f}° 超出限位 ±{self.joint1_limit_deg}°, '
        #         f'已 clamp 到 {math.degrees(self.latest_joint_positions[0]):.1f}°')

        data_to_send = list(self.latest_joint_positions) + [self.latest_gripper_value] + [self.latest_plate_value] # 共6个浮点数

        self.send_serial_data(data_to_send)

    def gripper_callback(self, msg):
        self.latest_gripper_value = 1.0 if msg.data else 0.0
        self.get_logger().info(f'夹爪状态已更新: {self.latest_gripper_value}')
        data_to_send = list(self.latest_joint_positions) + [self.latest_gripper_value] + [self.latest_plate_value] # 共6个浮点数
        self.send_serial_data(data_to_send)

    def plate_callback(self, msg):
        self.latest_plate_value = msg.data
        self.get_logger().info(f'物料盘状态已更新: {self.latest_plate_value}')

        data_to_send = list(self.latest_joint_positions) + [self.latest_gripper_value] + [self.latest_plate_value] # 共6个浮点数

        self.send_serial_data(data_to_send)

    def send_serial_data(self, values):
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
            data_pack = b''.join(struct.pack('<f', v) for v in values)

            # CRC计算（对数据部分）
            crc = self.crc_func(data_pack)
            crc_bytes = struct.pack('<H', crc)

            # 组装完整帧
            frame = FRAME_HEADER + data_pack + crc_bytes + FRAME_TAIL

            # 日志
            info = (f'关节: {[round(v, 3) for v in values[:4]]}, '
                    f'夹爪: {int(round(values[4]))},'
                    f'plate: {float(round(values[5]))}')

            if self.simulate or self.ser is None:
                # SIM 模式: 只打印帧, 不真实发送
                self.get_logger().info(
                    f'[SIM] 已发送帧，长度={len(frame)}字节, {info}, Hex: {frame.hex()}'
                )
            else:
                # 真实发送
                self.ser.write(frame)
                self.get_logger().info(
                    f'已发送帧，长度={len(frame)}字节, {info}'
                )

        except Exception as e:
            self.get_logger().error(f'发送失败: {e}')


    def read_serial(self):
        """定时读取串口数据，解析完整帧并发布"""
        if self.ser is None or not self.ser.is_open:
            return

        # 读取所有待处理数据
        if self.ser.in_waiting > 0:
            data = self.ser.read(self.ser.in_waiting)
            self.rx_buffer.extend(data)

        # 循环处理缓冲区中的完整帧
        while True:
            # 查找帧头位置
            header_pos = self.rx_buffer.find(FRAME_HEADER)
            if header_pos == -1:
                # 没有头，丢弃缓冲区（防止堆积无效数据）
                self.rx_buffer.clear()
                break

            # 移除头之前的数据（垃圾数据）
            if header_pos > 0:
                self.rx_buffer = self.rx_buffer[header_pos:]
                header_pos = 0

            # 至少需要 2(头)+4*N(数据)+2(CRC)+2(尾) 字节
            # 但数据长度未知，先尝试查找尾
            tail_pos = self.rx_buffer.find(FRAME_TAIL, 2)
            if tail_pos == -1:
                break   # 不完整的帧，等待更多数据

            # 计算总帧长 = tail_pos + len(FRAME_TAIL)
            frame_end = tail_pos + len(FRAME_TAIL)
            # 提取完整帧
            frame = self.rx_buffer[:frame_end]
            # 移除已处理帧
            self.rx_buffer = self.rx_buffer[frame_end:]

            # 解析该帧, 并发布
            self.parse_frame(frame)

    def parse_frame(self, frame):
        """解析单帧: 验证CRC, 提取浮点数列表, 并发布"""
        try:
            # 校验头部和尾部
            if not frame.startswith(FRAME_HEADER) or not frame.endswith(FRAME_TAIL):
                self.get_logger().warn('帧头尾错误，丢弃')
                return

            # 提取数据段（去掉头尾，并去掉最后的CRC）
            data_with_crc = frame[len(FRAME_HEADER):-len(FRAME_TAIL)]
            if len(data_with_crc) < 2:
                return

            # 分离数据部分和CRC（CRC占2字节，小端）
            data_bytes = data_with_crc[:-2]
            crc_received = struct.unpack('<H', data_with_crc[-2:])[0]

            # 计算CRC
            crc_calc = self.crc_func(data_bytes)
            if crc_received != crc_calc:
                self.get_logger().warn(f'CRC校验失败: 收到{crc_received}, 计算{crc_calc}')
                return

            # 解析浮点数（假设所有数据均为float32小端）
            num_floats = len(data_bytes) // 4
            values = struct.unpack('<{}f'.format(num_floats), data_bytes)
            # values 是元组，按需处理

            self.get_logger().info(f'接收解析得到 {num_floats} 个浮点数: {values}')

            # ---------- 根据您的协议映射发布 ----------
            # 假设下位机回复也是6个float：真实关节(4) + 真实夹爪(1) + 真实物料盘(1)
            # 您可根据实际协议调整
            if num_floats == 6:
                real_joint = list(values[:4])
                real_gripper = values[4]
                real_plate = values[5]
                # 发布到话题（需要先定义发布者）
                self.pub_real_joint.publish(JointState(position=real_joint))  # 需要构造JointState消息
                self.pub_real_gripper.publish(Bool(data=bool(round(real_gripper))))
                self.pub_real_plate.publish(Float32(data=real_plate))
            else:
                self.get_logger().warn(f'未知的数据长度: {num_floats}，请核对协议')

        except Exception as e:
            self.get_logger().error(f'解析帧异常: {e}')
    def __del__(self):
        if hasattr(self, 'ser') and self.ser is not None and self.ser.is_open:
            self.ser.close()
            if hasattr(self, 'get_logger'):
                self.get_logger().info('串口已关闭')


def main(args=None):
    rclpy.init(args=args)
    node = SerialPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""serial_driver —— 传输层（纯管道，不懂协议）

只做三件事：
  1. 打开并持有串口；
  2. 收到的字节原样发到 /rx（UInt8MultiArray）；
  3. 订阅 /tx（UInt8MultiArray），把字节原样写进串口。

协议解析、话题语义全部在 serial_bridge 里，本节点不认识任何帧结构。
这样分层的好处是 `ros2 topic echo /rx` 看到的就是串口上的真实字节流，
想抓帧调试直接 echo 就行。

为什么用 UInt8MultiArray 而不是 std_msgs/String 传字节：
rclpy 序列化 String 时会强制按 UTF-8 编码，latin-1 解出来的 0x80~0xFF
会被扩成两个字节，帧头 0xAA 直接变成 0xC2 0xAA —— 数据必坏。
UInt8MultiArray 才是字节流的正确类型。
"""

import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

import serial

DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUDRATE = 115200


class SerialDriver(Node):
    def __init__(self):
        super().__init__('serial_driver')

        self.declare_parameter('port', DEFAULT_PORT)
        self.declare_parameter('baudrate', DEFAULT_BAUDRATE)
        self.declare_parameter('simulate', False)
        self.declare_parameter('poll_hz', 100.0)        # 读串口的轮询频率
        self.declare_parameter('reconnect_period', 2.0)  # 掉线后重连间隔(s)
        self.declare_parameter('stats_period', 5.0)      # 流量统计打印间隔(s)

        self.port_ = self.get_parameter('port').value
        self.baudrate_ = self.get_parameter('baudrate').value
        self.simulate_ = self.get_parameter('simulate').value
        poll_hz = self.get_parameter('poll_hz').value
        self.reconnect_period_ = self.get_parameter('reconnect_period').value
        stats_period = self.get_parameter('stats_period').value

        self.ser = None
        self.write_lock = threading.Lock()   # 换多线程执行器时防止写撕裂
        self.rx_bytes = 0
        self.tx_bytes = 0

        self.pub_rx = self.create_publisher(UInt8MultiArray, '/rx', 50)
        self.sub_tx = self.create_subscription(
            UInt8MultiArray, '/tx', self.tx_callback, 50)

        if self.simulate_:
            self.get_logger().warn(
                f'[SIM] simulate=true，不打开串口 {self.port_}，只打印要发出的帧')
        else:
            self.open_serial()

        self.create_timer(1.0 / poll_hz, self.poll)
        self.create_timer(stats_period, self.log_stats)

    # ---------------- 串口开关 ----------------
    def open_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.port_,
                baudrate=self.baudrate_,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,          # 非阻塞读，靠 poll 轮询
                write_timeout=1.0,
            )
            self.get_logger().info(
                f'串口 {self.port_} 已打开 @ {self.baudrate_} 8N1')
            return True
        except Exception as e:
            self.ser = None
            self.get_logger().error(
                f'串口 {self.port_} 打开失败: {e}，'
                f'{self.reconnect_period_}s 后重试')
            return False

    def close_serial(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    # ---------------- 收 ----------------
    def poll(self):
        # 没开串口：sim 模式不用管，真串口模式就定期重连
        if self.ser is None:
            if not self.simulate_:
                self.open_serial()
            return

        try:
            waiting = self.ser.in_waiting
            if waiting <= 0:
                return
            data = self.ser.read(waiting)
        except Exception as e:
            self.get_logger().error(f'读串口失败: {e}，关闭后重连')
            self.close_serial()
            return

        if not data:
            return

        self.rx_bytes += len(data)
        self.pub_rx.publish(UInt8MultiArray(data=list(data)))

    # ---------------- 发 ----------------
    def tx_callback(self, msg: UInt8MultiArray):
        frame = bytes(msg.data)
        if not frame:
            return

        if self.simulate_ or self.ser is None:
            # SIM 模式 / 串口没开：只打印，不真实发送
            self.get_logger().info(f'[SIM] 发送 {len(frame)} 字节: {frame.hex()}')
            self.tx_bytes += len(frame)
            return

        try:
            with self.write_lock:
                self.ser.write(frame)
            self.tx_bytes += len(frame)
            self.get_logger().debug(f'已发送 {len(frame)} 字节: {frame.hex()}')
        except Exception as e:
            self.get_logger().error(f'写串口失败: {e}，关闭后重连')
            self.close_serial()

    # ---------------- 统计 ----------------
    def log_stats(self):
        if self.rx_bytes == 0 and self.tx_bytes == 0:
            return
        state = 'SIM' if (self.simulate_ or self.ser is None) else self.port_
        self.get_logger().info(
            f'[{state}] 收 {self.rx_bytes} 字节 / 发 {self.tx_bytes} 字节')
        self.rx_bytes = 0
        self.tx_bytes = 0

    def destroy_node(self):
        self.close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C 或被 launch/SIGTERM 收掉时，不要吐 traceback
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

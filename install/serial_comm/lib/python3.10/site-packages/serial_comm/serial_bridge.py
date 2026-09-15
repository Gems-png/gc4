#!/usr/bin/env python3
"""serial_bridge —— 协议层 + 语义层

帧格式（见 USART1_Protocol.md，AA55 统一协议）:

    帧头 2B      类型 1B   长度 1B   数据 N B
    0xAA 0x55     type     length   payload

    总长 = 4 + length，小端，无 CRC、无帧尾

当前只做机械臂这一组:

    发   0x04 关节信息  上→下  18B   ← /joint_states /gripper_cmd /plate_cmd
    发   0x03 Tag 数据  上→下  12B   ← /cv/tag_recognize_topic
    收   0x00 关节状态  下→上  18B   → /real_joint_states /real_gripper_status /real_plate_status

底盘组（0x01 / 0x02 / 0x05 / 0x10 / 0x11）暂时不实现，但解帧器认识它们，
只做丢弃；以后要做底盘，在 rx_callback 里加分支即可。

话题约定：不带 real_ 的是上位机指令（希望下位机到达的位置），
带 real_ 的是下位机回传的实际值，两者类型一一对应，方便下游比较。
"""

import struct

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, String, UInt8MultiArray

# ---------------------------------------------------------------
# 协议常量
# ---------------------------------------------------------------
FRAME_HEADER = b'\xAA\x55'

TYPE_JOINT_STATE = 0x00   # 关节状态  下→上  18B
TYPE_TAG = 0x03           # Tag 数据  上→下  12B
TYPE_JOINT_CMD = 0x04     # 关节信息  上→下  18B

# 类型 → 数据段长度（spec 第二节）。解帧靠它校验。
# 必须把全部合法类型都列进来：不认识的类型会被当成垃圾逐字节重同步，
# 而逐字节扫描 32 字节的里程计帧时，很可能在载荷里撞出假的 AA 55 帧头。
TYPE_LEN = {
    0x00: 18,   # 关节状态      下→上
    0x01: 32,   # 轮式里程计    下→上
    0x02: 32,   # IMU 数据      下→上（保留）
    0x03: 12,   # Tag 数据      上→下
    0x04: 18,   # 关节信息      上→下
    0x05: 12,   # 移速指令      上→下
    0x10: 12,   # 时间同步请求  上→下
    0x11: 28,   # 时间同步响应  下→上
}

JOINT_NUM = 4
TAG_LEN = 12              # 4 组 × 3 位 ASCII 数字
SERVO1_VALUES = (0, 1, 2, 3)   # 1/2/3 ↔ 0°/90°/270°，0 = 还没收到过指令


# ---------------------------------------------------------------
# 打包 / 解包（纯函数，不依赖 ROS，可以单独喂数据测）
# ---------------------------------------------------------------
def build_frame(type_id: int, payload: bytes) -> bytes:
    """拼一个完整帧：帧头 + 类型 + 长度 + 数据。"""
    return FRAME_HEADER + bytes([type_id, len(payload)]) + payload


def pack_joint(type_id: int, joints, gripper: int, servo1: int) -> bytes:
    """
    打包关节类数据（0x00 / 0x04 同构）:
        4×float32 关节角(rad) + uint8 夹爪 + uint8 转盘舵机 = 18 字节
    """
    payload = struct.pack(
        '<4fBB',
        float(joints[0]), float(joints[1]), float(joints[2]), float(joints[3]),
        int(gripper) & 0xFF, int(servo1) & 0xFF,
    )
    return build_frame(type_id, payload)


def unpack_joint(payload: bytes):
    """解出 (关节角列表, 夹爪, 转盘舵机)。"""
    j0, j1, j2, j3, gripper, servo1 = struct.unpack('<4fBB', payload)
    return [j0, j1, j2, j3], gripper, servo1


def joints_are_finite(joints) -> bool:
    """
    挡住 NaN / inf。

    协议里没有校验字节，NaN 是唯一会"静默通过"的坏值：
    IK 解不出解的时候会吐 NaN，而 NaN 参与任何比较都是 False，
    下位机就算做了范围检查也拦不住它。所以在发送前显式挡一道。
    """
    return all(j == j and j not in (float('inf'), float('-inf')) for j in joints)


def deframe(buf: bytearray):
    """
    从缓冲区里尽量多地取出完整帧，就地消费 buf。
    返回 [(type_id, payload), ...]。

    新协议没有 CRC，length 是唯一的帧边界依据，所以这里的规则要小心：
      * 找不到帧头 → 只保留最后 1 字节（它可能是 AA），其余丢弃；
      * 类型和长度对不上（含长度白名单外的值）→ 判定为假帧头，
        只丢 1 个字节重新找，**不能清空缓冲区**，
        否则会把"帧头还在路上"的半帧一起扔掉，从此永久丢帧；
      * 数据段还没收全 → 直接退出，等下一批数据。
    """
    frames = []
    while True:
        pos = buf.find(FRAME_HEADER)
        if pos < 0:
            del buf[:-1]        # 留 1 字节，防止 0xAA 被切断
            break

        if pos > 0:
            del buf[:pos]

        if len(buf) < 4:
            break               # 头和长度字段都没收全

        type_id, length = buf[2], buf[3]
        if TYPE_LEN.get(type_id) != length:
            del buf[0]          # 假帧头，退 1 字节重找
            continue

        if len(buf) < 4 + length:
            break               # 半帧，等数据

        frames.append((type_id, bytes(buf[4:4 + length])))
        del buf[:4 + length]

    return frames


def parse_tag(text: str):
    """
    把识别到的二维码文本整理成 12 位 ASCII 数字。
    不合法返回 None（不补零、不截断，宁可报错也不要发个错的 tag 下去）。
    """
    if text is None:
        return None
    data = text.replace('+', '').strip()
    if len(data) != TAG_LEN or not data.isdigit():
        return None
    return data.encode('ascii')


# ---------------------------------------------------------------
# 节点
# ---------------------------------------------------------------
class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # 0x03 没有 ACK，只能靠连发保证下位机收到，沿用旧 serial_tag 的连发 5 次
        self.declare_parameter('tag_repeat', 5)
        self.declare_parameter('tag_interval', 0.1)

        self.tag_repeat_ = self.get_parameter('tag_repeat').value
        tag_interval = self.get_parameter('tag_interval').value

        # 收：下位机 → 上位机
        self.rx_buf = bytearray()
        self.sub_rx = self.create_subscription(
            UInt8MultiArray, '/rx', self.rx_callback, 50)
        self.pub_real_joint = self.create_publisher(
            JointState, '/real_joint_states', 10)
        self.pub_real_gripper = self.create_publisher(
            Bool, '/real_gripper_status', 10)
        self.pub_real_plate = self.create_publisher(
            Float32, '/real_plate_status', 10)

        # 发：上位机 → 下位机
        self.tx_pub = self.create_publisher(UInt8MultiArray, '/tx', 10)
        self.sub_joint = self.create_subscription(
            JointState, '/joint_states', self.joint_cmd_callback, 10)
        self.sub_gripper = self.create_subscription(
            Bool, '/gripper_cmd', self.gripper_cmd_callback, 10)
        self.sub_plate = self.create_subscription(
            Float32, '/plate_cmd', self.plate_cmd_callback, 10)
        self.sub_tag = self.create_subscription(
            String, '/cv/tag_recognize_topic', self.tag_cmd_callback, 10)

        # 待发状态缓存：三个话题各自更新，谁变了就整帧重发一次
        self.cmd_joints = None
        self.cmd_gripper = 0        # 0=打开 1=合上（与 /gripper_cmd 的 Bool 同向）
        self.cmd_servo1 = 0         # 初值 0，收到 /plate_cmd 后变成 1/2/3
        self.last_tx_frame = None

        # 待发 tag：连发计数器
        self.tag_payload = None
        self.tag_left = 0
        self.create_timer(tag_interval, self.tag_timer_callback)

    # ---------------- 收：下位机 → 上位机 ----------------
    def rx_callback(self, msg: UInt8MultiArray):
        self.rx_buf.extend(msg.data)
        for type_id, payload in deframe(self.rx_buf):
            if type_id == TYPE_JOINT_STATE:
                self.handle_joint_state(payload)
            else:
                # 底盘组的包暂时不用，但已经从缓冲区正确消费掉了
                self.get_logger().debug(
                    f'忽略未处理的包: type=0x{type_id:02X}, len={len(payload)}')

    def handle_joint_state(self, payload: bytes):
        joints, gripper, servo1 = unpack_joint(payload)

        if not joints_are_finite(joints):
            self.get_logger().warn(
                '0x00 关节状态含 NaN/inf，整帧丢弃',
                throttle_duration_sec=2.0)
            return

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = 'base_link'
        js.name = ['joint1', 'joint2', 'joint3', 'joint4']
        js.position = joints
        self.pub_real_joint.publish(js)

        self.pub_real_gripper.publish(Bool(data=bool(gripper)))
        self.pub_real_plate.publish(Float32(data=float(servo1)))

    # ---------------- 发：上位机 → 下位机 ----------------
    def joint_cmd_callback(self, msg: JointState):
        if len(msg.position) < JOINT_NUM:
            self.get_logger().warn(
                f'/joint_states 只有 {len(msg.position)} 个关节，'
                f'至少需要 {JOINT_NUM} 个，忽略',
                throttle_duration_sec=2.0)
            return

        joints = list(msg.position[:JOINT_NUM])
        if not joints_are_finite(joints):
            self.get_logger().warn(
                '/joint_states 含 NaN/inf，本帧不发送'
                '（IK 解不出解时就是这里拦住）',
                throttle_duration_sec=2.0)
            return

        self.cmd_joints = joints
        self.send_if_changed()

    def gripper_cmd_callback(self, msg: Bool):
        # Bool: true=闭合 → 协议 uint8: 1=合上，方向一致，直接映射
        self.cmd_gripper = 1 if msg.data else 0
        self.send_if_changed()

    def plate_cmd_callback(self, msg: Float32):
        # /plate_cmd 传的就是 1/2/3，直接取整
        value = int(round(msg.data))
        if value not in SERVO1_VALUES:
            self.get_logger().warn(
                f'/plate_cmd 收到 {msg.data}，不是 0/1/2/3，按 0 发送',
                throttle_duration_sec=2.0)
            value = 0
        self.cmd_servo1 = value
        self.send_if_changed()

    def tag_cmd_callback(self, msg: String):
        """收到识别结果 → 排队连发 0x03（0x03 没有 ACK，连发是唯一的保底）。"""
        payload = parse_tag(msg.data)
        if payload is None:
            self.get_logger().warn(
                f'Tag 数据不合法（要求 {TAG_LEN} 位纯数字）: {msg.data!r}')
            return

        self.tag_payload = payload
        self.tag_left = self.tag_repeat_
        self.get_logger().info(
            f'收到 Tag: {payload.decode()!r}，连发 {self.tag_left} 次')

    def tag_timer_callback(self):
        if self.tag_left <= 0:
            return
        self.tag_left -= 1
        frame = build_frame(TYPE_TAG, self.tag_payload)
        self.tx_pub.publish(UInt8MultiArray(data=list(frame)))
        self.get_logger().debug(f'发送 0x03: {frame.hex()}')

    def send_if_changed(self):
        """0x04 按需发送：内容变了才发一帧，没变就静默。"""
        if self.cmd_joints is None:
            return      # 还没收到过 /joint_states，不发

        frame = pack_joint(
            TYPE_JOINT_CMD, self.cmd_joints, self.cmd_gripper, self.cmd_servo1)
        if frame == self.last_tx_frame:
            return      # 和上次发的一模一样，跳过

        self.last_tx_frame = frame
        self.tx_pub.publish(UInt8MultiArray(data=list(frame)))
        self.get_logger().debug(f'发送 0x04: {frame.hex()}')


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
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

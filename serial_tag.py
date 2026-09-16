import os
os.environ['OPENCV_LOG_LEVEL'] = 'ERROR'  # 必须在 import cv2 之前，压制 QUIRC 警告
import cv2

import serial
import time

# 改成你自己的串口
#PORT = 'COM3'          # Windows 示例
PORT = '/dev/ttyUSB0'  # Linux 示例
BAUDRATE = 115200

# ---------------------------------------------------------------
# AA55 统一协议（详见 USART1_Protocol.md）
#
#   帧头 2B   类型 1B   长度 1B   数据 N B
#   0xAA 0x55   type     length   payload
#
#   总长 = 4 + length，全部小端，无 CRC、无帧尾
# ---------------------------------------------------------------
FRAME_HEADER = b'\xAA\x55'
TYPE_TAG = 0x03     # Tag 数据（上→下）
DATA_LEN = 12       # 0x03 数据段固定 12 字节 ASCII 数字


def detect_qrcode(frame, detector, clahe):
    """
    检测一帧图像中的二维码，返回解码文本。
    预处理：灰度化 + CLAHE 局部自适应增强，提高恶劣光照下的识别稳定性。
    """
    try:
        # 转为灰度图
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 应用 CLAHE 增强局部对比度
        enhanced = clahe.apply(gray)
        # 使用增强后的图像进行二维码检测
        data, bbox, _ = detector.detectAndDecode(enhanced)
        if bbox is not None and data:
            return data.strip()
    except cv2.error:
        # 忽略 OpenCV 内部错误（如无效轮廓）
        pass
    return None


# ---------------------------------------------------------------
# 打包
# ---------------------------------------------------------------
def build_frame(data: str) -> bytes:
    """
    把 12 位 ASCII 数字打包成 0x03 Tag 数据帧。

    数据段：4 组 × 3 位，如 "123456789012"，共 12 字节。
    整帧固定长度：4 + 12 = 16 字节。
    """
    data = data.replace("+", "")
    data_bytes = data.encode('ascii')

    if len(data_bytes) != DATA_LEN:
        raise ValueError(f"DATA 必须为 {DATA_LEN} 字节，当前为 {len(data_bytes)}")

    if not data_bytes.isdigit():
        raise ValueError("DATA 必须全部是数字 0-9")

    # length 为数据段长度，本协议无 CRC、无帧尾
    return FRAME_HEADER + bytes([TYPE_TAG, DATA_LEN]) + data_bytes


# ---------------------------------------------------------------
# 打开串口
# ---------------------------------------------------------------
def open_serial(port: str,
                baudrate: int = BAUDRATE,
                timeout: float = 1.0) -> serial.Serial:
    """
    打开串口。
    port 例如 'COM3'（Windows）或 '/dev/ttyUSB0'（Linux）
    """
    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
    )
    return ser


# ---------------------------------------------------------------
# 发送函数
# ---------------------------------------------------------------
def send_to_serial(data: str, ser: serial.Serial) -> bytes:
    """
    把 data（12 位数字字符串）打包成 0x03 Tag 数据帧，通过串口发送。
    返回实际发出的字节。
    """
    frame = build_frame(data)
    ser.write(frame)
    ser.flush()
    return frame


def main():

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头，请检查连接。")
        return

    # 创建 QR 检测器和 CLAHE 对象（复用，提高效率）
    detector = cv2.QRCodeDetector()
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取视频帧，退出。")
            break

        text = detect_qrcode(frame, detector, clahe)
        print(f"检测到二维码: {text}")

        if text:
            try:
                with open_serial(PORT, BAUDRATE) as ser:
                    for _ in range(5):
                        send_to_serial(text, ser)
                        time.sleep(0.1)
                # cap.release()
                # cv2.destroyAllWindows()
                # return
            except (ValueError, serial.SerialException) as e:
                print(f"发送失败: {e}")

        # 显示实时画面（方便调试）
        #cv2.imshow("QR Scanner", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


# ---------------------------------------------------------------
# 主程序
# ---------------------------------------------------------------
if __name__ == '__main__':
    main()

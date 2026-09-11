import os
os.environ['OPENCV_LOG_LEVEL'] = 'ERROR'  # 必须在 import cv2 之前，压制 QUIRC 警告
import cv2

# 改成你自己的串口
#PORT = 'COM3'          # Windows 示例
PORT = '/dev/ttyUSB0'  # Linux 示例
BAUDRATE = 115200
FRAME_HEADER = b'\x55\xAA'
FRAME_TAIL = b'\x3C\x3E'
DATA_LEN = 12



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

# -*- coding: utf-8 -*-
"""
自定义帧协议：
FRAME_HEADER  2 字节   0x5A 0xA5
DATA          12 字节  ASCII 数字，例如 b"123123123123"
CRC16         2 字节   CRC16/MODBUS，低字节在前
FRAME_TAIL    2 字节   0x3C 0x3E

CRC 计算范围：FRAME_HEADER + DATA
整帧固定长度：22 字节
"""

import serial
import time



# ---------------------------------------------------------------
# CRC16/MODBUS
# ---------------------------------------------------------------
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


# ---------------------------------------------------------------
# 打包
# ---------------------------------------------------------------
def build_frame(data: str) -> bytes:
    """
    把 12 位 ASCII 数字打包成完整帧。
    """
    data = data.replace("+", "")
    data_bytes = data.encode('ascii')

    if len(data_bytes) != DATA_LEN:
        raise ValueError(f"DATA 必须为 {DATA_LEN} 字节，当前为 {len(data_bytes)}")

    if not data_bytes.isdigit():
        raise ValueError("DATA 必须全部是数字 0-9")

    crc = crc16_modbus(FRAME_HEADER + data_bytes)
    crc_bytes = crc.to_bytes(2, byteorder='little')

    return FRAME_HEADER + data_bytes + crc_bytes + FRAME_TAIL


# ---------------------------------------------------------------
# 打开串口
# ---------------------------------------------------------------
def open_serial(port: str,
                baudrate: int = 115200,
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
    把 data（12 位数字字符串）打包成帧，通过串口发送。
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
                    for i in range(5):
                        send_to_serial(text, ser)
                        time.sleep(0.1)
                cap.release()
                cv2.destroyAllWindows()
                return 
            except (ValueError, serial.SerialException) as e:
                print(f"发送失败: {e}")

        # 显示实时画面（方便调试）
        cv2.imshow("QR Scanner", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


# ---------------------------------------------------------------
# 主程序
# ---------------------------------------------------------------
if __name__ == '__main__':
    main()
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""摄像头基准测试：什么格式/分辨率能跑满 15fps，压缩能省多少

给树莓派上用，默认开的就是你那台摄像头的 by-id 名字，不用改参数直接跑：
    python3 cam_bench.py

测三件事：
  1. 采集：格式 × 分辨率，自由跑能出多少 fps、read() 一次多久、吃多少 CPU。
     read() 如果是用户态忙等，CPU 会接近 100% —— 就是之前"只有 4Hz 又吃满一核"
     那个现象，树莓派核少，这里的数据比桌面上更重要。
  2. 编码：同一帧 bgr8 raw 多大 vs JPEG q80 多大。乘 15fps 就是 DDS 要扛的 MB/s
     （640x480 bgr8 一帧 900KB，15fps = 13.8MB/s，raw 那条路基本就是被这个拖住的）
  3. --proc 加上真实的颜色识别 + QR 识别，看整条链路还剩多少余量；
     --ros  真的挂 DDS 往返，对比 raw 和 compressed 订阅端能收到多少 fps。

不动项目里的任何东西，只读摄像头。
"""

import argparse
import os
import sys
import time

os.environ.setdefault('OPENCV_LOG_LEVEL', 'ERROR')

import cv2

# ============ 摄像头：树莓派上就是这台，一般不用改 ============
# /dev/videoN 的 N 是枚举顺序，插拔一次就漂，所以认 by-id 的稳定名字。
CAMERA_BY_ID = 'usb-Generic_Integrated_Webcam_200901010001-video-index0'
V4L_BY_ID = '/dev/v4l/by-id'

# 默认测这几组：MJPG 三档分辨率 + YUYV 一档做对照
COMBOS = [
    ('MJPG', 640, 480),
    ('MJPG', 1280, 720),
    ('MJPG', 1920, 1080),
    ('YUYV', 640, 480),
]

TARGET_FPS = 15.0          # 比赛链路的目标帧率
JPEG_QUALITY = 80


def resolve_device(device):
    """按 by-id 名字拿到设备路径。名字对不上就把现状打出来，别默默退回 /dev/video0。"""
    if device:
        print(f'[相机] 用指定的 {device} -> {os.path.realpath(device)}')
        return device
    link = os.path.join(V4L_BY_ID, CAMERA_BY_ID)
    if os.path.exists(link):
        print(f'[相机] {CAMERA_BY_ID} -> {os.path.realpath(link)}')
        return link
    print(f'[相机] 没有 {link}')
    if os.path.isdir(V4L_BY_ID):
        print('[相机] 当前可用的：')
        for e in sorted(os.listdir(V4L_BY_ID)):
            print(f'         {e} -> {os.path.realpath(os.path.join(V4L_BY_ID, e))}')
    print('[相机] 名字不一样就用 --device 直接给路径（或改脚本顶上的 CAMERA_BY_ID）')
    return ''


def open_cap(device, fourcc, w, h, fps=None, buffersize=1):
    """MJPG 一定要在设分辨率之前设，很多摄像头换了尺寸就不认后面的格式。"""
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        return None, None
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    if fps:
        cap.set(cv2.CAP_PROP_FPS, fps)
    if buffersize:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, buffersize)
    cc = int(cap.get(cv2.CAP_PROP_FOURCC))
    real = {
        'w': int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        'h': int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        'fps': cap.get(cv2.CAP_PROP_FPS),
        'fourcc': ''.join(chr((cc >> (8 * i)) & 0xFF) for i in range(4)),
    }
    return cap, real


def measure(cap, seconds, proc=None):
    """连读 seconds 秒。返回 (fps, CPU占比, 平均读帧ms, 平均处理ms, 最后一帧)。

    CPU 占比 = 进程 CPU 时间 / 墙钟时间，read() 忙等的话会接近 100%。
    """
    for _ in range(10):                    # 预热，丢掉开头不稳的几帧
        cap.read()

    n = 0
    proc_ms_sum = 0.0
    read_ms_sum = 0.0
    frame = None
    t_cpu = time.process_time()
    t0 = time.time()
    while time.time() - t0 < seconds:
        t_r = time.time()
        ret, f = cap.read()
        read_ms_sum += (time.time() - t_r) * 1000.0
        if not ret:
            break
        n += 1
        frame = f
        if proc is not None:
            t_p = time.time()
            proc(f)
            proc_ms_sum += (time.time() - t_p) * 1000.0
    wall = time.time() - t0
    cpu = (time.process_time() - t_cpu) / wall * 100.0 if wall > 0 else 0.0
    return (n / wall, cpu, read_ms_sum / max(n, 1),
            proc_ms_sum / max(n, 1), frame)


def jpeg_size(frame, quality=JPEG_QUALITY):
    t = time.time()
    ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    return (len(buf) if ok else 0), (time.time() - t) * 1000.0


def bench_combo(device, fourcc, w, h, seconds, proc=None):
    print(f'\n===== {fourcc} {w}x{h} =====')

    # --- 自由跑：不限速，看摄像头到底能给多少 ---
    cap, real = open_cap(device, fourcc, w, h, fps=None)
    if cap is None:
        print('  打不开，跳过')
        return None
    note = '' if (real['w'], real['h']) == (w, h) else \
        f"  ⚠ 分辨率没设上，实际 {real['w']}x{real['h']}"
    print(f"  驱动报 {real['w']}x{real['h']} @ {real['fps']:.1f}fps {real['fourcc']}{note}")
    fps_free, cpu_free, read_ms, proc_ms, frame = measure(cap, seconds, proc)
    cap.release()
    if frame is None:
        print('  读不到帧，跳过')
        return None

    # --- 限速：CAP_PROP_FPS=15，驱动认了的话 read() 就阻塞在驱动里，不烧 CPU ---
    cap, real2 = open_cap(device, fourcc, w, h, fps=TARGET_FPS)
    fps_lim, cpu_lim, read_ms_lim, _p, _f = measure(cap, seconds, None)
    cap.release()

    raw_bytes = frame.shape[0] * frame.shape[1] * 3
    jpg_bytes, jpg_ms = jpeg_size(frame)

    print(f"  自由跑     {fps_free:5.1f} fps   CPU {cpu_free:5.1f}%   read {read_ms:5.1f} ms"
          + (f'   识别 {proc_ms:5.1f} ms' if proc else ''))
    print(f"  设{TARGET_FPS:.0f}fps后  {fps_lim:5.1f} fps   CPU {cpu_lim:5.1f}%   "
          f"read {read_ms_lim:5.1f} ms")
    print(f"  一帧 raw  {raw_bytes / 1024:7.1f} KB = {raw_bytes * TARGET_FPS / 1e6:6.2f} MB/s"
          f" @{TARGET_FPS:.0f}fps")
    print(f"  一帧 jpeg {jpg_bytes / 1024:7.1f} KB = {jpg_bytes * TARGET_FPS / 1e6:6.2f} MB/s"
          f" @{TARGET_FPS:.0f}fps   小 {raw_bytes / max(jpg_bytes, 1):.0f} 倍，"
          f"编码 {jpg_ms:.1f} ms/帧")
    print(f"  结论       {'能' if fps_free >= TARGET_FPS else '不能'}跑满 {TARGET_FPS:.0f}fps"
          + ("；CPU 接近满，read() 在忙等" if cpu_free > 80 else ''))
    if proc:
        budget = 1000.0 / TARGET_FPS
        print(f"             识别 {proc_ms:.1f} ms/帧，占 {TARGET_FPS:.0f}fps 预算"
              f"({budget:.1f} ms)的 {proc_ms / budget * 100:.0f}%")

    return {'fourcc': fourcc, 'w': real['w'], 'h': real['h'],
            'fps_free': fps_free, 'fps_lim': fps_lim, 'cpu_free': cpu_free,
            'jpg_kb': jpg_bytes / 1024.0, 'proc_ms': proc_ms,
            'raw_mbs': raw_bytes * TARGET_FPS / 1e6,
            'jpg_mbs': jpg_bytes * TARGET_FPS / 1e6}


def ros_roundtrip(device, w, h, fourcc, seconds, quality=JPEG_QUALITY):
    """raw 和 compressed 各挂一次 DDS 往返，看订阅端实际收到多少 fps / MB/s。"""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import CompressedImage, Image
    except ImportError:
        print('[ros] 引不到 rclpy（先 source /opt/ros/humble/setup.bash），跳过')
        return

    cap, real = open_cap(device, fourcc, w, h, fps=TARGET_FPS)
    if cap is None:
        print('[ros] 打不开摄像头，跳过')
        return

    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    for mode in ('raw', 'compressed'):
        rclpy.init()
        node = Node(f'cam_bench_{mode}')
        stat = {'n': 0, 'bytes': 0, 't_first': None, 't_last': None}
        is_raw = (mode == 'raw')
        topic = '/cam_bench/image' if is_raw else '/cam_bench/image/compressed'
        pub = node.create_publisher(Image if is_raw else CompressedImage, topic, qos)

        def cb(m, stat=stat):
            stat['n'] += 1
            stat['bytes'] += len(m.data)
            stat['t_last'] = time.time()

        node.create_subscription(Image if is_raw else CompressedImage, topic, cb, qos)

        interval = 1.0 / TARGET_FPS
        t0 = time.time()
        t_cpu = time.process_time()
        sent = 0
        while time.time() - t0 < seconds:
            t_r = time.time()
            ret, frame = cap.read()
            if not ret:
                break
            if stat['t_first'] is None:
                stat['t_first'] = time.time()
            if is_raw:
                msg = Image()
                msg.height, msg.width = frame.shape[0], frame.shape[1]
                msg.encoding = 'bgr8'
                msg.step = frame.shape[1] * 3
                msg.data = frame.tobytes()
            else:
                ok, buf = cv2.imencode('.jpg', frame,
                                       [int(cv2.IMWRITE_JPEG_QUALITY), quality])
                if not ok:
                    continue
                msg = CompressedImage()
                msg.format = 'jpeg'
                msg.data = buf.tobytes()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'
            pub.publish(msg)
            sent += 1
            rclpy.spin_once(node, timeout_sec=0)
            rest = interval - (time.time() - t_r)
            if rest > 0:
                time.sleep(rest)

        t_end = time.time() + 0.5                     # 收尾，把在路上的收完
        while time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.05)

        span = (stat['t_last'] - stat['t_first']) if stat['t_first'] and stat['t_last'] else 0.0
        cpu = (time.process_time() - t_cpu) / (time.time() - t0) * 100.0
        if stat['n'] > 1 and span > 0:
            print(f"[ros] {mode:10s} 发 {sent:3d} 收 {stat['n']:3d}  "
                  f"{stat['n'] / span:5.1f} fps   {stat['bytes'] / span / 1e6:6.2f} MB/s   "
                  f"CPU {cpu:5.1f}%")
        else:
            print(f"[ros] {mode:10s} 发 {sent:3d} 收 {stat['n']:3d}  —— 没收到，QoS 对不上？")
        node.destroy_node()
        rclpy.shutdown()

    cap.release()


def main():
    global TARGET_FPS        # 必须在读它之前声明，不然 "used prior to global declaration"

    ap = argparse.ArgumentParser(description='摄像头基准：分辨率/格式/压缩')
    ap.add_argument('--device', default='',
                    help=f'设备路径；留空就用 by-id 名字 {CAMERA_BY_ID}')
    ap.add_argument('--only', default='', help='只测一组，形如 MJPG:640x480')
    ap.add_argument('--seconds', type=float, default=3.0, help='每档测几秒')
    ap.add_argument('--fps', type=float, default=TARGET_FPS, help='目标帧率')
    ap.add_argument('--proc', action='store_true', help='加上颜色识别 + QR 识别')
    ap.add_argument('--ros', action='store_true', help='再加 DDS 往返测试')
    args = ap.parse_args()

    TARGET_FPS = args.fps

    dev = resolve_device(args.device)
    if not dev:
        return 1
    print(f'[相机] 目标 {TARGET_FPS:.0f} fps，每档测 {args.seconds:.1f} 秒')

    combos = COMBOS
    if args.only:
        cc, res = args.only.split(':')
        w, h = res.lower().split('x')
        combos = [(cc.upper(), int(w), int(h))]

    proc = None
    if args.proc:
        try:
            from schooltest2 import block_areas, detect_qrcode
            detector = cv2.QRCodeDetector()
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

            def proc(frame):          # 和 schooltest2 主循环里干的事一样
                detect_qrcode(frame, detector, clahe)
                hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
                block_areas(hsv)
        except ImportError as e:
            print(f'[识别] 引不到 schooltest2（{e}），--proc 跳过')

    rows = []
    for fourcc, w, h in combos:
        try:
            r = bench_combo(dev, fourcc, w, h, args.seconds, proc)
        except KeyboardInterrupt:
            print('\n中断')
            return 1
        if r:
            rows.append(r)

    if rows:
        print(f'\n===== 汇总（目标 {TARGET_FPS:.0f} fps）=====')
        print(f"{'格式':6s}{'分辨率':11s}{'自由跑':>8s}{'限速后':>8s}{'CPU%':>7s}"
              f"{'jpeg KB':>9s}{'rawMB/s':>9s}{'jpgMB/s':>9s}")
        for r in rows:
            print(f"{r['fourcc']:6s}{str(r['w']) + 'x' + str(r['h']):11s}"
                  f"{r['fps_free']:8.1f}{r['fps_lim']:8.1f}{r['cpu_free']:7.0f}"
                  f"{r['jpg_kb']:9.1f}{r['raw_mbs']:9.2f}{r['jpg_mbs']:9.2f}")
        best = [r for r in rows if r['fps_free'] >= TARGET_FPS]
        if best:
            b = max(best, key=lambda r: r['w'] * r['h'])
            print(f"\n能跑满 {TARGET_FPS:.0f}fps 里分辨率最高的是 {b['fourcc']} {b['w']}x{b['h']}："
                  f"一帧 jpeg {b['jpg_kb']:.0f}KB（{b['jpg_mbs']:.2f}MB/s），"
                  f"raw 要 {b['raw_mbs']:.2f}MB/s")
        else:
            print(f'\n没有一组能跑满 {TARGET_FPS:.0f}fps，看上面的 CPU 和 read 耗时找瓶颈')

    if args.ros:
        fourcc, w, h = combos[0]
        print(f'\n===== DDS 往返：{fourcc} {w}x{h} @{TARGET_FPS:.0f}fps =====')
        ros_roundtrip(dev, w, h, fourcc, args.seconds)

    return 0


if __name__ == '__main__':
    sys.exit(main())

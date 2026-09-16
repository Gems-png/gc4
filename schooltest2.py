#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""校赛测试脚本 2：tag 识别 + 物料（颜色大区域）识别 + 0x20/0x21 握手

基于校赛版本的 serial_tag.py，多了四件事：

1. **两路相机**：tag 和物料各一路，分别开、分别识别。
       tag 相机     -> QR 识别（车到停顿点之前就要认出来）
       物料相机     -> 颜色大区域（对着转盘上的物料）
   两路的设备、分辨率、帧率都能单独给，互不影响。哪路挂了另一路照跑。
   派上现在：物料 = USB "2M"（/dev/video0），tag = USB "Integrated Webcam"（/dev/video2）。

2. 设备**不写死 /dev/videoN**。N 是 USB 枚举顺序，插拔一次、上电顺序变一下就会漂
   （这里就踩过：只有一台相机时 Integrated Webcam 是 video0，插上第二台它变成 video2）。
   所以默认按**板卡名**匹配：'name:2M'、'name:Integrated'，名字是摄像头自己报的，不漂。
   也能直接给路径或 /dev/v4l/by-id 里的名字。--list 会把现在的节点和名字都打出来。
   UVC 摄像头一般 index0 = 取流节点、index1 = metadata，别开错。
   注意 /dev/video19 挂在 rpivid 下面是硬件解码器，不是相机。

3. 物料识别：**看图里的圆**。物料是回转体，投影是圆（斜看是椭圆），
   所以用 HoughCircles 找圆，检出圆的就算物料。
   颜色**不参与判断**，只是附带的信息：把圆内占比最高的那种颜色打出来，
   让你知道抓的是哪一块；认不出颜色（比如黑物料、光照太偏）也照样认，不影响。
   颜色编号：红1 黄2 蓝3 绿4 黑5 浅蓝6，规则里每轮抽签只用其中三种。
   半径范围跟分辨率绑死，换分辨率要重标（启动时会打印实际用的像素值）。

4. 和电控的握手（见 USART1_Cmd_Protocol.md）—— 这就是"什么时候能发"：
       0x20 指令状态(下→上, 1B, 每 100ms 上报一次)
           0x00 等待二维码     → 上位机可以发 0x21 cmd=0x01(识别完毕,继续行进)
           0x01 等待颜色信息   → 上位机可以发 0x21 cmd=0x02(抓取一次)
                                还要**等圆停稳**才发：物料停下才好抓，所以圆连续
                                --still-time 秒不动（--still-tol 像素内）才发，
                                还在动就等（终端会打"圆稳 x/y 秒"）。
           0x10 运动中         → 不要发指令打扰，必须发就发 0x00(空指令,兼心跳)
       0x21 指令下发(上→下, 1B)  帧长固定 5 字节
   tag 不受状态影响，识别到就发（burst 5 次，和校赛版一样）；
   指令是**动作**，一次只发一发，绝不 burst（连发 5 次 = 抓 5 次）。

用法：
    python3 schooltest2.py --list                  # 看这台机器上有哪些摄像头
    python3 schooltest2.py                         # 默认两路都开，只打印不发
    python3 schooltest2.py --dev 'name:2M' --tag-dev 'name:Integrated'
    python3 schooltest2.py --dev /dev/video0       # 也可以直接给节点
    python3 schooltest2.py --tag-dev ''            # 只开物料相机
    python3 schooltest2.py --dev '' --tag-dev 'name:Integrated'   # 只开 tag 相机
    python3 schooltest2.py --width 1280 --height 720 --area 6000
    python3 schooltest2.py --rx-log                # 把下位机发来的每一帧都打出来
    python3 schooltest2.py --send                  # 真的发（先干跑看清楚了再开）
"""

import argparse
import os
import sys
import threading
import time
import traceback
from collections import namedtuple

os.environ.setdefault('OPENCV_LOG_LEVEL', 'ERROR')

import cv2
import numpy as np
import serial

# ---------------- 相机默认值 ----------------
# 'name:片段' = 按**板卡名**找（就是 v4l2-ctl 打的那一列）。
# 板卡名是摄像头自己报的，不随 USB 枚举顺序变，比写死 /dev/videoN 稳得多。
#   物料相机 = USB 的 "2M"          （派上现在是 /dev/video0）
#   tag 相机 = USB 的 "Integrated Webcam"（派上现在是 /dev/video2）
# 也可以直接给设备路径 /dev/videoN，或给 /dev/v4l/by-id 里的名字。
# 注意：/dev/video19 挂在 rpivid 下面是硬件解码器，不是相机。
MATERIAL_DEV = 'name:2M'
TAG_DEV = 'name:Integrated'
VIDEO_INDEX = 0        # UVC 一般 index0 = 取流节点，index1 是 metadata

V4L_BY_ID = '/dev/v4l/by-id'
V4L_BY_PATH = '/dev/v4l/by-path'
V4L_SYSFS = '/sys/class/video4linux'

CAMERA_FOURCC = 'MJPG'  # 不压缩(YUYV)在 640x480 就只有 ~5-10fps，MJPG 才能跑满

# ---------------- 串口 / 协议 ----------------
PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
FRAME_HEADER = b'\xAA\x55'
TYPE_TAG = 0x03
TAG_DATA_LEN = 12

# 握手：USART1_Cmd_Protocol.md
TYPE_STATE = 0x20      # 下→上，指令状态
TYPE_CMD = 0x21        # 上→下，指令
CMD_LEN = 1
STATE_WAIT_QR = 0x00       # 等待二维码
STATE_WAIT_COLOR = 0x01    # 等待颜色信息
STATE_MOVING = 0x10        # 运动中
CMD_IDLE = 0x00            # 空指令 / 心跳填充（不动作）
CMD_QR_DONE = 0x01         # 二维码识别完毕，继续行进
CMD_GRAB = 0x02            # 抓取一次

STATE_TIMEOUT = 0.5        # 下位机 100ms 一发，超过这么久没收到就当状态未知：不发指令
GRAB_COOLDOWN = 1.5        # 同一次"等待颜色信息"里，两次"抓取一次"至少隔这么久

# 圆停稳判定：物料停下来才好抓，所以"等待颜色信息"时还要等圆在画面里不动了才发 0x02。
# 判据是最大的那个圆的圆心/半径连续 STILL_TIME 秒没超出容差（帧间抖动几个像素是正常的）。
STILL_TIME = 0.5           # 要连续静止多少秒才算停稳（--still-time）
STILL_TOL = 6              # 圆心挪了多少像素就算还在动（--still-tol）
STILL_R_FRAC = 0.10        # 半径变化超过这个比例也算还在动

# 类型 -> 数据段长度的白名单。协议里没有 CRC 也没有帧尾，
# 帧边界只能靠这张表认，所以新类型必须在这里登记，否则会被当成噪声跳过。
TYPE_LEN = {
    0x00: 18, 0x01: 32, 0x02: 32, 0x03: 12,
    0x04: 18, 0x05: 12, 0x10: 12, 0x11: 28,
    TYPE_STATE: CMD_LEN,
    TYPE_CMD: CMD_LEN,
}

TAG_SEND_TIMES = 5     # 和校赛版一样：tag 是数据帧，连着发 5 次，丢一帧也不怕

# 如果电控还要把颜色本身也发过去（状态名叫"等待颜色信息"），
# 大概率是走 0x03 这条视觉数据通道，12 字节 ASCII 数字。到时候：
#   1. MATERIAL_TAG 改成 True
#   2. MATERIAL_TAG_CONTENT 填电控给的内容
MATERIAL_TAG = False
MATERIAL_TAG_CONTENT = '000000000000'

# ---------------- 颜色 / 物料 ----------------
# 阈值表和 src/cv/cv/material.py 保持一致，那边改了这边也要改
HSV_RANGES = {
    "red":        [((0, 110, 90),   (10, 255, 255)),
                   ((170, 110, 90), (180, 255, 255))],
    "yellow":     [((20, 110, 110), (33, 255, 255))],
    "green":      [((38, 70, 60),   (85, 255, 255))],
    "blue":       [((100, 120, 80), (128, 255, 255))],
    "light_blue": [((86, 60, 130),  (100, 255, 255))],
    "black":      [((0, 0, 0),      (180, 90, 45))],
}
COLOR_ORDER = ["red", "yellow", "green", "blue", "light_blue", "black"]
COLOR_CN = {"red": "红", "yellow": "黄", "green": "绿",
            "blue": "蓝", "light_blue": "浅蓝", "black": "黑"}
COLOR_CODE = {"red": "1", "yellow": "2", "blue": "3",
              "green": "4", "black": "5", "light_blue": "6"}   # 规则里的颜色编号
# 画框用的 BGR（cv2.putText 画不了中文，框边上的字用英文）
COLOR_BGR = {"red": (0, 0, 255), "yellow": (0, 255, 255), "green": (0, 255, 0),
             "blue": (255, 0, 0), "light_blue": (255, 180, 0), "black": (90, 90, 90)}

# --- 圆的判据（物料 = 一个圆）---
CIRCLE_R_MIN_FRAC = 0.10 # 最小半径 = 帧短边的这个比例（--min-radius 给了像素值就用那个）
CIRCLE_R_MAX_FRAC = 0.45   # 最大半径 = 帧短边的这个比例
_DISK_SCALE = 0.85         # 认颜色时取样用的盘 = 检出半径的 0.85（躲开边缘过渡色）

# HoughCircles 两种方法。**默认 ALT**：合成图上它一个假圆都不出。
# ALT 的 param2 是"圆的完美度"(0~1，越大越严)；经典 GRADIENT 的 param2 是"累加器票数"，
# 票数跟周长走，量纲完全不同别混用（ALT 上填 15 会一个圆都检不出来）。
# 抗锯齿合成图上 0.8/0.85/0.9 都是 6/6、空图 0 假圆，噪声 sigma 到 16 也一样；
# 0.85 取中间，两边都留点余量。
#
# ⚠ 调参前必读：ALT 对**硬边**极敏感。合成图要是用 cv2.circle 默认的 LINE_8 画，
#   640x480 下扫 40 个半径会漏 18 个（看着像算法烂，其实是渲染出来的硬边）；
#   换成 LINE_AA（真机摄像头就是这种软边）同一个参数只漏 1 个，而且那个还超出了 maxRadius。
#   所以拿合成图扫参数一定开 LINE_AA，否则量的是渲染的毛病。param1 实测基本不影响结果。
# 物料互相遮挡 / 被机械臂挡掉一块时，GRADIENT 反而更能认出来（ALT 要求圆弧完整）。
HOUGH_METHOD = 'alt'
HOUGH_DEFAULTS = {
    'alt':      {'param1': 300, 'param2': 0.85, 'dp': 1.5},
    'gradient': {'param1': 120, 'param2': 15,  'dp': 1.0},
}

_MORPH = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
_MORPH_SMALL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

STATE_CN = {STATE_WAIT_QR: "等待二维码", STATE_WAIT_COLOR: "等待颜色信息",
            STATE_MOVING: "运动中"}
CMD_CN = {CMD_IDLE: "空指令(心跳)", CMD_QR_DONE: "二维码识别完毕,继续行进",
          CMD_GRAB: "抓取一次"}


# ==================== 相机：找设备 ====================
def board_name(dev):
    """读这个节点的板卡名（v4l2-ctl 打的那一列，比如 "2M: 2M"）。"""
    node = os.path.basename(os.path.realpath(dev))
    try:
        with open(os.path.join(V4L_SYSFS, node, 'name')) as f:
            return f.read().strip()
    except OSError:
        return '?'


def _video_nodes():
    """所有 /dev/videoN，按编号从小到大。"""
    if not os.path.isdir(V4L_SYSFS):
        return []
    names = [n for n in os.listdir(V4L_SYSFS) if n.startswith('video')]
    names.sort(key=lambda s: int(s[5:]) if s[5:].isdigit() else 999)
    return [f'/dev/{n}' for n in names]


def _v4l_aliases():
    """{ '/dev/videoN': ['by-id 或 by-path 里的名字', ...] }，反查用。"""
    aliases = {}
    for d in (V4L_BY_ID, V4L_BY_PATH):
        if not os.path.isdir(d):
            continue
        for e in sorted(os.listdir(d)):
            dev = os.path.realpath(os.path.join(d, e))
            aliases.setdefault(dev, []).append(e)
    return aliases


def list_cameras():
    """等价于 v4l2-ctl --list-devices，外加每个节点的 by-id / by-path 别名。

    by-id 是"跟着设备走"的名字（插拔、换 USB 口、重启都不变），优先用它；
    两个同型号摄像头序列号也一样时 by-id 会撞名，那时用 by-path 按物理口认。
    """
    aliases = _v4l_aliases()
    nodes = _video_nodes()
    if not nodes:
        print(f'{V4L_SYSFS} 不存在或没有 video 节点（这台机器没有 v4l 设备？）')
        return
    print('/dev/video* 节点（等价 v4l2-ctl --list-devices）：')
    for dev in nodes:
        print(f'  {dev:16s} {board_name(dev)}')      # 板卡名就是 v4l2-ctl 打的那一列
        for a in aliases.get(dev, []):
            print(f'      {a}')
    print('\n用法：')
    print("  --dev 'name:板卡名片段'   # 推荐，板卡名不随枚举顺序变")
    print('  --dev /dev/videoN        # 也可以直接给节点')
    print('提示：UVC 摄像头通常 index0 = 取流节点、index1 = metadata，别开错。')
    print('      哪个是取流节点拿不准就一个个试，脚本会告诉你读不读得到帧。')


def resolve_device(spec, index=0, by='id'):
    """把设备说明解析成真实路径。认四种写法，认不出来返回 ''（绝不默默退回 video0）。"""
    if not spec:
        return ''

    # 0) 'name:片段' 按板卡名找（默认就是这么写的）
    if spec.startswith('name:'):
        frag = spec[5:].strip()
        hits = [d for d in _video_nodes() if frag.lower() in board_name(d).lower()]
        if not hits:
            print(f'[相机] 没有板卡名含 "{frag}" 的节点')
            for d in _video_nodes():
                print(f'[相机]   {d}  "{board_name(d)}"')
            return ''
        aliases = _v4l_aliases()
        # 一块板子一般挂两个节点（取流 + metadata），优先挑 by-id 里带
        # -video-indexN 的那个 —— 那才是取流节点
        pref = [d for d in hits
                if any(a.endswith(f'-video-index{index}') for a in aliases.get(d, []))]
        pick = pref[0] if pref else hits[0]
        print(f'[相机] 板卡名含 "{frag}" 的节点: {", ".join(hits)} → 用 {pick}')
        if len(hits) > 1 and not pref:
            print(f'[相机] 这几个的板卡名一样，分不出取流/metadata（没有 by-id 别名），'
                  f'先按编号小的来；要是读不到帧就换 {hits[1]} 试试')
        return os.path.realpath(pick)

    # 1) 直接给路径（/dev/video0、/dev/v4l/by-id/xxx）
    if spec.startswith('/dev/'):
        if os.path.exists(spec):
            dev = os.path.realpath(spec)
            print(f'[相机] {spec} -> {dev}')
            return dev
        print(f'[相机] {spec} 不存在')
        return ''

    # 2) 给 by-id / by-path 里的名字（-video-indexN 可带可不带）
    v4l_dir = V4L_BY_PATH if by == 'path' else V4L_BY_ID
    for name in (f'{spec}-video-index{index}', spec):
        link = os.path.join(v4l_dir, name)
        if os.path.exists(link):
            dev = os.path.realpath(link)
            print(f'[相机] {name} -> {dev}')
            return dev

    # 3) 名字对不上（换机器/换摄像头）：把现状打出来，别瞎猜
    print(f'[相机] {v4l_dir} 里没有 {spec}（-video-index{index}）')
    entries = sorted(os.listdir(v4l_dir)) if os.path.isdir(v4l_dir) else []
    if not entries:
        print(f'[相机] {v4l_dir} 不存在或为空（用 --list 看现状）')
        return ''
    print('[相机] 当前可用：')
    for e in entries:
        print(f'         {e} -> {os.path.realpath(os.path.join(v4l_dir, e))}')

    suffix = f'-video-index{index}'
    same = [e for e in entries if e.endswith(suffix)]
    if len(same) == 1:
        prefix = same[0][:-len(suffix)]
        print(f'[相机] 名字对不上，但只有 {same[0]} 一个取流节点，先用它')
        print(f'[相机] 想固定住就把设备名设成 {prefix}')
        return os.path.realpath(os.path.join(v4l_dir, same[0]))
    print(f'[相机] 匹配到 {len(same)} 个取流节点，分不清哪个是哪个，'
          f'直接给设备路径（同型号撞名就用 --by path）')
    return ''


def open_camera(dev, width, height, fps, fourcc):
    """MJPG 一定要在设分辨率之前设，很多摄像头换了尺寸就不认后面的格式。"""
    cap = cv2.VideoCapture(dev)
    if not cap.isOpened():
        return None
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # 只留最新一帧，别积压

    real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    real_fps = cap.get(cv2.CAP_PROP_FPS)
    real_cc = int(cap.get(cv2.CAP_PROP_FOURCC))
    real_fourcc = ''.join(chr((real_cc >> (8 * i)) & 0xFF) for i in range(4))
    print(f'[相机] {dev} "{board_name(dev)}" 实际 {real_w}x{real_h} @ {real_fps:.1f}fps '
          f'{real_fourcc}（要的 {width}x{height} @ {fps}fps {fourcc}）')
    if real_fourcc.strip('\x00') not in ('', fourcc):
        print(f'[相机] {dev} 格式没设上（要 {fourcc} 实际 {real_fourcc}），'
              f'YUYV 在高分辨率下帧率会掉，换 MJPG 再试')
    return cap


# V4L2 的自动曝光取值：1=手动(锁死当前值) 3=自动。别写成 0/1，那是另一套后端的用法。
AUTO_EXPOSURE_MANUAL = 1
AUTO_EXPOSURE_AUTO = 3


def apply_ctrls(cap, dev, ctrls):
    """设亮度/增益/曝光，并读回来核对 —— 摄像头不认这个控制时 cap.set 是静默失败的，
    不打回来的话你会以为设上了。

    注意：曝光要先把自动曝光关掉才设得进去，顺序不能反。
    """
    ae = ctrls.get('auto_exposure')
    if ae is not None:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE,
                AUTO_EXPOSURE_AUTO if ae == 'auto' else AUTO_EXPOSURE_MANUAL)

    failed = False
    for name, prop in (('brightness', cv2.CAP_PROP_BRIGHTNESS),
                       ('gain', cv2.CAP_PROP_GAIN),
                       ('exposure', cv2.CAP_PROP_EXPOSURE)):
        want = ctrls.get(name)
        if want is None:
            continue
        before = cap.get(prop)
        cap.set(prop, want)
        got = cap.get(prop)
        if abs(got - want) >= 1e-6:
            failed = True
        print(f'[相机] {dev} {name}: 要 {want:g} 实际 {got:g}'
              + ('' if abs(got - want) < 1e-6 else f'（没设上，原来是 {before:g}）'))
    if failed:
        list_ctrls_hint(dev)
    if ae is not None:
        print(f'[相机] {dev} 自动曝光: {"开" if ae == "auto" else "关(手动)"}'
              f'（读到 {cap.get(cv2.CAP_PROP_AUTO_EXPOSURE):g}）')


def list_ctrls_hint(dev):
    """这台摄像头支持哪些控制、范围多少，只有驱动知道 —— 让它自己说。"""
    print(f'[相机] 想看清楚 {dev} 支持哪些控制/范围，在树莓派上跑：'
          f'v4l2-ctl -d {dev} --list-ctrls')


# ==================== 相机：采集+识别线程 ====================
class CameraWorker(threading.Thread):
    """一路相机：自己开、自己读、自己识别，结果放在锁保护的字段里给主循环取。

    两路相机的分辨率和帧率可以完全不同，所以各自一条线程独立跑；
    主循环只做协议判断、不碰摄像头，这样 100ms 的状态上报不会被读帧拖慢。
    """

    def __init__(self, role, dev, width, height, fps, fourcc, hp=None, ctrls=None):
        super().__init__(daemon=True)
        self.role = role               # 'tag' 或 'material'
        self.dev = dev
        self.width, self.height = width, height
        self.fps_want, self.fourcc = fps, fourcc
        self.ctrls = ctrls or {}       # 亮度/增益/曝光（没给就不碰）
        self.hp_in = hp or default_hp()   # Hough 参数（0=auto，按分辨率换算）
        self.hp = {}                   # 换算后的实际值，预热完填上
        self.lock = threading.Lock()
        self._frame = None
        self._tag = None               # tag 路：最近认到的二维码内容
        self._materials = []           # 物料路：检出的圆 Circle
        self._fps = 0.0
        self._proc_ms = 0.0
        self.error = ''
        self._running = True

    def stop(self):
        self._running = False

    def snapshot(self):
        with self.lock:
            return self._frame, self._tag, list(self._materials)

    def material_detail(self):
        with self.lock:
            return list(self._materials), dict(self.hp)

    def stats(self):
        with self.lock:
            return self._fps, self._proc_ms

    def run(self):
        """线程入口。整条包一层：线程里崩了默认只往 stderr 吐一行堆栈，主循环
        还当它活着（error 是空的），预览就少一个窗口、看着像"相机没开" —— 踩过一次。"""
        try:
            self._run()
        except Exception as e:
            self.error = f'线程崩了: {e!r}'
            traceback.print_exc()

    def _run(self):
        cap = open_camera(self.dev, self.width, self.height, self.fps_want, self.fourcc)
        if cap is None:
            self.error = f'打不开 {self.dev}'
            return

        if self.ctrls:
            apply_ctrls(cap, self.dev, self.ctrls)

        # 预热，顺便确认这个节点真的出帧。能 open 但永远没帧的节点不少：
        # UVC 的 metadata 节点、rpivid 那种硬件解码器节点，都是这样。
        first = None
        for _ in range(10):
            ret, f = cap.read()
            if ret and f is not None:
                first = f
                break
        if first is None:
            self.error = (f'{self.dev} 打得开但读不到帧 —— 这个节点多半不是取流节点。'
                          f'用 v4l2-ctl -d {self.dev} --list-formats-ext 确认，'
                          f'要的是带 Video Capture + YUYV/MJPG 的那一组')
            cap.release()
            return

        detector = cv2.QRCodeDetector() if self.role == 'tag' else None
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        if self.role == 'material':
            # 半径按实际帧的短边算，所以拿预热那帧的尺寸（不是循环里的 frame，那时候还没有）
            self.hp = eff_hough(self.hp_in, first.shape[0], first.shape[1])

        n, t0, proc_ms = 0, time.time(), 0.0
        while self._running:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue
            t = time.time()
            if self.role == 'tag':
                text = detect_qrcode(frame, detector, clahe)
                with self.lock:
                    self._frame = frame
                    self._tag = text
            else:
                hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
                mats = detect_materials(frame, hsv, self.hp)
                with self.lock:
                    self._frame = frame
                    self._materials = mats
            proc_ms = 0.9 * proc_ms + 0.1 * (time.time() - t) * 1000.0

            n += 1
            elapsed = time.time() - t0
            if elapsed >= 2.0:
                with self.lock:
                    self._fps, self._proc_ms = n / elapsed, proc_ms
                n, t0 = 0, time.time()

        cap.release()


# ==================== 帧 ====================
def detect_qrcode(frame, detector, clahe):
    """校赛版原样：灰度 + CLAHE + QRCodeDetector。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = clahe.apply(gray)
    data, _, _ = detector.detectAndDecode(gray)
    return data.strip() if data else None


def build_tag_frame(data):
    """0x03 tag 帧：12 字节 ASCII 数字。"""
    data = data.replace('+', '')
    if (not data.isdigit()) or len(data) != TAG_DATA_LEN:
        raise ValueError(f'二维码内容必须是 {TAG_DATA_LEN} 位数字，实际: {data!r}')
    return FRAME_HEADER + bytes([TYPE_TAG, TAG_DATA_LEN]) + data.encode('ascii')


def build_cmd_frame(cmd):
    """0x21 指令帧：固定 5 字节 AA 55 21 01 CC。"""
    if cmd not in (CMD_IDLE, CMD_QR_DONE, CMD_GRAB):
        raise ValueError(f'指令字 0x{cmd:02X} 不在协议里（只有 00/01/02 合法）')
    return FRAME_HEADER + bytes([TYPE_CMD, CMD_LEN, cmd])


# ==================== 颜色 / 物料 ====================
def _mask_of_color(hsv, color):
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in HSV_RANGES[color]:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    # 先 open 掉小点, 再 close 补洞. open 用 3x3 更温和, 避免吃掉薄剪影
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _MORPH_SMALL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH)
    return mask


def color_masks(hsv):
    """6 种颜色各一张掩码。一帧只算一次，找圆和找色块都复用。"""
    return {c: _mask_of_color(hsv, c) for c in COLOR_ORDER}


# ---- 圆：物料的判据 ----
Circle = namedtuple('Circle', 'color hsv area center radius fill')
# color=圆内占比最高的颜色（认不出就是 None，不影响判定）
# hsv=(H,S,V) 圈内像素的代表值，调阈值就看这个（认不出颜色时是整个盘的代表值）
# area=πr²(px²)  center=(x,y)  radius=px  fill=那种颜色占取样盘的比例（只是信息）


def _hsv_of(hsv, mask):
    """mask 内像素的代表 HSV。S/V 取中位数；H 取圆均值 —— 红色跨 0/180 两头，
    直接取中位数会算出个青绿来（0 和 179 的中位数是 90）。"""
    sel = mask > 0
    n = int(cv2.countNonZero(mask))
    if n == 0:
        return None
    ang = np.radians(hsv[:, :, 0][sel].astype(np.float64))
    h = int(round(np.degrees(np.arctan2(np.sin(ang).mean(),
                                        np.cos(ang).mean())))) % 180
    return (h, int(np.median(hsv[:, :, 1][sel])), int(np.median(hsv[:, :, 2][sel])))


def default_hp(method=HOUGH_METHOD):
    """默认的 Hough 参数。param1/param2/min_dist/半径 给 0 = auto。"""
    d = HOUGH_DEFAULTS[method]
    return {'method': method, 'param1': d['param1'], 'param2': d['param2'],
            'min_dist': 0, 'min_radius': 0, 'max_radius': 0}


def eff_hough(hp, height, width):
    """把 0=auto 的 Hough 参数按这一路的分辨率换算成像素，返回实际用的值。

    半径用"帧短边的比例"表示，这样同一个默认值在 640x480 和 1280x720 下都说得通；
    实际用的值启动时会打印出来。param1/param2 给 0 就用这种方法自己的默认值。
    """
    eff = dict(hp)
    method = eff.get('method', HOUGH_METHOD)
    if method == 'alt' and not hasattr(cv2, 'HOUGH_GRADIENT_ALT'):
        print(f'[物料] 这个 OpenCV（{cv2.__version__}）没有 HOUGH_GRADIENT_ALT，'
              f'改用经典 GRADIENT')
        method = 'gradient'
    d = HOUGH_DEFAULTS[method]
    eff['method'] = method
    eff['dp'] = d['dp']
    if not eff.get('param1'):
        eff['param1'] = d['param1']
    if not eff.get('param2'):
        eff['param2'] = d['param2']
    elif method == 'alt' and eff['param2'] > 1:
        print(f'[物料] ALT 的 param2 是"完美度"，要 0~1（比如 0.85）；'
              f'现在的 {eff["param2"]} 是 GRADIENT 那套的量纲，会一个圆都检不出来')

    short = min(height, width)
    if eff['min_radius'] <= 0:
        eff['min_radius'] = max(4, int(short * CIRCLE_R_MIN_FRAC))
    if eff['max_radius'] <= 0:
        eff['max_radius'] = max(eff['min_radius'] + 1, int(short * CIRCLE_R_MAX_FRAC))
    eff['min_radius'] = int(eff['min_radius'])
    eff['max_radius'] = int(eff['max_radius'])
    if eff['min_dist'] <= 0:
        eff['min_dist'] = max(10, eff['min_radius'])   # 圆心太近就当同一个圆
    return eff


def detect_materials(frame, hsv, hp):
    """找物料 = 找圆。HoughCircles 检出几个圆就有几块物料，按面积从大到小。

    颜色不参与判定：每个圆只是顺带报一下圈内占比最高的颜色和它的 HSV
    （认不出就是 None），这样打印和预览能看出抓的是哪一块，也方便对着调 HSV 阈值。
    黑物料、光照偏、HSV 阈值没调好都不影响认圆。
    """
    masks = color_masks(hsv)
    gray = cv2.medianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 5)
    method = (cv2.HOUGH_GRADIENT_ALT if hp.get('method') == 'alt'
              else cv2.HOUGH_GRADIENT)
    found = cv2.HoughCircles(gray, method, dp=hp.get('dp', 1.0), minDist=hp['min_dist'],
                             param1=hp['param1'], param2=hp['param2'],
                             minRadius=hp['min_radius'], maxRadius=hp['max_radius'])
    materials = []
    if found is None:
        return materials

    for cx, cy, r in np.round(found[0]).astype(int):
        cx, cy, r = int(cx), int(cy), int(r)
        if r <= 1:
            continue
        # 取样盘比检出的小一圈：边缘那圈是物料到背景的过渡色，算进去会把占比压低
        disk = np.zeros(gray.shape, dtype=np.uint8)
        cv2.circle(disk, (cx, cy), max(2, int(r * _DISK_SCALE)), 255, -1)
        disk_area = float(cv2.countNonZero(disk))
        best_c, best_n, best_m = None, 0, None
        for c in COLOR_ORDER:
            m = cv2.bitwise_and(masks[c], disk)
            n = cv2.countNonZero(m)
            if n > best_n:
                best_c, best_n, best_m = c, n, m
        fill = best_n / disk_area if disk_area > 0 else 0.0
        # HSV 取"判成那个颜色的那些像素"的代表值 —— 调阈值时对着看的就该是这群像素；
        # 一个颜色都没判出来（比如黑物料）就退回整个盘，也有个参考
        mats_hsv = _hsv_of(hsv, best_m) if best_c else _hsv_of(hsv, disk)
        materials.append(Circle(best_c, mats_hsv, float(np.pi * r * r), (cx, cy), r, fill))

    materials.sort(key=lambda k: -k.area)
    return materials


# ==================== 串口 + 握手 ====================
class McuLink:
    """和下位机的串口链路：常开 + 后台收 0x20 状态 + 发送加锁。

    状态就是"现在能不能发指令"的依据，所以状态的新鲜度也要管：
    下位机 100ms 上报一次，超过 STATE_TIMEOUT 没收到就当状态未知，那时不发指令。
    端口不在（比如在桌面上干跑）也不影响，隔一会儿重试，只打印不发。
    """

    def __init__(self, port, baudrate, verbose=False):
        self.port = port
        self.baudrate = baudrate
        self.verbose = verbose
        self.ser = None
        self.lock = threading.Lock()
        self.state = None            # 最近一次 0x20 的状态，None = 还没收到过
        self.state_time = 0.0
        self.rx_count = {}           # 各类型收到多少帧
        self._unknown = set()        # 类型表里没有的帧，同一种只提醒一次
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._running = False

    def fresh_state(self):
        """返回 (状态, 是否新鲜)。不新鲜就别拿它做发送决定。"""
        if self.state is None:
            return None, False
        return self.state, (time.time() - self.state_time) <= STATE_TIMEOUT

    def _connect(self):
        try:
            self.ser = serial.Serial(
                port=self.port, baudrate=self.baudrate, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.1)
            print(f'[串口] 打开 {self.port} @ {self.baudrate}')
            return True
        except (serial.SerialException, OSError) as e:
            print(f'[串口] 打不开 {self.port}（{e}），5 秒后重试；这段时间只打印不发')
            return False

    def _loop(self):
        buf = bytearray()
        while self._running:
            if self.ser is None:
                if not self._connect():
                    for _ in range(50):          # 5 秒，等的时候也能被 stop 打断
                        if not self._running:
                            return
                        time.sleep(0.1)
                else:
                    buf.clear()
                continue
            try:
                data = self.ser.read(64)
            except (serial.SerialException, OSError) as e:
                print(f'[串口] 读失败（{e}），重新连接')
                self.ser = None
                continue
            if data:
                buf += data
                self._parse(buf)

    def _parse(self, buf):
        """按 帧头 + 类型长度白名单 切帧。没有 CRC/帧尾，白名单就是唯一的边界证据。"""
        while True:
            i = buf.find(FRAME_HEADER)
            if i < 0:
                del buf[:-1]                  # 留住最后一个字节，可能是半个帧头
                return
            if i > 0:
                del buf[:i]
            if len(buf) < 4:
                return
            mtype, mlen = buf[2], buf[3]
            if TYPE_LEN.get(mtype) != mlen:
                key = (mtype, mlen)
                if key not in self._unknown:
                    self._unknown.add(key)
                    print(f'[串口] 帧头后面是 0x{mtype:02X}/{mlen}B，不在类型表里，'
                          f'跳过（--rx-log 看原始字节）')
                del buf[0]                    # 假帧头，往后挪一个字节重新找
                continue
            if len(buf) < 4 + mlen:
                return                        # 数据段还没收全
            payload = bytes(buf[4:4 + mlen])
            del buf[:4 + mlen]
            self._dispatch(mtype, payload)

    def _dispatch(self, mtype, payload):
        self.rx_count[mtype] = self.rx_count.get(mtype, 0) + 1
        if mtype == TYPE_STATE:
            state = payload[0]
            self.state = state
            self.state_time = time.time()
            if state not in STATE_CN:
                print(f'[握手] 下位机报了未知状态 0x{state:02X}（--rx-log 看原始字节）')
        elif self.verbose:
            print(f'[串口] 收到 0x{mtype:02X} ({len(payload)}B): {payload.hex(" ").upper()}')

    def send(self, frame, what, times=1):
        """发一帧。指令(times=1)只发一次 —— 连发 = 连抓好几次。"""
        if self.ser is None:
            print(f'[发送] {what}: 串口没连上，没发出去')
            return False
        with self.lock:
            try:
                for i in range(times):
                    self.ser.write(frame)
                    self.ser.flush()
                    tag = f' 第 {i + 1}/{times} 次' if times > 1 else ''
                    print(f'[发送] {what}{tag}: {frame.hex(" ").upper()}')
                    if times > 1:
                        time.sleep(0.1)
                return True
            except (serial.SerialException, OSError) as e:
                print(f'[发送] {what} 失败: {e}')
                self.ser = None
                return False


# ==================== 主循环 ====================
def main():
    ap = argparse.ArgumentParser(description='校赛测试：tag + 物料 + 0x20/0x21 握手')
    ap.add_argument('--list', action='store_true', help='只列出摄像头就退出')
    ap.add_argument('--dev', default=MATERIAL_DEV,
                    help="物料相机：'name:板卡名片段' / 设备路径 / by-id 名字；留空 = 不开这一路")
    ap.add_argument('--tag-dev', default=TAG_DEV,
                    help="tag 相机：'name:板卡名片段' / 设备路径 / by-id 名字；留空 = 不开这一路")
    ap.add_argument('--by', choices=['id', 'path'], default='id',
                    help='id=按设备名（默认）  path=按 USB 物理口（同型号撞名时用）')
    ap.add_argument('--video-index', type=int, default=VIDEO_INDEX,
                    help='UVC 一般 0=取流 1=metadata')
    ap.add_argument('--width', type=int, default=640, help='物料相机宽（实际值启动时打印）')
    ap.add_argument('--height', type=int, default=480, help='物料相机高')
    ap.add_argument('--fps', type=float, default=15.0, help='物料相机帧率')
    ap.add_argument('--tag-width', type=int, default=640, help='tag 相机宽')
    ap.add_argument('--tag-height', type=int, default=480, help='tag 相机高')
    ap.add_argument('--tag-fps', type=float, default=15.0, help='tag 相机帧率')
    ap.add_argument('--fourcc', default=CAMERA_FOURCC,
                    help="像素格式，默认 MJPG；'' 表示不改")
    ap.add_argument('--min-radius', type=int, default=0,
                    help='圆最小半径 px；0=auto（帧短边的 %.0f%%）' % (CIRCLE_R_MIN_FRAC * 100))
    ap.add_argument('--max-radius', type=int, default=0,
                    help='圆最大半径 px；0=auto（帧短边的 %.0f%%）' % (CIRCLE_R_MAX_FRAC * 100))
    ap.add_argument('--hough-method', choices=['alt', 'gradient'], default=HOUGH_METHOD,
                    help='alt(默认,完美度阈值,不出假圆) / gradient(经典,能认被遮挡的圆)')
    ap.add_argument('--hough-param1', type=float, default=0, help='Canny 阈值；0=auto')
    ap.add_argument('--hough-param2', type=float, default=0,
                    help='灵敏度：alt 是完美度 0~1（调小更容易出圆），'
                         'gradient 是票数（调小更容易出圆）；0=auto')
    ap.add_argument('--circle-mindist', type=int, default=0,
                    help='两个圆心的最小间距 px；0=auto（等于最小半径）')
    ap.add_argument('--port', default=PORT, help='串口设备')
    ap.add_argument('--hb-hz', type=float, default=5.0,
                    help='运动中(0x10)发空指令当心跳的频率，0=不发')
    ap.add_argument('--still-time', type=float, default=STILL_TIME,
                    help='"等待颜色信息"时，圆要连续静止这么多秒才发抓取（默认 0.5）')
    ap.add_argument('--still-tol', type=float, default=STILL_TOL,
                    help='圆心挪动超过这么多像素就算还在动（默认 6）')
    ap.add_argument('--grab-cooldown', type=float, default=GRAB_COOLDOWN,
                    help='两次"抓取一次"之间至少隔几秒')
    ap.add_argument('--send', action='store_true', help='真的发给下位机（默认只打印）')
    ap.add_argument('--rx-log', action='store_true', help='打印下位机发来的每一帧')
    ap.add_argument('--no-preview', action='store_true', help='不开预览窗口')
    # 画面亮暗是摄像头自己的自动曝光/增益定的，脚本只是把它设下来。
    # 不填就一点都不碰（保持摄像头原来的设置），填了会读回来核对并打印。
    ap.add_argument('--brightness', type=float, default=None,
                    help='亮度；不填=不碰。范围因摄像头而异，用 v4l2-ctl --list-ctrls 看')
    ap.add_argument('--gain', type=float, default=None, help='增益；不填=不碰')
    ap.add_argument('--exposure', type=float, default=None,
                    help='曝光（绝对）；不填=不碰。要先把 --auto-exposure manual 关掉才设得进去')
    ap.add_argument('--auto-exposure', choices=['auto', 'manual'], default=None,
                    help='自动曝光开关；画面太亮就 manual（不填=不碰）')
    args = ap.parse_args()

    if args.list:
        list_cameras()
        return 0

    # ---- 开两路相机，哪路挂了另一路照跑 ----
    workers = []
    for role, spec, w, h, fps in (('material', args.dev, args.width, args.height, args.fps),
                                  ('tag', args.tag_dev, args.tag_width,
                                   args.tag_height, args.tag_fps)):
        if not spec:
            print(f'[相机] {role} 这一路没给设备，不开')
            continue
        dev = resolve_device(spec, args.video_index, args.by)
        if not dev:
            print(f'[相机] {role} 相机没找到，这一路不开（用 --list 看现状）')
            continue
        hp = {'method': args.hough_method, 'param1': args.hough_param1,
              'param2': args.hough_param2, 'min_dist': args.circle_mindist,
              'min_radius': args.min_radius, 'max_radius': args.max_radius}
        ctrls = {k: v for k, v in (('brightness', args.brightness), ('gain', args.gain),
                                   ('exposure', args.exposure),
                                   ('auto_exposure', args.auto_exposure)) if v is not None}
        workers.append(CameraWorker(role, dev, w, h, fps, args.fourcc, hp=hp, ctrls=ctrls))
    if not workers:
        print('[相机] 一路相机都没开起来，退出')
        return 1

    for wk in workers:
        wk.start()
    time.sleep(0.5)          # 等它们把"打不开/读不到帧"报出来
    for wk in workers:
        if wk.error:
            print(f'[相机] {wk.role} 相机有问题: {wk.error}')
    alive = [wk for wk in workers if not wk.error]
    if not alive:
        print('[相机] 两路都没出帧，退出')
        return 1
    tag_wk = next((wk for wk in alive if wk.role == 'tag'), None)
    mat_wk = next((wk for wk in alive if wk.role == 'material'), None)

    if mat_wk:
        _m, hp = mat_wk.material_detail()
        print(f'[物料] 判据：Hough 找圆，检出圆的就算物料。方法 {hp.get("method")}'
              f'（dp={hp.get("dp")} param1={hp.get("param1")} param2={hp.get("param2")}），'
              f'半径 {hp.get("min_radius")}~{hp.get("max_radius")}px，'
              f'圆心间距≥{hp.get("min_dist")}px')
        print('[物料] 颜色只是附带信息（圆内占比最高的那种），认不出颜色不影响判定')
    else:
        print('[物料] 没有可用物料相机，"抓取一次"那一路不会触发')
    if tag_wk:
        print(f'[Tag] tag 不看状态，识别到就发（burst {TAG_SEND_TIMES} 次）')
    else:
        print('[Tag] 没有可用 tag 相机，二维码那一路不会触发')
    print('[握手] 0x20 状态 -> 0x21 指令: 等待二维码(00) 发 01; '
          '等待颜色信息(01) 且圆停稳发 02; 运动中(10) 不发'
          + (f'(心跳 {args.hb_hz:.0f}Hz)' if args.hb_hz > 0 else ''))
    print(f'[发送] {"真发" if args.send else "只打印（加 --send 才真发）"}')

    link = McuLink(args.port, BAUDRATE, verbose=args.rx_log)
    link.start()

    # 有显示器才开预览窗口；车上/无 DISPLAY 时自动关掉，不会崩
    show = bool(os.environ.get('DISPLAY')) and not args.no_preview
    if show:
        print('[预览] 每路相机一个窗口（--no-preview 关掉）')

    last_tag = None          # 上一次的 tag 内容，变了才打印，别刷屏
    last_colors = None       # 上一次的圆（数量/位置/颜色），变了才打印
    last_state = None        # 上一次收到的 0x20 状态
    last_skip = None         # 上一次"想发但被状态挡住"的原因，变了才打印
    qr_text = None           # 认到的二维码（留着，等到 0x20 说等待二维码时发指令）
    qr_done_episode = None   # 本轮"等待二维码"里已经发过 01 了
    last_grab = 0.0          # 上次发"抓取一次"的时刻
    still_ref = None         # 上次看到的那个圆 (center, radius)，用来判断动没动
    still_since = None       # 最后一次"看到圆在动"的时刻；None = 现在没圆
    last_hb = 0.0            # 上次发心跳的时刻
    dead_roles = set()       # 已经报过"中途挂了"的相机，别每 2s 刷一遍
    no_state_warned = False
    state_episode = 0
    frames = 0
    t0 = time.time()

    def decide(state, now):
        """按 0x20 状态决定要发哪条 0x21 指令；返回 (cmd, 被挡的原因)。"""
        if state == STATE_WAIT_QR:
            if qr_text is None:
                return None, '等待二维码，但还没认到 tag'
            if qr_done_episode == state_episode:
                return None, None            # 这一轮已经发过 01 了，等下一次等待
            return CMD_QR_DONE, None
        if state == STATE_WAIT_COLOR:
            if not materials:
                return None, '等待颜色信息，但画面里没有物料'
            if now - last_grab < args.grab_cooldown:
                return None, None
            if still_since is None or now - still_since < args.still_time:
                return None, '等待颜色信息，圆还在动（等它停稳再抓）'
            return CMD_GRAB, None
        if state == STATE_MOVING:
            if args.hb_hz > 0 and now - last_hb >= 1.0 / args.hb_hz:
                return CMD_IDLE, None
            return None, None
        return None, f'未知状态 0x{state:02X}'

    while True:
        now = time.time()

        # ---- tag：不受状态影响，看到就发 ----
        tag_text = None
        if tag_wk:
            _f, tag_text, _m = tag_wk.snapshot()
        if tag_text and tag_text != last_tag:
            print(f'[Tag] 识别到: {tag_text}')
            qr_text = tag_text
            qr_done_episode = None           # 内容变了，允许再通知一次
            try:
                f = build_tag_frame(tag_text)
                if args.send:
                    link.send(f, 'tag 0x03', times=TAG_SEND_TIMES)
                else:
                    print(f'[Tag] 只打印不发: {f.hex(" ").upper()}'
                          f'（burst {TAG_SEND_TIMES} 次）')
            except ValueError as e:
                print(f'[Tag] {e}')
        last_tag = tag_text

        # ---- 物料：找圆 ----
        materials = []
        if mat_wk:
            _f, _t, materials = mat_wk.snapshot()
        # 圆的数量/位置/颜色/HSV 变了才打印，不然每帧刷屏
        sig = tuple((c.color, c.hsv, c.radius, c.center) for c in materials)

        if sig != last_colors:
            if materials:
                detail = '  '.join(
                    ((f'{COLOR_CN[c.color]}({COLOR_CODE[c.color]}) {c.fill:.0%}'
                      if c.color else '颜色认不出')
                     + (f' HSV={c.hsv[0]},{c.hsv[1]},{c.hsv[2]}' if c.hsv else ' HSV=-')
                     + f' r={c.radius} @{c.center}')
                    for c in materials)
                print(f'[物料] 检出 {len(materials)} 个圆: {detail}')
            else:
                print('[物料] 画面里没有圆')
        last_colors = sig

        # ---- 圆停稳了没：物料停下才好抓，等它不动了再发 ----
        # 只看最大的那个圆（materials 按面积排过序），它就是要抓的那块物料。
        # still_since = 最后一次"看到它在动"的时刻；它离现在够久 = 停稳了。
        if materials:
            big = materials[0]
            if still_ref is None:
                still_since = now                      # 第一次看到，从现在开始计时
            else:
                move = max(abs(big.center[0] - still_ref[0][0]),
                           abs(big.center[1] - still_ref[0][1]))
                dr = abs(big.radius - still_ref[1])
                if move > args.still_tol or dr > max(2, still_ref[1] * STILL_R_FRAC):
                    still_since = now                  # 动了，重新计时
            still_ref = (big.center, big.radius)
        else:
            still_ref, still_since = None, None        # 没圆就当没停稳，从零开始算

        # ---- 握手：按 0x20 的状态发 0x21 指令 ----
        state, fresh = link.fresh_state()
        if state != last_state:
            if state is None:
                if not no_state_warned:
                    print('[握手] 还没收到下位机的 0x20 状态，先不发指令')
                    no_state_warned = True
            else:
                state_episode += 1
                print(f'[握手] 下位机状态: 0x{state:02X} {STATE_CN.get(state, "?")}')
            last_state = state

        if fresh:
            cmd, skip = decide(state, now)
            if cmd is not None:
                try:
                    f = build_cmd_frame(cmd)
                    if args.send:
                        link.send(f, f'指令 0x21 {CMD_CN[cmd]}')
                    else:
                        print(f'[握手] 状态 {STATE_CN.get(state, "?")} → 发指令 '
                              f'{f.hex(" ").upper()}（{CMD_CN[cmd]}）  [只打印，没真发]')
                    if cmd == CMD_QR_DONE:
                        qr_done_episode = state_episode
                    elif cmd == CMD_GRAB:
                        last_grab = now
                        if MATERIAL_TAG:
                            mt = build_tag_frame(MATERIAL_TAG_CONTENT)
                            print(f'[物料] 顺带发 0x03 颜色内容: {mt.hex(" ").upper()}')
                            if args.send:
                                link.send(mt, '物料 0x03', times=TAG_SEND_TIMES)
                    elif cmd == CMD_IDLE:
                        last_hb = now
                except ValueError as e:
                    print(f'[握手] {e}')
            if skip != last_skip and skip is not None:
                print(f'[握手] 先不发: {skip}')
            last_skip = skip

        # ---- 预览：每路一个窗口 ----
        if show:
            for wk in alive:
                frame, t_text, mats = wk.snapshot()
                if frame is None:
                    continue
                view = frame.copy()
                if wk.role == 'material':
                    # 检出的圆都画出来；认得出颜色就用自己的颜色画，认不出画白圈
                    for c in mats:
                        col = COLOR_BGR.get(c.color, (255, 255, 255))
                        cv2.circle(view, c.center, c.radius, col, 3)
                        label = (f'{c.color} {COLOR_CODE.get(c.color, "?")} {c.fill:.0%}'
                                 if c.color else f'? r={c.radius}')
                        if c.hsv:
                            label += f' HSV {c.hsv[0]},{c.hsv[1]},{c.hsv[2]}'
                        cv2.putText(view, label,
                                    (max(0, c.center[0] - c.radius),
                                     max(12, c.center[1] - c.radius - 6)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
                    head = 'material'
                else:
                    head = 'tag'
                    if t_text:
                        cv2.putText(view, t_text, (8, 52),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                if state is None:
                    label, col = 'no-state', (128, 128, 128)
                elif not fresh:
                    label, col = f'state 0x{state:02X} STALE', (0, 165, 255)
                else:
                    label = f'state 0x{state:02X} {STATE_CN.get(state, "?")}'
                    col = (0, 255, 0) if state != STATE_MOVING else (255, 255, 0)
                cv2.putText(view, f'{head}  {label}', (8, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, col, 2)
                cv2.imshow(f'schooltest2-{wk.role}', view)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            time.sleep(0.02)        # 不预览时也别空转烧核

        frames += 1
        elapsed = time.time() - t0
        if elapsed >= 2.0:
            # 线程跑着跑着死了（开完相机才崩的）也得说出来，别让人对着少一个窗口猜
            for wk in alive:
                if wk.error and wk.role not in dead_roles:
                    dead_roles.add(wk.role)
                    print(f'[相机] {wk.role} 相机中途挂了: {wk.error}')
            cam_stat = '  '.join(f'{wk.role} {wk.stats()[0]:.1f}fps/{wk.stats()[1]:.0f}ms'
                                 for wk in alive)
            rx = ' '.join(f'{t:02X}:{n}' for t, n in sorted(link.rx_count.items()))
            st = (f'0x{state:02X} {STATE_CN.get(state, "?")}'
                  if state is not None and fresh else '未知')
            # 等待颜色信息时把"圆稳了多久"打出来，不然光看它不发会以为是卡住了
            still = ''
            if state == STATE_WAIT_COLOR:
                held = now - still_since if still_since is not None else 0.0
                still = f' 圆稳 {held:.1f}/{args.still_time:.1f}s'
            print(f'[状态] 循环 {frames / elapsed:.1f}Hz  相机 {cam_stat}  '
                  f'下位机 {st}{still}  收到帧 {rx or "无"}')
            frames, t0 = 0, time.time()

    for wk in workers:
        wk.stop()
    link.stop()
    time.sleep(0.3)
    if show:
        cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())

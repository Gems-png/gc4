import os
import cv2

V4L_SYSFS = '/sys/class/video4linux'
V4L_BY_ID = '/dev/v4l/by-id'
V4L_BY_PATH = '/dev/v4l/by-path'


def _video_nodes():
    """所有 /dev/videoN，按编号从小到大。"""
    if not os.path.isdir(V4L_SYSFS):
        return []
    names = [n for n in os.listdir(V4L_SYSFS) if n.startswith('video')]
    names.sort(key=lambda s: int(s[5:]) if s[5:].isdigit() else 999)
    return [f'/dev/{n}' for n in names]


def _board_name(dev):
    """读这个节点的板卡名（v4l2-ctl 打的那一列）。"""
    node = os.path.basename(os.path.realpath(dev))
    try:
        with open(os.path.join(V4L_SYSFS, node, 'name')) as f:
            return f.read().strip()
    except OSError:
        return '?'


def _aliases():
    """{ '/dev/videoN': ['by-id 或 by-path 里的名字', ...] }"""
    out = {}
    for d in (V4L_BY_ID, V4L_BY_PATH):
        if not os.path.isdir(d):
            continue
        for e in sorted(os.listdir(d)):
            dev = os.path.realpath(os.path.join(d, e))
            out.setdefault(dev, []).append(e)
    return out


def find_dev(name_fragment, index=0):
    """按板卡名片段找取流节点，返回 /dev/videoN；找不到/分不清返回 ''。"""
    hits = [d for d in _video_nodes()
            if name_fragment.lower() in _board_name(d).lower()]
    if not hits:
        return ''
    names = sorted({_board_name(d) for d in hits})
    if len(names) > 1:
        print(f'[相机] "{name_fragment}" 命中 {len(names)} 块不同板卡，分不清：')
        for d in hits:
            print(f'    {d}  "{_board_name(d)}"')
        return ''
    aliases = _aliases()
    pref = [d for d in hits
            if any(a.endswith(f'-video-index{index}') for a in aliases.get(d, []))]
    return os.path.realpath(pref[0] if pref else hits[0])


def open_cap(name_fragment, width=640, height=480, fps=15.0, fourcc='MJPG'):
    """输入板卡名片段，直接返回已打开的 cv2.VideoCapture；失败返回 None。

    用法：
        cap = open_cap('2M')                 # 物料相机
        cap = open_cap('Integrated', 1280, 720, 30)
        ret, frame = cap.read()
    """
    dev = find_dev(name_fragment)
    if not dev:
        print(f'[相机] 没找到板卡名含 "{name_fragment}" 的取流节点，'
              f'用 --list 看现状')
        return None

    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f'[相机] 打不开 {dev}')
        return None

    # FOURCC 必须在设分辨率之前设：很多 UVC 换了尺寸就不认后面的格式
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    rw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    rh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    rfps = cap.get(cv2.CAP_PROP_FPS)
    cc = int(cap.get(cv2.CAP_PROP_FOURCC))
    rcc = ''.join(chr((cc >> (8 * i)) & 0xFF) for i in range(4))
    print(f'[相机] {dev} "{_board_name(dev)}" 实际 {rw}x{rh} @ {rfps:.1f}fps {rcc}')

    # 预热：能 open 但永远没帧的节点不少（UVC metadata、rpivid 解码器）
    for _ in range(10):
        ret, f = cap.read()
        if ret and f is not None:
            return cap
    print(f'[相机] {dev} 打得开但读不到帧 —— 多半不是取流节点')
    cap.release()
    return None


if __name__ == '__main__':
    # 先看看机器上有哪些相机
    for d in _video_nodes():
        print(f'{d:16s} {_board_name(d)}')

    cap = open_cap('2M')
    if cap is not None:
        ret, frame = cap.read()
        print('读到一帧:', None if frame is None else frame.shape)
        cap.release()
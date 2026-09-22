# 普通 tag = 二维码 —— 纯函数模块, 由 tag_recognize 节点通过 import 引入
"""tag 相机认**普通 tag (二维码)**。

⚠ 跟 apriltag.py 是两回事, 别混: 那边认的是 AprilTag(36h11) 方块, 算位姿给视觉
自定义控制器用; 这边认的是规则里印的二维码, 内容是 4 组三位数, 给 tag 相机用。
两个模块各管各的, 互不 import。

仿 Minit.open_cap 的封装: 一个功能连同它的**全部参数**都封在这一个文件里。调用方
写一句

    tag = read_qrcode(frame)     # 认出来 -> QrReading, 没认出来 -> None

就到手结果; 参数不用翻代码改 —— 传一份 QrParams 进去, 或者按名字覆盖几个:

    tag = read_qrcode(frame, clahe_clip=3.0)

节点里**每帧**都要认的时候别用 read_qrcode (它每次重建 detector), 拿住一个 QrReader:

    reader = QrReader()          # 开一次
    tag = reader.read(frame)     # 每帧

算法照搬校内赛 schooltest2.detect_qrcode 那条实测过的路线:
    灰度 → CLAHE 局部对比度增强 → cv2.QRCodeDetector.detectAndDecode
CLAHE 是给现场光照用的: 顶灯直射、画面一半亮一半暗的时候, 不增强整张直接解不出来。

tag 内容是规则里那 4 组三位数 (用 + 连接, 比如 452+321+254+312):
    raw     原样 (可能带 +)
    digits  只留数字 —— 发给下位机的 0x03 帧要的就是这 12 位
    groups  按三位一组切开 ['452', '321', '254', '312']
            第 1 组 = 第一批物料的搬运颜色和顺序 (见 rules_text.txt)
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import List, Optional

import cv2
import numpy as np

TAG_DIGITS = 12        # 0x03 帧的数据段长度, 也就是有效 tag 该有的数字个数
GROUP_LEN = 3          # 规则里每三位一组


# ============================================================
# 参数 (全封在这里)
# ============================================================

@dataclass(frozen=True)
class QrParams:
    """认二维码的全部参数。

    clahe       False = 不增强, 直接拿灰度解 (画面很干净时反而更稳)。
    clahe_clip  对比度增强的强度上限。太大灰噪点会被放大成假边, 2~3 是常规值。
    clahe_tile  局部窗口大小 (8 = 把画面切成 8x8 块各算各的)。
    group_len   每组几位数 (规则里是 3)。
    expect_digits  认出来的数字该有几位。0 = 不检查。位数对不上**不会**丢掉结果,
                   只是 reading.ok 为 False —— 都认出来了, 用不用交给调用方决定
                   (下位机那条 0x03 帧是定长 12 位, 那儿必须卡死)。
    """
    clahe: bool = True
    clahe_clip: float = 2.0
    clahe_tile: int = 8
    group_len: int = GROUP_LEN
    expect_digits: int = TAG_DIGITS

    def describe(self) -> str:
        return (f'CLAHE={"开" if self.clahe else "关"} clip={self.clahe_clip:g} '
                f'tile={self.clahe_tile} 每组{self.group_len}位 '
                f'期望{self.expect_digits or "任意"}位')


def default_params() -> QrParams:
    return QrParams()


# ============================================================
# 结果
# ============================================================

@dataclass(frozen=True)
class QrReading:
    """认到的一个普通 tag。

    raw     原样文本 (可能带 + 和空白)
    digits  只留数字的那串
    groups  按 group_len 切好的组; 末尾不够一组的也留着 (总比丢掉强)
    """
    raw: str
    digits: str
    groups: List[str]

    @property
    def ok(self) -> bool:
        """有没有数字 (位数那一道由 QrReader.read_ok 按 expect_digits 卡)。"""
        return bool(self.digits)

    def group(self, n: int) -> Optional[str]:
        """第 n 组 (从 1 数起, 规则里的说法)。没有就 None。"""
        return self.groups[n - 1] if 1 <= n <= len(self.groups) else None

    def describe(self) -> str:
        return f'{self.raw!r} → 数字 {self.digits} 组 [{" ".join(self.groups)}]'


# ============================================================
# 识别
# ============================================================

def parse(text: str, params: Optional[QrParams] = None) -> QrReading:
    """把解出来的文本理成 raw / digits / groups (纯字符串处理, 不碰图像)。"""
    p = params if params is not None else default_params()
    raw = (text or '').strip()
    digits = ''.join(ch for ch in raw if ch.isdigit())
    n = max(1, int(p.group_len))
    groups = [digits[i:i + n] for i in range(0, len(digits), n)]
    return QrReading(raw=raw, digits=digits, groups=groups)


class QrReader:
    """认二维码的那套东西 (QRCodeDetector + CLAHE), **建一次反复用**。

    每帧重建 detector 很浪费 —— 节点里拿住一个实例, 每帧 read() 就行。
    """

    def __init__(self, params: Optional[QrParams] = None, **overrides):
        p = default_params() if params is None else params
        if overrides:
            p = replace(p, **overrides)      # 名字写错直接 TypeError, 不静默忽略
        self.params = p
        self.detector = cv2.QRCodeDetector()
        self.clahe = None
        if p.clahe:
            tile = max(1, int(p.clahe_tile))
            self.clahe = cv2.createCLAHE(clipLimit=float(p.clahe_clip),
                                         tileGridSize=(tile, tile))

    def read(self, frame: np.ndarray) -> Optional[QrReading]:
        """认一帧。认到返回 QrReading, 没认到返回 None (画面上没码是常态, 不是错)。

        frame 是 BGR (cv2 读出来的样子)。detectAndDecode 内部会自己转灰度, 但 CLAHE
        得先有灰度图, 所以这里显式转一次。
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.clahe is not None:
            gray = self.clahe.apply(gray)
        try:
            data, _, _ = self.detector.detectAndDecode(gray)
        except cv2.error:
            # 画面太糊/太花时 OpenCV 内部会抛。这一帧当没认到就完了 —— 别让它把取图
            # 线程带走 (一次异常整个 tag 那一路就再也不认了, 校赛版踩过)。
            return None
        if not data:
            return None
        return parse(data, self.params)

    def read_ok(self, frame: np.ndarray) -> Optional[QrReading]:
        """只要**位数对得上**的 (expect_digits 卡一道), 其余当没认到。

        发给下位机的 0x03 帧是定长 12 位, 位数不对的帧填不进去 —— 所以线上要用这个,
        别用裸 read()。
        """
        r = self.read(frame)
        return r if (r is not None and self.digits_ok(r)) else None

    def digits_ok(self, reading: QrReading) -> bool:
        want = int(self.params.expect_digits)
        return want <= 0 or len(reading.digits) == want


def read_qrcode(frame: np.ndarray, params: Optional[QrParams] = None,
                **overrides) -> Optional[QrReading]:
    """一次性认一帧 (内部建个 QrReader, 认完就扔)。

    脚本/试一下用这个; 节点里每帧都认就建个 QrReader 拿住, 别每帧重建 detector。
    """
    return QrReader(params, **overrides).read(frame)


# ============================================================
# 自检: python3 qrcode.py (合成二维码, 不碰任何设备)
# ============================================================

def _make_test_image(text: str, size: int = 320) -> Optional[np.ndarray]:
    """合成一张二维码图。需要 OpenCV 带 QRCodeEncoder (4.5.4+)。"""
    if not hasattr(cv2, 'QRCodeEncoder_create'):
        return None
    qr = cv2.QRCodeEncoder_create().encode(text)
    qr = cv2.resize(qr, (size, size), interpolation=cv2.INTER_NEAREST)
    # 二维码四周要留白 (quiet zone), 贴着边界解不出来
    canvas = np.full((size + 80, size + 80), 255, np.uint8)
    canvas[40:40 + size, 40:40 + size] = qr
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


def _selftest() -> bool:
    ok = True

    def check(name, cond, extra=''):
        nonlocal ok
        ok = ok and bool(cond)
        print(f'{"PASS" if cond else "FAIL"}  {name}{"  " + extra if extra else ""}')

    p = default_params()
    print(f'参数: {p.describe()}')

    # ---- 纯字符串部分: 不用相机也不用 OpenCV ----
    r = parse('452+321+254+312', p)
    check('带 + 的 tag 切得出 4 组', r.groups == ['452', '321', '254', '312'], r.describe())
    check('digits 只留数字', r.digits == '452321254312', f'→ {r.digits}')
    check('第 1 组 = 452 (第一批的搬运顺序)', r.group(1) == '452', f'→ {r.group(1)}')
    check('没有第 5 组', r.group(5) is None)
    check('带空白的也认', parse('  452+321+254+312\n', p).digits == '452321254312')
    check('位数不对也照样给出数字 (要不要用交给调用方)',
          parse('12345', p).digits == '12345')

    # ---- 图像部分: 合成二维码再解回来 ----
    img = _make_test_image('452+321+254+312')
    if img is None:
        print(f'注：这个 OpenCV（{cv2.__version__}）没有 QRCodeEncoder, '
              f'跳过合成图那几项 (真机上拿实物 tag 试)')
        print('\n' + ('全部通过' if ok else '有失败项'))
        return ok

    reader = QrReader(p)
    got = reader.read(img)
    check('合成二维码认得出来', got is not None,
          got.describe() if got else '→ 没认到')
    if got is not None:
        check('解出来的数字和印上去的一样',
              got.digits == '452321254312', f'→ {got.digits}')
        check('read_ok 位数对得上就放行', reader.read_ok(img) is not None)

    # 空图/噪声图: 不该认到东西 (认到就是假码, 后面照着假方向走)
    blank = np.full((480, 640, 3), 130, np.uint8)
    check('纯色空图不报 tag', reader.read(blank) is None)
    rng = np.random.default_rng(0)
    noise = rng.integers(0, 255, (480, 640, 3), dtype=np.uint8)
    check('噪声图不报 tag', reader.read(noise) is None)

    print('\n' + ('全部通过' if ok else '有失败项'))
    return ok


if __name__ == '__main__':
    import sys
    sys.exit(0 if _selftest() else 1)

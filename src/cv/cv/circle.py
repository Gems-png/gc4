# 识别位于地面的同心圆 (Hough ALT 模式) —— 纯函数模块, 由 MainCam 通过 import 引入
"""找圆 = 找地面上的同心圆靶心。

仿 Minit.open_cap 的封装: 一个功能连同它的**全部参数**都封在这一个文件里, 调用方
只写一句

    c = find_circle(frame)

就到手一个结果 (没找到就是 None); 参数不用翻代码改 —— 传一份参数进去, 或者按名字
只覆盖其中几个:

    c = find_circle(frame, param2=0.9)
    c = find_circle(frame, default_params('gradient'))

节点里跟 open_cap 一个写法 (参数在模块里, 节点不 declare):

    class MainCam(Node):
        from circle import find_circle
        ...
        c = self.find_circle(frame)

算法照搬校内赛 schooltest2.py 那条实测过的路线 (detect_materials / eff_hough / merge_cones):
    1. 灰度 medianBlur(5) → HoughCircles, **默认 ALT** (HOUGH_GRADIENT_ALT)。
       ALT 的 param2 是"圆的完美度"(0~1), 经典 GRADIENT 的 param2 是"累加器票数",
       票数跟周长走 —— 两者量纲不同, 别混用 (ALT 上填 15 会一个圆都检不出来)。
       ⚠ ALT 对**硬边**极敏感: 合成图一定要用 cv2.LINE_AA 画 (真机摄像头就是软边),
         用 LINE_8 的硬边会平白漏掉一堆半径, 那是渲染的毛病不是算法烂。
       圆被挡住一块 / 两个圆互相遮挡时 ALT 反而认不出来 (它要求圆弧完整) →
       method='gradient' 换回经典那套。
    2. 半径按**帧短边的比例**给 (0=auto), 所以换分辨率不用重标参数。
    3. 每个圆报一下圈内占比最高的颜色 + 整圆的平均 HSV (跟阈值表无关, 拿去调阈值用)。
       颜色**不参与判定**: 黑靶心、光照偏、阈值没调好都照样认圆, 认不出颜色就是 None。
    4. **同心圆合并**: 靶心/圆台投影出来是大小几个同心圆, 物理上是同一个东西, 留大的
       那个 (圆心更稳), merged 记下并掉了几个。不合并的话"最大的那个圆"会在几层之间
       跳来跳去 —— 圆心一跳动, 下游的停稳判定每帧都判成"还在动"。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


# ============================================================
# 颜色表 (和 material.py / 校赛 schooltest2.py 是同一张表, 改一处要改三处)
# ============================================================

HSV_RANGES: Dict[str, List[Tuple[Tuple[int, int, int], Tuple[int, int, int]]]] = {
    "red":        [((0, 110, 90),   (10, 255, 255)),
                   ((170, 110, 90), (180, 255, 255))],
    "yellow":     [((10, 30, 110), (33, 255, 255))],
    "green":      [((50, 90, 60),  (80, 255, 255))],
    "blue":       [((100, 55, 80), (140, 255, 255))],
    "light_blue": [((86, 30, 130), (100, 255, 255))],
    "black":      [((0, 0, 0),     (180, 50, 150))],
}

# 一个像素同时符合两种颜色时 (表之间有重叠) 取排在前面的那个 —— 同票数不换人
COLOR_ORDER = ["red", "yellow", "green", "blue", "light_blue", "black"]
COLOR_CN = {"red": "红", "yellow": "黄", "green": "绿",
            "blue": "蓝", "light_blue": "浅蓝", "black": "黑"}
COLOR_BGR = {"red": (0, 0, 255), "yellow": (0, 255, 255), "green": (0, 255, 0),
             "blue": (255, 0, 0), "light_blue": (255, 180, 0), "black": (90, 90, 90)}

_MORPH = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
_MORPH_SMALL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

# 两种 Hough 方法各自的默认值。param1/param2 的**量纲不一样**, 对应的默认值见这里
HOUGH_DEFAULTS = {
    'alt':      {'dp': 1.5, 'param1': 300.0, 'param2': 0.85},
    'gradient': {'dp': 1.0, 'param1': 120.0, 'param2': 15.0},
}


# ============================================================
# 参数 (全封在这里, 0 = auto)
# ============================================================

@dataclass(frozen=True)
class CircleParams:
    """找圆的全部参数。跟 open_cap 的 width/height/fps 一样, 每个都有能用的默认值。

    method      'alt' (默认) 或 'gradient'。ALT 认完整的圆最干净, 空图不出假圆;
                被遮挡时换 'gradient'。
    param1/param2  0 = 用这个方法自己的默认值 (见 HOUGH_DEFAULTS)。
                **换方法要一起换这两个** —— 量纲不同, 照搬必然检不出圆。
    min_dist    圆心距小于它就当同一个圆, 0 = max(10, min_radius)。
    min/max_radius  0 = 按帧短边的 r_min_frac / r_max_frac 换算。
    r_min_frac/r_max_frac  半径下限/上限占**帧短边**的比例 (默认 0.04~0.45)。
    disk_scale  认颜色时取样盘 = 检出半径 × 这个值, 比圆小一圈躲开边缘过渡色。
    merge_frac  小圆圆心落在大圆半径 × 这个值以内 = 同一块的同心圆, 合并。
    blur        找圆前的 medianBlur 核 (必须奇数; <=1 表示不模糊)。
    """
    method: str = 'alt'
    dp: float = 0.0
    param1: float = 0.0
    param2: float = 0.0
    min_dist: int = 0
    min_radius: int = 0
    max_radius: int = 0
    r_min_frac: float = 0.04
    r_max_frac: float = 0.45
    disk_scale: float = 0.85
    merge_frac: float = 1.0
    blur: int = 5

    def resolved(self, height: int, width: int) -> 'CircleParams':
        """把 0=auto 的参数按这一帧的分辨率换算成实际值, 返回一份新的。

        幂等: 换算过的再喂进来不会变 (所以 resolve 完的可以缓存起来反复用)。
        """
        p = self
        method = p.method
        if method == 'alt' and not hasattr(cv2, 'HOUGH_GRADIENT_ALT'):
            print(f'[找圆] 这个 OpenCV（{cv2.__version__}）没有 HOUGH_GRADIENT_ALT, '
                  f'改用经典 GRADIENT')
            method = 'gradient'
        if method not in HOUGH_DEFAULTS:
            raise ValueError(f'不认识的方法 {p.method!r}（只有 alt / gradient）')
        d = HOUGH_DEFAULTS[method]

        dp = p.dp or d['dp']
        param1 = p.param1 or d['param1']
        param2 = p.param2 or d['param2']
        if method == 'alt' and param2 > 1.0:
            print(f'[找圆] ALT 的 param2 是"完美度", 要 0~1（默认 0.85）; 现在的 '
                  f'{param2} 是 GRADIENT 那套的量纲, 会一个圆都检不出来')

        short = min(int(height), int(width))
        min_radius = p.min_radius or max(4, int(short * p.r_min_frac))
        max_radius = p.max_radius or max(min_radius + 1, int(short * p.r_max_frac))
        # 圆心太近 = 同一个圆的重复检出, 下限跟最小半径绑
        min_dist = p.min_dist or max(10, min_radius)

        return replace(p, method=method, dp=dp, param1=param1, param2=param2,
                       min_radius=int(min_radius), max_radius=int(max_radius),
                       min_dist=int(min_dist))

    def describe(self) -> str:
        """一行把当前参数打出来 (跟 open_cap 打完实际分辨率那样, 方便对着调)。"""
        return (f'方法={self.method} dp={self.dp:g} param1={self.param1:g} '
                f'param2={self.param2:g} minDist={self.min_dist} '
                f'r=[{self.min_radius},{self.max_radius}] '
                f'disk={self.disk_scale:g} merge={self.merge_frac:g} blur={self.blur}')


def default_params(method: str = 'alt') -> CircleParams:
    """默认参数。method='gradient' 换经典那套 (默认值不同, 见 HOUGH_DEFAULTS)。"""
    return CircleParams(method=method)


# ============================================================
# 结果
# ============================================================

@dataclass(frozen=True)
class Circle:
    """检出的一个圆。

    color  圆内占比最高的颜色 (认不出就是 None —— **不影响**这个圆被认出来)
    hsv    整个圆内的**平均** HSV (0.85r 的盘, 避开边缘过渡色), 跟阈值表无关,
           就是给你照着调 HSV_RANGES 用的
    center (u, v) 像素; radius 像素; fill 那种颜色占取样盘的比例 (只是信息)
    merged 这一块上并掉了几个同心圆 (靶心的内外圈、圆台的两层)。0 = 就检出一个圆
    """
    color: Optional[str]
    hsv: Optional[Tuple[int, int, int]]
    center: Tuple[int, int]
    radius: int
    fill: float = 0.0
    merged: int = 0

    @property
    def area(self) -> float:
        return math.pi * self.radius * self.radius

    def describe(self) -> str:
        col = f'{COLOR_CN.get(self.color, self.color)} {self.fill:.0%}' if self.color \
            else '颜色认不出'
        return (f'圆心({self.center[0]}, {self.center[1]}) r={self.radius}px '
                f'{col} HSV={self.hsv}'
                + (f' 同心层×{self.merged + 1}' if self.merged else ''))


# ============================================================
# 颜色
# ============================================================

def _mask_of_color(hsv: np.ndarray, color: str) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in HSV_RANGES[color]:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    # 先 open 掉小点, 再 close 补洞。open 用 3x3 更温和, 避免吃掉薄的圆环
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _MORPH_SMALL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH)
    return mask


def _mean_hsv(hsv: np.ndarray, mask: np.ndarray) -> Optional[Tuple[int, int, int]]:
    """mask 内像素的平均 HSV。H 用**圆均值**: 红色跨 0/180 两头, 直接取算术平均
    会算出个青绿来 (0 和 179 平均是 90)。"""
    sel = mask > 0
    if not np.any(sel):
        return None
    ang = np.radians(hsv[:, :, 0][sel].astype(np.float64))
    h = int(round(np.degrees(np.arctan2(np.sin(ang).mean(),
                                        np.cos(ang).mean())))) % 180
    return (h, int(round(hsv[:, :, 1][sel].mean())), int(round(hsv[:, :, 2][sel].mean())))


# ============================================================
# 主流程
# ============================================================

def find_circles(frame: np.ndarray,
                 params: Optional[CircleParams] = None,
                 **overrides) -> List[Circle]:
    """找一帧里的圆, 按面积**从大到小**返回 (同心圆已合并)。找不到就是空列表。

    params    一套参数 (用 default_params() 造)。不给就用默认的 ALT 那套。
    overrides 按名字只改几个, 比如 find_circle(frame, param2=0.9)。名字写错会
              直接报 TypeError, 不会静默忽略。

    frame 是 BGR (cv2 读出来的样子)。颜色只是附带信息, 不参与判定。
    """
    p = default_params() if params is None else params
    if overrides:
        p = replace(p, **overrides)
    h, w = frame.shape[:2]
    p = p.resolved(h, w)

    if p.blur and p.blur > 1:
        k = int(p.blur) | 1                                   # medianBlur 只吃奇数
        gray = cv2.medianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), k)
    else:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    cvm = (cv2.HOUGH_GRADIENT_ALT if p.method == 'alt' else cv2.HOUGH_GRADIENT)
    found = cv2.HoughCircles(gray, cvm, dp=p.dp, minDist=p.min_dist,
                            param1=p.param1, param2=p.param2,
                            minRadius=p.min_radius, maxRadius=p.max_radius)
    if found is None:
        return []

    blurred = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    masks = {c: _mask_of_color(hsv, c) for c in COLOR_ORDER}

    circles: List[Circle] = []
    for cx, cy, r in np.round(found[0]).astype(int):
        cx, cy, r = int(cx), int(cy), int(r)
        if r <= 1:
            continue
        # 取样盘比检出的小一圈: 边缘那圈是圆到背景的过渡色, 算进去会把占比压低
        disk = np.zeros(gray.shape, dtype=np.uint8)
        cv2.circle(disk, (cx, cy), max(2, int(r * p.disk_scale)), 255, -1)
        disk_area = float(cv2.countNonZero(disk))

        best_c, best_n = None, 0
        for c in COLOR_ORDER:
            n = cv2.countNonZero(cv2.bitwise_and(masks[c], disk))
            if n > best_n:
                best_c, best_n = c, n
        circles.append(Circle(color=best_c,
                              hsv=_mean_hsv(hsv, disk),
                              center=(cx, cy), radius=r,
                              fill=best_n / disk_area if disk_area > 0 else 0.0))

    return _merge_concentric(circles, p.merge_frac)


def find_circle(frame: np.ndarray,
                params: Optional[CircleParams] = None,
                **overrides) -> Optional[Circle]:
    """只挑**最大的那个圆** (下游对准/停稳都看它)。没有圆就是 None。"""
    circles = find_circles(frame, params, **overrides)
    return circles[0] if circles else None


def _merge_concentric(circles: List[Circle], merge_frac: float) -> List[Circle]:
    """把同一块的同心圆合成一个, 留大的那个。

    判据是"小圆的圆心落在大圆的盘里"(圆心距 ≤ 大圆半径 × merge_frac), 不是"两圆心
    几乎重合" —— 靶心斜着看时内圈是偏的 (实测那个圆台是 r=62@(264,218) 和
    r=51@(298,218), 圆心差 34px), 按"重合"判就合不上; 而两块并排的东西圆心至少隔
    2 个半径, 合不上。

    **自己先按面积排一遍**, 不靠调用方排: 判据是"小的落进大的盘里", 先收下谁就决定
    留下谁 —— 倒着喂进来就会留下小的那个。
    """
    kept: List[Circle] = []
    for m in sorted(circles, key=lambda k: -k.area):
        for i, k in enumerate(kept):
            dist = max(abs(m.center[0] - k.center[0]), abs(m.center[1] - k.center[1]))
            if dist <= k.radius * merge_frac:
                kept[i] = replace(k, merged=k.merged + 1)
                break
        else:
            kept.append(m)
    return kept


# ============================================================
# 可视化
# ============================================================

def draw_debug(frame: np.ndarray, circles: List[Circle],
               params: Optional[CircleParams] = None) -> np.ndarray:
    """把检出的圆画到图上 (返回新图, 不改原图), 给 /camera/image_result 用。"""
    out = frame.copy()
    for i, c in enumerate(circles):
        col = COLOR_BGR.get(c.color or '', (255, 255, 255))
        thick = 3 if i == 0 else 1                       # 最大的那个画粗一点
        cv2.circle(out, c.center, c.radius, col, thick)
        cv2.circle(out, c.center, 3, (0, 255, 255), -1)
        cv2.putText(out, f'{i + 1}: r={c.radius} {c.color or "-"}',
                    (c.center[0] - c.radius, c.center[1] - c.radius - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(out, f'{i + 1}: r={c.radius} {c.color or "-"}',
                    (c.center[0] - c.radius, c.center[1] - c.radius - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)

    head = f'circles: {len(circles)}'
    if circles:
        head += f'  biggest r={circles[0].radius}px @{circles[0].center}'
    if params is not None:
        head += f'  [{params.method} p2={params.param2:g} ' \
                f'r<{params.max_radius}]'
    cv2.putText(out, head, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
    cv2.putText(out, head, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
    return out


# ============================================================
# 自检: python3 circle.py (合成图, 不碰任何设备)
# ============================================================

def _selftest() -> bool:
    """铺一张灰底图放几个圆, 看找不找得到、颜色和 HSV 报得对不对。

    ⚠ 图形一律用 LINE_AA 画: ALT 对硬边极敏感, LINE_8 的硬边会平白漏掉一堆半径
    (640x480 下扫 40 个半径, 硬边漏 18 个, 抗锯齿只漏 1 个)。拿合成图调参一定开抗锯齿。
    """
    h, w = 480, 640
    bg = (120, 120, 120)
    bgr = {'red': (0, 0, 220), 'yellow': (0, 220, 220), 'green': (0, 200, 0),
           'blue': (220, 0, 0), 'light_blue': (220, 215, 130), 'black': (25, 25, 25)}
    img = np.full((h, w, 3), bg, np.uint8)

    rmin = int(min(h, w) * CircleParams().r_min_frac)      # 当前的最小半径
    big = rmin + 8                                         # 刚过下限的圆
    # (名字, 颜色, 圆心, 半径)。灰圆要比底色**亮**, 不然跟底色一样就没有边
    cases = [('红', 'red', (100, 130), 52),
             ('黄', 'yellow', (320, 130), big + 10),
             ('蓝', 'blue', (540, 130), 52),
             ('浅蓝', 'light_blue', (100, 350), big),
             ('黑', 'black', (320, 350), 52),
             ('灰(无颜色)', None, (540, 350), 52)]
    for _, color, c, r in cases:
        cv2.circle(img, c, r, bgr[color] if color else (185, 185, 185), -1, cv2.LINE_AA)
    # 靶心: 四层同心圆, 应该被合并成**一个**圆 (留最大的那层)
    for r, color in ((70, bgr['black']), (50, bgr['red']),
                     (30, bgr['black']), (12, bgr['red'])):
        cv2.circle(img, (320, 240), r, color, -1, cv2.LINE_AA)
    # 一行字, 顺手看看底线: 下限以下的小圆不该被找到
    small = (595, 445)
    cv2.circle(img, small, max(6, rmin - 20), bgr['green'], -1, cv2.LINE_AA)

    p = default_params()
    circles = find_circles(img, p)
    print(f'参数: {p.resolved(h, w).describe()}')
    print(f'（另画了个 r={max(6, rmin - 20)} 的小圆, 下限是 {rmin}, 它不该被找到）')
    print(f'检出 {len(circles)} 个圆:')
    for c in circles:
        print(f'  {c.describe()}')

    def near(x, y, tol=15):
        m = min(circles, key=lambda k: (k.center[0] - x) ** 2 + (k.center[1] - y) ** 2,
                default=None)
        return m if m and abs(m.center[0] - x) <= tol and abs(m.center[1] - y) <= tol \
            else None

    ok = True

    def check(name, cond, extra=''):
        nonlocal ok
        ok = ok and bool(cond)
        print(f'{"PASS" if cond else "FAIL"}  {name}{"  " + extra if extra else ""}')

    for label, color, (cx, cy), r in cases:
        m = near(cx, cy)
        check(f'{label} 圆 (r={r}) 找得到、位置半径对',
              m is not None and abs(m.radius - r) <= max(8, 0.2 * r),
              f'→ {m.center} r={m.radius}' if m else '→ 没找到')
        if color:
            check(f'{label} 报的颜色对', m is not None and m.color == color,
                  f'→ {m.color}' if m else '')
            want = cv2.cvtColor(np.uint8([[bgr[color]]]), cv2.COLOR_BGR2HSV)[0][0]
            got = m.hsv if m else None
            dh = min(abs(got[0] - int(want[0])), 180 - abs(got[0] - int(want[0]))) \
                if got else 999
            check(f'{label} 报的 HSV 和色本身对得上',
                  got is not None and dh <= 10
                  and abs(got[1] - int(want[1])) <= 80 and abs(got[2] - int(want[2])) <= 80,
                  f'→ 报 {got}，这块色本身 {tuple(int(v) for v in want)}')
        else:
            check('灰圆认不出颜色（不影响判定）', m is not None and m.color is None)

    # 靶心: 四层同心圆合并成一个, 留最大的那层 (r=70), 圆心在正中间
    b = near(320, 240)
    check('靶心四层同心圆合并成一个 (留最大层 r=70)',
          b is not None and b.radius >= 60 and b.merged >= 1,
          f'→ {b.describe()}' if b else '→ 没找到')

    # 空图不该有圆 —— ALT 在这点上很干净 (合成图实测 0 个假圆)
    blank = np.full((h, w, 3), 120, np.uint8)
    n_blank = len(find_circles(blank, p))
    check('纯色空图不报圆', n_blank == 0, f'→ {n_blank} 个')

    check(f'r={max(6, rmin - 20)} 的小圆（下限 {rmin}）被挡掉',
          near(*small) is None)

    # 两套方法都得出得来 (量纲不同的那两组默认值都得能用)
    for method in ('gradient',):
        n = len(find_circles(img, default_params(method)))
        check(f'{method} 那套默认值也检得出圆', n >= len(cases) // 2, f'→ {n} 个')

    print('\n' + ('全部通过' if ok else '有失败项'))
    return ok


if __name__ == '__main__':
    import sys
    sys.exit(0 if _selftest() else 1)

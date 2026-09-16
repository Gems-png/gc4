#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线验证找圆：合成图，不碰任何设备。

注意所有图形都用 LINE_AA 画：ALT 对硬边极敏感，LINE_8 的硬边会平白漏掉一堆半径
（640x480 下扫 40 个半径，硬边漏 18 个，抗锯齿只漏 1 个）。拿合成图调参一定开抗锯齿。

铺一张灰底图，放几个圆（红/黄/蓝/浅蓝/黑/灰）+ 一个绿方块 + 一个太小的小圆，
看 detect_materials 找不找得到、颜色和 HSV 报得对不对。

跑：python3 check_circle.py
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cv2
import numpy as np
import schooltest2 as st

# light_blue 原来是 (230,170,60)，它 H≈100.5 正好压在 blue(100~128) 和 light_blue(86~100)
# 的分界线上，被算成 blue 是颜色表本身的重叠，不是找圆的毛病 —— 换成 H≈91 的浅蓝。
BGR = {'red': (0, 0, 220), 'yellow': (0, 220, 220), 'green': (0, 200, 0),
       'blue': (220, 0, 0), 'light_blue': (220, 215, 130), 'black': (25, 25, 25)}

H, W = 480, 640                                   # 和脚本默认的 640x480 一致
BG = (120, 120, 120)
img = np.full((H, W, 3), BG, np.uint8)           # 灰底

# 半径跟着脚本里的 CIRCLE_R_MIN_FRAC 走，别再写死像素值 —— 一调那个比例，
# 写死的半径就会掉到下限以下，测试报一堆"没找到"，看着像找圆的毛病，其实是被过滤了。
RMIN = int(min(H, W) * st.CIRCLE_R_MIN_FRAC)     # 脚本当前的最小半径
BIG = RMIN + 8                                   # 刚过下限的圆

# (名字, 颜色, 圆心, 半径)。灰圆要比底色亮，不然和底色一样就根本没有边（第一次就画错了）
CIRCLES = [
    ('红', 'red',          (100, 130), 52),
    ('黄', 'yellow',       (320, 130), BIG + 10),
    ('蓝', 'blue',         (540, 130), 52),
    ('浅蓝', 'light_blue', (100, 350), BIG),
    ('黑', 'black',        (320, 350), 52),
    ('灰(无颜色)', None,     (540, 350), 52),
]
for name, color, c, r in CIRCLES:
    cv2.circle(img, c, r, BGR[color] if color else (185, 185, 185), -1, cv2.LINE_AA)
# 绿方块塞在四个圆的中间空档，别压到任何圆上（压上去就不是完整的圆了）
SQUARE = ((165, 195), (255, 285))
cv2.rectangle(img, SQUARE[0], SQUARE[1], BGR['green'], -1, cv2.LINE_AA)
# 一个故意画得太小的圆：下限以下应该找不到（这就是 CIRCLE_R_MIN_FRAC 的作用）
SMALL = ((595, 445), max(6, RMIN - 20))
cv2.circle(img, SMALL[0], SMALL[1], BGR['green'], -1, cv2.LINE_AA)

hsv = cv2.cvtColor(cv2.GaussianBlur(img, (5, 5), 0), cv2.COLOR_BGR2HSV)
hp = st.eff_hough(st.default_hp(), H, W)
print(f'Hough 实际参数: {hp}')
print(f'（另画了个 r={SMALL[1]} 的小圆，下限是 {RMIN}，它不该被找到）\n')

mats = st.detect_materials(img, hsv, hp)
print(f'检出 {len(mats)} 个圆:')
for c in mats:
    col = f'{c.color} {c.fill:.0%}' if c.color else '颜色认不出'
    print(f'  {str(c.center):>12s} r={c.radius:3d}  {col}  HSV={c.hsv}')

# ---- 判定 ----
def nearest(cx, cy, tol=15):
    """只认离得够近的圆。原来不限距离地取最近那个，红圆没检出时会把远处的黄圆
    当成它，报出来的失败看着像"位置错"，其实是"没找到"。"""
    if not mats:
        return None
    m = min(mats, key=lambda k: (k.center[0] - cx) ** 2 + (k.center[1] - cy) ** 2)
    return m if abs(m.center[0] - cx) <= tol and abs(m.center[1] - cy) <= tol else None

ok = True
def check(name, cond, extra=''):
    global ok
    ok = ok and cond
    print(f'{"PASS" if cond else "FAIL"}  {name}{"  " + extra if extra else ""}')

for label, color, (cx, cy), r in CIRCLES:
    m = nearest(cx, cy)
    hit = (m is not None and abs(m.center[0] - cx) <= 15 and abs(m.center[1] - cy) <= 15
           and abs(m.radius - r) <= max(8, 0.2 * r))
    check(f'{label} 圆 (r={r}) 找得到、位置半径对', hit,
          f'→ {m.center} r={m.radius}' if m else '→ 没找到')
    if color:
        check(f'{label} 报的颜色对', m is not None and m.color == color,
              f'→ {m.color}' if m else '')
        # 报出来的 HSV 得跟这块色本身对得上，不然拿去调阈值就是错的
        want_hsv = cv2.cvtColor(np.uint8([[BGR[color]]]), cv2.COLOR_BGR2HSV)[0][0]
        got = m.hsv if m else None
        dh = min(abs(got[0] - int(want_hsv[0])), 180 - abs(got[0] - int(want_hsv[0]))) if got else 999
        ok_hsv = got is not None and dh <= 10 and abs(got[1] - int(want_hsv[1])) <= 80 \
            and abs(got[2] - int(want_hsv[2])) <= 80
        check(f'{label} 报的 HSV 和色本身对得上', ok_hsv,
              f'→ 报 {got}，这块色本身 {tuple(int(v) for v in want_hsv)}')
    else:
        # 灰色圆：认不出颜色才对，但圆本身必须找到
        check('灰圆认不出颜色（不影响判定）', m is not None and m.color is None,
              f'→ {m.color}' if m else '')

# 方块：现在只看圆，方块里能不能圈出圆 —— 这是这个判据的已知边界，如实看
sq = [m for m in mats if SQUARE[0][0] <= m.center[0] <= SQUARE[1][0]
      and SQUARE[0][1] <= m.center[1] <= SQUARE[1][1]]
print(f'\n注：绿方块里{"也圈出了 " + str(len(sq)) + " 个圆（只看圆的判据挡不住方块）" if sq else "没圈出圆"}')

# 下限以下的小圆：不该被找到（被 minRadius 挡掉）
near_small = [m for m in mats
              if abs(m.center[0] - SMALL[0][0]) <= 15 and abs(m.center[1] - SMALL[0][1]) <= 15]
check(f'r={SMALL[1]} 的小圆（下限 {RMIN}）被挡掉', not near_small,
      f'→ 找到了 {near_small[0].radius}' if near_small else '')

# 空图：不该有圆
blank = np.full((H, W, 3), 120, np.uint8)
blank_mats = st.detect_materials(blank, cv2.cvtColor(blank, cv2.COLOR_BGR2HSV), hp)
check('纯色空图不报圆', len(blank_mats) == 0, f'→ {len(blank_mats)} 个')

print('\n' + ('全部通过' if ok else '有失败项'))
sys.exit(0 if ok else 1)

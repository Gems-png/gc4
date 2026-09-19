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
def _h_overlap(a, b):
    """HSV_RANGES 里两种颜色的 H 区间重叠段，没有就返回 None。

    叠上了就是**表**的问题：一个像素同时符合两种颜色时，detect_materials 取
    COLOR_ORDER 里靠前的那个（`n > best_n` 的同数不换人），跟找圆、跟代码都无关。
    """
    for lo1, hi1 in st.HSV_RANGES[a]:
        for lo2, hi2 in st.HSV_RANGES[b]:
            lo, hi = max(lo1[0], lo2[0]), min(hi1[0], hi2[0])
            if lo <= hi:
                return lo, hi
    return None


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
        right = m is not None and m.color == color
        check(f'{label} 报的颜色对', right, f'→ {m.color}' if m else '')
        if not right and m is not None and m.color is not None:
            ov = _h_overlap(m.color, color)
            if ov:
                print(f'      注：表里 {m.color} 和 {color} 的 H 在 {ov[0]}~{ov[1]} 叠着，'
                      f'{m.color} 在 COLOR_ORDER 里靠前所以赢了 —— 是 HSV_RANGES 要收窄，'
                      f'不是找圆的毛病')
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

# ---- 对准：检出的圆心 -> 该往哪边挪 ----
# 对准点默认是画面正中。方向约定：画面上方 = 车头前方，所以圆心偏右就发右(0x33)、
# 偏下发后(0x31)；两轴差一样大时先修横向。这里把 6 个圆的期望值手写死再比一遍，
# 免得 align_cmd 里的正负号写反了没人发现（反了车会朝反方向一路挪出去）。
aim = st.resolve_aim(-1, -1, H, W)
check('对准点默认 = 画面正中', aim == (320, 240), f'{aim}')
AIM_WANT = [('红(左上)', 100, 130, st.CMD_ADJUST_LEFT),
            ('蓝(右上)', 540, 130, st.CMD_ADJUST_RIGHT),
            ('浅蓝(左下)', 100, 350, st.CMD_ADJUST_LEFT),
            ('黄(中上)', 320, 130, st.CMD_ADJUST_FWD),
            ('黑(中下)', 320, 350, st.CMD_ADJUST_BACK),
            ('灰(右下)', 540, 350, st.CMD_ADJUST_RIGHT)]
for label, cx, cy, want in AIM_WANT:
    m = nearest(cx, cy)
    err = (m.center[0] - aim[0], m.center[1] - aim[1]) if m else None
    got = st.align_cmd(*err) if err else None
    check(f'{label} -> {st.CMD_CN[want]}', got == want,
          f'误差 {err} -> {st.CMD_CN.get(got, got)}')

# 正对准的那种：圆心就在对准点上，这时不该再发微调，该发抓取了
img3 = np.full((H, W, 3), BG, np.uint8)
cv2.circle(img3, (W // 2, H // 2), 60, BGR['red'], -1, cv2.LINE_AA)
hsv3 = cv2.cvtColor(cv2.GaussianBlur(img3, (5, 5), 0), cv2.COLOR_BGR2HSV)
m3 = st.detect_materials(img3, hsv3, hp)
err3 = ((m3[0].center[0] - aim[0], m3[0].center[1] - aim[1]) if m3 else None)
check('同心圆放在正中 = 已经对准（不用再挪）',
      err3 is not None and st.align_cmd_for(err3[0], err3[1],
                                            st.AIM_TOL_X, st.AIM_TOL_Y) is None,
      f'误差 {err3}')

# 空图：不该有圆
blank = np.full((H, W, 3), 120, np.uint8)
blank_mats = st.detect_materials(blank, cv2.cvtColor(blank, cv2.COLOR_BGR2HSV), hp)
check('纯色空图不报圆', len(blank_mats) == 0, f'→ {len(blank_mats)} 个')


# ---- 报的 HSV 必须是"整个圆的平均"，不能被阈值表筛过 ----
# 这是踩过的坑：原来 HSV 取的是"匹配上某种颜色的那些像素"的值，等于拿阈值筛完再告诉你
# 阈值筛出来的结果 —— 范围没调对时只会看到漏进去的一两个像素的 HSV。
# 这里画一个 90% 是表外颜色(品红 H≈150) + 10% 绿的圆：报的应该接近品红，而不是绿。
img2 = np.full((H, W, 3), BG, np.uint8)
cv2.circle(img2, (320, 240), 60, (200, 0, 200), -1, cv2.LINE_AA)      # 品红：表里没有
cv2.circle(img2, (320, 240), 20, BGR['green'], -1, cv2.LINE_AA)       # 中间塞点绿：会被判成绿
hsv2 = cv2.cvtColor(cv2.GaussianBlur(img2, (5, 5), 0), cv2.COLOR_BGR2HSV)
m2 = st.detect_materials(img2, hsv2, hp)
c2 = None
for m in m2:
    if abs(m.center[0] - 320) <= 15 and abs(m.center[1] - 240) <= 15:
        c2 = m
if c2 is None:
    check('品红+绿点缀的圆找得到', False)
else:
    green_hsv = cv2.cvtColor(np.uint8([[BGR['green']]]), cv2.COLOR_BGR2HSV)[0][0]
    magenta_hsv = cv2.cvtColor(np.uint8([[(200, 0, 200)]]), cv2.COLOR_BGR2HSV)[0][0]
    check('品红+绿点缀的圆找得到', True, f'color={c2.color} fill={c2.fill:.0%}')
    d_mag = min(abs(c2.hsv[0] - int(magenta_hsv[0])), 180 - abs(c2.hsv[0] - int(magenta_hsv[0])))
    d_grn = min(abs(c2.hsv[0] - int(green_hsv[0])), 180 - abs(c2.hsv[0] - int(green_hsv[0])))
    check('报的是整圆的平均（偏向品红），不是那 10% 绿的',
          d_mag < d_grn,
          f'报 H={c2.hsv[0]}，品红 H={int(magenta_hsv[0])}（距离 {d_mag}）'
          f' vs 绿 H={int(green_hsv[0])}（距离 {d_grn}）')

print('\n' + ('全部通过' if ok else '有失败项'))
sys.exit(0 if ok else 1)

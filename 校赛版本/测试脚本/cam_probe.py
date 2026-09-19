#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M2 相机探针：看当前控制值、抓帧存图、扫曝光找合适的值、找黄色圆。

这台相机（2M，现在挂在 /dev/video4）能控制曝光，所以先别急着在算法上补：
    v4l2-ctl -d /dev/video4 --list-ctrls
  暴露出来的问题是 `exposure_time_absolute` 现在顶在 **10000（最大值）**，
  auto_exposure=3（自动）时它是 inactive 用不上，一旦切成手动就会用 10000 —— 亮爆。
  brightness 也被抬到 50（默认 0），hue 顶到 2000（默认 0，这会把颜色整个拧偏）。

用法：
    python3 cam_probe.py                 # 看一眼现在什么样，存图 + 找黄圆
    python3 cam_probe.py --sweep         # 扫一遍曝光，给出建议值（存图对比）
    python3 cam_probe.py --exposure 150 --brightness 0   # 指定值抓一帧看效果
    python3 cam_probe.py --circle        # 顺便跑一遍脚本里的找圆，报黄圆在哪
"""
import argparse
import os
import sys
import time

os.environ.setdefault('OPENCV_LOG_LEVEL', 'ERROR')
import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import schooltest2 as st

OUT_DIR = '/tmp/m2probe'

# OpenCV 的 CAP_PROP_* -> V4L2 名字（打印用）
CTRLS = [('亮度', cv2.CAP_PROP_BRIGHTNESS, 'brightness'),
         ('对比度', cv2.CAP_PROP_CONTRAST, 'contrast'),
         ('饱和度', cv2.CAP_PROP_SATURATION, 'saturation'),
         ('色调hue', cv2.CAP_PROP_HUE, 'hue'),
         ('增益', cv2.CAP_PROP_GAIN, 'gain'),
         ('自动曝光', cv2.CAP_PROP_AUTO_EXPOSURE, 'auto_exposure'),
         ('曝光', cv2.CAP_PROP_EXPOSURE, 'exposure_time_absolute')]


def dump_ctrls(cap, dev):
    print(f'--- {dev} 现在的控制值 ---')
    for cn, prop, v4l2 in CTRLS:
        v = cap.get(prop)
        print(f'  {cn:<8} (v4l2 {v4l2:<24}) = {v:g}')
    print('  （详细范围：v4l2-ctl -d %s --list-ctrls）' % dev)


def stats(frame, tag=''):
    """画面亮暗：V 通道的中位数、顶到 250 以上的比例（过曝）、压在 30 以下的比例。
    过曝那一项是这次的关键 —— 黄圆糊成一片白就是它。"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    v = hsv[:, :, 2]
    hot = float((v >= 250).mean())
    dark = float((v <= 30).mean())
    s = (f'{tag}中位V={int(np.median(v)):3d}  过曝(V>=250) {hot:5.1%}  '
         f'欠曝(V<=30) {dark:5.1%}  BGR均值={frame.reshape(-1, 3).mean(0).round(0)}')
    return s, hot


def yellow_report(frame, verbose=True):
    """按脚本里的 HSV 表找黄色，报占比最大的那块在哪。"""
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
    masks = st.color_masks(hsv)
    out = {}
    for c in st.COLOR_ORDER:
        n = int(cv2.countNonZero(masks[c]))
        out[c] = n / masks[c].size
    if verbose:
        top = sorted(out.items(), key=lambda kv: -kv[1])[:4]
        print('  颜色占比: ' + '  '.join(f'{st.COLOR_CN[k]}{v:.2%}' for k, v in top))
    return out, hsv


def biggest_yellow(hsv, hp):
    """黄色掩码里最大的那块连通域，返回 (圆心, 半径, 占画面比例)。"""
    m = st._mask_of_color(hsv, 'yellow')
    n, lab, stats_, cent = cv2.connectedComponentsWithStats(m, 8)
    if n <= 1:
        return None
    i = 1 + int(np.argmax(stats_[1:, cv2.CC_STAT_AREA]))
    area = int(stats_[i, cv2.CC_STAT_AREA])
    if area < 200:
        return None
    cx, cy = cent[i]
    return (int(cx), int(cy)), int(np.sqrt(area / np.pi)), area / m.size


def shot(cap, name, settle=8):
    """丢掉前几帧（自动曝光/白平衡收敛要时间），再取一帧存图。"""
    frame = None
    for _ in range(settle):
        ok, f = cap.read()
        if ok and f is not None:
            frame = f
        time.sleep(0.03)
    if frame is None:
        return None
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, f'{name}.png')
    cv2.imwrite(path, frame)
    return frame, path


def set_ctrl(cap, prop, val, name):
    before = cap.get(prop)
    cap.set(prop, val)
    after = cap.get(prop)
    flag = 'OK' if abs(after - val) < 1e-6 else '没设上'
    print(f'    {name}: 要 {val:g} -> 现在 {after:g}（原来 {before:g}）[{flag}]')
    return abs(after - val) < 1e-6


def main():
    ap = argparse.ArgumentParser(description='M2 相机探针')
    ap.add_argument('--dev', default='name:2M', help="设备；默认按板卡名找 2M")
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fourcc', default='MJPG')
    ap.add_argument('--sweep', action='store_true', help='扫一遍曝光存图对比')
    ap.add_argument('--exposure', type=float, default=None, help='指定曝光(绝对)')
    ap.add_argument('--brightness', type=float, default=None)
    ap.add_argument('--hue', type=float, default=None)
    ap.add_argument('--gain', type=float, default=None)
    ap.add_argument('--circle', action='store_true', help='跑一遍脚本里的找圆')
    args = ap.parse_args()

    dev = st.resolve_device(args.dev)
    if not dev:
        print(f'找不到设备 {args.dev}（python3 schooltest2.py --list 看现状）')
        return 1
    print(f'用 {dev}（"{st.board_name(dev)}"）')

    cap = st.open_camera(dev, args.width, args.height, 30.0, args.fourcc)
    if cap is None:
        print(f'打不开 {dev} —— 相机可能被别的程序占着（fuser -v {dev} 看看）')
        return 1
    try:
        dump_ctrls(cap, dev)

        if args.sweep:
            print('--- 扫曝光（先关自动曝光，再逐个值抓帧存图）---')
            set_ctrl(cap, cv2.CAP_PROP_AUTO_EXPOSURE, st.AUTO_EXPOSURE_MANUAL, '自动曝光')
            if args.brightness is not None:
                set_ctrl(cap, cv2.CAP_PROP_BRIGHTNESS, args.brightness, '亮度')
            # 这台相机 exposure 范围 78~10000（v4l2-ctl --list-ctrls 看的）
            best = None
            for exp in (78, 120, 200, 312, 500, 800, 1500, 3000, 10000):
                if not set_ctrl(cap, cv2.CAP_PROP_EXPOSURE, exp, '曝光'):
                    break
                got = shot(cap, f'sweep_exp{exp}')
                if got is None:
                    print('    读不到帧')
                    continue
                frame, path = got
                s, hot = stats(frame, f'曝光{exp:>5}  ')
                print(f'  {s}')
                if best is None and hot < 0.01:
                    best = exp
            if best:
                print(f'  建议曝光 ≈ {best}（第一个过曝<1% 的）—— 再用 --exposure {best} 看一眼')
            print(f'  图都在 {OUT_DIR}/sweep_exp*.png，自己瞄一眼挑一个')
            return 0

        # 单次抓帧：先把手动曝光摆上，再抓
        touched = False
        if args.exposure is not None:
            set_ctrl(cap, cv2.CAP_PROP_AUTO_EXPOSURE, st.AUTO_EXPOSURE_MANUAL, '自动曝光')
            set_ctrl(cap, cv2.CAP_PROP_EXPOSURE, args.exposure, '曝光')
            touched = True
        if args.brightness is not None:
            set_ctrl(cap, cv2.CAP_PROP_BRIGHTNESS, args.brightness, '亮度')
            touched = True
        if args.hue is not None:
            set_ctrl(cap, cv2.CAP_PROP_HUE, args.hue, '色调hue')
            touched = True
        if args.gain is not None:
            set_ctrl(cap, cv2.CAP_PROP_GAIN, args.gain, '增益')
            touched = True
        if touched:
            dump_ctrls(cap, dev)

        got = shot(cap, 'now')
        if got is None:
            print('读不到帧 —— 这个节点多半不是取流节点（用 v4l2-ctl --list-formats-ext 确认）')
            return 1
        frame, path = got
        s, hot = stats(frame)
        print(f'--- 抓到的这一帧 ---\n  {s}')
        if hot > 0.02:
            print(f'  ⚠ 有 {hot:.1%} 的像素已经顶到 255，黄圆会糊成白块，'
                  f'先把曝光压下来（--sweep 找值）')
        print(f'  存图: {path}')

        _, hsv = yellow_report(frame)

        y = biggest_yellow(hsv, st.default_hp())
        if y:
            c, r, frac = y
            print(f'  黄色最大一块: 圆心 {c} 半径≈{r}px 占画面 {frac:.2%}')
        else:
            print('  没找到成块的黄色（可能画面里就没有黄色，或者过曝/偏色）')

        if args.circle:
            mats, hp = None, st.eff_hough(st.default_hp(), frame.shape[0], frame.shape[1])
            mats = st.detect_materials(frame, hsv, hp)
            print(f'--- 脚本的找圆（{hp["method"]} param2={hp["param2"]}，'
                  f'半径 {hp["min_radius"]}~{hp["max_radius"]}px）---')
            if not mats:
                print('  一个圆都没找到')
            for m in mats:
                cn = f'{st.COLOR_CN.get(m.color, "?")}({st.COLOR_CODE.get(m.color, "")})' \
                    if m.color else '颜色认不出'
                hsvs = f'{m.hsv[0]},{m.hsv[1]},{m.hsv[2]}' if m.hsv else '-'
                print(f'  {m.center} r={m.radius:3d} {cn} 占比{m.fill:.0%} HSV={hsvs}')
            vis = frame.copy()
            for m in mats:
                cv2.circle(vis, m.center, m.radius, st.COLOR_BGR.get(m.color, (255, 255, 255)), 3)
            p = os.path.join(OUT_DIR, 'now_circles.png')
            cv2.imwrite(p, vis)
            print(f'  画了圆圈的图: {p}')
        return 0
    finally:
        cap.release()


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线验证 CameraWorker 能跑起来（不开真摄像头，用假 VideoCapture 顶替）。

这个测试就是冲着踩过的那个坑写的：物料那条线程里 hp 换算用了还没赋值的 frame，
线程一启动就崩，主循环却以为它活着 —— 表现成"只打开了一个摄像头"。
假 cap 一跑就复现。

跑：python3 check_worker.py
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import time
import cv2
import numpy as np
import schooltest2 as st

ok = True
def check(name, cond, extra=''):
    global ok
    ok = ok and cond
    print(f'{"PASS" if cond else "FAIL"}  {name}{"  " + extra if extra else ""}')


class FakeCap:
    """假的 VideoCapture：按时吐同一张合成图。"""
    def __init__(self, w=640, h=480):
        self.w, self.h = w, h
        self.frame = np.full((h, w, 3), 120, np.uint8)
        cv2.circle(self.frame, (110, 110), 60, (0, 0, 220), -1, cv2.LINE_AA)

    def isOpened(self):
        return True

    def set(self, *a):
        return True

    def get(self, prop):
        return {cv2.CAP_PROP_FRAME_WIDTH: self.w,
                cv2.CAP_PROP_FRAME_HEIGHT: self.h,
                cv2.CAP_PROP_FPS: 30.0,
                cv2.CAP_PROP_FOURCC: cv2.VideoWriter_fourcc(*'MJPG')}.get(prop, 0)

    def read(self):
        return True, self.frame.copy()

    def release(self):
        pass


class DeadCap(FakeCap):
    """打得开但一帧都读不到（metadata 节点那种）。"""
    def read(self):
        return False, None


def run_worker(role, cap, secs=1.2):
    st.open_camera = lambda *a, **k: cap          # 顶替掉真开相机那一步
    wk = st.CameraWorker(role, '/dev/fake', 640, 480, 30, 'MJPG')
    wk.start()
    time.sleep(secs)
    wk.stop()
    time.sleep(0.1)
    return wk


print('--- 物料路 ---')
wk = run_worker('material', FakeCap())
check('物料线程没崩', wk.error == '', f'error={wk.error!r}')
check('物料 hp 换算出来了（原来就是这步崩的）', bool(wk.hp), f'{wk.hp}')
mats, _ = wk.material_detail()
check('物料认出了那个红圆', len(mats) == 1 and mats[0].color == 'red',
      f'{[(m.center, m.radius, m.color) for m in mats]}')
check('报的 HSV 跟红对得上', mats and mats[0].hsv == (0, 255, 220),
      f'{mats[0].hsv if mats else None}')
check('物料出帧了', wk.snapshot()[0] is not None)

print('--- tag 路 ---')
wk = run_worker('tag', FakeCap())
check('tag 线程没崩', wk.error == '', f'error={wk.error!r}')
check('tag 出帧了', wk.snapshot()[0] is not None)

print('--- 读不到帧的节点 ---')
wk = run_worker('material', DeadCap(), secs=0.6)
check('读不到帧会报 error（不是静默活着）', bool(wk.error), f'{wk.error[:40]}…')

print('\n' + ('全部通过' if ok else '有失败项'))
sys.exit(0 if ok else 1)

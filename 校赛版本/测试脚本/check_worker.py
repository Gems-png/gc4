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
import contextlib
import io
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


def run_worker(role, cap, secs=1.2, cam_rot=0):
    st.open_camera = lambda *a, **k: cap          # 顶替掉真开相机那一步
    wk = st.CameraWorker(role, '/dev/fake', 640, 480, 30, 'MJPG', cam_rot=cam_rot)
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

print('--- 相机装歪了转正（--cam-rot）---')
# 相机**逆时针**转了 90° 装着，画面里的东西就是**顺时针**躺着的，
# 所以要拿 ROTATE_90_COUNTERCLOCKWISE 把画面**逆时针**转回来。
# 这块像素几何是踩不起坑的（转反了物料会跑到画面外，或者前后左右全反），
# 所以这里拿一个亮块钉死：左上角 -> 左下角，就是逆时针转。
src = np.zeros((480, 640, 3), np.uint8)
cv2.circle(src, (100, 60), 20, (255, 255, 255), -1)
r1 = st.rot_frame(1, src)
check('相机逆时针90° -> 画面宽高互换', r1.shape[:2] == (640, 480), f'{r1.shape[:2]}')
ys, xs = np.nonzero(r1[:, :, 0])
check('左上角的块转到了左下角（=画面逆时针转）',
      abs(int(xs.mean()) - 60) <= 3 and abs(int(ys.mean()) - 539) <= 3,
      f'({int(xs.mean())},{int(ys.mean())})，逆时针转应该在 (60,539)')
check('rot 0 = 正装，画面不动', np.array_equal(st.rot_frame(0, src), src))
check('rot 2 = 180°', np.array_equal(st.rot_frame(2, src), cv2.rotate(src, cv2.ROTATE_180)))
check('rot 3 = 相机顺时针90°（画面顺时针转回来）',
      np.array_equal(st.rot_frame(3, src), cv2.rotate(src, cv2.ROTATE_90_CLOCKWISE)))
check('rot 4 取模回 0', np.array_equal(st.rot_frame(4, src), src))

# 转正之后识别照样得能用：红圆换了个位置，还得是那个红圆
wk = run_worker('material', FakeCap(), cam_rot=1)
frame = wk.snapshot()[0]
mats, hp = wk.material_detail()
check('转正后出帧尺寸跟着换', frame is not None and frame.shape[:2] == (640, 480),
      f'{None if frame is None else frame.shape[:2]}')
check('转正后半径范围按转正后的短边算（还是 480）',
      hp.get('min_radius') == int(480 * st.CIRCLE_R_MIN_FRAC), f'{hp.get("min_radius")}')
# FakeCap 的圆心 (110,110)、r=60，逆时针转完应该到 (110, 640-1-110=529)
check('转正后红圆在新位置认出来了',
      len(mats) == 1 and mats[0].color == 'red'
      and abs(mats[0].center[0] - 110) <= 6 and abs(mats[0].center[1] - 529) <= 6,
      f'{[(m.center, m.radius, m.color) for m in mats]}')

print('--- 读不到帧的节点 ---')
wk = run_worker('material', DeadCap(), secs=0.6)
check('读不到帧会报 error（不是静默活着）', bool(wk.error), f'{wk.error[:40]}…')


# ==================== 设备名解析：片段撞上好几块板卡时不许瞎猜 ====================
# 真机上踩的坑（2026-09-18）：笔记本上 "Integrated" 同时命中内置摄像头、内置红外、
# 刚插的 USB 摄像头，resolve_device 按编号挑了 /dev/video0 = 笔记本自己的摄像头，
# 而且是取流不出帧的那个节点 —— 表现成"明明插着 tag 相机却没画面"。
# 这里拿假的 sysfs 数据钉死：命中的板卡名不止一种就必须**拒绝**，不能挑一个凑合。
def with_fake_devices(nodes):
    """nodes = [(节点, 板卡名, [别名...])] → 顶掉 sysfs 那几个查询，返回还原函数。"""
    old = (st._video_nodes, st.board_name, st._v4l_aliases)
    st._video_nodes = lambda: [d for d, _, _ in nodes]
    st.board_name = lambda d: {n: name for n, name, _ in nodes}.get(d, '?')
    st._v4l_aliases = lambda: {n: list(a) for n, _, a in nodes}
    def restore():
        st._video_nodes, st.board_name, st._v4l_aliases = old
    return restore


print('--- 设备名解析 ---')
NODES = [('/dev/video0', 'Laptop Cam: L', ['usb-A-video-index0']),
         ('/dev/video1', 'Laptop Cam: L', ['usb-A-video-index1']),
         ('/dev/video4', 'TagCam: T', ['usb-B-video-index0']),
         ('/dev/video5', 'TagCam: T', ['usb-B-video-index1'])]
restore = with_fake_devices(NODES)
out = io.StringIO()
with contextlib.redirect_stdout(out):
    amb = st.resolve_device('name:Cam')          # 两块板卡都含 "Cam"
    one = st.resolve_device('name:TagCam')       # 只有一块板子含
    meta = st.resolve_device('name:Laptop')      # 一块板子两个节点：要挑取流那个
restore()
txt = out.getvalue()
check('片段撞上两种板卡名 -> 拒绝，不瞎挑一个（笔记内置摄像头就是这么被当成 tag 相机的）',
      amb == '', f'→ {amb!r}')
check('拒绝时把候选板卡名和能直接抄的写法都打出来',
      'Laptop Cam: L' in txt and 'TagCam: T' in txt and "'name:TagCam: T'" in txt)
check('唯一命中才解析，取的是取流节点', one == '/dev/video4', f'→ {one!r}')
check('一块板子两个节点（取流+metadata）挑 -video-index0 那个',
      meta == '/dev/video0', f'→ {meta!r}')

out = io.StringIO()
with contextlib.redirect_stdout(out):
    st.resolve_device('name:TagCam', quiet=True)
check('quiet 一个诊断都不打（重试时别每 5 秒刷一屏）', out.getvalue() == '',
      f'→ {out.getvalue()!r}')


# ==================== 起不来 / 掉了会自己重试 ====================
# 用户要的："没有相机要多打印几次，和串口没连上一样，或者添加显著标识"。
# 串口那条是每 5 秒重连一次，相机这边照这个办：**每一轮都打一遍显著提醒**，
# 起来了就接着用 —— 插晚了、换 USB 口、节点号变了都不用重启脚本。
class FlakyOpen:
    """头 fails 次 open 给个读不到帧的（模拟"还没插上 / 节点不对"），之后给好的。"""
    def __init__(self, fails):
        self.fails, self.calls, self.good = fails, 0, FakeCap()
    def __call__(self, *a, **k):
        self.calls += 1
        return DeadCap() if self.calls <= self.fails else self.good


class DyingCap(FakeCap):
    """跑一会儿就再也读不到帧了（模拟拔掉 / 被别的程序抢走）。"""
    def __init__(self, good_frames=20):
        super().__init__()
        self.left, self.reads = good_frames, 0
    def read(self):
        self.reads += 1
        return (False, None) if self.reads > self.left else super().read()


class SwitchOpen:
    """第一次给会中途死掉的，之后给好的。"""
    def __init__(self):
        self.calls, self.good = 0, FakeCap()
    def __call__(self, *a, **k):
        self.calls += 1
        return DyingCap() if self.calls == 1 else self.good


print('--- 相机起不来会自己重试 ---')
old_interval, old_failmax = st.CAM_RETRY_INTERVAL, st.CAM_READ_FAIL_MAX
st.CAM_RETRY_INTERVAL = 0.2          # 测试里等不起 5 秒一轮
st.CAM_READ_FAIL_MAX = 5             # 也别等 100 帧
opener = FlakyOpen(fails=2)
st.open_camera = opener
# 设备说明给 /dev/null：resolve_device 只认"这个路径存不存在"，真开相机那步被顶掉了
wk = st.CameraWorker('tag', '', 640, 480, 30, 'MJPG', spec='/dev/null')
out = io.StringIO()
with contextlib.redirect_stdout(out):
    wk.start()
    time.sleep(0.9)                  # 够重试两轮了
    wk.stop()
txt = out.getvalue()
check('起不来会一直重试（不是报一次就闭嘴）', opener.calls >= 3,
      f'→ 试了 {opener.calls} 次')
check('每一轮都打一遍显著提醒', txt.count('这一路没开起来') >= 2 and '!!!!!!' in txt,
      f'→ 提醒了 {txt.count("这一路没开起来")} 次，试了 {opener.calls} 次')
check('提醒里有是哪一路、给的是什么设备说明',
      'tag 这一路没开起来' in txt and '/dev/null' in txt)
check('后面起来了 -> error 清掉，这一路能用了', wk.error == '' and wk.up(),
      f'→ error={wk.error!r}')
frame, _, _ = wk.snapshot()
check('重试上来之后真的在出帧', frame is not None)

print('--- 跑到一半掉了也会自己接上 ---')
st.open_camera = SwitchOpen()
wk = st.CameraWorker('material', '', 640, 480, 30, 'MJPG', spec='/dev/null')
out = io.StringIO()
with contextlib.redirect_stdout(out):
    wk.start()
    time.sleep(1.0)
    wk.stop()
txt = out.getvalue()
check('读不到帧不再空转，会报出来', '读不到' in txt, f'→ {txt.strip()[-60:]!r}')
check('掉了之后自己重开，又出帧了', wk.error == '' and wk.snapshot()[0] is not None,
      f'→ error={wk.error!r}')
st.CAM_RETRY_INTERVAL, st.CAM_READ_FAIL_MAX = old_interval, old_failmax

print('\n' + ('全部通过' if ok else '有失败项'))
sys.stdout.flush()
# 直接退：daemon 线程里还压着 cv2 的取流，正常退会在解释器清理时 abort
# （"terminate called without an active exception"），退出码就不是 0/1 了。
# check_search.py 里也是这么收尾的。
os._exit(0 if ok else 1)

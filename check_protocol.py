#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线验证 schooltest2 的 0x20/0x21 握手解析和发送门控（不用摄像头、不用串口）。

跑：python3 check_protocol.py
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import schooltest2 as st

ok = True
def check(name, cond, extra=''):
    global ok
    ok = ok and cond
    print(f'{"PASS" if cond else "FAIL"}  {name}{"  " + extra if extra else ""}')


# ---- 1. 帧构造 ----
check('0x21 cmd=0x02 帧 = AA 55 21 01 02',
      st.build_cmd_frame(st.CMD_GRAB) == bytes([0xAA, 0x55, 0x21, 0x01, 0x02]),
      st.build_cmd_frame(st.CMD_GRAB).hex(' ').upper())
check('0x21 cmd=0x01 帧', st.build_cmd_frame(st.CMD_QR_DONE).hex(' ').upper() == 'AA 55 21 01 01')
check('0x21 cmd=0x00 帧(心跳)', st.build_cmd_frame(st.CMD_IDLE).hex(' ').upper() == 'AA 55 21 01 00')
try:
    st.build_cmd_frame(0x03)
    check('表外指令字被拒', False)
except ValueError as e:
    check('表外指令字被拒', True, str(e))

# tag 帧原样
check('0x03 tag 帧', st.build_tag_frame('123456789012').hex(' ').upper()
      == 'AA 55 03 0C 31 32 33 34 35 36 37 38 39 30 31 32')
try:
    st.build_tag_frame('123')
    check('非 12 位 tag 被拒', False)
except ValueError:
    check('非 12 位 tag 被拒', True)

# ---- 2. 白名单 ----
check('白名单登记了 0x20/0x21', st.TYPE_LEN[0x20] == 1 and st.TYPE_LEN[0x21] == 1)
check('白名单长度 = 协议 0x00~0x11',
      all(st.TYPE_LEN[t] == n for t, n in
          {0x00: 18, 0x01: 32, 0x02: 32, 0x03: 12, 0x04: 18, 0x05: 12,
           0x10: 12, 0x11: 28}.items()))

# ---- 3. 解析：状态帧 ----
link = st.McuLink('/dev/ttyNOPE', 115200)

link._parse(bytearray(bytes([0xAA, 0x55, 0x20, 0x01, 0x00])))
check('state=0x00 等待二维码', link.state == 0x00 and link.fresh_state()[1],
      f'{link.state}')

link._parse(bytearray(bytes([0xAA, 0x55, 0x20, 0x01, 0x01])))
check('state=0x01 等待颜色信息', link.state == 0x01)

link._parse(bytearray(bytes([0xAA, 0x55, 0x20, 0x01, 0x10])))
check('state=0x10 运动中', link.state == 0x10)

# ---- 4. 解析：脏数据、半个帧、拼接帧、假帧头 ----
link.state = None
link._parse(bytearray(bytes([0x11, 0x22, 0x33])))                       # 纯噪声
check('纯噪声不产生状态', link.state is None)

# 真实链路里 buf 是同一个对象跨多次 read 累积的（见 _loop），所以这里也复用它
buf = bytearray(bytes([0xAA, 0x55, 0x20]))                              # 半帧
link._parse(buf)
check('半帧不发状态、buf 留着', link.state is None and len(buf) == 3, f'buf={bytes(buf).hex(" ")}')
buf += bytes([0x01, 0x10])                                              # 补上剩下两字节
link._parse(buf)
check('补齐后解析出 0x10', link.state == 0x10 and len(buf) == 0)

link.state = None
link._parse(bytearray(bytes([0xAA, 0x55, 0x20, 0x01, 0x00,               # 两帧粘一起
                             0xAA, 0x55, 0x20, 0x01, 0x01])))
check('粘包两帧都解出来', link.state == 0x01 and link.rx_count.get(0x20, 0) >= 1)

link.state = None
link._parse(bytearray(bytes([0xAA, 0xAA, 0x55, 0x20, 0x01, 0x00])))     # 假帧头 AA
check('假帧头 AA 能重新对齐', link.state == 0x00)

fresh = st.McuLink('/dev/ttyNOPE', 115200)                              # 新链路，计数从零开始
fresh._parse(bytearray(bytes([0xAA, 0x55, 0x03, 0xFF, 0x00] * 3)))      # 类型对长度错
check('(类型,长度) 不匹配就跳过', fresh.state is None and fresh.rx_count == {}, f'{fresh.rx_count}')
# 混一条真的进去，错帧不该把后面的好帧带偏
fresh._parse(bytearray(bytes([0xAA, 0x55, 0x03, 0xFF, 0x00,
                              0xAA, 0x55, 0x20, 0x01, 0x01])))
check('错帧后面紧跟真帧仍能解出', fresh.state == 0x01)

# ---- 5. 握手门控（把 decide 的判据照抄一遍验证语义）----
def gate(state, has_qr, has_material, hb_hz=5.0, still_for=None, still_time=0.5):
    """复刻 decide 的语义：返回这条状态允许发什么。
    still_for = 圆已经静止了多少秒（None = 没看到圆 / 刚看到）。"""
    if state == st.STATE_WAIT_QR:
        return st.CMD_QR_DONE if has_qr else None
    if state == st.STATE_WAIT_COLOR:
        if not has_material:
            return None
        return st.CMD_GRAB if (still_for is not None and still_for >= still_time) else None
    if state == st.STATE_MOVING:
        return st.CMD_IDLE if hb_hz > 0 else None
    return None

check('等待二维码 + 有 tag  -> 发 0x01', gate(0x00, True, False) == st.CMD_QR_DONE)
check('等待二维码 + 没 tag  -> 不发', gate(0x00, False, False) is None)
check('等待颜色 + 有物料 + 圆停稳 -> 发 0x02',
      gate(0x01, False, True, still_for=0.5) == st.CMD_GRAB)
check('等待颜色 + 有物料 + 圆还在动 -> 不发', gate(0x01, False, True, still_for=0.2) is None)
check('等待颜色 + 有物料 + 刚看到(还没计时) -> 不发', gate(0x01, False, True, still_for=None) is None)
check('等待颜色 + 没物料  -> 不发', gate(0x01, False, False) is None)
check('静止门限可调：--still-time 1.5 时 0.5s 不发',
      gate(0x01, False, True, still_for=0.5, still_time=1.5) is None)
check('运动中 + 心跳开      -> 只发 0x00', gate(0x10, True, True) == st.CMD_IDLE)
check('运动中 + 心跳关      -> 不发', gate(0x10, True, True, hb_hz=0) is None)
check('状态 0x10 时绝不发 02/01',
      all(gate(0x10, True, True, hb_hz=h) in (st.CMD_IDLE, None) for h in (0, 5.0)))
check('状态 0x00 时绝不发 02',
      all(gate(0x00, q, m, still_for=9) in (st.CMD_QR_DONE, None)
          for q in (True, False) for m in (True, False)))

# ---- 6. 停稳判定（复刻主循环里那段跟踪逻辑）----
def track(frames, tol=6, r_frac=0.10):
    """frames = [(center, radius), ...]，返回最后一次"看到它在动"的下标。"""
    ref, since = None, None
    for i, (c, r) in enumerate(frames):
        if ref is None:
            since = i
        else:
            move = max(abs(c[0] - ref[0][0]), abs(c[1] - ref[0][1]))
            dr = abs(r - ref[1])
            if move > tol or dr > max(2, ref[1] * r_frac):
                since = i
        ref = (c, r)
    return since

# 帧间抖 2 像素：算静止，从第 0 帧开始算
jitter = [((100 + (i % 3), 100 - (i % 2)), 50) for i in range(10)]
check('帧间抖 2px 算静止（从头计时）', track(jitter) == 0, f'since={track(jitter)}')
# 第 6 帧挪了 40 像素：那时才算"动了"，要重新计时
jump = [((100, 100), 50)] * 6 + [((140, 100), 50)] * 4
check('第 6 帧挪 40px -> 从第 6 帧重新计时', track(jump) == 6, f'since={track(jump)}')
# 半径一直在长大（在朝相机靠近）：也算动
grow = [((100, 100), 50 + i * 20) for i in range(6)]
check('半径每帧涨 20px -> 一直在动', track(grow) == 5, f'since={track(grow)}')
# 容差是按比例给的（10% of r），所以固定 8px 的涨幅会随着半径变大"落进容差内"：
# 74->82 是 10.8% 算动，82->90 只有 9.8% 就不算了。这是有意的（半径的抖动本来就随半径放大），
# 但得知道有这么个边界，别指望它去抓"大圆慢慢靠近"。
band = [((100, 100), 50 + i * 8) for i in range(6)]        # 50,58,...,90
check('固定 8px 涨幅：r>80 后落进 10% 容差 -> 停在 since=4',
      track(band) == 4, f'since={track(band)}')
noise_r = [((100, 100), 50 + (i % 2) * 4) for i in range(8)]
check('半径抖 8%（容差内）-> 算静止', track(noise_r) == 0, f'since={track(noise_r)}')
# 圆心不动、半径猛涨：这是"物料朝相机过来"，不能当停稳
approach = [((100, 100), 50)] + [((100, 100), 90)]
check('圆心不动但半径突然变大 -> 从那一帧重新计时',
      track(approach) == 1, f'since={track(approach)}')

# ---- 7. 状态超时 ----
import time
link.state, link.state_time = 0x01, time.time() - st.STATE_TIMEOUT - 0.01
check(f'超过 {st.STATE_TIMEOUT}s 没上报 -> 状态不新鲜', link.fresh_state()[1] is False)

print('\n' + ('全部通过' if ok else '有失败项'))
sys.exit(0 if ok else 1)

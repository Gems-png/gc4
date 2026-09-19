#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线验证 schooltest2 的 0x20/0x21 握手解析和发送门控（不用摄像头、不用串口）。

跑：python3 check_protocol.py
"""
import sys
import os
import math
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
check('state=0x01 等待抓取指令', link.state == 0x01)

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
def gate(state, has_qr, has_material, hb_hz=5.0, still_for=None, still_time=0.5,
         err=None, aim_tol=25, aim_tol_y=None, align_left=99, align_enable=True,
         wait_nudge=False, stuck=False, no_circle_for=None, search_left=99,
         search_interval=2.0, skip_color=False):
    """复刻 decide 的语义：返回这条状态允许发什么。
    still_for  = 圆已经静止了多少秒（None = 没看到圆 / 刚看到）
    err        = (ex, ey) 圆心离对准点的像素差（None = 没圆 / 还没出帧）
    aim_tol    = 左右容差（--aim-tol-x）；aim_tol_y 不填就跟它一样（= 旧的单容差行为）
    align_left = --align-max 还允许发几条微调（<=0 = 已经发满了）
    align_enable = 顶部 ALIGN_ENABLE；关着时一条微调都不发
    wait_nudge = True = 上一发微调还在走（不许挪、也还不许抓，车在动）
    stuck      = 上一发卡住了（超时没等到 0x01）-> 挪不了了，照抓
    no_circle_for = 已经连续多少秒一个圆都没看到（None = 刚看不到）
    search_left   = --search-max 还允许往前找几次（<=0 = 找满了）
    skip_color    = 挑中的那个圆的颜色抓过了（--color-policy once）-> 停稳/对准照走，
                    只是压住不发抓取（复刻 decide 里的 want_grab）"""
    if aim_tol_y is None:
        aim_tol_y = aim_tol

    def grab():
        """该发抓取了 —— 这个颜色抓过就压住不发（见 schooltest2.decide 的 want_grab）。"""
        return None if skip_color else st.CMD_GRAB

    if state == st.STATE_WAIT_QR:
        return st.CMD_QR_DONE if has_qr else None
    if state == st.STATE_WAIT_GRAB:
        if not has_material:
            # 没圆：够 search_interval 就往前的找一发。往前找走的也是 0x30~0x33 那条通道，
            # 所以同样受"一次一按"管（车在走 / 上一发卡住都不能发）。
            # 找满了就**停着等，不盲抓** —— 没圆根本不知道物料在哪。
            if not align_enable or search_left <= 0 or stuck or wait_nudge:
                return None
            if no_circle_for is None or no_circle_for < search_interval:
                return None
            return st.CMD_ADJUST_FWD
        if err is None:
            return None
        if still_for is None or still_for < still_time:
            return None
        # 挑轴用的是 align_cmd_for（谁超了自己的容差就修谁），不是"误差大的轴"
        want = st.align_cmd_for(err[0], err[1], aim_tol, aim_tol_y)
        if want is not None and align_enable:
            # 没对准：能挪就挪一发；挪不动了（发满 / 卡住）就**照抓**。
            # 顺序照 decide 抄：卡住 -> 照抓；还在走 -> 等它落地（别抢着抓）；发满 -> 照抓
            if stuck:
                return grab()
            if wait_nudge:
                return None
            if align_left <= 0:
                return grab()
            return want
        return grab()                       # 关着的时候对准这关整个跳过，停稳了就抓
    if state == st.STATE_MOVING:
        return st.CMD_IDLE if hb_hz > 0 else None
    return None

check('等待二维码 + 有 tag  -> 发 0x01', gate(0x00, True, False) == st.CMD_QR_DONE)
check('等待二维码 + 没 tag  -> 不发', gate(0x00, False, False) is None)
check('等待抓取 + 有圆 + 停稳 + 已对准 -> 发 0x02',
      gate(0x01, False, True, still_for=0.5, err=(0, 0)) == st.CMD_GRAB)
check('等待抓取 + 有圆 + 还在动 -> 不发', gate(0x01, False, True, still_for=0.2, err=(0, 0)) is None)
check('等待抓取 + 有圆 + 刚看到(还没计时) -> 不发',
      gate(0x01, False, True, still_for=None, err=(0, 0)) is None)
check('等待抓取 + 没圆  -> 不发', gate(0x01, False, False) is None)
check('等待抓取 + 有圆但还没出帧(算不出对准点) -> 不发',
      gate(0x01, False, True, still_for=9, err=None) is None)
check('静止门限可调：--still-time 1.5 时 0.5s 不发',
      gate(0x01, False, True, still_for=0.5, still_time=1.5, err=(0, 0)) is None)
check('--still-time 0 = 一看到圆就动（不等）',
      gate(0x01, False, True, still_for=0.0, still_time=0, err=(0, 0)) == st.CMD_GRAB)
check('运动中 + 心跳开      -> 只发 0x00', gate(0x10, True, True) == st.CMD_IDLE)
check('运动中 + 心跳关      -> 不发', gate(0x10, True, True, hb_hz=0) is None)
check('状态 0x10 时绝不发 02/01',
      all(gate(0x10, True, True, hb_hz=h) in (st.CMD_IDLE, None) for h in (0, 5.0)))
check('状态 0x00 时绝不发 02',
      all(gate(0x00, q, m, still_for=9) in (st.CMD_QR_DONE, None)
          for q in (True, False) for m in (True, False)))

# ---- 5b. 底盘微调（0x30~0x33）----
check('微调帧 0x30=前', st.build_cmd_frame(st.CMD_ADJUST_FWD).hex(' ').upper()
      == 'AA 55 21 01 30')
check('微调帧 0x31=后', st.build_cmd_frame(st.CMD_ADJUST_BACK).hex(' ').upper()
      == 'AA 55 21 01 31')
check('微调帧 0x32=左', st.build_cmd_frame(st.CMD_ADJUST_LEFT).hex(' ').upper()
      == 'AA 55 21 01 32')
check('微调帧 0x33=右', st.build_cmd_frame(st.CMD_ADJUST_RIGHT).hex(' ').upper()
      == 'AA 55 21 01 33')
check('微调帧和 0x21 一样是 5 字节（不是新类型）',
      all(len(st.build_cmd_frame(c)) == 5 for c in st.ADJUST_CMDS))
check('指令字白名单 = 00/01/02 + 微调 30~33', st.LEGAL_CMDS == (0x00, 0x01, 0x02, 0x30, 0x31, 0x32, 0x33))

# 方向：画面上方 = 车头前方。圆心偏右 -> 车往右(0x33)；偏下 -> 车往后退(0x31)
check('圆心偏右 -> 右(0x33)', st.align_cmd(30, 0) == st.CMD_ADJUST_RIGHT)
check('圆心偏左 -> 左(0x32)', st.align_cmd(-30, 0) == st.CMD_ADJUST_LEFT)
check('圆心偏下 -> 后(0x31)', st.align_cmd(0, 30) == st.CMD_ADJUST_BACK)
check('圆心偏上 -> 前(0x30)', st.align_cmd(0, -30) == st.CMD_ADJUST_FWD)
check('右下 -> 先修误差大的轴（左上都 30 时取横向）',
      st.align_cmd(40, -30) == st.CMD_ADJUST_RIGHT
      and st.align_cmd(30, -40) == st.CMD_ADJUST_FWD)
check('两轴一样大 -> 修横向', st.align_cmd(30, 30) == st.CMD_ADJUST_RIGHT)

check('没对准 + 停稳 -> 发微调', gate(0x01, False, True, still_for=9, err=(40, 0))
      == st.CMD_ADJUST_RIGHT)
check('没对准但还在动 -> 不发（先等停稳）',
      gate(0x01, False, True, still_for=0.1, err=(40, 0)) is None)
# 挪不动了就照抓：抓偏一点也比整轮卡在这儿强（发满 --align-max / 上一发卡住）
check('没对准 + 微调发满了 -> 不再挪车，但**照抓**',
      gate(0x01, False, True, still_for=9, err=(40, 0), align_left=0) == st.CMD_GRAB)
check('没对准 + 上一发卡住(超时没回 0x01) -> 也不干等，照抓',
      gate(0x01, False, True, still_for=9, err=(40, 0), stuck=True) == st.CMD_GRAB)
check('上一发还在走 -> 既不挪也不抓（车在动，抓了也是废的）',
      gate(0x01, False, True, still_for=9, err=(40, 0), wait_nudge=True) is None)
check('上一发还在走 + 步数已发满 -> 还是等它落地，不抢着抓',
      gate(0x01, False, True, still_for=9, err=(40, 0), wait_nudge=True,
           align_left=0) is None)
check('对准容差内（误差 = 容差）就算对准',
      gate(0x01, False, True, still_for=9, err=(25, 0), aim_tol=25) == st.CMD_GRAB)
check('刚出容差就发微调',
      gate(0x01, False, True, still_for=9, err=(26, 0), aim_tol=25) == st.CMD_ADJUST_RIGHT)
check('状态 0x00 / 0x10 绝不发微调',
      all(gate(s, True, True, still_for=9, err=(99, 99), hb_hz=5.0) not in st.ADJUST_CMDS
          for s in (0x00, 0x10)))

# 底盘微调总开关（ALIGN_ENABLE）：车上默认**开着** —— 微调已经是默认流程的一部分。
# 关掉 = 对准这关整个跳过：停稳了照抓，差多少像素只报不打紧。
check('车上默认：底盘微调是开着的', st.ALIGN_ENABLE is True, f'ALIGN_ENABLE={st.ALIGN_ENABLE}')
check('关着的时候：停稳了就抓，不对准也抓',
      gate(0x01, False, True, still_for=9, err=(40, 0), align_enable=False) == st.CMD_GRAB)
check('关着的时候：差得再远也照抓（只是报出来）',
      gate(0x01, False, True, still_for=9, err=(300, -200), align_enable=False) == st.CMD_GRAB)
check('关着的时候：一条微调都出不来',
      all(gate(0x01, False, True, still_for=9, err=e, align_enable=False) not in st.ADJUST_CMDS
          for e in ((99, 99), (-99, 0), (0, 99), (40, -40))))
check('关着的时候：还在动就不抓（停稳这一关照旧管用）',
      gate(0x01, False, True, still_for=0.1, err=(40, 0), align_enable=False) is None)
check('打开了就回到原样（电控好了改这一个常量）',
      gate(0x01, False, True, still_for=9, err=(40, 0), align_enable=True)
      == st.CMD_ADJUST_RIGHT)

# ---- 5b-2. 微调节拍：由下位机的 0x01 给，不是定时器 ----
# 协议（USART1_Nudge_Protocol.md §四/§五）：发出一发微调后，下位机把上报切成 0x10，
# 走完约 0.3s 自己切回 0x01 —— 那期间再发的会被**静默丢弃**。所以必须"一次一按"。
def mkpacer(interval=0.0, timeout=3.0, gap=1.0):
    """interval 默认给 0，好把"状态门"单独拎出来测（间隔兜底另有测试）。"""
    return st.NudgePacer(interval, timeout, gap)

p = mkpacer()
t = 1000.0
p.note(st.STATE_WAIT_GRAB, t)
p.sent(t, real=True)
check('发完一发：立刻再发 -> 挡住', p.ready(t)[0] is False)
check('0x10（还在走）时 -> 挡住',
      p.note(st.STATE_MOVING, t + 0.1) is None and p.ready(t + 0.1)[0] is False)
check('0x10 之后回到 0x01 = 这一发走完 -> 放行',
      p.note(st.STATE_WAIT_GRAB, t + 0.45) == 'done' and p.ready(t + 0.45)[0] is True)

# 连发三发（想挪 9cm）：每两发之间都必须先看到 0x10 再回到 0x01
p = mkpacer()
t = 1000.0
p.note(st.STATE_WAIT_GRAB, t)
seq = []
for _ in range(3):
    seq.append(p.ready(t)[0])                 # 门开着 -> 发
    p.sent(t, real=True)
    seq.append(p.ready(t + 0.1)[0])           # 刚发完 -> 关
    p.note(st.STATE_MOVING, t + 0.1)
    p.note(st.STATE_WAIT_GRAB, t + 0.4)
    t += 0.5
check('连发三发：每发之间都等到了 0x01（开/关/开/关/开/关）',
      seq == [True, False] * 3, f'seq={seq}')
check('三发都记上数了（--align-max 卡的就是它）', p.count == 3)

# 开环"发完 sleep 再发"是错的：状态一直 0x10 时，第二发根本不准出去
p = mkpacer()
t = 1000.0
p.note(st.STATE_WAIT_GRAB, t)
p.sent(t, real=True)
p.note(st.STATE_MOVING, t + 0.1)
check('发完 sleep 0.5s 就急着发第二发 -> 挡住（开环写法会少走 3cm）',
      p.ready(t + 0.6)[0] is False)

# 超时：发出去了但状态从没变过（被门拒了 / 帧丢了）-> 判为没生效，这一站不再挪
p = mkpacer(timeout=3.0)
t = 1000.0
p.note(st.STATE_WAIT_GRAB, t)
p.sent(t, real=True)
check('还没到超时：只是等着，不报警', p.ready(t + 2.0) == (False, None))
check('超时了 -> 报 stuck', p.note(st.STATE_WAIT_GRAB, t + 3.5) == 'stuck')
can, why = p.ready(t + 3.5)          # 注意别叫 ok —— 那是上面那个全局累加器
check('stuck 之后这一站不再发微调，而且有话说', can is False and bool(why))

# 新的一站：车自己跑过来停下（离开 0x01 超过 gap）-> 次数清零、重新开门
p = mkpacer(gap=1.0)
t = 1000.0
p.note(st.STATE_WAIT_GRAB, t)
p.sent(t, real=True)
p.note(st.STATE_MOVING, t + 0.1)
p.note(st.STATE_WAIT_GRAB, t + 0.5)        # 我们挪的那一发走完了
p.sent(t + 0.5, real=True)
check('同一站里发第二发：次数累加（不清零）', p.count == 2)
p.note(st.STATE_MOVING, t + 0.6)           # 车接着跑走了（不是我们挪的）
check('隔了 4 秒多才停回 0x01 = 新的一站 -> 次数清零',
      p.note(st.STATE_WAIT_GRAB, t + 5.0) == 'station' and p.count == 0)
p = mkpacer(gap=1.0)
t = 1000.0
p.note(st.STATE_WAIT_GRAB, t)
p.sent(t, real=True)
p.note(st.STATE_MOVING, t + 0.1)
check('我们自己挪出来的那次 0x01 不算新的一站（不然 --align-max 永远卡不住）',
      p.note(st.STATE_WAIT_GRAB, t + 0.5) == 'done' and p.count == 1)

# 离线干跑（--dry-run / 串口没连上）：没有回执可等 -> 退回按 interval 定时
p = mkpacer(interval=0.6)
t = 1000.0
p.note(st.STATE_WAIT_GRAB, t)
p.sent(t, real=False)
check('离线：还没到间隔 -> 挡住', p.ready(t + 0.3)[0] is False)
check('离线：过了间隔就放行（不等状态）', p.ready(t + 0.7)[0] is True)
check('离线：状态从没变过也不误报 stuck', p.note(st.STATE_WAIT_GRAB, t + 99.0) is None)

# 参数本身：协议说一发往返 0.4~0.5s，超时要宽裕；兜底间隔不能比往返还长
check('超时(3s)比协议往返(0.4~0.5s)宽裕得多', st.ALIGN_RETURN_TIMEOUT >= 2.0)
check('兜底间隔不比一发往返长太多（不然节拍被它拖慢）', st.ALIGN_INTERVAL <= 0.6)

# ---- 5b-3. 微调一步有多大 / 容差够不够 ----
# 底盘一步是下位机固定的 3cm；容差 T 只要 < 半步（S/2），车就必然在容差两侧来回动 ——
# 车上看着就是"小车来回动"。所以要能把 S 量出来并提示该填多少。
note = st.nudge_step_note((190, 5), (12, 3), st.CMD_ADJUST_RIGHT, 25, 60)
check('一步 178px 而左右容差只有 25 -> 直接说"必然来回"并给出该填多少',
      note is not None and '来回' in note and '89' in note and 'AIM_TOL_X' in note, note)
note = st.nudge_step_note((120, 5), (12, 3), st.CMD_ADJUST_RIGHT, 100, 60)
check('容差 ≥ 半步 -> 只说走完一步，不报警',
      note is not None and '来回' not in note and '108' in note, note)
check('量的是**发出去的那个方向**的轴（前后），拿的是前后容差',
      'y 方向' in st.nudge_step_note((3, -60), (2, -10), st.CMD_ADJUST_BACK, 25, 100))
check('同一组数（一步 50px）：前后容差 100 不报警，收紧到 20 就报警',
      '来回' not in st.nudge_step_note((3, -60), (2, -10), st.CMD_ADJUST_BACK, 25, 100)
      and '来回' in st.nudge_step_note((3, -60), (2, -10), st.CMD_ADJUST_BACK, 25, 20))
check('冲过头（误差反号）照样能量出步长',
      '120' in st.nudge_step_note((-40, 0), (80, 0), st.CMD_ADJUST_RIGHT, 100, 60))
check('这一步画面里没动 -> 提示查打滑/堵转',
      '打滑' in st.nudge_step_note((60, 0), (60, 0), st.CMD_ADJUST_RIGHT, 25, 60))
check('这帧没检出圆 -> 不瞎报（返回 None）',
      st.nudge_step_note((60, 0), None, st.CMD_ADJUST_RIGHT, 25, 60) is None
      and st.nudge_step_note(None, (60, 0), st.CMD_ADJUST_RIGHT, 25, 60) is None)
check('容差比帧间抖动大得多（不然抖动本身就能让误差在容差两侧跳）',
      min(st.AIM_TOL_X, st.AIM_TOL_Y) >= 3 * st.STILL_TOL,
      f'X={st.AIM_TOL_X} Y={st.AIM_TOL_Y} STILL_TOL={st.STILL_TOL}')

# ---- 5b-4. 两个轴各自的容差（左右可以放得比前后松）----
# 光按"误差大的轴"挑会出错：左右容差 60、前后 20 时，ex=50 在容差里、ey=30 超了，
# 照"误差大"会去白挪一步左右。
TOL_X, TOL_Y = 60, 20
check('只超前后 -> 修前后（哪怕左右的误差更大）',
      st.align_cmd_for(50, 30, TOL_X, TOL_Y) == st.CMD_ADJUST_BACK)
check('只超左右 -> 修左右',
      st.align_cmd_for(80, 5, TOL_X, TOL_Y) == st.CMD_ADJUST_RIGHT)
check('两个都超 -> 修误差大的那个',
      st.align_cmd_for(90, -45, TOL_X, TOL_Y) == st.CMD_ADJUST_RIGHT
      and st.align_cmd_for(-30, -70, TOL_X, TOL_Y) == st.CMD_ADJUST_FWD)
check('都在各自的容差里 -> 不挪（None）', st.align_cmd_for(59, -19, TOL_X, TOL_Y) is None)
check('正好等于容差 -> 算在容差里（跟以前一样是 > 才算超）',
      st.align_cmd_for(60, 20, TOL_X, TOL_Y) is None)
check('左右放宽真的少挪：以前要挪的 (50,0)，现在在容差里',
      st.align_cmd_for(50, 0, TOL_X, TOL_Y) is None
      and st.align_cmd_for(50, 0, 25, 25) == st.CMD_ADJUST_RIGHT)
check('默认就是左右比前后松', st.AIM_TOL_X > st.AIM_TOL_Y,
      f'X={st.AIM_TOL_X} Y={st.AIM_TOL_Y}')

# ---- 5b-5. 圆台：大小两个同心圆要合成一块，留大的 ----
# 物料是圆台，投影下来底面轮廓 + 顶面是两个圆，都会被检出。实测那个红圆台是
# r=62@(264,218) 和 r=51@(298,218)，**圆心差 34px** —— 按"圆心重合"判是合不上的。
C = st.Circle
CONE_BIG = C('red', (178, 160, 240), math.pi * 62 * 62, (264, 218), 62, 0.90)
CONE_SMALL = C('red', (178, 160, 240), math.pi * 51 * 51, (298, 218), 51, 0.90)
cone = st.merge_cones([CONE_BIG, CONE_SMALL])
check('圆台的两层合成一块物料', len(cone) == 1, f'→ {len(cone)} 块')
check('合成后留的是**大的**那个（r=62），底盘校准用它',
      cone and cone[0].radius == 62, f'→ r={cone[0].radius}' if cone else '')
check('并掉的层数记在 merged 里（打印看得出确实检出两层）',
      cone and cone[0].merged == 1, f'→ merged={cone[0].merged}' if cone else '')
# 倒着喂进去也得留大的：判据是"小的圆心落在大圆的盘里"，先收下谁就留下谁
check('先把小的给进去，也还是留大的（合并自己会先排序）',
      st.merge_cones([CONE_SMALL, CONE_BIG])[0].radius == 62)
# 并排的两块物料圆心至少隔 2 个半径，不能合
PAIR = [C('red', None, math.pi * 52 * 52, (100, 130), 52, 0.9),
        C('blue', None, math.pi * 52 * 52, (540, 130), 52, 0.9)]
check('并排的两块物料不会被并成一块', len(st.merge_cones(PAIR)) == 2,
      f'→ {len(st.merge_cones(PAIR))} 块')
check('只检出一个圆时 merged=0（没圆台或只认出一层）',
      st.merge_cones([CONE_BIG])[0].merged == 0)
check('合并后仍然是从大到小排（materials[0] 就是最大的）',
      [c.radius for c in st.merge_cones([CONE_SMALL, CONE_BIG])] == [62])

# ---- 5b-6. 找不到圆就往前找（最多 --search-max 次）----
check('没圆 + 还没到 2s -> 先不发',
      gate(0x01, False, False, no_circle_for=1.0) is None)
check('没圆 + 够 2s -> 往前找一发（0x30）',
      gate(0x01, False, False, no_circle_for=2.0) == st.CMD_ADJUST_FWD)
check('差一点不算（1.9s）-> 还不是时候',
      gate(0x01, False, False, no_circle_for=1.9) is None)
check('第 3 发还能发（search_left=1）',
      gate(0x01, False, False, no_circle_for=2.0, search_left=1) == st.CMD_ADJUST_FWD)
check('找满 3 发就不再往前，也不盲抓（就停着等）',
      gate(0x01, False, False, no_circle_for=99.0, search_left=0) is None)
check('上一发还在走 -> 不能往前拱（一次一按）',
      gate(0x01, False, False, no_circle_for=2.0, wait_nudge=True) is None)
check('上一发卡住了 -> 也不再往前拱',
      gate(0x01, False, False, no_circle_for=2.0, stuck=True) is None)
check('底盘微调关着 -> 一条微调都不发，包括"往前找"',
      gate(0x01, False, False, no_circle_for=99.0, align_enable=False) is None)
check('有圆的时候走的是对准那套，绝不因为"没圆"往前找',
      gate(0x01, False, True, no_circle_for=99.0, still_for=0.5, err=(0, 0))
      == st.CMD_GRAB)

# ---- 5b-7. 抓过的颜色不再抓（物料底下压着一个同色的圆）----
ALIGNED = dict(still_for=9, err=(0, 0))
check('没抓过的颜色 -> 停稳对准了照抓',
      gate(0x01, False, True, **ALIGNED, skip_color=False) == st.CMD_GRAB)
check('抓过的颜色 -> 压住不发抓取',
      gate(0x01, False, True, **ALIGNED, skip_color=True) is None)
# 这条是这次改动的要害：压住抓取**不等于**"没看到圆"。要是判成了没圆，
# 第 6 条那套就会接管 —— 车会对着一堆"抓过色的圆"往前拱三下，全白跑。
check('抓过的颜色仍然算"看到圆"：绝不触发往前找',
      gate(0x01, False, True, no_circle_for=99.0, **ALIGNED, skip_color=True) is None)
check('抓过的颜色照样对准（车会对准它，只是不抓）',
      gate(0x01, False, True, still_for=9, err=(50, 0), skip_color=True)
      == st.CMD_ADJUST_RIGHT)
check('抓过的颜色 + 微调发满 -> 也不"照抓"',
      gate(0x01, False, True, still_for=9, err=(50, 0), align_left=0, skip_color=True) is None)
check('抓过的颜色 + 上一发卡住 -> 也不"照抓"',
      gate(0x01, False, True, still_for=9, err=(50, 0), stuck=True, skip_color=True) is None)
check('抓过的颜色 + 圆还在动 -> 当然也不发',
      gate(0x01, False, True, still_for=0.1, err=(0, 0), skip_color=True) is None)

# pick_target：挑"最大的、颜色没抓过的"那个
C = st.Circle
T_BIG_RED = C('red', None, math.pi * 62 * 62, (264, 218), 62, 0.90)
T_MID_BLUE = C('blue', None, math.pi * 40 * 40, (400, 300), 40, 0.90)
T_SMALL_RED = C('red', None, math.pi * 20 * 20, (100, 100), 20, 0.90)
check('谁都没抓过 -> 抓最大的', st.pick_target([T_BIG_RED, T_MID_BLUE]) is T_BIG_RED)
check('最大的那个颜色抓过了 -> 改抓下一个没抓过的',
      st.pick_target([T_BIG_RED, T_MID_BLUE], {'red'}) is T_MID_BLUE)
check('全抓过了 -> 退回最大的那个（算"看到圆"，由 decide 压住不抓）',
      st.pick_target([T_BIG_RED, T_MID_BLUE], {'red', 'blue'}) is T_BIG_RED)
check('没圆 -> None（走"没看到圆"那套）', st.pick_target([], {'red'}) is None)
check('颜色认不出（None）不算抓过，照抓',
      st.pick_target([C(None, None, 1.0, (1, 1), 1, 0.0)], {'red'}).color is None)

# ---- 5c. 对准点 ----
check('--aim-x/-y 给 -1 -> 画面正中', st.resolve_aim(-1, -1, 480, 640) == (320, 240))
check('对准点给了像素值就用给的', st.resolve_aim(100, 50, 480, 640) == (100, 50))
# 转正之后的帧尺寸才是识别用的尺寸，对准点得按它算
check('转了 90° 的画面按转正后的尺寸取中心',
      st.resolve_aim(-1, -1, 640, 480) == (240, 320))

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

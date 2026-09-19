#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""遥控模式（remote.py）的离线测试：不碰键盘、不碰串口、不碰相机。

分两段：

A 段 **纯逻辑**：同一个进程里按时间线推 RemoteCore / StepPacer / KeyHold / KeyReader。
   不 sleep、不 IO，毫秒级跑完。手感那一套（新鲜度窗口、额度、节拍）全在这里钉死 ——
   它是整个遥控里最容易写错、又最难在车上复现的部分。

B 段 **端到端**：把假串口、假键盘塞进**真的 remote.main()** 让它自己跑，只看串口上
   到底写出了什么。一个场景一个进程（main 里的 while 循环靠 --run-secs 停，每个场景
   各跑各的，别互相抢 CPU 把时间量歪 —— 和 check_search.py 一个路数）。

为什么非要 B 段：A 段量的是"我认为的"时间线，真正决定发不发的是主循环里
"读键 → 喂状态 → 发 → 回报"这一圈的先后顺序。写反了 A 段全绿，车上就是不发或者连发。

跑：python3 check_remote.py
"""
import contextlib
import io
import os
import pty
import signal
import subprocess
import sys
import termios
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault('OPENCV_LOG_LEVEL', 'ERROR')
import schooltest2 as st
import remote

T0 = 1000.0        # A 段的钟从这个值起（不是 0），理由见 sim()

ok = True
def check(name, cond, extra=''):
    global ok
    ok = ok and cond
    print(f'{"PASS" if cond else "FAIL"}  {name}{"  " + extra if extra else ""}')


def hold(key, t0, t1, period=1 / 30.0):
    """终端自动重复：t0 按下，之后每 period 吐一个字符，到 t1 松手。"""
    out, t = [], t0
    while t <= t1:
        out.append((t, key))
        t += period
    return out


# ==================== A 段：纯逻辑 ====================
def sim(keys, secs, state=st.STATE_WAIT_GRAB, fresh=True, mode='time',
        interval=remote.NUDGE_INTERVAL):
    """按时间线跑 RemoteCore，返回 (core, [(时刻, 指令字)], 事件行)。

    钟从 T0 起：NudgePacer 里 `at=0.0` 是"刚发过一条"的初值，钟在 0 附近会把第一条
    平白压住一个 interval。真机上是 time.time()，不会有这个问题。
    """
    core = remote.RemoteCore(pacer=remote.StepPacer(interval=interval, mode=mode))
    sent, logs = [], []
    dt, i, t = 1 / 50.0, 0, T0
    while t <= T0 + secs:
        while i < len(keys) and T0 + keys[i][0] <= t:
            core.on_key(keys[i][1], t)
            i += 1
        s = state(t - T0) if callable(state) else state
        f = fresh(t - T0) if callable(fresh) else fresh
        frame, _what, msgs = core.tick(t, s, f)
        logs += msgs
        if frame is not None:
            sent.append((t - T0, frame[4]))
            core.sent(t, True)          # 当成真发出去了
        t += dt
    return core, sent, logs


def gaps_of(sent):
    return [round(b[0] - a[0], 3) for a, b in zip(sent, sent[1:])]


def a_keyhold():
    print('--- A: KeyHold（按住就走、松手停）---')
    h = remote.KeyHold()

    # 轻点一下：只欠一步，扣掉之后不会自己再走
    h.press(st.CMD_ADJUST_FWD, T0)
    d, _ = h.held(T0)
    check('轻点：认得出方向', d == st.CMD_ADJUST_FWD)
    h.consume(d)
    d2, _ = h.held(T0 + 0.1)
    check('轻点：发完那一步就停住（不会一直走）', d2 is None)
    hb = remote.KeyHold()
    hb.press(st.CMD_ADJUST_FWD, T0 + 0.2)      # 换一个干净的，免得上一次按下量出的 gap 掺进来
    d3, _ = hb.held(T0 + 0.9)
    check('只按了一下（没量到重复周期）：窗口是最宽的 KEY_FRESH_MAX(0.75s)',
          d3 == st.CMD_ADJUST_FWD, f'→ age=0.7s，窗口 {hb.fresh(st.CMD_ADJUST_FWD):.2f}s')

    # 按住：30Hz 自动重复 → 窗口收紧到 KEY_FRESH_MIN，松手 0.15s 内停
    h2 = remote.KeyHold()
    for t, k in hold(remote.KEY_UP, 0.0, 1.0):
        h2.press(remote.KEYMAP[k], T0 + t)
    last = T0 + 1.0
    check('按住：量到重复周期后窗口收紧到 0.15s',
          abs(h2.fresh(st.CMD_ADJUST_FWD) - remote.KEY_FRESH_MIN) < 1e-6,
          f'→ gap={h2.gap[st.CMD_ADJUST_FWD]:.3f}s fresh={h2.fresh(st.CMD_ADJUST_FWD):.3f}s')
    h2.consume(st.CMD_ADJUST_FWD)              # 这一步发出去了
    check('按住中：扣掉额度后还没有新事件 → 不补发', h2.held(last + 0.05)[0] is None)
    h2.press(st.CMD_ADJUST_FWD, last + 0.033)  # 终端自动重复的下一个事件
    d4, _ = h2.held(last + 0.05)
    check('按住中：下一个自动重复事件给回额度', d4 == st.CMD_ADJUST_FWD)
    h2.consume(st.CMD_ADJUST_FWD)
    d5, _ = h2.held(last + 0.25)
    check('松手 0.15s 后就不认这个方向了（松手停）', d5 is None)

    # 同时按着两个：最后按下的赢
    h3 = remote.KeyHold()
    h3.press(st.CMD_ADJUST_LEFT, T0)
    h3.press(st.CMD_ADJUST_FWD, T0 + 0.05)
    d6, _ = h3.held(T0 + 0.06)
    check('同时按着两个：最后按下的那个赢', d6 == st.CMD_ADJUST_FWD)

    h4 = remote.KeyHold()
    h4.press(st.CMD_ADJUST_FWD, T0)
    h4.clear()
    d7, _ = h4.held(T0 + 0.01)
    check('急停 clear() 之后立刻不走（手上的键也不算）', d7 is None)


def a_keyreader():
    print('--- A: KeyReader（转义序列怎么切）---')
    r = remote.KeyReader()
    check('一整个方向键', r.feed(b'\x1b[A') == [remote.KEY_UP])
    r2 = remote.KeyReader()
    check('分两次读到的半截序列：先攒着',
          r2.feed(b'\x1b') == [] and r2.feed(b'[A') == [remote.KEY_UP])
    r3 = remote.KeyReader()
    check('一次读里好几组（按住时就是会长这样）',
          r3.feed(remote.KEY_UP + remote.KEY_DOWN + remote.KEY_LEFT)
          == [remote.KEY_UP, remote.KEY_DOWN, remote.KEY_LEFT])
    r4 = remote.KeyReader()
    check('认不出的转义序列（PageUp）丢掉，不卡在缓冲里',
          r4.feed(b'\x1b[5~') == [] and r4.feed(b'\x1b[A') == [remote.KEY_UP])
    r5 = remote.KeyReader()
    check('单键：空格/q 认得，别的字母静默忽略',
          r5.feed(b' ') == [remote.KEY_STOP] and r5.feed(b'q') == [remote.KEY_QUIT]
          and r5.feed(b'z') == [])
    r6 = remote.KeyReader()
    r6.feed(b'\x1b')
    r6.flush(r6.esc_at + remote.ESC_GRACE + 0.01)   # feed 里记的是 time.time()，别拿 T0 比
    check('超时的半截 ESC 会被丢掉（不然它会永久挡住后面的方向键）',
          r6.buf == b'' and r6.feed(b'\x1b[A') == [remote.KEY_UP])


def a_pacer():
    print('--- A: StepPacer（节拍）---')
    check('指令帧就是 AA 55 21 01 30',
          st.build_cmd_frame(st.CMD_ADJUST_FWD) == b'\xaa\x55\x21\x01\x30')

    # mode=auto + 固件给回执：0x01 → 0x10 → 0x01
    core = remote.RemoteCore(pacer=remote.StepPacer(mode='auto'))
    t = T0
    core.on_key(remote.KEY_UP, t)
    f1, _w, _m = core.tick(t, st.STATE_WAIT_GRAB, True)
    check('第一条按下就发（不用先等一个间隔）', f1 is not None)
    core.sent(t, True)
    f2, _, _ = core.tick(t + 0.1, st.STATE_MOVING, True)
    f3, _, _ = core.tick(t + 0.3, st.STATE_MOVING, True)
    check('0x10(运动中) 期间不发 —— 那会儿发出去会被静默丢弃',
          f2 is None and f3 is None)
    f4, _, _ = core.tick(t + 0.5, st.STATE_WAIT_GRAB, True)     # 回执：走完了
    check('回到 0x01 = 上一步走完，但没有新的按键事件 → 不补发',
          f4 is None and core.pacer.confirmed == 1)
    core.on_key(remote.KEY_UP, t + 0.55)                        # 自动重复的下一个事件
    f5, _, _ = core.tick(t + 0.56, st.STATE_WAIT_GRAB, True)
    check('有回执时按回执照走：确认过才发下一条', f5 is not None)
    check('判定成了"按回执"', core.pacer.state_paced is True)

    # mode=auto + 固件不给回执（电控把门放开、0x01 一直发）→ 自适应成定时
    p = remote.StepPacer(mode='auto')
    p.sent(T0, real=True)
    msgs = p.tick(st.STATE_WAIT_GRAB, T0 + remote.NUDGE_RETURN_TIMEOUT + 0.05)
    check('0.8s 没见到 0x10 → 判成"这版固件不给回执"',
          p.state_paced is False and any('没报 0x10' in m for m in msgs), f'→ {msgs}')
    can, _ = p.ready(T0 + remote.NUDGE_RETURN_TIMEOUT + 0.06)
    check('自适应之后立刻能继续发（不等满一个 timeout 才放行）', can)
    p.sent(T0 + 1.0, real=True)
    can2, why2 = p.ready(T0 + 1.1)
    check('定时节拍：没到 interval 就不发', not can2, f'→ {why2}')

    # 真 stuck：已知给回执却一直没动 → 冷却，但**不永久停手**
    p2 = remote.StepPacer(mode='state')
    p2.sent(T0, real=True)
    t_stuck = T0 + remote.NUDGE_RETURN_TIMEOUT + 0.05
    msgs2 = p2.tick(st.STATE_WAIT_GRAB, t_stuck)
    check('真没生效时打一行原因', any('没等到下位机动一下' in m for m in msgs2), f'→ {msgs2}')
    can3, why3 = p2.ready(T0 + 0.5)
    check('冷却期间不发，并且说清楚还有多久', not can3 and '还能再试' in (why3 or ''),
          f'→ {why3}')
    can4, _ = p2.ready(t_stuck + remote.NUDGE_STUCK_RETRY + 0.2)
    check('冷却过了还能再发（遥控不永久停手，比赛那套是永久锁死的）', can4)


def a_core():
    print('--- A: RemoteCore（一帧里先看什么后看什么）---')
    core, sent, _ = sim([(0.05, remote.KEY_UP)], 1.0)
    check('轻点一下只走一步', len(sent) == 1, f'→ {sent}')
    check('轻点的那一步是"前"(0x30)', sent and sent[0][1] == st.CMD_ADJUST_FWD)

    core, sent, _ = sim(hold(remote.KEY_UP, 0.05, 2.0), 2.3)
    check('按住 2 秒走出 4~6 步（间隔 0.4s）', 4 <= len(sent) <= 6, f'→ {len(sent)} 步 {gaps_of(sent)}')
    check('每两条之间都隔够了 interval', all(g >= remote.NUDGE_INTERVAL - 0.02
                                             for g in gaps_of(sent)), f'→ {gaps_of(sent)}')

    core, sent, _ = sim(hold(remote.KEY_UP, 0.05, 0.5), 1.4)
    tail = [t for t, _ in sent if t > 0.55]
    check('松手之后不再走（0.5s 松手，后面 0.9s 一步都没发）', not tail, f'→ {tail}')

    core, sent, _ = sim(hold(remote.KEY_UP, 0.05, 1.0), 1.2, state=st.STATE_WAIT_QR)
    check('门没开(0x00) 时一帧都不发', not sent)
    check('门没开的原因写在 HUD 上（不是不吭声）',
          '门没开' in core.blocked and '0x00' in core.blocked, f'→ {core.blocked}')

    core, sent, _ = sim(hold(remote.KEY_UP, 0.05, 1.0), 1.2, state=st.STATE_MOVING)
    check('车在跑(0x10) 时也不发', not sent and '门没开' in core.blocked, f'→ {core.blocked}')

    core, sent, _ = sim(hold(remote.KEY_UP, 0.05, 1.0), 1.2, fresh=False)
    check('状态不新鲜（0x20 断了）时一帧都不发', not sent)
    check('不新鲜的原因说清楚了', '状态不新鲜' in core.blocked, f'→ {core.blocked}')

    core, sent, _ = sim([(0.05, remote.KEY_UP), (0.06, remote.KEY_STOP)], 1.0)
    check('空格急停：欠着的步子作废，一步都不发', not sent, f'→ {sent}')

    core, sent, _ = sim([(0.05, remote.KEY_QUIT)], 0.2)
    check('q 的动作名是 quit', core.on_key(remote.KEY_QUIT, T0) == 'quit')
    check('认不出的键给 unknown（静默忽略，不刷屏）',
          core.on_key(b'z', T0) == 'unknown')


# ==================== B 段：端到端（假串口 + 假键盘）====================
class ScriptLink:
    """假下位机：按时间表报 0x20，把真正发出去的指令记下来。

    states = [(起, 止, 状态)]，没被盖住的时候报 0x01（等待抓取）。
    receipt=True 时收到微调后按协议把上报切成 0x10 一小会儿再切回 —— 主循环
    "一发一发来"的节拍就靠这个回执；receipt=False 模拟电控把门放开、0x01 一直发。
    """

    def __init__(self, states=(), receipt=True):
        self.states = list(states)
        self.receipt = receipt
        self.t0 = time.time()
        self.sent = []               # [(时刻, 指令字, 发出去那一刻下位机报的状态)]
        self.rx_count = {}
        self.moving_until = 0.0
        self.lock = threading.Lock()
        self.ser = 'fake'            # 主循环拿它判"连没连上"，非 None = 已连
        self.out = ''                # 主循环这段时间里打出来的话

    def start(self):
        pass

    def stop(self):
        pass

    def _state(self):
        if self.receipt and time.time() < self.moving_until:
            return st.STATE_MOVING
        t = time.time() - self.t0
        for a, b, s in self.states:
            if a <= t < b:
                return s
        return st.STATE_WAIT_GRAB

    def fresh_state(self):
        self.rx_count[st.TYPE_STATE] = self.rx_count.get(st.TYPE_STATE, 0) + 1
        return self._state(), True

    def send(self, frame, what, times=1):
        # 指令帧固定 5 字节：AA 55 21 01 CC，最后一个字节就是指令字
        if len(frame) == 5 and frame[0] == 0xAA and frame[1] == 0x55 and frame[2] == st.TYPE_CMD:
            with self.lock:
                self.sent.append((time.time() - self.t0, frame[4], self._state()))
                if self.receipt and frame[4] in st.ADJUST_CMDS:
                    self.moving_until = time.time() + 0.35   # 协议：微调期间报 0x10
        return True

    def steps(self):
        with self.lock:
            return [(t, c) for t, c, _s in self.sent if c in st.ADJUST_CMDS]

    def show(self):
        print(f'\n主循环跑了这段时间，串口上实际发出去 {len(self.sent)} 条：')
        for t, c, s in self.sent[:14]:
            print(f'  {t:5.2f}s  {st.CMD_CN.get(c, hex(c))}   （那一刻下位机报 0x{s:02X}）')
        if len(self.sent) > 14:
            print(f'  …（还有 {len(self.sent) - 14} 条）')
        print()


class ScriptKeys(remote.KeyReader):
    """假键盘：按时间线吐按键。走的是真的 KeyReader.feed()，所以转义序列切分一起测了。"""

    def __init__(self, script):
        super().__init__()
        self.script = sorted(script)
        self.t0 = time.time()
        self.i = 0

    def open(self):
        pass

    def close(self):
        pass

    def read(self):
        now = time.time() - self.t0
        data = b''
        while self.i < len(self.script) and self.script[self.i][0] <= now:
            data += self.script[self.i][1]
            self.i += 1
        return self.feed(data) if data else []


def run_remote(secs, keys, states=(), receipt=True, dry_run=False, extra_args=()):
    """把假串口/假键盘塞进真 remote.main()，跑 secs 秒（--run-secs 会让它自己退）。"""
    link = ScriptLink(states, receipt=receipt)
    remote.make_link = lambda *a, **k: link
    remote.make_keys = lambda: ScriptKeys(keys)
    sys.argv = (['remote.py', '--run-secs', str(secs)]
                + (['--dry-run'] if dry_run else []) + list(extra_args))
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        remote.main()
    link.out = buf.getvalue()
    return link


def scene_hold():
    """按住 ↑ 两秒，假下位机按协议给 0x10 回执 → 节拍由回执给。"""
    link = run_remote(2.8, hold(remote.KEY_UP, 0.05, 2.2))
    stp = link.steps()
    ts = [t for t, _c in stp]
    check('按住 2 秒走出 4~7 步', 4 <= len(stp) <= 7, f'→ {len(stp)} 步 {[round(t, 2) for t in ts]}')
    check('每两条之间都等了下位机回到 0x01（≥0.35s）',
          all(b - a >= 0.35 for a, b in zip(ts, ts[1:])),
          f'→ {[round(b - a, 2) for a, b in zip(ts, ts[1:])]}')
    check('没有一条是在 0x10(运动中) 期间发出去的',
          all(s != st.STATE_MOVING for _t, _c, s in link.sent if _c in st.ADJUST_CMDS))
    check('走的都是"前"(0x30)', all(c == st.CMD_ADJUST_FWD for _t, c in stp))
    check('HUD 上写着"下位机确认 N"（回执数对得上）', '下位机确认' in link.out)
    link.show()


def scene_noreceipt():
    """电控把门放开：0x01 一直发、没有 0x10 → 自适应成定时节拍，照样能走。"""
    link = run_remote(2.8, hold(remote.KEY_UP, 0.05, 2.2), receipt=False)
    stp = link.steps()
    ts = [t for t, _c in stp]
    check('没有回执也能走起来', len(stp) >= 4, f'→ {len(stp)} 步 {[round(t, 2) for t in ts]}')
    gaps = [b - a for a, b in zip(ts, ts[1:])]
    check('自适应之后按定时节拍发（约 0.4s）',
          all(0.3 <= g <= 0.7 for g in gaps[1:]) if len(gaps) > 1 else False,
          f'→ {[round(g, 2) for g in gaps]}')
    check('把"为什么改成定时"说明白了（不是偷偷换节拍）', '没报 0x10' in link.out)
    link.show()


def scene_closed():
    """门没开（一直报 0x00）→ 一条指令都不发，并且把原因打出来。"""
    link = run_remote(1.6, hold(remote.KEY_UP, 0.05, 1.4),
                      states=[(0, 99, st.STATE_WAIT_QR)])
    check('门没开时一条指令都不发', not link.steps(), f'→ {link.steps()}')
    check('把"门没开"和当前状态打出来了', '门没开' in link.out and '0x00' in link.out)
    check('也不是什么都没干：HUD 在刷（不是卡死了）', link.rx_count.get(st.TYPE_STATE, 0) > 20,
          f'→ 收到 {link.rx_count.get(st.TYPE_STATE, 0)} 帧状态')


def scene_recover():
    """先门没开，车跑到抓取点之后门开了 → 能接上接着走。"""
    link = run_remote(3.0, hold(remote.KEY_UP, 0.05, 2.6),
                      states=[(0, 1.2, st.STATE_WAIT_QR)])
    ts = [t for t, _c in link.steps()]
    check('前 1.2 秒（门没开）一步不走', not [t for t in ts if t < 1.2], f'→ {ts}')
    check('门开了之后马上能走', [t for t in ts if t > 1.25], f'→ {ts}')


def scene_dryrun():
    """--dry-run：不发串口，假下位机永远当 0x01 —— 桌面上试手感用的那条路。"""
    link = run_remote(2.8, hold(remote.KEY_UP, 0.05, 2.2), dry_run=True)
    check('干跑不碰串口（一条都没发出去）', not link.sent)
    n = link.out.count('[遥控] 走一步')
    check('干跑照样按节拍"走"（事件行在打）', 4 <= n <= 6, f'→ {n} 条')
    check('干跑的 HUD 写的是"干跑"而不是"串口没连上"', '干跑' in link.out)
    check('退出时把总步数说清楚了', '共 ' in link.out and '步' in link.out)


def pty_run(quit_key, keys_before=b'', secs=5.0):
    """在**真 pty** 里跑一遍 remote.py，返回 (跑起来的 lflag, 按键后的 lflag,
    退出后的 lflag, 输出, 退出码)。

    pty 主机侧的 tcgetattr 拿到的就是从机那套 termios，所以从外面就能盯着子进程
    有没有把终端改坏 —— 普通管道里根本跑不到这条路（stdin 不是 tty 就不设 termios）。

    **用 pty.fork() 而不是 Popen**：fork 出来的子进程是那个 pty 的会话首领、pty 就是
    它的控制终端，于是往主机侧写 0x03 才会真的变成 SIGINT（Ctrl-C 那条路才有得测）。
    Popen 的子进程没有控制终端，0x03 不会变成信号，测了个寂寞。
    """
    sys.stdout.flush()
    pid, master = pty.fork()
    if pid == 0:                               # 子进程：换掉镜像去跑真的 remote.py
        try:
            os.execv(sys.executable,
                     [sys.executable, os.path.abspath(remote.__file__),
                      '--dry-run', '--run-secs', str(secs)])
        finally:
            os._exit(127)
    chunks = []

    def drain():
        while True:
            try:
                d = os.read(master, 4096)      # 不读走的话 pty 缓冲满了子进程会卡在写
            except OSError:
                return
            if not d:
                return
            chunks.append(d)

    threading.Thread(target=drain, daemon=True).start()

    def lflag():
        return termios.tcgetattr(master)[3]

    time.sleep(1.0)
    running = lflag()
    if keys_before:
        os.write(master, keys_before)
        time.sleep(0.8)
    mid = lflag()
    os.write(master, quit_key)

    rc, deadline = None, time.time() + 8
    while time.time() < deadline:
        wpid, status = os.waitpid(pid, os.WNOHANG)
        if wpid == pid:
            rc = os.waitstatus_to_exitcode(status)
            break
        time.sleep(0.05)
    if rc is None:                             # 没退（比如 Ctrl-C 没生效）就掐掉，别挂着
        os.kill(pid, signal.SIGKILL)
        os.waitpid(pid, 0)
    time.sleep(0.3)                            # 让 drain 把那点尾巴收干净
    return running, mid, lflag(), b''.join(chunks).decode('utf-8', 'replace'), rc


def scene_pty():
    """真终端（pty）这条路：cbreak 进去了、按键认、两条退出路径都**还原**终端。

    这条必须单独测：KeyReader 关掉的是 ECHO/ICANON，**没还原就等于把人的 shell
    弄成不回声的砖**。而且 pty 上还藏着另一个坑 —— VMIN=0 时"没数据"的 read()
    返回 0 字节，和 EOF 长得一样，只按管道测是发现不了的。
    """
    run_l, mid_l, after_l, out, rc = pty_run(remote.KEY_QUIT, remote.KEY_UP * 2)
    check('跑起来之后终端进了 cbreak（ICANON/ECHO 关掉，ISIG 留着 → Ctrl-C 还能用）',
          not (run_l & termios.ICANON) and not (run_l & termios.ECHO),
          f'→ ICANON={bool(run_l & termios.ICANON)} ECHO={bool(run_l & termios.ECHO)}')
    check('ISIG 留着（Ctrl-C 还能当信号使，不用自己认 0x03）', bool(run_l & termios.ISIG))
    check('pty 里的方向键也认（转义序列照样切得开）', '走一步 ↑前' in out)
    check('没按键的时候不能误判成 EOF（VMIN=0 的坑）', 'EOF' not in out)
    check('q 能退出', rc == 0, f'→ 退出码 {rc}')
    check('q 退出后终端**还原了**（ICANON/ECHO 回来 → shell 不会变砖）',
          bool(after_l & termios.ICANON) and bool(after_l & termios.ECHO))

    _r, _m, after2, out2, rc2 = pty_run(b'\x03')     # Ctrl-C：出事了最常用的那条退路
    check('Ctrl-C 也能退出（真终端上 SIGINT 要走 finally）', rc2 in (0, 130), f'→ 退出码 {rc2}')
    check('Ctrl-C 之后终端也还原了', bool(after2 & termios.ICANON) and bool(after2 & termios.ECHO))
    check('退出时说清楚了是 Ctrl-C', 'Ctrl-C' in out2)


SCENES = {'hold': scene_hold, 'noreceipt': scene_noreceipt, 'closed': scene_closed,
          'recover': scene_recover, 'dryrun': scene_dryrun, 'pty': scene_pty}


def main():
    if len(sys.argv) > 1 and sys.argv[1] in SCENES:
        SCENES[sys.argv[1]]()
        print('\n' + ('全部通过' if ok else '有失败项'))
        sys.stdout.flush()
        os._exit(0 if ok else 1)          # 和 check_search.py 一个收尾写法

    a_keyhold()
    a_keyreader()
    a_pacer()
    a_core()
    print('\n===== B 段：端到端（每个场景一个进程）=====')
    results = {}
    for name in SCENES:
        print(f'----- 场景 {name} -----')
        p = subprocess.run([sys.executable, os.path.abspath(__file__), name],
                           capture_output=True, text=True)
        print(p.stdout, end='')
        if p.stderr.strip():
            print(p.stderr, end='')
        results[name] = (p.returncode == 0)
    bad = [n for n, good in results.items() if not good]
    print('\n' + ('全部通过' if not bad else f'有失败项：{"、".join(bad)}'))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""遥控模式：键盘手动开这台车（**赛后玩具，比赛不能用**）。

rules_text.txt 第 18~19 行写得很清楚："允许与笔记本电脑进行通讯，比赛中不能触碰
笔记本电脑，**不允许任何形式的遥控**。" 所以这个脚本只在赛后练手/遛车时用，
别在比赛里跑。

为什么另起一个文件：比赛那套 schooltest2.py 一个字都不用动（用户要求），这里
`import schooltest2 as st` 复用串口、协议常量和 NudgePacer，自己只管"哪个键按着 → 发哪条"。

    key:  ↑ / w 前    ↓ / s 后    ← / a 左    → / d 右    空格 急停    q 退出

怎么动的：只有微调这一条路。`0x21` 指令帧的合法指令字总共就 7 个
（00/01/02 和 30~33），能开车的就是 0x30~0x33 那 4 个，**一次固定走 3 cm**
（下位机侧定的值，上位机没地方传距离，见 USART1_Nudge_Protocol.md §二）。
所以遥控是"一步一发"的点动，不是连续调速 —— 按住时的速度 = 步频 × 3cm。

门：**只有下位机在周期上报 0x01（等待抓取指令）时才收微调**，别的时候静默丢弃
（不计错、不缓存）。本来那个窗口只在第 3 个停顿点开，赛后由电控放开 —— 但上位机
仍然要自己看状态，不能盲发（协议 §三原话）。

两个固件行为都要能跑，StepPacer 会自己判（见它的 docstring）：
  (a) 微调期间报 0x10、走完回 0x01  → 节拍由下位机给，一步不丢；
  (b) 门放开后 0x01 一直发、没有 0x10 → 收不到回执，退回按固定间隔定时发。

不做相机预览，理由：cv2.imshow 的窗口一聚焦，方向键就发给窗口而不是终端，遥控
直接失灵；SSH 进来时 DISPLAY 为空根本出不来窗口；遥控是开环的，画面既不参与
决策也不影响安全。要看画面就另开一个 schooltest2.py（**但这俩会抢串口，别同时跑**）。

跑法（开发机上只能这样试，真机去树莓派）：

    python3 remote.py --dry-run            # 桌面上试手感：不发串口，假下位机永远当 0x01
    printf '\\033[A\\033[A' | python3 remote.py --dry-run --run-secs 2    # 管道喂键
    python3 remote.py                      # 真发（串口没连上会自动降级成只打印）
"""

import argparse
import atexit
import fcntl
import os
import select
import signal
import sys
import termios
import time

import schooltest2 as st

# ==================== 可调参数（都在这里改，命令行只是临时覆盖）====================

# ---------------- 键位 ----------------
# 方向键在终端里是转义序列（ESC [ A/B/C/D），一次 read 可能只读到半截，
# 交给 KeyReader.feed() 去拼。想改键或者加键，只改下面这两张表。
KEY_UP = b'\x1b[A'
KEY_DOWN = b'\x1b[B'
KEY_LEFT = b'\x1b[D'
KEY_RIGHT = b'\x1b[C'
KEY_W = b'w'                     # WASD 是兜底：有些终端不自动重复方向键，
KEY_S = b's'                     # 而"按住就走"完全靠终端的自动重复
KEY_A = b'a'
KEY_D = b'd'
KEY_STOP = b' '                  # 空格 = 急停：手上按着的方向全清掉
KEY_QUIT = b'q'                  # q = 退出（Ctrl-C 也行）
KEYMAP = {
    KEY_UP: st.CMD_ADJUST_FWD, KEY_W: st.CMD_ADJUST_FWD,
    KEY_DOWN: st.CMD_ADJUST_BACK, KEY_S: st.CMD_ADJUST_BACK,
    KEY_LEFT: st.CMD_ADJUST_LEFT, KEY_A: st.CMD_ADJUST_LEFT,
    KEY_RIGHT: st.CMD_ADJUST_RIGHT, KEY_D: st.CMD_ADJUST_RIGHT,
}
DIR_CN = {st.CMD_ADJUST_FWD: '↑前', st.CMD_ADJUST_BACK: '↓后',
          st.CMD_ADJUST_LEFT: '←左', st.CMD_ADJUST_RIGHT: '→右'}
DIR_EN = {st.CMD_ADJUST_FWD: 'fwd', st.CMD_ADJUST_BACK: 'back',
          st.CMD_ADJUST_LEFT: 'left', st.CMD_ADJUST_RIGHT: 'right'}
# 除了方向键，主循环还要认这两个单键（空格/q 不在这张表里，所以单独列出来）
SPECIAL_KEYS = (KEY_STOP, KEY_QUIT)

# ---------------- 手感（"按住就走、松手停"）----------------
# 终端的自动重复：按下后先等 REPEAT_DELAY（xset 默认 660ms 左右）才开始，
# 之后每 REPEAT_PERIOD（约 30~40ms）吐一个字符。所以"哪个方向还按着"不能只看
# 有没有事件，要看**最近一次事件有多新** —— 这就是下面这两个窗口的作用。
KEY_FRESH_MAX = 0.75     # 新鲜度窗口上限。要扛过自动重复的初始延迟（660ms），
                         # 所以比它大一点；同时也是"轻点一下"那一步的存活时间
KEY_FRESH_MIN = 0.15     # 量到自动重复周期之后收紧到它 → 松手 0.15s 内就停住了
KEY_REPEAT_FACT = 2.5    # 窗口 = 2.5 × 实测的重复周期（30 次/秒 → 0.083s，被 MIN 兜住）
ESC_GRACE = 0.05         # 单独一个 ESC 留这么久等后面的 "[A"（可能分两次 read 到）
LOOP_HZ = 50.0           # 主循环频率。它对速度没有影响（真正的限速是节拍器），
                         # 但要够快才能及时读到按键
RUN_SECS = 0.0           # 跑这么多秒自己退出。**0 = 一直跑**（--run-secs）。
                         # 只给离线测试和"试两秒手感"用，正常玩就留 0

# ---------------- 节拍（两条微调之间隔多久）----------------
NUDGE_INTERVAL = 0.40    # 两条微调之间**至少**隔这么久。下位机报 0x10 时真节拍是它的
                         # 0x01（往返 0.4~0.5s），这个值只在"门放开、一直报 0x01"和
                         # 干跑时才是唯一节拍。一步本来就走 0.3s，所以别小于 0.35 —— 会被丢
NUDGE_RETURN_TIMEOUT = 0.8   # 发出去这么久一次 0x10 都没见到 → 认定这一版固件不给回执
                             # （判据见 StepPacer）。给 0.8s 是"判得快"和"别把慢回执
                             # 误判成没回执"之间的折中，协议说往返只要 0.4~0.5s
NUDGE_STUCK_RETRY = 2.0  # 真没生效（帧丢了/被门拒）时，隔这么久允许再试。
                         # 比赛代码里 stuck 是"这一站不再挪车"，遥控锁死就没法玩了
NUDGE_STEP_CM = 3.0      # 协议固定步数。上位机没地方传距离，所以只能拿它"估"走了多远
                         # （打滑和门拒的不算数，所以 HUD 里还挂一个下位机确认过的步数）
BLOCK_HOLD = 3.0         # "为什么发不出去"那句话在 HUD 上多留几秒。不然按键一松手
                         # 它就没了 —— 轻点一下没反应的人根本来不及看清原因
PACE = 'auto'            # auto  = 按上面自适应：见到 0x10 就按回执走，见不到就切定时
                         # state = 只等 0x01 回执（下位机一定会报 0x10 时用）
                         # time  = 只按 NUDGE_INTERVAL 定时（明知不给回执时用，省得等一轮）

# ---------------- 串口 / 心跳 ----------------
HB_HZ = 5.0              # 空指令(0x21 的 0x00)心跳频率，和比赛代码一个口径。
                         # **0 = 不发心跳**（--hb-hz 0），只在确认下位机不需要心跳时用

# ---------------- 终端 HUD ----------------
HUD_HZ = 10.0            # 状态行最多这么勤重画（内容没变根本不画）
HUD_MAX_AGE = 1.0        # 这么久没重画过就强制画一次（兜底，防止画面停在旧状态）
HUD_PLAIN_INTERVAL = 2.0 # stdout 不是终端时（重定向/管道）退化成每 2 秒打一整行，
                         # 免得把日志塞满，也免得把 \r\x1b[K 这种转义写进文件
HUD_WIDTH = 100          # 拿不到终端宽度时的兜底宽度。**HUD 必须截断**：
                         # 一旦折行，\r 只能回到最后一行的行首，画面会开始叠罗汉


# ==================== "按住就走、松手停"的判断 ====================
class KeyHold:
    """哪个方向还按着 / 现在该不该走一步。

    纯逻辑：不碰键盘、不碰串口、时间从外面传进来 —— 这样离线测试能直接把时间线
    推出来（check_remote.py 的 A 段就是这么测的）。

    判定式（就这一条）：

        held(d, now)  ⇔  token[d] == 1 且 now - last[d] <= fresh(d)
        fresh(d)      =  KEY_FRESH_MAX                      # 还没量到重复周期
                       =  clamp(2.5 × gap[d], MIN, MAX)     # 量到了就收紧

    `token` 是"还欠这个方向一步"的额度，**上限 1**：

    - 按住时终端每 30ms 吐一个事件，每个事件给一次额度 → 走完一步扣掉，
      下一个事件马上又给回来，于是连着走（真正的限速在节拍器那边）；
    - 轻点一下只有一个事件：额度扣掉之前一直留着，等节拍器腾出手就补上这一步
      → **轻点不会被节拍器吞掉**；
    - 额度上限 1 是关键：连发 20 个事件也只欠 1 步，不会攒成一串追着走。
    """

    def __init__(self, fresh_max=KEY_FRESH_MAX, fresh_min=KEY_FRESH_MIN,
                 repeat_fact=KEY_REPEAT_FACT):
        self.fresh_max = fresh_max
        self.fresh_min = fresh_min
        self.repeat_fact = repeat_fact
        self.last = {}      # cmd -> 最近一次按键事件的时刻
        self.gap = {}       # cmd -> 相邻两次事件的间隔（= 终端自动重复的周期）
        self.token = {}     # cmd -> 还欠几步（0 或 1）

    def press(self, cmd, now):
        """收到一个该方向的按键事件（可能是第一次按下，也可能是自动重复）。"""
        prev = self.last.get(cmd)
        if prev is not None and now > prev:
            self.gap[cmd] = now - prev      # 实测重复周期，下一帧起就拿来收紧窗口
        self.last[cmd] = now
        self.token[cmd] = 1

    def fresh(self, cmd):
        g = self.gap.get(cmd)
        if g is None:
            return self.fresh_max           # 还没量到周期：刚按下，或者只轻点了一下
        return min(self.fresh_max, max(self.fresh_min, self.repeat_fact * g))

    def held(self, now):
        """这一刻还算按着哪个方向？返回 (指令字, 距上次事件多久)；都没有就是 (None, None)。

        同时按着好几个时**最后按下的那个赢**（取事件最新的），跟人的直觉一致。
        """
        best, age = None, None
        for cmd, t in self.last.items():
            if not self.token.get(cmd):
                continue                    # 这一步已经发出去了，等下一个事件
            a = now - t
            if a > self.fresh(cmd):
                continue                    # 太旧了 = 松手了（或者那个事件早被吃掉了）
            if age is None or a < age:
                best, age = cmd, a
        return best, age

    def consume(self, cmd):
        """真要发这一步了：把额度扣掉（下次想走要等新的按键事件）。"""
        self.token[cmd] = 0

    def clear(self):
        """急停：所有额度清零，手上还按着也不会再走（要重新按一下才动）。"""
        self.token.clear()


# ==================== 键盘（termios 非阻塞单键）====================
class KeyReader:
    """非阻塞读单键。终端用 cbreak，管道也能读。

    - **cbreak 而不是 raw**：关 ICANON/ECHO，**留着 ISIG** —— Ctrl-C 照样变成
      SIGINT（退出和还原终端就有现成的路子），而且 OPOST 还在，普通 print() 照常
      工作（HUD 的事件行不用自己补 \\r）。
    - **stdin 不是终端也照样读字节**，只是不调 termios。于是
      `printf '\\033[A' | python3 remote.py --dry-run` 直接能玩，端到端测试也能
      用管道喂按键。
    - **必须非阻塞**：终端上钉 VMIN=0/VTIME=0，管道上加 O_NONBLOCK。不然主循环会
      卡在 read 上，HUD 不刷新、按键事件堆在缓冲里一起涌出来。
    """

    def __init__(self, fd=0, keymap=None, esc_grace=ESC_GRACE):
        self.fd = fd
        self.keymap = dict(keymap or KEYMAP)
        self.known = tuple(self.keymap) + tuple(SPECIAL_KEYS)
        # 已知序列的所有**真前缀**（不含自身）：只有落在这些前缀上才值得留在缓冲里等
        self.prefixes = {k[:i] for k in self.known for i in range(1, len(k))}
        self.esc_grace = esc_grace
        self.old = None
        self.buf = b''
        self.esc_at = None
        self.eof = False
        self.tty = False

    def open(self):
        try:
            self.tty = os.isatty(self.fd)
        except OSError:
            self.tty = False
        if self.tty:
            try:
                self.old = termios.tcgetattr(self.fd)
                new = termios.tcgetattr(self.fd)
                new[3] &= ~(termios.ICANON | termios.ECHO)   # lflag；ISIG 故意留着
                new[6][termios.VMIN] = 0                     # cc：不等字节、不等时间
                new[6][termios.VTIME] = 0
                termios.tcsetattr(self.fd, termios.TCSADRAIN, new)
            except termios.error as e:
                self.old = None
                print(f'[遥控] 终端进不了单键模式（{e}），按键要回车才生效')
        else:
            try:
                fl = fcntl.fcntl(self.fd, fcntl.F_GETFL)
                fcntl.fcntl(self.fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
            except OSError:
                pass

    def close(self):
        """还原终端。**不退就等着 shell 变成不回声的砖**，所以退出路径一定要走到。"""
        if self.old is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
            except termios.error:
                pass
            self.old = None

    def read(self):
        """读一次（有多少读多少），返回认出来的键。EOF 之后不再读。

        **必须先 select 探一下有没有数据**：VMIN=0 的终端上，没数据时 read() 返回
        的是**0 字节**，和"管道写完了"的 EOF 长得一模一样 —— 直接拿它当 EOF 的话，
        真终端上开机第一帧就会打一句"stdin 到 EOF"然后再也不认按键（整个遥控就是
        死的）。select 说可读之后 read 还返回 0 字节，那才是真的挂断/EOF。
        """
        if self.eof:
            return []
        try:
            ready, _, _ = select.select([self.fd], [], [], 0)
        except (OSError, ValueError):
            ready = [self.fd]                # select 用不了就退回直接读
        if not ready:
            return []
        try:
            data = os.read(self.fd, 64)
        except (BlockingIOError, OSError):
            return []                        # 非阻塞，没数据是正常的
        if not data:
            self.eof = True                  # 管道喂完了：再读会一直返回 b''，会空转烧核
            print('[遥控] stdin 到 EOF，后面不会再有按键了（按 Ctrl-C 或 q 退出）')
            return []
        return self.feed(data)

    def feed(self, data):
        """把字节流切成一个个键。**纯函数**，测试直接喂字节就行（不经 os.read）。"""
        self.buf += data
        out = []
        while self.buf:
            hit = None
            for k in self.known:             # 已知键里最长的先试（方向键 3 字节）
                if self.buf.startswith(k):
                    hit = k
                    break
            if hit is not None:
                out.append(hit)
                self.buf = self.buf[len(hit):]
                self.esc_at = None
                continue
            if self.buf in self.prefixes:
                # 还只是某个键的前半截（ESC 和 [A 分两次 read 到）→ 留着等下一批
                if self.buf[:1] == b'\x1b' and self.esc_at is None:
                    self.esc_at = time.time()
                break
            # 认不出来：丢掉一个字节重来。这样 \x1b[5~(PageUp)、\x1bOP(F1) 这类
            # 杂序列会被一个字节一个字节吃掉，不会卡在缓冲里挡住后面的键
            self.buf = self.buf[1:]
            self.esc_at = None
        return out

    def flush(self, now):
        """超时的半截转义序列丢掉。

        不然一个误按的 ESC 会永久占着缓冲，后面的方向键全被当成"半截序列"吞掉。
        """
        if self.buf and self.buf[:1] == b'\x1b' and self.esc_at is not None \
                and now - self.esc_at > self.esc_grace:
            self.buf = b''
            self.esc_at = None


# ==================== 节拍（两条微调之间）====================
class StepPacer:
    """st.NudgePacer 外面包一层。只有两处语义和比赛代码不同：

    1. **stuck 不永久停手**。比赛里 stuck 是"这一站不再挪车"，遥控锁死就没法玩了：
       清掉 latch，`NUDGE_STUCK_RETRY` 秒后允许再试。
    2. **自适应"微调期间不报 0x10"的固件**。协议 §四 画的是 0x01 → 0x10 → 0x01，
       节拍就靠那个回来的 0x01；但电控把门放开之后 0x01 会一直发，这条路走不到。
       判据：发出去了、`NUDGE_RETURN_TIMEOUT` 秒内一次 0x10 都没见到 → 认定这版
       固件不给回执，之后走 `sent(real=False)` —— **那是 NudgePacer 自带的干跑路径**
       （不等回执，退回按 interval 定时），不用改它一行。
    """

    def __init__(self, interval=NUDGE_INTERVAL, timeout=NUDGE_RETURN_TIMEOUT,
                 retry=NUDGE_STUCK_RETRY, gap=None, mode=PACE):
        self.pacer = st.NudgePacer(interval=interval, timeout=timeout,
                                   gap=st.ALIGN_NEW_STOP_GAP if gap is None else gap)
        self.mode = mode
        self.retry = retry
        # None = 还没判出来 / True = 固件给 0x10 回执 / False = 不给，按定时发
        self.state_paced = None if mode == 'auto' else (mode == 'state')
        self.retry_at = 0.0
        self.confirmed = 0      # 下位机回 0x01 确认走完的步数
        # NudgePacer 里 `at` 初值是 0.0，意思是"刚发过一条"，于是**第一条也要等满
        # interval** 才放行。真跑时时间戳是 time.time()（十几亿），差这一下看不出来；
        # 但离线测试的钟从 0 起，第一条就会被平白压 0.4s，量出来的时间线全是歪的。
        # 拨到 -interval：第一条按下就发，和真机上的手感一致。
        self.pacer.at = -self.pacer.interval

    def tick(self, state, now):
        """每帧喂一次 0x20 状态，返回这一刻要打的事件行（可能为空）。"""
        msgs = []
        ev = self.pacer.note(state, now)
        if ev == 'done':
            self.confirmed += 1
            if self.state_paced is None:
                self.state_paced = True      # 见到 0x10 回执了 → 按状态走是最稳的
        elif ev == 'stuck':
            if self.mode == 'auto' and self.state_paced is None:
                self.state_paced = False
                msgs.append(f'下位机微调期间没报 0x10（电控把门放开了 / 这版固件不给回执）'
                            f'→ 节拍改成按 {self.pacer.interval:.2f}s 定时发')
            else:
                self.retry_at = now + self.retry
                msgs.append(f'上一发 {self.pacer.timeout:.1f}s 没等到下位机动一下'
                            f'（帧丢了或被门拒了）→ {self.retry:.0f}s 后还能再试')
            self.pacer.new_stop()            # 清掉 latch，别永久停手
        return msgs

    def ready(self, now):
        """现在能不能发下一发？返回 (能不能, 不能的原因)。"""
        if now < self.retry_at:
            return False, f'上一发没生效，{self.retry_at - now:.1f}s 后还能再试'
        can, why = self.pacer.ready(now)
        if can:
            return True, None
        if why:
            return False, why
        if self.pacer.pending:
            return False, f'上一发还在走（{now - self.pacer.at:.1f}s），等它回到 0x01'
        return False, f'两条之间至少隔 {self.pacer.interval:.2f}s（下位机一步要走 0.3s）'

    def sent(self, now, real=True):
        """刚发出去一条。`real` = 这一帧真写进串口了吗（干跑/串口没连上就是 False）。"""
        expect_receipt = (self.state_paced is not False) and real
        self.pacer.sent(now, real=expect_receipt)


# ==================== 遥控的核心判断（纯逻辑，可离线测）====================
class RemoteCore:
    """把"按键 → 发哪条指令"整套判断收在这里：不碰串口、不碰键盘、时间从外面传。

    主循环只管 IO：读键喂 on_key()，每帧喂状态调 tick()，发完调 sent()。
    """

    def __init__(self, dry_run=False, hold=None, pacer=None, step_cm=NUDGE_STEP_CM):
        self.hold = hold or KeyHold()
        self.pacer = pacer or StepPacer()
        self.dry_run = dry_run
        self.step_cm = step_cm
        self.steps = 0          # 决定要发的步数（一步一步攒出来的）
        self.frames = 0         # 真写进串口的帧数（和 steps 不一致就说明有发送失败）
        self.blocked = ''       # 最近一次"发不出去"的原因（HUD 挂着它）
        self.blocked_at = 0.0   # 记这个是为了让它多留 BLOCK_HOLD 秒，好让人看清
        self.dir = None         # 当前按着的方向，给 HUD 用

    def _block(self, now, msg):
        """记下"这一步为什么不发"，HUD 和主循环都拿它说话（不是不吭声）。"""
        self.blocked = msg
        self.blocked_at = now

    def on_key(self, key, now):
        """收到一个键。返回动作名：'fwd'/'back'/'left'/'right'/'stop'/'quit'/'unknown'。"""
        if key == KEY_QUIT:
            return 'quit'
        if key == KEY_STOP:
            self.hold.clear()                # 急停：欠着的步子全作废（要重按才动）
            return 'stop'
        cmd = KEYMAP.get(key)
        if cmd is None:
            return 'unknown'                 # 认不出的键静默忽略，不刷屏
        self.hold.press(cmd, now)
        return DIR_EN[cmd]

    def tick(self, now, state, fresh):
        """一帧。返回 (要发的帧|None, 这一发是什么|None, 要打的事件行列表)。

        **判定顺序本身就是安全设计，别调换**：先收节拍器的账，再看有没有按键，
        再看状态可不可信、门开没开，最后才问节拍器能不能发。
        """
        if self.dry_run:
            # 干跑 = 假下位机：永远当 0x01、永远新鲜，好让桌面上能试出手感。
            # 真要看状态就别加 --dry-run（串口没连上时状态不新鲜，什么都不会发）
            state, fresh = st.STATE_WAIT_GRAB, True

        msgs = self.pacer.tick(state, now)

        d, _age = self.hold.held(now)
        self.dir = d
        if d is None:
            return None, None, msgs             # 没按着 = 不发（松手就停）

        if not fresh:
            self._block(now, f'状态不新鲜：{st.STATE_TIMEOUT:.1f}s 没收到 0x20 状态帧，'
                             f'不敢发（串口接上了吗）')
            return None, None, msgs

        if state != st.STATE_WAIT_GRAB:
            self._block(now, f'门没开：微调只在 0x01(等待抓取指令) 收，现在 '
                             f'0x{state:02X} {st.STATE_CN.get(state, "未知")}，'
                             f'发出去会被下位机静默丢弃')
            return None, None, msgs

        can, why = self.pacer.ready(now)
        if not can:
            self._block(now, why or '还不能发')
            return None, None, msgs

        self.hold.consume(d)
        self.steps += 1
        self.blocked = ''                       # 发出去了，之前那句"为什么不动"就撤掉
        return (st.build_cmd_frame(d), f'走一步 {DIR_CN[d]}（第 {self.steps} 步）', msgs)

    def sent(self, now, real):
        """主循环发完回报。`real` = 真写进串口了吗。"""
        self.pacer.sent(now, real=real)
        if real:
            self.frames += 1

    def hud_text(self, now, state, fresh, port, linked, rx=None):
        """状态行。截断由 Hud 负责。"""
        if self.dry_run:
            st_txt = '干跑（假下位机，当 0x01）'
        elif state is None:
            st_txt = '还没收到过 0x20 状态'
        else:
            st_txt = f'0x{state:02X} {st.STATE_CN.get(state, "未知")}'
            if not fresh:
                st_txt += '（不新鲜）'
        pace = {True: '按回执', False: '定时', None: '待判'}[self.pacer.state_paced]
        parts = [
            '微调 3cm/步·' + pace,
            st_txt,
            '方向 ' + (DIR_CN.get(self.dir, '—') if self.dir else '—'),
            f'已走 {self.steps} 步 ≈{self.steps * self.step_cm:.0f}cm'
            + (f'（下位机确认 {self.pacer.confirmed}）' if self.pacer.state_paced else ''),
            ('干跑·不发串口' if self.dry_run
             else f'{port} {"已连" if linked else "没连上·只打印"}'),
        ]
        if rx:
            parts.append('收 ' + ' '.join(f'0x{k:02X}:{v}' for k, v in sorted(rx.items())))
        parts.append('空格=急停 q=退出')
        line = '[遥控] ' + ' | '.join(parts)
        if self.blocked and now - self.blocked_at <= BLOCK_HOLD:
            line += '  ⚠ ' + self.blocked
        return line


# ==================== 串口 ====================
class QuietLink(st.McuLink):
    """不发 `[发送] ...` 那一行的 McuLink。

    子类化是为了**一行都不动 schooltest2.py**（用户明确要求）。代价：这份 send()
    和上游会漂 —— schooltest2.McuLink.send 改了要记得同步这 15 行。

    为什么非躲不可：心跳 5Hz + 步进，每帧一行会把 HUD 冲掉，SSH 下的终端 IO 还会
    把主循环拖慢。发出去的每一条由 HUD 的事件行自己记（"走一步 ↑前 第 7 步"），
    比裸 hex 有用；要看原始字节就加 --tx-log。
    """

    def send(self, frame, what, times=1, quiet=None):
        quiet = self.verbose if quiet is None else quiet
        if self.ser is None:
            if not self._no_port_warned:
                self._no_port_warned = True
                print(f'[串口] {self.port} 没连上，往后只打印不发（接上会自动重连）')
            return False
        with self.lock:
            try:
                for i in range(times):
                    self.ser.write(frame)
                    self.ser.flush()
                    if not quiet:
                        tag = f' 第 {i + 1}/{times} 次' if times > 1 else ''
                        print(f'[发送] {what}{tag}: {frame.hex(" ").upper()}')
                    if times > 1:
                        time.sleep(0.1)
                return True
            except (st.serial.SerialException, OSError) as e:
                print(f'[发送] {what} 失败: {e}')
                self.ser = None
                return False


def make_link(port, baudrate, tx_log):
    """接缝：测试把它换成假串口。"""
    link = QuietLink(port, baudrate, verbose=tx_log)
    link.start()
    return link


def make_keys():
    """接缝：测试把它换成按时间线吐键的假键盘。"""
    return KeyReader()


# ==================== 终端 HUD ====================
def term_width(default=HUD_WIDTH):
    try:
        return os.get_terminal_size().columns
    except OSError:
        return default


class Hud:
    """一行状态 + 事件行。**只在主循环里画**（收包线程/相机线程一律不碰 stdout）。

    状态行原地重画（`\\r\\x1b[K`），事件行擦掉状态行再打。stdout 不是终端时
    （重定向/管道/测试）退化成每 HUD_PLAIN_INTERVAL 秒打一整行，不写转义。
    """

    def __init__(self, out=None, plain=None, interval=HUD_PLAIN_INTERVAL):
        self.out = out or sys.stdout
        self.plain = (not self.out.isatty()) if plain is None else plain
        self.interval = interval
        self.width = term_width()
        self.last = ''
        self.t = -1e9

    def log(self, msg):
        """事件行：先擦掉状态行，再打这一行。"""
        if self.plain:
            self.out.write(msg + '\n')
        else:
            self.out.write('\r\x1b[K' + msg + '\n')
            self.last = ''                   # 状态行被擦了，下一帧要重画
        self.out.flush()

    def render(self, text, now, force=False):
        if self.plain:
            if not force and now - self.t < self.interval:
                return
            self.out.write(text + '\n')
            self.out.flush()
            self.t = now
            return
        if not force:
            if now - self.t < 1.0 / HUD_HZ:
                return
            if text == self.last and now - self.t < HUD_MAX_AGE:
                return
        self.out.write('\r\x1b[K' + text[:self.width - 1])   # 截断：折行会让画面叠罗汉
        self.out.flush()
        self.last = text
        self.t = now

    def close(self):
        if not self.plain:
            self.out.write('\n')             # 别让 shell 提示符盖住状态行
            self.out.flush()


# ==================== 主循环 ====================
BANNER = """\
============================================================
 遥控模式（赛后玩具）—— 比赛规则不允许任何形式的遥控，别在比赛里用
   ↑/w 前   ↓/s 后   ←/a 左   →/d 右   空格 急停   q 退出
 一步固定 3cm，按住就走、松手停（速度 ≈ 每秒 2 步 ≈ 6cm/s）
 门：下位机报 0x01 时才收微调，别的时候会被静默丢弃（HUD 会说为什么）
============================================================"""


def build_parser():
    ap = argparse.ArgumentParser(
        description='键盘遥控（赛后玩具，比赛不能用）。默认真发串口，串口没连上自动只打印。',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=st.PORT, help=f'串口设备（默认 {st.PORT}）')
    ap.add_argument('--baud', type=int, default=st.BAUDRATE, help=f'波特率（默认 {st.BAUDRATE}）')
    ap.add_argument('--dry-run', action='store_true',
                    help='只打印不发给下位机，并且把下位机当成一直在 0x01 —— '
                         '桌面上试手感用。默认是**真发**；串口没连上时自动只打印')
    ap.add_argument('--run-secs', type=float, default=RUN_SECS,
                    help=f'跑这么多秒自己退出；0 = 一直跑（默认 {RUN_SECS:.0f}）')
    ap.add_argument('--pace', choices=['auto', 'state', 'time'], default=PACE,
                    help='节拍来源：auto=见到 0x10 就按回执、见不到就切定时（默认）；'
                         'state=只等 0x01 回执；time=只按定时')
    ap.add_argument('--nudge-interval', type=float, default=NUDGE_INTERVAL,
                    help=f'两条微调之间至少隔几秒（默认 {NUDGE_INTERVAL:.2f}，'
                         f'小于 0.35 有被下位机丢掉的风险）')
    ap.add_argument('--hb-hz', type=float, default=HB_HZ,
                    help=f'空指令心跳频率，0 = 不发（默认 {HB_HZ:.0f}Hz）')
    ap.add_argument('--tx-log', action='store_true', help='打印每一帧发出去的原始字节')
    return ap


def loop(core, keys, link, hud, args):
    """主循环。返回退出原因字符串。"""
    t0 = time.time()
    last_hb = t0
    last_blocked = ''
    while True:
        now = time.time()

        for key in keys.read():
            act = core.on_key(key, now)
            if act == 'quit':
                return '收到 q'
            if act == 'stop':
                hud.log('[遥控] 急停：欠着的步子都作废了，要动得重新按')

        keys.flush(now)

        if link is None:
            state, fresh = None, False       # 干跑：core 自己会当成 0x01
        else:
            state, fresh = link.fresh_state()
            if args.hb_hz > 0 and now - last_hb >= 1.0 / args.hb_hz:
                last_hb = now
                link.send(st.build_cmd_frame(st.CMD_IDLE), '空指令(心跳)')

        frame, what, msgs = core.tick(now, state, fresh)
        for m in msgs:
            hud.log('[遥控] ' + m)
        # "按了没反应"的原因只说一次（原因变了才说），别每帧刷屏；
        # 持续期间它挂在状态行尾巴上（BLOCK_HOLD 秒）
        if core.blocked != last_blocked:
            if core.blocked:
                hud.log('[遥控] ⚠ ' + core.blocked)
            last_blocked = core.blocked

        if frame is not None:
            if link is None:
                real = False
                if args.tx_log:
                    hud.log(f'[干跑] {what}: {frame.hex(" ").upper()}')
            else:
                real = link.send(frame, what)
            core.sent(now, real)
            hud.log(f'[遥控] {what}')

        hud.render(core.hud_text(now, state, fresh, args.port,
                                 bool(link is not None and link.ser is not None),
                                 None if link is None else link.rx_count), now)

        if args.run_secs > 0 and now - t0 >= args.run_secs:
            return f'跑满 --run-secs {args.run_secs:g}s'
        time.sleep(max(0.0, 1.0 / LOOP_HZ - (time.time() - now)))


def main():
    args = build_parser().parse_args()
    print(BANNER)
    print(f'[遥控] 串口 {args.port} @{args.baud}  '
          + ('**干跑**：只打印不发，下位机当成一直在 0x01' if args.dry_run
             else '真发（串口没连上会自动降级成只打印）'))
    print(f'[遥控] 节拍：{args.pace}，两条微调至少隔 {args.nudge_interval:.2f}s；'
          + (f'心跳 {args.hb_hz:.0f}Hz' if args.hb_hz > 0 else '不发心跳')
          + '（别同时跑 schooltest2.py，会抢串口）')

    keys = make_keys()
    hud = Hud()
    link = None
    try:
        keys.open()
        if not args.dry_run:
            link = make_link(args.port, args.baud, args.tx_log)
        core = RemoteCore(dry_run=args.dry_run,
                          pacer=StepPacer(interval=args.nudge_interval, mode=args.pace))
        atexit.register(keys.close)
        signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))   # kill 也走 finally
        reason = loop(core, keys, link, hud, args)
        hud.log(f'[遥控] 退出：{reason}。共 {core.steps} 步 ≈{core.steps * core.step_cm:.0f}cm'
                f'（真发 {core.frames} 帧，下位机确认 {core.pacer.confirmed} 步）')
    except KeyboardInterrupt:
        hud.log('\n[遥控] Ctrl-C 退出')
    finally:
        keys.close()
        hud.close()
        if link is not None:
            link.stop()


if __name__ == '__main__':
    main()

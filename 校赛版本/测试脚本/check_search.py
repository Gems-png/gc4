#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跑**真的 main() 主循环**的离线端到端测试：假相机 + 假串口，不碰任何设备。

为什么不只测 decide()（check_protocol.py 里的 gate）：那两条判据的**状态**是在主循环里
数的，不是 decide 里算的 —— 写错了 gate 照样全绿：

  「没圆就往前找」：连续 2 秒没圆的计时。车往前挪完那一步之后要是忘了重新数，
                    就会一发接一发连拱三下，一次 3cm，物料直接从画面里冲过去。
  「抓过的颜色不再抓」：黑名单什么时候加进去（真抓了才算）、什么时候清空
                    （到新的一站，不是"状态回到 0x01"—— 抓取动作本身也会离开 0x01
                    再回来，那会儿清空就正好把底下那个圆放出来，白改了）。

所以这里把假相机、假串口塞进真主循环让它自己跑，只看串口上到底写出了什么。
两个场景各起一个进程（主循环是 while True，没法从外面停，留在同一个进程里会跟
下一个场景抢 CPU，把时间量歪）。

跑：python3 check_search.py
"""
import contextlib
import io
import os
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault('OPENCV_LOG_LEVEL', 'ERROR')
import cv2
import numpy as np
import schooltest2 as st

RED = (0, 0, 220)        # BGR
BLUE = (220, 0, 0)
YELLOW = (0, 220, 220)

ok = True
def check(name, cond, extra=''):
    global ok
    ok = ok and cond
    print(f'{"PASS" if cond else "FAIL"}  {name}{"  " + extra if extra else ""}')


class ScriptCap:
    """假相机：按时间表吐画面。circles = [(起, 止, BGR 颜色, 圆心, 半径), ...]。

    没被任何一段盖住的时候是空画面（一个圆都没有）—— 那正是"往前找"要的场景。
    """

    def __init__(self, circles):
        self.w, self.h = 640, 480
        self.circles = circles
        self.t0 = time.time()
        self.props = {}

    def _frame(self):
        f = np.full((self.h, self.w, 3), 120, np.uint8)
        t = time.time() - self.t0
        for t0, t1, bgr, center, r in self.circles:
            if t0 <= t < t1:
                cv2.circle(f, center, r, bgr, -1, cv2.LINE_AA)
        return f

    def isOpened(self):
        return True

    # 控制读回来要能对上，否则 apply_ctrls 会以为"没设上"去打 v4l2-ctl 问 /dev/null
    def set(self, prop, val):
        self.props[prop] = val
        return True

    def get(self, prop):
        if prop in self.props:
            return self.props[prop]
        return {cv2.CAP_PROP_FRAME_WIDTH: self.w, cv2.CAP_PROP_FRAME_HEIGHT: self.h,
                cv2.CAP_PROP_FPS: 30.0,
                cv2.CAP_PROP_FOURCC: cv2.VideoWriter_fourcc(*'MJPG')}.get(prop, 0)

    def read(self):
        return True, self._frame()

    def release(self):
        pass


class ScriptLink:
    """假的下位机：按时间表报 0x20 状态，把真正写出去的指令记下来。

    states = [(起, 止, 状态), ...]，没被盖住的时候报 0x01（等待抓取）。
    收到微调之后按协议把上报切成 0x10 一小会儿再切回 —— 主循环"一发一发来"
    的节拍就靠这个回执，不模拟它的话量出来的发信频率是假的。
    """

    def __init__(self, states, t0=None):
        self.states = states
        self.t0 = t0 or time.time()
        self.sent = []               # [(时刻, 指令)]
        self.rx_count = {}           # 主循环的状态行要打它（收到过哪些类型的帧）
        self.out = ''                # capture=True 时主循环这 secs 秒里打出来的话
        self.lock = threading.Lock()
        self.moving_until = 0.0

    def start(self):
        pass

    def stop(self):
        pass

    def _state(self):
        if time.time() < self.moving_until:
            return st.STATE_MOVING
        t = time.time() - self.t0
        for t0, t1, s in self.states:
            if t0 <= t < t1:
                return s
        return st.STATE_WAIT_GRAB

    def fresh_state(self):
        return self._state(), True

    def send(self, frame, what, times=1):
        # 指令帧固定 5 字节：AA 55 21 01 CC，最后一个字节就是指令字
        if len(frame) == 5 and frame[0] == 0xAA and frame[1] == 0x55 and frame[2] == st.TYPE_CMD:
            with self.lock:
                self.sent.append((time.time() - self.t0, frame[4]))
                if frame[4] in st.ADJUST_CMDS:
                    self.moving_until = time.time() + 0.35   # 协议：微调期间报 0x10
        return True

    def at(self, cmd):
        with self.lock:
            return [t for t, c in self.sent if c == cmd]

    def show(self, secs):
        print(f'\n主循环跑了 {secs:.1f}s，串口上实际发出去 {len(self.sent)} 条：')
        for t, c in self.sent[:30]:
            print(f'  {t:5.1f}s  {st.CMD_CN.get(c, hex(c))}')
        if len(self.sent) > 30:
            print(f'  …（还有 {len(self.sent) - 30} 条，后面都是到点重复抓）')
        print()


def run_loop(circles, states, secs, extra_args=(), capture=False, tag_text=None):
    """把假相机/假串口塞进真 main()，跑 secs 秒，返回假串口。

    --dev 给 /dev/null：resolve_device 只认"这个路径存不存在"，真开相机那步被
    open_camera 顶掉了，所以不需要真的有摄像头，也不用手改 resolve_device。

    capture=True 时把这 secs 秒里主循环打出来的话留在 link.out 里（要断言终端说了
    什么的时候用 —— 比如"车跑到新的一站"那句清空提示）。

    tag_text 给了就**把 tag 那一路也开起来**（两路共用同一个假相机），并且把 QR 识别
    整个换掉（合成图上画不出二维码，真识别只会一直返回空）—— 用来测"按 tag 顺序抓"。
    """
    # 两个假的都在这里建：时间表用的 t0 于是基本同时开始（差的就是线程起来的几十毫秒，
    # 已经落在下面的容差里了），别一个在 main() 里建、一个在外面建
    cap = ScriptCap(circles)
    link = ScriptLink(states)
    st.McuLink = lambda *a, **k: link
    st.open_camera = lambda *a, **k: cap
    sys.argv = (['schooltest2.py', '--dev', '/dev/null',
                 '--tag-dev', '/dev/null' if tag_text else '', '--no-preview']
                + list(extra_args))
    saved_detect = st.detect_qrcode
    if tag_text is not None:
        st.detect_qrcode = lambda *a, **k: tag_text
    buf = io.StringIO()
    try:
        with (contextlib.redirect_stdout(buf) if capture else contextlib.nullcontext()):
            threading.Thread(target=st.main, daemon=True).start()
            time.sleep(secs)
    finally:
        st.detect_qrcode = saved_detect
    link.out = buf.getvalue()
    return link


# ==================== 场景 1：找不到圆就往前找 ====================
def scene_search():
    """前 9 秒画面里一个圆都没有（逼出"往前找"），之后放一个静止的红圆进去。

    预期：每 SEARCH_INTERVAL 秒往前一发（每发之后重新数满一个间隔），够 SEARCH_MAX
    发就闭嘴；圆一出现就回到正常流程（停稳 → 对准 → 抓），不是卡在"找"里出不来。

    时间线**从常量算出来**，别写死：SEARCH_MAX 是车上要调的量（用户从 3 调到过 6），
    写死的话常量一动这个场景就红 —— 而且是红在"发够了几发"这种没营养的地方。
    """
    FIND_AT = st.SEARCH_INTERVAL * (st.SEARCH_MAX + 0.5) + 1.5   # 凑够所有"往前找"再放圆
    SECS = FIND_AT + 4.5                                        # 留出"找到之后抓一次"
    link = run_loop([(FIND_AT, 99, RED, (320, 240), 60)], [], SECS)
    link.show(SECS)

    fwd = link.at(st.CMD_ADJUST_FWD)
    grabs = link.at(st.CMD_GRAB)

    check('一直没圆就会往前找（0x30）', len(fwd) > 0, f'→ 发了 {len(fwd)} 发')
    check(f'最多 {st.SEARCH_MAX} 发，不多不少', len(fwd) == st.SEARCH_MAX,
          f'→ 发了 {len(fwd)} 发（--search-max {st.SEARCH_MAX}）')
    if fwd:
        # 下界是"别一到站就拱"，上界是"也别拖太久"。实测第一发落在 2.5s 而不是 2.0s：
        # 主循环从起线程到第一帧要 ~0.5s（相机线程起来、曝光体检读 20 帧），
        # "多久没圆"是从**主循环看到第一帧**开始数的 —— 计时本身没错，那 0.5s 是启动开销
        check(f'第一发等满 {st.SEARCH_INTERVAL:.0f}s（不是一到站就往前拱，也没拖太久）',
              st.SEARCH_INTERVAL - 0.2 <= fwd[0] <= st.SEARCH_INTERVAL + 1.5,
              f'→ 第一发在 {fwd[0]:.1f}s')
        gaps = [b - a for a, b in zip(fwd, fwd[1:])]
        check('每发之间重新数满一个间隔（不是连发）',
              all(g >= st.SEARCH_INTERVAL - 0.2 for g in gaps),
              f'→ 间隔 {[round(g, 1) for g in gaps]}s')
        check('找的那些发都落在圆出现之前', fwd[-1] < FIND_AT,
              f'→ 最后一发 {fwd[-1]:.1f}s，圆 {FIND_AT:.0f}s 才出现')
    # 找的时候只能往前走：写成"没圆就左右乱扫"的话，一次扫几个 3cm 会偏出去
    others = [(t, c) for t, c in link.sent
              if c in st.ADJUST_CMDS and c != st.CMD_ADJUST_FWD]
    check('没圆的时候只往前走，不发左右/后退',
          all(t >= FIND_AT for t, c in others),
          f'→ 其它方向 {[(round(t, 1), st.CMD_CN.get(c)) for t, c in others] or "一条都没有"}')
    check('没圆的时候**没有盲抓**（抓都在圆出现之后）',
          all(t >= FIND_AT for t in grabs), f'→ 抓了 {len(grabs)} 次')
    check('圆出现后照样抓（没卡在"找"里出不来）', len(grabs) > 0,
          f'→ 抓了 {len(grabs)} 次')
    if grabs:
        check('抓发生在圆出现并停稳之后',
              grabs[0] >= FIND_AT + st.STILL_TIME - 0.2,
              f'→ 第一次抓在 {grabs[0]:.1f}s（圆 {FIND_AT:.0f}s 出现 + '
              f'停稳 {st.STILL_TIME:.1f}s）')


# ==================== 场景 2：抓过的颜色不再抓 ====================
def scene_color_skip():
    """物料底下压着一个同色的圆：抓走物料之后那个圆还在 → 不能再抓，也不能往前找。

    时间表：红(0~4) → 蓝(4~7.5) → 红(7.5~11.0，压着不抓) → 车开走 0x10(9.5~11.0)
            → 新的一站又是红(11.0~14)

    预期：红抓 1 次；蓝出现后改抓蓝；红再出现**不抓**（黑名单）且**不往前找**
    （它是"看到圆"，不是"没看到圆"）；车跑到新的一站后黑名单清空，红又能抓了。
    """
    SECS = 14.5
    RED_AGAIN = (7.5, 11.0)
    # 车自己开走那一段：**必须比"新的一站"那个间隔长**，不然不算新的一站
    # （那个间隔就是用来把"车开了好几秒"和"我们挪的那 0.4s"分开的）。
    # 车上那个常量（ALIGN_NEW_STOP_GAP）现在是几十秒 —— 拿真值来测的话这一段要开走
    # 半分钟，所以这里用 --align-new-stop-gap 把它临时压到 0.5s，1.5s 的开走照样算数。
    GAP = 0.5
    DRIVE = (9.5, 11.0)
    link = run_loop([(0.0, 4.0, RED, (320, 240), 60),
                     (4.0, 7.5, BLUE, (320, 240), 60),
                     (RED_AGAIN[0], 99, RED, (320, 240), 60)],
                    [(DRIVE[0], DRIVE[1], st.STATE_MOVING)], SECS, capture=True,
                    extra_args=('--align-new-stop-gap', str(GAP)))
    link.show(SECS)

    grabs = link.at(st.CMD_GRAB)
    fwd = link.at(st.CMD_ADJUST_FWD)

    def in_window(t, lo, hi):
        return lo - 0.2 <= t < hi

    red1 = [t for t in grabs if t < 4.0]
    blue = [t for t in grabs if in_window(t, 4.0, 7.5)]
    red_again = [t for t in grabs if in_window(t, RED_AGAIN[0], RED_AGAIN[1])]
    after = [t for t in grabs if t >= DRIVE[1] - 0.2]

    check('红圆停稳对准了 -> 抓', len(red1) > 0, f'→ {[round(t, 1) for t in red1]}s')
    check('抓过红色之后，红色不再抓（底下那个同色的圆不是物料）',
          len(red_again) == 0, f'→ 那 {RED_AGAIN[1] - RED_AGAIN[0]:.1f} 秒里抓了 {len(red_again)} 次')
    # 这条是这次改动的要害：压住抓取**不等于**"没看到圆"。判成没圆的话，
    # 「2 秒没圆就往前找」会接管，车对着一堆抓过色的圆往前拱三下，全白跑。
    check('压着不抓的那几秒里**也不往前找**（它是"看到圆"）',
          not [t for t in fwd if in_window(t, RED_AGAIN[0], RED_AGAIN[1])],
          f'→ 往前找发了 {[round(t, 1) for t in fwd]}s')
    check('画面里换了个没抓过的颜色（蓝）-> 改抓蓝',
          len(blue) > 0, f'→ 蓝那段里抓了 {len(blue)} 次')
    check('车开到新的一站（离开 0x01 超过 ALIGN_NEW_STOP_GAP）-> 黑名单清空，红又能抓',
          len(after) > 0, f'→ 开走之后抓了 {len(after)} 次')
    # 抓取过程自己**不会**触发清空：协议 §四 里三次抓取全程都是 0x01
    # （发 0x02 → 抓完回 0x01 → 再发 0x02，只有第 3 次抓完才报 0x10）。
    # 所以抓了就压住、一直压到车真开走 —— 总共就该只有红、蓝、开走后的红这三次
    check('整场只抓 3 次（红 / 蓝 / 开走之后的红），没有多余的抓',
          len(grabs) == 3, f'→ 抓了 {len(grabs)} 次：{[round(t, 1) for t in grabs]}')
    check('压住的时候终端说清了为什么（不是不吭声）',
          '已经抓过一次了' in link.out and '压着不抓' in link.out)
    check('开机说清楚用的是 once（不许重复抓）', '颜色重复抓：不许' in link.out)


# ==================== 场景 3：--color-policy repeat（同色的照抓）====================
def scene_color_repeat():
    """同一个画面，开关拨到 repeat：红圆抓过之后**照抓**（老行为）。

    和场景 2 前半段同一张时间表（红一直挂在那儿、不动），差别只有 --color-policy：
      once（默认）-> 红只抓 1 次，之后每一发都被压住
      repeat      -> 红照抓，每 GRAB_COOLDOWN 一发
    留这个后路是给"同一站又来了一个同色物料"用的（代价：底下那个同色定位圆也会
    被当成物料再抓一次）。这条同时钉住开关真的接到了主循环上。
    """
    SECS = 7.0
    link = run_loop([(0.0, 99, RED, (320, 240), 60)], [], SECS,
                    extra_args=('--color-policy', 'repeat'), capture=True)
    grabs = link.at(st.CMD_GRAB)

    check('repeat：同色的圆照抓（不是抓一次就压住）', len(grabs) >= 2,
          f'→ 抓了 {len(grabs)} 次：{[round(t, 1) for t in grabs]}')
    # 每一发之间还是要有 cooldown —— repeat 放开的是"颜色"，不是"频率"
    gaps = [b - a for a, b in zip(grabs, grabs[1:])]
    check(f'repeat 也得守 {st.GRAB_COOLDOWN:.1f}s 的抓取间隔',
          all(g >= st.GRAB_COOLDOWN - 0.2 for g in gaps), f'→ 间隔 {[round(g, 1) for g in gaps]}')
    check('repeat：不记黑名单，终端上不该出现"记下了"', '记下了' not in link.out,
          f'→ {"出现了" if "记下了" in link.out else "没有"}')
    check('repeat：开机会说清楚用的是哪一种', '颜色重复抓：允许' in link.out)


# ==================== 场景 4：按 tag 前三位数字的顺序抓 ====================
def scene_order():
    """tag 说先红后蓝，画面里**蓝的更大** —— 顺序得压过"谁大抓谁"。

    时间表：红(r=45)和蓝(r=80)一直在画面里不动，tag = '132+321+254+312'
    （第 1 组 132 = 红1 蓝3 黄2，第 2 组是放置位置，不管）。

    预期：第 1 个抓红（不是更大的蓝），第 2 个抓蓝，然后顺序里剩黄 —— 黄不在画面里，
    等满 --order-miss-wait 之后按 biggest 兜底也轮不到它（红蓝都抓过了，画面里剩下的
    就是底下压着的那两个同色定位圆），所以**总共只抓 2 次**，不空抓。
    """
    SECS = 7.0
    link = run_loop([(0.0, 99, RED, (240, 240), 45),
                     (0.0, 99, BLUE, (430, 240), 80)],
                    [], SECS, capture=True, tag_text='132+321+254+312',
                    # 对准那套在这儿只是噪声（假画面的圆不会因为发了微调就动），关掉
                    # 就一看到圆停稳直接抓，量出来的顺序干净
                    extra_args=('--no-align-enable',))
    link.show(SECS)

    grabs = link.at(st.CMD_GRAB)
    out = link.out
    # 主循环自己打的那两行：谁先谁后一目了然（0x02 帧本身只有指令字，看不出颜色）
    said = [ln for ln in out.splitlines() if '[顺序] 抓走一个' in ln]

    check('认到 tag 就把顺序排出来（前三位 132 = 红 → 蓝 → 黄）',
          '按这个顺序抓：红(1) → 蓝(3) → 黄(2)' in out)
    check('开机说清楚是按 tag 顺序抓', '按 tag 第 1 组三位数字的顺序抓' in out)
    check('抓了 2 次（顺序里的黄不在画面里，按 biggest 兜底也没得抓 -> 不空抓）',
          len(grabs) == 2, f'→ 抓了 {len(grabs)} 次：{[round(t, 1) for t in grabs]}')
    check('第 1 个抓的是红（顺序压过"谁大抓谁"：蓝的半径 80 比红的 45 大得多）',
          len(said) >= 1 and '顺序里的红色' in said[0],
          f'→ {said[0] if said else "一句话都没说"}')
    check('第 2 个抓的是蓝（顺序往前挪了一位）',
          len(said) >= 2 and '顺序里的蓝色' in said[1],
          f'→ {said[1] if len(said) > 1 else "没有第二句"}')
    check('顺序里剩下的黄没抓到 -> 说清了为什么（不是不吭声压住）',
          '画面里没有黄色的圆' in out)
    check('没按顺序抓的时候（这一步该抓黄）不发 0x02', len(grabs) == 2,
          f'→ 0x02 一共 {len(grabs)} 发')


# ========= 场景 5：顺序里该抓的颜色压根不在画面里（--order-miss-policy biggest）======
def scene_order_miss():
    """tag 说先红，画面里**只有蓝**（红认不出来 / 没进画面）：等满 --order-miss-wait
    就照抓最大的那个 —— 宁可抓偏也不整轮卡死（跟"微调发满就照抓"一个口径）。

    预期：0.8s 那会儿不抓（还压着），等到 ~2s（等满 1.5s）抓蓝，并且**说出来**它抓的
    不是顺序里的颜色。之后顺序里剩蓝、黄，蓝已经抓过 -> 接着压住，总共就抓这一发。
    """
    SECS = 5.0
    link = run_loop([(0.0, 99, BLUE, (430, 240), 80)], [], SECS, capture=True,
                    tag_text='132+321+254+312',
                    extra_args=('--no-align-enable',))
    grabs = link.at(st.CMD_GRAB)
    out = link.out

    check('该抓的颜色没看到时先压住（不是一发现没有就乱抓）',
          len(grabs) <= 1 and (not grabs or grabs[0] >= st.TAG_ORDER_MISS_WAIT),
          f'→ 第一发在 {[round(t, 1) for t in grabs]}s'
          f'（--order-miss-wait {st.TAG_ORDER_MISS_WAIT}）')
    check('压住的时候终端说清了：没有红色 + 顺序是什么', '画面里没有红色的圆' in out)
    check('等满 --order-miss-wait 之后照抓（biggest 兜底，不是一直卡着）',
          len(grabs) == 1, f'→ 抓了 {len(grabs)} 次')
    check('兜底抓的时候**明说**抓的不是顺序里的颜色',
          '不是顺序里的颜色' in out and '顺序要的是红色' in out)
    check('抓完之后顺序往前挪了（剩蓝→黄），画面里剩下的不再抓', len(grabs) == 1,
          f'→ 0x02 一共 {len(grabs)} 发')


# ========= 场景 6：顺序里三个都抓完了就不再多抓 ==========================
def scene_order_done():
    """红蓝黄都在画面里、tag 说 红→蓝→黄：三个按顺序抓完，之后**一个都不再抓**。

    转盘上就三个物料，抓完还留在画面里的（定位圆、下一批的）不能当第四个抓走 ——
    "顺序抓完"是比颜色黑名单更硬的一道闸。等到车跑到新的一站才从头来。
    """
    SECS = 7.0
    link = run_loop([(0.0, 99, RED, (150, 240), 45),
                     (0.0, 99, BLUE, (320, 240), 45),
                     (0.0, 99, YELLOW, (490, 240), 45)],
                    [], SECS, capture=True, tag_text='132+321+254+312',
                    extra_args=('--no-align-enable',))
    link.show(SECS)

    grabs = link.at(st.CMD_GRAB)
    said = [ln for ln in link.out.splitlines() if '[顺序] 抓走一个' in ln]

    check('三个都抓了，而且**按 tag 的顺序**（红 → 蓝 → 黄，跟画面里的位置无关）',
          len(said) == 3
          and '顺序里的红色' in said[0]
          and '顺序里的蓝色' in said[1]
          and '顺序里的黄色' in said[2],
          f'→ {[ln.split("：")[1] for ln in said]}')
    check('抓满三个就停手（不抓第四个）', len(grabs) == 3,
          f'→ 抓了 {len(grabs)} 次：{[round(t, 1) for t in grabs]}')
    check('停手的原因打出来了（不是不吭声）', '都抓完了' in link.out)


SCENES = {'search': scene_search, 'color': scene_color_skip,
          'repeat': scene_color_repeat, 'order': scene_order,
          'miss': scene_order_miss, 'done': scene_order_done}


def main():
    # 主循环是 while True，停不下来 —— 一个场景一个进程，跑完各自退出，别互相抢 CPU
    if len(sys.argv) > 1 and sys.argv[1] in SCENES:
        SCENES[sys.argv[1]]()
        print('\n' + ('全部通过' if ok else '有失败项'))
        sys.stdout.flush()
        # 直接退：daemon 线程里还压着 cv2 的取流，正常退会在解释器清理时 abort
        # （"terminate called without an active exception" + core dump），退出码就不是 0/1 了
        os._exit(0 if ok else 1)

    results = {}
    for name in SCENES:
        print(f'===== 场景 {name} =====')
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

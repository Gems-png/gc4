#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""校赛测试脚本 2：tag 识别 + 物料（找圆）识别 + 0x20/0x21 握手

基于校赛版本的 serial_tag.py，多了四件事：

1. **两路相机**：tag 和物料各一路，分别开、分别识别。
       tag 相机     -> QR 识别（车到停顿点之前就要认出来）
       物料相机     -> 找圆（对着转盘上的物料）
   两路的设备、分辨率、帧率都能单独给，互不影响。哪路挂了另一路照跑。
   派上现在：物料 = USB "2M"（/dev/video0），tag = USB "Integrated Webcam"（/dev/video2）。

2. 设备**不写死 /dev/videoN**。N 是 USB 枚举顺序，插拔一次、上电顺序变一下就会漂
   （这里就踩过：只有一台相机时 Integrated Webcam 是 video0，插上第二台它变成 video2）。
   所以默认按**板卡名**匹配：'name:2M'、'name:Integrated'，名字是摄像头自己报的，不漂。
   也能直接给路径或 /dev/v4l/by-id 里的名字。--list 会把现在的节点和名字都打出来。
   UVC 摄像头一般 index0 = 取流节点、index1 = metadata，别开错。
   注意 /dev/video19 挂在 rpivid 下面是硬件解码器，不是相机。
   片段短了会**同时命中好几块板子**（笔记本上 "Integrated" 一下命中内置摄像头、
   内置红外、和刚插的 USB 摄像头），这时脚本**不开**，把候选板卡名和能直接抄的
   写法打出来让你挑 —— 随便挑一个的话开起来的多半是笔记本自己那个，而且它的
   取流节点根本不出帧，表现成"相机明明插着却没画面"（2026-09-18 就是这么踩的）。
   **相机没起来不退出**：每 CAM_RETRY_INTERVAL 秒重新解析 + 重开一次，每轮都打
   一遍感叹号框住的提醒（跟串口没连上那条一个口径），状态行里也一直挂着
   "**没开起来**"。插晚了、换 USB 口、节点号漂了都会自己接上，不用重启脚本。

3. 物料识别：**看图里的圆**。物料是回转体，投影是圆（斜看是椭圆），
   所以用 HoughCircles 找圆，检出圆的就算物料 —— **不需要特定 HSV**，认出圆就发。
   颜色**不参与判断**，只是附带的信息：把圆内占比最高的那种颜色打出来，
   让你知道抓的是哪一块；认不出颜色（比如黑物料、光照太偏）也照样认，不影响。
   每行还会打**整个圆内的平均 HSV**（跟颜色阈值表无关），就是给你照着调阈值的。
   颜色编号：红1 黄2 蓝3 绿4 黑5 浅蓝6，规则里每轮抽签只用其中三种。
   半径范围跟分辨率绑死，换分辨率要重标（启动时会打印实际用的像素值）。

4. 和电控的握手（见 USART1_Cmd_Protocol.md）—— 这就是"什么时候能发"：
       0x20 指令状态(下→上, 1B, 每 100ms 上报一次)
           0x00 等待二维码     → 上位机可以发 0x21 cmd=0x01(识别完毕,继续行进)
           0x01 等待抓取指令   → 下位机停下来在等抓取。识别到圆 → **先对准**
                                （见第 5 条）→ 对准了发 0x21 cmd=0x02(抓取一次)。
                                另外还要**等圆停稳**：物料停下才好抓，所以圆连续
                                --still-time 秒不动（--still-tol 像素内）才发，
                                还在动就等（终端会打"圆稳 x/y 秒"）。
                                想"一看到就动"就 --still-time 0（微调和对准也
                                跟着这个门槛走）。
           0x10 运动中         → 不要发指令打扰，必须发就发 0x00(空指令,兼心跳)
       0x21 指令下发(上→下, 1B)  帧长固定 5 字节
   tag 不受状态影响，识别到就发（burst 5 次，和校赛版一样）；
   指令是**动作**，一次只发一发，绝不 burst（连发 5 次 = 抓 5 次）。

5. 抓之前先对准：圆停稳了但没在**对准点**上（默认画面正中，--aim-x/--aim-y 挪），
   就发底盘微调指令把车挪过去 —— 0x21 的指令字多四个：0x30 前 / 0x31 后 /
   0x32 左 / 0x33 右（和 00/01/02 共用第 5 个字节，帧长一样是 5 字节）。
   圆心**横竖各自进了自己的容差**才算对准（--aim-tol-x 左右 / --aim-tol-y 前后，
   两个轴分开设），这时才允许发抓取。谁超了自己的容差就修谁；两个都超就修误差大的那个。
   **微调只在 0x20 状态 = 0x01 时发**：那是下位机停下来等指令的时刻，
   0x00 还在等二维码、0x10 正在按轨迹走，都不该去挪底盘。
   一次只发一条（前后+左右同时发会斜着走），而且**发完要等它回到 0x01 才发下一发**：
   下位机挪的时候把上报切成 0x10，那期间发出去的微调会被**静默丢弃**（不计错、
   不缓存），所以"发完 sleep 再发下一条"会少走几厘米而且没有任何报错。
   节拍由下位机的 0x01 给，别自己定（见 USART1_Nudge_Protocol.md §四/§五）。
   一次停稳里最多发 --align-max 条（0=不限）——对不上就别一直挪，
   免得车照着个假圆一路挪出去。**发满了不再挪，但照样抓**：
   抓偏一点总比整轮卡在这儿不抓好（原因会打在终端上）。

   方向约定：画面**摆正之后**（见 --cam-rot），画面上方 = 车头前方。
   圆心偏画面右 → 物料在车右边 → 发右(0x33)；偏下 → 物料比对准点离车近 →
   发后(0x31)。这跟相机是朝前看还是朝下看无关，只要求画面摆正、上方朝车头。
   拿不准就先看预览：物料窗口里画了黄色十字(对准点)，还有一条从十字指向物料圆心
   的箭头（= 车要往哪边挪），右下角写着这一刻要发哪个方向。
   箭头方向跟实际挪的方向反了，就是朝向没摆正/装反了，改 --cam-rot。

   相机装歪了就靠 --cam-rot 摆正：0=正着装，1=相机逆时针转了90°（**默认**，
   车上现在就是这么装的），2=转180°，3=顺时针90°。转的是**相机怎么转的**，
   脚本会把画面反着转回来，所以预览里看到的就已经是摆正的画面
   （识别、对准全在摆正后的画面上做）。tag 那一路相机单独一个开关 --tag-cam-rot。

6. 状态是 0x01 但**一个圆都看不到**（车停偏了、物料没进画面）：连续
   --search-interval 秒（默认 2s）没圆，就往前拱一小步找找 —— 还是 3 cm 一步、
   走同一条 0x30 通道，所以节拍照样听下位机的 0x01，不能连发；一次停稳里最多
   --search-max 发（默认 3），这几发也占 --align-max 的总预算。
   找满还是没圆就**不再往前、也不盲抓**，停着等（终端会打为什么）——
   没圆就不知道物料在哪，盲抓既可能空抓也可能撞到东西。跟着 --align-enable 走：
   那个总开关关着就一条都不发（也不往前找）。

7. 同一个颜色能不能重复抓（--color-policy，默认 once，启动时选）：
   **once**（默认）= 抓过的颜色不再抓。物料底下压着一个**同色**的圆（定位圆/靶心），
   物料被抓走以后那个圆还在画面里，看着还是个同色的圆 —— 再抓一次就是空抓。
   所以真发过一次抓取就把那个颜色记下来，之后同色的圆**不再抓**；但**照样算"看到圆"**
   （停稳、对准照走，车不会因为滤掉一个圆去触发第 6 条的往前找），只是最后压住不发 0x02。
   画面里还有别的没抓过的颜色时自动改抓那个（谁大抓谁）。**车自己跑到新的一次0x00**
   就清空重来（NudgePacer 的 'station' 事件：离开 0x01 超过 ALIGN_NEW_STOP_GAP 才
   回来 = 车真开走了一段路）。不能用"回到 0x01"当换站信号 —— 虽然协议 §四 里三次
   抓取全程都是 0x01（抓取本身不触发 'station'），但下一站不一定停在二维码点，
   可能直接就是下一个抓取点，等 0x00 再清就晚了，同一色的物料会抓不着。
   **repeat** = 同色的圆照抓（老行为）：同一站又来了一个同色物料时用得上，
   代价是底下那个同色定位圆也可能被当成物料再抓一次。

8. **按 tag 前三位数字的顺序抓**（--tag-order，默认开）：规则里二维码是 4 组三位数、
   用 + 连接，第 1 组就是第一批物料的**搬运颜色和顺序**（颜色编号 红1 黄2 蓝3 绿4 黑5
   浅蓝6）。例：452+321+254+312 的前三位 = 4、5、2 → 先抓绿、再抓黑、最后抓黄。
   认到 tag 就把顺序排出来，之后每一帧挑**队首那个颜色**的圆：同色有好几个就抓最大的
   那个（跟"谁大抓谁"一个口径）；**真发了 0x02 才**往前挪一位（干跑也一样，状态机
   走的路要和线上一样）。三个都抓完 = 这一批完了，画面里剩下的**不再抓**（多半是定位圆
   或者下一批），等车跑到新的一站或者重新认到 tag 才从头来。
   认不到 tag / --no-tag-order 就退回老的"谁大抓谁"。
   该抓的颜色**画面里没有**（认不出颜色、被挡住、转盘没转到位）时看 --order-miss-policy：
   **biggest**（默认）先等 --order-miss-wait 秒（默认 1.5s），还没有就抓画面里最大的那个
   没抓过的 —— 跟"微调发满就照抓"一个口径，宁可抓偏也不整轮卡死不抓，原因会打在终端上；
   **wait** 就一直压着等（严格按顺序，代价是颜色认不出来就整轮卡住）。
   压住不发 0x02 的时候停稳/对准**照走**，所以那不算"没看到圆"，不会去触发第 6 条的往前找。
   跟第 7 条一起用时以顺序为准：顺序要的颜色永远是首选，抓过的颜色只在"顺序里没有它"
   的时候才被滤掉（tag 里同一个颜色出现两次会打警告 —— 同一批两个同色物料分不出来，
   第二个会被压住不抓）。
   "车跑到新的一站"由 --align-new-stop-gap 判定，微调次数、抓过的颜色、tag 顺序
   都在这一个事件上重置 —— 那个间隔要卡在"抓取动作报的 0x10"和"真开走一段路"之间。

用法：
    python3 schooltest2.py --list                  # 看这台机器上有哪些摄像头
    python3 schooltest2.py                         # 默认两路都开，真发给下位机
    python3 schooltest2.py --dev 'name:2M' --tag-dev 'name:Integrated'
    python3 schooltest2.py --dev /dev/video0       # 也可以直接给节点
    python3 schooltest2.py --tag-dev ''            # 只开物料相机
    python3 schooltest2.py --dev '' --tag-dev 'name:Integrated'   # 只开 tag 相机
    python3 schooltest2.py --width 1280 --height 720 --area 6000
    python3 schooltest2.py --rx-log                # 把下位机发来的每一帧都打出来
    python3 schooltest2.py --dry-run               # 只打印不发给下位机（默认是真发）
"""

import argparse
import os
import sys
import threading
import time
import traceback
from collections import namedtuple

os.environ.setdefault('OPENCV_LOG_LEVEL', 'ERROR')

import cv2
import numpy as np
import serial

# ---------------- 相机默认值 ----------------
# 'name:片段' = 按**板卡名**找（就是 v4l2-ctl 打的那一列）。
# 板卡名是摄像头自己报的，不随 USB 枚举顺序变，比写死 /dev/videoN 稳得多。
#   物料相机 = USB 的 "2M"          （派上现在是 /dev/video0）
#   tag 相机 = USB 的 "Integrated Webcam"（派上现在是 /dev/video2）
# 也可以直接给设备路径 /dev/videoN，或给 /dev/v4l/by-id 里的名字。
# 注意：/dev/video19 挂在 rpivid 下面是硬件解码器，不是相机。
MATERIAL_DEV = 'name:2M'
TAG_DEV = 'name:WebCam'
VIDEO_INDEX = 0        # UVC 一般 index0 = 取流节点，index1 是 metadata

V4L_BY_ID = '/dev/v4l/by-id'
V4L_BY_PATH = '/dev/v4l/by-path'
V4L_SYSFS = '/sys/class/video4linux'

# 相机没找到 / 打不开的时候：**不退出**，每这么多秒重来一轮，每轮都打一遍显著提醒。
# 跟串口没连上那条一模一样（见 McuLink._loop）—— 插晚了、换了个 USB 口、节点号变了，
# 都能自己接上，不用重启脚本。**相机的事只报一次是不够的**：日志一滚、窗口一黑，
# 人就对着屏幕猜（2026-09-18 的内置摄像头被当成 tag 相机开了起来，就是这么看走眼的）。
CAM_RETRY_INTERVAL = 5.0
CAM_READ_FAIL_MAX = 100  # 连续这么多帧读不到就判定"这一路掉了"（约 1s）→ 重开、重解析节点。
                         # 拔了/被别的程序抢了会走到这儿；重插上就自己回来

CAMERA_FOURCC = 'MJPG'  # 不压缩(YUYV)在 640x480 就只有 ~5-10fps，MJPG 才能跑满

# ---- 物料相机(M2)的曝光：默认就调，只调这一路 ----
# 为什么非调不可：这颗 2M 模组出厂是 auto_exposure=3(Aperture Priority)，不管它就会冲爆，
# 黄色物料糊成白块，HSV 判据直接失效（"曝光太厉害"就是这个）。
#
# 实测（这台机器的 /dev/video4，640x480 MJPG）——**这颗模组只有一个真旋钮：brightness**：
#   brightness 单调、线性得漂亮，1 格 ≈ 1 个 V。同一个桌面同一天，hue=0：
#        -32 → V中位59  过曝 0.7%  压黑40%   红盘 S 194
#        -16 → V中位76  过曝 3.8%  压黑31%   红盘 S 180
#          0 → V中位93  过曝28.5%  压黑18%   红盘 S 164   ← 原来填的就是这个
#        +32 → V中位128 过曝38.5%  压黑 0%   红盘 S 131
#        +64 → V中位162 过曝41.3%  压黑 0%   红盘 S  99
#     **看最后那列**：越亮饱和度越低。HSV_RANGES 里每种颜色都有 S 下限，画面一冲爆，
#     物料的 S 就掉到门槛以下 —— 这就是"曝光不对"表现成"HSV 判据失效"的原因，
#     压亮度能同时把过曝和掉饱和度一起治好。
#   auto_exposure 是**死控制**：1(手动)/3(自动) 反复切三轮，V中位
#        107→95→95→94→94→94，之后就一点不动。写 manual 并不像原来注释说的那样
#        "不让它自己发挥"，该多亮还是多亮。照写（别的模组上是对的），但别指望它。
#   exposure_time_absolute 也是**死控制**：78/312/1250/5000 换着写、重开设备再写，
#        V中位一个像素都不变。设了能读回来，就是没接上。
#   hue 是**活的**，出厂 0；这台机器上被探控件时留在了 2000(顶格)，画面整个转色
#        （红盘变橙、蓝杯变紫、绿垫变青，存图对比过）。脚本以前从不设 hue ——
#        在 PC 上调好的 HSV 表搬到车上（那边是出厂 0）颜色根本对不上。所以现在
#        **开机就把 hue 钉到 MAT_HUE**，两台机器看到一样的颜色，表才搬得动。
#
# 结论：嫌亮嫌暗就改 MAT_BRIGHTNESS（唯一真旋钮）。开机后脚本会打一行"曝光体检"，
# 照着那行的过曝% 调，别靠眼睛。tag 相机（Integrated）不用调，保持它自己的默认。
MAT_AUTO_EXPOSURE = 'manual'  # 'manual'=设成手动 / 'auto'=设成自动 / None=不碰。
                              # 注：这颗模组上两种一个样（见上面实测），留着是给别款相机用的
MAT_HUE = 0                   # 色相，-2000~2000，出厂 0。钉死到 0 是为了 PC 和车上
                              # 看到同一个颜色（--hue）。**在 PC 上调 HSV 表之前先确认这行是 0**
MAT_BRIGHTNESS = -48          # 亮度，-64~64，驱动默认 0。**唯一真旋钮**，1 格 ≈ 1 个 V（--brightness）
MAT_GAIN = None               # 增益。这颗模组没暴露 gain 控制，填了也是静默失败（会打"没设上"）
MAT_EXPOSURE = None           # 曝光(绝对)。**在这颗模组上是死的**，填了没用，留着是为了别款相机

# ---------------- 串口 / 协议 ----------------
PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
FRAME_HEADER = b'\xAA\x55'
TYPE_TAG = 0x03
TAG_DATA_LEN = 12

# 握手：USART1_Cmd_Protocol.md（微调那组在 USART1_Nudge_Protocol.md）
# 注：0x01 在微调协议/下位机代码里也叫"等待颜色信息"，同一个状态两种叫法。
TYPE_STATE = 0x20      # 下→上，指令状态
TYPE_CMD = 0x21        # 上→下，指令
CMD_LEN = 1
STATE_WAIT_QR = 0x00       # 等待二维码
STATE_WAIT_GRAB = 0x01     # 等待抓取指令（下位机停下来在等 0x02）
STATE_MOVING = 0x10        # 运动中
CMD_IDLE = 0x00            # 空指令 / 心跳填充（不动作）
CMD_QR_DONE = 0x01         # 二维码识别完毕，继续行进
CMD_GRAB = 0x02            # 抓取一次
# 底盘微调。**和 00/01/02 共用 0x21 那一个指令字**（帧长一样是 5 字节
# AA 55 21 01 xx），不是新的类型。只在 0x20 状态 = 0x01 时才发（见 decide）。
# 协议以 USART1_Nudge_Protocol.md / 电控代码为准（那份是自包含的，能单独发给对接的人）：
#   - 一发 = 朝该方向走 **3 cm** 后自己停，上位机**不传距离**，想走 9cm 就发三次；
#   - 微调期间下位机把上报切成 0x10，走完（约 0.3s）自动切回 0x01 —— **看到 0x01
#     = 上一发走完了**；没回到 0x01 就发的会被**静默丢弃**（不计错、不缓存）。
#     所以节拍不能按定时器硬发，详见 decide 里 ALIGN_INTERVAL 那段。
CMD_ADJUST_FWD = 0x30      # 底盘微调：前
CMD_ADJUST_BACK = 0x31     # 后
CMD_ADJUST_LEFT = 0x32     # 左
CMD_ADJUST_RIGHT = 0x33    # 右
ADJUST_CMDS = (CMD_ADJUST_FWD, CMD_ADJUST_BACK, CMD_ADJUST_LEFT, CMD_ADJUST_RIGHT)
LEGAL_CMDS = (CMD_IDLE, CMD_QR_DONE, CMD_GRAB) + ADJUST_CMDS   # 0x21 指令字的白名单

STATE_TIMEOUT = 0.5        # 下位机 100ms 一发，超过这么久没收到就当状态未知：不发指令
GRAB_COOLDOWN = 1.5        # 同一次"等待抓取指令"里，两次"抓取一次"至少隔这么久

# ---------------- 相机朝向 / 对准（微调） / 停稳 ----------------
# 这块是"把物料挪到该在的位置"的全部可调值。车上装好了就不用动；
# 相机重新装过、或者换了个抓取位置，就改这里（命令行也能覆盖，但默认走这里）。

# 相机是"怎么转着装的"，脚本会把画面反着转回来，所以预览里已经是摆正的画面。
#   0 = 正着装
#   1 = 相机**逆时针转了 90°**（画面里东西是顺时针躺着的）
#   2 = 转了 180°（画面上下颠倒）
#   3 = 顺时针转了 90°
# 转回来之后才做识别 / 画预览 / 算方向，所以下面的方向约定不用管相机怎么装的。
CAM_ROT = 0               # 物料相机（--cam-rot）
# 底盘微调（0x30~0x33）总开关。车上默认**开着** —— 微调已经是默认流程的一部分
# （协议见 USART1_Nudge_Protocol.md）：
#   True  = 误差超过各自的容差（AIM_TOL_X / AIM_TOL_Y）就发微调，挪进去再抓取
#           （一次一发，等它回 0x01）
#   False = 只对准不动车 —— 圆没对准也**不发微调**，按"还差多少像素"报出来，照抓
# 想临时不挪车（比如下位机那套还没烧上），命令行 --no-align-enable 就行，不用改这里。
ALIGN_ENABLE = True       # （--align-enable / --no-align-enable）
TAG_CAM_ROT = 0            # tag 相机，单独一个（--tag-cam-rot）
CAM_ROT_CN = {0: '正装(画面不转)', 1: '相机逆时针转了90°', 2: '相机转了180°',
              3: '相机顺时针转了90°'}

# 对准点 = 想让物料停在画面的哪个位置。两个轴**各自独立**，都是：
#   -1 = 自动取这一轴的正中（640x480 就是 x=320 / y=240）
#   给了像素值就用给的 —— 想让准星**往下**移，就是把 AIM_Y 往**大**改。
# 相机装的位置和机械臂的抓取点对不上时，量一下差多少像素挪这里（预览里画着准星）。
AIM_X = 290                 # （--aim-x）
AIM_Y = 250                # （--aim-y）280 = 正中(240)再往下 40px。
                           # 每 +10 准星往下 10px；想回正中写 -1
# 对准容差，**左右和前后分开**（两个轴各管各的）：
#   AIM_TOL_X = 左右的容差（画面横轴，超了就发 0x32 左 / 0x33 右）
#   AIM_TOL_Y = 前后的容差（画面纵轴，超了就发 0x30 前 / 0x31 后）
# 谁超了自己的容差就修谁；两个都超就修误差大的那个；都在容差里才算对准了。
# 左右可以放得比前后松（横移对抓取的影响小、而且横移那一路下位机不做航向修正），
# 想收紧哪个就改哪个，另一个不受影响。
#
# 但任何一个都**必须 ≥ 半步，不然那个方向一定来回动**：
#   底盘一步是下位机固定的 3 cm（协议里上位机不传距离），这一步在画面里跨 S 个像素。
#   容差 T < S/2 时：差一点没进容差 → 挪一步 → 冲过头到另一边 → 再挪回来 → 又冲过头……
#   永远在容差两侧来回。收敛条件是 T ≥ S/2（半步 = 这套机械的精度上限，≈1.5 cm）。
# S 是量出来的，不用猜：微调走完一步时终端会打
#   「走完一步：x 方向差 +190 → +12px（3cm ≈ 178px）」
# 括号里就是 S，日志还会直接告诉你该填多少（S/2 再多一点）。
# **两个方向的 S 不一定一样**（横移和前后走的距离标定各是各的），各量各的。
AIM_TOL_X = 40             # （--aim-tol-x）左右容差，px
AIM_TOL_Y = 25             # （--aim-tol-y）前后容差，px

# 方向约定：画面**摆正之后**，画面上方 = 车头前方。
#   圆心偏画面右 → 物料在车的右边 → 发右(0x33)，把车挪过去
#   圆心偏画面下 → 物料比对准点离车更近 → 发后(0x31)
# 跟相机朝前看还是朝下看无关，只要求画面摆正、上方朝车头。
# 反了的话先在预览里看那条黄色箭头指的方向对不对，再回头查 CAM_ROT。
ALIGN_INTERVAL = 0.6       # 两条微调指令**至少**隔这么久（--align-interval）。
                           # 注意它**不是节拍** —— 节拍由下位机的 0x01 给
                           # （见下面的 ALIGN_RETURN_TIMEOUT 和 NudgePacer）；
                           # 它只是个兜底的最小间隔，防止状态抖动时连发。
                           # 离线干跑（--dry-run / 串口没连上）时没有状态可等，
                           # 它就退化成唯一的节拍 —— 和以前的定时器行为一样
ALIGN_RETURN_TIMEOUT = 3.0 # 发完一发微调后，最多等这么久让它回到 0x01
                           # （--align-return-timeout）。协议里一发往返只要
                           # 0.4~0.5 s，3 s 很宽裕。超时 = 这一发压根没生效
                           # （被门拒了 / 帧丢了），这一站就不再挪车 ——
                           # 不然会一直对着一个等不到的 0x01 空转
ALIGN_NEW_STOP_GAP = 2.0   # 离开 0x01 超过这么久才回来 = 车自己跑到新的一站了
                           # （不是我们挪的那一下）。一发微调只让车走 0.3 s、
                           # 0x10 也就报 0.4~0.5 s，真跑一段路是好几秒 ——
                           # 用这个间隔把两者分开，好决定微调次数从哪重新算
ALIGN_MAX = 12             # 一站（一次停稳）里最多发几条微调（--align-max，0=不限）。
                           # 用满之后**不再挪车、照抓**：抓偏一点总比整轮卡在这儿不抓好。
                           # 想让它对不上就一直挪，填 0 —— 但认成假圆时车会照着它
                           # 一路挪出去，所以默认给个上限
# 「找不到圆就往前找」：车停偏了、物料压根没进画面时的补救。
# 等待抓取状态里连续 SEARCH_INTERVAL 秒一个圆都没有，就往前拱一小步（还是 3 cm 一步，
# 和微调同一条 0x30 通道），一站最多 SEARCH_MAX 发。往前找的这几发**也算进 --align-max
# 的总预算**（同一套节拍、同一个 pacer），所以别把两个上限都顶满。
# 找满 SEARCH_MAX 次还是没圆：**不再往前，也不盲抓**，就那么停着等（终端会说为什么）——
# 没有圆就不知道物料在哪，盲抓一下既可能空抓也可能撞到东西。想改成照抓说一声。
# 往前找也是在挪车，所以跟着 ALIGN_ENABLE 那个总开关走：关着就一条都不发。
SEARCH_INTERVAL = 2.0      # 连续这么久没看到圆就往前找一次（--search-interval）
SEARCH_MAX = 6             # 一站最多往前找几次（--search-max，0=不限）
# 圆停稳判定：物料停下来才好抓，所以"等待抓取指令"时还要等圆在画面里不动了才发 0x02。
# 判据是最大的那个圆的圆心/半径连续 STILL_TIME 秒没超出容差（帧间抖动几个像素是正常的）。
# 微调也卡在这个门槛上 —— 挪完一步画面会动，所以每条微调之间自然隔开一段。
STILL_TIME = 0.3           # 要连续静止多少秒才算停稳（--still-time）
STILL_TOL = 6              # 圆心挪了多少像素就算还在动（--still-tol）
STILL_R_FRAC = 0.10        # 半径变化超过这个比例也算还在动
# 「同一个颜色能不能重复抓」——**启动时就选**（--color-policy，默认 once）。
# 这个是拿来防**空抓**的：物料底下压着一个**同色**的圆（定位圆/靶心），物料被抓走以后
# 那个圆还在画面里，看着还是一个同色的圆 —— 再抓一次就是空抓（用户 2026-09-18 报的现象）。
#   'once'   一个颜色在一站里抓过一次就不再抓。同色的圆**不再抓**，但照样算"看到圆"：
#            停稳、对准照走（车不会因为滤掉一个圆就往前拱），只是最后压住不发 0x02。
#            拉黑之后画面里又有别的没抓过的颜色时，会自动改抓那个（谁大抓谁，见 pick_target）。
#   'repeat' 同色的圆照抓（老行为）。同一站**又来了一个同色物料**时用得着 ——
#            代价是底下那个同色定位圆也可能被当成物料，再抓一次（空抓）。
# 清空时机：**车自己跑到新的一站**（NudgePacer 的 'station' 事件 = 离开 0x01 超过
# ALIGN_NEW_STOP_GAP 才回来，见主循环里 ev == 'station' 那一段）。
# 为什么用这个而不是"回到 0x01"：协议 §四里三次抓取**全程都是 0x01**
# （发 0x02 → 抓完回 0x01 → 再发 0x02，只有第 3 次抓完才报 0x10 开走），
# 所以抓取本身不会触发 'station'；能触发的只有"车真的开走了一段路"。
# 也不能等到报 0x00（等待二维码）才清：下一站**不一定**是二维码点，可能直接就是
# 下一个抓取点，那会儿还压着黑名单，同一色的物料就抓不着了（用户 2026-09-18 要的
# 就是"到新的抓取点就清"）。
COLOR_POLICY = 'once'      # （--color-policy once / repeat）
                           # once   = 抓过的颜色不再抓（默认，防空抓）
                           # repeat = 同色的圆照抓（老的"谁大抓谁"）

# 「按 tag 前三位数字的顺序抓」—— 规则：二维码是 4 组三位数、用 + 连接，
# 第 1 组 = 第一批物料的**搬运颜色和顺序**，第 2 组是第一批的放置位置，
# 第 3、4 组是第二批的（颜色编号：红1 黄2 蓝3 绿4 黑5 浅蓝6，见 COLOR_CODE）。
#   例：452+321+254+312 的前三位 = 4、5、2 → 先抓绿(4)、再抓黑(5)、最后抓黄(2)。
# 怎么用的：每一帧挑"顺序里排头那个颜色"的圆 —— 同一个颜色在画面里有好几个就抓
# 最大的那个（跟原来"谁大抓谁"一个口径）；**真发了 0x02 才**把顺序往前挪一位。
# 三个都抓完 = 这一批完了，**不再抓**（画面上剩下的多半是定位圆/别的批次的），
# 等车跑到新的一站或者重新认到 tag 才从头来。
# 和 --color-policy 是两件事：那个管"同一个颜色能不能抓第二次"（防底下那个同色
# 定位圆的空抓），这个管"该抓哪个颜色"。两个都开着时以顺序为准。
TAG_ORDER_ENABLE = True     # （--tag-order / --no-tag-order）关掉 = 老的"谁大抓谁"
TAG_ORDER_GROUP = 1         # 用第几组三位数当抓取顺序（--order-group）：
                            # 规则里 1 = 第一批、3 = 第二批。**第二批要按第 3 组抓的话
                            # 得连批次一起管**（现在两批都会用这一组），说一声再加
TAG_ORDER_MISS_POLICY = 'biggest'
                            # 该抓的颜色**没出现在画面里**时怎么办（--order-miss-policy）。
                            # 认不出颜色（光照偏、黑物料）、被机械臂挡住、转盘还没转到位
                            # 都会这样：
                            #   'biggest' 先等 TAG_ORDER_MISS_WAIT 秒，还没有就抓画面里
                            #             最大的那个没抓过的（默认）—— 跟"微调发满就照抓"
                            #             一个口径：宁可抓偏也不整轮卡着不抓。原因会打出来
                            #   'wait'    一直压着不抓、等它出现（严格按顺序；
                            #             代价：颜色认不出来就整轮卡住）
TAG_ORDER_MISS_WAIT = 1.5   # 'biggest' 时等满几秒就放弃（--order-miss-wait）


# 类型 -> 数据段长度的白名单。协议里没有 CRC 也没有帧尾，
# 帧边界只能靠这张表认，所以新类型必须在这里登记，否则会被当成噪声跳过。
TYPE_LEN = {
    0x00: 18, 0x01: 32, 0x02: 32, 0x03: 12,
    0x04: 18, 0x05: 12, 0x10: 12, 0x11: 28,
    TYPE_STATE: CMD_LEN,
    TYPE_CMD: CMD_LEN,
}

TAG_SEND_TIMES = 5     # 和校赛版一样：tag 是数据帧，连着发 5 次，丢一帧也不怕

# ---------------- 颜色 / 物料 ----------------
# 阈值表和 src/cv/cv/material.py 保持一致，那边改了这边也要改
HSV_RANGES = {
    "red":        [((0, 110, 90),   (10, 255, 255)),
                   ((170, 110, 90), (180, 255, 255))],
    "yellow":     [((10, 30, 110), (33, 255, 255))],
    "green":      [((50, 90, 60),   (80, 255, 255))],
    "blue":       [((100, 55, 80), (140, 255, 255))],
    "light_blue": [((86, 30, 130),  (100, 255, 255))],
    "black":      [((0, 0, 0),      (180, 50, 150))],
}
COLOR_ORDER = ["red", "yellow", "green", "blue", "light_blue", "black"]
COLOR_CN = {"red": "红", "yellow": "黄", "green": "绿",
            "blue": "蓝", "light_blue": "浅蓝", "black": "黑"}
COLOR_CODE = {"red": "1", "yellow": "2", "blue": "3",
              "green": "4", "black": "5", "light_blue": "6"}   # 规则里的颜色编号
CODE_COLOR = {code: name for name, code in COLOR_CODE.items()}  # 反过来：'1' -> 'red'
                                                                # （tag 里那三位数字用）
# 画框用的 BGR（cv2.putText 画不了中文，框边上的字用英文）
COLOR_BGR = {"red": (0, 0, 255), "yellow": (0, 255, 255), "green": (0, 255, 0),
             "blue": (255, 0, 0), "light_blue": (255, 180, 0), "black": (90, 90, 90)}

# --- 圆的判据（物料 = 一个圆）---
CIRCLE_R_MIN_FRAC = 0.04 # 最小半径 = 帧短边的这个比例（--min-radius 给了像素值就用那个）
CIRCLE_R_MAX_FRAC = 0.45   # 最大半径 = 帧短边的这个比例
_DISK_SCALE = 0.85         # 认颜色时取样用的盘 = 检出半径的 0.85（躲开边缘过渡色）

# 物料是**圆台**，投影下来是大小两个圆（底面轮廓 + 顶面），两个都会被 Hough 检出。
# 它们物理上就是**同一块物料**，必须合成一块 —— 不合并的话 materials[0]（"最大的那个圆"，
# 对准和停稳判定都看它）会在两帧之间换人：换人了圆心/半径就跳一下，停稳判定每帧都判成
# "还在动"，车会一直等下去、永远不抓。
# 合并后**留大的那个**：半径大、圆心更稳，底盘校准用它。
#
# 判据是"小圆的圆心落在大圆的盘里"（dist ≤ 大圆半径 × 这个比例），不是"两个圆心几乎重合" ——
# 圆台斜着看时顶面是偏的：2026-09-18 实测那个红圆台是 r=62@(264,218) 和 r=51@(298,218)，
# **圆心差 34px**，按"重合"判就合不上。而两块并排的物料圆心至少隔 2 个半径，合不上。
CONE_MERGE_FRAC = 1.0      # 圆心距 ≤ 大圆半径 × 这个比例 = 同一个圆台的两层（1.0 = 落在大圆盘里）

# HoughCircles 两种方法。**默认 ALT**：合成图上它一个假圆都不出。
# ALT 的 param2 是"圆的完美度"(0~1，越大越严)；经典 GRADIENT 的 param2 是"累加器票数"，
# 票数跟周长走，量纲完全不同别混用（ALT 上填 15 会一个圆都检不出来）。
# 抗锯齿合成图上 0.8/0.85/0.9 都是 6/6、空图 0 假圆，噪声 sigma 到 16 也一样；
# 0.85 取中间，两边都留点余量。
#
# ⚠ 调参前必读：ALT 对**硬边**极敏感。合成图要是用 cv2.circle 默认的 LINE_8 画，
#   640x480 下扫 40 个半径会漏 18 个（看着像算法烂，其实是渲染出来的硬边）；
#   换成 LINE_AA（真机摄像头就是这种软边）同一个参数只漏 1 个，而且那个还超出了 maxRadius。
#   所以拿合成图扫参数一定开 LINE_AA，否则量的是渲染的毛病。param1 实测基本不影响结果。
# 物料互相遮挡 / 被机械臂挡掉一块时，GRADIENT 反而更能认出来（ALT 要求圆弧完整）。
HOUGH_METHOD = 'alt'
HOUGH_DEFAULTS = {
    'alt':      {'param1': 300, 'param2': 0.85, 'dp': 1.5},
    'gradient': {'param1': 120, 'param2': 15,  'dp': 1.0},
}

_MORPH = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
_MORPH_SMALL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

STATE_CN = {STATE_WAIT_QR: "等待二维码", STATE_WAIT_GRAB: "等待抓取指令",
            STATE_MOVING: "运动中"}
CMD_CN = {CMD_IDLE: "空指令(心跳)", CMD_QR_DONE: "二维码识别完毕,继续行进",
          CMD_GRAB: "抓取一次",
          CMD_ADJUST_FWD: "微调-前", CMD_ADJUST_BACK: "微调-后",
          CMD_ADJUST_LEFT: "微调-左", CMD_ADJUST_RIGHT: "微调-右"}
# 微调的英文短名，画在预览里（cv2.putText 画不了中文）
ADJUST_EN = {CMD_ADJUST_FWD: "FWD", CMD_ADJUST_BACK: "BACK",
             CMD_ADJUST_LEFT: "LEFT", CMD_ADJUST_RIGHT: "RIGHT"}


# ==================== 相机：找设备 ====================
def board_name(dev):
    """读这个节点的板卡名（v4l2-ctl 打的那一列，比如 "2M: 2M"）。"""
    node = os.path.basename(os.path.realpath(dev))
    try:
        with open(os.path.join(V4L_SYSFS, node, 'name')) as f:
            return f.read().strip()
    except OSError:
        return '?'


def _video_nodes():
    """所有 /dev/videoN，按编号从小到大。"""
    if not os.path.isdir(V4L_SYSFS):
        return []
    names = [n for n in os.listdir(V4L_SYSFS) if n.startswith('video')]
    names.sort(key=lambda s: int(s[5:]) if s[5:].isdigit() else 999)
    return [f'/dev/{n}' for n in names]


def _v4l_aliases():
    """{ '/dev/videoN': ['by-id 或 by-path 里的名字', ...] }，反查用。"""
    aliases = {}
    for d in (V4L_BY_ID, V4L_BY_PATH):
        if not os.path.isdir(d):
            continue
        for e in sorted(os.listdir(d)):
            dev = os.path.realpath(os.path.join(d, e))
            aliases.setdefault(dev, []).append(e)
    return aliases


def list_cameras():
    """等价于 v4l2-ctl --list-devices，外加每个节点的 by-id / by-path 别名。

    by-id 是"跟着设备走"的名字（插拔、换 USB 口、重启都不变），优先用它；
    两个同型号摄像头序列号也一样时 by-id 会撞名，那时用 by-path 按物理口认。
    """
    aliases = _v4l_aliases()
    nodes = _video_nodes()
    if not nodes:
        print(f'{V4L_SYSFS} 不存在或没有 video 节点（这台机器没有 v4l 设备？）')
        return
    print('/dev/video* 节点（等价 v4l2-ctl --list-devices）：')
    for dev in nodes:
        print(f'  {dev:16s} {board_name(dev)}')      # 板卡名就是 v4l2-ctl 打的那一列
        for a in aliases.get(dev, []):
            print(f'      {a}')
    print('\n用法：')
    print("  --dev 'name:板卡名片段'   # 推荐，板卡名不随枚举顺序变")
    print('  --dev /dev/videoN        # 也可以直接给节点')
    print('提示：UVC 摄像头通常 index0 = 取流节点、index1 = metadata，别开错。')
    print('      哪个是取流节点拿不准就一个个试，脚本会告诉你读不读得到帧。')


def warn_camera(msg):
    """相机没开起来时的显著提醒（一整条感叹号框住，滚过去了也看得见）。

    跟串口"没连上、5 秒后重试"一个口径：**每轮重试都打一遍**，不是只报一次。
    """
    bar = '!' * 68
    print(f'[相机] {bar}')
    for line in msg.splitlines():
        print(f'[相机] !!! {line}')
    print(f'[相机] {bar}')


def resolve_device(spec, index=0, by='id', quiet=False):
    """把设备说明解析成真实路径。认四种写法，认不出来返回 ''（绝不默默退回 video0）。

    quiet=True 只影响打印：重试时同一个诊断每 5 秒刷一遍会淹掉别的东西，
    这时只留调用方那一行显著提醒（见 CameraWorker._report）。
    """
    if not spec:
        return ''

    # 0) 'name:片段' 按板卡名找（默认就是这么写的）
    if spec.startswith('name:'):
        frag = spec[5:].strip()
        hits = [d for d in _video_nodes() if frag.lower() in board_name(d).lower()]
        if not hits:
            if not quiet:
                print(f'[相机] 没有板卡名含 "{frag}" 的节点')
                for d in _video_nodes():
                    print(f'[相机]   {d}  "{board_name(d)}"')
            return ''
        # 片段短一点就会**同时命中好几块不同的板子**。笔记本上 "Integrated" 一抓三个：
        # 内置摄像头、内置红外、还有刚插上的 USB 摄像头。这时候按编号挑第一个，
        # 开起来的多半就是笔记本自己的那个 —— 而且它取流节点给不出帧，画面全黑。
        # 用户 2026-09-18 报的"把电脑摄像头当成 tag 相机、还没画面"就是这个。
        # 所以：命中的板卡名不止一种就**不开**，把现状和能直接抄的写法打出来。
        # （车上只有一块板子含这个词，写短片段照样能匹配上，不影响）
        names = sorted({board_name(d) for d in hits})
        if len(names) > 1:
            if not quiet:
                print(f'[相机] "{frag}" 一下子命中了 {len(names)} 块**不同的板卡**，'
                      f'分不清要哪一块，这一路先不开：')
                for d in hits:
                    print(f'[相机]   {d}  "{board_name(d)}"')
                print('[相机] 把板卡名写长一点就不用猜了（挑只在你要的那块板上出现的词），'
                      '填进 --dev / --tag-dev：')
                for n in names:
                    print(f"[相机]   'name:{n}'")
                print('[相机] 也可以直接给节点路径（/dev/videoN）。看现状：--list')
            return ''
        aliases = _v4l_aliases()
        # 一块板子一般挂两个节点（取流 + metadata），优先挑 by-id 里带
        # -video-indexN 的那个 —— 那才是取流节点
        pref = [d for d in hits
                if any(a.endswith(f'-video-index{index}') for a in aliases.get(d, []))]
        pick = pref[0] if pref else hits[0]
        if not quiet:
            print(f'[相机] 板卡名含 "{frag}" 的节点: {", ".join(hits)} → 用 {pick}')
            if len(hits) > 1 and not pref:
                print(f'[相机] 这几个的板卡名一样，分不出取流/metadata（没有 by-id 别名），'
                      f'先按编号小的来；要是读不到帧就换 {hits[1]} 试试')
        return os.path.realpath(pick)

    # 1) 直接给路径（/dev/video0、/dev/v4l/by-id/xxx）
    if spec.startswith('/dev/'):
        if os.path.exists(spec):
            dev = os.path.realpath(spec)
            if not quiet:
                print(f'[相机] {spec} -> {dev}')
            return dev
        if not quiet:
            print(f'[相机] {spec} 不存在')
        return ''

    # 2) 给 by-id / by-path 里的名字（-video-indexN 可带可不带）
    v4l_dir = V4L_BY_PATH if by == 'path' else V4L_BY_ID
    for name in (f'{spec}-video-index{index}', spec):
        link = os.path.join(v4l_dir, name)
        if os.path.exists(link):
            dev = os.path.realpath(link)
            if not quiet:
                print(f'[相机] {name} -> {dev}')
            return dev

    # 3) 名字对不上（换机器/换摄像头）：把现状打出来，别瞎猜
    entries = sorted(os.listdir(v4l_dir)) if os.path.isdir(v4l_dir) else []
    if not quiet:
        print(f'[相机] {v4l_dir} 里没有 {spec}（-video-index{index}）')
        if not entries:
            print(f'[相机] {v4l_dir} 不存在或为空（用 --list 看现状）')
        else:
            print('[相机] 当前可用：')
            for e in entries:
                print(f'         {e} -> {os.path.realpath(os.path.join(v4l_dir, e))}')
    if not entries:
        return ''

    suffix = f'-video-index{index}'
    same = [e for e in entries if e.endswith(suffix)]
    if len(same) == 1:
        if not quiet:
            print(f'[相机] 名字对不上，但只有 {same[0]} 一个取流节点，先用它')
            print(f'[相机] 想固定住就把设备名设成 {same[0][:-len(suffix)]}')
        return os.path.realpath(os.path.join(v4l_dir, same[0]))
    if not quiet:
        print(f'[相机] 匹配到 {len(same)} 个取流节点，分不清哪个是哪个，'
              f'直接给设备路径（同型号撞名就用 --by path）')
    return ''


def open_camera(dev, width, height, fps, fourcc):
    """MJPG 一定要在设分辨率之前设，很多摄像头换了尺寸就不认后面的格式。

    必须点名 CAP_V4L2：这台机器上 OpenCV 默认会挑 GStreamer 后端，它只认尺寸/帧率
    的 caps，**格式和曝光这些控制全部丢掉** —— 冷机开机（设备默认 1920x1080 YUYV@5）
    时尺寸也会协商失败，于是一路 1920x1080@5 跑着，看着像"能出图"，其实又糊又慢。
    """
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None
    if fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # 只留最新一帧，别积压

    real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    real_fps = cap.get(cv2.CAP_PROP_FPS)
    real_cc = int(cap.get(cv2.CAP_PROP_FOURCC))
    real_fourcc = ''.join(chr((real_cc >> (8 * i)) & 0xFF) for i in range(4))
    print(f'[相机] {dev} "{board_name(dev)}" 实际 {real_w}x{real_h} @ {real_fps:.1f}fps '
          f'{real_fourcc}（要的 {width}x{height} @ {fps}fps {fourcc}）')
    if real_fourcc.strip('\x00') not in ('', fourcc):
        print(f'[相机] {dev} 格式没设上（要 {fourcc} 实际 {real_fourcc}），'
              f'YUYV 在高分辨率下帧率会掉，换 MJPG 再试')
    return cap


# V4L2 的自动曝光取值：1=手动(锁死当前值) 3=自动。别写成 0/1，那是另一套后端的用法。
AUTO_EXPOSURE_MANUAL = 1
AUTO_EXPOSURE_AUTO = 3


def apply_ctrls(cap, dev, ctrls):
    """设色相/亮度/增益/曝光，并读回来核对 —— 摄像头不认这个控制时 cap.set 是静默失败的，
    不打回来的话你会以为设上了。

    别纠结 auto_exposure 和 exposure 谁先设：这颗 2M 模组上两个都是死控制（见文件顶上的实测），
    先设后设、设不设都一样，真正改变画面的只有 brightness。
    """
    ae = ctrls.get('auto_exposure')
    if ae is not None:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE,
                AUTO_EXPOSURE_AUTO if ae == 'auto' else AUTO_EXPOSURE_MANUAL)

    failed = False
    for name, prop in (('hue', cv2.CAP_PROP_HUE),
                       ('brightness', cv2.CAP_PROP_BRIGHTNESS),
                       ('gain', cv2.CAP_PROP_GAIN),
                       ('exposure', cv2.CAP_PROP_EXPOSURE)):
        want = ctrls.get(name)
        if want is None:
            continue
        before = cap.get(prop)
        cap.set(prop, want)
        got = cap.get(prop)
        if abs(got - want) >= 1e-6:
            failed = True
        print(f'[相机] {dev} {name}: 要 {want:g} 实际 {got:g}'
              + ('' if abs(got - want) < 1e-6 else f'（没设上，原来是 {before:g}）'))
    if failed:
        list_ctrls_hint(dev)
    if ae is not None:
        print(f'[相机] {dev} 自动曝光: {"开" if ae == "auto" else "关(手动)"}'
              f'（读到 {cap.get(cv2.CAP_PROP_AUTO_EXPOSURE):g}）')


def exposure_check(cap, dev, ctrls, frames=20):
    """开完相机量一张的亮度分布，打一行"曝光体检"。

    曝成什么样只有量出来才知道：这颗模组上 auto_exposure/exposure 都是死控制，
    唯一的旋钮是 brightness，所以直接给一行数字照着调，别靠眼睛看预览。
    判据（都是实测出来的）：过曝 >10% 就偏高 —— HSV_RANGES 里每种颜色都有 S 下限，
    画面一冲爆物料的饱和度就掉到门槛以下，"曝光不对"就是这么表现成"HSV 判据失效"的。
    """
    f = None
    for _ in range(frames):
        ret, fr = cap.read()
        if ret and fr is not None:
            f = fr
    if f is None:
        return
    v = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)[:, :, 2]
    med = float(np.median(v))
    hot = float((v >= 250).mean()) * 100
    dark = float((v <= 30).mean()) * 100
    br = ctrls.get('brightness')
    knob = f'brightness 现在 {br:g}' if br is not None else '这一路没设 brightness'
    print(f'[相机] {dev} 曝光体检: V中位 {med:.0f}  过曝 {hot:.1f}%  压黑 {dark:.1f}%'
          f'（{knob}）')
    if hot > 10:
        print(f'[相机] 过曝 {hot:.1f}% 偏高：把 brightness 往负调，1 格 ≈ 1 个 V')
    elif med < 60:
        print(f'[相机] 画面偏暗（V中位 {med:.0f}）：把 brightness 往正调')


def list_ctrls_hint(dev):
    """这台摄像头支持哪些控制、范围多少，只有驱动知道 —— 让它自己说。"""
    print(f'[相机] 想看清楚 {dev} 支持哪些控制/范围，在树莓派上跑：'
          f'v4l2-ctl -d {dev} --list-ctrls')


# ==================== 相机：采集+识别线程 ====================
class CameraWorker(threading.Thread):
    """一路相机：自己开、自己读、自己识别，结果放在锁保护的字段里给主循环取。

    两路相机的分辨率和帧率可以完全不同，所以各自一条线程独立跑；
    主循环只做协议判断、不碰摄像头，这样 100ms 的状态上报不会被读帧拖慢。
    """

    def __init__(self, role, dev, width, height, fps, fourcc, hp=None, ctrls=None,
                 cam_rot=CAM_ROT, spec='', video_index=VIDEO_INDEX, by='id'):
        super().__init__(daemon=True)
        self.role = role               # 'tag' 或 'material'
        self.dev = dev                 # 解析好的节点路径。'' = 还没解析出来 / 掉了，
                                       # 每 CAM_RETRY_INTERVAL 秒重新解析一次
        self.spec = spec or dev        # 用户给的设备说明（'name:xxx' / /dev/videoN / by-id 名）
        self.video_index, self.by = video_index, by
        self.width, self.height = width, height
        self.fps_want, self.fourcc = fps, fourcc
        self.ctrls = ctrls or {}       # 亮度/增益/曝光（没给就不碰）
        self.cam_rot = cam_rot         # 相机是怎么转着装的，默认取 CAM_ROT：
                                       #   0=正装(画面不转)  1=逆时针90°(现在的车)
                                       #   2=180°  3=顺时针90°
        self.hp_in = hp or default_hp()   # Hough 参数（0=auto，按分辨率换算）
        self.hp = {}                   # 换算后的实际值，预热完填上
        self.lock = threading.Lock()
        self._frame = None
        self._tag = None               # tag 路：最近认到的二维码内容
        self._materials = []           # 物料路：检出的圆 Circle
        self._fps = 0.0
        self._proc_ms = 0.0
        self.error = ''                # 非空 = 这一路现在没在出帧（线程会自己重试）
        self.tries = 0                 # 试了几轮。第一轮打全，之后每轮只打一行，别刷屏
        self._running = True

    def up(self):
        """这一路现在能用吗。**随时会变**：线程每一轮重试都可能把 error 清掉。"""
        return not self.error

    def stop(self):
        self._running = False

    def snapshot(self):
        with self.lock:
            return self._frame, self._tag, list(self._materials)

    def material_detail(self):
        with self.lock:
            return list(self._materials), dict(self.hp)

    def stats(self):
        with self.lock:
            return self._fps, self._proc_ms

    def run(self):
        """线程入口。整条包一层：线程里崩了默认只往 stderr 吐一行堆栈，主循环
        还当它活着（error 是空的），预览就少一个窗口、看着像"相机没开" —— 踩过一次。"""
        try:
            self._run()
        except Exception as e:
            self.error = f'线程崩了: {e!r}'
            traceback.print_exc()

    def _report(self, msg):
        """这一路现在起不来：error 记下来（主循环的状态行会显示"没开起来"），
        再打一条**显著**提醒。每轮重试都打一遍 —— 跟串口没连上那个一个口径。"""
        self.error = msg
        warn_camera(f'{self.role} 这一路没开起来（第 {self.tries} 次试）：{msg}\n'
                    f'每 {CAM_RETRY_INTERVAL:.0f}s 重试一次，插上/换口/节点变了都会自己接上；'
                    f'看设备现状用 --list')

    def _retry_wait(self):
        """等下一轮重试。拆成小步睡，stop() 一叫就能马上退出来。"""
        for _ in range(int(CAM_RETRY_INTERVAL / 0.1)):
            if not self._running:
                return
            time.sleep(0.1)

    def _run(self):
        # 起不来就每 CAM_RETRY_INTERVAL 秒重来一轮：相机没插、插晚了、USB 口换了、
        # 枚举顺序变了（节点号从 video5 变成 video4）都能自己接上，不用重启脚本。
        # 串口那边就是这么干的（McuLink._loop），相机这边原来只报一次就再也不管了。
        while self._running:
            if self._run_once():
                return                      # 正常退出（stop 了）
            self._retry_wait()              # 这一轮没起来：等一下再来

    def _run_once(self):
        """开相机 + 跑到 stop 为止。返回 True = 正常结束；False = 这一轮没跑起来，该重试。"""
        self.tries += 1
        quiet = self.tries > 1              # 重试时别再刷一遍一屏诊断
        if not self.dev:
            # 每次重试都重新解析：节点号是会变的（插拔之后 video5 可能变成 video4），
            # 死抱着开机时的路径重试，就永远开不起来了
            self.dev = resolve_device(self.spec, self.video_index, self.by, quiet=quiet)
            if not self.dev:
                self._report(f'相机没找到：设备说明是 {self.spec!r}')
                return False
        cap = open_camera(self.dev, self.width, self.height, self.fps_want, self.fourcc)
        if cap is None:
            dev, self.dev = self.dev, ''    # 清掉路径，下一轮重新解析（可能已经拔了）
            self._report(f'打不开 {dev}（不在了？被别的程序占着？）')
            return False

        if self.ctrls:
            apply_ctrls(cap, self.dev, self.ctrls)
            exposure_check(cap, self.dev, self.ctrls)

        # 预热，顺便确认这个节点真的出帧。能 open 但永远没帧的节点不少：
        # UVC 的 metadata 节点、rpivid 那种硬件解码器节点，都是这样。
        first = None
        for _ in range(10):
            ret, f = cap.read()
            if ret and f is not None:
                first = rot_frame(self.cam_rot, f)     # 先转正，后面全按转正的算
                break
        if first is None:
            dev, self.dev = self.dev, ''
            cap.release()
            self._report(f'{dev} 打得开但读不到帧 —— 这个节点多半不是取流节点。'
                         f'用 v4l2-ctl -d {dev} --list-formats-ext 确认，'
                         f'要的是带 Video Capture + YUYV/MJPG 的那一组')
            return False
        self.error = ''                     # 起来了（原来是坏的也要清掉，状态行别一直挂着）

        if self.cam_rot % 4:
            print(f'[相机] {self.role} {CAM_ROT_CN[self.cam_rot % 4]}，'
                  f'画面已转正，实际用 {first.shape[1]}x{first.shape[0]}'
                  f'（跟上面打印的传感器尺寸宽高是反的，正常）')

        detector = cv2.QRCodeDetector() if self.role == 'tag' else None
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        if self.role == 'material':
            # 半径按实际帧的短边算，所以拿预热那帧的尺寸（不是循环里的 frame，那时候还没有）
            self.hp = eff_hough(self.hp_in, first.shape[0], first.shape[1])

        n, t0, proc_ms, bad = 0, time.time(), 0.0, 0
        while self._running:
            ret, frame = cap.read()
            if not ret or frame is None:
                bad += 1
                if bad >= CAM_READ_FAIL_MAX:
                    # 拔了 / 被别的程序抢了 / 这个节点本来就不出帧。别在这儿空转，
                    # 交回 _run 去重开重解析 —— 重插上就自己回来了
                    dev, self.dev = self.dev, ''
                    cap.release()
                    self._report(f'{dev} 连着 {CAM_READ_FAIL_MAX} 帧读不到（拔了？被别的程序抢了？）')
                    return False
                time.sleep(0.01)
                continue
            bad = 0
            if self.cam_rot % 4:
                frame = rot_frame(self.cam_rot, frame)   # 转正了才存/才识别
            t = time.time()
            if self.role == 'tag':
                text = detect_qrcode(frame, detector, clahe)
                with self.lock:
                    self._frame = frame
                    self._tag = text
            else:
                hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
                mats = detect_materials(frame, hsv, self.hp)
                with self.lock:
                    self._frame = frame
                    self._materials = mats
            proc_ms = 0.9 * proc_ms + 0.1 * (time.time() - t) * 1000.0

            n += 1
            elapsed = time.time() - t0
            if elapsed >= 2.0:
                with self.lock:
                    self._fps, self._proc_ms = n / elapsed, proc_ms
                n, t0 = 0, time.time()

        cap.release()
        return True


# ==================== 帧 ====================
def detect_qrcode(frame, detector, clahe):
    """校赛版原样：灰度 + CLAHE + QRCodeDetector。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = clahe.apply(gray)
    data, _, _ = detector.detectAndDecode(gray)
    return data.strip() if data else None


def build_tag_frame(data):
    """0x03 tag 帧：12 字节 ASCII 数字。"""
    data = data.replace('+', '')
    if (not data.isdigit()) or len(data) != TAG_DATA_LEN:
        raise ValueError(f'二维码内容必须是 {TAG_DATA_LEN} 位数字，实际: {data!r}')
    return FRAME_HEADER + bytes([TYPE_TAG, TAG_DATA_LEN]) + data.encode('ascii')


def parse_tag_order(text, group=TAG_ORDER_GROUP):
    """tag 内容 -> 这一批要抓的颜色顺序（list），认不出来返回 None。

    规则：二维码是 4 组三位数、用 + 连接，第 1 组就是第一批物料的搬运颜色**和顺序**。
    所以 452+321+254+312（或者去掉 + 的 452321254312）第 1 组取出来是 '452'
    → ['green', 'black', 'yellow']（颜色编号见 COLOR_CODE）。

    组里认不出的数字（0/7/8/9）会跳过并说一声：跳一个还剩两个，照样按顺序抓，
    总比整组不认、退回去"谁大抓谁"要好。整组一个都认不出才返回 None。
    """
    digits = ''.join(ch for ch in text if ch.isdigit())
    end = group * 3
    if group < 1 or len(digits) < end:
        print(f'[顺序] tag 是 {text!r}，凑不出第 {group} 组三位数'
              f'（一共才 {len(digits)} 位数字），这一轮不按顺序抓')
        return None
    chunk = digits[end - 3:end]
    order, bad = [], []
    for ch in chunk:
        name = CODE_COLOR.get(ch)
        if name is None:
            bad.append(ch)
        else:
            order.append(name)
    if bad:
        print(f'[顺序] 第 {group} 组「{chunk}」里的 {"、".join(bad)} 不是颜色编号'
              f'（红1 黄2 蓝3 绿4 黑5 浅蓝6），这几位跳过')
    if not order:
        print(f'[顺序] 第 {group} 组「{chunk}」一个颜色编号都对不上，这一轮不按顺序抓')
        return None
    if len(set(order)) != len(order):
        # 同一批里两个同色物料：抓走第一个之后，它底下压着的同色定位圆还在原处，
        # 第二个就分不出来了（--color-policy once 会把它压住不抓，不会空抓，
        # 但那一批也就少抓一个）。先把话说明白，真遇上了再想办法
        print(f'[顺序] ⚠ 第 {group} 组「{chunk}」里有重复的颜色，同一批两个同色物料'
              f'没法区分（底下压着的定位圆也是这个颜色），第二个会被压住不抓')
    print(f'[顺序] tag 第 {group} 组「{chunk}」= 按这个顺序抓：'
          + ' → '.join(f'{COLOR_CN[c]}({COLOR_CODE[c]})' for c in order))
    return order


def build_cmd_frame(cmd):
    """0x21 指令帧：固定 5 字节 AA 55 21 01 CC。
    微调(30~33)也是走这条帧，只是第 5 个字节不同。"""
    if cmd not in LEGAL_CMDS:
        raise ValueError(f'指令字 0x{cmd:02X} 不在协议里'
                         f'（只有 00/01/02 和微调 30/31/32/33 合法）')
    return FRAME_HEADER + bytes([TYPE_CMD, CMD_LEN, cmd])


def rot_frame(rot, frame):
    """按相机是怎么装着转的，把画面转回来（预览和识别都用转回来的画面）。

    rot 是**相机转的方向**：相机逆时针转了 90°，画面里的东西就是顺时针躺着的，
    所以要把画面逆时针转回去。转完画面宽高互换（640x480 -> 480x640）。
    """
    rot = int(rot) % 4
    if rot == 1:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if rot == 2:
        return cv2.rotate(frame, cv2.ROTATE_180)
    if rot == 3:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    return frame


def resolve_aim(aim_x, aim_y, height, width):
    """对准点 = 想让物料出现在画面的哪里。默认画面正中；给了像素值就用给的。

    注意 h/w 要传**转正之后**那一帧的尺寸（CAM_ROT 非 0 时宽高是互换的）。
    """
    return (width // 2 if aim_x < 0 else int(aim_x),
            height // 2 if aim_y < 0 else int(aim_y))


def align_cmd(ex, ey):
    """按圆心离对准点的误差，给一条底盘微调指令。

    ex = 圆心 x - 对准点 x（正 = 圆在画面右边），ey = 圆心 y - 对准点 y（正 = 圆偏下）。
    一次只给一条：前后和左右同时发底盘会斜着走，而且哪条生效了看不出来，
    所以先修误差大的那个轴（一样大就修左右）。
    """
    result = None
    if abs(ex) >= abs(ey):
        result = CMD_ADJUST_RIGHT if ex > 0 else CMD_ADJUST_LEFT
        if(result == CMD_ADJUST_LEFT):
            print("[微调] ex<=0，按协议优先修左右，发 CMD_ADJUST_LEFT")
        elif(result == CMD_ADJUST_RIGHT):
            print("[微调] ex>0，按协议优先修左右，发 CMD_ADJUST_RIGHT")
        return result
    result = CMD_ADJUST_BACK if ey > 0 else CMD_ADJUST_FWD
    if(result == CMD_ADJUST_FWD):
        print("[微调] ey<=0，按协议修前后，发 CMD_ADJUST_FWD")
    elif(result == CMD_ADJUST_BACK):
        print("[微调] ey>0，按协议修前后，发 CMD_ADJUST_BACK")
    return result


def align_cmd_for(ex, ey, tol_x, tol_y):
    """按**两个轴各自的容差**决定这一步该修哪边；都在容差里返回 None。

    不能光按"误差大的轴"挑（align_cmd 的挑法）：左右容差放宽到 60、前后还是 20 时，
    ex=50 在容差里、ey=30 已经超了 —— 照"误差大"会去挪左右，白挪一步。
    所以先看谁超了自己的容差：只超一个就修那个，两个都超才比大小。
    """
    out_x, out_y = abs(ex) > tol_x, abs(ey) > tol_y
    if out_x and out_y:
        return align_cmd(ex, ey)              # 都超：照旧修误差大的那个
    if out_x:
        return CMD_ADJUST_RIGHT if ex > 0 else CMD_ADJUST_LEFT
    if out_y:
        return CMD_ADJUST_BACK if ey > 0 else CMD_ADJUST_FWD
    return None



def nudge_step_note(before, after, cmd, tol_x, tol_y):
    """微调走完一步后，量一下"3cm 在画面里是多少像素"，并判断容差够不够。

    before / after = 挪之前、挪之后的圆心误差 (ex, ey)。任一为 None 就没得比，
    返回 None（这帧没看到圆是正常的，不算错）。
    cmd = 那一步实际发出去的方向（0x30~0x33）—— 用它定"量哪个轴、拿哪个容差"，
    比照着误差大小猜靠谱（两个轴的容差不一样，猜错了会指错该改哪个常量）。

    为什么要量它：底盘一步是**下位机固定的 3 cm**，上位机没地方传距离，所以误差只能
    一步一步逼近。一步在画面里跨 S 个像素，容差 T 只要 < S/2，就会"差一点没进容差 →
    挪一步 → 冲过头到另一边 → 再挪回来"，永远在容差两侧来回 —— 车上看着就是"小车来回动"。
    收敛的必要条件就是 **T ≥ S/2**，S 只能实测。

    另一半步（≈1.5 cm）就是这套机械的**精度上限**：3 cm 的步长下，最后一发落地时
    残差最理想也就是半步。要更准只能让电控把步长改小，上位机这边调不出来。
    """
    if before is None or after is None:
        return None
    ax = 0 if cmd in (CMD_ADJUST_LEFT, CMD_ADJUST_RIGHT) else 1
    tol = tol_x if ax == 0 else tol_y
    b, a = before[ax], after[ax]
    step = abs(b - a)
    msg = f'[微调] 走完一步: {"xy"[ax]} 方向差 {b:+.0f} → {a:+.0f}px（3cm ≈ {step:.0f}px）'
    if step < 1:
        return msg + '；画面里没动 —— 查轮子打滑/堵转（协议 §七）'
    if step > 2 * tol:
        msg += (f'；**一步比容差的 2 倍还大**，必然在两侧来回 —— '
                f'把 {"AIM_TOL_X" if ax == 0 else "AIM_TOL_Y"} 提到 '
                f'{step / 2:.0f} 以上（半步 = 精度上限）')
    elif abs(a) > tol:
        msg += '；还没进容差，继续挪'
    return msg


class NudgePacer:
    """微调的节拍器：一次一按，节拍由下位机的 0x01 给（**不是定时器**）。

    见 USART1_Nudge_Protocol.md §四/§五：发出 `0x30`~`0x33` 之后，下位机把周期上报
    切成 `0x10`（运动中），走完约 0.3 s 自己切回 `0x01`。所以**看到 `0x01` = 上一发
    走完了**；那期间（`0x10` 里）发出去的微调会被静默丢弃 —— 不计错、不缓存。
    "发完 sleep 0.5 s 再发下一发"的开环写法会少走几厘米，而且没有任何报错。

    用法（主循环每帧喂状态、发出去时报一声）：

        ev = pacer.note(state, now)     # 'station' / 'done' / 'stuck' / None
        ok, why = pacer.ready(now)      # 想发之前问一句
        ...真的发出去之后...
        pacer.sent(now, real=True)      # real=False（离线干跑）不用等回执

    `real=False` 时没有回执可等，就退回按 `interval` 定时 —— 桌面上干跑的行为
    和以前的定时器版一样，不然一发之后就永远等不到状态变化了。
    """

    def __init__(self, interval=ALIGN_INTERVAL, timeout=ALIGN_RETURN_TIMEOUT,
                 gap=ALIGN_NEW_STOP_GAP):
        self.interval = interval
        self.timeout = timeout
        self.gap = gap
        self.count = 0           # 这一站发了几条（--align-max 卡它）
        self.pending = False     # 发出去的那一发还没等到它回到 0x01
        self.saw_moving = False  # pending 期间看见过 0x10 了吗（协议里的"收到没"证据）
        self.stuck = False       # 等超时都没回到 0x01 → 这一站不再挪车
        self.at = 0.0            # 发上一发的时刻（兜底间隔和超时都从它算）
        self.left_at = None      # 最近一次离开 0x01 的时刻（判断是不是新的一站）
        self.last_state = None

    def new_stop(self):
        """车跑到新的一站：微调次数从头算、重新开门。"""
        self.count = 0
        self.stuck = False

    def note(self, state, now):
        """每帧喂一次当前 0x20 状态，返回这一刻发生的事（都没有就是 None）：

        'done'    上一发微调走完了（0x01 回来了）——这是"它收到了"的直接证据
        'stuck'   发出去了但一直没动，超时了
        'station' 车自己跑过来停下的（不是我们挪的），微调次数清零
        """
        ev = None
        if self.pending:
            if state == STATE_MOVING:
                self.saw_moving = True
            elif state == STATE_WAIT_GRAB:
                if self.saw_moving:
                    self.pending = False        # 0x01 回来了 = 走完了，可以发下一发
                    self.saw_moving = False
                    ev = 'done'
                elif now - self.at > self.timeout:
                    self.pending = False        # 它压根没动过 → 这一发没生效
                    self.stuck = True
                    ev = 'stuck'
        if (state == STATE_WAIT_GRAB and self.last_state is not None
                and self.last_state != STATE_WAIT_GRAB
                and (self.left_at is None or now - self.left_at > self.gap)):
            ev = 'station'                      # 离开得够久，是自己跑过来的
            self.new_stop()
        if self.last_state == STATE_WAIT_GRAB and state != STATE_WAIT_GRAB:
            self.left_at = now                  # 只在"离开"那一刻记时，回来才算时长
        self.last_state = state
        return ev

    def sent(self, now, real=True):
        """刚发出去一条微调。real=False（离线干跑）时没有回执可等。"""
        self.count += 1
        self.at = now
        self.pending = bool(real)
        self.saw_moving = False

    def ready(self, now):
        """现在能不能发下一发微调？返回 (能不能, 不能的原因)。"""
        if self.stuck:
            return False, (f'上一发微调发出去 {self.timeout:.0f}s 没等到下位机回到 0x01，'
                           f'这一站不再挪车了（对着 USART1_Nudge_Protocol.md §七 查）')
        if self.pending:
            return False, None        # 正在走，正常等待，不用刷屏
        if now - self.at < self.interval:
            return False, None        # 兜底的最小间隔
        return True, None


# ==================== 颜色 / 物料 ====================
def _mask_of_color(hsv, color):
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in HSV_RANGES[color]:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    # 先 open 掉小点, 再 close 补洞. open 用 3x3 更温和, 避免吃掉薄剪影
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _MORPH_SMALL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH)
    return mask


def color_masks(hsv):
    """6 种颜色各一张掩码。一帧只算一次，找圆和找色块都复用。"""
    return {c: _mask_of_color(hsv, c) for c in COLOR_ORDER}


# ---- 圆：物料的判据 ----
# defaults=(0,) 是给 merged 的：老的 6 个参数的写法（check_circle.py 那种）照样能用
Circle = namedtuple('Circle', 'color hsv area center radius fill merged', defaults=(0,))
# merged=这块物料上并掉了几个同心圆（圆台的另一层）。0 = 就检出一个圆。
#        只是信息 —— 判定、对准、停稳全都不看它，看的是这个圆本身（合并后留的是大的那个）
# color=圆内占比最高的颜色（认不出就是 None，不影响判定）
# hsv=(H,S,V) **整个圆内的平均**（0.85r 的盘，避开边缘过渡色），跟阈值表无关，
#             就是给你照着调 HSV_RANGES 用的
# area=πr²(px²)  center=(x,y)  radius=px  fill=那种颜色占取样盘的比例（只是信息）


def _hsv_of(hsv, mask):
    """mask 内像素的**平均** HSV，用来照着调阈值。

    H 用圆均值：红色跨 0/180 两头，直接取算术平均会算出个青绿来（0 和 179 平均是 90）。
    """
    sel = mask > 0
    if not np.any(sel):
        return None
    ang = np.radians(hsv[:, :, 0][sel].astype(np.float64))
    h = int(round(np.degrees(np.arctan2(np.sin(ang).mean(),
                                        np.cos(ang).mean())))) % 180
    return (h, int(round(hsv[:, :, 1][sel].mean())), int(round(hsv[:, :, 2][sel].mean())))


def default_hp(method=HOUGH_METHOD):
    """默认的 Hough 参数。param1/param2/min_dist/半径 给 0 = auto。"""
    d = HOUGH_DEFAULTS[method]
    return {'method': method, 'param1': d['param1'], 'param2': d['param2'],
            'min_dist': 0, 'min_radius': 0, 'max_radius': 0}


def eff_hough(hp, height, width):
    """把 0=auto 的 Hough 参数按这一路的分辨率换算成像素，返回实际用的值。

    半径用"帧短边的比例"表示，这样同一个默认值在 640x480 和 1280x720 下都说得通；
    实际用的值启动时会打印出来。param1/param2 给 0 就用这种方法自己的默认值。
    """
    eff = dict(hp)
    method = eff.get('method', HOUGH_METHOD)
    if method == 'alt' and not hasattr(cv2, 'HOUGH_GRADIENT_ALT'):
        print(f'[物料] 这个 OpenCV（{cv2.__version__}）没有 HOUGH_GRADIENT_ALT，'
              f'改用经典 GRADIENT')
        method = 'gradient'
    d = HOUGH_DEFAULTS[method]
    eff['method'] = method
    eff['dp'] = d['dp']
    if not eff.get('param1'):
        eff['param1'] = d['param1']
    if not eff.get('param2'):
        eff['param2'] = d['param2']
    elif method == 'alt' and eff['param2'] > 1:
        print(f'[物料] ALT 的 param2 是"完美度"，要 0~1（比如 0.85）；'
              f'现在的 {eff["param2"]} 是 GRADIENT 那套的量纲，会一个圆都检不出来')

    short = min(height, width)
    if eff['min_radius'] <= 0:
        eff['min_radius'] = max(4, int(short * CIRCLE_R_MIN_FRAC))
    if eff['max_radius'] <= 0:
        eff['max_radius'] = max(eff['min_radius'] + 1, int(short * CIRCLE_R_MAX_FRAC))
    eff['min_radius'] = int(eff['min_radius'])
    eff['max_radius'] = int(eff['max_radius'])
    if eff['min_dist'] <= 0:
        eff['min_dist'] = max(10, eff['min_radius'])   # 圆心太近就当同一个圆
    return eff


def detect_materials(frame, hsv, hp):
    """找物料 = 找圆。HoughCircles 检出几个圆就有几块物料，按面积从大到小。

    颜色不参与判定：每个圆只是顺带报一下圈内占比最高的颜色和它的 HSV
    （认不出就是 None），这样打印和预览能看出抓的是哪一块，也方便对着调 HSV 阈值。
    黑物料、光照偏、HSV 阈值没调好都不影响认圆。
    """
    masks = color_masks(hsv)
    gray = cv2.medianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 5)
    method = (cv2.HOUGH_GRADIENT_ALT if hp.get('method') == 'alt'
              else cv2.HOUGH_GRADIENT)
    found = cv2.HoughCircles(gray, method, dp=hp.get('dp', 1.0), minDist=hp['min_dist'],
                             param1=hp['param1'], param2=hp['param2'],
                             minRadius=hp['min_radius'], maxRadius=hp['max_radius'])
    materials = []
    if found is None:
        return materials

    for cx, cy, r in np.round(found[0]).astype(int):
        cx, cy, r = int(cx), int(cy), int(r)
        if r <= 1:
            continue
        # 取样盘比检出的小一圈：边缘那圈是物料到背景的过渡色，算进去会把占比压低
        disk = np.zeros(gray.shape, dtype=np.uint8)
        cv2.circle(disk, (cx, cy), max(2, int(r * _DISK_SCALE)), 255, -1)
        disk_area = float(cv2.countNonZero(disk))
        best_c, best_n = None, 0
        for c in COLOR_ORDER:
            n = cv2.countNonZero(cv2.bitwise_and(masks[c], disk))
            if n > best_n:
                best_c, best_n = c, n
        fill = best_n / disk_area if disk_area > 0 else 0.0
        # HSV 报**整个圆内**的平均，跟颜色阈值表无关。别改成"取匹配上的那些像素"：
        # 那样等于拿阈值筛完再告诉你阈值筛出来的东西，范围不对时你只会看到漏进去的
        # 那一两个像素的 HSV（实测遇过：圆里 1% 匹配上绿色，就报那 1% 的 HSV）。
        mats_hsv = _hsv_of(hsv, disk)
        materials.append(Circle(best_c, mats_hsv, float(np.pi * r * r), (cx, cy), r, fill))

    # 排序和圆台合并都在 merge_cones 里（它自己排，不靠这里），返回的还是从大到小
    return merge_cones(materials)


def merge_cones(materials):
    """把同一块圆台的大小两个圆合成一块物料，留大的那个（见 CONE_MERGE_FRAC）。

    按面积从大到小过一遍：小的圆心要是落在已经收下的那个大圆的盘里，就并进它，
    并把大圆的 merged 加一（打印用，让人看得出确实检出了两层）。
    返回的列表还是从大到小排的 —— materials[0] 就是"最大的那个圆"。

    这里**自己先排一遍**，不靠调用方排：判据是"小的落在大的盘里"，先收下谁就决定了
    留下谁 —— 倒着喂进来就会留下小的那个（小的盘小，34px 的圆心差照样算"落在盘里"）。
    """
    materials = sorted(materials, key=lambda k: -k.area)
    kept = []
    for m in materials:
        for i, k in enumerate(kept):
            dist = max(abs(m.center[0] - k.center[0]), abs(m.center[1] - k.center[1]))
            if dist <= k.radius * CONE_MERGE_FRAC:
                kept[i] = k._replace(merged=k.merged + 1)
                break
        else:
            kept.append(m)
    return kept


def pick_target(materials, skip_colors=()):
    """从检出的一堆圆里挑出这一帧要抓的那块：**最大的、颜色没被抓过**的那个。

    materials 是从大到小排好的（merge_cones 排过），取第一个合格的就是。
    全都在黑名单里（= 剩下的都是物料底下压着的那几个同色定位圆）就**退回最大的那个**：
    还算"看到圆"（停稳、对准照走，车不会因为滤掉一个圆就往前面拱），
    decide() 看到颜色在黑名单里会压住不发抓取。没有圆就返回 None。
    """
    for c in materials:
        if c.color not in skip_colors:
            return c
    return materials[0] if materials else None


# ==================== 串口 + 握手 ====================
class McuLink:
    """和下位机的串口链路：常开 + 后台收 0x20 状态 + 发送加锁。

    状态就是"现在能不能发指令"的依据，所以状态的新鲜度也要管：
    下位机 100ms 上报一次，超过 STATE_TIMEOUT 没收到就当状态未知，那时不发指令。
    端口不在（比如在桌面上干跑）也不影响，隔一会儿重试，只打印不发。
    """

    def __init__(self, port, baudrate, verbose=False):
        self.port = port
        self.baudrate = baudrate
        self.verbose = verbose
        self.ser = None
        self.lock = threading.Lock()
        self.state = None            # 最近一次 0x20 的状态，None = 还没收到过
        self.state_time = 0.0
        self.rx_count = {}           # 各类型收到多少帧
        self._unknown = set()        # 类型表里没有的帧，同一种只提醒一次
        self._no_port_warned = False # "串口没连上，只打印"这一句也只提醒一次
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._running = False

    def fresh_state(self):
        """返回 (状态, 是否新鲜)。不新鲜就别拿它做发送决定。"""
        if self.state is None:
            return None, False
        return self.state, (time.time() - self.state_time) <= STATE_TIMEOUT

    def _connect(self):
        try:
            self.ser = serial.Serial(
                port=self.port, baudrate=self.baudrate, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.1)
            print(f'[串口] 打开 {self.port} @ {self.baudrate}')
            self._no_port_warned = False
            return True
        except (serial.SerialException, OSError) as e:
            print(f'[串口] 打不开 {self.port}（{e}），5 秒后重试；这段时间只打印不发')
            return False

    def _loop(self):
        buf = bytearray()
        while self._running:
            if self.ser is None:
                if not self._connect():
                    for _ in range(50):          # 5 秒，等的时候也能被 stop 打断
                        if not self._running:
                            return
                        time.sleep(0.1)
                else:
                    buf.clear()
                continue
            try:
                data = self.ser.read(64)
            except (serial.SerialException, OSError) as e:
                print(f'[串口] 读失败（{e}），重新连接')
                self.ser = None
                continue
            if data:
                buf += data
                self._parse(buf)

    def _parse(self, buf):
        """按 帧头 + 类型长度白名单 切帧。没有 CRC/帧尾，白名单就是唯一的边界证据。"""
        while True:
            i = buf.find(FRAME_HEADER)
            if i < 0:
                del buf[:-1]                  # 留住最后一个字节，可能是半个帧头
                return
            if i > 0:
                del buf[:i]
            if len(buf) < 4:
                return
            mtype, mlen = buf[2], buf[3]
            if TYPE_LEN.get(mtype) != mlen:
                key = (mtype, mlen)
                if key not in self._unknown:
                    self._unknown.add(key)
                    print(f'[串口] 帧头后面是 0x{mtype:02X}/{mlen}B，不在类型表里，'
                          f'跳过（--rx-log 看原始字节）')
                del buf[0]                    # 假帧头，往后挪一个字节重新找
                continue
            if len(buf) < 4 + mlen:
                return                        # 数据段还没收全
            payload = bytes(buf[4:4 + mlen])
            del buf[:4 + mlen]
            self._dispatch(mtype, payload)

    def _dispatch(self, mtype, payload):
        self.rx_count[mtype] = self.rx_count.get(mtype, 0) + 1
        if mtype == TYPE_STATE:
            state = payload[0]
            self.state = state
            self.state_time = time.time()
            if state not in STATE_CN:
                print(f'[握手] 下位机报了未知状态 0x{state:02X}（--rx-log 看原始字节）')
        elif self.verbose:
            print(f'[串口] 收到 0x{mtype:02X} ({len(payload)}B): {payload.hex(" ").upper()}')

    def send(self, frame, what, times=1):
        """发一帧。指令(times=1)只发一次 —— 连发 = 连抓好几次。

        串口没连上就返回 False，由调用方决定怎么打印（默认就是"只打印"）。
        这里只提醒一次：tag 是 burst 5 次，每条都提醒就刷屏了。
        """
        if self.ser is None:
            if not self._no_port_warned:
                self._no_port_warned = True
                for _ in range(5):
                    print(f'[串口] {self.port} 没连上，往后只打印不发（接上会自动重连）')
            return False
        with self.lock:
            try:
                for i in range(times):
                    self.ser.write(frame)
                    self.ser.flush()
                    tag = f' 第 {i + 1}/{times} 次' if times > 1 else ''
                    print(f'[发送] {what}{tag}: {frame.hex(" ").upper()}')
                    if times > 1:
                        time.sleep(0.1)
                return True
            except (serial.SerialException, OSError) as e:
                print(f'[发送] {what} 失败: {e}')
                self.ser = None
                return False


# ==================== 主循环 ====================
def main():
    ap = argparse.ArgumentParser(description='校赛测试：tag + 物料 + 0x20/0x21 握手')
    ap.add_argument('--list', action='store_true', help='只列出摄像头就退出')
    ap.add_argument('--dev', default=MATERIAL_DEV,
                    help="物料相机：'name:板卡名片段' / 设备路径 / by-id 名字；留空 = 不开这一路")
    ap.add_argument('--tag-dev', default=TAG_DEV,
                    help="tag 相机：'name:板卡名片段' / 设备路径 / by-id 名字；留空 = 不开这一路")
    ap.add_argument('--by', choices=['id', 'path'], default='id',
                    help='id=按设备名（默认）  path=按 USB 物理口（同型号撞名时用）')
    ap.add_argument('--video-index', type=int, default=VIDEO_INDEX,
                    help='UVC 一般 0=取流 1=metadata')
    ap.add_argument('--width', type=int, default=640, help='物料相机宽（实际值启动时打印）')
    ap.add_argument('--height', type=int, default=480, help='物料相机高')
    ap.add_argument('--fps', type=float, default=15.0, help='物料相机帧率')
    ap.add_argument('--tag-width', type=int, default=640, help='tag 相机宽')
    ap.add_argument('--tag-height', type=int, default=480, help='tag 相机高')
    ap.add_argument('--tag-fps', type=float, default=15.0, help='tag 相机帧率')
    ap.add_argument('--fourcc', default=CAMERA_FOURCC,
                    help="像素格式，默认 MJPG；'' 表示不改")
    # help 里的百分号要写成 %%：argparse 对 help 还会再做一次 % 格式化，
    # 单个 % 会让 --help 直接崩掉（ValueError: unsupported format character）
    ap.add_argument('--min-radius', type=int, default=0,
                    help=f'圆最小半径 px；0=auto（帧短边的 {CIRCLE_R_MIN_FRAC * 100:.0f}%%）')
    ap.add_argument('--max-radius', type=int, default=0,
                    help=f'圆最大半径 px；0=auto（帧短边的 {CIRCLE_R_MAX_FRAC * 100:.0f}%%）')
    ap.add_argument('--hough-method', choices=['alt', 'gradient'], default=HOUGH_METHOD,
                    help='alt(默认,完美度阈值,不出假圆) / gradient(经典,能认被遮挡的圆)')
    ap.add_argument('--hough-param1', type=float, default=0, help='Canny 阈值；0=auto')
    ap.add_argument('--hough-param2', type=float, default=0,
                    help='灵敏度：alt 是完美度 0~1（调小更容易出圆），'
                         'gradient 是票数（调小更容易出圆）；0=auto')
    ap.add_argument('--circle-mindist', type=int, default=0,
                    help='两个圆心的最小间距 px；0=auto（等于最小半径）')
    ap.add_argument('--port', default=PORT, help='串口设备')
    ap.add_argument('--hb-hz', type=float, default=5.0,
                    help='运动中(0x10)发空指令当心跳的频率，0=不发')
    ap.add_argument('--cam-rot', type=int, default=CAM_ROT, choices=[0, 1, 2, 3],
                    help='物料相机是怎么转着装的：0=正装 1=逆时针90°(默认,车上现在这样) '
                         '2=180° 3=顺时针90°（画面会转正，见模块里的 CAM_ROT）')
    ap.add_argument('--tag-cam-rot', type=int, default=TAG_CAM_ROT, choices=[0, 1, 2, 3],
                    help='tag 相机同上的朝向，默认 0=正装')
    ap.add_argument('--aim-x', type=int, default=AIM_X,
                    help='对准点 x 像素；-1(默认)=画面正中')
    ap.add_argument('--aim-y', type=int, default=AIM_Y,
                    help=f'对准点 y 像素；-1=画面正中。想让准星往下移就往大改'
                         f'（默认 {AIM_Y} = 正中再往下一点；预览里画着准星）')
    ap.add_argument('--aim-tol-x', type=float, default=AIM_TOL_X,
                    help=f'**左右**的容差（默认 {AIM_TOL_X:.0f}px）：圆心横向离对准点多少'
                         f'像素以内算对准，超了就发左/右微调')
    ap.add_argument('--aim-tol-y', type=float, default=AIM_TOL_Y,
                    help=f'**前后**的容差（默认 {AIM_TOL_Y:.0f}px）：圆心纵向离对准点多少'
                         f'像素以内算对准，超了就发前/后微调。'
                         f'两个轴各管各的；都**必须 ≥ 半步**，否则那个方向一定来回动 —— '
                         f'S 看日志里"走完一步…（3cm ≈ Npx）"那行的 N，填 N/2 再多一点')
    ap.add_argument('--align-enable', action=argparse.BooleanOptionalAction,
                    default=ALIGN_ENABLE,
                    help=f'底盘微调总开关（默认 {"开" if ALIGN_ENABLE else "关"}）。'
                         f'关着的时候只报"还差多少像素、该往哪边挪"，一条微调都不发')
    ap.add_argument('--align-interval', type=float, default=ALIGN_INTERVAL,
                    help='两条底盘微调指令**至少**隔几秒（默认 0.6）。'
                         '这不是节拍：线上节拍是下位机给的（发完等它回到 0x01，'
                         '见 --align-return-timeout）；它只是兜底的最小间隔，'
                         '离线干跑时才是唯一节拍')
    ap.add_argument('--align-return-timeout', type=float, default=ALIGN_RETURN_TIMEOUT,
                    help='发完一发微调后最多等几秒让它回到 0x01（默认 3.0）。'
                         '协议里一发往返 0.4~0.5s；超时 = 这发没生效，'
                         '这一站就不再挪车（不然会一直空等）')
    ap.add_argument('--align-new-stop-gap', type=float, default=ALIGN_NEW_STOP_GAP,
                    help=f'"车自己跑到新的一站"的判据：离开 0x01 超过这么久才回来'
                         f'（默认 {ALIGN_NEW_STOP_GAP:.0f}s）。微调次数、抓过的颜色、'
                         f'tag 顺序都在这儿重置。**要卡在"抓取动作报的 0x10"和"真开走'
                         f'一段路"之间**：太短，抓完回到 0x01 那下就被当成新的一站'
                         f'（黑名单白清，2026-09-18 踩过）；太长，车开到下一个抓取点'
                         f'也不重置（同一批的颜色会一直压着不抓）')
    ap.add_argument('--align-max', type=int, default=ALIGN_MAX,
                    help=f'一次停稳里最多发几条微调（默认 {ALIGN_MAX}）。用满了就**不再挪车、'
                         f'照抓**（抓偏一点也比整轮卡着不抓好），原因会打出来；'
                         f'0=不限（不建议：认成假圆时会一路挪出去）')
    ap.add_argument('--search-interval', type=float, default=SEARCH_INTERVAL,
                    help=f'等待抓取时连续这么久没看到圆就往前找一次'
                         f'（默认 {SEARCH_INTERVAL:.0f}s，一次还是 3cm）')
    ap.add_argument('--search-max', type=int, default=SEARCH_MAX,
                    help=f'一次停稳里最多往前找几次（默认 {SEARCH_MAX}，0=不限）。'
                         f'找满还是没圆就不动了；这几发也算进 --align-max 的总预算')
    ap.add_argument('--color-policy', dest='color_policy', choices=('once', 'repeat'),
                    default=COLOR_POLICY,
                    help=f'同一个颜色能不能重复抓（默认 {COLOR_POLICY}）。'
                         f'once = 一个颜色抓过一次就不再抓：物料底下压着同色的定位圆，'
                         f'物料抓走了它还在，再抓就是空抓；'
                         f'repeat = 同色的圆照抓（老行为，同一站又来个同色物料时用）。'
                         f'车跑到新的一站自动清空')
    # 老写法保留：命令行里已经写过 --color-blacklist / --no-color-blacklist 的不用改
    ap.add_argument('--color-blacklist', dest='color_policy', action='store_const',
                    const='once', default=COLOR_POLICY,
                    help='老写法，等于 --color-policy once')
    ap.add_argument('--no-color-blacklist', dest='color_policy', action='store_const',
                    const='repeat', default=COLOR_POLICY,
                    help='老写法，等于 --color-policy repeat')
    ap.add_argument('--tag-order', action=argparse.BooleanOptionalAction,
                    default=TAG_ORDER_ENABLE,
                    help=f'按 tag 前三位数字的顺序抓（默认 {"开" if TAG_ORDER_ENABLE else "关"}）。'
                         f'规则：二维码 4 组三位数、+ 连接，第 1 组 = 第一批物料的颜色和顺序'
                         f'（452 = 先绿再黑后黄）。关掉 = 谁大抓谁（老行为）')
    ap.add_argument('--order-group', type=int, default=TAG_ORDER_GROUP,
                    help=f'用 tag 里第几组三位数当抓取顺序（默认 {TAG_ORDER_GROUP}）：'
                         f'第 1 组 = 第一批物料，第 3 组 = 第二批（第 2/4 组是放置位置，'
                         f'不是颜色）。现在两批都用这一组，要一批一组说一声')
    ap.add_argument('--order-miss-policy', choices=('biggest', 'wait'),
                    default=TAG_ORDER_MISS_POLICY,
                    help=f'按顺序该抓的颜色**画面里没有**时怎么办（默认 '
                         f'{TAG_ORDER_MISS_POLICY}）。biggest = 等 --order-miss-wait 秒'
                         f'还没有就抓画面里最大的那个没抓过的（跟"微调发满就照抓"一个口径）；'
                         f'wait = 一直等它出现（严格按顺序，颜色认不出来就整轮卡住）')
    ap.add_argument('--order-miss-wait', type=float, default=TAG_ORDER_MISS_WAIT,
                    help=f'--order-miss-policy biggest 时等几秒才放弃（默认 '
                         f'{TAG_ORDER_MISS_WAIT}）')
    ap.add_argument('--still-time', type=float, default=STILL_TIME,
                    help='"等待抓取指令"时，圆要连续静止这么多秒才准动（默认 0.5）；'
                         '0=一看到圆就动（微调和对准都跟着这个门槛走）')
    ap.add_argument('--still-tol', type=float, default=STILL_TOL,
                    help='圆心挪动超过这么多像素就算还在动（默认 6）')
    ap.add_argument('--grab-cooldown', type=float, default=GRAB_COOLDOWN,
                    help='两次"抓取一次"之间至少隔几秒')
    ap.add_argument('--dry-run', action='store_true',
                    help='只打印不发给下位机。默认是**真发**；串口没连上时自动只打印')
    ap.add_argument('--rx-log', action='store_true', help='打印下位机发来的每一帧')
    ap.add_argument('--no-preview', action='store_true', help='不开预览窗口')
    # 物料相机(M2)的曝光：默认按文件顶上 MAT_* 那几个常量来（默认钉 hue=0 + 压一点亮度）。
    # 这几个开关**只作用在物料那一路**，tag 相机不动（它自己认得挺好）。
    # 想临时压一格就 --brightness -20，不用改文件。
    ap.add_argument('--hue', type=float, default=MAT_HUE,
                    help=f'物料相机色相，-2000~2000（默认 {MAT_HUE}）。钉住是为了 PC 和车上'
                         f'看到同一个颜色；文件顶上 MAT_HUE 填 None 就是不碰')
    ap.add_argument('--brightness', type=float, default=MAT_BRIGHTNESS,
                    help=f'物料相机亮度，范围 -64~64（默认 {MAT_BRIGHTNESS}，1 格 ≈ 1 个 V 亮度）。'
                         f'**这颗模组上唯一真管用的曝光旋钮**')
    ap.add_argument('--gain', type=float, default=MAT_GAIN,
                    help='物料相机增益；这颗模组没有 gain 控制，填了会打"没设上"')
    ap.add_argument('--exposure', type=float, default=MAT_EXPOSURE,
                    help='物料相机曝光（绝对）；**这颗模组上是死的**，填了没用')
    ap.add_argument('--auto-exposure', choices=['auto', 'manual'], default=MAT_AUTO_EXPOSURE,
                    help=f'物料相机自动曝光（默认 {MAT_AUTO_EXPOSURE}）。'
                         f'注：这颗模组上 auto/manual 一个样，改它没用，改 --brightness')
    args = ap.parse_args()

    if args.list:
        list_cameras()
        return 0

    # --color-policy 落成一个布尔：「滤掉抓过的颜色」还是「照抓」。后面三处
    # （选目标 pick_target、decide 里压住 0x02、真发出去之后记黑名单）都看它，
    # 别再各比一次字符串。布尔在这块儿先算出来，是因为开机的提示就要用它
    skip_grabbed = (args.color_policy == 'once')

    # ---- 开两路相机，哪路挂了另一路照跑 ----
    workers = []
    for role, spec, w, h, fps, rot in (
            ('material', args.dev, args.width, args.height, args.fps, args.cam_rot),
            ('tag', args.tag_dev, args.tag_width, args.tag_height, args.tag_fps,
             args.tag_cam_rot)):
        if not spec:
            print(f'[相机] {role} 这一路没给设备，不开')
            continue
        # 设备只在线程里解析（这里不解析）：线程第一轮把诊断全打出来，之后每
        # CAM_RETRY_INTERVAL 秒重新解析 + 重开一次（插晚了、换 USB 口、节点号变了
        # 都能自己接上），重试时只打一行提醒。在这里也解析一遍的话，开机那堆
        # "分不清是哪块板卡"会原样打两遍。
        hp = {'method': args.hough_method, 'param1': args.hough_param1,
              'param2': args.hough_param2, 'min_dist': args.circle_mindist,
              'min_radius': args.min_radius, 'max_radius': args.max_radius}
        # 曝光只往物料(M2)那一路打；tag 相机一个控制都不碰（见顶部"物料相机(M2)的曝光"）
        ctrls = ({k: v for k, v in (('hue', args.hue), ('brightness', args.brightness),
                                    ('gain', args.gain), ('exposure', args.exposure),
                                    ('auto_exposure', args.auto_exposure)) if v is not None}
                 if role == 'material' else {})
        workers.append(CameraWorker(role, '', w, h, fps, args.fourcc, hp=hp, ctrls=ctrls,
                                    cam_rot=rot, spec=spec,
                                    video_index=args.video_index, by=args.by))
    if not workers:
        print('[相机] 两路都没给设备（--dev / --tag-dev），没有相机可开，退出')
        return 1

    for wk in workers:
        wk.start()
    time.sleep(0.5)          # 等它们把"打不开/读不到帧"报出来
    # 两路都没起来也**不退出**：线程每 CAM_RETRY_INTERVAL 秒重试一轮，插上就自己接上
    # （串口没连上也是这么办的）。起不来的那一路线程自己会打显著提醒，这里就不再重复一遍。
    tag_wk = next((wk for wk in workers if wk.role == 'tag'), None)
    mat_wk = next((wk for wk in workers if wk.role == 'material'), None)

    if mat_wk and mat_wk.up():
        _m, hp = mat_wk.material_detail()
        print(f'[物料] 判据：Hough 找圆，检出圆的就算物料。方法 {hp.get("method")}'
              f'（dp={hp.get("dp")} param1={hp.get("param1")} param2={hp.get("param2")}），'
              f'半径 {hp.get("min_radius")}~{hp.get("max_radius")}px，'
              f'圆心间距≥{hp.get("min_dist")}px')
        print('[物料] 颜色只是附带信息（圆内占比最高的那种），认不出颜色不影响判定')
        print('[物料] 每行的 HSV 是**整个圆内的平均**，跟 HSV_RANGES 那张表无关 —— '
              '照着它调表里的范围就行')
    else:
        print('[物料] 物料相机现在没起来，"抓取一次"那一路不会触发（起来了会自己接上）')
    if tag_wk and tag_wk.up():
        print(f'[Tag] tag 不看状态，识别到就发（burst {TAG_SEND_TIMES} 次）')
    else:
        print('[Tag] tag 相机现在没起来，二维码那一路不会触发（起来了会自己接上）')
    print('[握手] 0x20 状态 -> 0x21 指令: 等待二维码(00) 发 01; '
          '等待抓取指令(01) 且圆停稳 -> 没对准发微调(30/31/32/33)、对准了发 02; '
          '运动中(10) 不发'
          + (f'(心跳 {args.hb_hz:.0f}Hz)' if args.hb_hz > 0 else ''))
    # 转正后的画面宽高：CAM_ROT 不是 0 时和设的宽高是反的。这里只是启动时打个预告，
    # 真正用的对准点每帧按**实际帧**的尺寸算（相机不一定按你要的分辨率给）
    fh, fw = (args.width, args.height) if args.cam_rot % 4 else (args.height, args.width)
    aim_show = resolve_aim(args.aim_x, args.aim_y, fh, fw)
    print(f'[对准] 对准点 {aim_show}（{"画面正中" if args.aim_x < 0 and args.aim_y < 0 else "指定的"}），'
          f'容差 左右 {args.aim_tol_x:.0f}px / 前后 {args.aim_tol_y:.0f}px，'
          f'一次停稳最多挪 {args.align_max if args.align_max > 0 else "无限"} 步')
    print(f'[对准] 微调节拍跟着下位机走：一发一发来，等它报回 0x01 才发下一发'
          f'（等不到就 {args.align_return_timeout:.0f}s 超时停手）；'
          f'另有 {args.align_interval:.1f}s 的最小间隔兜底；'
          f'离开 0x01 超过 {args.align_new_stop_gap:.0f}s 才回来 = 车跑到新的一站'
          f'（微调次数/抓过的颜色/tag 顺序都在这儿重置）')
    print(f'[朝向] 物料相机 {CAM_ROT_CN[args.cam_rot % 4]}，'
          f'tag 相机 {CAM_ROT_CN[args.tag_cam_rot % 4]}')
    print(f'[发送] {"只打印（--dry-run）" if args.dry_run else "真发"}')
    # 这条是这次抓取会怎么判的关键，开机就说清楚用的是哪一种（--color-policy）
    print('[物料] 颜色重复抓：' + ('不许 —— 一个颜色抓过一次就不再抓（防底下那个同色定位圆的空抓），'
                                   '车跑到新的一站清空'
                                   if skip_grabbed else
                                   '允许 —— 同色的圆照抓（--color-policy repeat）'))
    # 这一轮"抓哪一个"的总口径：按 tag 顺序还是谁大抓谁
    if args.tag_order:
        print(f'[顺序] 按 tag 第 {args.order_group} 组三位数字的顺序抓：认到 tag 就把顺序'
              f'排出来，每抓走一个往前挪一位，三个抓完就不再抓（等新的一站/新 tag）。'
              f'该抓的颜色画面里没有时 '
              + ('一直等着不抓（--order-miss-policy wait）'
                 if args.order_miss_policy == 'wait' else
                 f'先等 {args.order_miss_wait:.1f}s，还没有就抓最大的那个没抓过的'
                 f'（--order-miss-policy biggest）'))
    else:
        print('[顺序] 不按 tag 顺序抓（--no-tag-order）：谁大抓谁')

    link = McuLink(args.port, BAUDRATE, verbose=args.rx_log)
    link.start()

    # 有显示器才开预览窗口；车上/无 DISPLAY 时自动关掉，不会崩
    show = bool(os.environ.get('DISPLAY')) and not args.no_preview
    if show:
        print('[预览] 每路相机一个窗口（--no-preview 关掉）')

    last_tag = None          # 上一次的 tag 内容，变了才打印，别刷屏
    last_colors = None       # 上一次的圆（数量/位置/颜色），变了才打印
    last_state = None        # 上一次收到的 0x20 状态
    last_skip = None         # 上一次"想发但被状态挡住"的原因，变了才打印
    order_queue = None       # tag 前三位数字给的抓取顺序（list）。None = 没按顺序抓
                             # （没认到 tag / --no-tag-order）；非空 = 队首那个颜色先抓；
                             # 空 list = 这一批三个都抓完了，画面里剩下的不再抓
    order_miss_since = None  # 队首那个颜色"画面里没有"是从什么时候开始的；看到就清
    order_miss_reason = ''   # 为什么算没有（压根没这个颜色 / 这个颜色这一站抓过了）
    qr_text = None           # 认到的二维码（留着，等到 0x20 说等待二维码时发指令）
    qr_done_episode = None   # 本轮"等待二维码"里已经发过 01 了
    last_grab = 0.0          # 上次发"抓取一次"的时刻
    still_ref = None         # 上次看到的那个圆 (center, radius)，用来判断动没动
    still_since = None       # 最后一次"看到圆在动"的时刻；None = 现在没圆
    no_circle_since = None   # 最后一次"一个圆都没看到"是从什么时候开始的；看到圆就清掉。
                             # 够 SEARCH_INTERVAL 就往前的找一发（见"找不到圆就往前找"）
    search_count = 0         # 这一站已经往前找了几发；新的一站从头算
    grabbed_colors = set()   # 已经真发过抓取的那些颜色。skip_grabbed 时里面的颜色不再抓 ——
                             # 物料底下压着一个同色的圆，物料没了它还在，再抓就是空抓
                             # （清空时机见 COLOR_POLICY）
    target_color = None      # 这一帧挑中要抓的那个圆是什么颜色（aim/still 看的也是它）
    aim = None               # 对准点 (x, y)，按这一帧的实际尺寸算；None = 还没出帧
    aim_err = None           # 大圆圆心 - 对准点 = (ex, ey)；None = 没有圆
    # 微调节拍器：发完一发要等它回到 0x01 才准发下一发（USART1_Nudge_Protocol.md §四）
    pacer = NudgePacer(args.align_interval, args.align_return_timeout,
                       args.align_new_stop_gap)
    offline_align_warned = False   # "帧没真发出去、退回定时节拍"这句只提醒一次
    align_sent_err = None    # 发那条微调时的误差 (ex, ey)，等它走完拿来回量"一步多少像素"
    align_sent_cmd = None    # 那条微调的方向（0x30~0x33），用来定量的是哪个轴
    last_hb = 0.0            # 上次发心跳的时刻
    last_cam_err = {}        # {role: 上次看到的 error}。相机开起来/掉下去只在**变的时候**
                             # 说一句，别每 2s 刷一遍（线程自己每轮重试已经打显著提醒了）
    no_state_warned = False
    state_episode = 0
    frames = 0
    t0 = time.time()

    def decide(state, now):
        """按 0x20 状态决定要发哪条 0x21 指令；返回 (cmd, 被挡的原因)。"""
        if state == STATE_WAIT_QR:
            if qr_text is None:
                return None, '等待二维码，但还没认到 tag'
            if qr_done_episode == state_episode:
                return None, None            # 这一轮已经发过 01 了，等下一次等待
            return CMD_QR_DONE, None
        if state == STATE_WAIT_GRAB:
            if not materials:
                # 车停偏了、物料压根没进画面：往前拱一小步找找（一次还是 3 cm，
                # 走的是同一条 0x30~0x33 通道，所以节拍照样听 pacer 的，不能连发）。
                # 往前找也是在挪车，所以和微调共用一个总开关：ALIGN_ENABLE=False
                # （下位机那套还没烧上时）就一条都不发，也不往前找。
                if not args.align_enable:
                    return None, ('等待抓取指令，没看到圆；底盘微调关着'
                                  '（ALIGN_ENABLE=False），不往前找')
                if args.search_max > 0 and search_count >= args.search_max:
                    return None, (f'等待抓取指令，往前找了 {search_count} 次还是没看到圆，'
                                  f'不再往前了（--search-max）；不盲抓，就这么等着')
                if no_circle_since is not None and now - no_circle_since >= args.search_interval:
                    can, why = pacer.ready(now)
                    if can:
                        return CMD_ADJUST_FWD, None
                    if why is not None:
                        return None, why          # 上一发卡住了，别再往前拱
                    return None, None             # 正在走 / 还没到最小间隔，等它
                return None, (f'等待抓取指令，还没看到圆'
                              f'（{args.search_interval:.0f}s 没圆就往前找，'
                              f'已找 {search_count}/{args.search_max}）')
            if aim is None or aim_err is None:
                return None, None            # 还没出帧，对准点算不出来

            def want_grab(reason):
                """该发抓取了 —— 但下面三种情况先压住不发。

                压住只是不发 0x02：上面的停稳、对准那几关**照走**（车还是会对准它），
                所以拿到的不是"没物料"，不会去触发往前找。那些圆就让它那么放着。
                """
                # (1) tag 顺序里的三个都抓完了：这一批完了。画面里剩下的不是这一批的
                # 物料（多半是定位圆或者下一批），再抓就是多抓
                if order_queue is not None and not order_queue:
                    return None, ('tag 顺序里的物料都抓完了（这一批完了），画面里剩下的'
                                  '不再抓；车跑到新的一站或者重新认到 tag 才从头来')
                # (2) 按顺序该抓的颜色还没出现（--order-miss-policy biggest 等满
                # --order-miss-wait 秒就放行，下面照抓画面里最大的那个；wait 就一直等）
                if order_miss_since is not None:
                    waited = now - order_miss_since
                    if args.order_miss_policy == 'wait' or waited < args.order_miss_wait:
                        return None, (f'{order_miss_reason}；tag 顺序是 '
                                      + ' → '.join(COLOR_CN.get(c, c) for c in order_queue)
                                      + f'，该抓{COLOR_CN.get(order_color, order_color)}色，'
                                      f'先压住不发 0x02'
                                      f'（--order-miss-policy {args.order_miss_policy}'
                                      + (f'，等满 {args.order_miss_wait:.1f}s 还没有就抓'
                                         f'画面里最大的那个没抓过的'
                                         if args.order_miss_policy == 'biggest' else '') + '）')
                    reason = ((reason + '；') if reason else '') + (
                        f'{order_miss_reason}，等满 {args.order_miss_wait:.1f}s 了，'
                        f'照抓画面里最大的那个没抓过的'
                        f'（--order-miss-policy biggest；顺序要的是'
                        f'{COLOR_CN.get(order_color, order_color)}色）')
                # (3) 这个颜色这一站抓过（--color-policy once）
                if skip_grabbed and target_color in grabbed_colors:
                    return None, (f'{COLOR_CN.get(target_color, target_color)}色已经抓过一次了，'
                                  f'画面里这个同色的圆是它底下压着的那个，不抓'
                                  f'（--color-policy once；改成 repeat 就照抓）')
                return CMD_GRAB, reason

            if now - last_grab < args.grab_cooldown:
                return None, None
            if still_since is None or now - still_since < args.still_time:
                return None, '等待抓取指令，圆还在动（等它停稳再对准/抓）'
            ex, ey = aim_err
            # 左右和前后各有各的容差（--aim-tol-x / --aim-tol-y），
            # 谁超了自己的容差就修谁；都在容差里才算对准。见 align_cmd_for。
            want = align_cmd_for(ex, ey, args.aim_tol_x, args.aim_tol_y)
            # 底盘微调关着（ALIGN_ENABLE=False）时，对准这一关整个跳过：
            # 圆停稳了就直接抓，差多少像素只打在状态行里看看，不影响发什么。
            if want is not None and args.align_enable:
                # 停稳了但没对准：能挪就挪一发。微调是**动作**，一次只发一条，
                # 而且**发完要等它回到 0x01 才准发下一发** —— 下位机挪的时候报 0x10，
                # 那期间发的会被静默丢弃（USART1_Nudge_Protocol.md §四）。
                # 节拍由 pacer 管，这里只问它一句"现在能发吗"。
                #
                # 挪不动了就**照抓**（微调发满 --align-max、或上一发卡住超时）：
                # 抓偏一点也比整轮卡在这儿不抓好。原因照样打出来。
                can, why = pacer.ready(now)          # why 有值 = 挪不了了
                full = args.align_max > 0 and pacer.count >= args.align_max
                if can and not full:
                    return want, None                # want 已经按各自的容差挑好轴了
                if why is not None:                  # 上一发卡住了（没等到 0x01）
                    return want_grab(why + '；**照抓**')
                if pacer.pending:
                    return None, None                # 上一发还在走，等它落地（车在动，别抓）
                if full:
                    return want_grab(f'还差 ({ex:+.0f},{ey:+.0f})px，微调发满 '
                                     f'{args.align_max} 条，不再挪了；**照抓**')
                return None, None                    # 还没到最小间隔，等一下
            return want_grab(None)
        if state == STATE_MOVING:
            if args.hb_hz > 0 and now - last_hb >= 1.0 / args.hb_hz:
                return CMD_IDLE, None
            return None, None
        return None, f'未知状态 0x{state:02X}'

    while True:
        now = time.time()

        # ---- tag：不受状态影响，看到就发 ----
        tag_text = None
        if tag_wk:
            _f, tag_text, _m = tag_wk.snapshot()
        if tag_text:
            if tag_text != last_tag:
                # 同一个 tag 停在画面里会**连续认到好几十帧**，所以"认到新的 tag"只能看
                # 内容变没变 —— 每帧都重置顺序的话，指针永远停在第一位走不动
                print(f'[Tag] 识别到: {tag_text}')
                if args.tag_order:
                    order_queue = parse_tag_order(tag_text, args.order_group)
                    order_miss_since = None
                else:
                    order_queue = None
            qr_text = tag_text
            qr_done_episode = None           # 内容变了，允许再通知一次
        
            try:
                f = build_tag_frame(tag_text)
                if args.dry_run:
                    print(f'[Tag] 只打印不发（--dry-run）: {f.hex(" ").upper()}'
                        f'（burst {TAG_SEND_TIMES} 次）')
                elif not link.send(f, 'tag 0x03', times=TAG_SEND_TIMES):
                    print(f'[Tag] 串口没连上，只打印: {f.hex(" ").upper()}'
                        f'（burst {TAG_SEND_TIMES} 次）')
            except ValueError as e:
                print(f'[Tag] {e}')
        last_tag = tag_text

        # ---- 物料：找圆 ----
        materials = []
        mat_frame = None
        aim = None
        if mat_wk:
            mat_frame, _t, materials = mat_wk.snapshot()
            if mat_frame is not None:
                # 对准点按**这一帧的实际尺寸**算：转了画面宽高是反的，
                # 而且相机不一定按你要的分辨率给。识别的就是转正后的帧，对得上。
                aim = resolve_aim(args.aim_x, args.aim_y,
                                  mat_frame.shape[0], mat_frame.shape[1])
        # 圆的数量/位置/颜色/HSV 变了才打印，不然每帧刷屏。
        # 黑名单也进签名：抓过之后那几个圆还是老样子，但"压着不抓"得让人看见
        sig = (tuple((c.color, c.hsv, c.radius, c.center) for c in materials),
               tuple(sorted(grabbed_colors, key=str)))

        if sig != last_colors:
            if materials:
                detail = '  '.join(
                    ((f'{COLOR_CN[c.color]}({COLOR_CODE[c.color]}) {c.fill:.0%}'
                      if c.color else '颜色认不出')
                     + (f' HSV={c.hsv[0]},{c.hsv[1]},{c.hsv[2]}' if c.hsv else ' HSV=-')
                     + f' r={c.radius} @{c.center}'
                     # 圆台会检出大小两层，合并成一块了；说一声省得以为漏了一个圆
                     + (f'（圆台：并掉了 {c.merged} 个同心圆）' if c.merged else '')
                     # 抓过的颜色：说明它为什么一直在画面里却不抓
                     + ('（这个颜色抓过了，压着不抓）' if c.color in grabbed_colors else ''))
                    for c in materials)
                print(f'[物料] 检出 {len(materials)} 块物料: {detail}')
            else:
                print('[物料] 画面里没有圆')
        last_colors = sig

        # ---- 圆停稳了没 + 离对准点还差多少 ----
        # 只看那一块要抓的圆（= 最大的、颜色没被抓过的那个，见 pick_target），
        # 它就是要抓的那块物料。still_since = 最后一次"看到它在动"的时刻；
        # 它离现在够久 = 停稳了。微调挪一步车之后画面会动一下 → 这里自动重新计时，
        # 所以两条微调之间天然隔开一段（想挪快点就调小 --still-time）。
        order_color, order_ok, order_miss_reason = None, False, ''
        if materials:
            # 这一帧该抓哪一块：有 tag 顺序就按顺序挑（顺序里那个颜色在画面里有好几个
            # 就抓最大的那个），没有顺序（没认到 tag / --no-tag-order）才是老的"谁大抓谁"。
            if order_queue:
                order_color = order_queue[0]
                cands = [c for c in materials if c.color == order_color]
                if skip_grabbed:
                    # 这一站已经抓过这个颜色了（tag 里同一个颜色出现了两次）：
                    # 画面里还剩的同色圆是物料底下压着的定位圆，不算数
                    cands = [c for c in cands if c.color not in grabbed_colors]
                hit = pick_target(cands, ())       # 空表 -> None
                if hit is not None:
                    big, order_ok = hit, True
                else:
                    order_miss_reason = (
                        f'{COLOR_CN.get(order_color, order_color)}色这一站已经抓过一次了'
                        f'（tag 里同一个颜色出现了两次？）'
                        if any(c.color == order_color for c in materials) else
                        f'画面里没有{COLOR_CN.get(order_color, order_color)}色的圆')
            if not order_ok:
                big = pick_target(materials, grabbed_colors if skip_grabbed else ())
            target_color = big.color
            if still_ref is None:
                still_since = now                      # 第一次看到，从现在开始计时
            else:
                move = max(abs(big.center[0] - still_ref[0][0]),
                           abs(big.center[1] - still_ref[0][1]))
                dr = abs(big.radius - still_ref[1])
                if move > args.still_tol or dr > max(2, still_ref[1] * STILL_R_FRAC):
                    still_since = now                  # 动了，重新计时
            still_ref = (big.center, big.radius)
            aim_err = ((big.center[0] - aim[0], big.center[1] - aim[1])
                       if aim is not None else None)
            no_circle_since = None                     # 看到圆了，"多久没圆"从零数
            # 顺序里该抓的那个颜色没看到：从第一次没看到那一刻开始等
            # （order-miss-policy biggest 等满几秒就抓大的那个；wait 就一直等）。
            # 看到了就清零 —— 等一下再挪车的时候画面会动，别把它当成"又开始等了"
            if order_color is not None and not order_ok:
                if order_miss_since is None:
                    order_miss_since = now
            else:
                order_miss_since = None
        else:
            still_ref, still_since = None, None        # 没圆就当没停稳，从零开始算
            aim_err = None
            target_color = None
            order_miss_since = None                    # 一个圆都没有，不是"缺某个颜色"
            if no_circle_since is None:
                no_circle_since = now                  # 从"一个圆都没看到"开始计时

        # ---- 握手：按 0x20 的状态发 0x21 指令 ----
        state, fresh = link.fresh_state()
        if state != last_state:
            if state is None:
                if not no_state_warned:
                    print('[握手] 还没收到下位机的 0x20 状态，先不发指令')
                    no_state_warned = True
            else:
                state_episode += 1
                print(f'[握手] 下位机状态: 0x{state:02X} {STATE_CN.get(state, "?")}')
            last_state = state

        if fresh:
            # 微调节拍跟着状态走：'done' = 上一发走完了，'stuck' = 发出去没动静，
            # 'station' = 车自己跑到新的一站（微调次数从头算）
            ev = pacer.note(state, now)
            if ev == 'station':
                # 车自己跑到新的一站：往前找的次数从头算，"多久没圆"也重新数
                # （不能拿上一站的时刻接着数，不然一到站就立刻往前拱）
                search_count = 0
                no_circle_since = None
                # 上一站抓过的颜色重新可抓 —— 底下的定位圆跟着车走了，这一站是新的物料。
                # 为什么要等这个事件、不等"回到 0x01"或者"报 0x00"，见 COLOR_POLICY
                if grabbed_colors:
                    for _ in range(5):
                        print(f'[物料] 车跑到新的一站，颜色过滤清空（本来不抓：'
                            f'{"、".join(COLOR_CN.get(c, str(c)) for c in sorted(grabbed_colors, key=str))}）')
                    grabbed_colors.clear()
                # tag 顺序也从头来：新的一站是新的物料，还要按同一个顺序抓三个
                # （第二批要按第 3 组抓的话，见 TAG_ORDER_GROUP）
                if args.tag_order and qr_text is not None:
                    if order_queue is not None:
                        print('[顺序] 车跑到新的一站，tag 顺序从头来')
                    order_queue = parse_tag_order(qr_text, args.order_group)
                    order_miss_since = None
            elif ev == 'stuck':
                print(f'[微调] 上一发发出去 {args.align_return_timeout:.0f}s 没等到下位机'
                      f'回到 0x01，这一站不再挪车了'
                      f'（状态一直是 0x{state:02X}；对着 USART1_Nudge_Protocol.md §七 查）')
            elif ev == 'done':
                # 走完一步正好量一次"3cm 是多少像素"，顺便看容差够不够
                _note = nudge_step_note(align_sent_err, aim_err, align_sent_cmd,
                                        args.aim_tol_x, args.aim_tol_y)
                if _note:
                    print(_note)

            cmd, skip = decide(state, now)
            if cmd is not None:
                try:
                    f = build_cmd_frame(cmd)
                    shown = (f'状态 {STATE_CN.get(state, "?")} → '
                             f'{f.hex(" ").upper()}（{CMD_CN[cmd]}）')
                    # 这一帧到底写进串口了没有。微调的节拍要看它：
                    # 真发出去了 → 等下位机把状态切到 0x10 再切回 0x01，才算这一发走完；
                    # 没发出去（--dry-run / 串口没连上）→ 等不到任何回执，
                    # 再等下去只会超时报"没生效"，所以那种情况退回按 ALIGN_INTERVAL 定时
                    sent_real = False
                    if args.dry_run:
                        print(f'[握手] {shown}  [只打印，--dry-run]')
                    elif link.send(f, f'指令 0x21 {CMD_CN[cmd]}'):
                        sent_real = True
                    else:
                        print(f'[握手] {shown}  [串口没连上，只打印]')
                    if cmd == CMD_QR_DONE:
                        qr_done_episode = state_episode
                    elif cmd == CMD_GRAB:
                        last_grab = now
                        # tag 顺序往前挪一位。**跟黑名单一个口径：不看 sent_real** ——
                        # 干跑时状态机走的路要和线上一样，不然桌面上测出来的顺序对不上车
                        if order_queue:
                            wanted = order_queue.pop(0)
                            got = COLOR_CN.get(target_color, target_color) \
                                if target_color else '认不出颜色'
                            if target_color == wanted:
                                print(f'[顺序] 抓走一个：顺序里的{COLOR_CN[wanted]}色，'
                                      f'剩 {len(order_queue)} 个：'
                                      + (' → '.join(COLOR_CN.get(c, c) for c in order_queue)
                                         if order_queue else '（这一批完了）'))
                            else:
                                print(f'[顺序] ⚠ 抓走一个，但**不是顺序里的颜色**：'
                                      f'顺序要的是{COLOR_CN[wanted]}色，实际抓的是{got}；'
                                      f'剩 {len(order_queue)} 个：'
                                      + (' → '.join(COLOR_CN.get(c, c) for c in order_queue)
                                         if order_queue else '（这一批完了）'))
                        # 这个颜色抓过了，底下压着的那个同色的圆别再抓（--color-policy once）。
                        # 不看 sent_real：跟 last_grab 一个口径 —— 干跑时状态机走的路
                        # 要和线上一样，不然桌面上测出来的行为对不上车
                        if skip_grabbed and target_color is not None \
                                and target_color not in grabbed_colors:
                            for color in grabbed_colors:
                                print(f'[物料] 颜色过滤：{COLOR_CN.get(color, color)}色已抓过')
                            grabbed_colors.add(target_color)
                            for _ in range(5):
                                print(f'[物料] {COLOR_CN.get(target_color, target_color)}色记下了')
                    elif cmd == CMD_IDLE:
                        last_hb = now
                    elif cmd in ADJUST_CMDS:
                        pacer.sent(now, real=sent_real)
                        align_sent_err = aim_err     # 挪之前的误差，走完了好量一步多大
                        align_sent_cmd = cmd         # 发的是哪个方向（定量的是哪个轴）
                        no_circle_since = None       # 车要动了，画面会变，"多久没圆"重新数
                        if not materials:
                            search_count += 1        # 画面里没圆 → 这一发是"往前找"
                        if not sent_real and not offline_align_warned:
                            offline_align_warned = True
                            print(f'[微调] 帧没真发出去，节拍退回按 {args.align_interval:.1f}s '
                                  f'定时（线上真发时是等下位机回 0x01）')
                except ValueError as e:
                    print(f'[握手] {e}')
            if skip != last_skip and skip is not None:
                # skip 有话说但 cmd 也是有的（照抓那种）：别写"先不发"，它已经发了
                print(f'[握手] {skip}' if cmd is not None else f'[握手] 先不发: {skip}')
            last_skip = skip

        # ---- 预览：每路一个窗口（没出帧的那路跳过，窗口开出来就是空的没意义）----
        if show:
            for wk in workers:
                frame, t_text, mats = wk.snapshot()
                if frame is None:
                    continue
                view = frame.copy()
                if wk.role == 'material':
                    # 检出的圆都画出来；认得出颜色就用自己的颜色画，认不出画白圈
                    for c in mats:
                        col = COLOR_BGR.get(c.color, (255, 255, 255))
                        cv2.circle(view, c.center, c.radius, col, 3)
                        label = (f'{c.color} {COLOR_CODE.get(c.color, "?")} {c.fill:.0%}'
                                 if c.color else f'? r={c.radius}')
                        if c.hsv:
                            label += f' HSV {c.hsv[0]},{c.hsv[1]},{c.hsv[2]}'
                        cv2.putText(view, label,
                                    (max(0, c.center[0] - c.radius),
                                     max(12, c.center[1] - c.radius - 6)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
                    # 对准点 + 这一刻要挪的方向。方向反了就是装反了，先看这里。
                    if aim is not None:
                        cv2.drawMarker(view, aim, (0, 255, 255), cv2.MARKER_CROSS, 26, 2)
                        # 容差框：横竖两个容差不一样，圆圈表达不了，画成矩形
                        # （半宽 = 左右容差，半高 = 前后容差）。圆心落进框里 = 对准了。
                        cv2.rectangle(view,
                                      (int(aim[0] - args.aim_tol_x), int(aim[1] - args.aim_tol_y)),
                                      (int(aim[0] + args.aim_tol_x), int(aim[1] + args.aim_tol_y)),
                                      (0, 255, 255), 1)
                        if aim_err is not None:
                            ex, ey = aim_err
                            # 箭头从准星(对准点)指向物料圆心 = **车要往哪边挪**
                            # （车往右挪，物料就在画面里往左滑向准星，方向跟箭头一致）
                            cv2.arrowedLine(view, aim, (int(aim[0] + ex), int(aim[1] + ey)),
                                            (0, 255, 255), 2, tipLength=0.12)
                            want = align_cmd_for(ex, ey, args.aim_tol_x, args.aim_tol_y)
                            if want is not None:
                                txt = f'-> {ADJUST_EN[want]} ({ex:+.0f},{ey:+.0f})'
                                tcol = (0, 255, 255)
                            else:
                                txt, tcol = 'AIMED: grab', (0, 255, 0)
                            cv2.putText(view, txt, (8, view.shape[0] - 12),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, tcol, 2)
                    # tag 顺序还剩哪几个（数字好认，中文 putText 画不了）
                    if order_queue:
                        cv2.putText(view, 'order '
                                    + '>'.join(COLOR_CODE.get(c, '?') for c in order_queue),
                                    (8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    head = 'material'
                else:
                    head = 'tag'
                    if t_text:
                        cv2.putText(view, t_text, (8, 52),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                if state is None:
                    label, col = 'no-state', (128, 128, 128)
                elif not fresh:
                    label, col = f'state 0x{state:02X} STALE', (0, 165, 255)
                else:
                    label = f'state 0x{state:02X} {STATE_CN.get(state, "?")}'
                    col = (0, 255, 0) if state != STATE_MOVING else (255, 255, 0)
                cv2.putText(view, f'{head}  {label}', (8, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, col, 2)
                cv2.imshow(f'schooltest2-{wk.role}', view)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            time.sleep(0.02)        # 不预览时也别空转烧核

        frames += 1
        elapsed = time.time() - t0
        if elapsed >= 2.0:
            # 哪一路能用是**会变的**（线程每 CAM_RETRY_INTERVAL 秒重试一轮）：掉下去、
            # 自己接上都在这里报一句，别让人对着黑窗口猜。没变就闭嘴，别每 2s 刷。
            for wk in workers:
                if wk.error != last_cam_err.get(wk.role):
                    last_cam_err[wk.role] = wk.error
                    if wk.error:
                        print(f'[相机] {wk.role} 这一路现在是坏的: {wk.error}')
                    else:
                        print(f'[相机] {wk.role} 这一路恢复了，出帧了')
            # 没起来的那路在状态行里也要一直看得见（不止那条 5 秒一次的提醒）
            cam_stat = '  '.join((f'{wk.role} **没开起来**' if wk.error else
                                  f'{wk.role} {wk.stats()[0]:.1f}fps/{wk.stats()[1]:.0f}ms')
                                 for wk in workers)
            rx = ' '.join(f'{t:02X}:{n}' for t, n in sorted(link.rx_count.items()))
            st = (f'0x{state:02X} {STATE_CN.get(state, "?")}'
                  if state is not None and fresh else '未知')
            # 等待抓取指令时把"圆稳了多久 / 离对准点还差多少"打出来，
            # 不然光看它不发会以为是卡住了
            still = ''
            if state == STATE_WAIT_GRAB:
                held = now - still_since if still_since is not None else 0.0
                still = f' 圆稳 {held:.1f}/{args.still_time:.1f}s'
                if aim_err is not None:
                    ex, ey = aim_err
                    # 两个轴各自的容差：谁超了谁就要挪（框画在预览里）
                    want = align_cmd_for(ex, ey, args.aim_tol_x, args.aim_tol_y)
                    if want is not None:
                        # 微调关着的时候这条只是情报：该往哪边挪写出来，但车不会动，
                        # 圆停稳了照样抓（见 decide）
                        still += f' 差 ({ex:+.0f},{ey:+.0f})px'
                        if args.align_enable:
                            # 把"卡在哪一步"写出来：等回 0x01 是正常节拍，
                            # 停手 = 上一发没生效（对着协议 §七 查）
                            if pacer.stuck:
                                pace = '上一发没等到 0x01，这一站停手了'
                            elif pacer.pending:
                                pace = '正在走，等它回 0x01'
                            elif args.align_max > 0 and pacer.count >= args.align_max:
                                pace = f'发满 {args.align_max} 步，停手了 —— 照抓'
                            else:
                                pace = f'已发 {pacer.count} 步'
                            still += f' 要挪{ADJUST_EN[want]}（{pace}）'
                        else:
                            still += '（微调关着，照抓）'
                    else:
                        still += ' **已对准**'
            # tag 顺序：还剩哪几个颜色没抓、队首那个在不在画面里（不在就是压着不抓）
            if order_queue:
                ord_stat = '  顺序 ' + '→'.join(COLOR_CN.get(c, str(c)) for c in order_queue)
                if order_miss_since is not None:
                    ord_stat += (f'（该抓{COLOR_CN.get(order_queue[0], "?")}色，'
                                 f'画面里没有，已等 {now - order_miss_since:.1f}s）')
            elif order_queue is not None:
                ord_stat = '  顺序 抓完了（等新的一站/新 tag）'
            else:
                ord_stat = ''
            print(f'[状态] 循环 {frames / elapsed:.1f}Hz  相机 {cam_stat}  '
                  f'下位机 {st}{still}{ord_stat}  收到帧 {rx or "无"}')
            frames, t0 = 0, time.time()

    for wk in workers:
        wk.stop()
    link.stop()
    time.sleep(0.3)
    if show:
        cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())

"""触摸标定抓取：复位 -> 等 BLE 重连 -> 清统计 -> 让用户点屏幕 -> 打最终统计。

用法:
    python tap_touch.py COM11 [warmup秒] [capture秒]

阶段（都写进 _serial.txt，方便事后对齐时间线）:
    1. RTS 复位，抓 boot 日志
    2. warmup 秒（等 Windows 自动重连 + ChatGPT 握手完成）
    3. 向串口发 "r" 清零 min/max 与轨迹  <- 从这一刻起用户才开始点
    4. capture 秒，实时记录 [tp] 行
    5. 向串口发 "s" 打一条汇总 [status]，里面有 min/max

为什么要在中间发 "r"：开机到重连那几秒可能有杂散触摸事件（上电抖动、
手指搭在屏上），不清掉的话 min/max 会被污染，量程就测不准了。
"""
import os
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM11"
warmup = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
capture = float(sys.argv[3]) if len(sys.argv) > 3 else 45.0
# 第 4 个参数：
#   verify（默认）= 只验证已有的坐标修正（固件开机就应用了，直接点即可）
#   cal           = 先跑一遍六点校准（要重新点 18 下）
mode = sys.argv[4] if len(sys.argv) > 4 else "verify"

# 运行中注入通道：窗口开着的时候，往这个文件里写一行命令，脚本会把它原样发给
# 板子、然后删掉文件。用处是同一个窗口里就能切 a1/a0 做 A/B、或者随时 d 一下
# 读寄存器，不用"关窗口 -> 重开"（重开会让 COM 口独占权空出来，还打断现场）。
INJECT = "_inject.txt"

out = []
t0 = time.time()


def log(msg):
    line = "[%7.2f] %s" % (time.time() - t0, msg)
    out.append(line)
    print(line, flush=True)
    with open("_serial.txt", "w", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")


def send(ser, cmd):
    ser.write((cmd + "\n").encode())
    ser.flush()
    log(">>> SENT: %s" % cmd)


def poll_inject(ser):
    """把 _inject.txt 里的命令行转发给板子，然后删掉它。"""
    if not os.path.exists(INJECT):
        return
    try:
        with open(INJECT, "r", encoding="utf-8") as f:
            cmds = [ln.strip() for ln in f if ln.strip()]
    except Exception as exc:
        log("(inject read error: %s)" % exc)
        return
    try:
        os.remove(INJECT)
    except Exception:
        pass
    for cmd in cmds:
        send(ser, cmd)


ser = serial.Serial(port, 115200, timeout=0.5, rtscts=False, dsrdtr=False)
ser.dtr = False
ser.rts = False
time.sleep(0.05)
ser.rts = True          # 只拉 RTS = 正常复位；DTR+RTS 会进下载模式
time.sleep(0.2)
ser.rts = False

reset_at = time.time()
phase = "warmup"
armed_at = 0.0
stop_at = 0.0
next_remind = 0.0
# 第一次看到触摸事件的时刻。命中后给 8 秒把后续事件收全，就提前进收尾阶段——
# 这样老胡一点到，窗口立刻结束并把结果推回来，不用干等满时长。
first_touch_at = None
# 校准期间（c 命令的 18 次取点）不能触发提前收尾 —— 那些也是真实的按下，
# 会把 90 秒倒计时从「第一次取点」就开始算，把校准腰斩。
# 判据：板子打出拟合结果（"[cal] model" / 残差行）之后才算校准结束。
cal_done = (mode != "cal")  # verify 模式没有校准阶段，一开始就算「已完成」

while True:
    elapsed = time.time() - reset_at

    if phase == "warmup" and elapsed >= warmup:
        log("=== 预热结束（%.1fs），清零触摸统计 ===" % warmup)
        send(ser, "r")
        # 窗口开始的基线快照，必须有：
        #   a = 触摸自动休眠状态（0xFE 到底写进去没有）
        #   d = 原始寄存器 + 轮询计数 pol（后面 pol 涨不涨，决定"没人碰"还是"轮询没跑"）
        send(ser, "a")
        send(ser, "d")
        # q = 中断自检（写 0xFA=0x80 让芯片自发性拉低 INT 1.2s）。
        # 放在窗口最开头：如果这里 irq 都不涨，"没人碰"这个解释才站得住。
        send(ser, "q")
        # v1 = 触摸调试：屏上画触点十字（肉眼对命中位置）+ 串口打 [tmove]
        # 每个移动样本（含 dx/dy/距离）。十字颜色就是修正开关的状态：黄=关 青=开。
        send(ser, "v1")
        if mode == "cal":
            # c = 六点校准：屏上高亮 A1..A6，逐个点它们的圆心，固件自动拟合
            # raw->screen 的 4 参数模型并写进 NVS。
            # ⚠️ 这里**不能**先发 z0：校准取的是芯片 raw 坐标，与当前是否应用
            # 修正无关（见 beginGesture 注释）；而一旦先 z0，老胡看到的十字就
            # 变回黄色原始位置，反而看不出"点得准不准"。
            send(ser, "c")
            log("=== 现在点屏幕：先跟着屏上 CAL 提示点完 A1..A6，共 %.0f 秒 ===" % capture)
        else:
            # 验证模式：固件开机已应用修正（NVS 或出厂默认），直接点。
            # 发一次 z1 只是幂等确认，顺便让串口打印当前常量的来源。
            send(ser, "z1")
            log("=== 验证模式：修正已生效，十字应为青色。现在点 A1..A6 / 表盘 / 滑动，共 %.0f 秒 ===" % capture)
        phase = "capture"
        armed_at = time.time()
        stop_at = armed_at + capture
        next_remind = armed_at + 45
        continue

    # 每 45 秒在日志里留一个时间锚点，事后好判断"某一时刻到底有没有人在点"。
    if phase == "capture" and time.time() >= next_remind:
        next_remind = time.time() + 45
        log("--- 提醒：请继续在屏幕上点/划（剩余 %.0f 秒） ---"
            % max(0.0, stop_at - time.time()))

    if phase == "capture" and (time.time() >= stop_at or
                               (first_touch_at is not None
                                and time.time() >= first_touch_at + 90.0)):
        log("=== 采集窗口结束 ===")
        # d = 原始寄存器 + 驱动解码（判"没人点"还是"点了读不到"）
        # u = 此刻屏幕应该长什么样（版式 / 六键状态 / 填充色），跟肉眼看到的对
        # s = 汇总状态（含 irq / smp / commits）
        send(ser, "d")
        send(ser, "u")
        send(ser, "s")
        phase = "tail"
        stop_at = time.time() + 2.5
        continue

    if phase == "tail" and time.time() >= stop_at:
        break

    # 运行中注入通道：外部往 _inject.txt 写一行命令即可现场干预。
    poll_inject(ser)

    try:
        raw = ser.readline()
    except Exception as exc:
        log("(read error: %s)" % exc)
        continue
    if not raw:
        continue
    line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
    if not line:
        continue
    log(line)
    if not cal_done and (line.startswith("[cal] model") or
                         "拟合残差" in line or "已丢弃" in line):
        cal_done = True
        log("=== 校准结束，之后才开始算提前收尾的倒计时 ===")
    # 提前收尾的触发条件：板子报了任何触摸事件，或 status 行里 smp（有效样本）
    # 不再为 0。两个都留着，免得某一路打印恰好被 USB 写超时吞掉就漏判。
    # 校准期间只认 [touch]（那时不会有），smp 规则要等 cal_done。
    if phase == "capture" and first_touch_at is None:
        hit = line.startswith("[touch]") or " swipe " in line
        if not hit and cal_done and line.startswith("[status]"):
            for tok in line.split():
                if tok.startswith("smp=") and tok[4:].isdigit():
                    hit = int(tok[4:]) > 0
        if hit:
            first_touch_at = time.time()
            log("=== 检测到触摸事件，90 秒后提前收尾（要留够时间做一次长拖拽） ===")

ser.close()
log("--- done ---")

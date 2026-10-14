"""不复位地偷看串口：以 DTR/RTS=false 打开端口，然后下发命令读回结果。

为什么需要这个脚本：
    板子上 ESP32-S3 用的是原生 USB（USB-Serial-JTAG）。它对宿主机的
    DTR/RTS 有特殊语义 —— **RTS=1 等于把 EN 拉低 = 复位板子**。
    pyserial 的 `Serial(port, ...)` 构造完就立刻 open，而 open 时
    默认 dtr/rts 是 True，于是"一打开端口板子就重启"，抓到第一行永远是
    `ESP-ROM:esp32s3-...`。对调试没影响（我们本来就要看开机日志），
    但**要读 RAM 里的东西（比如事件环）就必须避开这一下**。

    做法：先 `Serial()` 不打开 -> 设 dtr/rts=False -> 再 open。
    pyserial 在未打开时设 dtr/rts 只是存值，open() 时才写进驱动，
    所以整个打开过程驱动都没收到过 SET_CONTROL_LINE_STATE(RTS=1)。

用法:
    python tap_peek.py COM11 xl 8
参数:
    port  串口
    cmd   打开后要下发的命令（不含换行）
    hold  发出命令后继续读多少秒
"""
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM11"
cmd = sys.argv[2] if len(sys.argv) > 2 else "xl"
hold = float(sys.argv[3]) if len(sys.argv) > 3 else 8.0

out = []
t0 = time.time()


def log(msg):
    line = "[%7.2f] %s" % (time.time() - t0, msg)
    out.append(line)
    print(line, flush=True)
    with open("_serial.txt", "w", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")


# ★ 两次式打开：先构造不打开，把 dtr/rts 置 0，再 open。
#
# 重试是必须的：本机还有一个常驻脚本（EInkInfo 的 codex_bridge.py）每 5 秒
# 独占打开一次所有 USB 串口写一行 KV —— 撞上它的瞬间我们就是「拒绝访问」。
# 它持有时间很短，退避重试几次就能穿过去，不用去动那个进程。
def open_no_reset():
    s = serial.Serial()
    s.port = port
    s.baudrate = 115200
    s.timeout = 1
    s.dtr = False
    s.rts = False
    s.open()
    return s


ser = None
last_exc = None
for attempt in range(1, 41):
    try:
        ser = open_no_reset()
        if attempt > 1:
            log("--- 第 %d 次尝试打开成功（前 %d 次被别的进程占着）---"
                % (attempt, attempt - 1))
        break
    except Exception as exc:
        last_exc = exc
        time.sleep(0.5)
if ser is None:
    log("!!! 打开失败（重试 40 次 / 20 秒）：%s" % last_exc)
    log("!!! 说明有进程**持续**占着 %s，不是瞬时抢占。"
        "本机嫌疑最大的是 EInkInfo 的 codex_bridge.py。" % port)
    raise SystemExit(1)

log("--- port opened with dtr=0 rts=0 (no reset pulse) ---")

# 静默听 2 秒：没复位的话这里应该是空的（心跳 60 秒才一次）；
# 如果蹦出 ESP-ROM / setup 日志，说明还是复位了。
quiet_until = time.time() + 2.0
while time.time() < quiet_until:
    raw = ser.readline()
    if raw:
        log(raw.decode("utf-8", "replace").rstrip("\r\n"))

ser.reset_input_buffer()
# ★ 先单独送一个换行，把可能残留的半行冲掉。
#   实测踩过：连上后直接发 xl，固件回的是 "[cmd] unknown: gxl" ——
#   它的行缓冲里留着一个 'g'（上一个进程写在半截被杀 / 手动敲了没回车），
#   于是 xl 被拼成了 gxl 而被当成未知命令。空行让残留自成一行、无害地走掉。
ser.write(b"\n")
ser.flush()
time.sleep(0.3)
ser.write((cmd + "\n").encode())
ser.flush()
log(">>> SENT: %s" % cmd)

start = time.time()
while time.time() - start < hold:
    try:
        raw = ser.readline()
    except Exception as exc:
        log("(read timeout: %s)" % exc)
        continue
    if not raw:
        continue
    log(raw.decode("utf-8", "replace").rstrip("\r\n"))

ser.close()
log("--- done ---")

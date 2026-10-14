"""复位板子 -> 等待 BLE 重连 -> 从串口下发一条命令 -> 继续读回结果。

用法:
    python tap_send.py COM11 6 "k ACT10 1" 10
参数:
    port   串口
    delay  复位后等待多少秒再发命令（留给 BLE 重连 + ChatGPT 握手）
    cmd    要写入串口的命令（不含换行，脚本自动补 \n）
    hold   发完命令后再读多少秒
"""
import sys
import time
import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM11"
delay = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
cmd = sys.argv[3] if len(sys.argv) > 3 else "k ACT10 1"
hold = float(sys.argv[4]) if len(sys.argv) > 4 else 10.0

out = []
t0 = time.time()


def log(msg):
    line = "[%7.2f] %s" % (time.time() - t0, msg)
    out.append(line)
    print(line, flush=True)
    with open("_serial.txt", "w", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")


ser = serial.Serial(port, 115200, timeout=1, rtscts=False, dsrdtr=False)
# 只拉 RTS 触发复位（DTR+RTS 同时拉会把板子打进下载模式，实测过）
ser.dtr = False
ser.rts = True
time.sleep(0.2)
ser.rts = False
log("--- reset pulse sent ---")

sent = False
while True:
    elapsed = time.time() - t0
    if not sent and elapsed >= delay:
        ser.write((cmd + "\n").encode())
        ser.flush()
        log(">>> SENT: %s" % cmd)
        sent = True
    if sent and elapsed >= delay + hold:
        break
    if elapsed > delay + hold + 5:
        break
    try:
        line = ser.readline().decode("utf-8", "replace").rstrip("\r\n")
        if line:
            log(line)
    except Exception as exc:  # 读超时属正常
        log("(read timeout: %s)" % exc)

ser.close()
log("--- done ---")

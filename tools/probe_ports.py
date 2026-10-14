"""探测本机各串口能不能打开 —— 只 open/close，不写任何字节。

为什么要单独做这一步：
    读事件环失败在 PermissionError(13 拒绝访问)，有两种完全不同的成因，
    下一步动作也完全相反：
      ① 某个进程（串口终端 / 常驻脚本）正独占着 **COM11** 这一个口
         -> 把它关掉就好
      ② 这台机器上的企业安全软件（本机装了深信服 / 亚信 / 天擎等好几套）
         对外设做了管控，**所有** 串口都打不开
         -> 关谁都没用，得换机器或换手段

    判据：COM4/5/11/12/13 全 FAIL = ②，只有 COM11 FAIL = ①。

⚠️ 打开时 dtr/rts 都置 0：板子是 ESP32-S3 原生 USB（USB-Serial-JTAG），
   它的 RTS=1 就是"把 EN 拉低 = 复位"，绝不能顺手拉一下 —— 那会把
   RAM 里的事件环清掉，正是我们要读的东西。
"""
import serial

PORTS = ["COM11", "COM4", "COM5", "COM12", "COM13"]

ok, fail = [], []
for p in PORTS:
    try:
        s = serial.Serial()
        s.port = p
        s.baudrate = 115200
        s.timeout = 0.2
        s.dtr = False
        s.rts = False
        s.open()
        s.close()
        print("%-6s OPEN OK" % p, flush=True)
        ok.append(p)
    except Exception as exc:
        print("%-6s FAIL  %s" % (p, exc), flush=True)
        fail.append(p)

print("---- 小结 ----")
print("能打开: %s" % (ok or "无"))
print("打不开: %s" % (fail or "无"))
print("全部打不开 = 硬件/安全软件层面的管控；只有 COM11 = 被某个进程独占")

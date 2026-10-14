"""Reset the board with an RTS pulse and record the boot log.

Usage: python tap_reset.py [port] [seconds]
"""
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "COM11"
seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0

ser = serial.Serial(port, 115200, timeout=0.5)
ser.dtr = False
ser.rts = False
time.sleep(0.05)
ser.rts = True          # pull EN low for 200 ms
time.sleep(0.2)
ser.rts = False

start = time.monotonic()
with open("_serial.txt", "w", encoding="utf-8", errors="replace") as out:
    out.write("--- reset pulse sent ---\n")
    while time.monotonic() - start < seconds:
        try:
            raw = ser.readline()
        except Exception as exc:
            out.write(f"READ ERROR {exc}\n")
            out.flush()
            continue
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
        out.write(f"[{time.monotonic() - start:7.2f}] {line}\n")
        out.flush()
ser.close()

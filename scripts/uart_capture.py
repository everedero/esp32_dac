#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Reset the ESP32 through RTS, print its UART until PASS/FAIL or timeout.

Exit code: 0 if a PASS line was seen, 1 on FAIL, 2 on timeout.
Usage: uart_capture.py [port] [timeout_s] [results_needed]
"""
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
needed = int(sys.argv[3]) if len(sys.argv) > 3 else 3

with serial.Serial(port, 115200, timeout=0.2) as s:
    # NodeMCU auto-reset: RTS -> EN, DTR -> IO0. Keep IO0 high, pulse EN.
    s.dtr = False
    s.rts = True
    time.sleep(0.1)
    s.rts = False
    s.reset_input_buffer()

    results = []
    end = time.time() + timeout
    while time.time() < end and len(results) < needed:
        line = s.readline().decode(errors="replace").rstrip()
        if not line:
            continue
        print(line, flush=True)
        if "PASS" in line or "FAIL" in line:
            results.append("PASS" in line)

if not results:
    sys.exit(2)
sys.exit(0 if all(results) else 1)

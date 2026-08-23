"""Send the new configuration commands to the board and read STATUS back, to
confirm each one is actually accepted and retained. Sends no brake commands.
"""
import time

import serial

s = serial.Serial()
s.port, s.baudrate, s.timeout = "/dev/ttyUSB0", 115200, 0.2
s.dsrdtr = s.rtscts = False
s.dtr = s.rts = False
s.open()
s.dtr = s.rts = False
time.sleep(1.5)
s.reset_input_buffer()

# Deliberately odd values so a default can't be mistaken for a success.
for cmd in ["TEETH,7", "RATIO,1.75", "RPM_FILTER,3,8500", "RPM_AVG,6",
            "BRAKE_RANGE,25,3200", "PRELOAD,17", "INVERT,0",
            "STEPPER_SPEED,1234", "STEPPER_ACCEL,4321",
            "CAL_MECH,3.0", "CAL_PRESS,480,0.75", "PRESS_LIMIT,1450",
            "PID_SWEEP,0.44,0.055,0.011"]:
    s.write((cmd + "\n").encode())
    time.sleep(0.06)

# Range checks must be refused, not silently clamped. Collect the replies as
# they arrive — flushing first would throw away the very ERR lines under test.
errs = []
time.sleep(0.4)
s.reset_input_buffer()
for bad in ["TEETH,0", "RPM_FILTER,9", "BRAKE_RANGE,900,100", "RATIO,0"]:
    s.write((bad + "\n").encode())
    until = time.time() + 0.5
    while time.time() < until:
        raw = s.readline()
        if raw:
            line = raw.decode("ascii", errors="replace").strip()
            if line.startswith("ERR,"):
                errs.append(line[4:])

s.write(b"STATUS\n")

cfg = {}
deadline = time.time() + 4.0
while time.time() < deadline:
    raw = s.readline()
    if not raw:
        continue
    line = raw.decode("ascii", errors="replace").strip()
    if line.startswith("CFG,"):
        parts = line.split(",")
        cfg[parts[1]] = ",".join(parts[2:])
    elif line.startswith("ERR,"):
        errs.append(line[4:])
s.close()

expect = {
    "TEETH": "7", "RATIO": "1.7500", "RPM_FILTER": "3,8500.0", "RPM_AVG": "6",
    "BRAKE_RANGE": "25,3200", "PRELOAD": "17.0", "INVERT": "0",
    "STEPPER": "1234.0,4321.0", "MECH": "3.0000",
    "PRESS": "480.0000,0.750000,1", "PRESS_LIMIT": "1450.0",
    "PID_SWEEP": "0.4400,0.0550,0.0110", "SIM": "0",
}
fails = []
for key, want in expect.items():
    got = cfg.get(key)
    ok = got == want
    print(f"{'PASS' if ok else 'FAIL'}  CFG,{key}: got {got!r} want {want!r}")
    if not ok:
        fails.append(key)

print()
print("rejections seen:", errs)
for token in ("TEETH", "RPM_FILTER", "BRAKE_RANGE", "RATIO"):
    ok = any(token in e for e in errs)
    print(f"{'PASS' if ok else 'FAIL'}  bad {token} rejected")
    if not ok:
        fails.append(f"reject-{token}")

print()
print("FAILURES:", fails if fails else "none")

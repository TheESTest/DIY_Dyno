"""Cam configuration plumbing, without commanding any brake movement.

Checks that every cam setting is accepted, retained and reported back, and that
malformed or out-of-order input is refused. The mapping maths itself needs the
stepper to move, so it lives in verify_cam.py.
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

fails = []
errs = []


def send(cmd, collect=False, settle=0.12):
    s.write((cmd + "\n").encode())
    if not collect:
        time.sleep(settle)
        return
    until = time.time() + settle + 0.4
    while time.time() < until:
        raw = s.readline()
        if raw:
            line = raw.decode("ascii", errors="replace").strip()
            if line.startswith("ERR,"):
                errs.append(line[4:])


for cmd in ["CAM_SPD,40", "CAM_NPTS,4",
            "CAM_PT,0,0,0", "CAM_PT,1,25,5", "CAM_PT,2,50,40", "CAM_PT,3,100,100",
            "CAM_MODEL,2", "CAM_LIN,1"]:
    send(cmd)

# Each of these must be refused.
for bad in ["CAM_MODEL,7", "CAM_SPD,0", "CAM_NPTS,99", "CAM_PT,99,10,10",
            "CAM_PT,2,1,50"]:            # last one breaks increasing travel %
    send(bad, collect=True)

send("STATUS", settle=0.2)

cfg, pts = {}, {}
deadline = time.time() + 4.0
while time.time() < deadline:
    raw = s.readline()
    if not raw:
        continue
    line = raw.decode("ascii", errors="replace").strip()
    if line.startswith("CFG,CAM_PT,"):
        p = line.split(",")
        pts[int(p[2])] = (float(p[3]), float(p[4]))
    elif line.startswith("CFG,"):
        p = line.split(",")
        cfg[p[1]] = ",".join(p[2:])
    elif line.startswith("ERR,"):
        errs.append(line[4:])


def check(name, got, want):
    ok = got == want
    print(f"{'PASS' if ok else 'FAIL'}  {name}: got {got!r} want {want!r}")
    if not ok:
        fails.append(name)


check("CFG,CAM model/spd/lin/npts", cfg.get("CAM"), "2,40.0000,1,4")
check("point 0", pts.get(0), (0.0, 0.0))
check("point 1", pts.get(1), (25.0, 5.0))
check("point 2", pts.get(2), (50.0, 40.0))
check("point 3", pts.get(3), (100.0, 100.0))

print()
print("rejections seen:")
for e in errs:
    print("   ", e)
for token, label in [("CAM_MODEL must", "bad model"),
                     ("CAM_SPD must", "zero steps/deg"),
                     ("CAM_NPTS must", "bad point count"),
                     ("CAM_PT index", "bad point index"),
                     ("increase down", "out-of-order travel %")]:
    ok = any(token in e for e in errs)
    print(f"{'PASS' if ok else 'FAIL'}  {label} refused")
    if not ok:
        fails.append(label)

# Leave the board linear and unconfigured, brake untouched.
for cmd in ["CAM_MODEL,0", "CAM_LIN,0", "CAM_NPTS,0"]:
    send(cmd)
s.close()

print()
print("FAILURES:", fails if fails else "none")

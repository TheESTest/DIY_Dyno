"""Check the cam mapping on the board itself.

Drives the brake to known positions with each model selected and reads the
brake % the firmware reports back, which is the number the operator actually
sees. Positions are commanded, so this moves the stepper.
"""
import math
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


def send(cmd, settle=0.08):
    s.write((cmd + "\n").encode())
    time.sleep(settle)


def brake_pct_at(steps, settle=2.0):
    """Command a position, wait for the stepper to arrive, read brake %."""
    send(f"BRAKE,{steps}")
    time.sleep(settle)
    s.reset_input_buffer()
    deadline = time.time() + 2.0
    while time.time() < deadline:
        raw = s.readline()
        if not raw:
            continue
        line = raw.decode("ascii", errors="replace").strip()
        if line.startswith("DATA,"):
            parts = line.split(",")
            if len(parts) >= 14 and int(float(parts[7])) == steps:
                return float(parts[13])
    return None


def check(name, got, want, tol=1.5):
    ok = got is not None and abs(got - want) <= tol
    shown = f"{got:.2f}" if got is not None else "None"
    print(f"{'PASS' if ok else 'FAIL'}  {name}: got {shown} want {want:.2f} ±{tol}")
    if not ok:
        fails.append(name)


# Known range so travel fractions are easy to reason about.
send("BRAKE_RANGE,0,4000")
send("CAM_LIN,0")

# ── Linear: brake % is travel % ──────────────────────────────
send("CAM_MODEL,0")
for steps, want in [(0, 0.0), (1000, 25.0), (2000, 50.0), (4000, 100.0)]:
    check(f"linear @ {steps}", brake_pct_at(steps), want)

# ── Eccentric: lift ∝ (1 - cos θ) ────────────────────────────
# 4000 steps over 40 steps/deg = 100 degrees of cam.
send("CAM_SPD,40")
send("CAM_MODEL,1")
th_full = math.radians(100.0)
denom = 1.0 - math.cos(th_full)
for steps in (0, 1000, 2000, 3000, 4000):
    u = steps / 4000.0
    want = (1.0 - math.cos(u * th_full)) / denom * 100.0
    check(f"eccentric @ {steps}", brake_pct_at(steps), want)

# The whole point: mid travel is NOT mid brake on a cam.
mid = brake_pct_at(2000)
print(f"      (eccentric mid-travel reads {mid:.1f}% brake, not 50%)")
if mid is None or abs(mid - 50.0) < 5.0:
    fails.append("eccentric should be non-linear")

# ── Measured table, interpolated ─────────────────────────────
send("CAM_MODEL,0")
send("CAM_NPTS,4")
send("CAM_PT,0,0,0")
send("CAM_PT,1,25,5")
send("CAM_PT,2,50,40")
send("CAM_PT,3,100,100")
send("CAM_MODEL,2")
check("table @ 0",    brake_pct_at(0),    0.0)
check("table @ 25%",  brake_pct_at(1000), 5.0)
check("table @ 50%",  brake_pct_at(2000), 40.0)
check("table @ 100%", brake_pct_at(4000), 100.0)
# Halfway between the 50 and 100 rows -> halfway between 40 and 100.
check("table interpolates @ 75%", brake_pct_at(3000), 70.0)

# ── Out-of-order rows are refused ────────────────────────────
s.reset_input_buffer()
send("CAM_PT,1,90,10")
errs = []
until = time.time() + 0.6
while time.time() < until:
    raw = s.readline()
    if raw:
        line = raw.decode("ascii", errors="replace").strip()
        if line.startswith("ERR,"):
            errs.append(line)
ok = any("increase" in e for e in errs)
print(f"{'PASS' if ok else 'FAIL'}  out-of-order cam point refused: {errs}")
if not ok:
    fails.append("cam order check")

# Leave the brake released and the mapping back to linear.
send("CAM_MODEL,0")
send("CAM_NPTS,0")
send("BRAKE,0")
time.sleep(1.0)
send("STOP")
s.close()

print()
print("FAILURES:", fails if fails else "none")

"""Check the three items Matt asked for, against the board, without moving anything.

  1. Pulses per revolution is a variable
  2. Stepper range-of-motion limits are configurable
  3. Load cell voltage calibration actually reaches the torque calculation

(3) is the interesting one: storing a scale proves nothing if it never reaches
the maths. With scale, lever arm and mechanical factor known, every DATA frame
lets us solve back for the tare offset. If that implied offset comes out the
same no matter which multiplier we change, all three are genuinely in the chain.
"""
import statistics
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


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}{('  ' + detail) if detail else ''}")
    if not ok:
        fails.append(name)


def send(cmd, settle=0.1):
    s.write((cmd + "\n").encode())
    time.sleep(settle)


def read_cfg():
    s.reset_input_buffer()
    s.write(b"STATUS\n")
    cfg = {}
    deadline = time.time() + 3.0
    while time.time() < deadline:
        raw = s.readline()
        if not raw:
            continue
        line = raw.decode("ascii", errors="replace").strip()
        if line.startswith("CFG,") and not line.startswith("CFG,CAM_PT,"):
            p = line.split(",")
            cfg[p[1]] = ",".join(p[2:])
    return cfg


def sample(n=14):
    """Mean load-cell mV and torque over n frames."""
    s.reset_input_buffer()
    raws, tqs = [], []
    deadline = time.time() + 6.0
    while len(raws) < n and time.time() < deadline:
        line = s.readline().decode("ascii", errors="replace").strip()
        if line.startswith("DATA,"):
            p = line.split(",")
            if len(p) >= 6:
                tqs.append(float(p[3]))
                raws.append(float(p[4]))
    if not raws:
        return None, None
    return statistics.mean(raws), statistics.mean(tqs)


print("=== 1. Pulses per revolution ===")
send("TEETH,11")
check("TEETH accepted and retained", read_cfg().get("TEETH") == "11",
      f"CFG,TEETH={read_cfg().get('TEETH')}")
send("TEETH,3")
check("restored to 3", read_cfg().get("TEETH") == "3")

print()
print("=== 2. Stepper range of motion ===")
send("BRAKE_RANGE,120,3450")
cfg = read_cfg()
check("BRAKE_RANGE accepted", cfg.get("BRAKE_RANGE") == "120,3450",
      f"CFG,BRAKE_RANGE={cfg.get('BRAKE_RANGE')}")
check("stepper speed/accel configurable", cfg.get("STEPPER") is not None,
      f"CFG,STEPPER={cfg.get('STEPPER')}")
send("BRAKE_RANGE,0,250")

print()
print("=== 3. Load cell calibration reaches the torque maths ===")
# A large scale so ADC noise alone gives a torque well clear of rounding.
BASE_SCALE, BASE_ARM = 100000.0, 1.0
implied = {}
for label, scale, arm, mech in [
        ("scale=1e5 arm=1.0 mech=1", BASE_SCALE, BASE_ARM, 1.0),
        ("scale=1e5 arm=1.0 mech=3", BASE_SCALE, BASE_ARM, 3.0),
        ("scale=1e5 arm=2.5 mech=1", BASE_SCALE, 2.5, 1.0),
        ("scale=2e5 arm=1.0 mech=1", 2 * BASE_SCALE, BASE_ARM, 1.0)]:
    send(f"CAL_SCALE,{scale}")
    send(f"CAL_ARM,{arm}")
    send(f"CAL_MECH,{mech}")
    time.sleep(1.0)                     # let the torque average refill
    raw, tq = sample()
    if raw is None:
        print(f"   {label}: no data")
        continue
    # torque = (raw - offset) * scale * arm * mech  ->  solve for offset
    off = raw - tq / (scale * arm * mech)
    implied[label] = (raw, tq, off)
    print(f"   {label}: loadRaw={raw:8.3f}  torque={tq:12.3f}  implied offset={off:8.4f}")

if len(implied) == 4:
    torques = [v[1] for v in implied.values()]
    offs = [v[2] for v in implied.values()]
    spread = max(offs) - min(offs)
    signal = max(abs(t) for t in torques)
    if signal < 1.0:
        print()
        print("   INCONCLUSIVE: the load cell channel is sitting exactly on its tare")
        print("   offset, so every torque is zero and the multipliers cancel out.")
        print("   Re-run with a load applied to prove the chain end to end.")
        fails.append("load cell chain (inconclusive, needs a load)")
    else:
        check("scale, lever arm and mechanical factor all reach the torque maths",
              spread < 0.05,
              f"implied offset agrees to {spread:.4f} mV across all four configs")
else:
    check("collected all four configurations", False)

# Back to harmless defaults.
for cmd in ["CAL_SCALE,1.0", "CAL_ARM,0.3", "CAL_MECH,1.0"]:
    send(cmd)
cfg = read_cfg()
print()
print(f"   restored: SCALE={cfg.get('SCALE')} ARM={cfg.get('ARM')} MECH={cfg.get('MECH')}")
s.close()

print()
print("FAILURES:", fails if fails else "none")

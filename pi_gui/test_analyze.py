"""The standalone analyser.

Synthetic data throughout, so each check is exercised against a case where the
right answer is known. The distinctions that matter are the ones that look
alike in a summary: clamped against stalling, and a real run against a file
recorded while nothing was connected.
"""
import csv, json, os, tempfile
import numpy as np
import dyno_analyze as A

fails = []
def check(what, got, want):
    ok = got == want
    if not ok:
        fails.append(f"{what}: got {got!r} want {want!r}")
    print(("  ok   " if ok else "  FAIL ") + what)

def near(what, got, want, tol):
    ok = got is not None and abs(got - want) <= tol
    if not ok:
        fails.append(f"{what}: got {got!r} want {want}±{tol}")
    print(("  ok   " if ok else "  FAIL ") + f"{what} ({got})")


def sweep_rows(n=200, travel=600, ratio=1.0, degrade=None, takeup=150,
               slope=3.0, cap=None, dead=False):
    """A synthetic outward leg. ratio<1 is a steady shortfall; degrade makes
    the shortfall worsen with travel, the way lost steps do."""
    rows = []
    for i in range(n):
        cmd = travel * i / (n - 1)
        r = ratio if degrade is None else ratio - degrade * (i / (n - 1))
        pos = cmd * r
        psi = max(0.0, (pos - takeup) * slope)
        if cap is not None:
            psi = min(psi, cap)
        if dead:
            pos = psi = 0.0
        rows.append({"Time_s": f"{i * 0.05:.3f}", "Phase": "up",
                     "Commanded_Steps": f"{cmd:.1f}", "Brake_Pos": f"{pos:.0f}",
                     "Brake_Pct": "0", "Brake_PSI": f"{psi:.2f}",
                     "Pressure_mV": "0" if dead else f"{500 + psi:.1f}",
                     "LoadCell_mV": "0" if dead else "800",
                     "RPM": "0", "Stall_Suspected": "0"})
    return rows


# --- telemetry that was never there --------------------------------------
live, why = A.telemetry_is_real(sweep_rows(dead=True))
check("a disconnected recording is caught", live, False)
check("and says every channel is zero", "constant zero" in why, True)
live, _ = A.telemetry_is_real(sweep_rows())
check("a real recording passes", live, True)

# --- clamped against stalling: the distinction that matters --------------
def follow(rows):
    return A.follow_quality(
        np.array([float(r["Commanded_Steps"]) for r in rows]),
        np.array([float(r["Brake_Pos"]) for r in rows]))

check("a full-travel sweep is followed", follow(sweep_rows())["verdict"],
      "followed")
fq = follow(sweep_rows(ratio=0.742))
check("a steady shortfall reads as clamped", fq["verdict"], "clamped")
near("and the ratio is recovered", fq["mean"], 0.742, 0.01)
fq = follow(sweep_rows(ratio=0.95, degrade=0.45))
check("a worsening shortfall reads as losing steps", fq["verdict"], "losing")
check("early beats late when losing", fq["early"] > fq["late"], True)

# --- the effective range -------------------------------------------------
rows = sweep_rows(takeup=150, slope=3.0, cap=900)
pos = np.array([float(r["Brake_Pos"]) for r in rows])
psi = np.array([float(r["Brake_PSI"]) for r in rows])
er = A.effective_range(pos, psi)
check("a responding sweep is usable", er["usable"], True)
near("takeup found", er["takeup"], 150, 25)
near("slope recovered", er["psi_per_step"], 3.0, 0.5)

flat = A.effective_range(np.linspace(0, 600, 200), np.zeros(200))
check("flat pressure is refused", flat["usable"], False)
still = A.effective_range(np.zeros(200), np.zeros(200))
check("an actuator that never moved is refused", still["usable"], False)
check("and says so", "never moved" in still["reason"], True)

# --- the pressure collapse ----------------------------------------------
# The real events peaked at 1123-1146 PSI and fell to about 320, so the
# synthetic one is built to the same shape rather than sitting on the
# threshold where either answer would be defensible.
p = np.linspace(0, 600, 200)
q = np.clip((p - 150) * 3.85, 0, None)
q[150:] = 320.0
col = A.find_pressure_collapse(p, q)
check("a collapse is found", col is not None, True)
if col:
    near("at the right pressure", col["peak"], q[149], 1)
    check("and reported as a large drop", col["drop_pct"] > 60, True)
check("a clean curve has no collapse",
      A.find_pressure_collapse(p, np.clip((p - 150) * 3.0, 0, None)), None)
# a brake simply being released is not a collapse
p2 = np.linspace(600, 0, 200)
q2 = np.clip((p2 - 150) * 3.0, 0, None)
check("retracting is not called a collapse",
      A.find_pressure_collapse(p2, q2), None)

# --- gains judged against the range they work on -------------------------
g = A.gain_sanity({"brake_min": "0", "brake_max": "667",
                   "pid_hold": {"kp": "0.0117754", "ki": "0", "kd": "0"}})
near("the real case: full brake at 56,644 RPM", g["rpm_for_full"], 56644, 5)
g = A.gain_sanity({"brake_min": "140", "brake_max": "470",
                   "pid_hold": {"kp": "0.33", "ki": "0.5", "kd": "0"}})
near("a sane gain is about 1000 RPM", g["rpm_for_full"], 1000, 5)
check("a missing conditions file gives nothing", A.gain_sanity(None), None)
check("a malformed one does not crash", A.gain_sanity({"brake_min": "x"}), None)

# --- end to end on files -------------------------------------------------
d = tempfile.mkdtemp()
def write(name, rows, cond=None):
    p = os.path.join(d, name)
    with open(p, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    if cond:
        with open(os.path.splitext(p)[0] + "_conditions.json", "w") as f:
            json.dump(cond, f)
    return p

cond = {"brake_min": "0", "brake_max": "667", "press_limit": "1500",
        "_ui_version": "1.13.0", "_fw_version": "1.7.0",
        "pid_hold": {"kp": "0.0117754", "ki": "0", "kd": "0"}}
good = write("brake_char_20260901_000001.csv", sweep_rows(cap=900), cond)
short = write("brake_char_20260901_000002.csv", sweep_rows(ratio=0.742), cond)
dead = write("brake_char_20260901_000003.csv", sweep_rows(dead=True), cond)

check("kind detected from the columns", A.kind_of(good, A.read_csv(good)), "sweep")
check("conditions found beside it", A.conditions_for(good)["brake_max"], "667")
check("a missing conditions file is not an error",
      A.conditions_for(os.path.join(d, "nothing.csv")), None)

import io as _io, contextlib
def run(path):
    buf = _io.StringIO()
    with contextlib.redirect_stdout(buf):
        rows = A.read_csv(path)
        A.analyse_sweep(path, rows, A.conditions_for(path))
    return buf.getvalue()

out = run(good)
check("a good sweep recommends a range", "set the brake range to" in out, True)
check("and flags the weak gain", "56,644 RPM" in out, True)
check("and the useless pressure limit", "never intervene" in out, True)

out = run(short)
check("a clamped sweep says so", "CONSTANT" in out, True)
check("and refuses to recommend a range from it",
      "do NOT set the brake range" in out, True)
check("and does not also recommend one", "set the brake range to" in out, False)

out = run(dead)
check("a dead sweep is called out", "NOT A REAL SWEEP" in out, True)
check("and nothing is concluded from it", "takeup" in out, False)

# plotting must not fall over on either kind
png = A.plot_file(good, A.read_csv(good), "sweep", d)
check("a sweep plot is written", os.path.exists(png), True)
check("and is a real PNG", open(png, "rb").read(4), b"\x89PNG")

print("FAILURES: " + ("none" if not fails else "\n  " + "\n  ".join(fails)))
raise SystemExit(1 if fails else 0)

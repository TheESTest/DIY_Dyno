"""Effective brake range from a characterisation sweep, and the guards that
stop a sweep recording nothing.

The case this exists for is real: two sweeps were saved as calibrations while
the controller was silent, and both looked like valid files.
"""
import os, csv, time, tempfile, tkinter as tk
os.environ.setdefault("MPLBACKEND", "Agg")
import numpy as np
import dyno_gui
import tempfile as _tf, os as _os
dyno_gui.SESSION_FILE = _os.path.join(_tf.mkdtemp(), "session.json")
from dyno_gui import messagebox

fails = []
def check(what, got, want):
    ok = got == want
    if not ok:
        fails.append(f"{what}: got {got!r} want {want!r}")
    print(("  ok   " if ok else "  FAIL ") + what)

def near(what, got, want, tol):
    ok = abs(got - want) <= tol
    if not ok:
        fails.append(f"{what}: got {got!r} want {want}±{tol}")
    print(("  ok   " if ok else "  FAIL ") + f"{what} ({got:.1f})")

shown = []
answer = [True]
messagebox.showinfo = lambda t, m="", **k: shown.append(("info", t, m))
messagebox.showwarning = lambda t, m="", **k: shown.append(("warn", t, m))
messagebox.showerror = lambda t, m="", **k: shown.append(("error", t, m))
messagebox.askyesno = lambda t, m="", **k: (shown.append(("ask", t, m)), answer[0])[1]

ER = dyno_gui.DynoApp.effective_range

# ── a rig that responds: the numbers must come back out ──────────────────
pos = np.linspace(0, 500, 300)
psi = np.clip((pos - 180.0) * 3.2, 0, 800.0)
r = ER(pos, psi)
check("a responding rig is usable", r["usable"], True)
near("takeup found near the real dead zone", r["takeup_steps"], 180, 25)
near("saturation found near the real knee", r["saturation_steps"], 430, 25)
near("slope recovered", r["psi_per_step"], 3.2, 0.4)
near("dead travel reported", r["dead_below_pct"], 36, 8)
check("effective span is the difference",
      round(r["effective_span"], 1),
      round(r["saturation_steps"] - r["takeup_steps"], 1))

# noise must not move the answer much
rng = np.random.default_rng(7)
r2 = ER(pos, psi + rng.normal(0, 4, pos.size))
near("noise does not move takeup", r2["takeup_steps"], r["takeup_steps"], 20)
near("noise does not move saturation", r2["saturation_steps"],
     r["saturation_steps"], 20)

# a sensor with a real offset is handled by the baseline, not by assuming zero
r3 = ER(pos, psi + 120.0)
near("offset absorbed into baseline", r3["takeup_steps"], r["takeup_steps"], 15)
check("baseline reported, not assumed zero", round(r3["baseline_psi"]) > 100, True)

# ── the degenerate cases, which is what the real files were ──────────────
flat = ER(np.linspace(0, 500, 300), np.zeros(300))
check("flat pressure is refused", flat["usable"], False)
check("and says pressure never rose", "never rose" in flat["reason"], True)

still = ER(np.zeros(300), np.zeros(300))
check("an actuator that never moved is refused", still["usable"], False)
check("and says so plainly", "never moved" in still["reason"], True)

check("too few samples refused", ER([0, 1], [0, 1])["usable"], False)
noisy_only = ER(np.linspace(0, 500, 300), np.full(300, 12.0))
check("a constant non-zero reading is refused", noisy_only["usable"], False)

# ── reading it out of a sweep file ───────────────────────────────────────
root = tk.Tk(); app = dyno_gui.DynoApp(root)
sent = []; app._send = lambda c: sent.append(c)

d = tempfile.mkdtemp()
good = os.path.join(d, "brake_char_good.csv")
with open(good, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["Time_s", "Phase", "Commanded_Steps", "Brake_Pos", "Brake_Pct",
                "Brake_PSI", "Pressure_mV", "LoadCell_mV", "RPM",
                "Stall_Suspected"])
    for i, (p, q) in enumerate(zip(pos, psi)):
        w.writerow([f"{i*0.05:.3f}", "up", f"{p:.1f}", f"{p:.0f}", "0",
                    f"{q:.2f}", "0", "0", "0", "0"])
rows = list(csv.reader(open(good)))[1:]
check("range read from sweep rows", app._range_from_rows(rows)["usable"], True)

dead = os.path.join(d, "brake_char_dead.csv")
with open(dead, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["Time_s", "Phase", "Commanded_Steps", "Brake_Pos", "Brake_Pct",
                "Brake_PSI", "Pressure_mV", "LoadCell_mV", "RPM",
                "Stall_Suspected"])
    for i in range(300):          # exactly the shape of the two real files
        w.writerow([f"{i*0.05:.3f}", "up", f"{i*1.67:.1f}", "0", "0.00",
                    "0.00", "0.0", "0.00", "0.0", "0"])
rows_dead = list(csv.reader(open(dead)))[1:]
res_dead = app._range_from_rows(rows_dead)
check("a sweep that recorded nothing is refused", res_dead["usable"], False)

# applying it through the dialog
dyno_gui.filedialog.askopenfilename = lambda **k: good
app.cfg_vars["brake_min"].set("0"); app.cfg_vars["brake_max"].set("500")
answer[0] = True
shown.clear(); app._find_effective_range()
check("range applied to the brake limits",
      float(app.cfg_vars["brake_min"].get()) > 150
      and float(app.cfg_vars["brake_max"].get()) < 460, True)
check("explained the takeup", any("Takeup" in x[2] for x in shown), True)
check("explained the dead travel", any("below takeup" in x[2] for x in shown), True)
check("result still passes validation", app._validate_cfg(), [])

# declining leaves the limits alone
app.cfg_vars["brake_min"].set("0"); app.cfg_vars["brake_max"].set("500")
answer[0] = False
shown.clear(); app._find_effective_range()
check("declining changes nothing", app.cfg_vars["brake_min"].get(), "0")

# an unusable file is refused, not applied
dyno_gui.filedialog.askopenfilename = lambda **k: dead
answer[0] = True
shown.clear(); app._find_effective_range()
check("unusable file reported as an error",
      any(x[0] == "error" for x in shown), True)
check("names what was wrong with it",
      any("does not show the brake responding" in x[2] for x in shown), True)
check("and the limits are untouched", app.cfg_vars["brake_max"].get(), "500")

# ── the guards that would have prevented those files ─────────────────────
class FakeSer:
    is_open = True
app.ser = FakeSer()
app.ready_flags["homed"] = True
with app._lock:
    app.live["rpm"] = 0.0

app._last_data_at = None                       # nothing has ever arrived
shown.clear(); app._start_brake_char()
check("no sweep without telemetry", app._char_active, False)
check("said the port is open but silent",
      any("no readings are arriving" in x[2] for x in shown), True)
check("said none had arrived at all",
      any("none have arrived" in x[2] for x in shown), True)

app._last_data_at = time.monotonic() - 30.0    # stale
shown.clear(); app._start_brake_char()
check("stale telemetry also refused", app._char_active, False)
check("and says how long ago", any("s ago" in x[2] for x in shown), True)

app._last_data_at = time.monotonic()           # live
check("live telemetry passes the check", app._telemetry_is_live(), True)
check("age is reported", app._telemetry_age_s() < 1.0, True)

# telemetry dying mid-sweep stops it rather than logging zeros
app._char_active = True
app._char_rows = []
app._char_phase_sent = None
app._char_home = 0
app._char_hi = 500.0
app._char_engaged = False
app._char_base_psi = None
app._char_susp_since = None
app._char_stall_at = None
app._char_t0 = time.monotonic()
app.root.after = lambda ms, fn=None, *a: None
app._char_tick()
check("a live tick records a sample", len(app._char_rows), 1)
app._last_data_at = time.monotonic() - 30.0
app._char_tick()
check("sweep aborted when readings stop", app._char_active, False)
check("abort says why",
      "stopped arriving" in app.char_progress.cget("text"), True)

app.ser = None
app.on_close()
print("FAILURES: " + ("none" if not fails else "\n  " + "\n  ".join(fails)))
raise SystemExit(1 if fails else 0)

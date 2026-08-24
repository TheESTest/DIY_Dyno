"""The gains belong to a brake span, and only to that span.

Regression for a real failure: Apply drivetrain inferred "the old scale" from
the brake range fields, which the effective-range finder also writes. Alternating
between the two rescaled the gains every cycle, compounding without bound, and
produced a Kp 57x too small that still looked like a plausible number. On the
stand the brake commanded 12 steps where it needed 140.
"""
import os, tempfile, tkinter as tk
os.environ.setdefault("MPLBACKEND", "Agg")
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
    print(("  ok   " if ok else "  FAIL ") + f"{what} ({got:.4g})")

answer = [True]
messagebox.showinfo = messagebox.showwarning = messagebox.showerror = lambda *a, **k: None
messagebox.askyesno = lambda *a, **k: answer[0]

root = tk.Tk(); app = dyno_gui.DynoApp(root)
app._send = lambda c: None

def kp():
    return float(app.pid_vars["kp"].get())

def authority():
    """The error it takes to reach full brake - the thing that must not drift."""
    return app._gain_span / kp()

def set_range(lo, hi):
    app.cfg_vars["brake_min"].set(str(lo))
    app.cfg_vars["brake_max"].set(str(hi))

def find_range(lo, hi):
    """What the effective-range finder does to the fields."""
    set_range(lo, hi)
    app._rescale_gains_for_span(app._current_span(), "measured range")

# --- the baseline exists and is the startup span -------------------------
check("a baseline is established at startup", app._gain_span is not None, True)
near("and it is the startup span", app._gain_span, app._current_span(), 0.01)

app.cfg_vars["cam_angle"].set("45")
set_range(0, 500)
app.pid_vars["kp"].set("0.5"); app.pid_vars["ki"].set("0.8")
app.pid_sweep_vars["kp"].set("0.3")
app._gain_span = 500.0
start_authority = authority()
near("start: full brake at 1000 RPM of error", start_authority, 1000, 1)

# --- narrowing to the measured range keeps the authority -----------------
find_range(140, 420)
near("narrowed span recorded", app._gain_span, 280, 0.01)
near("Kp scaled with it", kp(), 0.28, 0.001)
near("authority preserved", authority(), start_authority, 1)

# --- the sequence that used to compound ----------------------------------
app._apply_drivetrain()
first = (kp(), app._gain_span)
for i in range(4):
    app._apply_drivetrain()
    check(f"Apply #{i + 2} changes nothing", (kp(), app._gain_span), first)
near("authority still preserved", authority(), start_authority, 1)

find_range(140, 420)
after_find = kp()
app._apply_drivetrain()
back = kp()
find_range(140, 420)
check("alternating returns to the same gain", round(kp(), 9), round(after_find, 9))
app._apply_drivetrain()
check("and back again to the same one", round(kp(), 9), round(back, 9))
near("no drift in authority after four alternations",
     authority(), start_authority, 1)

# --- the min is physical and must never be scaled ------------------------
set_range(140, 420)
app._gain_span = app._current_span()
app.cfg_vars["cam_angle"].set("60")
app._apply_drivetrain()
check("takeup left alone by the drivetrain",
      app.cfg_vars["brake_min"].get(), "140")
check("only the max follows the drivetrain",
      app.cfg_vars["brake_max"].get(), "667")

# a min above the new travel would be rejected by the controller, so it resets
set_range(900, 1000)
app._gain_span = 100.0
app.cfg_vars["cam_angle"].set("45")
app._apply_drivetrain()
check("an impossible min is reset rather than left",
      float(app.cfg_vars["brake_min"].get()) < float(app.cfg_vars["brake_max"].get()),
      True)
check("and the result passes validation", app._validate_cfg(), [])

# --- step rates come from the driver, not from any span factor -----------
app.cfg_vars["driver_steps"].set("400")
app.cfg_vars["cam_angle"].set("45")
app._apply_drivetrain()
near("speed is 2 motor rev/s", float(app.cfg_vars["step_speed"].get()), 800, 1)
near("accel matches", float(app.cfg_vars["step_accel"].get()), 4000, 1)
app.cfg_vars["driver_steps"].set("5000")
app._apply_drivetrain()
near("and follows a driver change", float(app.cfg_vars["step_speed"].get()),
     10000, 1)
app.cfg_vars["driver_steps"].set("400")
app._apply_drivetrain()

# --- declining still records the baseline --------------------------------
# Otherwise the next change is measured from a span the gains no longer match,
# which is how the drift started.
answer[0] = False
set_range(0, 500)
app._gain_span = 500.0
before = kp()
find_range(100, 300)
check("declining leaves the gains alone", kp(), before)
near("but the baseline moves anyway", app._gain_span, 200, 0.01)
answer[0] = True

# --- it survives a profile round trip ------------------------------------
set_range(140, 420)
app._gain_span = 280.0
app.pid_vars["kp"].set("0.28")
snap = app._profile_snapshot()
check("baseline stored in the profile", "gain_span" in snap, True)
app._gain_span = 99999.0
app._apply_profile(snap)
near("baseline restored", app._gain_span, 280, 0.01)
# and a profile written before this existed must not poison it
old_style = dict(snap)
old_style.pop("gain_span")
app._apply_profile(old_style)
near("an older profile falls back to the current span",
     app._gain_span, app._current_span(), 0.01)

app.on_close()
print("FAILURES: " + ("none" if not fails else "\n  " + "\n  ".join(fails)))
raise SystemExit(1 if fails else 0)

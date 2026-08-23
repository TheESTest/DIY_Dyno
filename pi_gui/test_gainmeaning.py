"""Kp translated into the error it takes to reach full brake.

Exists because a gain left behind by a drivetrain change was 57x too small and
still looked like a plausible number.
"""
import os, tkinter as tk
os.environ.setdefault("MPLBACKEND", "Agg")
import dyno_gui
import tempfile as _tf, os as _os
dyno_gui.SESSION_FILE = _os.path.join(_tf.mkdtemp(), "session.json")
from dyno_gui import messagebox
messagebox.showinfo = messagebox.showwarning = messagebox.showerror = lambda *a, **k: None

fails = []
def check(what, got, want):
    ok = got == want
    if not ok:
        fails.append(f"{what}: got {got!r} want {want!r}")
    print(("  ok   " if ok else "  FAIL ") + what)

root = tk.Tk(); app = dyno_gui.DynoApp(root)
app._send = lambda c: None

def state(lo, hi, kp, ki="0.5", kps="0.3"):
    app.cfg_vars["brake_min"].set(lo); app.cfg_vars["brake_max"].set(hi)
    app.pid_vars["kp"].set(kp); app.pid_vars["ki"].set(ki)
    app.pid_sweep_vars["kp"].set(kps)
    app._update_gain_meaning()
    return app.gain_meaning.cget("text"), str(app.gain_meaning.cget("foreground"))

# --- the arithmetic ------------------------------------------------------
t, c = state("140", "470", "0.33")
check("span and full-brake error stated", "over 330 steps" in t, True)
check("1000 RPM of error for Kp 0.33", "1,000 RPM of error" in t, True)
check("a sane gain is not flagged", c, "gray")

t, c = state("0", "500", "0.5")
check("full brake at 1000 for 500 steps", "1,000 RPM of error" in t, True)

# --- the case that actually happened -------------------------------------
t, c = state("0", "667", "0.0117754", ki="0")
check("a 57x-too-small gain is called out", "far too weak" in t, True)
check("and flagged red", c, "#B03A2E")
check("the implied error is enormous", "56,644 RPM" in t, True)
check("zero Ki called out", "Ki is zero" in t, True)

# --- the opposite mistake ------------------------------------------------
t, c = state("140", "470", "10")
check("an over-stiff gain is called out", "close to on/off" in t, True)
check("and flagged red", c, "#B03A2E")

# --- Ki alone is a warning, not an error ---------------------------------
t, c = state("140", "470", "0.33", ki="0")
check("zero Ki alone is amber", c, "#B9770E")
check("and explains the dead travel problem", "dead travel" in t, True)
t, c = state("140", "470", "0.33", ki="0.5")
check("a non-zero Ki is silent", "Ki is zero" in t, False)

# --- it updates as the fields are typed ----------------------------------
app.cfg_vars["brake_max"].set("470")
app.pid_vars["kp"].set("0.33")
before = app.gain_meaning.cget("text")
app.pid_vars["kp"].set("0.66")
check("changing Kp updates it", app.gain_meaning.cget("text") != before, True)
before = app.gain_meaning.cget("text")
app.cfg_vars["brake_max"].set("900")
check("changing the range updates it too",
      app.gain_meaning.cget("text") != before, True)

# --- nonsense must not crash it ------------------------------------------
for lo, hi, kp in (("abc", "470", "0.3"), ("140", "140", "0.3"),
                   ("140", "470", "0"), ("140", "470", "-1")):
    app.cfg_vars["brake_min"].set(lo); app.cfg_vars["brake_max"].set(hi)
    app.pid_vars["kp"].set(kp)
    app._update_gain_meaning()
    check(f"survives ({lo},{hi},{kp})", isinstance(app.gain_meaning.cget("text"), str), True)

app.on_close()
print("FAILURES: " + ("none" if not fails else "\n  " + "\n  ".join(fails)))
raise SystemExit(1 if fails else 0)

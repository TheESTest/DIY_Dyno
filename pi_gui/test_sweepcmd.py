"""The characterisation sweep commands each leg once, not every tick.

Re-issuing a moving target at 20 Hz makes the controller decelerate to a stop
at every intermediate point, which is what made the motor lurch.
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
sent = []; app._send = lambda c: sent.append(c)

app._char_home = 0
app._char_hi = 6250.0
app._char_phase_sent = None
app._char_rows = []
app._char_engaged = False
app._char_base_psi = None
app._char_susp_since = None
app._char_stall_at = None
app._char_active = True
app.cfg_vars["char_stop_on_stall"].set(False)

# walk the whole sweep without the tk timer, driving the tick body directly
import time as _time
t0 = _time.monotonic()
app._char_t0 = t0
orig_after = app.root.after
app.root.after = lambda ms, fn=None, *a: None      # stop it rescheduling itself
for step in range(int((dyno_gui.CHAR_UP_S + dyno_gui.CHAR_HOLD_S
                       + dyno_gui.CHAR_DOWN_S) / 0.05) + 4):
    app._char_t0 = _time.monotonic() - step * 0.05
    app._last_data_at = _time.monotonic()      # a live controller, faked
    if not app._char_active:
        break
    app._char_tick()
app.root.after = orig_after

sweeps = [c for c in sent if c.startswith("BRAKE_SWEEP,")]
plain = [c for c in sent if c.startswith("BRAKE,")]
check("one command per moving leg", len(sweeps), 2)
check("outward leg commanded to the top over its full duration",
      sweeps[0], f"BRAKE_SWEEP,6250,{int(dyno_gui.CHAR_UP_S * 1000)}")
check("return leg commanded back home",
      sweeps[1], f"BRAKE_SWEEP,0,{int(dyno_gui.CHAR_DOWN_S * 1000)}")
check("no per-tick position commands", plain, ["BRAKE,0"])
check("far fewer commands than ticks", len(sent) < 5, True)
check("samples still collected at 20 Hz", len(app._char_rows) > 300, True)

app.on_close()
print("FAILURES: " + ("none" if not fails else "\n  " + "\n  ".join(fails)))
raise SystemExit(1 if fails else 0)

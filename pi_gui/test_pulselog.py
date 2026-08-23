"""Raw pulse capture, and what it can tell us about dropped teeth."""
import os, csv, glob, tempfile, tkinter as tk
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
    print(("  ok   " if ok else "  FAIL ") + f"{what} ({got:.2f})")

shown = []
messagebox.showinfo = lambda t, m="", **k: shown.append(("info", t, m))
messagebox.showwarning = lambda t, m="", **k: shown.append(("warn", t, m))
messagebox.showerror = lambda t, m="", **k: shown.append(("error", t, m))

R = dyno_gui.DynoApp.pulse_report

def train(n, interval, drop_at=(), noise_at=()):
    """A synthetic pulse train; drop_at indexes teeth that never arrive."""
    out, t, since = [], 0, 0
    for i in range(n):
        t += interval
        since += interval
        if i in drop_at:
            continue                      # the tooth simply never arrives
        out.append((t, since, 1))
        since = 0
        if i in noise_at:                 # a rejected edge just after a real one
            out.append((t + 300, 300, 0))
    return out

# --- a clean pickup ------------------------------------------------------
clean = train(200, 13333)
r = R(clean, 3)
check("a clean train is usable", r["usable"], True)
check("nothing counted as dropped", r["dropped_estimate"], 0)
check("dropout percentage is zero", round(r["dropout_pct"], 3), 0.0)
check("median interval recovered", r["median_interval_us"], 13333)
near("RPM from the median", r["rpm_from_median"], 1500, 1)
check("no rejected edges", r["rejected"], 0)

# --- teeth that go missing ----------------------------------------------
missing = train(200, 13333, drop_at=(50, 100, 150))
r = R(missing, 3)
check("three missing teeth are found", r["dropped_estimate"], 3)
near("as a percentage", r["dropout_pct"], 100 * 3 / 200, 0.2)
check("reported as double gaps", r["multiples"].get(2), 3)

# two in a row shows up as a triple gap
back_to_back = train(200, 13333, drop_at=(50, 51))
r = R(back_to_back, 3)
check("two consecutive misses counted as two", r["dropped_estimate"], 2)
check("and seen as one triple gap", r["multiples"].get(3), 1)

# --- a changing engine speed must not read as dropouts -------------------
accel, t, prev = [], 0, None
for i in range(200):
    iv = int(13333 * (1 - i * 0.002))      # speeding up steadily
    t += iv
    accel.append((t, iv, 1))
r = R(accel, 3)
check("acceleration is not mistaken for dropouts", r["dropped_estimate"], 0)

# --- noise edges are counted separately, not as teeth --------------------
noisy = train(200, 13333, noise_at=(10, 20, 30))
r = R(noisy, 3)
check("rejected edges counted", r["rejected"], 3)
check("and not confused with dropouts", r["dropped_estimate"], 0)
check("accepted count excludes them", r["accepted"], 200)

# --- too little to say anything -----------------------------------------
check("a short capture is refused", R(train(5, 13333), 3)["usable"], False)
check("and says why", "only" in R(train(5, 13333), 3)["reason"], True)
check("an empty capture is refused", R([], 3)["usable"], False)

# --- through the GUI -----------------------------------------------------
root = tk.Tk(); app = dyno_gui.DynoApp(root)
sent = []; app._send = lambda c: sent.append(c)

check("capture length defaults to 10 s", app.cfg_vars["pulse_secs"].get(), "10")

class FakeSer:
    is_open = True
app.ser = FakeSer()
import time as _t
app._last_data_at = _t.monotonic()
sent.clear(); app._start_pulse_capture()
check("capture requested", "PULSELOG,10" in sent, True)

for bad in ("0", "61", "x"):
    app.cfg_vars["pulse_secs"].set(bad)
    sent.clear(); shown.clear(); app._start_pulse_capture()
    check(f"{bad} seconds refused", sent, [])
app.cfg_vars["pulse_secs"].set("10")

# it will not capture into a dead link
app._last_data_at = None
sent.clear(); shown.clear(); app._start_pulse_capture()
check("no capture without telemetry", sent, [])
check("and says so", any("No readings" in x[2] for x in shown), True)
app._last_data_at = _t.monotonic()

# --- the board's lines are parsed and the file written -------------------
d = tempfile.mkdtemp()
app.cfg_vars["data_dir"].set(d)
app._parse_line("PULSELOG_START,10,3")
check("capture marked active", app._pulse_active, True)
check("pulses per rev taken from the board", app._pulse_ppr, 3)
for t_us, dt, ok in missing:
    app._parse_line(f"P,{t_us},{dt},{ok}")
check("pulses collected", len(app._pulses), len(missing))
shown.clear()
app._finish_pulse_capture("PULSELOG_DONE,197,0")
check("capture no longer active", app._pulse_active, False)

files = glob.glob(os.path.join(d, "pulses_*.csv"))
check("a pulse file was written", len(files), 1)
rows = list(csv.reader(open(files[0])))
check("header names the columns",
      rows[0], ["Micros", "Interval_us", "Accepted", "Implied_RPM",
                "Multiple_of_median"])
check("every pulse written", len(rows) - 1, len(missing))
mult = [float(r[4]) for r in rows[1:] if r[4]]
check("the gaps stand out as multiples", sum(1 for m in mult if m > 1.8), 3)
check("result reported", any("missed" in x[2] or "missed" in x[1]
                             for x in shown) or
      "missed" in app.pulse_status.cget("text"), True)
check("dropouts flagged in the status line",
      "3 missed" in app.pulse_status.cget("text"), True)

# a full board buffer is reported rather than hidden
app._pulses = list(clean)
shown.clear()
app._finish_pulse_capture("PULSELOG_DONE,200,17")
check("lost events surfaced", "lost to a full buffer" in app.pulse_status.cget("text"),
      True)

app.ser = None
app.on_close()
print("FAILURES: " + ("none" if not fails else "\n  " + "\n  ".join(fails)))
raise SystemExit(1 if fails else 0)

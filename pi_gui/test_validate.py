"""Settings are validated against the firmware's own limits before sending.

The case this exists for: a profile holding a value the board will reject
should be caught here, not half-applied and reported as a controller error.
"""
import os, tkinter as tk
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

shown = []
messagebox.showerror = lambda t, m, **k: shown.append((t, m))
messagebox.showinfo = messagebox.showwarning = lambda *a, **k: None

root = tk.Tk(); app = dyno_gui.DynoApp(root)
sent = []; app._send = lambda c: sent.append(c)

# --- a clean set passes ---------------------------------------------------
check("defaults are valid", app._validate_cfg(), [])

# --- the value that actually bit: rpm_avg 500 is now in range ------------
app.cfg_vars["rpm_avg"].set("500")
check("500 is accepted after the cap was raised", app._validate_cfg(), [])
app.cfg_vars["rpm_avg"].set("501")
p = app._validate_cfg()
check("above the cap is caught", len(p), 1)
check("says which field and what the limit is",
      "Average over" in p[0] and "500" in p[0], True)
app.cfg_vars["rpm_avg"].set("0")
check("zero is caught", len(app._validate_cfg()), 1)
app.cfg_vars["rpm_avg"].set("abc")
check("nonsense is caught", "not a number" in app._validate_cfg()[0], True)
app.cfg_vars["rpm_avg"].set("3")

# --- the other firmware limits -------------------------------------------
cases = [
    ("teeth", "0", "pulses per rev below 1"),
    ("teeth", "61", "pulses per rev above 60"),
    ("rpm_extrap_n", "11", "fit points above 10"),
    ("rpm_extrap_max", "0", "max run below 1"),
    ("rpm_median", "4", "median not one of 1/3/5/7"),
    ("rpm_ratio", "0.5", "ratio gate between 0 and 1"),
    ("rpm_slew", "-1", "negative slew"),
    ("drive_ratio", "0", "zero drive ratio"),
]
for key, bad, what in cases:
    good = app.cfg_vars[key].get()
    app.cfg_vars[key].set(bad)
    check(f"catches {what}", len(app._validate_cfg()) >= 1, True)
    app.cfg_vars[key].set(good)

# a band whose max is not clear of its min
app.cfg_vars["rpm_band_min"].set("1000")
app.cfg_vars["rpm_band_max"].set("1050")
check("catches too-narrow band",
      any("at least 100 above" in x for x in app._validate_cfg()), True)
app.cfg_vars["rpm_band_min"].set("800")
app.cfg_vars["rpm_band_max"].set("6000")
check("valid band passes", app._validate_cfg(), [])

# --- nothing is sent when the set is bad ---------------------------------
class FakeSerial:
    is_open = True
app.ser = FakeSerial()
app.cfg_vars["rpm_avg"].set("500000")
sent.clear(); shown.clear()
app._send_all_config()
check("send all sends nothing when invalid", sent, [])
check("and says why", len(shown), 1)
check("names the offending field", "Average over" in shown[0][1], True)

app.cfg_vars["rpm_avg"].set("3")
sent.clear(); shown.clear()
app._send_all_config()
check("valid set is sent", any(c.startswith("RPM_AVG,") for c in sent), True)
check("no error raised", shown, [])

# --- the lag readout ------------------------------------------------------
app.cfg_vars["teeth"].set("3")
app.cfg_vars["rpm_avg"].set("3")
with app._lock:
    app.live["rpm"] = 2000.0
lag = app._rpm_avg_lag_s()
check("short window is quick", round(lag, 3), 0.03)
app.cfg_vars["rpm_avg"].set("500")
lag = app._rpm_avg_lag_s()
check("500 pulses is 5 s at 2000 RPM / 3 teeth", round(lag, 2), 5.0)
app._update_gui()
check("lag shown", "5.00 s" in app.rpm_lag_label.cget("text"), True)
check("and called out as too slow",
      "too slow" in app.rpm_lag_label.cget("text"), True)
check("flagged red", str(app.rpm_lag_label.cget("foreground")), "#B03A2E")
app.cfg_vars["rpm_avg"].set("3")
app._update_gui()
check("short window not flagged",
      "too slow" in app.rpm_lag_label.cget("text"), False)

app.ser = None
app.on_close()
print("FAILURES: " + ("none" if not fails else "\n  " + "\n  ".join(fails)))
raise SystemExit(1 if fails else 0)

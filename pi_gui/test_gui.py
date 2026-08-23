"""Drive the GUI's parsing and calibration paths with synthetic controller
traffic. Verifies the things that used to be silently dropped."""
import tkinter as tk

import dyno_gui
# Keep the suite off the real session file: DynoApp saves settings on close,
# so without this every test would leave its values behind for the next one
# and for the operator's next real start.
import tempfile as _tf, os as _os
dyno_gui.SESSION_FILE = _os.path.join(_tf.mkdtemp(), 'session.json')

from dyno_gui import messagebox

fails = []


def check(name, got, want):
    ok = got == want
    print(f"{'PASS' if ok else 'FAIL'}  {name}: got {got!r} want {want!r}")
    if not ok:
        fails.append(name)


# Swallow dialogs so the calibration paths run unattended.
messagebox.showinfo = lambda *a, **k: None
messagebox.showwarning = lambda *a, **k: None
messagebox.showerror = lambda *a, **k: None

root = tk.Tk()
app = dyno_gui.DynoApp(root)

# ── DATA with the new trailing fields ────────────────────────
app._parse_line("DATA,12345,3210.5,88.25,1234.5,1234.5,777.0,2500,3300.0,"
                "SWEEP,1.5,2.5,3.5,42.7,910.5,1,55.5")
check("rpm", app.live["rpm"], 3210.5)
check("torque", app.live["torque"], 88.25)
check("load mV (field 5)", app.live["adc0"], 1234.5)
check("pressure mV (field 6)", app.live["press_mv"], 777.0)
check("spare aux (field 16)", app.live["spare_aux"], 55.5)
check("brake pct", app.live["brake_pct"], 42.7)
check("brake psi", app.live["press_psi"], 910.5)
check("fault bits", app.live["faults"], 1)

# ── Old-format DATA (13 fields) must still parse ─────────────
app._parse_line("DATA,999,1000.0,10.0,5.0,5.0,6.0,100,1000.0,HOLD_RPM,0,0,0")
check("legacy frame rpm", app.live["rpm"], 1000.0)
check("legacy frame defaults pct", app.live["brake_pct"], 0.0)

# ── READY with and without the sim field ─────────────────────
app._parse_line("READY,1,1,1,1,0")
check("sim flag false", app.ready_flags["sim"], False)
app._parse_line("READY,1,1,1,1,1")
check("sim flag true", app.ready_flags["sim"], True)
app._parse_line("READY,1,1,1,1")
check("missing sim field treated as suspect", app.ready_flags["sim"], True)
app._parse_line("READY,1,1,1,1,0,1")
check("pressure ADC present", app.ready_flags["press_adc"], True)
app._parse_line("READY,1,1,1,1,0")
check("missing pressure ADC field reads absent", app.ready_flags["press_adc"], False)

# ── Errors and config are captured, not discarded ────────────
before = len(app.events)
app._parse_line("ERR,NOT_READY")
check("error recorded", app.last_error, "NOT_READY")
app._parse_line("CFG,BRAKE_RANGE,0,5000")
check("events queued", len(app.events) > before, True)

# ── Fault banner text ────────────────────────────────────────
app._update_alerts(dyno_gui.FAULT_TACH)
check("tach banner shown", "TACH SIGNAL LOST" in app.alert_var.get(), True)
app.last_error = ""
app._update_alerts(0)
check("banner clears", app.alert_var.get(), "")

# ── Load-cell calibration maths ──────────────────────────────
check("lb to N", round(app._weight_to_newtons(100, "lb"), 3), 444.822)
check("kg to N", round(app._weight_to_newtons(10, "kg"), 4), 98.0665)
app.calib_vars["zero_mv"].set("100")
app.calib_vars["load_mv"].set("600")
app.calib_vars["known_wt"].set("100")
app.calib_vars["wt_units"].set("lb")
app._compute_load_cell()
# 444.822 N over 500 mV = 0.889644 N/mV
check("derived scale", round(float(app.cfg_vars["cal_scale"].get()), 6), 0.889644)

# Identical readings must be refused, not divided by zero.
app.cfg_vars["cal_scale"].set("SENTINEL")
app.calib_vars["load_mv"].set("100")
app._compute_load_cell()
check("zero-delta refused", app.cfg_vars["cal_scale"].get(), "SENTINEL")

# ── Pressure two-point maths ─────────────────────────────────
app.press_cal_vars["mv1"].set("500")
app.press_cal_vars["psi1"].set("0")
app.press_cal_vars["mv2"].set("2500")
app.press_cal_vars["psi2"].set("2000")
app._compute_pressure()
check("psi per mV", round(float(app.cfg_vars["press_psi_mv"].get()), 6), 1.0)
check("pressure offset", round(float(app.cfg_vars["press_off_mv"].get()), 3), 500.0)

# ── Profile round-trip ───────────────────────────────────────
app.cfg_vars["teeth"].set("7")
app.pid_sweep_vars["kp"].set("0.42")
snap = app._profile_snapshot()
check("profile captures teeth", snap["teeth"], "7")
check("profile captures sweep gain", snap["pid_sweep"]["kp"], "0.42")
check("profile captures units", snap["units"], "lb-ft")

# ── Log cap announces itself instead of silently truncating ──
app.recording = True
app.run_rpm.extend([0.0] * dyno_gui.MAX_LOG_POINTS)
app._parse_line("DATA,1,100,1,1,1,1,0,100,SWEEP,0,0,0")
check("log cap warned", app._log_capped_warned, True)

root.destroy()
print()
print("FAILURES:", fails if fails else "none")

"""Second-pass tests: the transcript items added after the first review —
RPM conditioning settings, drive ratio, stepper motion, brake slider,
auto-save and the run folder.  Appended to the originals in test_gui.py.
"""
import os
import shutil
import tempfile
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


messagebox.showinfo = lambda *a, **k: None
messagebox.showwarning = lambda *a, **k: None
messagebox.showerror = lambda *a, **k: None

root = tk.Tk()
app = dyno_gui.DynoApp(root)

# Capture what would go down the wire instead of opening a port.
sent = []
app._send = lambda cmd: sent.append(cmd)

# RPM conditioning has its own suite now (test_rpmfilter.py); just confirm the
# wheel geometry still goes out with it.
sent.clear()
app.cfg_vars["teeth"].set("3")
app.cfg_vars["drive_ratio"].set("2.5")
app._send_rpm_cfg()
check("drive ratio sent", "RATIO,2.5" in sent, True)
check("teeth sent", "TEETH,3" in sent, True)

# ── Stepper motion goes out with the brake config ──
sent.clear()
app.cfg_vars["step_speed"].set("1800")
app.cfg_vars["step_accel"].set("900")
app.cfg_vars["brake_min"].set("10")
app.cfg_vars["brake_max"].set("4000")
app._send_brake_cfg()
check("stepper speed sent", "STEPPER_SPEED,1800" in sent, True)
check("stepper accel sent", "STEPPER_ACCEL,900" in sent, True)
check("brake range sent", "BRAKE_RANGE,10,4000" in sent, True)

# ── Brake slider tracks the configured range, cannot over-stroke ──
check("slider max follows range", float(app.brake_slider.cget("to")), 4000.0)
app.brake_slider_var.set(9999)
app.cfg_vars["brake_max"].set("2000")
app._sync_brake_slider()
check("slider clamped down with range", app.brake_slider_var.get() <= 2000, True)

sent.clear()
app.brake_slider_var.set(1500)
app._brake_slider_commit()
check("slider release commands brake", "BRAKE,1500" in sent, True)
check("entry mirrors slider", app.brake_var.get(), "1500")

# ── Auto-save writes a finished run without anyone clicking ──
tmp = tempfile.mkdtemp(prefix="dynotest_")
try:
    app.cfg_vars["data_dir"].set(tmp)
    app.cfg_vars["run_prefix"].set("pull")
    app.cfg_vars["autosave"].set(True)
    app.recorded_torque_is_nm = True
    with app._lock:
        app.log_rows.append(["1", "1000.0", "50.0", "7.0", "1.0", "1.0", "2.0",
                             "100", "1000.0", "SWEEP", "0", "0", "0",
                             "2.0", "0.0", "0"])
    app._autosave_run()
    files = [f for f in os.listdir(tmp) if f.endswith(".csv")]
    check("auto-saved one file", len(files), 1)
    check("uses the prefix", files[0].startswith("pull_"), True)
    with open(os.path.join(tmp, files[0])) as f:
        head = f.readline().strip().split(",")
    # Layout now leads with the quantities an operator actually reads.
    check("leads with the four key channels", head[:4],
          ["Time_s", "Timestamp_ms", "Target_RPM", "RPM"])
    check("carries raw and filtered torque",
          ["Torque_raw_lb-ft", "Torque_filt_lb-ft"] == head[4:6], True)
    check("carries raw and filtered HP", ["HP_raw", "HP_filt"] == head[6:8], True)
    check("still carries the pressure channel", "Pressure_mV" in head, True)
    check("still carries the spare aux", "Spare_Aux_mV" in head, True)

    # Off means off.
    app.cfg_vars["autosave"].set(False)
    app._autosave_run()
    check("no extra file when disabled",
          len([f for f in os.listdir(tmp) if f.endswith(".csv")]), 1)

    # A bad folder reports rather than throwing into the serial thread.
    app.cfg_vars["autosave"].set(True)
    app.cfg_vars["data_dir"].set(os.path.join(tmp, "nested", "deeper"))
    app._autosave_run()
    check("creates nested folders",
          os.path.isdir(os.path.join(tmp, "nested", "deeper")), True)
finally:
    shutil.rmtree(tmp, ignore_errors=True)

# ── Cam geometry ──
def set_cam(rows):
    for (xv, yv), pair in zip(app.cam_rows, rows + [("", "")] * len(app.cam_rows)):
        xv.set(pair[0]); yv.set(pair[1])

set_cam([("0", "0"), ("25", "5"), ("50", "40"), ("100", "100")])
check("blank rows ignored", app._cam_points(),
      [(0.0, 0.0), (25.0, 5.0), (50.0, 40.0), (100.0, 100.0)])

# Travel % must increase, or a position maps to two brake values.
set_cam([("0", "0"), ("50", "40"), ("20", "60")])
check("out-of-order rows refused", app._cam_points(), None)

# Half-filled row is a typo, not a data point.
set_cam([("0", "0"), ("50", "")])
check("half-filled row refused", app._cam_points(), None)

# Out-of-range percentages refused.
set_cam([("0", "0"), ("150", "40")])
check("over-100% refused", app._cam_points(), None)

# Eccentric model without steps/degree can't be evaluated — don't send it.
set_cam([])
sent.clear()
app.cfg_vars["cam_model"].set(dyno_gui.CAM_MODELS[1])
app.cfg_vars["cam_spd"].set("0")
app._send_cam_cfg()
check("eccentric without steps/deg not sent", sent, [])

# Table model with fewer than two points can't interpolate.
sent.clear()
app.cfg_vars["cam_model"].set(dyno_gui.CAM_MODELS[2])
set_cam([("10", "10")])
app._send_cam_cfg()
check("single-point table not sent", sent, [])

# A good table sends points then selects the model.
sent.clear()
set_cam([("0", "0"), ("50", "40"), ("100", "100")])
app.cfg_vars["cam_lin"].set(True)
app._send_cam_cfg()
check("point count sent", "CAM_NPTS,3" in sent, True)
check("points sent in order",
      [c for c in sent if c.startswith("CAM_PT,")],
      ["CAM_PT,0,0.0,0.0", "CAM_PT,1,50.0,40.0", "CAM_PT,2,100.0,100.0"])
check("model selected after points", sent.index("CAM_MODEL,2") > sent.index("CAM_NPTS,3"), True)
check("linearise flag sent", "CAM_LIN,1" in sent, True)

# ── Pressure sensor from the datasheet spec (500-4500 mV) ──
sent.clear()
app.cfg_vars["press_fs_psi"].set("2000")
app._pressure_from_spec()
check("spec zero point", float(app.cfg_vars["press_off_mv"].get()), 500.0)
check("spec scale for 2000 PSI", round(float(app.cfg_vars["press_psi_mv"].get()), 6), 0.5)
check("spec sends calibration", any(c.startswith("CAL_PRESS,500") for c in sent), True)
check("hardware scaling sent before calibration",
      [c.split(",")[0] for c in sent if c.split(",")[0] in
       ("PRESS_PGA", "PRESS_DIV", "CAL_PRESS", "PRESS_LIMIT")],
      ["PRESS_PGA", "PRESS_DIV", "CAL_PRESS", "PRESS_LIMIT"])
check("default ADC range is the widest",
      sent[[c.split(",")[0] for c in sent].index("PRESS_PGA")], "PRESS_PGA,0")
check("default divider is direct",
      sent[[c.split(",")[0] for c in sent].index("PRESS_DIV")], "PRESS_DIV,1.0")
check("spec seeds the two-point rows",
      (app.press_cal_vars["mv1"].get(), app.press_cal_vars["psi1"].get(),
       app.press_cal_vars["mv2"].get(), app.press_cal_vars["psi2"].get()),
      ("500", "0", "4500", "2000"))

# A different full scale on the same electrical span.
app.cfg_vars["press_fs_psi"].set("1000")
app._pressure_from_spec()
check("spec scale for 1000 PSI", round(float(app.cfg_vars["press_psi_mv"].get()), 6), 0.25)

# Nonsense full scale must not produce a calibration.
app.cfg_vars["press_psi_mv"].set("SENTINEL")
app.cfg_vars["press_fs_psi"].set("0")
app._pressure_from_spec()
check("zero full scale refused", app.cfg_vars["press_psi_mv"].get(), "SENTINEL")

# The measured two-point calibration must still override the spec.
app.press_cal_vars["mv1"].set("500");  app.press_cal_vars["psi1"].set("0")
app.press_cal_vars["mv2"].set("4500"); app.press_cal_vars["psi2"].set("2320")
app._compute_pressure()
check("gauge measurement overrides spec",
      round(float(app.cfg_vars["press_psi_mv"].get()), 4), 0.58)

# ── Brake travel defaults match the built geometry ──
check("full travel is 6250 microsteps", dyno_gui.BRAKE_FULL_TRAVEL_STEPS, 6250)
check("microsteps per degree", round(dyno_gui.CAM_STEPS_PER_DEGREE, 3), 138.889)
check("5000 microsteps/rev x 10:1 over 45 deg gives the travel",
      round(5000 * 10 * 45 / 360), dyno_gui.BRAKE_FULL_TRAVEL_STEPS)
# fresh instance: earlier tests in this file deliberately mutate step_speed
_r = tk.Tk(); _a = dyno_gui.DynoApp(_r)
check("stepper speed default sized for microsteps",
      int(_a.cfg_vars["step_speed"].get()) >= 10000, True)
check("acceleration raised to match",
      int(_a.cfg_vars["step_accel"].get()) >= 20000, True)
_r.destroy()

# ── Profile round-trips every new setting ──
snap = app._profile_snapshot()
for key in ("drive_ratio", "rpm_band_min", "rpm_band_max", "rpm_median", "rpm_ratio",
            "rpm_slew", "rpm_avg",
            "step_speed", "step_accel", "data_dir", "run_prefix", "autosave",
            "cam_model", "cam_spd", "cam_lin"):
    check(f"profile carries {key}", key in snap, True)
check("profile carries cam table", snap["cam_table"][1], ["50", "40"])

root.destroy()
print()
print("FAILURES:", fails if fails else "none")

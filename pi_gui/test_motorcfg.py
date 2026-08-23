"""Motor and drivetrain configuration, and the software panel.

The figure that matters: driver steps per rev already carries the
microstepping, so 400 on a 200-step motor is half stepping, and everything
downstream is derived rather than typed in twice.
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

asked = []
messagebox.showinfo = messagebox.showwarning = messagebox.showerror = lambda *a, **k: None
messagebox.askyesno = lambda t, m, **k: (asked.append((t, m)), True)[1]

# --- the maths, independent of any GUI ------------------------------------
d = dyno_gui.drivetrain(400, 10.0, 45.0, 200)
check("half stepping detected", d["microstep_factor"], 2.0)
check("steps per motor rev", d["per_motor_rev"], 400)
check("steps per cam rev", d["per_output_rev"], 4000.0)
check("steps per degree", round(d["per_degree"], 3), 11.111)
check("quarter cam turn is 1000", d["quarter_output_turn"], 1000.0)
check("45 deg of cam is 500", round(d["cam_travel"]), 500)

# the previous setup, for the factor between them
old = dyno_gui.drivetrain(5000, 10.0, 45.0, 200)
check("old setup was 6250", round(old["cam_travel"]), 6250)
check("change is a factor of 12.5",
      round(old["cam_travel"] / d["cam_travel"], 4), 12.5)

# --- rubbish in gives nothing, not a crash --------------------------------
for bad in ((0, 10, 45), (400, 0, 45), (400, 10, 0), (-400, 10, 45)):
    check(f"rejects {bad}", dyno_gui.drivetrain(*bad, 200), None)

root = tk.Tk(); app = dyno_gui.DynoApp(root)
sent = []; app._send = lambda c: sent.append(c)

# --- defaults reflect the rig as built ------------------------------------
check("motor steps default", app.cfg_vars["motor_steps"].get(), "200")
check("driver steps default", app.cfg_vars["driver_steps"].get(), "400")
check("gearbox default", app.cfg_vars["gearbox"].get(), "10.0")
check("brake range matches the drivetrain",
      app.cfg_vars["brake_max"].get(), "500")

# --- the readout updates as it is typed -----------------------------------
app.cfg_vars["driver_steps"].set("400")
app._update_drivetrain()
txt = app.drivetrain_label.cget("text")
check("readout names the cam revolution", "4000 steps per cam revolution" in txt, True)
check("readout gives the quarter turn", "quarter cam turn = 1000" in txt, True)
check("readout names the microstepping", "2x microstepping" in txt, True)
app.cfg_vars["driver_steps"].set("")
app._update_drivetrain()
check("unusable entry says so",
      "positive numbers" in app.drivetrain_label.cget("text"), True)
app.cfg_vars["driver_steps"].set("400")

# --- applying it rescales the gains with the travel -----------------------
app.cfg_vars["brake_max"].set("6250")          # pretend we are still on the old setup
app.pid_vars["kp"].set("6.25"); app.pid_vars["ki"].set("10.0")
app.pid_vars["kd"].set("0.25")
app.pid_sweep_vars["kp"].set("3.75")
app.cfg_vars["step_speed"].set("10000"); app.cfg_vars["step_accel"].set("50000")
asked.clear()
app._apply_drivetrain()
check("travel taken from the drivetrain", app.cfg_vars["brake_max"].get(), "500")
check("cam scale taken too", app.cfg_vars["cam_spd"].get(), "11.1111")
check("asked before rescaling", len(asked), 1)
check("said what the factor was", "0.08" in asked[0][1], True)
check("warned it would be too strong", "slams on" in asked[0][1], True)
check("hold Kp rescaled", float(app.pid_vars["kp"].get()), 0.5)
check("hold Ki rescaled", float(app.pid_vars["ki"].get()), 0.8)
check("hold Kd rescaled", float(app.pid_vars["kd"].get()), 0.02)
check("sweep Kp rescaled", float(app.pid_sweep_vars["kp"].get()), 0.3)
check("speed rescaled", float(app.cfg_vars["step_speed"].get()), 800.0)
check("acceleration rescaled", float(app.cfg_vars["step_accel"].get()), 4000.0)

# same drivetrain again is not a change, so nothing is asked
asked.clear()
app._apply_drivetrain()
check("no rescale prompt when nothing changed", len(asked), 0)
check("gains untouched", float(app.pid_vars["kp"].get()), 0.5)

# --- declining the rescale leaves the gains alone -------------------------
messagebox.askyesno = lambda t, m, **k: False
app.cfg_vars["brake_max"].set("6250")
app.pid_vars["kp"].set("6.25")
app._apply_drivetrain()
check("travel still applied when declined", app.cfg_vars["brake_max"].get(), "500")
check("gains left alone when declined", float(app.pid_vars["kp"].get()), 6.25)

# --- the software panel ---------------------------------------------------
check("interface version shown", dyno_gui.UI_VERSION in root.title(), True)
check("firmware unknown until the board answers",
      app.fw_version_label.cget("text"), "unknown")
app._parse_line("CFG,FW_VERSION,1.4.0,Aug 23 2026 09:00:00")
app._update_gui()
check("firmware version on the settings page",
      app.fw_version_label.cget("text"), "1.4.0")
check("build stamp shown too",
      app.fw_build_label.cget("text"), "Aug 23 2026 09:00:00")
check("no longer flagged",
      str(app.fw_version_label.cget("foreground")), "black")

check("update button exists", app.update_btn is not None, True)
check("update button greyed out", str(app.update_btn.cget("state")), "disabled")

# --- carried by profiles --------------------------------------------------
snap = app._profile_snapshot()
for k in ("motor_steps", "driver_steps", "gearbox", "cam_angle"):
    check(f"profile keeps {k}", k in snap, True)


# --- branding -------------------------------------------------------------
import os as _o
check("logo ships with the code", _o.path.exists(dyno_gui.LOGO_FILE), True)
check("logo loaded", app.logo_image is not None, True)
# The name must appear whether or not the image can be read.
found = []
def walk(w):
    for c in w.winfo_children():
        try:
            if c.cget("text") == "Centurial Inc":
                found.append(c)
        except Exception:
            pass
        walk(c)
walk(root)
check("company name shown", len(found) >= 1, True)

app.on_close()
print("FAILURES: " + ("none" if not fails else "\n  " + "\n  ".join(fails)))
raise SystemExit(1 if fails else 0)

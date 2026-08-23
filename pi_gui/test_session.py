"""Settings must survive a restart, and Restore Defaults must undo everything."""
import os, json, tkinter as tk
import dyno_gui
# Keep the suite off the real session file: DynoApp saves settings on close,
# so without this every test would leave its values behind for the next one
# and for the operator's next real start.
import tempfile as _tf, os as _os
dyno_gui.SESSION_FILE = _os.path.join(_tf.mkdtemp(), 'session.json')

from dyno_gui import messagebox
fails=[]
def check(n,g,w):
    ok=g==w; print(f"{'PASS' if ok else 'FAIL'}  {n}: got {g!r} want {w!r}")
    if not ok: fails.append(n)
messagebox.showinfo=messagebox.showwarning=messagebox.showerror=lambda *a,**k: None

SF = dyno_gui.SESSION_FILE
backup = None
if os.path.exists(SF):
    backup = open(SF).read(); os.remove(SF)
try:
    # --- first run: no session file, so defaults ---
    r1=tk.Tk(); a1=dyno_gui.DynoApp(r1)
    check("starts on defaults with no session file",
          a1.cfg_vars["cutoff_rpm"].get(), "1200")
    default_kp = a1.pid_vars["kp"].get()

    # operator changes a spread of things across different tabs
    a1.cfg_vars["cutoff_rpm"].set("1450")
    a1.cfg_vars["cal_scale"].set("0.87431")
    a1.cfg_vars["rpm_band_max"].set("7000")
    a1.pid_vars["kp"].set("9.9")
    a1.param_vars["end_rpm"].set("6200")
    a1.calib_vars["zero_mv"].set("845.25")
    a1.filter_param_vars["window"].set("42")
    a1.trace_vars["psi"].set(False)
    a1.units_var.set("Nm")
    a1.on_close()                      # closing writes the session
    check("session file written on close", os.path.exists(SF), True)

    # --- second run: everything should come back ---
    r2=tk.Tk(); a2=dyno_gui.DynoApp(r2)
    check("cutoff restored",      a2.cfg_vars["cutoff_rpm"].get(), "1450")
    check("calibration restored", a2.cfg_vars["cal_scale"].get(), "0.87431")
    check("RPM band restored",    a2.cfg_vars["rpm_band_max"].get(), "7000")
    check("hold gain restored",   a2.pid_vars["kp"].get(), "9.9")
    check("run params restored",  a2.param_vars["end_rpm"].get(), "6200")
    check("captured tare restored", a2.calib_vars["zero_mv"].get(), "845.25")
    check("analysis setting restored", a2.filter_param_vars["window"].get(), "42")
    check("trace visibility restored", a2.trace_vars["psi"].get(), False)
    check("units restored",       a2.units_var.get(), "Nm")

    # --- Restore Defaults puts it all back ---
    messagebox.askyesno = lambda *a, **k: True
    a2._restore_defaults()
    check("defaults: cutoff",   a2.cfg_vars["cutoff_rpm"].get(), "1200")
    check("defaults: cal",      a2.cfg_vars["cal_scale"].get(), "1.0")
    check("defaults: gain",     a2.pid_vars["kp"].get(), default_kp)
    check("defaults: captured tare cleared", a2.calib_vars["zero_mv"].get(), "")
    check("defaults: analysis", a2.filter_param_vars["window"].get(), "20")
    check("defaults: traces back on", a2.trace_vars["psi"].get(), True)
    check("defaults: units",    a2.units_var.get(), "lb-ft")

    # declining the prompt must change nothing
    a2.cfg_vars["cutoff_rpm"].set("999")
    messagebox.askyesno = lambda *a, **k: False
    a2._restore_defaults()
    check("declining leaves settings alone", a2.cfg_vars["cutoff_rpm"].get(), "999")
    r2.destroy()   # r1 was already destroyed by on_close()

    # --- a corrupt session file must not stop startup ---
    open(SF,"w").write("{ this is not json")
    r3=tk.Tk(); a3=dyno_gui.DynoApp(r3)
    check("survives a damaged session file",
          a3.cfg_vars["cutoff_rpm"].get(), "1200")
    r3.destroy()
finally:
    if backup is not None:
        open(SF,"w").write(backup)
    elif os.path.exists(SF):
        os.remove(SF)
print(); print("FAILURES:", fails if fails else "none")

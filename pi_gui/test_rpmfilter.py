"""RPM conditioning controls, glitch counter, and the conditions sidecar."""
import os, glob, json, shutil, tempfile, tkinter as tk
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
root=tk.Tk(); app=dyno_gui.DynoApp(root)
sent=[]; app._send=lambda c: sent.append(c)

# --- the four gates are sent independently ---
app.cfg_vars["rpm_band_min"].set("900")
app.cfg_vars["rpm_band_max"].set("6500")
app.cfg_vars["rpm_median"].set("5")
app.cfg_vars["rpm_ratio"].set("2.5")
app.cfg_vars["rpm_slew"].set("15000")
app.cfg_vars["rpm_avg"].set("4")
app._send_rpm_cfg()
for cmd in ("RPM_BAND,900,6500","RPM_MEDIAN,5","RPM_RATIO,2.5","RPM_SLEW,15000","RPM_AVG,4"):
    check(f"sends {cmd.split(',')[0]}", cmd in sent, True)
check("no stale RPM_FILTER command", any(c.startswith("RPM_FILTER") for c in sent), False)

# --- defaults are the ones that fix the observed noise ---
root2=tk.Tk(); a2=dyno_gui.DynoApp(root2)
check("band min defaults to idle-ish", a2.cfg_vars["rpm_band_min"].get(), "800")
check("band max defaults above redline", a2.cfg_vars["rpm_band_max"].get(), "6000")
check("median defaults on", a2.cfg_vars["rpm_median"].get(), "3")
check("ratio gate defaults on", a2.cfg_vars["rpm_ratio"].get(), "3.0")
root2.destroy()

# --- glitch counter is parsed and displayed ---
app._parse_line("DATA,1,2050,80,900,900,500,120,2000,HOLD_RPM,1,2,3,48,400,0,9.9,0")
app._update_gui()
check("no glitches shows plain", app.status_labels["glitches"].cget("text"), "0")
app._parse_line("DATA,2,2050,80,900,900,500,120,2000,HOLD_RPM,1,2,3,48,400,0,9.9,1477")
app._update_gui()
check("glitch count surfaces", app.status_labels["glitches"].cget("text"), "1477")
check("and is flagged red", str(app.status_labels["glitches"].cget("foreground")), "#B03A2E")

# --- conditions file written beside the run ---
tmp=tempfile.mkdtemp(prefix="cond_")
try:
    app.cfg_vars["data_dir"].set(tmp); app.cfg_vars["run_prefix"].set("pull")
    app.cfg_vars["autosave"].set(True); app.recorded_torque_is_nm=True
    app.recording=True
    for i in range(150):
        app._parse_line(f"DATA,{9000+i*50},{2500+i*10},{120+i*0.2},900,900,500,"
                        f"120,{2500+i*10},SWEEP,1,2,3,48,420,0,9.9,7")
    app.recording=False
    app._autosave_run()
    conds=glob.glob(os.path.join(tmp,"*_conditions.json"))
    check("conditions file written", len(conds), 1)
    d=json.load(open(conds[0]))
    check("has a timestamp", "_saved" in d, True)
    check("names its run file", d["_run_file"].endswith(".csv"), True)
    check("records the RPM band", (d["rpm_band_min"], d["rpm_band_max"]), ("900", "6500"))
    check("records the median window", d["rpm_median"], "5")
    check("records PID gains", "pid_sweep" in d and "kp" in d["pid_sweep"], True)
    check("records analysis settings", "analysis" in d, True)
    check("records units", d["_torque_units"], app.units_var.get())
    check("records sample count", d["_samples"], 150)
    # and it must still be loadable as a profile
    check("is a valid profile (has cfg keys)", "cutoff_rpm" in d, True)
    stem=conds[0].replace("_conditions.json","")
    check("sits beside the raw csv", os.path.exists(stem+".csv"), True)
    check("sits beside the filtered csv", os.path.exists(stem+"_filtered.csv"), True)
finally:
    shutil.rmtree(tmp, ignore_errors=True)

check("folder is dyno_runs beside the code",
      os.path.basename(dyno_gui.DEFAULT_DATA_DIR), "dyno_runs")
root.destroy()
print(); print("FAILURES:", fails if fails else "none")

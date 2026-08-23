"""A profile must round-trip every setting that describes the rig, and must
deliberately not carry session state."""
import json, os, tempfile, shutil, tkinter as tk
import dyno_gui
# Keep the suite off the real session file: DynoApp saves settings on close,
# so without this every test would leave its values behind for the next one
# and for the operator's next real start.
import tempfile as _tf, os as _os
dyno_gui.SESSION_FILE = _os.path.join(_tf.mkdtemp(), 'session.json')

from dyno_gui import messagebox, filedialog
fails=[]
def check(n,g,w):
    ok=g==w; print(f"{'PASS' if ok else 'FAIL'}  {n}: got {g!r} want {w!r}")
    if not ok: fails.append(n)
messagebox.showinfo=messagebox.showwarning=messagebox.showerror=lambda *a,**k: None
root=tk.Tk(); app=dyno_gui.DynoApp(root)

# --- coverage: what is still not captured? ---
def all_vars(a):
    found={}
    for name,val in vars(a).items():
        if isinstance(val, tk.Variable): found[name]=val
        elif isinstance(val, dict):
            for k,v in val.items():
                if isinstance(v, tk.Variable): found[f"{name}[{k}]"]=v
        elif isinstance(val,(list,tuple)):
            for i,item in enumerate(val):
                if isinstance(item,(list,tuple)):
                    for j,x in enumerate(item):
                        if isinstance(x, tk.Variable): found[f"{name}[{i}][{j}]"]=x
    return found

captured=set()
for g,vs in app._profile_groups().items():
    captured |= {id(v) for v in vs.values()}
captured |= {id(v) for v in app.cfg_vars.values()}
captured |= {id(v) for v in app.pid_vars.values()}
captured |= {id(v) for v in app.pid_sweep_vars.values()}
captured |= {id(v) for v in app.param_vars.values()}
captured.add(id(app.units_var))
for xv,yv in app.cam_rows: captured |= {id(xv), id(yv)}

found=all_vars(app)
missing=sorted(n for n,v in found.items() if id(v) not in captured)
print(f"  variables: {len(found)}, captured: {len(found)-len(missing)}, not: {len(missing)}")
print("  intentionally excluded:", missing)
expected_excluded={"alert_var","autoscroll_var","brake_slider_var","brake_var",
                   "port_var","replay_file_var","replay_motor_port_var",
                   "replay_progress_var","run_status_var","source_var"}
check("only session/UI state is left out", set(missing), expected_excluded)

# --- round trip ---
app.filter_type_var.set("Savitzky-Golay")
app.filter_param_vars["window"].set("31"); app.filter_param_vars["polyorder"].set("4")
app.rpm_bin_var.set("25"); app.rpm_min_var.set("2200"); app.rpm_max_var.set("6000")
app.despike_var.set(False); app.show_raw_var.set(False)
app.yd_vars["gauge"].set("7"); app.yd_vars["graph"].set("6"); app.yd_vars["spike"].set("2")
app.sae_apply_var.set(True); app.sae_vars["temp"].set("91.4"); app.sae_vars["hum"].set("22")
app.calib_vars["zero_mv"].set("812.5"); app.calib_vars["known_wt"].set("50")
app.calib_vars["wt_units"].set("kg")
app.press_cal_vars["mv1"].set("505"); app.press_cal_vars["psi2"].set("1980")
app.live_smooth_var.set(False); app.live_alpha_var.set("0.55")
app.replay_speed_var.set("2.0"); app.replay_motor_var.set(False)
app.cfg_vars["cutoff_rpm"].set("1350"); app.pid_sweep_vars["kp"].set("0.19")

tmp=tempfile.mkdtemp(prefix="prof_")
try:
    path=os.path.join(tmp,"p.json")
    filedialog.asksaveasfilename=lambda *a,**k: path
    filedialog.askopenfilename=lambda *a,**k: path
    app._save_profile()
    check("file written", os.path.exists(path), True)

    # wipe everything to defaults, then reload
    root2=tk.Tk(); app2=dyno_gui.DynoApp(root2)
    app2._load_profile()
    check("filter type", app2.filter_type_var.get(), "Savitzky-Golay")
    check("filter window", app2.filter_param_vars["window"].get(), "31")
    check("rpm bin", app2.rpm_bin_var.get(), "25")
    check("rpm min", app2.rpm_min_var.get(), "2200")
    check("despike bool restored as bool", app2.despike_var.get(), False)
    check("show raw bool", app2.show_raw_var.get(), False)
    check("yourdyno gauge", app2.yd_vars["gauge"].get(), "7")
    check("sae applied", app2.sae_apply_var.get(), True)
    check("sae temp", app2.sae_vars["temp"].get(), "91.4")
    check("captured zero mV", app2.calib_vars["zero_mv"].get(), "812.5")
    check("weight units", app2.calib_vars["wt_units"].get(), "kg")
    check("pressure point 1", app2.press_cal_vars["mv1"].get(), "505")
    check("pressure point 2", app2.press_cal_vars["psi2"].get(), "1980")
    check("live smoothing off", app2.live_smooth_var.get(), False)
    check("live alpha", app2.live_alpha_var.get(), "0.55")
    check("replay speed", app2.replay_speed_var.get(), "2.0")
    check("replay motor off", app2.replay_motor_var.get(), False)
    check("still restores cfg vars", app2.cfg_vars["cutoff_rpm"].get(), "1350")
    check("still restores sweep gains", app2.pid_sweep_vars["kp"].get(), "0.19")
    check("port NOT restored from profile", "port_var" in json.load(open(path)), False)
    root2.destroy()
finally:
    shutil.rmtree(tmp, ignore_errors=True)
root.destroy()
print(); print("FAILURES:", fails if fails else "none")

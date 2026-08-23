"""The run CSV must carry target RPM, actual RPM, torque and HP - raw AND
filtered - and must reload cleanly in our own tooling."""
import os, glob, shutil, tempfile, csv, tkinter as tk
import numpy as np
import dyno_gui, dyno_dsp as dsp
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

def frame(ms, rpm, tq, target):
    app._parse_line(f"DATA,{ms},{rpm},{tq},7.5,7.5,1500,120,{target},SWEEP,1,2,3,48,500,0,9.9")

app.recorded_torque_is_nm=True
app.units_var.set("lb-ft")
app.recording=True
for i in range(400):                       # a plausible sweep, 20 ms apart
    rpm=2500+i*7.5
    frame(500000+i*20, rpm, 150.0+25.0*np.sin(i/30.0), rpm-40)
app.recording=False

tmp=tempfile.mkdtemp(prefix="runcsv_")
try:
    path=os.path.join(tmp,"run.csv")
    n=app._write_run_csv(path)
    check("wrote every sample", n, 400)
    with open(path) as f:
        rows=list(csv.reader(f))
    hdr=rows[0]
    print("  header:", ",".join(hdr))
    for col in ("Target_RPM","RPM","Torque_raw_lb-ft","Torque_filt_lb-ft","HP_raw","HP_filt"):
        check(f"has {col}", col in hdr, True)
    check("Time_s is first so it reloads", hdr[0], "Time_s")

    i_t, i_tgt, i_rpm = hdr.index("Time_s"), hdr.index("Target_RPM"), hdr.index("RPM")
    i_tr, i_tf = hdr.index("Torque_raw_lb-ft"), hdr.index("Torque_filt_lb-ft")
    i_hr, i_hf = hdr.index("HP_raw"), hdr.index("HP_filt")
    r1 = rows[1]
    check("time starts at zero", float(r1[i_t]), 0.0)
    check("target and actual are distinct", float(r1[i_rpm])-float(r1[i_tgt]), 40.0)

    # HP must be derived from ACTUAL rpm and the torque in the same row.
    def hp(tq, rpm): return tq*rpm/dsp.HP_LBFT_CONST
    mid = rows[200]
    check("HP_raw = raw torque x actual RPM",
          round(float(mid[i_hr]),1), round(hp(float(mid[i_tr]), float(mid[i_rpm])),1))
    check("HP_filt = filtered torque x actual RPM",
          round(float(mid[i_hf]),1), round(hp(float(mid[i_tf]), float(mid[i_rpm])),1))

    # Filtered must actually be filtered - smoother than raw, not a copy.
    raw=np.array([float(r[i_tr]) for r in rows[1:]])
    flt=np.array([float(r[i_tf]) for r in rows[1:]])
    check("filtered column is populated", np.isfinite(flt).all(), True)
    check("filtered differs from raw", not np.allclose(raw, flt), True)
    check("filtered is smoother than raw",
          np.abs(np.diff(flt)).mean() < np.abs(np.diff(raw)).mean(), True)

    # And the file must load back in our own loader with a sane timebase.
    rec=dsp.load_recording(path)
    check("reloads with a sane duration", 7.0 < rec.duration < 9.0, True)
    check("reload finds RPM", rec.rpm is not None, True)
    print(f"  reloaded: {rec.n} pts, {rec.duration:.2f}s @ {rec.rate_hz:.0f}Hz")
finally:
    shutil.rmtree(tmp, ignore_errors=True)
root.destroy()
print(); print("FAILURES:", fails if fails else "none")

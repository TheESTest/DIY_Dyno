"""Scrolling time axis, and that a run saves raw AND filtered together."""
import os, glob, shutil, tempfile, tkinter as tk
import numpy as np
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
def frame(ms,rpm,tq):
    app._parse_line(f"DATA,{ms},{rpm},{tq},0,0,500,0,{rpm},IDLE,0,0,0,0,0,0,0")

# --- axis scrolls past the window instead of sticking at 0-60 ---
app.recording=False
for i in range(40):            # 40 s, board clock starting at 100 s
    frame(100000+i*1000, 2000, 30.0)
app._redraw_live_plot()
x0=app.ax_rpm.get_xlim()
check("early window starts at 0", x0[0], 0.0)

for i in range(40,150):        # out to 150 s of monitoring
    frame(100000+i*1000, 2000, 30.0)
app._redraw_live_plot()
x1=app.ax_rpm.get_xlim()
check("window scrolled forward", x1[0] > x0[0], True)
check("window is the configured width", round(x1[1]-x1[0]), dyno_gui.MONITOR_WINDOW_S)
check("right edge tracks the latest sample", x1[1] >= 149.0, True)
check("axis is not stuck at 0", x1[0] >= 80.0, True)

# --- a completed run saves raw and filtered together ---
tmp=tempfile.mkdtemp(prefix="dynosave_")
try:
    app.cfg_vars["data_dir"].set(tmp); app.cfg_vars["run_prefix"].set("pull")
    app.cfg_vars["autosave"].set(True)
    app.recorded_torque_is_nm=True
    app.recording=True
    # a plausible sweep so the RPM-binned filter has something to chew on
    for i in range(300):
        rpm=2500+i*10
        frame(200000+i*20, rpm, 100.0+20.0*np.sin(i/25.0))
    app.recording=False
    app._autosave_run()
    raws=sorted(glob.glob(os.path.join(tmp,"pull_*.csv")))
    filt=[f for f in raws if f.endswith("_filtered.csv")]
    raw=[f for f in raws if not f.endswith("_filtered.csv")]
    check("one raw file written", len(raw), 1)
    check("one filtered file written", len(filt), 1)
    check("filtered shares the raw timestamp",
          os.path.basename(filt[0]).replace("_filtered.csv",""),
          os.path.basename(raw[0]).replace(".csv",""))
    with open(raw[0]) as f:
        rh=f.readline().strip().split(",")
    with open(filt[0]) as f:
        f.readline()                     # settings comment
        fh=f.readline().strip().split(",")
    check("raw file is per-sample and time-first", rh[0], "Time_s")
    check("filtered file is per-RPM", fh[0], "RPM")
    check("filtered has raw and filtered columns",
          [c.split("_")[-1] for c in fh[1:]], ["lb-ft","lb-ft","raw","filt"])
    with open(filt[0]) as f:
        rows=sum(1 for _ in f)
    check("filtered file has data", rows > 3, True)
finally:
    shutil.rmtree(tmp, ignore_errors=True)
root.destroy()
print(); print("FAILURES:", fails if fails else "none")

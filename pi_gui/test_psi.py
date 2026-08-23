"""Brake PSI must be plotted on its own axis and saved with the run."""
import os, glob, csv, shutil, tempfile, tkinter as tk
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

def frame(ms, rpm, tq, psi, target):
    app._parse_line(f"DATA,{ms},{rpm},{tq},7.5,7.5,1500,120,{target},SWEEP,1,2,3,48,{psi},0,9.9")

# --- plotted ---
check("PSI has its own axis", hasattr(app, "ax_psi"), True)
check("axis is offset from the torque axis",
      app.ax_psi.spines["right"].get_position(), ("outward", 46))
check("PSI is in the legend",
      "Brake PSI" in [t.get_text() for t in app.ax_rpm.get_legend().get_texts()], True)

app.recording=False
for i in range(50):
    frame(1000+i*50, 3000, 120.0, 300.0+i*8, 3000)
check("monitor collects PSI", len(app.mon_psi), 50)
app._redraw_live_plot()
check("PSI line has points", len(app.line_psi.get_xdata()), 50)
lo, hi = app.ax_psi.get_ylim()
check("PSI axis scaled to the data", hi >= 692.0, True)
check("PSI axis is separate from torque",
      app.ax_psi.get_ylim() != app.ax_pwr.get_ylim(), True)

# PSI must not distort the torque axis
tq_lo, tq_hi = app.ax_pwr.get_ylim()
check("torque axis unaffected by a 700 PSI trace", tq_hi < 400, True)

# --- saved ---
app.recording=True
for i in range(120):
    frame(50000+i*50, 2500+i*20, 150.0, 400.0+i*2, 2500+i*20)
app.recording=False
tmp=tempfile.mkdtemp(prefix="psi_")
try:
    path=os.path.join(tmp,"r.csv")
    app._write_run_csv(path)
    rows=list(csv.reader(open(path)))
    hdr=rows[0]
    check("run CSV carries Brake_PSI", "Brake_PSI" in hdr, True)
    i_psi=hdr.index("Brake_PSI")
    vals=[float(r[i_psi]) for r in rows[1:]]
    check("PSI values are real, not zeros", vals[0], 400.0)
    check("PSI tracks over time", vals[-1] > vals[0], True)
    check("run buffer collected PSI", len(app.run_psi), 120)
finally:
    shutil.rmtree(tmp, ignore_errors=True)

# --- folder is beside the code ---
import dyno_gui as g
check("default folder sits next to dyno_gui.py",
      os.path.dirname(g.DEFAULT_DATA_DIR), os.path.dirname(os.path.abspath(g.__file__)))
check("named dyno_runs", os.path.basename(g.DEFAULT_DATA_DIR), "dyno_runs")
root.destroy()
print(); print("FAILURES:", fails if fails else "none")

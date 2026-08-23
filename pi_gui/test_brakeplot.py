"""Stepper position must be plotted on its own scale, logged, and hideable."""
import os, csv, shutil, tempfile, tkinter as tk
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
def frame(ms,rpm,pos,psi=400):
    app._parse_line(f"DATA,{ms},{rpm},120,900,900,1500,{pos},{rpm},SWEEP,1,2,3,48,{psi},0,9.9,0")

check("brake has its own axis", hasattr(app,"ax_brk"), True)
check("axis is outboard of the PSI axis",
      app.ax_brk.spines["right"].get_position()[1] >
      app.ax_psi.spines["right"].get_position()[1], True)
check("brake is in the legend",
      "Brake pos" in [t.get_text() for t in app.ax_rpm.get_legend().get_texts()], True)

app.recording=False
for i in range(60): frame(1000+i*50, 2000, 30+i*3)
check("monitor collects position", len(app.mon_brake), 60)
app._redraw_live_plot()
check("brake line has points", len(app.line_brake.get_xdata()), 60)
check("brake axis scaled to steps", app.ax_brk.get_ylim()[1] >= 207, True)
check("brake scale is separate from RPM",
      app.ax_brk.get_ylim() != app.ax_rpm.get_ylim(), True)
check("a 200-step trace does not distort the RPM axis",
      app.ax_rpm.get_ylim()[1] >= 2000, True)

# hiding a trace clears it and reclaims its axis
app.trace_vars["brake"].set(False); app._redraw_live_plot()
check("hidden brake trace is cleared", len(app.line_brake.get_xdata()), 0)
check("its axis is hidden too", app.ax_brk.get_visible(), False)
app.trace_vars["brake"].set(True); app._redraw_live_plot()
check("and comes back", len(app.line_brake.get_xdata()), 60)
check("axis restored", app.ax_brk.get_visible(), True)

# hiding one trace must not disturb the others
app.trace_vars["hp"].set(False); app._redraw_live_plot()
check("hiding HP leaves RPM alone", len(app.line_rpm.get_xdata()), 60)
check("HP cleared", len(app.line_hp.get_xdata()), 0)
app.trace_vars["hp"].set(True)

# logged to the run CSV
app.recording=True
for i in range(120): frame(50000+i*50, 2500+i*10, 20+i)
app.recording=False
tmp=tempfile.mkdtemp(prefix="brk_")
try:
    path=os.path.join(tmp,"r.csv"); app._write_run_csv(path)
    rows=list(csv.reader(open(path))); hdr=rows[0]
    check("run CSV carries Brake_Pos", "Brake_Pos" in hdr, True)
    i_b=hdr.index("Brake_Pos")
    check("first position logged", float(rows[1][i_b]), 20.0)
    check("position tracks over the run", float(rows[-1][i_b]) > float(rows[1][i_b]), True)
    check("run buffer collected position", len(app.run_brake), 120)
finally:
    shutil.rmtree(tmp, ignore_errors=True)

check("visibility is carried in the profile",
      "traces_shown" in app._profile_snapshot(), True)
check("all five traces are listed",
      sorted(app._profile_snapshot()["traces_shown"]),
      ["brake","hp","psi","rpm","torque"])
root.destroy()
print(); print("FAILURES:", fails if fails else "none")

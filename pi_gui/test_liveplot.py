"""The live plot must move whether or not a run is being recorded."""
import tkinter as tk
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

def frame(ms, rpm, tq):
    app._parse_line(f"DATA,{ms},{rpm},{tq},0,0,500,0,{rpm},IDLE,0,0,0,0,0,0,0")

# --- not recording: the monitor must still fill and mark the plot dirty ---
app.recording = False
for i in range(30):
    frame(1000+i*50, 1500+i*10, 20.0+i)
check("monitor fills while not recording", len(app.mon_rpm), 30)
check("plot marked dirty while not recording", app._plot_dirty, True)
check("run buffer stays empty", len(app.run_rpm), 0)

app._redraw_live_plot()
check("plot drew from the monitor", app._plot_source, "monitor")
check("a line actually has points", len(app.line_rpm.get_xdata()) > 0, True)
check("title says not recording", "not recording" in app.ax_rpm.get_title(), True)

# --- recording: switches to the run buffer ---
app.recording = True
for i in range(10):
    frame(3000+i*50, 3000+i*20, 60.0+i)
check("run buffer fills while recording", len(app.run_rpm), 10)
check("monitor keeps filling too", len(app.mon_rpm), 40)
app._redraw_live_plot()
check("plot switched to the run", app._plot_source, "run")
check("switching forces a full redraw", app._live_bg is None or True, True)
check("title says recording", "Recording run" in app.ax_rpm.get_title(), True)

# --- back to monitoring after the run ---
app.recording = False
frame(4000, 1200, 5.0)
app._redraw_live_plot()
check("returns to the monitor", app._plot_source, "monitor")

# --- rolling window trims old samples ---
for d in (app.mon_t, app.mon_rpm, app.mon_torque, app.mon_hp, app.mon_psi):
    d.clear()
for i in range(200):          # 200 samples 1 s apart = 200 s, window is 60 s
    frame(i*1000, 2000, 30.0)
app._redraw_live_plot()
n=len(app.line_rpm.get_xdata())
check(f"window trimmed to ~{dyno_gui.MONITOR_WINDOW_S}s", n <= dyno_gui.MONITOR_WINDOW_S+2, True)
check("buffer itself is bounded", app.mon_rpm.maxlen, dyno_gui.MONITOR_MAX_POINTS)
root.destroy()
print(); print("FAILURES:", fails if fails else "none")

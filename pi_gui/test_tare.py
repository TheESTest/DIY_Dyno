"""Tare behaviour: averaging, rail detection, and refusing when disconnected."""
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
    if not ok: fails.append(name)

info = {"n": 0}
messagebox.showinfo = lambda *a, **k: info.__setitem__("n", info["n"] + 1)
messagebox.showwarning = lambda *a, **k: None
messagebox.showerror = lambda *a, **k: None

root = tk.Tk(); app = dyno_gui.DynoApp(root)
sent = []; app._send = lambda c: sent.append(c)

def frame(mv):
    app._parse_line(f"DATA,1,1000.0,0.0,{mv},{mv},500.0,0,1000.0,IDLE,0,0,0,0,0,0,0")

# Disconnected: refuse, don't silently do nothing.
app._on_tare()
check("refuses while disconnected", info["n"], 1)
check("nothing sent while disconnected", sent, [])

# Pretend we're connected.
class FakeSer:
    is_open = True
app.ser = FakeSer(); app.source_var.set("hardware")

# --- a healthy, noisy signal ---
sent.clear()
app._on_tare()
check("TARE sent to the controller", "TARE" in sent, True)
for mv in (2000.0, 2010.0, 1990.0, 2004.0, 1996.0):
    frame(mv)
app._finish_tare()
check("zero is the mean, not one sample", round(app.load_zero_mv, 2), 2000.0)
check("feedback shows the zero and the noise",
      "2000.0" in app.tare_label.cget("text") and "20.0" in app.tare_label.cget("text"), True)

# Net reading now responds; raw does not.
frame(2050.0); app._update_gui()
check("net reads the delta", app.status_labels["load_net"].cget("text"), "50.0")
check("raw is unchanged by taring", app.status_labels["load_raw"].cget("text"), "2050.0")

# --- the rail case: this is what a clamped 0-10 V channel looks like ---
sent.clear()
app._on_tare()
for mv in (0.0, 0.0, 0.0, 0.0):
    frame(mv)
app._finish_tare()
check("rail is called out, not treated as a zero",
      "AT RAIL" in app.tare_label.cget("text"), True)
check("operator is told why", "bottom of its 0-10 V range" in app.last_error, True)
frame(0.0); app._update_gui()
check("net shows AT RAIL not a number",
      app.status_labels["load_net"].cget("text"), "AT RAIL")

# --- no data arriving ---
app.last_error = ""
app._on_tare(); app._finish_tare()
check("empty window reports failure", "No data", app.tare_label.cget("text"))

root.destroy()
print()
print("FAILURES:", fails if fails else "none")

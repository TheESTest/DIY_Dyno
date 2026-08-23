"""End-of-run settings: mode mapping, ordering, and the RAMP_DOWN state."""
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
sent=[]; app._send=lambda c: sent.append(c)

for idx,label in enumerate(dyno_gui.RAMPDOWN_MODES):
    sent.clear()
    app.cfg_vars["rampdown_mode"].set(label)
    app.cfg_vars["rampdown_rate"].set("450")
    app.cfg_vars["rampdown_brake"].set("25")
    app.cfg_vars["cutoff_rpm"].set("1100")
    app.cfg_vars["throttle_off"].set("60")
    app._send_rampdown_cfg()
    check(f"'{label}' -> mode {idx}", f"RAMPDOWN_MODE,{idx}" in sent, True)
check("ramp rate sent", "RAMPDOWN_RATE,450" in sent, True)
check("brake rate sent", "RAMPDOWN_BRAKE,25" in sent, True)
check("cutoff sent", "CUTOFF_RPM,1100" in sent, True)
check("lift threshold sent", "THROTTLE_OFF,60" in sent, True)

# 'Send all' must include the end-of-run settings.
sent.clear()
class FakeSer: is_open=True
app.ser=FakeSer()
app._send_all_config()
check("send-all includes ramp-down", any(c.startswith("RAMPDOWN_MODE") for c in sent), True)
check("send-all includes cutoff", any(c.startswith("CUTOFF_RPM") for c in sent), True)

# Manual trigger refuses when disconnected, sends when connected.
app.ser=None; sent.clear(); app._force_rampdown()
check("test button refuses while disconnected", sent, [])
app.ser=FakeSer(); sent.clear(); app._force_rampdown()
check("test button sends RAMPDOWN", sent, ["RAMPDOWN"])

# The new controller state must be shown, not silently ignored.
app._parse_line("DATA,1,1800.0,0,0,0,500,40,1800.0,RAMP_DOWN,0,0,0,16,0,0,0")
app._update_gui()
check("RAMP_DOWN reaches the status line",
      "Bringing RPM down" in app.run_status_var.get(), True)
check("START stays disabled during ramp-down",
      str(app.start_btn.cget("state")), "disabled")

# Profile carries them.
snap=app._profile_snapshot()
for k in ("rampdown_mode","rampdown_rate","rampdown_brake","cutoff_rpm","throttle_off"):
    check(f"profile carries {k}", k in snap, True)
root.destroy()
print(); print("FAILURES:", fails if fails else "none")

# --- stop rate ---
sent.clear()
app.cfg_vars["stop_rate"].set("180")
app._send_rampdown_cfg()
check("stop rate sent", "STOP_RATE,180" in sent, True)
check("profile carries stop_rate", "stop_rate" in app._profile_snapshot(), True)
print(); print("FAILURES:", fails if fails else "none")

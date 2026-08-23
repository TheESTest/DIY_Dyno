"""Both versions are tracked: reported, displayed and recorded with the run."""
import os, json, tempfile, tkinter as tk
os.environ.setdefault("MPLBACKEND", "Agg")
import dyno_gui
import tempfile as _tf, os as _os
dyno_gui.SESSION_FILE = _os.path.join(_tf.mkdtemp(), 'session.json')
from dyno_gui import messagebox
messagebox.showinfo = messagebox.showwarning = messagebox.showerror = lambda *a, **k: None

fails = []
def check(what, got, want):
    ok = got == want
    if not ok: fails.append(f"{what}: got {got!r} want {want!r}")
    print(("  ok   " if ok else "  FAIL ") + what)

root = tk.Tk(); app = dyno_gui.DynoApp(root)
sent = []; app._send = lambda c: sent.append(c)

check("UI version is set", bool(dyno_gui.UI_VERSION), True)
check("title carries it", dyno_gui.UI_VERSION in root.title(), True)
check("firmware unknown before the board answers", app.fw_version, "unknown")

# unknown firmware is flagged, not left blank
app._update_gui()
check("unknown shown", app.status_labels["fw_version"].cget("text"), "unknown")
check("unknown flagged amber",
      str(app.status_labels["fw_version"].cget("foreground")), "#B9770E")

# the board answers
app._parse_line("CFG,FW_VERSION,1.1.0,Aug 23 2026 07:45:12")
check("version captured", app.fw_version, "1.1.0")
check("build captured", app.fw_build, "Aug 23 2026 07:45:12")
app._update_gui()
check("version shown", app.status_labels["fw_version"].cget("text"), "1.1.0")
check("known version not flagged",
      str(app.status_labels["fw_version"].cget("foreground")), "black")

# A malformed line is ignored rather than allowed to wipe a known-good
# version - garbage on the wire should not make the record worse.
app._parse_line("CFG,FW_VERSION")
check("truncated line ignored, version kept", app.fw_version, "1.1.0")
app._parse_line("CFG,FW_VERSION,2.0.0")
check("version without build", (app.fw_version, app.fw_build), ("2.0.0", ""))

# recorded beside the run
app._parse_line("CFG,FW_VERSION,1.1.0,Aug 23 2026 07:45:12")
app.recording = True
for i in range(5):
    app._parse_line(f"DATA,{1000+i*50},2000.0,10.0,700,700,500,10,2000,SWEEP,"
                    f"1,2,3,5.0,50.0,0,0.0,0,0")
app.recording = False
d = tempfile.mkdtemp()
cond = os.path.join(d, "run_conditions.json")
app._write_conditions(cond, os.path.join(d, "run.csv"))
with open(cond) as f: c = json.load(f)
check("run records UI version", c["_ui_version"], dyno_gui.UI_VERSION)
check("run records firmware version", c["_fw_version"], "1.1.0")
check("run records firmware build", c["_fw_build"], "Aug 23 2026 07:45:12")

app.on_close()
print("FAILURES: " + ("none" if not fails else "\n  " + "\n  ".join(fails)))
raise SystemExit(1 if fails else 0)

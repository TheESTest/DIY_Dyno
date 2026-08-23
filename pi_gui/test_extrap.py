"""Extrapolation option: settings reach the board, the estimated counter is
parsed and shown, and both counters land in the saved run."""
import os, csv, tempfile, tkinter as tk
os.environ.setdefault("MPLBACKEND", "Agg")
import dyno_gui
# Keep the suite off the real session file: DynoApp saves settings on close,
# so without this every test would leave its values behind for the next one
# and for the operator's next real start.
import tempfile as _tf, os as _os
dyno_gui.SESSION_FILE = _os.path.join(_tf.mkdtemp(), 'session.json')

from dyno_gui import messagebox
messagebox.showinfo=messagebox.showwarning=messagebox.showerror=lambda *a,**k: None

fails = []
def check(what, got, want):
    ok = got == want
    if not ok:
        fails.append(f"{what}: got {got!r} want {want!r}")
    print(("  ok   " if ok else "  FAIL ") + what)

root = tk.Tk()
app = dyno_gui.DynoApp(root)
sent = []
app._send = lambda c: sent.append(c)

# --- defaults -------------------------------------------------------------
check("extrapolation off by default", app.cfg_vars["rpm_extrap"].get(), False)
check("fit points default", app.cfg_vars["rpm_extrap_n"].get(), "4")
check("max run default", app.cfg_vars["rpm_extrap_max"].get(), "5")

# --- the command ----------------------------------------------------------
app.cfg_vars["rpm_extrap"].set(True)
app.cfg_vars["rpm_extrap_n"].set("6")
app.cfg_vars["rpm_extrap_max"].set("3")
sent.clear(); app._send_rpm_cfg()
check("band still sent", "RPM_BAND,800,6000" in sent, True)
check("extrapolation sent", "RPM_EXTRAP,1,6,3" in sent, True)
app.cfg_vars["rpm_extrap"].set(False)
sent.clear(); app._send_rpm_cfg()
check("off sends 0", "RPM_EXTRAP,0,6,3" in sent, True)

# --- field 19 -------------------------------------------------------------
app._parse_line("DATA,1000,2500.0,10.0,700,700,500,100,2500,SWEEP,"
                 "1,2,3,10.0,50.0,0,0.0,7,4")
check("estimated parsed", app.live["estimated"], 4)
check("glitches parsed", app.live["glitches"], 7)

# Old firmware without the field must still parse.
app._parse_line("DATA,1100,2500.0,10.0,700,700,500,100,2500,SWEEP,"
                 "1,2,3,10.0,50.0,0,0.0,7")
check("older frame tolerated", app.live["estimated"], 0)

# --- the status readout ---------------------------------------------------
app._parse_line("DATA,1200,2500.0,10.0,700,700,500,100,2500,SWEEP,"
                 "1,2,3,10.0,50.0,0,0.0,9,6")
app._update_gui()
check("estimated shown", app.status_labels["estimated"].cget("text"), "6")
check("estimated flagged amber",
      str(app.status_labels["estimated"].cget("foreground")), "#B9770E")

# --- the saved run --------------------------------------------------------
app.recording = True
app.run_t.clear(); app.run_rpm.clear(); app.run_torque.clear()
app.run_hp.clear(); app.run_psi.clear(); app.run_brake.clear()
app.log_rows.clear()
for i in range(30):
    app._parse_line(
        f"DATA,{2000+i*50},{2000+i*10}.0,12.0,700,700,500,{100+i},2400,SWEEP,"
        f"1,2,3,10.0,50.0,0,0.0,{i},{i//3}")
app.recording = False
path = os.path.join(tempfile.mkdtemp(), "run.csv")
n = app._write_run_csv(path)
with open(path) as f:
    rows = list(csv.reader(f))
head, body = rows[0], rows[1:]
check("wrote every sample", n, 30)
check("glitch column present", "Tach_Glitches" in head, True)
check("estimated column present", "RPM_Estimated" in head, True)
gi, ei = head.index("Tach_Glitches"), head.index("RPM_Estimated")
check("glitch count tracked", [r[gi] for r in body][:4], ["0", "1", "2", "3"])
check("estimated count tracked", [r[ei] for r in body][:7],
      ["0", "0", "0", "1", "1", "1", "2"])
check("row width matches header", all(len(r) == len(head) for r in body), True)

# --- carried by profiles and the session ---------------------------------
app.cfg_vars["rpm_extrap"].set(True)
app.cfg_vars["rpm_extrap_n"].set("8")
snap = app._profile_snapshot()
check("profile records the switch", snap["rpm_extrap"], True)
check("profile records fit points", snap["rpm_extrap_n"], "8")
app.cfg_vars["rpm_extrap"].set(False)
app.cfg_vars["rpm_extrap_n"].set("2")
app._apply_profile(snap)
check("switch restored", app.cfg_vars["rpm_extrap"].get(), True)
check("fit points restored", app.cfg_vars["rpm_extrap_n"].get(), "8")

app.on_close()
print("FAILURES: " + ("none" if not fails else "\n  " + "\n  ".join(fails)))
raise SystemExit(1 if fails else 0)

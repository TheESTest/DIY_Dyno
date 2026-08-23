"""Three ways of measuring the same tach, all recorded, one driving the loop."""
import os, csv, tempfile, tkinter as tk
os.environ.setdefault("MPLBACKEND", "Agg")
import dyno_gui
import tempfile as _tf, os as _os
dyno_gui.SESSION_FILE = _os.path.join(_tf.mkdtemp(), "session.json")
from dyno_gui import messagebox
messagebox.showinfo = messagebox.showwarning = messagebox.showerror = lambda *a, **k: None

fails = []
def check(what, got, want):
    ok = got == want
    if not ok:
        fails.append(f"{what}: got {got!r} want {want!r}")
    print(("  ok   " if ok else "  FAIL ") + what)

root = tk.Tk(); app = dyno_gui.DynoApp(root)
sent = []; app._send = lambda c: sent.append(c)

# --- the selector -------------------------------------------------------
check("three sources offered", len(dyno_gui.RPM_SOURCES), 3)
check("counting is the default",
      app.cfg_vars["rpm_source"].get(), dyno_gui.RPM_SOURCES[1])
sent.clear(); app._send_rpm_cfg()
check("source sent as its number", "RPM_SOURCE,1" in sent, True)

# --- the counting window ------------------------------------------------
check("window defaults to 100 ms", app.cfg_vars["rpm_count_ms"].get(), "100")
check("window sent", "RPM_COUNT_MS,100" in sent, True)
app.cfg_vars["rpm_count_ms"].set("250")
sent.clear(); app._send_rpm_cfg()
check("a changed window is sent", "RPM_COUNT_MS,250" in sent, True)

# it is shown as a rate, because that is how it was asked for
app.cfg_vars["teeth"].set("3")
app.param_vars["hold_rpm"].set("1500")
app.cfg_vars["rpm_count_ms"].set("100")
app._update_count_window()
txt = app.count_window_label.cget("text")
check("100 ms shown as 10 Hz", "10.0 Hz" in txt, True)
check("pulses per window shown", "pulses per window" in txt, True)
app.cfg_vars["rpm_count_ms"].set("500")
app._update_count_window()
check("500 ms shown as 2 Hz", "2.0 Hz" in app.count_window_label.cget("text"), True)

# a window too short to hold teeth is called out
app.cfg_vars["rpm_count_ms"].set("20")
app.param_vars["hold_rpm"].set("800")
app._update_count_window()
check("too few pulses flagged",
      "too few" in app.count_window_label.cget("text"), True)
check("and flagged red",
      str(app.count_window_label.cget("foreground")), "#B03A2E")
app.cfg_vars["rpm_count_ms"].set("100")
app.param_vars["hold_rpm"].set("1500")
app._update_count_window()
check("a sane window is not flagged",
      "too few" in app.count_window_label.cget("text"), False)

# and the limits match the firmware
for bad in ("19", "5001", "abc"):
    app.cfg_vars["rpm_count_ms"].set(bad)
    check(f"window {bad} rejected",
          any("Counting window" in x for x in app._validate_cfg()), True)
app.cfg_vars["rpm_count_ms"].set("100")
check("valid window passes", app._validate_cfg(), [])
for i, label in enumerate(dyno_gui.RPM_SOURCES):
    app.cfg_vars["rpm_source"].set(label)
    sent.clear(); app._send_rpm_cfg()
    check(f"source {i} sent", f"RPM_SOURCE,{i}" in sent, True)
app.cfg_vars["rpm_source"].set(dyno_gui.RPM_SOURCES[0])

# --- the two extra measurements are parsed ------------------------------
app._parse_line("DATA,1000,1500.0,10.0,700,700,500,10,1500,IDLE,"
                "1,2,3,5.0,50.0,0,0.0,7,0,0,0,1480.5,1495.2")
check("counted parsed", app.live["rpm_counted"], 1480.5)
check("one-rev parsed", app.live["rpm_rev"], 1495.2)
app._update_gui()
check("counted shown", app.status_labels["rpm_counted"].cget("text"), "1480")
check("one-rev shown", app.status_labels["rpm_rev"].cget("text"), "1495")

# older firmware without the fields must still work
app._parse_line("DATA,1050,1500.0,10.0,700,700,500,10,1500,IDLE,"
                "1,2,3,5.0,50.0,0,0.0,7,0,0,0")
check("older frame tolerated", app.live["rpm_counted"], 0.0)

# --- all three reach the saved run --------------------------------------
app.recording = True
app.run_t.clear(); app.run_rpm.clear(); app.run_torque.clear()
app.run_hp.clear(); app.run_psi.clear(); app.run_brake.clear()
app.log_rows.clear()
for i in range(20):
    app._parse_line(
        f"DATA,{2000+i*50},{1500+i}.0,12.0,700,700,500,{10+i},1500,SWEEP,"
        f"1,2,3,5.0,50.0,0,0.0,0,0,0,0,{1490+i}.0,{1495+i}.0")
app.recording = False
d = tempfile.mkdtemp()
path = os.path.join(d, "run.csv")
n = app._write_run_csv(path)
with open(path) as f:
    rows = list(csv.reader(f))
hdr, body = rows[0], rows[1:]
check("all samples written", n, 20)
for col in ("RPM", "RPM_Counted", "RPM_Rev"):
    check(f"{col} column present", col in hdr, True)
gi, ci, ri = hdr.index("RPM"), hdr.index("RPM_Counted"), hdr.index("RPM_Rev")
check("driving method recorded", body[0][gi], "1500.0")
check("counted recorded", body[0][ci], "1490.0")
check("one-rev recorded", body[0][ri], "1495.0")
check("all three differ, so they can be compared",
      len({body[5][gi], body[5][ci], body[5][ri]}), 3)
check("row width matches header",
      all(len(r) == len(hdr) for r in body), True)

# --- carried by profiles ------------------------------------------------
app.cfg_vars["rpm_source"].set(dyno_gui.RPM_SOURCES[2])
snap = app._profile_snapshot()
check("profile keeps the source", snap["rpm_source"], dyno_gui.RPM_SOURCES[2])
app.cfg_vars["rpm_source"].set(dyno_gui.RPM_SOURCES[0])
app._apply_profile(snap)
check("source restored", app.cfg_vars["rpm_source"].get(), dyno_gui.RPM_SOURCES[2])

app.on_close()
print("FAILURES: " + ("none" if not fails else "\n  " + "\n  ".join(fails)))
raise SystemExit(1 if fails else 0)

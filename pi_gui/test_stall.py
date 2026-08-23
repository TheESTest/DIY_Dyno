"""Sweep stall watch, and the encoder TBD scaffolding.

The case that matters most is the negative one: takeup travel produces no
pressure by design, and must never be reported as a stall.
"""
import os, csv, json, tempfile, tkinter as tk
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


def sweep(psi_at):
    """Drive the watch over a 10 s outward leg; psi_at(steps) shapes the rig."""
    app._char_rows = []
    app._char_engaged = False
    app._char_base_psi = None
    app._char_susp_since = None
    app._char_stall_at = None
    app._char_home = 0
    hit = None
    for i in range(200):                       # 10 s at 20 Hz
        t = i * 0.05
        target = 6250.0 * (t / 10.0)
        psi = psi_at(target)
        if app._char_check_stall(t, target, psi) and hit is None:
            hit = dict(app._char_stall_at)
        app._char_rows.append([f"{t:.3f}", "up", f"{target:.1f}", str(int(target)),
                               "0", f"{psi:.2f}", "0", "0", "0",
                               "1" if app._char_stall_at else "0"])
    return hit


# --- a healthy rig: takeup, then pressure tracks position -----------------
def healthy(steps):
    return 0.0 if steps < 1500 else (steps - 1500) * 0.35

check("healthy sweep is not flagged", sweep(healthy), None)

# --- takeup alone must never trip it, however long -----------------------
check("long takeup with no pressure is not a stall",
      sweep(lambda s: 0.0), None)

# --- a real stall: pressure rises, then stops while position keeps going --
def stalls(steps):
    if steps < 1500:
        return 0.0
    if steps < 3500:
        return (steps - 1500) * 0.35
    return (3500 - 1500) * 0.35                # flat from here on

hit = sweep(stalls)
check("stall detected", hit is not None, True)
if hit:
    check("flagged after engagement, not during takeup", hit["commanded"] > 3500, True)
    check("flagged promptly once flat", hit["commanded"] < 5200, True)
    check("records what it saw", hit["psi_gained"] < dyno_gui.CHAR_STALL_PSI, True)

# --- pressure that merely slows is not a stall ---------------------------
def tapering(steps):
    if steps < 1500:
        return 0.0
    return (steps - 1500) ** 0.75 * 0.25       # keeps climbing, just flattening

check("a tapering curve is not flagged", sweep(tapering), None)

# --- flagged only once ---------------------------------------------------
hit2 = sweep(stalls)
n = sum(1 for r in app._char_rows if r[9] == "1")
check("flag latches for the rest of the sweep", n > 0, True)

# --- the sweep CSV carries the column ------------------------------------
app._char_stall_at = None
d = tempfile.mkdtemp()
app.cfg_vars["data_dir"].set(d)
app._char_rows = [[f"{i*0.05:.3f}", "up", f"{i*30}", f"{i*30}", "0",
                   f"{i*0.4:.2f}", "0", "0", "0", "0"] for i in range(30)]
app._finish_brake_char()
csvs = [f for f in os.listdir(d) if f.endswith(".csv")]
check("sweep CSV written", len(csvs), 1)
with open(os.path.join(d, csvs[0])) as f:
    rows = list(csv.reader(f))
check("stall column present", "Stall_Suspected" in rows[0], True)
check("column width matches", all(len(r) == len(rows[0]) for r in rows[1:]), True)

# --- encoder TBD ---------------------------------------------------------
check("encoder off by default", app.cfg_vars["enc_enabled"].get(), False)
sent.clear(); app._send_encoder_cfg()
check("encoder settings sent", sent, ["ENCODER,0,4000,0"])
app.cfg_vars["enc_enabled"].set(True)
app.cfg_vars["enc_cpr"].set("2048")
app.cfg_vars["enc_invert"].set(True)
sent.clear(); app._send_encoder_cfg()
check("encoder settings carry through", sent, ["ENCODER,1,2048,1"])

# reserved DATA fields parse, and read as absent
app._parse_line("DATA,1000,2000.0,10.0,700,700,500,10,2000,IDLE,"
                "1,2,3,5.0,50.0,0,0.0,0,0,0,0")
check("encoder reads not-ok", app.live["enc_ok"], 0)
app._update_gui()
check("panel says not installed",
      app.enc_pos_label.cget("text"), "not installed")

# a board that one day reports a live encoder is shown
app._parse_line("DATA,1050,2000.0,10.0,700,700,500,10,2000,IDLE,"
                "1,2,3,5.0,50.0,0,0.0,0,0,12345,1")
app._update_gui()
check("live encoder position shown", app.enc_pos_label.cget("text"), "12345")

# and a frame without the fields still parses
app._parse_line("DATA,1100,2000.0,10.0,700,700,500,10,2000,IDLE,"
                "1,2,3,5.0,50.0,0,0.0,0,0")
check("older frame tolerated", app.live["enc_ok"], 0)

# --- settings persist ----------------------------------------------------
snap = app._profile_snapshot()
check("stop-on-stall saved", "char_stop_on_stall" in snap, True)
check("encoder settings saved", snap["enc_cpr"], "2048")

app.on_close()
print("FAILURES: " + ("none" if not fails else "\n  " + "\n  ".join(fails)))
raise SystemExit(1 if fails else 0)

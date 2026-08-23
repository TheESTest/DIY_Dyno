"""The characterisation sweep: guards, profile shape, files, and the plot.

Drives it with a fake brake that has a dead zone before the pads bite and a
little hysteresis on the way back, so the output can be checked for the things
the test exists to reveal.
"""
import os, csv, glob, json, shutil, tempfile, tkinter as tk
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

class FakeSer: is_open=True
sent=[]
TAKEUP=2200.0          # nothing happens below this

# The controller now runs a whole leg from one command, so the fake has to model
# a move in flight rather than a position per tick: BRAKE_SWEEP starts a timed
# traverse, and the position is interpolated as time passes.
import time as _time
move={"from":0.0,"to":0.0,"t0":0.0,"secs":0.0}

def _apply(pos):
    over=max(0.0, pos-TAKEUP)
    psi = over*0.30
    if app._char_rows and app._char_rows[-1][1]=="down":
        psi *= 1.18              # retracting holds pressure a little longer
    with app._lock:
        app.live["brake_pos"]=int(pos)
        app.live["press_psi"]=psi
        app.live["press_mv"]=500+psi*2

def advance():
    """Where the traverse has got to by now."""
    if move["secs"] <= 0:
        return
    f = (_time.monotonic()-move["t0"])/move["secs"]
    f = 0.0 if f < 0 else (1.0 if f > 1 else f)
    _apply(move["from"] + (move["to"]-move["from"])*f)

def fake_send(cmd):
    sent.append(cmd)
    if cmd.startswith("BRAKE_SWEEP,"):
        _, tgt, ms = cmd.split(",")
        with app._lock:
            cur=float(app.live["brake_pos"])
        move.update({"from":cur,"to":float(tgt),
                     "t0":_time.monotonic(),"secs":float(ms)/1000.0})
    elif cmd.startswith("BRAKE,"):
        move["secs"]=0.0                       # a direct move lands at once
        _apply(float(cmd.split(",")[1]))
app._send=fake_send

# ---- guards ----
app.ser=None; app._start_brake_char()
check("refuses while disconnected", app._char_active, False)
app.ser=FakeSer(); app.ready_flags["homed"]=False
app._start_brake_char()
check("refuses when not homed", app._char_active, False)
app.ready_flags["homed"]=True
with app._lock: app.live["rpm"]=1800.0
app._start_brake_char()
check("refuses with the engine turning", app._char_active, False)
with app._lock: app.live["rpm"]=0.0

# ---- profile shape ----
app.cfg_vars["brake_max"].set("6250")
app._char_home=0
for elapsed, want_phase in ((0.0,"up"),(5.0,"up"),(10.5,"hold"),
                            (12.0,"down"),(21.5,"done")):
    tgt, ph = app._char_targets(elapsed)
    check(f"t={elapsed}s is '{want_phase}'", ph, want_phase)
check("starts at home", round(app._char_targets(0.0)[0]), 0)
check("half way up is half travel", round(app._char_targets(5.0)[0]), 3125)
check("tops out at full travel", round(app._char_targets(9.99)[0]), 6244)
check("holds at full travel", round(app._char_targets(10.5)[0]), 6250)
# down leg runs 11.0-21.0 s, so 20.9 s is 99% of the way back
check("nearly home at the end of the down leg", round(app._char_targets(20.9)[0]), 63)
check("exactly home once done", round(app._char_targets(21.0)[0]), 0)

# ---- run it fast by shrinking the phases ----
tmp=tempfile.mkdtemp(prefix="char_")
try:
    app.cfg_vars["data_dir"].set(tmp)
    dyno_gui.CHAR_UP_S, dyno_gui.CHAR_HOLD_S, dyno_gui.CHAR_DOWN_S = 1.0, 0.2, 1.0
    messagebox.askyesno=lambda *a,**k: True
    app._start_brake_char()
    check("sweep started", app._char_active, True)
    import time as _t
    for _ in range(400):
        if not app._char_active: break
        advance()                 # the traverse progresses between ticks
        app._char_tick(); _t.sleep(0.006)
    check("sweep finished", app._char_active, False)

    csvs=glob.glob(os.path.join(tmp,"brake_char_*.csv"))
    pngs=glob.glob(os.path.join(tmp,"brake_char_*.png"))
    conds=glob.glob(os.path.join(tmp,"brake_char_*_conditions.json"))
    check("csv written", len(csvs), 1)
    check("plot written", len(pngs), 1)
    check("conditions written", len(conds), 1)
    check("plot is a real png", open(pngs[0],'rb').read(4), b'\x89PNG')
    check("plot has content", os.path.getsize(pngs[0]) > 20000, True)

    rows=list(csv.reader(open(csvs[0])))
    hdr=rows[0]
    for col in ("Time_s","Phase","Commanded_Steps","Brake_Pos","Brake_PSI"):
        check(f"csv has {col}", col in hdr, True)
    phases={r[hdr.index("Phase")] for r in rows[1:]}
    check("all three phases recorded", sorted(phases), ["down","hold","up"])
    pos=[float(r[hdr.index("Brake_Pos")]) for r in rows[1:]]
    psi=[float(r[hdr.index("Brake_PSI")]) for r in rows[1:]]
    check("position swept the full range", max(pos) >= 6000, True)
    check("returned toward home", pos[-1] < 500, True)
    check("pressure stayed flat below takeup",
          max(p for p,q in zip(psi,pos) if q < 2000) < 1.0, True)
    check("pressure rose above takeup", max(psi) > 500, True)
finally:
    shutil.rmtree(tmp, ignore_errors=True)
root.destroy()
print(); print("FAILURES:", fails if fails else "none")

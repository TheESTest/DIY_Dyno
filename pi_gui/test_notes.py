"""Per-test notes, saved beside the run they were written for."""
import os, json, time, tempfile, tkinter as tk
os.environ.setdefault("MPLBACKEND", "Agg")
import dyno_gui
import tempfile as _tf, os as _os
dyno_gui.SESSION_FILE = _os.path.join(_tf.mkdtemp(), "session.json")
from dyno_gui import messagebox

fails = []
def check(what, got, want):
    ok = got == want
    if not ok:
        fails.append(f"{what}: got {got!r} want {want!r}")
    print(("  ok   " if ok else "  FAIL ") + what)

answer = [True]
messagebox.showinfo = messagebox.showwarning = messagebox.showerror = lambda *a, **k: None
messagebox.askyesno = lambda *a, **k: answer[0]

root = tk.Tk(); app = dyno_gui.DynoApp(root)
app._send = lambda c: None

# --- empty to start -------------------------------------------------------
check("starts empty", app._get_notes(), "")
check("says where it will go",
      "next run" in app.notes_status.cget("text"), True)

# --- typing ---------------------------------------------------------------
app.notes_text.insert("1.0", "Cold start, choke half out.\nPump gap 0.9 mm.")
app._on_notes_edited()
check("notes read back", app._get_notes(),
      "Cold start, choke half out.\nPump gap 0.9 mm.")
check("counts characters", "44 chars" in app.notes_status.cget("text"), True)
check("trailing whitespace trimmed",
      app._get_notes().endswith("0.9 mm."), True)

# --- saved with the run ---------------------------------------------------
app.recording = True
app._run_started = time.time()
app._on_notes_edited()                      # written after the run began
for i in range(5):
    app._parse_line(f"DATA,{1000+i*50},2000.0,10.0,700,700,500,10,2000,SWEEP,"
                    f"1,2,3,5.0,50.0,0,0.0,0,0,0,0")
app.recording = False
d = tempfile.mkdtemp()
cond = os.path.join(d, "run_conditions.json")
app._write_conditions(cond, os.path.join(d, "run.csv"))
with open(cond) as f:
    c = json.load(f)
check("notes recorded with the run", c["_notes"],
      "Cold start, choke half out.\nPump gap 0.9 mm.")
check("edit time recorded", isinstance(c["_notes_edited"], str), True)

# --- a note written before the run is flagged, not silently attached ------
app.recording = True
app._notes_edited = time.time() - 300        # typed five minutes ago
app._run_started = time.time()               # run started just now
app._update_notes_status()
check("stale note called out",
      "before this run started" in app.notes_status.cget("text"), True)
check("and flagged amber",
      str(app.notes_status.cget("foreground")), "#B9770E")
# it is still saved - flagged, not withheld
app._write_conditions(cond, os.path.join(d, "run.csv"))
with open(cond) as f:
    check("stale note still saved", json.load(f)["_notes"] != "", True)

app._notes_edited = time.time()
app._update_notes_status()
check("fresh note not flagged",
      "before this run started" in app.notes_status.cget("text"), False)
app.recording = False

# --- clearing -------------------------------------------------------------
answer[0] = False
app._clear_notes()
check("clear can be declined", app._get_notes() != "", True)
answer[0] = True
app._clear_notes()
check("cleared on confirm", app._get_notes(), "")
check("edit time reset", app._notes_edited, None)
check("status back to empty",
      "next run" in app.notes_status.cget("text"), True)

# --- notes are run metadata, not a setting --------------------------------
app.notes_text.insert("1.0", "should not travel with the profile")
snap = app._profile_snapshot()
check("not in profiles", any("should not travel" in str(v) for v in snap.values()),
      False)
app._apply_profile(snap)
check("a profile load does not touch them",
      app._get_notes(), "should not travel with the profile")

# --- an empty note still saves cleanly ------------------------------------
app.notes_text.delete("1.0", tk.END)
app._notes_edited = None
app._write_conditions(cond, os.path.join(d, "run.csv"))
with open(cond) as f:
    c = json.load(f)
check("empty notes save as empty", c["_notes"], "")
check("no edit time when never typed", c["_notes_edited"], None)

app.on_close()
print("FAILURES: " + ("none" if not fails else "\n  " + "\n  ".join(fails)))
raise SystemExit(1 if fails else 0)

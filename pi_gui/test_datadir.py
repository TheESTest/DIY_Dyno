"""Everything the program generates lives under the folder it runs from.

A sibling folder looks identical in a file browser, so the setting has to say
which it is - that is how a whole afternoon of runs ended up outside the dyno
directory without anyone noticing.
"""
import os, tempfile, tkinter as tk
os.environ.setdefault("MPLBACKEND", "Agg")
import dyno_gui
# Capture where the program really puts these BEFORE redirecting them, or the
# check below just measures the test's own temp folders.
REAL_PATHS = {n: getattr(dyno_gui, n) for n in
              ("DEFAULT_DATA_DIR", "SESSION_FILE", "LOGO_FILE",
               "UPLOAD_TOKEN_FILE", "UPLOAD_PENDING_FILE")}
import tempfile as _tf, os as _os
dyno_gui.SESSION_FILE = _os.path.join(_tf.mkdtemp(), "session.json")
dyno_gui.UPLOAD_PENDING_FILE = _os.path.join(_tf.mkdtemp(), "pending.json")
from dyno_gui import messagebox
messagebox.showinfo = messagebox.showwarning = messagebox.showerror = lambda *a, **k: None

fails = []
def check(what, got, want):
    ok = got == want
    if not ok:
        fails.append(f"{what}: got {got!r} want {want!r}")
    print(("  ok   " if ok else "  FAIL ") + what)

HERE = dyno_gui.PROGRAM_DIR

# --- every fixed location is inside the program folder -------------------
for name, p in REAL_PATHS.items():
    check(f"{name} is inside the program folder",
          os.path.realpath(p).startswith(os.path.realpath(HERE) + os.sep), True)

# --- the inside/outside test itself --------------------------------------
inside = dyno_gui.DynoApp.data_dir_is_inside
check("the program folder counts as inside", inside(HERE), True)
check("a subfolder is inside", inside(os.path.join(HERE, "dyno_runs")), True)
check("a deeper subfolder is inside",
      inside(os.path.join(HERE, "dyno_runs", "2026")), True)
check("the parent is outside", inside(os.path.dirname(HERE)), False)
# the case that actually happened: a sibling with a very similar name
sib = os.path.join(os.path.dirname(HERE), "dyno_runs")
check("a sibling folder is outside", inside(sib), False)
check("a name that merely starts the same is outside",
      inside(HERE + "_runs"), False)
check("an unrelated path is outside", inside(tempfile.gettempdir()), False)

root = tk.Tk(); app = dyno_gui.DynoApp(root)
app._send = lambda c: None

# --- the field says which it is ------------------------------------------
check("defaults to inside", app.cfg_vars["data_dir"].get(), dyno_gui.DEFAULT_DATA_DIR)
app._update_data_dir_note()
check("and says so", "inside" in app.data_dir_note.cget("text"), True)
check("in green", str(app.data_dir_note.cget("foreground")), "#1E8449")

app.cfg_vars["data_dir"].set(sib)
check("a sibling is called out",
      "OUTSIDE" in app.data_dir_note.cget("text"), True)
check("in red", str(app.data_dir_note.cget("foreground")), "#B03A2E")
check("and it says how to fix it",
      "Reset" in app.data_dir_note.cget("text"), True)

app._reset_data_dir()
check("reset puts it back", app.cfg_vars["data_dir"].get(), dyno_gui.DEFAULT_DATA_DIR)
check("and the note follows", "inside" in app.data_dir_note.cget("text"), True)

# an empty setting falls back to the default, which is inside
app.cfg_vars["data_dir"].set("")
app._update_data_dir_note()
check("empty falls back to inside", "inside" in app.data_dir_note.cget("text"), True)
app._reset_data_dir()

# --- what the writers actually use ---------------------------------------
# All four data writers share one setting, so checking it covers runs,
# sweeps and pulse captures alike.
d = os.path.join(dyno_gui.DEFAULT_DATA_DIR, "")
app.cfg_vars["data_dir"].set(dyno_gui.DEFAULT_DATA_DIR)
folder = app.cfg_vars["data_dir"].get().strip() or dyno_gui.DEFAULT_DATA_DIR
check("runs land inside", inside(folder), True)

app.on_close()
print("FAILURES: " + ("none" if not fails else "\n  " + "\n  ".join(fails)))
raise SystemExit(1 if fails else 0)

"""Publishing runs to the repository.

No network is used: the HTTP layer is replaced. The checks that matter are the
ones about the token and about what reaches a public repository.
"""
import os, json, tempfile, tkinter as tk
os.environ.setdefault("MPLBACKEND", "Agg")
import dyno_gui
import tempfile as _tf, os as _os
dyno_gui.SESSION_FILE = _os.path.join(_tf.mkdtemp(), "session.json")
dyno_gui.UPLOAD_PENDING_FILE = _os.path.join(_tf.mkdtemp(), "pending.json")
from dyno_gui import messagebox

fails = []
def check(what, got, want):
    ok = got == want
    if not ok:
        fails.append(f"{what}: got {got!r} want {want!r}")
    print(("  ok   " if ok else "  FAIL ") + what)

shown = []
answer = [True]
messagebox.showinfo = lambda t, m="", **k: shown.append(("info", t, m))
messagebox.showwarning = lambda t, m="", **k: shown.append(("warn", t, m))
messagebox.showerror = lambda t, m="", **k: shown.append(("error", t, m))
messagebox.askyesno = lambda t, m="", **k: (shown.append(("ask", t, m)), answer[0])[1]

root = tk.Tk(); app = dyno_gui.DynoApp(root)
app._send = lambda c: None

TOKEN = "ghp_pretend_secret_value_0123456789"
put = []
def fake_put(token, path, blob, message):
    put.append((token, path, len(blob), message))
    return {"content": {"path": path}}
app._github_put = fake_put

# a folder holding one run, complete with its sidecars
d = tempfile.mkdtemp()
app.cfg_vars["data_dir"].set(d)
base = os.path.join(d, "dyno_run_20260823_120000")
for suffix, body in ((".csv", "Time_s,RPM\n0,1500\n"),
                     ("_conditions.json", json.dumps({"_notes": "cold start"})),
                     ("_filtered.csv", "RPM,HP\n1500,12\n")):
    with open(base + suffix, "w") as f:
        f.write(body)

# --- no token: says so, uploads nothing ---------------------------------
os.environ.pop(dyno_gui.UPLOAD_TOKEN_ENV, None)
dyno_gui.UPLOAD_TOKEN_FILE = os.path.join(d, "no_such_token.txt")
put.clear(); shown.clear()
done, problems = app._upload_files([base + ".csv"])
check("nothing uploaded without a token", put, [])
check("and it is reported", problems, ["no token"])
check("the dialog says where to put one",
      any(dyno_gui.UPLOAD_TOKEN_ENV in x[2] for x in shown), True)

# --- the token is found, and used -------------------------------------
os.environ[dyno_gui.UPLOAD_TOKEN_ENV] = TOKEN
check("token read from the environment", app._github_token(), TOKEN)
tokfile = os.path.join(d, "tok.txt")
with open(tokfile, "w") as f:
    f.write("  file_token_value  \n")
dyno_gui.UPLOAD_TOKEN_FILE = tokfile
check("environment wins over the file", app._github_token(), TOKEN)
os.environ.pop(dyno_gui.UPLOAD_TOKEN_ENV)
check("falls back to the file, trimmed", app._github_token(), "file_token_value")
os.environ[dyno_gui.UPLOAD_TOKEN_ENV] = TOKEN

# --- a run goes up as a complete set ------------------------------------
put.clear(); shown.clear()
files = app._files_for(base + ".csv")
check("all three sidecars found", len(files), 3)
done, problems = app._upload_files(files)
check("all uploaded", done, 3)
check("no problems", problems, [])
check("into the data folder",
      all(p.startswith(dyno_gui.UPLOAD_DIR + "/") for _t, p, _n, _m in put), True)
check("under their own names",
      sorted(p.split("/")[-1] for _t, p, _n, _m in put),
      ["dyno_run_20260823_120000.csv",
       "dyno_run_20260823_120000_conditions.json",
       "dyno_run_20260823_120000_filtered.csv"])

# --- the operator is told the repository is public ----------------------
put.clear(); shown.clear()
answer[0] = True
app._upload_latest()
ask = [x for x in shown if x[0] == "ask"]
check("it asks first", len(ask), 1)
check("and says the repository is public", "PUBLIC" in ask[0][2], True)
check("and warns that notes go too", "notes" in ask[0][2], True)
check("naming the repository", dyno_gui.UPDATE_REPO in ask[0][2], True)
check("uploaded after confirming", len(put), 3)

answer[0] = False
put.clear(); shown.clear()
app._upload_latest()
check("declining uploads nothing", put, [])

# --- the token must never leak ------------------------------------------
def boom(token, path, blob, message):
    raise OSError(f"connection refused while sending {TOKEN} to {path}")
app._github_put = boom
answer[0] = True
put.clear(); shown.clear()
done, problems = app._upload_files([base + ".csv"])
check("failure reported", done, 0)
check("token scrubbed from the message",
      any(TOKEN in p for p in problems), False)
check("and replaced with a placeholder",
      any("<token>" in p for p in problems), True)
app._report_upload(done, problems, 1)
check("token not in the status line", TOKEN in app.upload_status.cget("text"), False)
check("token not in any dialog", any(TOKEN in x[2] for x in shown), False)
events = " ".join(str(e) for e in app.events)
check("token not in the event log", TOKEN in events, False)
app._github_put = fake_put

# it must not reach anything that gets saved or published either
snap = json.dumps(app._profile_snapshot())
check("token not in a profile", TOKEN in snap, False)
cond = os.path.join(d, "c.json")
app._write_conditions(cond, base + ".csv")
check("token not in a conditions file", TOKEN in open(cond).read(), False)

# --- publishing every test is the default -------------------------------
check("auto-upload on by default", app.cfg_vars["auto_upload"].get(), True)
os.environ[dyno_gui.UPLOAD_TOKEN_ENV] = TOKEN
dyno_gui.UPLOAD_TOKEN_FILE = os.path.join(d, "tok.txt")
app._upload_pending = []
put.clear()
app._maybe_auto_upload(base + ".csv")
check("a finished run publishes itself", len(put), 3)
check("nothing left owed", app._upload_pending, [])

# a sweep is a test too
sweep = os.path.join(d, "brake_char_20260823_121000")
for suffix in (".csv", "_conditions.json", ".png"):
    with open(sweep + suffix, "w") as f:
        f.write("x")
put.clear()
app._maybe_auto_upload(sweep + ".csv")
check("a calibration sweep publishes too", len(put), 3)
check("including its plot",
      any(p.endswith(".png") for _t, p, _n, _m in put), True)

# turning it off stops it
app.cfg_vars["auto_upload"].set(False)
put.clear()
app._maybe_auto_upload(base + ".csv")
check("respects the switch when off", put, [])
app.cfg_vars["auto_upload"].set(True)

# --- a run that finishes while the link is down is not lost -------------
def offline(token, path, blob, message):
    raise OSError("network is unreachable")
app._github_put = offline
app._upload_pending = []
put.clear(); shown.clear()
app._maybe_auto_upload(base + ".csv")
check("nothing uploaded while offline", put, [])
check("but the files are owed", len(app._upload_pending), 3)
check("said so without a popup", [x for x in shown if x[0] == "error"], [])
check("and the status line says they are waiting",
      "waiting" in app.upload_status.cget("text"), True)
check("the queue is on disk", os.path.exists(dyno_gui.UPLOAD_PENDING_FILE), True)

# it survives a restart
app._upload_pending = []
app._load_pending()
check("queue reloaded after a restart", len(app._upload_pending), 3)

# and goes up with the next successful run
app._github_put = fake_put
put.clear()
app._maybe_auto_upload(sweep + ".csv")
check("the backlog goes with the next test", len(put), 6)
check("nothing left owed", app._upload_pending, [])
check("and the queue file is gone",
      os.path.exists(dyno_gui.UPLOAD_PENDING_FILE), False)

# --- no token: queued, never dropped ------------------------------------
os.environ.pop(dyno_gui.UPLOAD_TOKEN_ENV)
dyno_gui.UPLOAD_TOKEN_FILE = os.path.join(d, "absent.txt")
app._upload_pending = []
put.clear(); shown.clear()
app._maybe_auto_upload(base + ".csv")
check("no token means nothing sent", put, [])
check("but still owed", len(app._upload_pending), 3)
check("and no popup mid-run", [x for x in shown if x[0] == "error"], [])
os.environ[dyno_gui.UPLOAD_TOKEN_ENV] = TOKEN
put.clear()
done, problems = app._flush_pending()
check("they go once a token appears", done, 3)

# a file deleted since is dropped rather than retried for ever
app._upload_pending = [os.path.join(d, "gone_forever.csv")]
done, problems = app._flush_pending()
check("a vanished file is not retried", app._upload_pending, [])

# --- the Publish button clears the backlog too --------------------------
# The backlog is a file from an earlier run, not part of the latest one, so
# the press has to carry both.
app._upload_pending = [base + "_conditions.json"]
put.clear(); shown.clear()
answer[0] = True
app._upload_latest()
names = sorted(p.split("/")[-1] for _t, p, _n, _m in put)
check("the latest sweep went up",
      "brake_char_20260823_121000.csv" in names, True)
check("and the older file owed from before",
      "dyno_run_20260823_120000_conditions.json" in names, True)
check("nothing left owed", app._upload_pending, [])

# the same file queued twice is only sent once
app._upload_pending = []
app._queue_for_upload([base + ".csv", base + ".csv"])
check("the queue does not duplicate", len(app._upload_pending), 1)

# --- an oversized file is refused, not truncated ------------------------
big = os.path.join(d, "dyno_run_20260823_130000.csv")
with open(big, "wb") as f:
    f.write(b"x" * (dyno_gui.UPLOAD_MAX_BYTES + 10))
put.clear()
done, problems = app._upload_files([big])
check("oversized file not uploaded", put, [])
check("and the reason given", "too large" in problems[0], True)

# --- an empty folder is handled -----------------------------------------
app.cfg_vars["data_dir"].set(tempfile.mkdtemp())
shown.clear()
app._upload_latest()
check("nothing to upload is said plainly",
      any("Nothing to upload" in x[1] for x in shown), True)

os.environ.pop(dyno_gui.UPLOAD_TOKEN_ENV, None)
app.on_close()
print("FAILURES: " + ("none" if not fails else "\n  " + "\n  ".join(fails)))
raise SystemExit(1 if fails else 0)

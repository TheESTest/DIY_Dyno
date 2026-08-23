"""Updating from the repository.

Everything here runs against a fake repository - no network is used. The cases
that matter are the refusals: code that will not compile must never reach the
disk, and an update must not run while the rig is doing something.
"""
import os, shutil, tempfile, tkinter as tk
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

shown = []
answer = [True]
messagebox.showinfo = lambda t, m="", **k: shown.append(("info", t, m))
messagebox.showwarning = lambda t, m="", **k: shown.append(("warn", t, m))
messagebox.showerror = lambda t, m="", **k: shown.append(("error", t, m))
messagebox.askyesno = lambda t, m="", **k: answer[0]

root = tk.Tk(); app = dyno_gui.DynoApp(root)
app._send = lambda c: None

# ── a fake repository ─────────────────────────────────────────────────────
GOOD_GUI = 'UI_VERSION = "9.9.9"\nprint("hello")\n'
BAD_GUI = 'UI_VERSION = "9.9.9"\ndef broken(:\n'
FW_SRC = '#define FW_VERSION "9.9.9"\n'
repo = {
    "pi_gui/dyno_gui.py": GOOD_GUI.encode(),
    "pi_gui/dyno_dsp.py": b"# dsp\n",
    "pi_gui/centurial_logo.png": b"\x89PNG fake",
    "esp32_firmware/src/main.cpp": FW_SRC.encode(),
    "esp32_firmware/build/firmware.bin": b"\x00" * 2048,
}
def fake_fetch(path):
    if path not in repo:
        raise OSError(f"{path}: HTTP 404")
    return repo[path]
app._remote_bytes = staticmethod(fake_fetch)
app._remote_bytes = fake_fetch

# ── version parsing, without importing anything ──────────────────────────
check("reads the interface version",
      dyno_gui.DynoApp._version_in(GOOD_GUI, "UI_VERSION"), "9.9.9")
check("reads the firmware version",
      dyno_gui.DynoApp._version_in(FW_SRC, "#define FW_VERSION"), "9.9.9")
check("missing version is not invented",
      dyno_gui.DynoApp._version_in("nothing here", "UI_VERSION"), None)
check("both versions read together", app._remote_versions(), ("9.9.9", "9.9.9"))

# ── it refuses while the rig is busy ─────────────────────────────────────
here = os.path.dirname(os.path.abspath(dyno_gui.__file__))
app.recording = True
shown.clear(); app._update_from_github()
check("refuses while recording", shown[0][0], "warn")
check("says why", "being recorded" in shown[0][2], True)
app.recording = False

app._char_active = True
shown.clear(); app._update_from_github()
check("refuses during a sweep", "sweep" in shown[0][2], True)
app._char_active = False

app.replay_running = True
shown.clear(); app._update_from_github()
check("refuses during a replay", "replay" in shown[0][2], True)
app.replay_running = False

with app._lock:
    app.live["state"] = "SWEEP"
shown.clear(); app._update_from_github()
check("refuses while the controller is running", "SWEEP" in shown[0][2], True)
with app._lock:
    app.live["state"] = "IDLE"
check("nothing blocks it when idle", app._update_blocked_reason(), None)

# ── declining changes nothing ────────────────────────────────────────────
work = tempfile.mkdtemp()
for n in ("dyno_gui.py", "dyno_dsp.py"):
    with open(os.path.join(work, n), "w") as f:
        f.write("# original\n")
orig_file = dyno_gui.__file__
dyno_gui.__file__ = os.path.join(work, "dyno_gui.py")
try:
    # No firmware has been fetched into this directory, so flashing has
    # nothing to work with. Checked here rather than against the real install
    # directory, which on a machine that has been flashing all day does have one.
    app.port_var.set("COM9")
    shown.clear(); app._flash_downloaded_firmware()
    check("nothing downloaded is reported clearly",
          any("Nothing to flash" in x[1] or "No downloaded firmware" in x[2]
              for x in shown), True)

    answer[0] = False
    shown.clear(); app._update_from_github()
    check("declining leaves the files alone",
          open(os.path.join(work, "dyno_gui.py")).read(), "# original\n")
    check("button re-enabled after declining",
          str(app.update_btn.cget("state")), "normal")

    # ── code that will not compile is rejected before anything is written ──
    answer[0] = True
    repo["pi_gui/dyno_gui.py"] = BAD_GUI.encode()
    shown.clear(); app._update_from_github()
    check("broken download is rejected",
          any(s[0] == "error" for s in shown), True)
    check("says it would not compile",
          any("does not compile" in s[2] for s in shown), True)
    check("and nothing was written",
          open(os.path.join(work, "dyno_gui.py")).read(), "# original\n")
    check("no backup left behind for a rejected update",
          [d for d in os.listdir(work) if d.startswith("backup_")], [])

    # ── a good update installs, keeping what it replaced ──────────────────
    repo["pi_gui/dyno_gui.py"] = GOOD_GUI.encode()
    shown.clear(); app._update_from_github()
    check("installed", open(os.path.join(work, "dyno_gui.py")).read(), GOOD_GUI)
    check("second file installed too",
          open(os.path.join(work, "dyno_dsp.py")).read(), "# dsp\n")
    backups = [d for d in os.listdir(work) if d.startswith("backup_")]
    check("previous files kept", len(backups), 1)
    check("backup holds what was replaced",
          open(os.path.join(work, backups[0], "dyno_gui.py")).read(),
          "# original\n")
    check("firmware downloaded, not flashed",
          os.path.exists(os.path.join(work, "fw_new", "firmware.bin")), True)
    check("told the operator it was not flashed",
          any("NOT" in s[2] and "flashed" in s[2] for s in shown), True)
    check("asked for a restart",
          any("Restart" in s[2] for s in shown), True)
    check("no leftover temporary files",
          [f for f in os.listdir(work) if f.endswith(".new")], [])


    # ── a read-only target stops it before anything is fetched ────────────
    # This is what a GUI previously run under sudo leaves behind: root-owned
    # files an ordinary user cannot replace.
    import stat as _stat
    fetched = []
    def counting_fetch(path):
        fetched.append(path)
        return fake_fetch(path)
    app._remote_bytes = counting_fetch
    target = os.path.join(work, "dyno_gui.py")
    os.chmod(target, _stat.S_IRUSR)          # read-only for the owner
    try:
        readonly = (not os.access(target, os.W_OK))
    except Exception:
        readonly = False
    if readonly:                              # root ignores the bit; skip if so
        shown.clear(); fetched.clear()
        app._update_from_github()
        check("read-only target reported",
              any(x[0] == "error" for x in shown), True)
        check("names the file", any("dyno_gui.py" in x[2] for x in shown), True)
        check("explains the sudo cause", any("sudo" in x[2] for x in shown), True)
        check("and nothing was downloaded", fetched, [])
    else:
        print("  skip  read-only check (running as root)")
    os.chmod(target, _stat.S_IRUSR | _stat.S_IWUSR)
    check("writable again once the mode is restored",
          app._unwritable_targets(), [])
    app._remote_bytes = fake_fetch

    # ── a repository that cannot be reached changes nothing ───────────────
    def dead(path):
        raise OSError("network is unreachable")
    app._remote_bytes = dead
    shown.clear(); app._update_from_github()
    check("unreachable repository reported",
          any(s[0] == "error" for s in shown), True)
    check("and says nothing changed",
          any("Nothing has been changed" in s[2] for s in shown), True)
    check("button usable again", str(app.update_btn.cget("state")), "normal")
    app._remote_bytes = fake_fetch

    # ── flashing is separate and guarded ──────────────────────────────────
    class FakeSer:
        is_open = True
    app.ser = FakeSer()
    shown.clear(); app._flash_downloaded_firmware()
    check("will not flash while connected",
          any("Disconnect" in s[1] for s in shown), True)
    app.ser = None
    app.recording = True
    shown.clear(); app._flash_downloaded_firmware()
    check("will not flash while recording", shown[0][0], "warn")
    app.recording = False
    app.port_var.set("")
    shown.clear(); app._flash_downloaded_firmware()
    check("will not flash without a port",
          any("port" in s[1].lower() for s in shown), True)

    # ── a flash that actually runs, with esptool faked ────────────────────
    # Every other flash case here stops at a guard, so nothing reached the end
    # of a successful flash - which is where a stale message was still living.
    import subprocess as _sp
    class _Done:
        returncode = 0
        stdout = ("Wrote 337312 bytes" + chr(10)
                  + "Hash of data verified." + chr(10))
        stderr = ""
    class _Failed:
        returncode = 2
        stdout = ""
        stderr = "A fatal error occurred: could not open port"
    real_run = _sp.run
    calls = []
    dyno_gui.subprocess.run = lambda cmd, **k: (calls.append(cmd), _Done())[1]
    os.makedirs(os.path.join(work, "fw_new"), exist_ok=True)
    with open(os.path.join(work, "fw_new", "firmware.bin"), "wb") as f:
        f.write(bytes(16))
    app.ser = None
    app.recording = False
    app.port_var.set("/dev/ttyUSB0")
    shown.clear(); calls.clear()
    app._flash_downloaded_firmware()
    check("flash reported as done", any("Flashed" == x[1] for x in shown), True)
    check("esptool actually invoked", bool(calls), True)
    check("wrote to the right offset", "0x10000" in calls[0], True)
    check("used the port that was chosen", "/dev/ttyUSB0" in calls[0], True)
    check("told the operator to re-send settings",
          any("Send all" in x[2] for x in shown), True)
    # nothing left over from the old placeholder
    check("no stale placeholder message",
          any("not implemented" in x[2].lower() for x in shown), False)
    check("exactly one dialog after confirming",
          len([x for x in shown if x[0] != "ASK"]), 1)

    # a failing esptool is reported, not silently treated as success
    dyno_gui.subprocess.run = lambda cmd, **k: _Failed()
    shown.clear()
    app._flash_downloaded_firmware()
    check("a failed flash is reported",
          any(x[0] == "error" for x in shown), True)
    check("and shows what esptool said",
          any("could not open port" in x[2] for x in shown), True)
    # exit 0 but no verification line must not count as success
    class _NoHash:
        returncode = 0
        stdout = "Wrote 337312 bytes" + chr(10)
        stderr = ""
    dyno_gui.subprocess.run = lambda cmd, **k: _NoHash()
    shown.clear()
    app._flash_downloaded_firmware()
    check("unverified write is not called success",
          any(x[0] == "error" for x in shown), True)
    dyno_gui.subprocess.run = real_run

finally:
    dyno_gui.__file__ = orig_file
    shutil.rmtree(work, ignore_errors=True)

app.on_close()
print("FAILURES: " + ("none" if not fails else "\n  " + "\n  ".join(fails)))
raise SystemExit(1 if fails else 0)

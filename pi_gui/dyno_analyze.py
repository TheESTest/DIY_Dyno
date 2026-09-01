#!/usr/bin/env python3
"""Analyse dyno runs and brake calibration sweeps.

Standalone: it does not import the GUI, so it runs anywhere the data does -
a laptop, a colleague's machine, or straight against the repository.

    python dyno_analyze.py                    the default run folder
    python dyno_analyze.py <file-or-folder>   a specific run, or a folder of them
    python dyno_analyze.py --github           fetch data/ from the repository
    python dyno_analyze.py --plot             also write a PNG per file

Every check here exists because it caught something real on this rig. The
point is not to draw pretty curves - it is to say, without being asked, when
a data set is not what it appears to be:

  * a run recorded while nothing was connected looks like a run until you
    notice every channel is a constant zero
  * a sweep whose position trails the command by a CONSTANT RATIO was clamped
    by the controller, which means the GUI and the firmware disagree about the
    brake range - quite different from a stall, where the gap grows
  * a Kp that reads as a plausible small number can be fifty times too small
    for the travel it is working against
  * a pressure that collapses at the same figure every time is a hydraulic
    limit, not noise
"""
import argparse
import csv
import glob
import json
import os
import sys

import numpy as np

REPO = "TheESTest/DIY_Dyno"
RAW = "https://raw.githubusercontent.com/{repo}/main/{path}"
API = "https://api.github.com/repos/{repo}/contents/data"
DEFAULT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dyno_runs")

# Pressure at which this rig has repeatedly let go. Not a sensor artefact: it
# has happened on separate sweeps at 1123, 1125 and 1146 PSI, each time a drop
# of 65-72% in a single 50 ms sample while position was still increasing.
COLLAPSE_WATCH_PSI = 900.0
COLLAPSE_FRAC = 0.35          # a fall to below this share of the peak

RESET = "\033[0m"
BOLD = "\033[1m"
RED = "\033[31m"
AMBER = "\033[33m"
GREEN = "\033[32m"
DIM = "\033[2m"


def paint(text, colour):
    """Colour for a terminal, plain when piped to a file."""
    if not sys.stdout.isatty():
        return text
    return f"{colour}{text}{RESET}"


def note(msg):
    print("    " + msg)


def finding(level, msg):
    """One conclusion. Levels: ok, warn, bad, info."""
    mark, colour = {"ok": ("OK  ", GREEN), "warn": ("NOTE", AMBER),
                    "bad": ("!!  ", RED), "info": ("    ", DIM)}[level]
    print(f"  {paint(mark, colour)} {msg}")


# ─────────────────────────────────────────────────────────────────────────
# Loading
# ─────────────────────────────────────────────────────────────────────────
def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def column(rows, name, default=None):
    """One column as floats, or `default` if it is absent or unparseable."""
    if not rows or name not in rows[0]:
        return default
    out = []
    for r in rows:
        try:
            out.append(float(r[name]))
        except (TypeError, ValueError):
            out.append(np.nan)
    return np.asarray(out, dtype=float)


def conditions_for(path):
    """The settings file written beside a run, if it is there."""
    stem = os.path.splitext(path)[0]
    for cand in (stem + "_conditions.json", stem + ".json"):
        if os.path.exists(cand):
            try:
                with open(cand) as f:
                    return json.load(f)
            except (OSError, ValueError):
                return None
    return None


def fetch_github(dest):
    """Pull the repository's data folder down so it can be analysed offline."""
    import urllib.request

    os.makedirs(dest, exist_ok=True)
    with urllib.request.urlopen(API.format(repo=REPO), timeout=30) as r:
        listing = json.loads(r.read().decode())
    got = []
    for entry in listing:
        if entry["type"] != "file" or entry["name"] == "README.md":
            continue
        target = os.path.join(dest, entry["name"])
        url = RAW.format(repo=REPO, path=f"data/{entry['name']}")
        with urllib.request.urlopen(url, timeout=60) as r:
            blob = r.read()
        with open(target, "wb") as f:
            f.write(blob)
        got.append(target)
    return got


# ─────────────────────────────────────────────────────────────────────────
# Checks shared by runs and sweeps
# ─────────────────────────────────────────────────────────────────────────
def telemetry_is_real(rows):
    """Was anything actually connected while this was recorded?

    A disconnected GUI still writes a complete-looking file: the timestamps
    and the commanded ramp are computed locally. What gives it away is that
    every measured channel is one constant value. The pressure sensor sits
    near 500 mV at atmosphere and the load cell near 800 mV, so a flat zero
    is not a reading at all.
    """
    measured = ("Pressure_mV", "LoadCell_mV", "Brake_PSI", "RPM", "Brake_Pos")
    present = [c for c in measured if c in (rows[0] if rows else {})]
    if not present:
        return True, "no measured channels to judge"
    dead = []
    for name in present:
        v = column(rows, name)
        if v is None:
            continue
        finite = v[np.isfinite(v)]
        if finite.size and np.all(finite == finite[0]) and finite[0] == 0.0:
            dead.append(name)
    if len(dead) >= 3:
        return False, ("every measured channel is a constant zero (" +
                       ", ".join(dead) + ") - nothing was connected")
    return True, ""


def gain_sanity(cond):
    """What the recorded gains are worth against the recorded brake range."""
    if not cond:
        return None
    try:
        lo = float(cond["brake_min"])
        hi = float(cond["brake_max"])
        kp = float(cond["pid_hold"]["kp"])
        ki = float(cond["pid_hold"]["ki"])
    except (KeyError, TypeError, ValueError):
        return None
    span = hi - lo
    if span <= 0 or kp <= 0:
        return {"span": span, "kp": kp, "ki": ki, "rpm_for_full": None}
    return {"span": span, "kp": kp, "ki": ki, "rpm_for_full": span / kp}


def find_pressure_collapse(pos, psi):
    """A sudden pressure loss while position is still going up.

    Returns the sample index of the largest single-sample drop, when that drop
    is big enough and high enough to be the hydraulic giving way rather than
    the brake simply being released.
    """
    if pos is None or psi is None or psi.size < 5:
        return None
    d = np.diff(psi)
    i = int(np.argmin(d))
    peak = psi[i]
    after = psi[i + 1]
    rising = pos[i + 1] >= pos[i]
    if peak >= COLLAPSE_WATCH_PSI and after < peak * COLLAPSE_FRAC and rising:
        return {"index": i, "peak": peak, "after": after, "pos": pos[i],
                "drop_pct": 100.0 * (peak - after) / peak}
    return None


def effective_range(pos, psi, rise=0.05, top=0.95, min_span=5.0):
    """Where actuator position actually produces pressure.

    Below takeup the linkage is closing up; above saturation pressure has
    stopped answering. Only the span between them is worth giving to a
    control loop.
    """
    if pos is None or psi is None or pos.size < 10:
        return {"usable": False, "reason": "not enough samples"}
    if float(np.nanmax(pos)) - float(np.nanmin(pos)) < 1.0:
        return {"usable": False,
                "reason": "the actuator never moved during the sweep"}
    order = np.argsort(pos)
    p, q = pos[order], psi[order]
    n0 = max(3, p.size // 10)
    base = float(np.median(q[:n0]))
    span = float(np.nanmax(q)) - base
    if not np.isfinite(span) or span < min_span:
        return {"usable": False,
                "reason": f"pressure never rose more than {span:.1f} PSI "
                          "above its baseline"}
    above = np.flatnonzero(q >= base + rise * span)
    below = np.flatnonzero(q >= base + top * span)
    takeup = float(p[above[0]]) if above.size else float(p[0])
    sat = float(p[below[0]]) if below.size else float(p[-1])
    if sat <= takeup:
        return {"usable": False, "reason": "pressure rose too abruptly to "
                                           "separate takeup from saturation"}
    return {"usable": True, "baseline_psi": base, "psi_span": span,
            "takeup": takeup, "saturation": sat, "span": sat - takeup,
            "psi_per_step": span * (top - rise) / (sat - takeup),
            "dead_pct": 100.0 * takeup / max(float(np.nanmax(p)), 1.0)}


def follow_quality(cmd, pos):
    """Did the actuator go where it was told, and if not, in what way?

    The distinction that matters: a CONSTANT ratio between reached and
    commanded means the controller was working to a different target than the
    interface thought - it clamps the target to its own brake range, so the
    two have fallen out of step. A ratio that DEGRADES as load builds is the
    motor losing steps. They look similar in a summary and are different
    faults.
    """
    if cmd is None or pos is None or cmd.size < 20:
        return None
    m = cmd > max(20.0, 0.05 * np.nanmax(cmd))
    if m.sum() < 10:
        return None
    ratio = pos[m] / cmd[m]
    ratio = ratio[np.isfinite(ratio)]
    if ratio.size < 10:
        return None
    half = ratio.size // 2
    early, late = float(np.mean(ratio[:half])), float(np.mean(ratio[half:]))
    out = {"mean": float(np.mean(ratio)), "sd": float(np.std(ratio)),
           "early": early, "late": late,
           "reached": float(np.nanmax(pos)), "commanded": float(np.nanmax(cmd))}
    if out["mean"] > 0.97:
        out["verdict"] = "followed"
    elif out["sd"] < 0.05 and abs(early - late) < 0.06:
        out["verdict"] = "clamped"       # steady shortfall: a different target
    else:
        out["verdict"] = "losing"        # gets worse under load: lost steps
    return out


# ─────────────────────────────────────────────────────────────────────────
# The two file kinds
# ─────────────────────────────────────────────────────────────────────────
def analyse_sweep(path, rows, cond):
    print(paint(f"\n{os.path.basename(path)}  -  brake calibration sweep", BOLD))
    t = column(rows, "Time_s")
    cmd = column(rows, "Commanded_Steps")
    pos = column(rows, "Brake_Pos")
    psi = column(rows, "Brake_PSI")
    phase = np.array([r.get("Phase", "") for r in rows])
    note(f"{len(rows)} samples over {t[-1]:.1f} s" if t is not None else
         f"{len(rows)} samples")

    live, why = telemetry_is_real(rows)
    if not live:
        finding("bad", f"NOT A REAL SWEEP: {why}")
        finding("info", "run it again with the controller connected and the "
                        "readings visibly moving")
        return
    finding("ok", "telemetry was live")

    up = phase == "up"
    if up.sum() < 10:
        up = np.ones(len(rows), bool)

    fq = follow_quality(cmd[up] if cmd is not None else None,
                        pos[up] if pos is not None else None)
    if fq:
        note(f"commanded up to {fq['commanded']:.0f} steps, reached "
             f"{fq['reached']:.0f}")
        if fq["verdict"] == "followed":
            finding("ok", "the actuator followed the command")
        elif fq["verdict"] == "clamped":
            short = 100 * (1 - fq["mean"])
            finding("bad",
                    f"the actuator ran a CONSTANT {fq['mean']:.3f} of the "
                    f"command ({short:.0f}% short, sd {fq['sd']:.3f})")
            finding("info",
                    "a steady ratio is not a stall - the controller was "
                    "working to a smaller target than the interface sent")
            finding("info",
                    "the controller clamps a sweep to its own brake range, so "
                    "press Send all to controller and sweep again")
            if cond and "brake_max" in cond:
                implied = fq["reached"]
                finding("info",
                        f"the interface had brake_max {cond['brake_max']}, "
                        f"the controller behaved as if it were ~{implied:.0f}")
        else:
            finding("bad",
                    f"the actuator fell progressively behind: {fq['early']:.2f} "
                    f"of the command early, {fq['late']:.2f} late")
            finding("info", "a ratio that worsens under load is lost steps - "
                            "check drive current and for binding")

    # A sweep that did not complete has only explored part of the travel, so
    # any range taken from it is a floor, not the answer. Saying otherwise
    # would hand over a confident number built on a truncated curve.
    complete = not fq or fq["verdict"] == "followed"
    er = effective_range(pos[up], psi[up])
    if er.get("usable"):
        finding("ok", f"takeup {er['takeup']:.0f} steps, saturation "
                      f"{er['saturation']:.0f}, {er['psi_per_step']:.2f} PSI "
                      f"per step")
        note(f"usable span {er['span']:.0f} steps, {er['psi_span']:.0f} PSI, "
             f"{er['dead_pct']:.0f}% of travel dead before the pads bite")
        if complete:
            finding("info", f"set the brake range to {er['takeup']:.0f}-"
                            f"{er['saturation']:.0f} so every commanded step "
                            "does something")
        else:
            finding("warn",
                    "do NOT set the brake range from this sweep - the "
                    "actuator never reached the end of its travel, so the "
                    "curve is cut short and the saturation figure is only "
                    "wherever it happened to stop")
            finding("info", "fix the shortfall above, sweep again, then take "
                            "the range from that")
    else:
        finding("warn", f"no usable range: {er.get('reason')}")

    col = find_pressure_collapse(pos, psi)
    if col:
        finding("bad", f"pressure collapsed {col['peak']:.0f} -> "
                       f"{col['after']:.0f} PSI ({col['drop_pct']:.0f}%) at "
                       f"{col['pos']:.0f} steps, while still advancing")
        finding("info", "this rig has done that repeatedly near 1150 PSI, "
                        "above the master cylinder's 1000 PSI rating - treat "
                        "it as a hard ceiling and lower PRESS_LIMIT to match")
    elif psi is not None and psi.size:
        finding("ok", f"peak pressure {np.nanmax(psi):.0f} PSI, no collapse")

    stall = [r.get("Stall_Suspected", "0") for r in rows]
    if "1" in stall:
        finding("warn", "the interface flagged a possible stall during this sweep")

    report_conditions(cond)


def analyse_run(path, rows, cond):
    print(paint(f"\n{os.path.basename(path)}  -  engine run", BOLD))
    t = column(rows, "Time_s")
    rpm = column(rows, "RPM")
    tgt = column(rows, "Target_RPM")
    pos = column(rows, "Brake_Pos")
    psi = column(rows, "Brake_PSI")
    state = [r.get("State", "") for r in rows]
    note(f"{len(rows)} samples over {t[-1]:.1f} s"
         f"   states: {', '.join(sorted(set(state)))}" if t is not None else "")

    live, why = telemetry_is_real(rows)
    if not live:
        finding("bad", f"NOT A REAL RUN: {why}")
        return
    finding("ok", "telemetry was live")

    if rpm is not None and rpm.size:
        zeros = int(np.sum(rpm == 0))
        running = rpm[rpm > 200]
        if running.size:
            note(f"RPM {running.min():.0f}-{running.max():.0f} while running, "
                 f"median {np.median(running):.0f}")
            # Detrended, so the engine genuinely changing speed is not counted
            # as noise. This is the number that says whether the tach is good.
            k = 21
            med = np.array([np.median(running[max(0, i - k // 2):i + k // 2 + 1])
                            for i in range(running.size)])
            resid = running - med
            pct = 100 * resid.std() / max(np.median(running), 1)
            if pct < 1.0:
                finding("ok", f"tach noise {resid.std():.1f} RPM ({pct:.2f}% "
                              "of reading) - a clean signal")
            else:
                finding("warn", f"tach noise {resid.std():.1f} RPM ({pct:.2f}% "
                                "of reading)")
        if zeros:
            finding("bad", f"{zeros} samples ({100 * zeros / rpm.size:.1f}%) "
                           "read exactly zero")
            finding("info", "dropouts to exactly zero are the no-pulse timeout "
                            "firing; firmware 1.6.1 fixed a race that caused "
                            "these spuriously - check the firmware version")
        else:
            finding("ok", "no dropouts to zero")

    # The three tach methods, when the firmware reported them
    counted, rev = column(rows, "RPM_Counted"), column(rows, "RPM_Rev")
    if counted is not None and rev is not None:
        m = (rpm > 200) & (counted > 200) & (rev > 200)
        if m.sum() > 20:
            note(f"method spread while running: gap-vs-counted "
                 f"{np.mean(np.abs(rpm[m] - counted[m])):.1f} RPM, "
                 f"gap-vs-revolution {np.mean(np.abs(rpm[m] - rev[m])):.1f} RPM")
            finding("info", "a large gap-vs-revolution difference means uneven "
                            "tooth spacing; revolution timing cancels it")

    gl, est = column(rows, "Tach_Glitches"), column(rows, "RPM_Estimated")
    if gl is not None and gl.size and t is not None and t[-1] > 0:
        rate = (gl[-1] - gl[0]) / t[-1]
        finding("ok" if rate < 1 else "warn",
                f"{gl[-1] - gl[0]:.0f} edges rejected as noise "
                f"({rate:.1f}/s)")
    if est is not None and est.size and est[-1] > est[0]:
        finding("warn", f"{est[-1] - est[0]:.0f} RPM samples were estimated, "
                        "not measured")

    # Did the brake actually do anything?
    holding = np.array(["HOLD" in s for s in state])
    if holding.any() and rpm is not None and tgt is not None:
        err = float(np.mean(rpm[holding] - tgt[holding]))
        note(f"during HOLD: target {np.mean(tgt[holding]):.0f}, actual "
             f"{np.mean(rpm[holding]):.0f}, error {err:+.0f} RPM")
        if pos is not None:
            moved = float(np.nanmax(pos[holding]) - np.nanmin(pos[holding]))
            note(f"brake moved {moved:.0f} steps during HOLD, peak pressure "
                 f"{np.nanmax(psi[holding]) if psi is not None else float('nan'):.0f} PSI")
            g = gain_sanity(cond)
            if g and g["rpm_for_full"] and abs(err) > 50:
                want = abs(err) * g["kp"]
                if moved < 0.5 * want or moved < 20:
                    finding("bad", f"the brake barely moved for {abs(err):.0f} "
                                   "RPM of error")
                if g["rpm_for_full"] > 3000:
                    finding("bad",
                            f"Kp {g['kp']:.6g} over {g['span']:.0f} steps means "
                            f"full brake at {g['rpm_for_full']:,.0f} RPM of "
                            "error - far too weak to control with")
                if g["ki"] == 0:
                    finding("bad", "Ki is zero, so steady error is never "
                                   "removed and dead travel is never walked out")

    report_conditions(cond)


def report_conditions(cond):
    if not cond:
        finding("warn", "no conditions file beside this run - the settings "
                        "that produced it are unknown")
        return
    ui, fw = cond.get("_ui_version"), cond.get("_fw_version")
    note(f"recorded by interface {ui}, firmware {fw}")
    if cond.get("_notes"):
        note(f"notes: {cond['_notes']}")
    g = gain_sanity(cond)
    if g and g["rpm_for_full"]:
        level = "ok" if 200 <= g["rpm_for_full"] <= 3000 else "bad"
        finding(level, f"Kp {g['kp']:.6g} over {g['span']:.0f} steps = full "
                       f"brake at {g['rpm_for_full']:,.0f} RPM of error")
    try:
        limit = float(cond.get("press_limit", 0))
        if limit >= 1400:
            finding("warn", f"PRESS_LIMIT is {limit:.0f} PSI, above where this "
                            "rig has let go - it will never intervene")
    except (TypeError, ValueError):
        pass


# ─────────────────────────────────────────────────────────────────────────
# Plotting
# ─────────────────────────────────────────────────────────────────────────
def plot_file(path, rows, kind, out_dir):
    """One figure per file. One measure per axis - never a shared y-scale."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
    BLUE, ORANGE = "#2a78d6", "#eb6834"

    def dress(ax, ylabel):
        ax.set_facecolor(SURFACE)
        ax.grid(True, color=GRID, linewidth=0.8)
        ax.set_axisbelow(True)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
        for s in ("left", "bottom"):
            ax.spines[s].set_color(GRID)
        ax.tick_params(colors=INK2, labelsize=9, length=0)
        ax.set_ylabel(ylabel, color=INK2, fontsize=9.5)

    stem = os.path.splitext(os.path.basename(path))[0]
    out = os.path.join(out_dir, stem + "_analysis.png")

    if kind == "sweep":
        cmd = column(rows, "Commanded_Steps")
        pos = column(rows, "Brake_Pos")
        psi = column(rows, "Brake_PSI")
        t = column(rows, "Time_s")
        ph = np.array([r.get("Phase", "") for r in rows])
        fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 5.2), dpi=120,
                                     facecolor=SURFACE)
        dress(a1, "Brake position (steps)")
        a1.plot(t, cmd, color=ORANGE, lw=2, ls=(0, (5, 3)))
        a1.plot(t, pos, color=BLUE, lw=2)
        a1.set_xlabel("Time (s)", color=INK2, fontsize=9.5)
        a1.annotate("commanded", (t[len(t) // 3], cmd[len(t) // 3]),
                    xytext=(0, 10), textcoords="offset points",
                    color=ORANGE, fontweight="bold", fontsize=9.5)
        a1.annotate("reached", (t[len(t) // 2], pos[len(t) // 2]),
                    xytext=(0, -18), textcoords="offset points",
                    color=BLUE, fontweight="bold", fontsize=9.5)
        a1.set_title("Did it go where it was told?", color=INK, fontsize=11,
                     fontweight="bold", loc="left")

        dress(a2, "Line pressure (PSI)")
        up, dn = ph == "up", ph == "down"
        a2.plot(pos[up], psi[up], color=BLUE, lw=2)
        if dn.any():
            a2.plot(pos[dn], psi[dn], color=ORANGE, lw=2)
        a2.set_xlabel("Brake position (steps)", color=INK2, fontsize=9.5)
        er = effective_range(pos[up], psi[up])
        if er.get("usable"):
            for x, lab in ((er["takeup"], "takeup"), (er["saturation"], "sat")):
                a2.axvline(x, color=ORANGE, lw=1.2, ls=(0, (4, 3)))
                a2.annotate(f"{lab} {x:.0f}", (x, np.nanmax(psi) * 0.9),
                            xytext=(5, 0), textcoords="offset points",
                            color=ORANGE, fontsize=9, fontweight="bold")
        a2.set_title("Pressure against position", color=INK, fontsize=11,
                     fontweight="bold", loc="left")
    else:
        t = column(rows, "Time_s")
        rpm, tgt = column(rows, "RPM"), column(rows, "Target_RPM")
        pos, psi = column(rows, "Brake_Pos"), column(rows, "Brake_PSI")
        fig, axes = plt.subplots(3, 1, figsize=(11, 7.5), dpi=120, sharex=True,
                                 facecolor=SURFACE)
        dress(axes[0], "RPM")
        axes[0].plot(t, rpm, color=BLUE, lw=1.8)
        if tgt is not None:
            axes[0].plot(t, tgt, color=ORANGE, lw=1.8, ls=(0, (5, 3)))
        axes[0].set_title("RPM against target", color=INK, fontsize=11,
                          fontweight="bold", loc="left")
        dress(axes[1], "Brake (steps)")
        if pos is not None:
            axes[1].plot(t, pos, color=BLUE, lw=1.8)
        dress(axes[2], "Pressure (PSI)")
        if psi is not None:
            axes[2].plot(t, psi, color=BLUE, lw=1.8)
        axes[2].set_xlabel("Time (s)", color=INK2, fontsize=9.5)

    fig.suptitle(stem, color=INK, fontsize=12, fontweight="bold", x=0.02,
                 ha="left")
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out, facecolor=SURFACE)
    plt.close(fig)
    return out


# ─────────────────────────────────────────────────────────────────────────
def kind_of(path, rows):
    name = os.path.basename(path).lower()
    if "brake_char" in name or (rows and "Commanded_Steps" in rows[0]):
        return "sweep"
    if "pulses" in name or (rows and "Interval_us" in rows[0]):
        return "pulses"
    return "run"


def analyse_pulses(path, rows):
    print(paint(f"\n{os.path.basename(path)}  -  raw pulse capture", BOLD))
    dt = column(rows, "Interval_us")
    ok = column(rows, "Accepted")
    if dt is None:
        finding("warn", "no Interval_us column")
        return
    acc = dt[(ok == 1)] if ok is not None else dt
    if acc.size < 10:
        finding("warn", f"only {acc.size} accepted pulses")
        return
    med = float(np.median(acc[1:]))
    dropped = 0
    for v in acc[1:]:
        r = v / med
        n = round(r)
        if n >= 2 and abs(r - n) < 0.15:
            dropped += int(n) - 1
    note(f"{acc.size} accepted pulses, median interval {med:.0f} us")
    total = acc.size + dropped
    if dropped:
        finding("bad", f"about {dropped} teeth missed "
                       f"({100 * dropped / max(total, 1):.2f}%)")
    else:
        finding("ok", "no missed teeth")
    if ok is not None:
        rej = int(np.sum(ok == 0))
        finding("ok" if rej == 0 else "warn",
                f"{rej} edges rejected as noise")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", nargs="?", default=None,
                    help="a CSV file, or a folder of them")
    ap.add_argument("--github", action="store_true",
                    help="fetch the repository's data folder first")
    ap.add_argument("--plot", action="store_true", help="write a PNG per file")
    args = ap.parse_args()

    target = args.target or DEFAULT_DIR
    if args.github:
        dest = target if args.target else os.path.join(DEFAULT_DIR, "from_github")
        print(f"fetching data/ from {REPO} into {dest}")
        try:
            got = fetch_github(dest)
        except Exception as e:
            print(paint(f"  could not fetch: {e}", RED))
            return 1
        print(f"  {len(got)} files")
        target = dest

    if os.path.isdir(target):
        files = sorted(f for f in glob.glob(os.path.join(target, "*.csv"))
                       if not f.endswith("_filtered.csv"))
    elif os.path.exists(target):
        files = [target]
    else:
        print(paint(f"nothing at {target}", RED))
        return 1
    if not files:
        print(f"no CSV files in {target}")
        return 1

    print(f"\n{len(files)} file(s) in {target}")
    for path in files:
        try:
            rows = read_csv(path)
        except OSError as e:
            print(f"\n{os.path.basename(path)}: {e}")
            continue
        if not rows:
            print(f"\n{os.path.basename(path)}: empty")
            continue
        kind = kind_of(path, rows)
        cond = conditions_for(path)
        if kind == "sweep":
            analyse_sweep(path, rows, cond)
        elif kind == "pulses":
            analyse_pulses(path, rows)
        else:
            analyse_run(path, rows, cond)
        if args.plot and kind in ("sweep", "run"):
            try:
                out = plot_file(path, rows, kind, os.path.dirname(path) or ".")
                note(f"plot: {os.path.basename(out)}")
            except Exception as e:
                finding("warn", f"plot failed: {e}")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())

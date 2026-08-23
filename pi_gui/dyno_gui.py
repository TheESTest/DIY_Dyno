#!/usr/bin/env python3
"""
DIY Engine Dyno — Raspberry Pi 5 GUI
Connects to ESP32-S3 via USB serial, OR replays a recorded CSV ("Dummy Data").

Operator procedure (manual throttle):
  1. Connect to ESP32, Home stepper, Tare load cell
  2. Set Hold RPM, sweep Start/End RPM, rate on Parameters screen
  3. Press START → PID holds brake at holdRPM
  4. Operator throttles up; PID resists to hold RPM
  5. At WOT, press RELEASE → sweep begins, recording auto-starts
  6. Sweep completes → hold at endRPM, recording auto-stops
  7. Operator throttles down → PID slowly releases brake
  8. RPM drops below holdRPM → auto-reset to IDLE, brake 0%

Two extra capabilities live in the tabbed UI:
  • "Live Run" tab — the live hardware run screen, with a Data Source selector.
    Choose "CSV Replay" to stream a recorded log through the full pipeline in
    real time (Dummy Data mode) — no hardware required — to demo the GUI/graphs.
  • "Analysis & Filtering" tab — load a recorded run (or the last live run) and
    try rolling-average / EMA / Savitzky-Golay / low-pass / polynomial filters,
    with a raw-vs-filtered Torque & HP vs RPM overlay and filtered-CSV export.
"""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import collections
import json
import threading
import time
import csv
import os

import numpy as np
import serial
import serial.tools.list_ports
import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

import dyno_dsp as dsp

# ── Constants ────────────────────────────────────────────────
BAUD_RATE = 115200
GUI_UPDATE_MS = 250
HP_FACTOR = dsp.HP_NM_CONST   # HP = Torque(Nm) * RPM / 7120.9
SERIAL_TIMEOUT = 0.05

# Logging cap. The old 5000 was 250 s at 20 Hz, which silently truncated any
# longer test — and would be well under one pull once the report rate goes up.
# The live plot decimates instead of dropping samples, so the log stays whole.
MAX_LOG_POINTS = 120000
PLOT_MAX_POINTS = 4000        # live plot strides down to roughly this many

# Rolling live view. The plot used to move only while recording, so between
# runs it sat frozen and you could not watch the engine at all.
MONITOR_WINDOW_S = 60
MONITOR_MAX_POINTS = 3000

STATUS_POLL_MS = 2000         # re-ask READY? so SIM/LIVE can't drift out of date
EVENT_LOG_LINES = 400

# RPM conditioning is four independent gates rather than one mode, because on
# this rig they address different things: a ~2500 Hz electrical burst on the
# pickup (gate 1), the odd isolated bad interval (gate 2), and residue (3, 4).
RPM_MEDIAN_WINDOWS = ["1 (off)", "3", "5", "7"]

# How stepper position translates into actual brake application. The pusher is
# cam driven, so these are not the same thing except in the linear case.
CAM_MODELS = ["Linear (steps = brake)", "Eccentric cam", "Measured table"]

# How the brake comes off once the operator lifts. Dumping it instantly is what
# the reference system warns against for engine dynos; holding it on stalls the
# engine. These are the strategies to try on the stand, in firmware order.
RAMPDOWN_MODES = [
    "Release immediately",
    "Controlled RPM ramp",
    "Timed brake release",
]
CAM_TABLE_ROWS = 8

# Brake actuator, as built, in MICROSTEPS: the driver runs 5000 microsteps per
# motor revolution through a 10:1 planetary, so 50000 per cam revolution.
#   50000 / 360 = 138.889 microsteps per degree;  45 deg = 6250 microsteps.
# Homing sits about a quarter motor turn clear of full travel, so a measured
# home-to-max span near 7600 is expected, not a sign of anything wrong.
BRAKE_FULL_TRAVEL_STEPS = 6250
CAM_STEPS_PER_DEGREE = 138.889

# AiM car/bike pressure sensor (Pegasus MC-327 and siblings). Every sensor in
# that range shares the same electrical characterisation — 500 mV at zero and
# 4500 mV at full scale — so only the full-scale pressure changes between parts.
# ADS1115 full-scale ranges, in the order the firmware numbers them. The widest
# is the default because this sensor reaches 4.5 V when wired straight through;
# the narrower ones are only reachable behind a divider.
PRESS_PGA_RANGES = [
    "±6.144 V  (direct)", "±4.096 V", "±2.048 V",
    "±1.024 V", "±0.512 V", "±0.256 V",
]

PRESSURE_ZERO_MV = 500.0
PRESSURE_FS_MV = 4500.0
PRESSURE_DEFAULT_FS_PSI = 2000.0

# Where completed runs land unless the operator picks somewhere else.
# Kept beside the code rather than in the home directory, so runs sit next
# to the program that made them and are easy to find over SSH.
# Interface version. Recorded beside every run together with the firmware
# version the board reported, so a result can always be traced back to the
# code that produced it.
UI_VERSION = "1.3.0"

DEFAULT_DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dyno_runs")

# Settings from the last session, reloaded on start so calibration and gains do
# not have to be re-entered (or re-remembered) every time the program opens.
SESSION_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "dyno_last_session.json")

# Brake characterisation sweep: walk the actuator across its whole travel and
# back while recording line pressure, to find where position actually starts
# producing braking. The return leg matters as much as the outward one - the
# gap between them is the mechanical and hydraulic hysteresis.
CHAR_UP_S = 10.0
CHAR_HOLD_S = 1.0
CHAR_DOWN_S = 10.0
CHAR_TICK_MS = 50            # command and sample at 20 Hz, matching DATA

# Stall watch for the sweep. Without an encoder there is no way to know the
# motor actually went where it was told, so pressure is used as the witness:
# once the brake is engaged, commanded position and pressure should climb
# together, and position climbing alone means the steps are not arriving.
# Nothing is judged until pressure has risen past CHAR_ENGAGE_PSI, because
# the takeup travel before the pads bite legitimately produces no pressure.
CHAR_ENGAGE_PSI = 20.0       # rise above baseline that counts as engaged
CHAR_STALL_STEPS = 100.0     # commanded increase over the window to judge on
CHAR_STALL_PSI = 2.0         # pressure rise expected across that increase
CHAR_STALL_WIN_S = 1.0       # sliding window the comparison is made over
CHAR_STALL_HOLD_S = 1.0      # how long it must persist before flagging

# Motor "reaction" during CSV replay — nudge the real brake stepper so it
# visibly responds to the replayed data, geared WAY down so it moves gently.
MOTOR_DEMO_SPEED = 1000    # stepper max speed (steps/s) — visible but not flying
MOTOR_DEMO_ACCEL = 1500    # stepper accel (steps/s^2)
MOTOR_FULL_STEPS = 6250    # matches firmware BRAKE_MAX_STEPS_DEF (full travel)
MOTOR_SEND_HZ    = 8       # BRAKE command rate to the ESP during replay

# DATA frame field layout (matches ESP32-S3 firmware sendDataReport()):
# DATA,millis,rpm,torque,loadRaw,adc0,adc1,brakePos,targetRPM,state,pidP,pidI,
#      pidD_out,brakePct,pressPSI,faultBits
# Fields 13-15 are appended by newer firmware; older boards simply omit them.
FAULT_TACH     = 0x01
FAULT_PRESSURE = 0x02
FAULT_NAMES = {
    FAULT_TACH: "TACH SIGNAL LOST — brake held at its last position. Close the throttle.",
    FAULT_PRESSURE: "BRAKE PRESSURE OVER LIMIT — brake will not advance further.",
}


class DynoApp:
    """Main application class for the DIY Dyno GUI."""

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title(f"DIY Engine Dyno - UI {UI_VERSION}")
        self.root.geometry("1400x960")
        self.root.minsize(1040, 640)

        # Serial state
        self.ser: serial.Serial | None = None
        self.serial_thread: threading.Thread | None = None
        self.running = False

        # Data source: "hardware" or "replay"
        self.source_var = tk.StringVar(value="hardware")
        # Imperial by default — this shop works in lb-ft.
        self.units_var = tk.StringVar(value="lb-ft")  # "Nm" or "lb-ft"

        # Replay (Dummy Data) state
        self.replay_thread: threading.Thread | None = None
        self.replay_running = False
        self.replay_paused = False
        self.replay_rec: dsp.Recording | None = None
        self.replay_speed_var = tk.StringVar(value="1.0")
        self.replay_file_var = tk.StringVar(value="")
        self.replay_progress_var = tk.DoubleVar(value=0.0)

        # Drive the real brake motor during replay (scaled down for a gentle demo)
        self.replay_motor_var = tk.BooleanVar(value=True)
        self.replay_motor_port_var = tk.StringVar(value="")
        self.replay_motor_gain_var = tk.StringVar(value="40")   # % of full travel
        self.motor_ser: serial.Serial | None = None
        self._motor_last_send = 0.0
        self._motor_max_rpm = 8000.0

        # Live values
        self._lock = threading.Lock()
        self.live = {
            "rpm": 0.0, "torque": 0.0, "load_raw": 0.0,
            "adc0": 0.0, "adc1": 0.0, "brake_pos": 0,
            "target_rpm": 0.0, "state": "IDLE",
            "pid_p": 0.0, "pid_i": 0.0, "pid_out": 0.0,
            "brake_pct": 0.0, "press_psi": 0.0, "faults": 0,
            "press_mv": 0.0, "spare_aux": 0.0, "glitches": 0,
            "estimated": 0, "enc_pos": 0, "enc_ok": 0,
        }
        # What the board says it is running. Unknown until it answers, and
        # firmware old enough not to report one leaves it that way.
        self.fw_version = "unknown"
        self.fw_build = ""

        # Readiness flags (from ESP). "sim" starts True so an unknown board is
        # treated as suspect until it tells us otherwise — the safe default is
        # to doubt the data, not to trust it.
        self.ready_flags = {
            "ready": False, "homed": False, "tared": False, "adc": False,
            "sim": True, "press_adc": False,
        }
        self._got_ready = False

        # Event log — appended from the serial thread, drained on the main
        # thread. A deque with maxlen is safe to append to from any thread.
        self.events = collections.deque(maxlen=EVENT_LOG_LINES)
        self.last_error = ""
        self._log_capped_warned = False

        # Run data for plotting (raw, as-received — never filtered in place)
        self.run_t: list[float] = []          # elapsed seconds (from millis)
        self.run_rpm: list[float] = []
        self.run_torque: list[float] = []     # native units (Nm from hardware)
        self.run_hp: list[float] = []
        self.run_psi: list[float] = []
        self.run_brake: list[float] = []
        # Always-on rolling buffers so the plot lives whether or not a run is
        # being recorded. Bounded, so an idling engine cannot grow them forever.
        self.mon_t = collections.deque(maxlen=MONITOR_MAX_POINTS)
        self.mon_rpm = collections.deque(maxlen=MONITOR_MAX_POINTS)
        self.mon_torque = collections.deque(maxlen=MONITOR_MAX_POINTS)
        self.mon_hp = collections.deque(maxlen=MONITOR_MAX_POINTS)
        self.mon_psi = collections.deque(maxlen=MONITOR_MAX_POINTS)
        self.mon_brake = collections.deque(maxlen=MONITOR_MAX_POINTS)
        self._plot_source = None
        # Five channels on four scales gets crowded on the Pi's screen, so each
        # one can be turned off. Carried in the profile like any other setting.
        self.trace_vars = {
            "rpm": tk.BooleanVar(value=True),
            "torque": tk.BooleanVar(value=True),
            "hp": tk.BooleanVar(value=True),
            "psi": tk.BooleanVar(value=True),
            "brake": tk.BooleanVar(value=True),
        }
        self._mon_xlim = None
        self._mon_t0 = None        # board clock at the first monitored sample
        self.recording = False
        self.auto_recording = False           # Auto-started by sweep lifecycle
        self.recorded_torque_is_nm = True     # hardware torque is Nm
        self._plot_dirty = False
        self._updating = False

        # Live-plot blitting state — keeps the live plot smooth on slow displays
        # (e.g. the Pi's XWayland desktop, where a full redraw is ~35 ms vs ~3 ms
        # to blit just the lines). A full redraw happens only when the axis
        # limits must grow; otherwise we blit the line artists over a cached bg.
        self._live_bg = None
        self._live_canvas_size = None
        self._live_xlim = None       # time axis
        self._live_lylim = None      # left y (RPM)
        self._live_rylim = None      # right y (Torque + HP)
        self._live_psilim = None     # outer right y (Brake PSI)
        self._live_brklim = None     # outermost right y (Brake position)
        self._trace_vis = None       # which traces were drawn last time

        # Live display smoothing (EMA)
        self.live_smooth_var = tk.BooleanVar(value=True)
        self.live_alpha_var = tk.StringVar(value="0.30")

        # Full log for CSV export
        self.log_rows: list[list[str]] = []

        # Analysis tab dataset
        self.analysis_rpm: np.ndarray | None = None
        self.analysis_torque: np.ndarray | None = None
        self.analysis_is_nm = True
        self.analysis_label = "(no data loaded)"

        # ── Hardware configuration, mirroring the ESP's runtime config ──
        # Kept in one dict so a profile can round-trip the whole set, and so
        # "send everything" after a reconnect is a single loop.
        self.cfg_vars: dict[str, tk.Variable] = {
            "teeth":        tk.StringVar(value="3"),
            "drive_ratio":  tk.StringVar(value="1.0"),
            "rpm_band_min":  tk.StringVar(value="800"),
            "rpm_band_max":  tk.StringVar(value="6000"),
            "rpm_extrap":     tk.BooleanVar(value=False),
            "rpm_extrap_n":   tk.StringVar(value="4"),
            "rpm_extrap_max": tk.StringVar(value="5"),
            # Brake sweep stall watch
            "char_stop_on_stall": tk.BooleanVar(value=False),
            # Stepper encoder - hardware not fitted yet, see the TBD panel
            "enc_enabled":   tk.BooleanVar(value=False),
            "enc_cpr":       tk.StringVar(value="4000"),
            "enc_invert":    tk.BooleanVar(value=False),
            "rpm_median":    tk.StringVar(value="3"),
            "rpm_ratio":     tk.StringVar(value="3.0"),
            "rpm_slew":      tk.StringVar(value="0"),
            "rpm_avg":       tk.StringVar(value="3"),
            "brake_min":    tk.StringVar(value="0"),
            "brake_max":    tk.StringVar(value=str(BRAKE_FULL_TRAVEL_STEPS)),
            "preload_pct":  tk.StringVar(value="20"),
            "rampdown_mode":  tk.StringVar(value=RAMPDOWN_MODES[1]),
            "rampdown_rate":  tk.StringVar(value="300"),
            "rampdown_brake": tk.StringVar(value="40"),
            "cutoff_rpm":     tk.StringVar(value="1200"),
            "throttle_off":   tk.StringVar(value="50"),
            "stop_rate":      tk.StringVar(value="200"),
            "invert":       tk.BooleanVar(value=True),
            "step_speed":   tk.StringVar(value="10000"),
            "step_accel":   tk.StringVar(value="50000"),
            "cam_model":    tk.StringVar(value=CAM_MODELS[0]),
            "cam_spd":      tk.StringVar(value=str(CAM_STEPS_PER_DEGREE)),
            "cam_lin":      tk.BooleanVar(value=False),
            "cal_scale":    tk.StringVar(value="1.0"),
            "lever_arm":    tk.StringVar(value="0.300"),
            "mech_ratio":   tk.StringVar(value="1.0"),
            "press_off_mv": tk.StringVar(value=str(PRESSURE_ZERO_MV)),
            "press_psi_mv": tk.StringVar(
                value=f"{PRESSURE_DEFAULT_FS_PSI / (PRESSURE_FS_MV - PRESSURE_ZERO_MV):.6f}"),
            "press_fs_psi": tk.StringVar(value=str(int(PRESSURE_DEFAULT_FS_PSI))),
            "press_div":    tk.StringVar(value="1.0"),
            "press_pga":    tk.StringVar(value=PRESS_PGA_RANGES[0]),
            "press_limit":  tk.StringVar(value="1500"),
            "data_dir":     tk.StringVar(value=DEFAULT_DATA_DIR),
            "run_prefix":   tk.StringVar(value="dyno_run"),
            "autosave":     tk.BooleanVar(value=True),
        }
        # Load-cell calibration scratchpad — captured readings, not sent as-is.
        self.calib_vars = {
            "zero_mv":  tk.StringVar(value=""),
            "load_mv":  tk.StringVar(value=""),
            "known_wt": tk.StringVar(value=""),
            "wt_units": tk.StringVar(value="lb"),
        }
        # Two-point pressure calibration scratchpad.
        self.press_cal_vars = {
            "mv1": tk.StringVar(value=""), "psi1": tk.StringVar(value="0"),
            "mv2": tk.StringVar(value=""), "psi2": tk.StringVar(value=""),
        }
        # Measured cam curve: stepper travel % against brake applied %.
        self.cam_rows = [(tk.StringVar(value=""), tk.StringVar(value=""))
                         for _ in range(CAM_TABLE_ROWS)]
        self.profile_path = ""
        # Display zero for the load cell, captured by averaging rather than by
        # trusting one sample of a channel that is full of engine vibration.
        self.load_zero_mv = None
        self._tare_samples = []
        self._tare_collecting = False
        self._brake_slider_last = 0.0
        self.last_saved_run = ""

        self._build_gui()
        # Snapshot the untouched state before anything overwrites it. This is
        # what Restore Defaults puts back, so there is no second copy of the
        # default values to drift out of step with the real ones.
        self._factory_defaults = self._profile_snapshot()
        self._session_written = None
        self._char_active = False
        self._char_rows = []
        self._char_home = 0
        self._char_hi = 0.0
        self._char_phase_sent = None
        self._char_engaged = False
        self._char_base_psi = None
        self._char_susp_since = None
        self._char_stall_at = None
        self._load_session()
        self._schedule_gui_update()
        self._schedule_status_poll()

    # ── GUI Construction ─────────────────────────────────────
    def _build_gui(self):
        top = ttk.Frame(self.root)
        top.pack(side=tk.TOP, fill=tk.X, padx=6, pady=4)
        self._build_connection_bar(top)
        self._build_alert_bar(self.root)

        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=6, pady=4)

        self.live_tab = ttk.Frame(self.notebook)
        self.calib_tab = ttk.Frame(self.notebook)
        self.analysis_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.live_tab, text="  Live Run  ")
        self.notebook.add(self.calib_tab, text="  Calibration & Setup  ")
        self.notebook.add(self.analysis_tab, text="  Analysis & Filtering  ")

        self._build_live_tab(self.live_tab)
        self._build_calib_tab(self.calib_tab)
        self._build_analysis_tab(self.analysis_tab)
        self._update_source_controls()
        # The slider's upper bound comes from the brake range on the other tab,
        # which only exists once both tabs are built.
        self._sync_brake_slider()

    # ── Alert bar (faults and firmware errors) ───────────────
    def _build_alert_bar(self, parent):
        """A single always-visible line for the thing that just went wrong.

        Previously every ERR the firmware sent was discarded, so a refused
        START looked identical to a GUI that had frozen.
        """
        self.alert_frame = tk.Frame(parent, background="#B03A2E")
        self.alert_var = tk.StringVar(value="")
        self.alert_label = tk.Label(
            self.alert_frame, textvariable=self.alert_var,
            background="#B03A2E", foreground="white",
            font=("Helvetica", 11, "bold"), anchor=tk.W, padx=10, pady=5)
        self.alert_label.pack(side=tk.LEFT, fill=tk.X, expand=True)
        tk.Button(self.alert_frame, text="Dismiss", command=self._clear_alert,
                  relief=tk.FLAT).pack(side=tk.RIGHT, padx=6, pady=3)
        # Packed only while there is something to say — see _show_alert().

    def _show_alert(self, text, colour="#B03A2E"):
        self.alert_var.set(text)
        self.alert_frame.configure(background=colour)
        self.alert_label.configure(background=colour)
        if not self.alert_frame.winfo_ismapped():
            self.alert_frame.pack(side=tk.TOP, fill=tk.X, padx=6, pady=(0, 2),
                                  before=self.notebook)

    def _clear_alert(self):
        self.alert_var.set("")
        self.last_error = ""
        if self.alert_frame.winfo_ismapped():
            self.alert_frame.pack_forget()

    def _log_event(self, text, tag="ack"):
        """Record a line for the event log. Safe to call from any thread."""
        self.events.append((tag, f"{time.strftime('%H:%M:%S')}  {text}"))

    # ── Connection bar ───────────────────────────────────────
    def _build_connection_bar(self, parent):
        frame = ttk.LabelFrame(parent, text="Connection")
        frame.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Data source selector
        ttk.Label(frame, text="Source:").pack(side=tk.LEFT, padx=(6, 2))
        ttk.Radiobutton(frame, text="Hardware", variable=self.source_var,
                        value="hardware", command=self._update_source_controls).pack(side=tk.LEFT)
        ttk.Radiobutton(frame, text="CSV Replay", variable=self.source_var,
                        value="replay", command=self._update_source_controls).pack(side=tk.LEFT, padx=(0, 8))

        ttk.Separator(frame, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, pady=2)

        ttk.Label(frame, text="Port:").pack(side=tk.LEFT, padx=4)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, width=18)
        self.port_combo.pack(side=tk.LEFT, padx=2)

        self.refresh_btn = ttk.Button(frame, text="Refresh", command=self._refresh_ports)
        self.refresh_btn.pack(side=tk.LEFT, padx=2)
        self.connect_btn = ttk.Button(frame, text="Connect", command=self._toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=4)

        self.conn_status = ttk.Label(frame, text="Disconnected", foreground="red")
        self.conn_status.pack(side=tk.LEFT, padx=8)

        # Units toggle (affects torque display & analysis)
        ttk.Label(frame, text="Torque units:").pack(side=tk.LEFT, padx=(10, 2))
        units_combo = ttk.Combobox(frame, textvariable=self.units_var, width=6,
                                   values=["Nm", "lb-ft"], state="readonly")
        units_combo.pack(side=tk.LEFT)
        units_combo.bind("<<ComboboxSelected>>", lambda e: self._on_units_changed())

        # Ready / Not Ready indicator
        self.ready_label = ttk.Label(frame, text="NOT READY",
                                     foreground="white", background="red",
                                     font=("Helvetica", 10, "bold"), padding=(8, 2))
        self.ready_label.pack(side=tk.RIGHT, padx=8)

        # SIM / LIVE indicator. This is the single most important thing to know
        # at a glance: in SIM the board produces a completely plausible RPM,
        # torque and brake display with no engine attached.
        self.sim_label = tk.Label(frame, text="MODE ?", foreground="white",
                                  background="#7F8C8D",
                                  font=("Helvetica", 10, "bold"), padx=8, pady=2)
        self.sim_label.pack(side=tk.RIGHT, padx=(8, 2))
        self.sim_btn = ttk.Button(frame, text="Use LIVE", width=11,
                                  command=self._toggle_sim)
        self.sim_btn.pack(side=tk.RIGHT, padx=2)

        self._refresh_ports()

    def _toggle_sim(self):
        """Switch the board between the virtual engine and real sensors."""
        if not (self.ser and self.ser.is_open):
            messagebox.showinfo("Not connected", "Connect to the ESP32 first.")
            return
        going_sim = not self.ready_flags.get("sim", True)
        if going_sim:
            if not messagebox.askyesno(
                    "Switch to SIM?",
                    "SIM mode replaces the real sensors with a virtual engine.\n\n"
                    "Every RPM, torque and pressure reading will be synthetic, "
                    "even though the display looks normal.\n\nSwitch to SIM?"):
                return
        self._send(f"SIM,{1 if going_sim else 0}")
        self.root.after(300, lambda: self._send("READY?"))
        self.root.after(500, lambda: self._send("VERSION"))

    # ── Live tab ─────────────────────────────────────────────
    def _build_live_tab(self, parent):
        # Replay transport bar (active only in CSV Replay mode)
        self.replay_bar = ttk.LabelFrame(parent, text="Dummy Data — CSV Replay")
        self.replay_bar.pack(side=tk.TOP, fill=tk.X, padx=4, pady=(2, 4))

        # Row 1 — transport
        row1 = ttk.Frame(self.replay_bar)
        row1.pack(side=tk.TOP, fill=tk.X)
        ttk.Button(row1, text="Load CSV…",
                   command=self._replay_choose_file).pack(side=tk.LEFT, padx=4, pady=3)
        ttk.Label(row1, textvariable=self.replay_file_var,
                  width=30, anchor=tk.W).pack(side=tk.LEFT, padx=2)
        ttk.Label(row1, text="Speed×:").pack(side=tk.LEFT, padx=(8, 2))
        ttk.Combobox(row1, textvariable=self.replay_speed_var, width=5,
                     values=["0.25", "0.5", "1.0", "2.0", "5.0", "10.0"]).pack(side=tk.LEFT)
        self.replay_play_btn = ttk.Button(row1, text="▶ Play",
                                          command=self._replay_play, width=9)
        self.replay_play_btn.pack(side=tk.LEFT, padx=(8, 2))
        self.replay_pause_btn = ttk.Button(row1, text="❚❚ Pause",
                                           command=self._replay_toggle_pause, width=9,
                                           state=tk.DISABLED)
        self.replay_pause_btn.pack(side=tk.LEFT, padx=2)
        self.replay_stop_btn = ttk.Button(row1, text="■ Stop",
                                          command=self._replay_stop, width=8,
                                          state=tk.DISABLED)
        self.replay_stop_btn.pack(side=tk.LEFT, padx=2)
        ttk.Progressbar(row1, variable=self.replay_progress_var,
                        maximum=100.0, length=160).pack(side=tk.LEFT, padx=8)
        self.replay_info = ttk.Label(row1, text="", foreground="gray")
        self.replay_info.pack(side=tk.LEFT, padx=6)

        # Row 2 — nudge the real brake motor so it reacts to the replayed data
        # (geared way down: the brake follows RPM, scaled, and moves slowly).
        row2 = ttk.Frame(self.replay_bar)
        row2.pack(side=tk.TOP, fill=tk.X, pady=(0, 3))
        ttk.Checkbutton(row2, text="Drive brake motor (gentle)",
                        variable=self.replay_motor_var).pack(side=tk.LEFT, padx=(4, 8))
        ttk.Label(row2, text="ESP port:").pack(side=tk.LEFT)
        self.replay_motor_port_combo = ttk.Combobox(
            row2, textvariable=self.replay_motor_port_var, width=16)
        self.replay_motor_port_combo.pack(side=tk.LEFT, padx=2)
        ttk.Label(row2, text="Travel %:").pack(side=tk.LEFT, padx=(8, 2))
        ttk.Entry(row2, textvariable=self.replay_motor_gain_var, width=5).pack(side=tk.LEFT)
        ttk.Label(row2, text="(brake tracks RPM, scaled — moves slowly)",
                  foreground="gray").pack(side=tk.LEFT, padx=8)
        self._refresh_motor_ports()

        # Event log along the bottom — packed first so it keeps its height when
        # the plot above it expands.
        self._build_event_log(parent)

        # Body: controls (left), plot (center), status (right)
        body = ttk.Frame(parent)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self._build_controls(body)
        self._build_status(body)
        self._build_plot(body)

    # ── Event log ────────────────────────────────────────────
    def _build_event_log(self, parent):
        lf = ttk.LabelFrame(parent, text="Controller Events  (ACK / ERR / CFG)")
        lf.pack(side=tk.BOTTOM, fill=tk.X, padx=4, pady=(2, 4))

        inner = ttk.Frame(lf)
        inner.pack(fill=tk.X, padx=4, pady=3)

        self.event_text = tk.Text(inner, height=4, wrap=tk.NONE,
                                  font=("Consolas", 9), state=tk.DISABLED)
        self.event_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb = ttk.Scrollbar(inner, orient=tk.VERTICAL, command=self.event_text.yview)
        sb.pack(side=tk.LEFT, fill=tk.Y)
        self.event_text.configure(yscrollcommand=sb.set)
        self.event_text.tag_configure("err", foreground="#B03A2E")
        self.event_text.tag_configure("ack", foreground="#1E6B4F")
        self.event_text.tag_configure("cfg", foreground="#1B5E8C")

        btns = ttk.Frame(lf)
        btns.pack(side=tk.BOTTOM, fill=tk.X, padx=4, pady=(0, 3))
        ttk.Button(btns, text="Clear", width=8,
                   command=self._clear_event_log).pack(side=tk.LEFT)
        ttk.Button(btns, text="Read Config (STATUS)", width=20,
                   command=lambda: self._send("STATUS")).pack(side=tk.LEFT, padx=4)
        self.autoscroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(btns, text="Auto-scroll",
                        variable=self.autoscroll_var).pack(side=tk.LEFT, padx=8)

    def _clear_event_log(self):
        self.events.clear()
        self.event_text.configure(state=tk.NORMAL)
        self.event_text.delete("1.0", tk.END)
        self.event_text.configure(state=tk.DISABLED)

    def _drain_events(self):
        """Move queued controller lines into the log widget (main thread)."""
        if not self.events:
            return
        self.event_text.configure(state=tk.NORMAL)
        while self.events:
            tag, line = self.events.popleft()
            self.event_text.insert(tk.END, line + "\n", tag)
        # Keep the widget bounded too, not just the deque.
        excess = int(self.event_text.index("end-1c").split(".")[0]) - EVENT_LOG_LINES
        if excess > 0:
            self.event_text.delete("1.0", f"{excess + 1}.0")
        if self.autoscroll_var.get():
            self.event_text.see(tk.END)
        self.event_text.configure(state=tk.DISABLED)

    # ── Left panel: parameters + controls ────────────────────
    @staticmethod
    def _scrollable(parent, width=None, expand=False):
        """A vertically scrolling column, returning the frame to fill.

        Several of these panels are taller than the window on the Pi's display,
        and a control that sits below the fold with no way to reach it — Save
        CSV, the run folder — may as well not exist.
        """
        outer = ttk.Frame(parent, width=width) if width else ttk.Frame(parent)
        outer.pack(side=tk.LEFT, fill=tk.BOTH if expand else tk.Y,
                   expand=expand, padx=(0, 6))
        if width:
            outer.pack_propagate(False)

        canvas = tk.Canvas(outer, highlightthickness=0, borderwidth=0)
        vsb = ttk.Scrollbar(outer, orient=tk.VERTICAL, command=canvas.yview)
        canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        inner = ttk.Frame(canvas)
        window = canvas.create_window((0, 0), window=inner, anchor="nw")
        inner.bind("<Configure>",
                   lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>",
                    lambda e: canvas.itemconfigure(window, width=e.width))

        # Scroll whichever pane the pointer is over. The wheel bindings are
        # global while the pointer is inside, so several scrollers can coexist
        # without the last one built capturing the wheel for the whole window.
        # X11 sends buttons 4/5 rather than <MouseWheel>.
        def _scroll(delta):
            canvas.yview_scroll(delta, "units")

        def _enter(_e):
            canvas.bind_all("<Button-4>", lambda e: _scroll(-2))
            canvas.bind_all("<Button-5>", lambda e: _scroll(2))
            canvas.bind_all("<MouseWheel>",
                            lambda e: _scroll(-2 if e.delta > 0 else 2))

        def _leave(_e):
            for seq in ("<Button-4>", "<Button-5>", "<MouseWheel>"):
                canvas.unbind_all(seq)

        canvas.bind("<Enter>", _enter)
        canvas.bind("<Leave>", _leave)
        return inner

    def _build_controls(self, parent):
        left = self._scrollable(parent, width=348)

        # -- Parameters --
        pf = ttk.LabelFrame(left, text="Run Parameters")
        pf.pack(fill=tk.X, pady=2)

        params = [
            ("Hold RPM:",      "hold_rpm",  "2000"),
            ("Start RPM:",     "start_rpm", "2000"),
            ("End RPM:",       "end_rpm",   "5500"),
            ("Rate (RPM/s):",  "rate",      "500"),
        ]
        self.param_vars: dict[str, tk.StringVar] = {}
        for label_text, key, default in params:
            row = ttk.Frame(pf)
            row.pack(fill=tk.X, padx=4, pady=1)
            ttk.Label(row, text=label_text, width=14, anchor=tk.W).pack(side=tk.LEFT)
            var = tk.StringVar(value=default)
            self.param_vars[key] = var
            ttk.Entry(row, textvariable=var, width=10).pack(side=tk.LEFT)

        ttk.Button(pf, text="Send Parameters", command=self._send_params).pack(padx=4, pady=4)

        # -- End of run --
        # Kept on the run tab, not buried in setup: this is something to adjust
        # between pulls while the engine is still warm.
        rdf = ttk.LabelFrame(left, text="End of Run  (throttle-off behaviour)")
        rdf.pack(fill=tk.X, pady=2)

        row = ttk.Frame(rdf)
        row.pack(fill=tk.X, padx=4, pady=2)
        ttk.Label(row, text="On lift:", width=10, anchor=tk.W).pack(side=tk.LEFT)
        ttk.Combobox(row, textvariable=self.cfg_vars["rampdown_mode"], width=19,
                     state="readonly", values=RAMPDOWN_MODES).pack(side=tk.LEFT)

        for label_text, key, suffix in [
                ("Ramp rate:",   "rampdown_rate",  "RPM/s down"),
                ("Brake off at:", "rampdown_brake", "% travel/s"),
                ("Cutoff RPM:",  "cutoff_rpm",     "brake fully off below"),
                ("Lift detect:", "throttle_off",   "% torque drop"),
                ("STOP release:", "stop_rate",     "% travel/s (safety stop)")]:
            r = ttk.Frame(rdf)
            r.pack(fill=tk.X, padx=4, pady=1)
            ttk.Label(r, text=label_text, width=12, anchor=tk.W).pack(side=tk.LEFT)
            ttk.Entry(r, textvariable=self.cfg_vars[key], width=8).pack(side=tk.LEFT)
            ttk.Label(r, text=suffix, foreground="gray").pack(side=tk.LEFT, padx=4)

        ttk.Label(rdf, wraplength=300, justify=tk.LEFT, foreground="gray",
                  text=("Ramp rate applies to the controlled mode: the brake holds "
                        "the engine to a descending target, and lets go by itself if "
                        "the engine falls faster than that. Below the cutoff the "
                        "brake always releases fully, whatever the mode.\n\n"
                        "Set the cutoff ABOVE idle. If it sits below idle the engine "
                        "settles above it, the ramp keeps pulling it down, and the "
                        "brake never lets go. STOP always walks the brake off at "
                        "its own rate, whatever mode is selected.")
                  ).pack(anchor=tk.W, padx=4, pady=(2, 3))

        rrow = ttk.Frame(rdf)
        rrow.pack(fill=tk.X, padx=4, pady=4)
        ttk.Button(rrow, text="Send", command=self._send_rampdown_cfg).pack(side=tk.LEFT)
        ttk.Button(rrow, text="Test ramp-down now", command=self._force_rampdown
                   ).pack(side=tk.LEFT, padx=6)

        # -- PID: two gain sets --
        # Hold gains catch the engine while the operator opens the throttle;
        # sweep gains are deliberately softer because the hydraulic brake is
        # near-instantaneous and a stiff loop just oscillates.
        pidf = ttk.LabelFrame(left, text="PID Tuning  (Hold / Sweep)")
        pidf.pack(fill=tk.X, pady=2)

        hdr = ttk.Frame(pidf)
        hdr.pack(fill=tk.X, padx=4, pady=(2, 0))
        ttk.Label(hdr, text="", width=6).pack(side=tk.LEFT)
        ttk.Label(hdr, text="Hold", width=9, anchor=tk.CENTER,
                  font=("Helvetica", 9, "bold")).pack(side=tk.LEFT)
        ttk.Label(hdr, text="Sweep", width=9, anchor=tk.CENTER,
                  font=("Helvetica", 9, "bold")).pack(side=tk.LEFT)

        # Kp is MICROSTEPS of brake per RPM of error, tied to the 6250-microstep
        # travel: 6.25 reaches full brake at about 1000 RPM of error. Starting
        # points to tune from, not tuned values.
        self.pid_vars: dict[str, tk.StringVar] = {}
        self.pid_sweep_vars: dict[str, tk.StringVar] = {}
        for label_text, key, hold_default, sweep_default in [
                ("Kp:", "kp", "6.25", "3.75"),
                ("Ki:", "ki", "10.0", "6.25"),
                ("Kd:", "kd", "0.25", "0.125")]:
            row = ttk.Frame(pidf)
            row.pack(fill=tk.X, padx=4, pady=1)
            ttk.Label(row, text=label_text, width=6, anchor=tk.W).pack(side=tk.LEFT)
            hv = tk.StringVar(value=hold_default)
            sv = tk.StringVar(value=sweep_default)
            self.pid_vars[key] = hv
            self.pid_sweep_vars[key] = sv
            ttk.Entry(row, textvariable=hv, width=8).pack(side=tk.LEFT, padx=1)
            ttk.Entry(row, textvariable=sv, width=8).pack(side=tk.LEFT, padx=1)

        ttk.Button(pidf, text="Apply PID", command=self._apply_pid).pack(padx=4, pady=4)

        # -- Manual Brake --
        bf = ttk.LabelFrame(left, text="Manual Brake")
        bf.pack(fill=tk.X, pady=2)

        row = ttk.Frame(bf)
        row.pack(fill=tk.X, padx=4, pady=1)
        ttk.Label(row, text="Position:", width=14, anchor=tk.W).pack(side=tk.LEFT)
        self.brake_var = tk.StringVar(value="0")
        ttk.Entry(row, textvariable=self.brake_var, width=10).pack(side=tk.LEFT)
        ttk.Button(row, text="Go", command=self._manual_brake).pack(side=tk.LEFT, padx=4)

        # Drag-to-test slider: the quickest way to confirm the actuator moves,
        # how fast, and which way — which is what it is for on the reference
        # system. Commands are throttled while dragging and the final position
        # is always sent on release, so the brake can't be left mid-drag.
        self.brake_slider_var = tk.DoubleVar(value=0.0)
        self.brake_slider = ttk.Scale(bf, from_=0, to=5000, orient=tk.HORIZONTAL,
                                      variable=self.brake_slider_var,
                                      command=self._on_brake_slider)
        self.brake_slider.pack(fill=tk.X, padx=6, pady=(6, 0))
        srow = ttk.Frame(bf)
        srow.pack(fill=tk.X, padx=6, pady=(0, 5))
        ttk.Label(srow, text="0", foreground="gray").pack(side=tk.LEFT)
        self.brake_slider_max_label = ttk.Label(srow, text="5000", foreground="gray")
        self.brake_slider_max_label.pack(side=tk.RIGHT)
        self.brake_slider_label = ttk.Label(srow, text="0 steps · 0%",
                                            anchor=tk.CENTER)
        self.brake_slider_label.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.brake_slider.bind("<ButtonRelease-1>",
                               lambda e: self._brake_slider_commit())

        # -- Control Buttons (operator procedure) --
        cf = ttk.LabelFrame(left, text="Run Controls")
        cf.pack(fill=tk.X, pady=2)

        btn_grid = ttk.Frame(cf)
        btn_grid.pack(padx=4, pady=4, fill=tk.X)

        ttk.Button(btn_grid, text="Home", command=self._on_home, width=14).grid(
            row=0, column=0, padx=3, pady=2)
        ttk.Button(btn_grid, text="Tare", command=self._on_tare, width=14).grid(
            row=0, column=1, padx=3, pady=2)

        self.start_btn = ttk.Button(btn_grid, text="START", command=self._on_start, width=14)
        self.start_btn.grid(row=1, column=0, padx=3, pady=2)

        self.release_btn = ttk.Button(btn_grid, text="RELEASE", command=self._on_release,
                                       width=14, state=tk.DISABLED)
        self.release_btn.grid(row=1, column=1, padx=3, pady=2)

        stop_btn = ttk.Button(btn_grid, text="STOP", command=self._on_stop, width=30)
        stop_btn.grid(row=2, column=0, columnspan=2, padx=3, pady=4)
        stop_style = ttk.Style()
        stop_style.configure("Stop.TButton", foreground="red")
        stop_btn.configure(style="Stop.TButton")

        self.run_status_var = tk.StringVar(value="Idle")
        self.run_status_label = ttk.Label(cf, textvariable=self.run_status_var,
                                           font=("Helvetica", 11, "bold"), anchor=tk.CENTER)
        self.run_status_label.pack(padx=4, pady=4)

        # -- Live display smoothing --
        lsf = ttk.LabelFrame(left, text="Live Display Smoothing")
        lsf.pack(fill=tk.X, pady=2)
        row = ttk.Frame(lsf)
        row.pack(fill=tk.X, padx=4, pady=2)
        ttk.Checkbutton(row, text="EMA smoothing", variable=self.live_smooth_var,
                        command=self._mark_plot_dirty).pack(side=tk.LEFT)
        ttk.Label(row, text="α:").pack(side=tk.LEFT, padx=(8, 2))
        alpha_e = ttk.Entry(row, textvariable=self.live_alpha_var, width=6)
        alpha_e.pack(side=tk.LEFT)
        alpha_e.bind("<Return>", lambda e: self._mark_plot_dirty())

        ttk.Label(lsf, text="Traces shown:", foreground="gray").pack(
            anchor=tk.W, padx=4, pady=(4, 0))
        trow1 = ttk.Frame(lsf); trow1.pack(fill=tk.X, padx=4)
        trow2 = ttk.Frame(lsf); trow2.pack(fill=tk.X, padx=4, pady=(0, 4))
        for parent, key, label in ((trow1, "rpm", "RPM"),
                                   (trow1, "torque", "Torque"),
                                   (trow1, "hp", "HP"),
                                   (trow2, "psi", "Brake PSI"),
                                   (trow2, "brake", "Brake pos")):
            ttk.Checkbutton(parent, text=label, variable=self.trace_vars[key],
                            command=self._mark_plot_dirty).pack(side=tk.LEFT, padx=(0, 8))

        # -- Data --
        df = ttk.LabelFrame(left, text="Data")
        df.pack(fill=tk.X, pady=2)

        data_row = ttk.Frame(df)
        data_row.pack(padx=4, pady=4)
        self.rec_btn = ttk.Button(data_row, text="● Record", command=self._toggle_record, width=12)
        self.rec_btn.pack(side=tk.LEFT, padx=2)
        ttk.Button(data_row, text="Clear Plot", command=self._clear_plot, width=12).pack(
            side=tk.LEFT, padx=2)

        data_row2 = ttk.Frame(df)
        data_row2.pack(padx=4, pady=2)
        ttk.Button(data_row2, text="Save CSV", command=self._save_csv, width=12).pack(
            side=tk.LEFT, padx=2)
        ttk.Button(data_row2, text="Send to Analysis →", command=self._send_run_to_analysis,
                   width=18).pack(side=tk.LEFT, padx=2)

        # Back-to-back pulls: bank this one and clear down for the next.
        data_row3 = ttk.Frame(df)
        data_row3.pack(padx=4, pady=2)
        ttk.Button(data_row3, text="Save & New Run", command=self._save_and_restart,
                   width=28).pack(side=tk.LEFT, padx=2)
        self.saved_label = ttk.Label(df, text="", foreground="gray",
                                     wraplength=300, anchor=tk.W)
        self.saved_label.pack(fill=tk.X, padx=6, pady=(0, 4))

    # ── Right panel: live status ─────────────────────────────
    def _build_status(self, parent):
        sf = ttk.LabelFrame(parent, text="Live Status", width=210)
        sf.pack(side=tk.RIGHT, fill=tk.Y, padx=(6, 0))
        sf.pack_propagate(False)

        self.status_labels: dict[str, ttk.Label] = {}
        items = [
            ("RPM",         "rpm",        "0"),
            ("Target RPM",  "target_rpm", "0"),
            ("Torque",      "torque",     "0.00"),
            ("HP",          "hp",         "0.0"),
            ("Brake %",     "brake_pct",  "0.0"),
            ("Brake Pos",   "brake_pos",  "0"),
            ("Brake PSI",   "press_psi",  "0.0"),
            ("State",       "state",      "IDLE"),
            ("Tach glitches","glitches",  "0"),
            ("RPM estimated","estimated", "0"),
            ("Firmware",     "fw_version","unknown"),
            ("Load mV",     "load_raw",   "0.0"),
            ("Load net mV", "load_net",   "--"),
            ("Pressure mV", "press_mv",   "0.0"),
        ]
        self.torque_status_row = None
        for label_text, key, default in items:
            row = ttk.Frame(sf)
            row.pack(fill=tk.X, padx=6, pady=3)
            lbl_txt = ttk.Label(row, text=label_text + ":", anchor=tk.W, width=12)
            lbl_txt.pack(side=tk.LEFT)
            if key == "torque":
                self.torque_label_text = lbl_txt
            lbl = ttk.Label(row, text=default, anchor=tk.E, width=10,
                            font=("Consolas", 11, "bold"))
            lbl.pack(side=tk.RIGHT)
            self.status_labels[key] = lbl

        # -- Readiness checklist --
        rf = ttk.LabelFrame(sf, text="Readiness")
        rf.pack(fill=tk.X, padx=6, pady=6)
        self.ready_checks: dict[str, ttk.Label] = {}
        for key, label_text in [("homed", "Homed"), ("tared", "Tared"),
                                 ("adc", "ADC / Load Cell"),
                                 ("press_adc", "Pressure ADC")]:
            row = ttk.Frame(rf)
            row.pack(fill=tk.X, padx=4, pady=1)
            ttk.Label(row, text=label_text + ":", width=12, anchor=tk.W).pack(side=tk.LEFT)
            lbl = ttk.Label(row, text="✗", foreground="red", width=4,
                            font=("Consolas", 11, "bold"))
            lbl.pack(side=tk.RIGHT)
            self.ready_checks[key] = lbl

        # -- PID debug --
        pf = ttk.LabelFrame(sf, text="PID Output")
        pf.pack(fill=tk.X, padx=6, pady=6)
        self.pid_labels: dict[str, ttk.Label] = {}
        for key, label_text in [("pid_p", "P term"), ("pid_i", "I term"), ("pid_out", "Output")]:
            row = ttk.Frame(pf)
            row.pack(fill=tk.X, padx=4, pady=1)
            ttk.Label(row, text=label_text + ":", width=10, anchor=tk.W).pack(side=tk.LEFT)
            lbl = ttk.Label(row, text="0.0", anchor=tk.E, width=8, font=("Consolas", 10))
            lbl.pack(side=tk.RIGHT)
            self.pid_labels[key] = lbl

    # ── Plot area (Live tab) ─────────────────────────────────
    def _build_plot(self, parent):
        # Live view is a TIME series: x = time, left y = RPM, right y = Torque & HP.
        self.fig = Figure(figsize=(7, 4), dpi=100)
        self.ax_rpm = self.fig.add_subplot(111)     # left axis: RPM
        self.ax_pwr = self.ax_rpm.twinx()           # right axis: Torque + HP
        # Pressure gets its own axis rather than sharing: 0-2000 PSI would
        # swamp a torque scale and make the torque trace unreadable.
        self.ax_psi = self.ax_rpm.twinx()
        self.ax_psi.spines["right"].set_position(("outward", 46))
        # Stepper position, in steps. Its own scale again: 0-250 steps against a
        # 7000 RPM axis would be a flat line at the bottom.
        self.ax_brk = self.ax_rpm.twinx()
        self.ax_brk.spines["right"].set_position(("outward", 104))

        self.ax_rpm.set_xlabel("Time (s)")
        self.ax_rpm.set_ylabel("RPM", color="tab:green")
        self.ax_pwr.set_ylabel(f"Torque ({self.units_var.get()}) / HP", color="tab:red")
        self.ax_psi.set_ylabel("Brake PSI", color="tab:purple")
        self.ax_psi.tick_params(axis="y", labelcolor="tab:purple", labelsize=8)
        self.ax_brk.set_ylabel("Brake position (steps)", color="tab:orange")
        self.ax_brk.tick_params(axis="y", labelcolor="tab:orange", labelsize=8)
        self.ax_rpm.set_title("Live — RPM, Torque, HP, PSI & Brake vs Time")
        self.ax_rpm.grid(True, alpha=0.3)

        # Faint raw traces (shown when live smoothing is on)
        self.line_rpm_raw, = self.ax_rpm.plot([], [], "-", color="tab:green",
                                              alpha=0.25, linewidth=0.8)
        self.line_torque_raw, = self.ax_pwr.plot([], [], "-", color="tab:blue",
                                                 alpha=0.25, linewidth=0.8)
        self.line_hp_raw, = self.ax_pwr.plot([], [], "-", color="tab:red",
                                             alpha=0.25, linewidth=0.8)
        # Bold (smoothed) traces
        self.line_rpm, = self.ax_rpm.plot([], [], "-", color="tab:green",
                                          linewidth=1.4, label="RPM")
        self.line_torque, = self.ax_pwr.plot([], [], "-", color="tab:blue",
                                             linewidth=1.4, label="Torque")
        self.line_hp, = self.ax_pwr.plot([], [], "-", color="tab:red",
                                         linewidth=1.4, label="HP")
        self.line_psi_raw, = self.ax_psi.plot([], [], "-", color="tab:purple",
                                              alpha=0.25, linewidth=0.8)
        self.line_psi, = self.ax_psi.plot([], [], "-", color="tab:purple",
                                          linewidth=1.4, label="Brake PSI")
        self.line_brake_raw, = self.ax_brk.plot([], [], "-", color="tab:orange",
                                                alpha=0.25, linewidth=0.8)
        self.line_brake, = self.ax_brk.plot([], [], "-", color="tab:orange",
                                            linewidth=1.4, label="Brake pos")

        lines = [self.line_rpm, self.line_torque, self.line_hp, self.line_psi,
                 self.line_brake]
        self.ax_rpm.legend(lines, [l.get_label() for l in lines], loc="upper left")
        self.fig.tight_layout()
        # tight_layout does not account for the offset spine, and 0.86 clipped
        # both the top PSI tick and its axis label off the canvas.
        self.fig.subplots_adjust(right=0.74)

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        # Draw once up front so the empty axes, grid and labels are visible
        # before the first sample arrives, instead of a blank white panel.
        self.canvas.draw_idle()
        # Invalidate the blit background on a real resize so it is recaptured at
        # the new size on the next redraw.
        self.canvas.get_tk_widget().bind("<Configure>", self._on_canvas_resize)

    # ══════════════════════════════════════════════════════════
    # Calibration & Setup tab
    # ══════════════════════════════════════════════════════════
    @staticmethod
    def _labelled_entry(parent, text, var, width=10, label_width=20, suffix=""):
        row = ttk.Frame(parent)
        row.pack(fill=tk.X, padx=4, pady=1)
        ttk.Label(row, text=text, width=label_width, anchor=tk.W).pack(side=tk.LEFT)
        e = ttk.Entry(row, textvariable=var, width=width)
        e.pack(side=tk.LEFT)
        if suffix:
            ttk.Label(row, text=suffix, foreground="gray").pack(side=tk.LEFT, padx=3)
        return row

    def _build_calib_tab(self, parent):
        # Both columns scroll: the right one in particular runs past the bottom
        # of the window once pressure and run storage are on it.
        left = self._scrollable(parent, expand=True)
        right = self._scrollable(parent, expand=True)

        # ── Live readings (needed while calibrating) ──────────
        lr = ttk.LabelFrame(left, text="Live readings")
        lr.pack(fill=tk.X, pady=2)
        self.calib_live_labels = {}
        for text, key in [("Load cell (mV):", "load_raw"),
                          ("Load cell, tared (mV):", "load_net"),
                          ("Pressure sensor (mV):", "press_mv"),
                          ("Brake pressure (PSI):", "press_psi"),
                          ("Torque (current units):", "torque")]:
            row = ttk.Frame(lr)
            row.pack(fill=tk.X, padx=4, pady=2)
            ttk.Label(row, text=text, width=24, anchor=tk.W).pack(side=tk.LEFT)
            lbl = ttk.Label(row, text="—", width=12, anchor=tk.E,
                            font=("Consolas", 11, "bold"))
            lbl.pack(side=tk.LEFT)
            self.calib_live_labels[key] = lbl
        trow = ttk.Frame(lr)
        trow.pack(fill=tk.X, padx=4, pady=(4, 2))
        ttk.Button(trow, text="Tare (zero the load cell)",
                   command=self._on_tare).pack(side=tk.LEFT)
        self.tare_label = ttk.Label(trow, text="not zeroed", foreground="gray")
        self.tare_label.pack(side=tk.LEFT, padx=8)
        ttk.Label(lr, wraplength=380, justify=tk.LEFT, foreground="gray",
                  text=("Averages about a second of samples rather than grabbing "
                        "one, and reports the noise it saw - which is a direct read "
                        "on how much vibration the cell is picking up. The raw mV "
                        "above does not change when you tare; the tared value does.")
                  ).pack(anchor=tk.W, padx=4, pady=(0, 3))

        # ── Load cell calibration ─────────────────────────────
        lc = ttk.LabelFrame(left, text="Load cell — volts to torque")
        lc.pack(fill=tk.X, pady=4)
        ttk.Label(lc, wraplength=380, justify=tk.LEFT, foreground="gray",
                  text=("Hang a known weight from the lever arm, capture the reading "
                        "before and after, and the scale is derived from the "
                        "difference. The mechanical factor is separate — see below.")
                  ).pack(anchor=tk.W, padx=4, pady=(3, 5))

        r = self._labelled_entry(lc, "1. Zero reading:", self.calib_vars["zero_mv"],
                                 label_width=18, suffix="mV")
        ttk.Button(r, text="Capture", width=9,
                   command=lambda: self._capture_mv("zero_mv")).pack(side=tk.LEFT, padx=6)

        self._labelled_entry(lc, "2. Known weight:", self.calib_vars["known_wt"],
                             label_width=18)
        row = ttk.Frame(lc)
        row.pack(fill=tk.X, padx=4, pady=1)
        ttk.Label(row, text="   units:", width=18, anchor=tk.W).pack(side=tk.LEFT)
        ttk.Combobox(row, textvariable=self.calib_vars["wt_units"], width=8,
                     state="readonly", values=["lb", "kg", "N"]).pack(side=tk.LEFT)

        r = self._labelled_entry(lc, "3. Loaded reading:", self.calib_vars["load_mv"],
                                 label_width=18, suffix="mV")
        ttk.Button(r, text="Capture", width=9,
                   command=lambda: self._capture_mv("load_mv")).pack(side=tk.LEFT, padx=6)

        self._labelled_entry(lc, "4. Lever arm:", self.cfg_vars["lever_arm"],
                             label_width=18, suffix="m")
        ttk.Button(lc, text="Compute scale and send",
                   command=self._compute_load_cell).pack(padx=4, pady=5, anchor=tk.W)

        ttk.Separator(lc, orient=tk.HORIZONTAL).pack(fill=tk.X, padx=4, pady=4)
        self._labelled_entry(lc, "Scale (N per mV):", self.cfg_vars["cal_scale"],
                             label_width=18)
        self._labelled_entry(lc, "Mechanical factor:", self.cfg_vars["mech_ratio"],
                             label_width=18, suffix="×")
        ttk.Label(lc, wraplength=380, justify=tk.LEFT, foreground="gray",
                  text=("Mechanical factor corrects for the 3-point mount: the cell "
                        "only carries part of the reaction force, so a reading of "
                        "one third of true load needs a factor of 3.")
                  ).pack(anchor=tk.W, padx=4, pady=(0, 4))
        ttk.Button(lc, text="Send load cell settings",
                   command=self._send_load_cell_cfg).pack(padx=4, pady=4, anchor=tk.W)

        # ── Trigger wheel and RPM conditioning ────────────────
        tw = ttk.LabelFrame(right, text="RPM pickup")
        tw.pack(fill=tk.X, pady=2)
        self._labelled_entry(tw, "Pulses per revolution:", self.cfg_vars["teeth"],
                             label_width=22)
        self._labelled_entry(tw, "Drive ratio:", self.cfg_vars["drive_ratio"],
                             label_width=22, suffix="engine rev / sensor rev")

        row = ttk.Frame(tw)
        row.pack(fill=tk.X, padx=4, pady=1)
        ttk.Label(row, text="Valid RPM band:", width=22, anchor=tk.W).pack(side=tk.LEFT)
        ttk.Entry(row, textvariable=self.cfg_vars["rpm_band_min"], width=7).pack(side=tk.LEFT)
        ttk.Label(row, text="to").pack(side=tk.LEFT, padx=3)
        ttk.Entry(row, textvariable=self.cfg_vars["rpm_band_max"], width=7).pack(side=tk.LEFT)
        ttk.Label(row, text="outside = ignored", foreground="gray").pack(side=tk.LEFT, padx=4)

        erow = ttk.Frame(tw)
        erow.pack(fill=tk.X, padx=4, pady=1)
        ttk.Checkbutton(erow, text="Estimate through bad readings",
                        variable=self.cfg_vars["rpm_extrap"]).pack(side=tk.LEFT)
        erow2 = ttk.Frame(tw)
        erow2.pack(fill=tk.X, padx=4, pady=1)
        ttk.Label(erow2, text="  fit points / max run:", width=22,
                  anchor=tk.W).pack(side=tk.LEFT)
        ttk.Entry(erow2, textvariable=self.cfg_vars["rpm_extrap_n"],
                  width=5).pack(side=tk.LEFT)
        ttk.Label(erow2, text="/").pack(side=tk.LEFT, padx=3)
        ttk.Entry(erow2, textvariable=self.cfg_vars["rpm_extrap_max"],
                  width=5).pack(side=tk.LEFT)
        ttk.Label(tw, wraplength=380, justify=tk.LEFT, foreground="gray",
                  text=("Off, an out-of-band reading is dropped and the last good "
                        "value stands until the next real pulse, which flat-spots "
                        "the trace during a sweep. On, the board projects the "
                        "trend through the gap instead. It gives up after the max "
                        "run and holds - a straight line carried through a long "
                        "burst would invent a runaway, and the brake acts on what "
                        "it is told. Watch RPM estimated to see how much of a run "
                        "was filled in.")
                  ).pack(anchor=tk.W, padx=4, pady=(2, 4))

        row = ttk.Frame(tw)
        row.pack(fill=tk.X, padx=4, pady=1)
        ttk.Label(row, text="Median window:", width=22, anchor=tk.W).pack(side=tk.LEFT)
        ttk.Combobox(row, textvariable=self.cfg_vars["rpm_median"], width=8,
                     state="readonly", values=["1", "3", "5", "7"]).pack(side=tk.LEFT)
        ttk.Label(row, text="pulses (1 = off)", foreground="gray").pack(side=tk.LEFT, padx=4)

        self._labelled_entry(tw, "Ratio gate:", self.cfg_vars["rpm_ratio"],
                             label_width=22, suffix="x away = reject (0 = off)")
        self._labelled_entry(tw, "Slew limit:", self.cfg_vars["rpm_slew"],
                             label_width=22, suffix="RPM/s (0 = off)")
        self._labelled_entry(tw, "Average over:", self.cfg_vars["rpm_avg"],
                             label_width=22, suffix="pulses, max 500")
        # This average feeds the PID, so a long window is not free. Showing
        # the lag in seconds makes the cost visible where the choice is made.
        self.rpm_lag_label = ttk.Label(tw, text="", foreground="gray")
        self.rpm_lag_label.pack(anchor=tk.W, padx=(26, 4))
        ttk.Label(tw, wraplength=380, justify=tk.LEFT, foreground="gray",
                  text=("The band is the important one: a reading outside it is "
                        "discarded rather than averaged in, and its upper end also "
                        "sets the interrupt threshold that throws away impossibly "
                        "fast edges before the control loop can answer them. Watch "
                        "Tach glitches on the Live Run tab - if it climbs, the "
                        "pickup line is picking up electrical noise. The median and "
                        "ratio gates catch what gets past it; averaging is last and "
                        "adds lag to the loop.")
                  ).pack(anchor=tk.W, padx=4, pady=(2, 3))

        ttk.Button(tw, text="Send", command=self._send_rpm_cfg).pack(
            padx=4, pady=4, anchor=tk.W)

        # ── Brake actuator ────────────────────────────────────
        bk = ttk.LabelFrame(right, text="Brake actuator")
        bk.pack(fill=tk.X, pady=4)
        self._labelled_entry(bk, "Range min:", self.cfg_vars["brake_min"],
                             label_width=22, suffix="steps")
        self._labelled_entry(bk, "Range max:", self.cfg_vars["brake_max"],
                             label_width=22, suffix="steps")
        self._labelled_entry(bk, "Preload at START:", self.cfg_vars["preload_pct"],
                             label_width=22, suffix="% of range")
        self._labelled_entry(bk, "Max speed:", self.cfg_vars["step_speed"],
                             label_width=22, suffix="steps/s")
        self._labelled_entry(bk, "Acceleration:", self.cfg_vars["step_accel"],
                             label_width=22, suffix="steps/s²")
        ttk.Checkbutton(bk, text="Invert direction (planetary gearbox)",
                        variable=self.cfg_vars["invert"]).pack(anchor=tk.W, padx=4, pady=2)
        ttk.Label(bk, wraplength=380, justify=tk.LEFT, foreground="gray",
                  text=("Range max is an over-stroke stop: past full travel the cam "
                        "goes back over centre and can break the linkage.")
                  ).pack(anchor=tk.W, padx=4, pady=(0, 3))
        ttk.Button(bk, text="Send", command=self._send_brake_cfg).pack(
            padx=4, pady=4, anchor=tk.W)

        enc = ttk.LabelFrame(bk, text="Stepper encoder  -  TBD, not yet fitted")
        enc.pack(fill=tk.X, padx=4, pady=(6, 2))
        ttk.Label(enc, wraplength=370, justify=tk.LEFT, foreground="#B9770E",
                  text=("PLACEHOLDER. No encoder is installed and nothing here "
                        "does anything yet. The firmware accepts and reports "
                        "these settings so the plumbing is already in place, "
                        "but position remains open-loop: the controller "
                        "commands steps and assumes they arrive. When the "
                        "encoder is fitted this is where it gets configured, "
                        "and the stall watch below can be replaced by a direct "
                        "measurement of where the shaft actually is.")
                  ).pack(anchor=tk.W, padx=4, pady=2)
        self.enc_chk = ttk.Checkbutton(
            enc, text="Encoder fitted", variable=self.cfg_vars["enc_enabled"],
            command=self._send_encoder_cfg)
        self.enc_chk.pack(anchor=tk.W, padx=4)
        enrow = ttk.Frame(enc)
        enrow.pack(fill=tk.X, padx=4, pady=1)
        ttk.Label(enrow, text="Counts per rev:", width=18,
                  anchor=tk.W).pack(side=tk.LEFT)
        ttk.Entry(enrow, textvariable=self.cfg_vars["enc_cpr"],
                  width=8).pack(side=tk.LEFT)
        ttk.Checkbutton(enrow, text="reverse",
                        variable=self.cfg_vars["enc_invert"]).pack(
                            side=tk.LEFT, padx=6)
        esrow = ttk.Frame(enc)
        esrow.pack(fill=tk.X, padx=4, pady=(1, 4))
        ttk.Label(esrow, text="Encoder position:", width=18,
                  anchor=tk.W).pack(side=tk.LEFT)
        self.enc_pos_label = ttk.Label(esrow, text="not installed",
                                       foreground="gray")
        self.enc_pos_label.pack(side=tk.LEFT)
        ttk.Button(enc, text="Send encoder settings", width=22,
                   command=self._send_encoder_cfg).pack(anchor=tk.W,
                                                        padx=4, pady=(0, 4))

        ttk.Separator(bk, orient=tk.HORIZONTAL).pack(fill=tk.X, padx=4, pady=4)
        crow = ttk.Frame(bk)
        crow.pack(fill=tk.X, padx=4, pady=(0, 2))
        self.char_btn = ttk.Button(crow, text="Characterise brake",
                                   command=self._start_brake_char, width=20)
        self.char_btn.pack(side=tk.LEFT)
        self.char_progress = ttk.Label(crow, text="", foreground="gray")
        self.char_progress.pack(side=tk.LEFT, padx=8)
        self.char_stall_label = ttk.Label(bk, text="", foreground="#B03A2E")
        self.char_stall_label.pack(anchor=tk.W, padx=4)
        ttk.Checkbutton(bk, text="Stop the sweep if a stall is suspected",
                        variable=self.cfg_vars["char_stop_on_stall"]).pack(
                            anchor=tk.W, padx=4)
        ttk.Label(bk, wraplength=380, justify=tk.LEFT, foreground="gray",
                  text=("With no encoder fitted, pressure is the only witness "
                        "that the motor went where it was told. Once the brake "
                        "is engaged, commanded position rising while pressure "
                        "stays flat is flagged as a possible stall - the takeup "
                        "travel before the pads bite is ignored, since no "
                        "pressure is expected there. A brake already fully "
                        "applied looks the same, so this is a suspicion to "
                        "check, not a verdict.")
                  ).pack(anchor=tk.W, padx=4, pady=(0, 4))
        ttk.Label(bk, wraplength=380, justify=tk.LEFT, foreground="gray",
                  text=("Homes first, then walks the brake from home to full travel "
                        "over 10 s, holds, and returns - recording position against "
                        "line pressure the whole way. Saves a CSV and a plot showing "
                        "where position actually starts making pressure, and how far "
                        "the return leg lags the outward one. Engine must be stopped.")
                  ).pack(anchor=tk.W, padx=4, pady=(0, 4))

        # ── Cam geometry ──────────────────────────────────────
        cm = ttk.LabelFrame(right, text="Cam geometry — position to brake applied")
        cm.pack(fill=tk.X, pady=4)
        ttk.Label(cm, wraplength=380, justify=tk.LEFT, foreground="gray",
                  text=("The pusher is cam driven through the gearbox, so brake "
                        "effort is not linear in stepper steps — the same 100 steps "
                        "near the base circle and near full lift do very different "
                        "things. This is what turns position into the brake % you "
                        "read.")).pack(anchor=tk.W, padx=4, pady=(3, 5))

        row = ttk.Frame(cm)
        row.pack(fill=tk.X, padx=4, pady=1)
        ttk.Label(row, text="Model:", width=22, anchor=tk.W).pack(side=tk.LEFT)
        ttk.Combobox(row, textvariable=self.cfg_vars["cam_model"], width=22,
                     state="readonly", values=CAM_MODELS).pack(side=tk.LEFT)

        self._labelled_entry(cm, "Steps per degree:", self.cfg_vars["cam_spd"],
                             label_width=22, suffix="cam rotation")
        ttk.Label(cm, wraplength=380, justify=tk.LEFT, foreground="gray",
                  text=("Eccentric cam assumes lift follows (1 − cos θ) and needs "
                        "steps per degree to know how much of the cam the travel "
                        "range covers.")).pack(anchor=tk.W, padx=4, pady=(0, 4))

        ttk.Label(cm, text="Measured table — travel % → brake applied %",
                  font=("Helvetica", 9, "bold")).pack(anchor=tk.W, padx=4, pady=(2, 1))
        ttk.Label(cm, wraplength=380, justify=tk.LEFT, foreground="gray",
                  text=("Fill in as many rows as you have, leave the rest blank. "
                        "Travel % must increase down the table. With the pressure "
                        "channel calibrated you can measure this directly: park the "
                        "brake at each travel step and record the line pressure.")
                  ).pack(anchor=tk.W, padx=4, pady=(0, 3))

        hdr = ttk.Frame(cm)
        hdr.pack(fill=tk.X, padx=4)
        ttk.Label(hdr, text="", width=4).pack(side=tk.LEFT)
        ttk.Label(hdr, text="travel %", width=10, anchor=tk.W,
                  foreground="gray").pack(side=tk.LEFT)
        ttk.Label(hdr, text="brake %", width=10, anchor=tk.W,
                  foreground="gray").pack(side=tk.LEFT)
        for i, (xv, yv) in enumerate(self.cam_rows):
            r = ttk.Frame(cm)
            r.pack(fill=tk.X, padx=4, pady=1)
            ttk.Label(r, text=f"{i + 1}.", width=4, anchor=tk.W).pack(side=tk.LEFT)
            ttk.Entry(r, textvariable=xv, width=9).pack(side=tk.LEFT, padx=(0, 2))
            ttk.Entry(r, textvariable=yv, width=9).pack(side=tk.LEFT)

        ttk.Checkbutton(cm, text="Also linearise the control loop (experimental)",
                        variable=self.cfg_vars["cam_lin"]).pack(anchor=tk.W, padx=4,
                                                                pady=(5, 1))
        ttk.Label(cm, wraplength=380, justify=tk.LEFT, foreground="gray",
                  text=("With this on the PID commands brake demand and the cam "
                        "curve decides the position, so a gain means the same thing "
                        "across the whole range. It changes what your existing gains "
                        "do, so tune with it off first.")
                  ).pack(anchor=tk.W, padx=4, pady=(0, 3))
        ttk.Button(cm, text="Send cam settings",
                   command=self._send_cam_cfg).pack(padx=4, pady=4, anchor=tk.W)

        # ── Brake line pressure ───────────────────────────────
        pr = ttk.LabelFrame(right, text="Brake line pressure (auxiliary channel)")
        pr.pack(fill=tk.X, pady=4)
        ttk.Label(pr, wraplength=380, justify=tk.LEFT, foreground="gray",
                  text=("AiM car/bike sensors read 500 mV at zero and 4500 mV at full "
                        "scale, so the spec alone gives a usable calibration. Confirm "
                        "it against the analogue gauge with the two points below — "
                        "that measurement wins over the datasheet.")
                  ).pack(anchor=tk.W, padx=4, pady=(3, 5))

        row = ttk.Frame(pr)
        row.pack(fill=tk.X, padx=4, pady=1)
        ttk.Label(row, text="ADC full scale:", width=16, anchor=tk.W).pack(side=tk.LEFT)
        ttk.Combobox(row, textvariable=self.cfg_vars["press_pga"], width=18,
                     state="readonly", values=PRESS_PGA_RANGES).pack(side=tk.LEFT)

        self._labelled_entry(pr, "Signal divider:", self.cfg_vars["press_div"],
                             label_width=16, suffix="sensor V per ADC V")
        ttk.Label(pr, wraplength=380, justify=tk.LEFT, foreground="gray",
                  text=("Leave the divider at 1.0 if the sensor feeds the ADS1115 "
                        "directly — which needs the ADS1115 on 5 V, because its "
                        "inputs must stay below its own supply and this sensor swings "
                        "to 4.5 V. On a 3.3 V ADS1115 you need a divider (2.0 for a "
                        "10k/10k pair) and a narrower full scale. The reading is "
                        "referred back to the sensor either way, so the calibration "
                        "below does not change.")).pack(anchor=tk.W, padx=4, pady=(1, 5))

        r = self._labelled_entry(pr, "Sensor full scale:", self.cfg_vars["press_fs_psi"],
                                 label_width=16, suffix="PSI")
        ttk.Button(r, text="Use spec", width=9,
                   command=self._pressure_from_spec).pack(side=tk.LEFT, padx=6)
        for n in ("1", "2"):
            row = ttk.Frame(pr)
            row.pack(fill=tk.X, padx=4, pady=1)
            ttk.Label(row, text=f"Point {n}:", width=10, anchor=tk.W).pack(side=tk.LEFT)
            ttk.Entry(row, textvariable=self.press_cal_vars[f"mv{n}"],
                      width=9).pack(side=tk.LEFT)
            ttk.Label(row, text="mV  =").pack(side=tk.LEFT, padx=3)
            ttk.Entry(row, textvariable=self.press_cal_vars[f"psi{n}"],
                      width=9).pack(side=tk.LEFT)
            ttk.Label(row, text="PSI").pack(side=tk.LEFT, padx=3)
            ttk.Button(row, text="Capture mV", width=11,
                       command=lambda k=f"mv{n}": self._capture_press_mv(k)
                       ).pack(side=tk.LEFT, padx=5)
        self._labelled_entry(pr, "Hose limit:", self.cfg_vars["press_limit"],
                             label_width=22, suffix="PSI")
        ttk.Button(pr, text="Compute and send",
                   command=self._compute_pressure).pack(padx=4, pady=5, anchor=tk.W)

        # ── Run storage ───────────────────────────────────────
        st = ttk.LabelFrame(right, text="Run storage")
        st.pack(fill=tk.X, pady=4)
        row = ttk.Frame(st)
        row.pack(fill=tk.X, padx=4, pady=(4, 1))
        ttk.Label(row, text="Folder:", width=10, anchor=tk.W).pack(side=tk.LEFT)
        ttk.Entry(row, textvariable=self.cfg_vars["data_dir"]).pack(
            side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(row, text="Browse…", width=9,
                   command=self._choose_data_dir).pack(side=tk.LEFT, padx=4)
        self._labelled_entry(st, "Filename prefix:", self.cfg_vars["run_prefix"],
                             label_width=15, width=18)
        ttk.Checkbutton(st, text="Save every completed run automatically",
                        variable=self.cfg_vars["autosave"]).pack(anchor=tk.W, padx=4, pady=2)
        ttk.Label(st, wraplength=380, justify=tk.LEFT, foreground="gray",
                  text=("With this on, a finished sweep is written to disk without "
                        "anyone clicking anything — so back-to-back pulls cannot lose "
                        "a run. Save CSV still works for saving one by hand. "
                        "Each run writes two files: the full per-sample log, "
                        "brake PSI included, and the RPM-domain power curve.")
                  ).pack(anchor=tk.W, padx=4, pady=(0, 4))

        # ── Profiles ──────────────────────────────────────────
        pf = ttk.LabelFrame(right, text="Profile")
        pf.pack(fill=tk.X, pady=4)
        self.profile_label = ttk.Label(pf, text="(unsaved)", foreground="gray",
                                       wraplength=380, anchor=tk.W)
        self.profile_label.pack(fill=tk.X, padx=4, pady=(3, 2))
        prow = ttk.Frame(pf)
        prow.pack(fill=tk.X, padx=4, pady=4)
        ttk.Button(prow, text="Save…", width=10,
                   command=self._save_profile).pack(side=tk.LEFT, padx=2)
        ttk.Button(prow, text="Load…", width=10,
                   command=self._load_profile).pack(side=tk.LEFT, padx=2)
        ttk.Button(prow, text="Send all to controller", width=22,
                   command=self._send_all_config).pack(side=tk.LEFT, padx=2)
        ttk.Button(pf, text="Restore Defaults", width=18,
                   command=self._restore_defaults).pack(anchor=tk.W, padx=6, pady=(0, 4))
        ttk.Label(pf, wraplength=380, justify=tk.LEFT, foreground="gray",
                  text=("Settings are remembered between sessions automatically - "
                        "reopening the program brings back whatever was on screen "
                        "when it was last closed, so a profile only needs loading "
                        "when switching between setups.")
                  ).pack(anchor=tk.W, padx=4, pady=(0, 4))

    # ══════════════════════════════════════════════════════════
    # Brake characterisation sweep
    # ══════════════════════════════════════════════════════════
    def _char_targets(self, elapsed):
        """Commanded position and phase name at `elapsed` seconds into the sweep."""
        lo = self._char_home
        try:
            hi = float(self.cfg_vars["brake_max"].get())
        except ValueError:
            hi = float(BRAKE_FULL_TRAVEL_STEPS)
        if elapsed < CHAR_UP_S:
            return lo + (hi - lo) * (elapsed / CHAR_UP_S), "up"
        if elapsed < CHAR_UP_S + CHAR_HOLD_S:
            return hi, "hold"
        if elapsed < CHAR_UP_S + CHAR_HOLD_S + CHAR_DOWN_S:
            f = (elapsed - CHAR_UP_S - CHAR_HOLD_S) / CHAR_DOWN_S
            return hi + (lo - hi) * f, "down"
        return lo, "done"

    def _send_encoder_cfg(self):
        """Push the encoder settings. Accepted and stored by the firmware,
        but inert until the hardware exists - see the TBD panel."""
        self._send(f"ENCODER,{1 if self.cfg_vars['enc_enabled'].get() else 0},"
                   f"{self.cfg_vars['enc_cpr'].get()},"
                   f"{1 if self.cfg_vars['enc_invert'].get() else 0}")

    def _start_brake_char(self):
        """Sweep the brake across its travel and back, logging position vs PSI."""
        if self._char_active:                 # button doubles as the abort
            self._abort_brake_char("Aborted by operator")
            return
        if not (self.ser and self.ser.is_open):
            messagebox.showinfo("Not connected", "Connect to the ESP32 first.")
            return
        if not self.ready_flags.get("homed"):
            messagebox.showwarning(
                "Home first",
                "The brake has to be homed before it can be characterised - "
                "without a known zero the positions recorded mean nothing.")
            return
        with self._lock:
            rpm = self.live["rpm"]
        if rpm > 100:
            # This drives the brake to full travel. With an engine turning that
            # is not a measurement, it is a sudden full brake application.
            messagebox.showwarning(
                "Engine is running",
                f"The engine is turning at {rpm:.0f} RPM.\n\nThis test drives the "
                "brake through its entire travel, which with an engine running "
                "would apply full braking. Stop the engine first.")
            return
        total = CHAR_UP_S + CHAR_HOLD_S + CHAR_DOWN_S
        if not messagebox.askyesno(
                "Characterise the brake?",
                f"The brake will move from home to full travel over {CHAR_UP_S:.0f} s, "
                f"hold for {CHAR_HOLD_S:.0f} s, then return over {CHAR_DOWN_S:.0f} s "
                f"({total:.0f} s total).\n\nMake sure nothing is in the way of the "
                "linkage.\n\nStart?"):
            return

        with self._lock:
            self._char_home = int(self.live["brake_pos"])
        try:
            self._char_hi = float(self.cfg_vars["brake_max"].get())
        except ValueError:
            self._char_hi = float(BRAKE_FULL_TRAVEL_STEPS)
        self._char_rows = []
        self._char_phase_sent = None
        self._char_engaged = False
        self._char_base_psi = None
        self._char_susp_since = None
        self._char_stall_at = None
        self._char_phase_sent = None
        self.char_stall_label.config(text="")
        self._char_active = True
        self._char_t0 = time.monotonic()
        self.char_btn.config(text="Abort sweep")
        self._log_event("Brake characterisation started", "ack")
        self._char_tick()

    def _char_tick(self):
        if not self._char_active:
            return
        elapsed = time.monotonic() - self._char_t0
        target, phase = self._char_targets(elapsed)
        if phase == "done":
            self._send(f"BRAKE,{int(self._char_home)}")
            self._finish_brake_char()
            return
        # One command per leg. Re-issuing a moving target every tick makes the
        # controller decelerate to a stop at each one - a lurch 20 times a
        # second rather than a traverse. The firmware is told the whole leg
        # and its duration, and runs it as a single steady move.
        if phase != self._char_phase_sent:
            self._char_phase_sent = phase
            if phase == 'up':
                self._send(f"BRAKE_SWEEP,{int(round(self._char_hi))},"
                           f"{int(CHAR_UP_S * 1000)}")
            elif phase == 'down':
                self._send(f"BRAKE_SWEEP,{int(round(self._char_home))},"
                           f"{int(CHAR_DOWN_S * 1000)}")
            # 'hold' needs no command: it is already where the leg ends.
        with self._lock:
            d = dict(self.live)
        # Only the outward leg: on the way back position falls by design, so
        # the comparison means nothing there.
        stalled = False
        if phase == "up":
            stalled = self._char_check_stall(
                elapsed, float(d["brake_pos"]), d["press_psi"])

        self._char_rows.append([
            f"{elapsed:.3f}", phase, f"{target:.1f}",
            f"{d['brake_pos']}", f"{d['brake_pct']:.2f}",
            f"{d['press_psi']:.2f}", f"{d['press_mv']:.1f}",
            f"{d['load_raw']:.2f}", f"{d['rpm']:.1f}",
            "1" if self._char_stall_at else "0",
        ])

        if stalled:
            s = self._char_stall_at
            msg = (f"Possible stall at {s['time_s']:.1f}s: commanded "
                   f"+{s['steps_gained']:.0f} steps for only "
                   f"+{s['psi_gained']:.1f} PSI")
            self._log_event(msg, "err")
            self.char_stall_label.config(
                text=f"STALL? {s['commanded']:.0f} steps / {s['psi']:.0f} PSI")
            if self.cfg_vars["char_stop_on_stall"].get():
                self._abort_brake_char(msg)
                return

        if self._char_stall_at:
            self.char_progress.config(
                text=f"{phase}  {elapsed:4.1f}s   cmd {int(target)}   "
                     f"{d['press_psi']:.0f} PSI   (stall suspected)")
        else:
            self.char_progress.config(
                text=f"{phase}  {elapsed:4.1f}s   cmd {int(target)}   "
                     f"{d['press_psi']:.0f} PSI")
        self.root.after(CHAR_TICK_MS, self._char_tick)

    def _char_check_stall(self, elapsed, pos, psi):
        """Watch for commanded position climbing while pressure does not.

        Returns True the moment a stall is first suspected. A fully applied
        brake looks the same as a stalled motor from here - both are position
        going up with pressure flat - so this reports a suspicion, not a
        verdict, and the sweep records where it happened rather than deciding
        what it means.
        """
        if self._char_stall_at is not None:
            return False                       # already flagged, once is enough
        if self._char_base_psi is None:
            self._char_base_psi = psi
            return False
        # Below the takeup point there is nothing to compare against: the
        # actuator is moving and no pressure is expected yet.
        if not self._char_engaged:
            if psi > self._char_base_psi + CHAR_ENGAGE_PSI:
                self._char_engaged = True
            return False

        cutoff = elapsed - CHAR_STALL_WIN_S
        past = None
        for r in self._char_rows:              # oldest sample still in the window
            if float(r[0]) >= cutoff:
                past = r
                break
        if past is None:
            return False

        dpos = pos - float(past[3])
        dpsi = psi - float(past[5])
        if dpos >= CHAR_STALL_STEPS and dpsi < CHAR_STALL_PSI:
            if self._char_susp_since is None:
                self._char_susp_since = elapsed
            elif elapsed - self._char_susp_since >= CHAR_STALL_HOLD_S:
                self._char_stall_at = {
                    "time_s": round(elapsed, 2),
                    "commanded": round(pos, 1),
                    "psi": round(psi, 2),
                    "steps_gained": round(dpos, 1),
                    "psi_gained": round(dpsi, 2),
                }
                return True
        else:
            self._char_susp_since = None       # pressure moved, not a stall
        return False

    def _abort_brake_char(self, why):
        self._char_active = False
        self._send(f"BRAKE,{int(self._char_home)}")
        self.char_btn.config(text="Characterise brake")
        self.char_progress.config(text=why)
        self._log_event(f"Brake characterisation stopped: {why}", "err")

    def _finish_brake_char(self):
        self._char_active = False
        self.char_btn.config(text="Characterise brake")
        rows = list(self._char_rows)
        if len(rows) < 10:
            self.char_progress.config(text="Too few samples - nothing saved")
            return

        folder = self.cfg_vars["data_dir"].get().strip() or DEFAULT_DATA_DIR
        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(folder, f"brake_char_{stamp}")
        try:
            os.makedirs(folder, exist_ok=True)
            with open(base + ".csv", "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["Time_s", "Phase", "Commanded_Steps", "Brake_Pos",
                            "Brake_Pct", "Brake_PSI", "Pressure_mV",
                            "LoadCell_mV", "RPM", "Stall_Suspected"])
                w.writerows(rows)
            self._write_conditions(base + "_conditions.json", base + ".csv")
            self._plot_brake_char(rows, base + ".png", stamp)
        except (OSError, ValueError) as e:
            self.char_progress.config(text=f"Save failed: {e}")
            self._log_event(f"Brake characterisation save failed: {e}", "err")
            return

        self.char_progress.config(text=f"Saved brake_char_{stamp}")
        self._log_event(f"Brake characterisation saved: brake_char_{stamp}"
                        f" ({len(rows)} samples)", "ack")
        if self._char_stall_at:
            st = self._char_stall_at
            messagebox.showwarning(
                "Possible stall during the sweep",
                f"At {st['time_s']:.1f}s the commanded position rose "
                f"{st['steps_gained']:.0f} microsteps for only "
                f"{st['psi_gained']:.1f} PSI, at {st['commanded']:.0f} steps "
                f"and {st['psi']:.0f} PSI.\n\n"
                "Either the motor stopped keeping up with the commanded "
                "position, or the brake was already fully applied - without "
                "an encoder those look identical from here. Treat travel "
                "beyond that point as unverified.\n\n"
                f"{len(rows)} samples saved to:\n{base}.csv")
        else:
            messagebox.showinfo(
                "Characterisation complete",
                f"{len(rows)} samples saved to:\n{base}.csv\n\nPlot: {base}.png")

    def _plot_brake_char(self, rows, path, stamp):
        """Two views: pressure against position, and both against time."""
        from matplotlib.backends.backend_agg import FigureCanvasAgg

        t = np.array([float(r[0]) for r in rows])
        phase = [r[1] for r in rows]
        pos = np.array([float(r[3]) for r in rows])
        psi = np.array([float(r[5]) for r in rows])
        up = np.array([p == "up" for p in phase])
        dn = np.array([p == "down" for p in phase])

        fig = Figure(figsize=(10, 7), dpi=110)
        FigureCanvasAgg(fig)

        # The characterisation itself: does moving the actuator make pressure?
        ax = fig.add_subplot(211)
        ax.plot(pos[up], psi[up], "-", color="tab:blue", linewidth=1.6,
                label="extending")
        ax.plot(pos[dn], psi[dn], "-", color="tab:orange", linewidth=1.6,
                label="retracting")
        ax.set_xlabel("Brake position (microsteps)")
        ax.set_ylabel("Brake line pressure (PSI)")
        ax.set_title(f"Brake characterisation  {stamp}")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left")

        # Where pressure first moves is the takeup point - the number that should
        # set the preload, rather than a guessed percentage.
        if psi[up].size and np.isfinite(psi[up]).any():
            base_psi = float(np.median(psi[up][:5])) if psi[up].size >= 5 else float(psi[up][0])
            rise = np.flatnonzero(psi[up] > base_psi + 20.0)
            if rise.size:
                x = float(pos[up][rise[0]])
                ax.axvline(x, color="tab:green", linestyle="--", linewidth=1.2)
                ax.annotate(f"takeup ≈ {x:.0f} steps",
                            xy=(x, ax.get_ylim()[1] * 0.85),
                            xytext=(6, 0), textcoords="offset points",
                            color="tab:green", fontsize=9)

        # Where the sweep stopped being trustworthy, marked on the curve it
        # affects rather than only in the log.
        if self._char_stall_at:
            sx = self._char_stall_at['commanded']
            ax.axvline(sx, color="tab:red", linestyle=":", linewidth=1.6)
            ax.annotate("possible stall", xy=(sx, ax.get_ylim()[1] * 0.55),
                        xytext=(6, 0), textcoords="offset points",
                        color="tab:red", fontsize=9)

        ax2 = fig.add_subplot(212)
        ax2.plot(t, pos, "-", color="tab:orange", linewidth=1.3, label="position")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Brake position (microsteps)", color="tab:orange")
        ax2.grid(True, alpha=0.3)
        ax2p = ax2.twinx()
        ax2p.plot(t, psi, "-", color="tab:purple", linewidth=1.3, label="PSI")
        ax2p.set_ylabel("Brake PSI", color="tab:purple")
        lines = ax2.get_lines() + ax2p.get_lines()
        ax2.legend(lines, [l.get_label() for l in lines], loc="upper left")

        fig.tight_layout()
        fig.savefig(path, facecolor="white")

    # ── Calibration helpers ──────────────────────────────────
    def _capture_mv(self, key):
        with self._lock:
            mv = self.live["load_raw"]
        self.calib_vars[key].set(f"{mv:.2f}")

    def _capture_press_mv(self, key):
        with self._lock:
            mv = self.live["press_mv"]
        self.press_cal_vars[key].set(f"{mv:.2f}")

    @staticmethod
    def _weight_to_newtons(value, units):
        if units == "lb":
            return value * 4.4482216153
        if units == "kg":
            return value * 9.80665
        return value                      # already newtons

    def _compute_load_cell(self):
        """Derive N-per-mV from the captured pair and push it to the controller."""
        try:
            zero = float(self.calib_vars["zero_mv"].get())
            load = float(self.calib_vars["load_mv"].get())
            weight = float(self.calib_vars["known_wt"].get())
        except ValueError:
            messagebox.showwarning(
                "Incomplete calibration",
                "Enter the zero reading, the loaded reading, and the known weight.")
            return
        delta = load - zero
        if abs(delta) < 1e-6:
            messagebox.showwarning(
                "No change between readings",
                "The loaded and unloaded readings are the same, so no scale can be "
                "derived. Check the load cell wiring and that the weight is applied.")
            return
        if weight <= 0:
            messagebox.showwarning("Invalid weight", "The known weight must be above zero.")
            return
        newtons = self._weight_to_newtons(weight, self.calib_vars["wt_units"].get())
        scale = newtons / delta
        self.cfg_vars["cal_scale"].set(f"{scale:.6f}")
        self._send_load_cell_cfg()
        messagebox.showinfo(
            "Calibration sent",
            f"{newtons:.2f} N over {delta:.2f} mV\n"
            f"Scale: {scale:.6f} N/mV\n\n"
            "Apply a different known weight to check it reads back correctly.")

    def _pressure_from_spec(self):
        """Fill the calibration from the sensor's rated span.

        Good enough to read pressure immediately after wiring; the two-point
        calibration against the gauge is what settles it properly, since the
        part's actual full scale may not be exactly what the catalogue says.
        """
        try:
            fs_psi = float(self.cfg_vars["press_fs_psi"].get())
        except ValueError:
            messagebox.showwarning("Sensor spec",
                                   "Enter the sensor's full-scale pressure in PSI.")
            return
        if fs_psi <= 0:
            messagebox.showwarning("Sensor spec", "Full scale must be above zero.")
            return
        span_mv = PRESSURE_FS_MV - PRESSURE_ZERO_MV
        self.cfg_vars["press_off_mv"].set(f"{PRESSURE_ZERO_MV:.1f}")
        self.cfg_vars["press_psi_mv"].set(f"{fs_psi / span_mv:.6f}")
        self.press_cal_vars["mv1"].set(f"{PRESSURE_ZERO_MV:.0f}")
        self.press_cal_vars["psi1"].set("0")
        self.press_cal_vars["mv2"].set(f"{PRESSURE_FS_MV:.0f}")
        self.press_cal_vars["psi2"].set(f"{fs_psi:g}")
        self._send_pressure_cfg()
        self._log_event(
            f"Pressure set from spec: {PRESSURE_ZERO_MV:.0f}–{PRESSURE_FS_MV:.0f} mV "
            f"= 0–{fs_psi:g} PSI", "ack")

    def _compute_pressure(self):
        try:
            mv1 = float(self.press_cal_vars["mv1"].get())
            psi1 = float(self.press_cal_vars["psi1"].get())
            mv2 = float(self.press_cal_vars["mv2"].get())
            psi2 = float(self.press_cal_vars["psi2"].get())
        except ValueError:
            messagebox.showwarning("Incomplete calibration",
                                   "Fill in both mV/PSI pairs first.")
            return
        if abs(mv2 - mv1) < 1e-6:
            messagebox.showwarning(
                "Points are identical",
                "The two calibration points have the same voltage, so no slope can "
                "be derived. Capture them at clearly different pressures.")
            return
        scale = (psi2 - psi1) / (mv2 - mv1)
        if scale == 0:
            messagebox.showwarning(
                "Flat calibration",
                "Both points give the same pressure, so the channel would stay at "
                "zero. Capture them at clearly different pressures.")
            return
        offset = mv1 - psi1 / scale
        self.cfg_vars["press_off_mv"].set(f"{offset:.4f}")
        self.cfg_vars["press_psi_mv"].set(f"{scale:.6f}")
        self._send_pressure_cfg()
        messagebox.showinfo("Pressure calibration sent",
                            f"{scale:.4f} PSI/mV, offset {offset:.2f} mV\n"
                            f"Limit: {self.cfg_vars['press_limit'].get()} PSI")

    # ── Config senders ───────────────────────────────────────
    def _send_load_cell_cfg(self):
        self._send(f"CAL_SCALE,{self.cfg_vars['cal_scale'].get()}")
        self._send(f"CAL_ARM,{self.cfg_vars['lever_arm'].get()}")
        self._send(f"CAL_MECH,{self.cfg_vars['mech_ratio'].get()}")

    # Limits copied from the firmware's own command handlers. Checking here
    # means a bad value is caught where it was typed, instead of being saved
    # into a profile and only surfacing as a controller error much later.
    CFG_LIMITS = [
        ("teeth",          "Pulses per rev",    "int",   1, 60),
        ("drive_ratio",    "Drive ratio",       "float", 1e-9, None),
        ("rpm_band_min",   "Valid RPM band min", "float", 0, None),
        ("rpm_band_max",   "Valid RPM band max", "float", 0, None),
        ("rpm_extrap_n",   "Extrapolation fit points", "int", 2, 10),
        ("rpm_extrap_max", "Extrapolation max run",    "int", 1, 50),
        ("rpm_avg",        "Average over",      "int",   1, 500),
        ("rpm_slew",       "Slew limit",        "float", 0, None),
    ]

    def _validate_cfg(self):
        """Return a list of human-readable problems with the current settings."""
        problems = []
        for key, label, kind, lo, hi in self.CFG_LIMITS:
            raw = self.cfg_vars[key].get().strip()
            try:
                val = int(raw) if kind == "int" else float(raw)
            except ValueError:
                problems.append(f"{label}: {raw!r} is not a number")
                continue
            if lo is not None and val < lo:
                problems.append(f"{label}: {raw} is below the minimum of {lo:g}")
            elif hi is not None and val > hi:
                problems.append(f"{label}: {raw} is above the maximum of {hi:g}")
        # Cross-field rules the firmware also enforces.
        try:
            lo = float(self.cfg_vars["rpm_band_min"].get())
            hi = float(self.cfg_vars["rpm_band_max"].get())
            if hi <= lo + 100:
                problems.append(
                    f"Valid RPM band: max ({hi:g}) must be at least 100 above "
                    f"min ({lo:g})")
        except ValueError:
            pass
        if self.cfg_vars["rpm_median"].get().strip() not in ("1", "3", "5", "7"):
            problems.append("Median window: must be 1, 3, 5 or 7")
        try:
            r = float(self.cfg_vars["rpm_ratio"].get())
            if r != 0.0 and r <= 1.0:
                problems.append("Ratio gate: must be 0 (off) or greater than 1")
        except ValueError:
            problems.append("Ratio gate: not a number")
        return problems

    def _rpm_avg_lag_s(self):
        """Seconds of averaging lag at the RPM we are actually running at.

        This average feeds the PID, so the window is not free: the loop is
        answering an RPM the engine had this long ago.
        """
        try:
            n = int(self.cfg_vars["rpm_avg"].get())
            ppr = int(self.cfg_vars["teeth"].get())
        except ValueError:
            return None
        with self._lock:
            rpm = self.live["rpm"]
        if rpm < 100:
            # Not running: fall back to whatever this run is aimed at, so
            # the figure still means something while setting up.
            for key in ("hold_rpm", "start_rpm"):
                try:
                    rpm = float(self.param_vars[key].get())
                    if rpm >= 100:
                        break
                except (KeyError, ValueError):
                    rpm = 0.0
        if rpm < 100 or ppr < 1 or n < 1:
            return None
        return n / (rpm * ppr / 60.0)

    def _send_rpm_cfg(self):
        self._send(f"TEETH,{self.cfg_vars['teeth'].get()}")
        self._send(f"RATIO,{self.cfg_vars['drive_ratio'].get()}")
        self._send(f"RPM_BAND,{self.cfg_vars['rpm_band_min'].get()},"
                   f"{self.cfg_vars['rpm_band_max'].get()}")
        self._send(f"RPM_EXTRAP,{1 if self.cfg_vars['rpm_extrap'].get() else 0},"
                   f"{self.cfg_vars['rpm_extrap_n'].get()},"
                   f"{self.cfg_vars['rpm_extrap_max'].get()}")
        self._send(f"RPM_MEDIAN,{self.cfg_vars['rpm_median'].get()}")
        self._send(f"RPM_RATIO,{self.cfg_vars['rpm_ratio'].get()}")
        self._send(f"RPM_SLEW,{self.cfg_vars['rpm_slew'].get()}")
        self._send(f"RPM_AVG,{self.cfg_vars['rpm_avg'].get()}")

    def _send_rampdown_cfg(self):
        # A cutoff below idle is the one setting here that can cause the problem
        # it exists to prevent: the engine settles above it, so the ramp keeps
        # commanding brake and never releases.
        try:
            if float(self.cfg_vars["cutoff_rpm"].get()) < 600:
                if not messagebox.askyesno(
                        "Cutoff below idle?",
                        "A cutoff under 600 RPM is below most idle speeds.\n\n"
                        "If the engine idles above the cutoff, the ramp keeps "
                        "pulling it down and the brake never fully releases - "
                        "which is how it stalls.\n\nSend it anyway?"):
                    return
        except ValueError:
            pass
        try:
            mode = RAMPDOWN_MODES.index(self.cfg_vars["rampdown_mode"].get())
        except ValueError:
            mode = 1
        self._send(f"RAMPDOWN_MODE,{mode}")
        self._send(f"RAMPDOWN_RATE,{self.cfg_vars['rampdown_rate'].get()}")
        self._send(f"RAMPDOWN_BRAKE,{self.cfg_vars['rampdown_brake'].get()}")
        self._send(f"CUTOFF_RPM,{self.cfg_vars['cutoff_rpm'].get()}")
        self._send(f"THROTTLE_OFF,{self.cfg_vars['throttle_off'].get()}")
        self._send(f"STOP_RATE,{self.cfg_vars['stop_rate'].get()}")

    def _force_rampdown(self):
        """Trigger the ramp-down by hand, to watch it without an engine."""
        if not (self.ser and self.ser.is_open):
            messagebox.showinfo("Not connected", "Connect to the ESP32 first.")
            return
        self._send("RAMPDOWN")

    def _send_brake_cfg(self):
        self._send(f"BRAKE_RANGE,{self.cfg_vars['brake_min'].get()},"
                   f"{self.cfg_vars['brake_max'].get()}")
        self._send(f"PRELOAD,{self.cfg_vars['preload_pct'].get()}")
        self._send(f"INVERT,{1 if self.cfg_vars['invert'].get() else 0}")
        self._send(f"STEPPER_SPEED,{self.cfg_vars['step_speed'].get()}")
        self._send(f"STEPPER_ACCEL,{self.cfg_vars['step_accel'].get()}")
        self._sync_brake_slider()

    def _cam_points(self):
        """Filled-in table rows as (travel %, brake %), in order.

        Returns None with a complaint if the rows don't describe a usable curve.
        """
        pts = []
        for i, (xv, yv) in enumerate(self.cam_rows):
            xs, ys = xv.get().strip(), yv.get().strip()
            if not xs and not ys:
                continue
            try:
                x, y = float(xs), float(ys)
            except ValueError:
                messagebox.showwarning(
                    "Cam table",
                    f"Row {i + 1} needs a number in both columns, or both blank.")
                return None
            if not (0.0 <= x <= 100.0 and 0.0 <= y <= 100.0):
                messagebox.showwarning("Cam table",
                                       f"Row {i + 1}: both values must be 0–100 %.")
                return None
            pts.append((x, y))
        for a, b in zip(pts, pts[1:]):
            if b[0] <= a[0]:
                messagebox.showwarning(
                    "Cam table",
                    "Travel % must increase down the table — otherwise there is no "
                    "single brake value for a given position.")
                return None
        return pts

    def _send_cam_cfg(self):
        model = CAM_MODELS.index(self.cfg_vars["cam_model"].get()) \
            if self.cfg_vars["cam_model"].get() in CAM_MODELS else 0
        pts = self._cam_points()
        if pts is None:
            return
        if model == 1 and float(self.cfg_vars["cam_spd"].get() or 0) <= 0:
            messagebox.showwarning(
                "Cam geometry",
                "The eccentric model needs steps per degree to know how much of "
                "the cam the travel range covers.")
            return
        if model == 2 and len(pts) < 2:
            messagebox.showwarning(
                "Cam geometry",
                "The measured table needs at least two rows to interpolate between.")
            return

        self._send(f"CAM_SPD,{self.cfg_vars['cam_spd'].get()}")
        self._send(f"CAM_NPTS,{len(pts)}")
        for i, (x, y) in enumerate(pts):
            self._send(f"CAM_PT,{i},{x},{y}")
        self._send(f"CAM_MODEL,{model}")
        self._send(f"CAM_LIN,{1 if self.cfg_vars['cam_lin'].get() else 0}")

    def _choose_data_dir(self):
        path = filedialog.askdirectory(
            title="Where should completed runs be saved?",
            initialdir=self.cfg_vars["data_dir"].get() or os.path.expanduser("~"))
        if path:
            self.cfg_vars["data_dir"].set(path)

    def _send_pressure_cfg(self):
        # Hardware scaling first, so the calibration that follows is interpreted
        # against the right full scale and divider.
        try:
            pga = PRESS_PGA_RANGES.index(self.cfg_vars["press_pga"].get())
        except ValueError:
            pga = 0
        self._send(f"PRESS_PGA,{pga}")
        self._send(f"PRESS_DIV,{self.cfg_vars['press_div'].get()}")
        self._send(f"CAL_PRESS,{self.cfg_vars['press_off_mv'].get()},"
                   f"{self.cfg_vars['press_psi_mv'].get()}")
        self._send(f"PRESS_LIMIT,{self.cfg_vars['press_limit'].get()}")

    def _send_all_config(self):
        """Push every setting to the controller — use after a reconnect."""
        if not (self.ser and self.ser.is_open):
            messagebox.showinfo("Not connected", "Connect to the ESP32 first.")
            return
        problems = self._validate_cfg()
        if problems:
            # Sending anyway would half-apply the configuration and bury the
            # reason in a stream of controller errors.
            messagebox.showerror(
                "Settings out of range",
                "These will be rejected by the controller, so nothing was "
                "sent:\n\n  - " + "\n  - ".join(problems) +
                "\n\nFix them and send again.")
            self._log_event("Send all aborted: " + "; ".join(problems), "err")
            return
        self._send_load_cell_cfg()
        self._send_rpm_cfg()
        self._send_brake_cfg()
        self._send_rampdown_cfg()
        self._send_cam_cfg()
        self._send_pressure_cfg()
        self._apply_pid()
        self._send_params()
        self.root.after(400, lambda: self._send("STATUS"))

    # ── Profiles ─────────────────────────────────────────────
    @staticmethod
    def _restore_var(var, value):
        """Set a tk variable from JSON, coping with bools stored as strings."""
        if isinstance(var, tk.BooleanVar):
            if isinstance(value, str):
                var.set(value.strip().lower() in ("1", "true", "yes", "on"))
            else:
                var.set(bool(value))
        else:
            var.set(str(value))

    def _profile_groups(self):
        """Every further setting a profile carries, grouped for readability.

        Declared in one place so saving and loading cannot drift apart - a
        setting added here is written and restored without touching either.
        Deliberately excluded: serial port, data source, manual brake position
        and transient UI state, none of which describe the rig.
        """
        analysis = {
            "filter": self.filter_type_var,
            "rpm_bin": self.rpm_bin_var,
            "rpm_min": self.rpm_min_var,
            "rpm_max": self.rpm_max_var,
            "despike": self.despike_var,
            "show_raw": self.show_raw_var,
            "sae_apply": self.sae_apply_var,
        }
        analysis.update({f"param_{k}": v for k, v in self.filter_param_vars.items()})
        analysis.update({f"yd_{k}": v for k, v in self.yd_vars.items()})
        analysis.update({f"sae_{k}": v for k, v in self.sae_vars.items()})
        return {
            "analysis": analysis,
            # The captured readings, not just the scale derived from them, so
            # the calibration can be checked or re-derived later.
            "load_cell_cal": dict(self.calib_vars),
            "pressure_points": dict(self.press_cal_vars),
            "live_smoothing": {"on": self.live_smooth_var,
                               "alpha": self.live_alpha_var},
            "traces_shown": dict(self.trace_vars),
            "replay": {"speed": self.replay_speed_var,
                       "drive_motor": self.replay_motor_var,
                       "travel_pct": self.replay_motor_gain_var},
        }

    def _profile_snapshot(self):
        data = {k: v.get() for k, v in self.cfg_vars.items()}
        data["pid_hold"] = {k: v.get() for k, v in self.pid_vars.items()}
        data["pid_sweep"] = {k: v.get() for k, v in self.pid_sweep_vars.items()}
        data["run"] = {k: v.get() for k, v in self.param_vars.items()}
        data["units"] = self.units_var.get()
        data["cam_table"] = [[xv.get(), yv.get()] for xv, yv in self.cam_rows]
        for group, variables in self._profile_groups().items():
            data[group] = {k: v.get() for k, v in variables.items()}
        return data

    def _save_profile(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".json", filetypes=[("Dyno profile", "*.json")],
            initialfile="dyno_profile.json")
        if not path:
            return
        try:
            with open(path, "w") as f:
                json.dump(self._profile_snapshot(), f, indent=2)
        except OSError as e:
            messagebox.showerror("Could not save profile", str(e))
            return
        self.profile_path = path
        self.profile_label.config(text=f"Saved: {os.path.basename(path)}")

    def _apply_profile(self, data):
        """Push a saved settings dict into the widgets. Shared by profile load,
        session restore and Restore Defaults, so all three behave identically."""
        for k, var in self.cfg_vars.items():
            if k in data:
                self._restore_var(var, data[k])
        for group, variables in self._profile_groups().items():
            for k, v in (data.get(group) or {}).items():
                if k in variables:
                    self._restore_var(variables[k], v)
        for section, target in (("pid_hold", self.pid_vars),
                                ("pid_sweep", self.pid_sweep_vars),
                                ("run", self.param_vars)):
            for k, v in (data.get(section) or {}).items():
                if k in target:
                    target[k].set(str(v))
        for (xv, yv), pair in zip(self.cam_rows, data.get("cam_table") or []):
            xv.set(str(pair[0]) if len(pair) > 0 else "")
            yv.set(str(pair[1]) if len(pair) > 1 else "")
        if data.get("units") in ("Nm", "lb-ft"):
            self.units_var.set(data["units"])
            self._on_units_changed()
        self._sync_brake_slider()

    # ── Session memory ───────────────────────────────────────
    def _load_session(self):
        """Restore the settings this program was last closed with."""
        if not os.path.exists(SESSION_FILE):
            return
        try:
            with open(SESSION_FILE) as f:
                data = json.load(f)
        except (OSError, ValueError) as e:
            # A damaged session file must never stop the program starting.
            self._log_event(f"Could not read last session ({e}) - using defaults", "err")
            return
        self._apply_profile(data)
        self._log_event("Restored settings from the last session", "ack")

    def _save_session(self):
        try:
            snap = self._profile_snapshot()
            if snap == self._session_written:
                return
            with open(SESSION_FILE, "w") as f:
                json.dump(snap, f, indent=2)
            self._session_written = snap
        except OSError:
            pass                    # never let this interrupt a run

    def _restore_defaults(self):
        if not messagebox.askyesno(
                "Restore defaults?",
                "This puts every field back to its startup default, including "
                "load cell and pressure calibration, PID gains and the cam "
                "table.\n\nYour saved profiles and recorded runs are untouched.\n\n"
                "Restore defaults?"):
            return
        self._apply_profile(self._factory_defaults)
        self._log_event("All settings restored to defaults", "ack")
        self.profile_label.config(text="(defaults)")
        if self.ser and self.ser.is_open:
            self._send_all_config()

    def _load_profile(self):
        path = filedialog.askopenfilename(filetypes=[("Dyno profile", "*.json"),
                                                     ("All files", "*.*")])
        if not path:
            return
        try:
            with open(path) as f:
                data = json.load(f)
        except (OSError, ValueError) as e:
            messagebox.showerror("Could not load profile", str(e))
            return

        self._apply_profile(data)
        # Redraw with the restored filtering, so the chart on screen always
        # matches the settings sitting next to it.
        self._refresh_analysis()

        self.profile_path = path
        self.profile_label.config(text=f"Loaded: {os.path.basename(path)}")
        if self.ser and self.ser.is_open:
            self._send_all_config()

    # ── Analysis & Filtering tab ─────────────────────────────
    def _build_analysis_tab(self, parent):
        ctrl = ttk.Frame(parent, width=320)
        ctrl.pack(side=tk.LEFT, fill=tk.Y, padx=4, pady=4)
        ctrl.pack_propagate(False)

        # Source
        sf = ttk.LabelFrame(ctrl, text="Dataset")
        sf.pack(fill=tk.X, pady=2)
        ttk.Button(sf, text="Use Last Recorded Run",
                   command=self._send_run_to_analysis).pack(fill=tk.X, padx=4, pady=2)
        ttk.Button(sf, text="Load CSV / Log…",
                   command=self._analysis_load_file).pack(fill=tk.X, padx=4, pady=2)
        self.analysis_src_label = ttk.Label(sf, text=self.analysis_label,
                                             foreground="gray", wraplength=290, anchor=tk.W)
        self.analysis_src_label.pack(fill=tk.X, padx=4, pady=2)

        # Filter selection
        ff = ttk.LabelFrame(ctrl, text="Filter")
        ff.pack(fill=tk.X, pady=2)
        self.filter_type_var = tk.StringVar(value=dsp.FILTER_YOURDYNO)
        ftype = ttk.Combobox(ff, textvariable=self.filter_type_var, state="readonly",
                             values=dsp.FILTER_NAMES)
        ftype.pack(fill=tk.X, padx=4, pady=3)
        ftype.bind("<<ComboboxSelected>>", lambda e: self._refresh_analysis())

        self.filter_param_vars = {
            "alpha": tk.StringVar(value="0.20"),
            "window": tk.StringVar(value="20"),
            "polyorder": tk.StringVar(value="3"),
            "tau": tk.StringVar(value="50"),       # RPM-domain time constant
            "degree": tk.StringVar(value="2"),
        }
        param_defs = [
            ("EMA α (0–1):",        "alpha"),
            ("MA / SG window:",     "window"),
            ("SG poly order:",      "polyorder"),
            ("Low-pass τ (RPM):",   "tau"),
            ("Poly fit degree:",    "degree"),
        ]
        for label_text, key in param_defs:
            row = ttk.Frame(ff)
            row.pack(fill=tk.X, padx=4, pady=1)
            ttk.Label(row, text=label_text, width=16, anchor=tk.W).pack(side=tk.LEFT)
            e = ttk.Entry(row, textvariable=self.filter_param_vars[key], width=8)
            e.pack(side=tk.LEFT)
            e.bind("<Return>", lambda ev: self._refresh_analysis())

        # RPM-domain options
        rf = ttk.LabelFrame(ctrl, text="RPM-domain processing")
        rf.pack(fill=tk.X, pady=2)
        for label_text, key, default in [("RPM bin step:", "bin", "100"),
                                         ("Pull min RPM:", "min", "1500"),
                                         ("Pull max RPM (0=auto):", "max", "0")]:
            row = ttk.Frame(rf)
            row.pack(fill=tk.X, padx=4, pady=1)
            ttk.Label(row, text=label_text, width=18, anchor=tk.W).pack(side=tk.LEFT)
            var = tk.StringVar(value=default)
            setattr(self, f"rpm_{key}_var", var)
            e = ttk.Entry(row, textvariable=var, width=8)
            e.pack(side=tk.LEFT)
            e.bind("<Return>", lambda ev: self._refresh_analysis())

        self.despike_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(rf, text="Spike removal (Hampel: RPM+Torque)", variable=self.despike_var,
                        command=self._refresh_analysis).pack(anchor=tk.W, padx=4, pady=1)
        self.show_raw_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(rf, text="Show raw overlay", variable=self.show_raw_var,
                        command=self._refresh_analysis).pack(anchor=tk.W, padx=4, pady=1)

        # YourDyno "Run analysis tool" method controls (only used when that filter
        # is selected). Gauge/Graph 0-10, Spike 0-5; binning uses 'RPM bin step'
        # above (set 100 for J1349 parity).
        yf = ttk.LabelFrame(ctrl, text="YourDyno Binned method")
        yf.pack(fill=tk.X, pady=2)
        self.yd_vars = {"gauge": tk.StringVar(value="3"),
                        "graph": tk.StringVar(value="3"),
                        "spike": tk.StringVar(value="0")}
        for label_text, key in [("Gauge filter (0-10):", "gauge"),
                                ("Graph filter (0-10):", "graph"),
                                ("RPM spike (0-5):", "spike")]:
            row = ttk.Frame(yf)
            row.pack(fill=tk.X, padx=4, pady=1)
            ttk.Label(row, text=label_text, width=18, anchor=tk.W).pack(side=tk.LEFT)
            e = ttk.Entry(row, textvariable=self.yd_vars[key], width=8)
            e.pack(side=tk.LEFT)
            e.bind("<Return>", lambda ev: self._refresh_analysis())

        # SAE J607 atmospheric correction
        sf2 = ttk.LabelFrame(ctrl, text="SAE J607 correction")
        sf2.pack(fill=tk.X, pady=2)
        self.sae_apply_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(sf2, text="Apply J607 correction", variable=self.sae_apply_var,
                        command=self._refresh_analysis).pack(anchor=tk.W, padx=4, pady=1)
        self.sae_vars = {"temp": tk.StringVar(value="77.0"),
                         "hum": tk.StringVar(value="50.0"),
                         "press": tk.StringVar(value="29.92")}
        for label_text, key in [("Temp (°F):", "temp"), ("Humidity (%):", "hum"),
                                ("Pressure (inHg):", "press")]:
            row = ttk.Frame(sf2)
            row.pack(fill=tk.X, padx=4, pady=1)
            ttk.Label(row, text=label_text, width=18, anchor=tk.W).pack(side=tk.LEFT)
            e = ttk.Entry(row, textvariable=self.sae_vars[key], width=8)
            e.pack(side=tk.LEFT)
            e.bind("<Return>", lambda ev: self._refresh_analysis())
        self.sae_factor_label = ttk.Label(sf2, text="", foreground="gray")
        self.sae_factor_label.pack(anchor=tk.W, padx=4, pady=1)

        ttk.Button(ctrl, text="Apply / Refresh", command=self._refresh_analysis).pack(
            fill=tk.X, padx=4, pady=4)
        ttk.Button(ctrl, text="Export Filtered CSV…", command=self._export_filtered).pack(
            fill=tk.X, padx=4, pady=2)

        self.analysis_peaks = ttk.Label(ctrl, text="", font=("Consolas", 10),
                                        justify=tk.LEFT, anchor=tk.W)
        self.analysis_peaks.pack(fill=tk.X, padx=4, pady=6)

        # Plot
        plot_frame = ttk.Frame(parent)
        plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=4, pady=4)
        self.an_fig = Figure(figsize=(7, 4.6), dpi=100)
        self.an_ax_t = self.an_fig.add_subplot(111)
        self.an_ax_hp = self.an_ax_t.twinx()
        self.an_ax_t.set_xlabel("RPM")
        self.an_ax_t.set_title("Filtered vs Raw — Torque & HP vs RPM")
        self.an_ax_t.grid(True, alpha=0.3)
        self.an_canvas = FigureCanvasTkAgg(self.an_fig, master=plot_frame)
        self.an_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        toolbar = NavigationToolbar2Tk(self.an_canvas, plot_frame)
        toolbar.update()

    # ══════════════════════════════════════════════════════════
    # Source mode handling
    # ══════════════════════════════════════════════════════════
    def _set_widget_tree_state(self, widget, state):
        """Recursively set state on every leaf widget under `widget`."""
        for w in widget.winfo_children():
            try:
                w.config(state=state)
            except tk.TclError:
                pass
            self._set_widget_tree_state(w, state)

    def _update_source_controls(self):
        """Enable/disable serial vs replay controls based on the selected source."""
        is_hw = self.source_var.get() == "hardware"
        hw_state = tk.NORMAL if is_hw else tk.DISABLED
        rp_state = tk.NORMAL if not is_hw else tk.DISABLED
        # Serial controls
        self.port_combo.config(state=("readonly" if is_hw else tk.DISABLED))
        self.refresh_btn.config(state=hw_state)
        self.connect_btn.config(state=hw_state)
        # Replay controls (recurse into the transport/motor sub-rows)
        self._set_widget_tree_state(self.replay_bar, rp_state)
        # If switching to replay while connected, drop the serial link.
        if not is_hw and self.ser and self.ser.is_open:
            self._disconnect()
        # If switching to hardware while replaying, stop replay.
        if is_hw and self.replay_running:
            self._replay_stop()

    def _on_units_changed(self):
        units = self.units_var.get()
        self.ax_pwr.set_ylabel(f"Torque ({units}) / HP", color="tab:red")
        if hasattr(self, "torque_label_text"):
            self.torque_label_text.config(text=f"Torque ({units}):")
        self._live_bg = None          # y-label/scale changed → bg stale
        self._plot_dirty = True
        self._refresh_analysis()

    def _torque_display(self, torque_nm):
        """Convert a torque value/array (assumed Nm) to the selected display unit."""
        if self.units_var.get() == "lb-ft":
            return dsp.nm_to_lbft(torque_nm)
        return torque_nm

    def _mark_plot_dirty(self):
        # Smoothing on/off or a new α rewrites the whole smoothed curve (not just
        # an appended point), so force a clean full redraw to avoid blit ghosting.
        self._live_bg = None
        self._plot_dirty = True

    # ══════════════════════════════════════════════════════════
    # Serial Helpers
    # ══════════════════════════════════════════════════════════
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
        self._refresh_motor_ports()

    def _refresh_motor_ports(self):
        """Populate the replay 'ESP port' dropdown (built after the main combo)."""
        if not hasattr(self, "replay_motor_port_combo"):
            return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.replay_motor_port_combo["values"] = ports
        if ports and not self.replay_motor_port_var.get():
            self.replay_motor_port_var.set(ports[0])

    def _toggle_connection(self):
        if self.ser and self.ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No Port", "Select a serial port first.")
            return
        self.connect_btn.config(state=tk.DISABLED)
        self.conn_status.config(text="Connecting...", foreground="orange")
        threading.Thread(target=self._connect_thread, args=(port,), daemon=True).start()

    def _connect_thread(self, port: str):
        try:
            s = serial.Serial()
            s.port = port
            s.baudrate = BAUD_RATE
            s.timeout = SERIAL_TIMEOUT
            s.dsrdtr = False
            s.rtscts = False
            # Keep DTR/RTS deasserted so an asserted RTS can't hold the ESP32 in
            # reset (EN stays high) — otherwise no DATA / no command handling.
            s.dtr = False
            s.rts = False
            s.open()
            s.dtr = False
            s.rts = False
            time.sleep(1.5)  # Wait for ESP32 to stabilize after connect
            s.reset_input_buffer()
            self.ser = s
            self.running = True
            self.recorded_torque_is_nm = True
            self.serial_thread = threading.Thread(target=self._serial_reader, daemon=True)
            self.serial_thread.start()
            self.root.after(0, lambda: self._on_connected(port))
        except Exception as e:
            self.root.after(0, lambda: self._on_connect_fail(str(e)))

    def _on_connected(self, port: str):
        self.connect_btn.config(text="Disconnect", state=tk.NORMAL)
        self.conn_status.config(text=f"Connected ({port})", foreground="green")
        self._log_event(f"Connected to {port}", "ack")
        self._send("READY?")
        # Arm the pressure channel with the datasheet calibration straight away.
        # It used to wait for someone to click, which left the hose interlock
        # disabled and the panel showing a convincing 0.0 PSI in the meantime.
        self.root.after(150, self._send_pressure_cfg)
        # Pull the board's current settings so the Calibration tab reflects
        # what is actually loaded rather than the GUI's defaults.
        self.root.after(400, lambda: self._send("STATUS"))

    def _on_connect_fail(self, err: str):
        self.connect_btn.config(state=tk.NORMAL)
        self.conn_status.config(text="Failed", foreground="red")
        messagebox.showerror("Serial Error", err)

    def _disconnect(self):
        self.running = False
        if self.serial_thread:
            self.serial_thread.join(timeout=2)
        if self.ser:
            self.ser.close()
            self.ser = None
        # Clear any (auto-)recording state so the Record button doesn't show a
        # stale "■ Stop Rec" on a disconnected source.
        self.recording = False
        self.auto_recording = False
        self._got_ready = False
        self._mon_t0 = None
        self.connect_btn.config(text="Connect")
        self.conn_status.config(text="Disconnected", foreground="red")

    def _send(self, cmd: str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd + "\n").encode("ascii"))
            except serial.SerialException:
                pass

    # ══════════════════════════════════════════════════════════
    # Serial Reader Thread
    # ══════════════════════════════════════════════════════════
    def _serial_reader(self):
        while self.running:
            try:
                if not self.ser or not self.ser.is_open:
                    break
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="replace").strip()
                if not line:
                    continue
                self._parse_line(line)
            except serial.SerialException:
                break
            except Exception:
                continue

    def _parse_line(self, line: str):
        # ---- DATA line (20 Hz) ----
        if line.startswith("DATA,"):
            parts = line.split(",")
            if len(parts) < 13:
                return
            try:
                rpm       = float(parts[2])
                torque    = float(parts[3])
                load_raw  = float(parts[4])
                adc0      = float(parts[5])
                adc1      = float(parts[6])
                brake_pos = int(float(parts[7]))
                target    = float(parts[8])
                state     = parts[9]
                pid_p     = float(parts[10])
                pid_i     = float(parts[11])
                pid_out   = float(parts[12])
            except (ValueError, IndexError):
                return

            # Fields 13-15 are only present on newer firmware.
            def opt(i, default=0.0):
                try:
                    return float(parts[i])
                except (ValueError, IndexError):
                    return default
            brake_pct = opt(13)
            press_psi = opt(14)
            faults    = int(opt(15))
            spare_aux = opt(16)
            glitches  = int(opt(17))
            estimated = int(opt(18))    # samples the board filled in itself
            # Reserved for the encoder. Always 0 until the hardware exists,
            # so the frame shape will not change when it is fitted.
            enc_pos   = int(opt(19))
            enc_ok    = int(opt(20))

            with self._lock:
                self.live.update({
                    "rpm": rpm, "torque": torque, "load_raw": load_raw,
                    "adc0": adc0, "adc1": adc1, "brake_pos": brake_pos,
                    "target_rpm": target, "state": state,
                    "pid_p": pid_p, "pid_i": pid_i, "pid_out": pid_out,
                    "brake_pct": brake_pct, "press_psi": press_psi,
                    "faults": faults,
                    # Field 6 is the pressure sensor's own millivolts now that it
                    # has a dedicated ADS1115; field 16 is the old DFRobot spare.
                    "press_mv": adc1, "spare_aux": spare_aux,
                    "glitches": glitches, "estimated": estimated,
                    "enc_pos": enc_pos, "enc_ok": enc_ok,
                })

            # Feed the rolling monitor on every frame, recording or not.
            hp_live = torque * rpm / HP_FACTOR if rpm > 0 else 0.0
            t_board = float(parts[1]) / 1000.0
            if self._mon_t0 is None:
                self._mon_t0 = t_board
            with self._lock:
                self.mon_t.append(t_board)
                self.mon_rpm.append(rpm)
                self.mon_torque.append(torque)
                self.mon_hp.append(hp_live)
                self.mon_psi.append(press_psi)
                self.mon_brake.append(float(brake_pos))
            self._plot_dirty = True

            if self._tare_collecting:
                # Plain list append is safe from the reader thread, and this way
                # we get exactly one sample per frame rather than whatever the
                # main loop happens to catch.
                self._tare_samples.append(load_raw)

            if self.recording:
                if len(self.run_rpm) >= MAX_LOG_POINTS:
                    # Say so rather than quietly dropping samples, as before.
                    if not self._log_capped_warned:
                        self._log_capped_warned = True
                        self._log_event(
                            f"Log full at {MAX_LOG_POINTS} points — recording stopped",
                            "err")
                    return
                hp = torque * rpm / HP_FACTOR if rpm > 0 else 0.0
                # Append the run channels under the lock so _redraw_live_plot
                # (main thread) never observes mismatched list lengths.
                with self._lock:
                    self.run_t.append(float(parts[1]) / 1000.0)
                    self.run_rpm.append(rpm)
                    self.run_torque.append(torque)
                    self.run_hp.append(hp)
                    self.run_psi.append(press_psi)
                    self.run_brake.append(float(brake_pos))
                    self.log_rows.append([
                        parts[1], f"{rpm:.1f}", f"{torque:.2f}", f"{hp:.1f}",
                        f"{load_raw:.1f}", f"{adc0:.1f}", f"{adc1:.1f}",
                        str(brake_pos), f"{target:.1f}", state,
                        f"{pid_p:.2f}", f"{pid_i:.2f}", f"{pid_out:.2f}",
                        f"{brake_pct:.1f}", f"{press_psi:.1f}", str(faults),
                        f"{spare_aux:.1f}",
                        str(glitches), str(estimated),
                    ])
                self._plot_dirty = True

        # ---- READY status ----
        elif line.startswith("READY,"):
            parts = line.split(",")
            if len(parts) >= 5:
                with self._lock:
                    self.ready_flags["ready"] = parts[1] == "1"
                    self.ready_flags["homed"] = parts[2] == "1"
                    self.ready_flags["tared"] = parts[3] == "1"
                    self.ready_flags["adc"]   = parts[4] == "1"
                    # Firmware without the sim field is old enough that it also
                    # defaulted to SIM — keep treating it as suspect.
                    self.ready_flags["sim"] = (parts[5] == "1") if len(parts) >= 6 else True
                    self.ready_flags["press_adc"] = (parts[6] == "1") if len(parts) >= 7 else False
                    self._got_ready = True

        # ---- ACK lines for lifecycle events ----
        elif line.startswith("ACK,"):
            msg = line[4:]
            # Echoes of our own commands are noise; only log real events.
            if msg.isupper() and "," not in msg:
                self._log_event(f"ACK  {msg}", "ack")
            if msg == "SWEEP_STARTED":
                self._auto_record_start()
            elif msg == "SWEEP_COMPLETE":
                self._auto_record_stop()

        # ---- Errors: previously discarded, which made a refused START look
        #      identical to a frozen GUI ----
        elif line.startswith("ERR,"):
            msg = line[4:]
            self.last_error = msg
            self._log_event(f"ERR  {msg}", "err")

        # ---- Config replies to STATUS ----
        elif line.startswith("CFG,"):
            if line.startswith("CFG,FW_VERSION,"):
                bits = line.split(",")
                self.fw_version = bits[2].strip() if len(bits) > 2 else "unknown"
                self.fw_build = bits[3].strip() if len(bits) > 3 else ""
            self._log_event(line, "cfg")

        # ---- Firmware boot / debug chatter ----
        elif line.startswith("[") or line.startswith("DIY_DYNO"):
            self._log_event(line, "ack")

    # ══════════════════════════════════════════════════════════
    # Dummy Data — CSV Replay
    # ══════════════════════════════════════════════════════════
    def _replay_choose_file(self):
        path = filedialog.askopenfilename(
            title="Select a recorded dyno log / CSV / Excel run to replay",
            filetypes=[("CSV / log / Excel files", "*.csv *.txt *.xlsx *.xlsm"),
                       ("All files", "*.*")],
        )
        if not path:
            return
        self._replay_load(path)

    def _replay_load(self, path):
        try:
            rec = dsp.load_recording(path)
        except Exception as e:
            messagebox.showerror("Load Error", f"Could not load:\n{path}\n\n{e}")
            return
        if rec.rpm is None or rec.torque is None:
            messagebox.showerror(
                "No RPM/Torque",
                f"This file has no RPM/Torque channel to replay (format: {rec.fmt}).\n"
                "Pressure logs and similar cannot be replayed as a pull.")
            return
        self.replay_rec = rec
        self.replay_file_var.set(os.path.basename(path))
        self.replay_info.config(
            text=f"{rec.fmt}, {rec.n} pts, {rec.duration:.1f}s @ {rec.rate_hz:.0f}Hz")

    # ── Brake-motor reaction during replay ───────────────────
    def _motor_open(self):
        """Open the ESP serial (if enabled) and set a gentle stepper speed so the
        brake reacts slowly to the replay instead of flying around."""
        self.motor_ser = None
        if not self.replay_motor_var.get():
            return
        port = self.replay_motor_port_var.get().strip()
        if not port:
            return
        try:
            s = serial.Serial()
            s.port = port
            s.baudrate = BAUD_RATE
            s.timeout = 0.1
            s.dsrdtr = False
            s.rtscts = False
            # Keep DTR/RTS deasserted so EN stays high — otherwise an asserted RTS
            # holds the ESP32 in reset and it never processes the BRAKE commands.
            s.dtr = False
            s.rts = False
            s.open()
            s.dtr = False
            s.rts = False               # reassert after open
            time.sleep(0.8)             # settle
            s.reset_input_buffer()
            for cmd in (f"STEPPER_SPEED,{MOTOR_DEMO_SPEED}",
                        f"STEPPER_ACCEL,{MOTOR_DEMO_ACCEL}", "BRAKE,0"):
                s.write((cmd + "\n").encode("ascii"))
            self.motor_ser = s
        except Exception as e:
            self.motor_ser = None
            messagebox.showwarning(
                "Brake motor",
                f"Could not open {port} to drive the brake motor:\n{e}\n\n"
                "Replay will continue without motor motion.")

    def _motor_update(self, rpm):
        """Send a scaled-down BRAKE command tracking the current RPM (throttled)."""
        s = self.motor_ser
        if not s:
            return
        now = time.monotonic()
        if now - self._motor_last_send < 1.0 / MOTOR_SEND_HZ:
            return
        self._motor_last_send = now
        try:
            gain = float(self.replay_motor_gain_var.get()) / 100.0
        except ValueError:
            gain = 0.08
        gain = min(max(gain, 0.0), 1.0)
        maxrpm = self._motor_max_rpm if self._motor_max_rpm > 1 else 8000.0
        frac = min(max(rpm / maxrpm, 0.0), 1.0)
        steps = int(frac * gain * MOTOR_FULL_STEPS)
        try:
            s.reset_input_buffer()      # discard the ESP's incoming DATA stream
            s.write((f"BRAKE,{steps}\n").encode("ascii"))
        except Exception:
            pass

    def _motor_close(self):
        """Release the brake and close the ESP serial."""
        s = self.motor_ser
        self.motor_ser = None
        if s:
            try:
                s.write(b"BRAKE,0\n")
                time.sleep(0.05)
                s.close()
            except Exception:
                pass

    def _replay_play(self):
        if self.replay_running:
            return
        if self.replay_rec is None:
            messagebox.showinfo("No File", "Load a CSV / log file first.")
            return
        try:
            speed = max(0.05, float(self.replay_speed_var.get()))
        except ValueError:
            speed = 1.0
        # Fresh plot + auto-record so the curve builds live, like a real sweep.
        self._clear_plot()
        self.recorded_torque_is_nm = self.replay_rec.torque_is_nm
        self.recording = True
        self.auto_recording = True
        self.replay_running = True
        self.replay_paused = False
        self.replay_play_btn.config(state=tk.DISABLED)
        self.replay_pause_btn.config(state=tk.NORMAL, text="❚❚ Pause")
        self.replay_stop_btn.config(state=tk.NORMAL)
        # Set up the (scaled) brake-motor reaction, keyed off the run's max RPM.
        try:
            self._motor_max_rpm = float(np.nanmax(self.replay_rec.rpm))
        except (ValueError, TypeError):
            self._motor_max_rpm = 8000.0
        if not np.isfinite(self._motor_max_rpm) or self._motor_max_rpm < 1:
            self._motor_max_rpm = 8000.0
        self._motor_last_send = 0.0
        self._motor_open()
        self.replay_thread = threading.Thread(
            target=self._replay_worker, args=(speed,), daemon=True)
        self.replay_thread.start()

    def _replay_toggle_pause(self):
        if not self.replay_running:
            return
        self.replay_paused = not self.replay_paused
        self.replay_pause_btn.config(text="▶ Resume" if self.replay_paused else "❚❚ Pause")

    def _replay_stop(self):
        self.replay_running = False
        self.replay_paused = False
        if self.replay_thread and self.replay_thread is not threading.current_thread():
            self.replay_thread.join(timeout=2)
        self.recording = False
        self.auto_recording = False
        self._motor_close()
        self.root.after(0, self._replay_reset_buttons)

    def _replay_reset_buttons(self):
        self.replay_play_btn.config(state=tk.NORMAL)
        self.replay_pause_btn.config(state=tk.DISABLED, text="❚❚ Pause")
        self.replay_stop_btn.config(state=tk.DISABLED)

    def _replay_worker(self, speed):
        """Stream the recording through _parse_line at real-time × speed."""
        # Decimate to a ~25 Hz frame cadence (matches firmware DATA rate).
        t, rpm, torque = dsp.build_replay_samples(self.replay_rec, target_hz=25.0)
        n = t.size
        wall0 = time.monotonic()
        i = 0
        while i < n and self.replay_running:
            if self.replay_paused:
                wall0 += 0.05            # freeze the virtual clock while paused
                time.sleep(0.05)
                continue
            # Pace to the recording's own timeline, scaled by playback speed.
            target_wall = wall0 + (t[i] / speed)
            now = time.monotonic()
            if target_wall > now:
                time.sleep(min(target_wall - now, 0.1))
                continue
            millis = int(t[i] * 1000.0)
            line = (f"DATA,{millis},{rpm[i]:.1f},{torque[i]:.3f},"
                    f"{torque[i]:.1f},0.0,0.0,0,{rpm[i]:.1f},REPLAY,0.0,0.0,0.0")
            self._parse_line(line)
            self._motor_update(rpm[i])       # nudge the real brake (scaled/gentle)
            self.root.after(0, lambda f=(i + 1) / n: self.replay_progress_var.set(f * 100.0))
            i += 1
        # Finished or stopped
        if self.replay_running:
            self.replay_running = False
            self.root.after(0, self._on_replay_finished)

    def _on_replay_finished(self):
        self.recording = False
        self.auto_recording = False
        self._motor_close()
        self._replay_reset_buttons()
        self.replay_progress_var.set(100.0)
        # Make the just-replayed run immediately available for analysis.
        self._send_run_to_analysis(switch_tab=False)

    # ══════════════════════════════════════════════════════════
    # Auto-recording tied to sweep lifecycle
    # ══════════════════════════════════════════════════════════
    def _auto_record_start(self):
        if not self.recording:
            self.recording = True
            self.auto_recording = True

    def _auto_record_stop(self):
        if self.auto_recording:
            self.recording = False
            self.auto_recording = False
            # Hop to the main thread: this reads tk variables, and the sweep
            # lifecycle arrives on the serial reader thread.
            self.root.after(0, self._autosave_run)

    # ══════════════════════════════════════════════════════════
    # GUI Update Timer (runs on main thread)
    # ══════════════════════════════════════════════════════════
    def _schedule_gui_update(self):
        if not self._updating:
            self._updating = True
            try:
                self._update_gui()
            except Exception:
                pass
            finally:
                self._updating = False
        self.root.after(GUI_UPDATE_MS, self._schedule_gui_update)

    def _live_alpha(self):
        try:
            return float(np.clip(float(self.live_alpha_var.get()), 0.0, 1.0))
        except ValueError:
            return 0.3

    def _schedule_status_poll(self):
        """Re-ask READY? so the SIM/LIVE badge can't sit stale after a change."""
        if self.ser and self.ser.is_open:
            self._send("READY?")
        # Persist settings as they change, so a power cut mid-session does not
        # lose an afternoon of calibration. Writes only on an actual change.
        self._save_session()
        self.root.after(STATUS_POLL_MS, self._schedule_status_poll)

    def _update_mode_indicator(self):
        connected = bool(self.ser and self.ser.is_open)
        if not connected or not self._got_ready:
            self.sim_label.config(text="MODE ?", background="#7F8C8D")
            self.sim_btn.config(text="Use LIVE", state=tk.DISABLED)
            return
        self.sim_btn.config(state=tk.NORMAL)
        if self.ready_flags.get("sim", True):
            self.sim_label.config(text="SIM — SYNTHETIC DATA", background="#A66A12")
            self.sim_btn.config(text="Use LIVE")
        else:
            self.sim_label.config(text="LIVE", background="#1E6B4F")
            self.sim_btn.config(text="Use SIM")

    def _update_alerts(self, faults):
        """Active faults outrank the last error; both outrank an empty bar."""
        active = [FAULT_NAMES[bit] for bit in (FAULT_TACH, FAULT_PRESSURE)
                  if faults & bit]
        if active:
            self._show_alert("   ".join(active))
        elif self.last_error:
            self._show_alert(f"Controller: {self.last_error}")
        elif self.alert_var.get():
            self._clear_alert()

    def _update_gui(self):
        with self._lock:
            d = dict(self.live)
            rf = dict(self.ready_flags)

        rpm = d["rpm"]
        torque = d["torque"]
        state = d["state"]
        hp = torque * rpm / HP_FACTOR if rpm > 0 else 0.0

        torque_disp = self._torque_display(torque)
        self.status_labels["rpm"].config(text=f"{rpm:.0f}")
        self.status_labels["target_rpm"].config(text=f"{d['target_rpm']:.0f}")
        self.status_labels["torque"].config(text=f"{torque_disp:.2f}")
        self.status_labels["hp"].config(text=f"{hp:.1f}")
        self.status_labels["brake_pct"].config(text=f"{d['brake_pct']:.1f}")
        self.status_labels["brake_pos"].config(text=str(d["brake_pos"]))
        self.status_labels["press_psi"].config(text=f"{d['press_psi']:.1f}")
        self.status_labels["state"].config(text=state)
        # Noise rejected at the interrupt. Climbing means the pickup line is
        # picking up interference, not that the engine is doing anything.
        gl = int(d.get("glitches", 0))
        self.status_labels["glitches"].config(
            text=str(gl), foreground=("#B03A2E" if gl else "black"))
        # How much of the trace was filled in by the board rather than
        # measured. Amber, not red: it is working as asked, but the operator
        # should know a rising number means estimated data in the run.
        es = int(d.get("estimated", 0))
        self.status_labels["estimated"].config(
            text=str(es), foreground=("#B9770E" if es else "black"))
        # Firmware too old to answer VERSION is flagged rather than left
        # blank: it also predates fields this GUI expects.
        lag = self._rpm_avg_lag_s()
        if lag is None:
            self.rpm_lag_label.config(text="", foreground="gray")
        else:
            # A control loop answering half-second-old RPM is already sluggish;
            # past a second it is fighting the engine rather than following it.
            self.rpm_lag_label.config(
                text=f"= {lag:.2f} s of lag into the PID at this RPM"
                     + ("  - too slow to control on" if lag > 0.5 else ""),
                foreground=("#B03A2E" if lag > 0.5 else "gray"))
        self.status_labels["fw_version"].config(
            text=self.fw_version,
            foreground=("#B9770E" if self.fw_version == "unknown" else "black"))
        # Reads 'not installed' until a board reports a live encoder, so the
        # panel never implies a measurement that is not being taken.
        if d.get("enc_ok"):
            self.enc_pos_label.config(text=str(d.get("enc_pos", 0)),
                                      foreground="black")
        else:
            self.enc_pos_label.config(text="not installed", foreground="gray")
        self.status_labels["load_raw"].config(text=f"{d['load_raw']:.1f}")
        # "At rail" rather than a number: the 0-10 V module clamps at zero, so a
        # reading of 0 is the bottom of the range, not a measurement.
        if d["load_raw"] <= 0.0:
            net_txt = "AT RAIL"
        elif self.load_zero_mv is None:
            net_txt = "--"
        else:
            net_txt = f"{d['load_raw'] - self.load_zero_mv:.1f}"
        self.status_labels["load_net"].config(text=net_txt)
        self.status_labels["press_mv"].config(text=f"{d['press_mv']:.1f}")

        # Mirror the readings the calibration tab needs
        if hasattr(self, "calib_live_labels"):
            self.calib_live_labels["load_raw"].config(text=f"{d['load_raw']:.2f}")
            self.calib_live_labels["load_net"].config(
                text=net_txt if net_txt != "--" else "--")
            self.calib_live_labels["press_mv"].config(text=f"{d['press_mv']:.2f}")
            self.calib_live_labels["press_psi"].config(text=f"{d['press_psi']:.1f}")
            self.calib_live_labels["torque"].config(text=f"{torque_disp:.2f}")

        self.pid_labels["pid_p"].config(text=f"{d['pid_p']:.2f}")
        self.pid_labels["pid_i"].config(text=f"{d['pid_i']:.2f}")
        self.pid_labels["pid_out"].config(text=f"{d['pid_out']:.2f}")

        is_ready = rf.get("ready", False)
        if is_ready:
            self.ready_label.config(text="READY", background="green", foreground="white")
        else:
            self.ready_label.config(text="NOT READY", background="red", foreground="white")

        self._update_mode_indicator()
        self._update_alerts(d.get("faults", 0))
        self._drain_events()

        for key in ["homed", "tared", "adc", "press_adc"]:
            lbl = self.ready_checks[key]
            if rf.get(key, False):
                lbl.config(text="✓", foreground="green")
            else:
                lbl.config(text="✗", foreground="red")

        # Running / Completed status + button enable
        if state == "IDLE":
            self.run_status_var.set("Idle")
            self.start_btn.config(state=tk.NORMAL)
            self.release_btn.config(state=tk.DISABLED)
        elif state == "HOLD_RPM":
            self.run_status_var.set("Holding RPM — throttle up then RELEASE")
            self.start_btn.config(state=tk.DISABLED)
            self.release_btn.config(state=tk.NORMAL)
        elif state == "SWEEP":
            self.run_status_var.set("Running sweep...")
            self.start_btn.config(state=tk.DISABLED)
            self.release_btn.config(state=tk.DISABLED)
        elif state == "SWEEP_DONE":
            self.run_status_var.set("Completed — throttle down to reset")
            self.start_btn.config(state=tk.DISABLED)
            self.release_btn.config(state=tk.DISABLED)
        elif state == "HOMING":
            self.run_status_var.set("Homing stepper...")
            self.start_btn.config(state=tk.DISABLED)
            self.release_btn.config(state=tk.DISABLED)
        elif state == "MANUAL":
            self.run_status_var.set("Manual brake control")
            self.start_btn.config(state=tk.NORMAL)
            self.release_btn.config(state=tk.DISABLED)
        elif state == "RAMP_DOWN":
            self.run_status_var.set("Bringing RPM down — keep off the throttle")
            self.start_btn.config(state=tk.DISABLED)
            self.release_btn.config(state=tk.DISABLED)
        elif state == "REPLAY":
            self.run_status_var.set("Replaying recorded data (Dummy mode)…")
            self.start_btn.config(state=tk.DISABLED)
            self.release_btn.config(state=tk.DISABLED)

        self.rec_btn.config(text="■ Stop Rec" if self.recording else "● Record")
        self.saved_label.config(
            text=f"Last saved: {os.path.basename(self.last_saved_run)}"
            if self.last_saved_run else "")

        # Update plot only when new recording data was added
        if self._plot_dirty and (self.run_rpm or self.mon_rpm):
            self._plot_dirty = False
            self._redraw_live_plot()

    def _on_canvas_resize(self, event):
        # Only invalidate on an actual size change. XWayland can emit spurious
        # <Configure> events at the same size, which would force a slow full
        # redraw every frame and defeat blitting.
        size = (event.width, event.height)
        if size != self._live_canvas_size:
            self._live_canvas_size = size
            self._live_bg = None

    @staticmethod
    def _axis_ceiling(arr, step, factor=1.12):
        """(lo, hi) with hi snapped UP to a multiple of `step` above the data.

        Snapping to a coarse step means the limit only changes every so often as
        the curve grows, so most frames stay within the current limits and can
        blit instead of doing a full (slow) redraw. lo always includes 0.
        """
        arr = np.asarray(arr, dtype=float)
        arr = arr[np.isfinite(arr)]
        if arr.size == 0:
            return (0.0, step)
        lo = min(0.0, float(arr.min()))
        hi = float(arr.max())
        hi = max(hi * factor, hi + step * 0.25, step)   # headroom; never < one step
        hi = float(np.ceil(hi / step) * step)
        return (lo, hi)

    def _live_limits_contain(self, x, left_all, right_all, psi_all=None,
                             brk_all=None):
        """True if all data still fits inside the cached axis limits."""
        if self._live_xlim is None:
            return False

        def within(arr, lim):
            arr = np.asarray(arr, dtype=float)
            arr = arr[np.isfinite(arr)]
            return arr.size == 0 or (arr.min() >= lim[0] and arr.max() <= lim[1])

        ok = (within(x, self._live_xlim) and within(left_all, self._live_lylim)
              and within(right_all, self._live_rylim))
        if psi_all is not None:
            ok = ok and self._live_psilim is not None and within(psi_all,
                                                                self._live_psilim)
        if brk_all is not None:
            ok = ok and self._live_brklim is not None and within(brk_all,
                                                                self._live_brklim)
        return ok

    def _redraw_live_plot(self):
        # Snapshot all channels together under the lock so a concurrent worker
        # append can't yield mismatched lengths (→ set_data ValueError).
        # A recorded run shows the whole run; otherwise show a rolling window of
        # the last MONITOR_WINDOW_S seconds so the engine is always visible.
        recording = self.recording and bool(self.run_rpm)
        with self._lock:
            if recording:
                t = np.asarray(self.run_t, dtype=float)
                rpm = np.asarray(self.run_rpm, dtype=float)
                tq_raw = np.asarray(self.run_torque, dtype=float)
                hp = np.asarray(self.run_hp, dtype=float)
                psi = np.asarray(self.run_psi, dtype=float)
                brk = np.asarray(self.run_brake, dtype=float)
            else:
                t = np.asarray(self.mon_t, dtype=float)
                rpm = np.asarray(self.mon_rpm, dtype=float)
                tq_raw = np.asarray(self.mon_torque, dtype=float)
                hp = np.asarray(self.mon_hp, dtype=float)
                psi = np.asarray(self.mon_psi, dtype=float)
                brk = np.asarray(self.mon_brake, dtype=float)

        source = "run" if recording else "monitor"
        if source != self._plot_source:
            # Switching source changes the whole curve; force a clean full
            # redraw so the blit cache cannot ghost the previous one.
            self._plot_source = source
            self._live_bg = None
            self.ax_rpm.set_title("Recording run — RPM, Torque, HP, PSI & Brake"
                                  if recording else
                                  f"Live — last {MONITOR_WINDOW_S}s (not recording)")

        # Align the channels before anything indexes them: a mask built from t
        # must never be applied to a channel of a different length.
        n = min(t.size, rpm.size, tq_raw.size, hp.size, psi.size, brk.size)
        t, rpm, tq_raw, hp, psi, brk = (t[:n], rpm[:n], tq_raw[:n], hp[:n],
                                        psi[:n], brk[:n])

        if not recording and t.size:
            # Elapsed since monitoring began, not since the window started, so the
            # axis scrolls (60-120, 120-180...) instead of sitting at 0-60 forever.
            t = t - (self._mon_t0 if self._mon_t0 is not None else t[0])
            # Snap the window to whole 5 s so it is not recomputed every frame,
            # then trim the data to it so blitting stays valid between snaps.
            hi = float(np.ceil(t[-1] / 5.0) * 5.0)
            # Clamp at zero so the axis grows from 0 while the buffer is still
            # filling, rather than showing negative time for the first minute.
            lo = max(0.0, hi - MONITOR_WINDOW_S)
            keep = t >= lo
            t, rpm, tq_raw, hp, psi, brk = (t[keep], rpm[keep], tq_raw[keep],
                                            hp[keep], psi[keep], brk[keep])
            self._mon_xlim = (lo, hi)
        else:
            self._mon_xlim = None
        n = t.size
        # Decimate for display only — the log keeps every sample. Without this a
        # long or high-rate run would push tens of thousands of points through
        # set_data every frame, which the Pi's display cannot keep up with.
        if n > PLOT_MAX_POINTS:
            stride = int(np.ceil(n / PLOT_MAX_POINTS))
            t, rpm = t[::stride], rpm[::stride]
            tq_raw, hp = tq_raw[::stride], hp[::stride]
            psi, brk = psi[::stride], brk[::stride]
            n = t.size
        if recording and t.size:
            t = t - t[0]                 # elapsed seconds from run start
        tq = self._torque_display(tq_raw)

        smooth_on = self.live_smooth_var.get() and n >= 3
        if smooth_on:
            a = self._live_alpha()
            rpm_s = dsp.ema(rpm, a, init=float(rpm[0]))
            tq_s = dsp.ema(tq, a, init=float(tq[0]))
            hp_s = dsp.ema(hp, a, init=float(hp[0]))
            psi_s = dsp.ema(psi, a, init=float(psi[0]))
            brk_s = dsp.ema(brk, a, init=float(brk[0]))
            self.line_rpm_raw.set_data(t, rpm)
            self.line_torque_raw.set_data(t, tq)
            self.line_hp_raw.set_data(t, hp)
            self.line_psi_raw.set_data(t, psi)
            self.line_brake_raw.set_data(t, brk)
        else:
            rpm_s, tq_s, hp_s, psi_s, brk_s = rpm, tq, hp, psi, brk
            self.line_rpm_raw.set_data([], [])
            self.line_torque_raw.set_data([], [])
            self.line_hp_raw.set_data([], [])
            self.line_psi_raw.set_data([], [])
            self.line_brake_raw.set_data([], [])
        self.line_rpm.set_data(t, rpm_s)
        self.line_torque.set_data(t, tq_s)
        self.line_hp.set_data(t, hp_s)
        self.line_psi.set_data(t, psi_s)
        self.line_brake.set_data(t, brk_s)

        # Blank whatever is switched off, and hide the axes that then have
        # nothing on them, so the plot is not squeezed by unused scales.
        vis = {k: v.get() for k, v in self.trace_vars.items()}
        if vis != self._trace_vis:
            self._trace_vis = vis
            self._live_bg = None            # layout changed, redraw in full
        for key, pair in (("rpm", (self.line_rpm, self.line_rpm_raw)),
                          ("torque", (self.line_torque, self.line_torque_raw)),
                          ("hp", (self.line_hp, self.line_hp_raw)),
                          ("psi", (self.line_psi, self.line_psi_raw)),
                          ("brake", (self.line_brake, self.line_brake_raw))):
            if not vis[key]:
                for ln in pair:
                    ln.set_data([], [])
        self.ax_psi.set_visible(vis["psi"])
        self.ax_brk.set_visible(vis["brake"])

        brk_all = (np.concatenate([brk, brk_s]) if (brk.size and vis["brake"])
                   else np.array([]))
        psi_all = (np.concatenate([psi, psi_s]) if (psi.size and vis["psi"])
                   else np.array([]))
        rpm_all = np.concatenate([rpm, rpm_s]) if rpm.size else rpm
        right_all = np.concatenate([tq, tq_s, hp, hp_s]) if tq.size else tq

        # Full (slow) redraw only when the background is stale — first draw, a
        # resize (handled by the canvas <Configure> bind), or the data grew past
        # the current axis ceilings. Every other frame just blits the six line
        # artists over the cached background (~10x cheaper → smooth on XWayland).
        need_full = (self._live_bg is None
                     or not self._live_limits_contain(t, rpm_all, right_all,
                                                      psi_all, brk_all))
        if need_full:
            self._live_xlim = (self._mon_xlim if self._mon_xlim
                               else self._axis_ceiling(t, 5.0))
            self._live_lylim = self._axis_ceiling(rpm_all, 1000.0)
            self._live_rylim = self._axis_ceiling(right_all, 25.0)
            self._live_psilim = self._axis_ceiling(psi_all, 250.0)
            self._live_brklim = self._axis_ceiling(brk_all, 50.0)
            self.ax_rpm.set_xlim(*self._live_xlim)
            self.ax_rpm.set_ylim(*self._live_lylim)
            self.ax_pwr.set_ylim(*self._live_rylim)
            self.ax_psi.set_ylim(*self._live_psilim)
            self.ax_brk.set_ylim(*self._live_brklim)
            self.canvas.draw()
            self._live_bg = self.canvas.copy_from_bbox(self.fig.bbox)
        else:
            self.canvas.restore_region(self._live_bg)
            for ln in (self.line_rpm_raw, self.line_torque_raw, self.line_hp_raw,
                       self.line_psi_raw, self.line_brake_raw, self.line_rpm,
                       self.line_torque, self.line_hp, self.line_psi,
                       self.line_brake):
                if ln.axes.get_visible():
                    ln.axes.draw_artist(ln)
            self.canvas.blit(self.fig.bbox)

    # ══════════════════════════════════════════════════════════
    # Button Handlers (hardware)
    # ══════════════════════════════════════════════════════════
    def _on_home(self):
        self._send("HOME")

    TARE_WINDOW_MS = 1200          # averaged over roughly a second of frames

    def _on_tare(self):
        """Zero the load cell, averaged over a window, with visible feedback.

        A single sample is a poor zero on a channel that picks up axle vibration
        -- catch it on a spike and the whole run is offset. Averaging also gives
        a free read on how much noise the cell is seeing before a pull starts.
        """
        if self.source_var.get() != "hardware" or not (self.ser and self.ser.is_open):
            messagebox.showinfo("Not connected",
                                "Connect to the ESP32 before zeroing the load cell.")
            return
        if self._tare_collecting:
            return
        self._tare_samples = []
        self._tare_collecting = True
        self._send("TARE")                      # zeroes the torque the ESP reports
        self._log_event("Tare: sampling...", "ack")
        self.tare_label.config(text="Sampling...", foreground="gray")
        self.root.after(self.TARE_WINDOW_MS, self._finish_tare)

    def _finish_tare(self):
        self._tare_collecting = False
        vals = list(self._tare_samples)
        self._send("READY?")
        if not vals:
            self._log_event("Tare failed: no data from the controller", "err")
            self.last_error = "Tare failed - no data arriving from the controller"
            self.tare_label.config(text="No data", foreground="#B03A2E")
            return

        mean = sum(vals) / len(vals)
        spread = max(vals) - min(vals)
        railed = sum(1 for v in vals if v <= 0.0)
        self.load_zero_mv = mean

        self._log_event(f"Tare: zero {mean:.2f} mV, noise {spread:.2f} mV p-p "
                        f"over {len(vals)} samples", "ack")

        if railed:
            # The 0-10 V module cannot represent a negative input, so a reading
            # pinned at 0 is not a measurement -- it is the bottom of the range.
            self.tare_label.config(
                text=f"{mean:.1f} mV - AT RAIL", foreground="#B03A2E")
            self._log_event(f"Tare warning: {railed}/{len(vals)} samples sat at "
                            "0 mV - channel is at the bottom of its range", "err")
            self.last_error = ("Load cell is at the bottom of its 0-10 V range. "
                               "Zeroing here is meaningless - give the amplifier "
                               "some offset, or reverse the cell's signal pair.")
        else:
            self.tare_label.config(
                text=f"{mean:.1f} mV  (noise {spread:.1f} p-p)", foreground="gray")

    def _send_params(self):
        self._send(f"HOLD,{self.param_vars['hold_rpm'].get()}")
        start = self.param_vars["start_rpm"].get()
        end = self.param_vars["end_rpm"].get()
        rate = self.param_vars["rate"].get()
        self._send(f"SWEEP,{start},{end},{rate}")

    def _on_start(self):
        self._send_params()
        self._send("START")

    def _on_release(self):
        self._send("RELEASE")

    def _on_stop(self):
        self._send("STOP")

    def _apply_pid(self):
        h, s = self.pid_vars, self.pid_sweep_vars
        self._send(f"PID,{h['kp'].get()},{h['ki'].get()},{h['kd'].get()}")
        self._send(f"PID_SWEEP,{s['kp'].get()},{s['ki'].get()},{s['kd'].get()}")

    def _manual_brake(self):
        self._send(f"BRAKE,{self.brake_var.get()}")

    # ── Brake test slider ────────────────────────────────────
    def _brake_max_steps(self):
        try:
            return max(1, int(float(self.cfg_vars["brake_max"].get())))
        except (ValueError, KeyError):
            return 5000

    def _sync_brake_slider(self):
        """Track the configured range so the slider can't command an over-stroke."""
        top = self._brake_max_steps()
        self.brake_slider.configure(to=top)
        self.brake_slider_max_label.config(text=str(top))
        if self.brake_slider_var.get() > top:
            self.brake_slider_var.set(top)

    def _on_brake_slider(self, _value):
        steps = int(float(self.brake_slider_var.get()))
        top = self._brake_max_steps()
        self.brake_slider_label.config(
            text=f"{steps} steps · {steps / top * 100:.0f}%")
        now = time.monotonic()
        if now - self._brake_slider_last < 0.1:      # 10 Hz while dragging
            return
        self._brake_slider_last = now
        self.brake_var.set(str(steps))
        self._send(f"BRAKE,{steps}")

    def _brake_slider_commit(self):
        steps = int(float(self.brake_slider_var.get()))
        self.brake_var.set(str(steps))
        self._send(f"BRAKE,{steps}")

    def _toggle_record(self):
        self.recording = not self.recording
        self.auto_recording = False

    def _clear_plot(self):
        with self._lock:
            self.run_t.clear()
            self.run_rpm.clear()
            self.run_torque.clear()
            self.run_hp.clear()
            self.run_psi.clear()
            self.run_brake.clear()
            self.log_rows.clear()
        self._log_capped_warned = False
        self.last_saved_run = ""
        self._live_bg = None          # invalidate blit cache
        self._live_xlim = self._live_lylim = self._live_rylim = None
        self._live_psilim = None
        self._live_brklim = None
        for ln in (self.line_rpm, self.line_torque, self.line_hp, self.line_psi,
                   self.line_rpm_raw, self.line_torque_raw, self.line_hp_raw,
                   self.line_psi_raw, self.line_brake, self.line_brake_raw):
            ln.set_data([], [])
        for ax in (self.ax_rpm, self.ax_pwr, self.ax_psi, self.ax_brk):
            ax.relim()
            ax.autoscale_view()
        self.canvas.draw_idle()

    def _hp_from_display_torque(self, tq_disp, rpm):
        """Horsepower from torque in whatever unit is on screen, and actual RPM."""
        const = (dsp.HP_LBFT_CONST if self.units_var.get() == "lb-ft"
                 else dsp.HP_NM_CONST)
        rpm = np.asarray(rpm, dtype=float)
        return np.where(rpm > 0, np.asarray(tq_disp, dtype=float) * rpm / const, 0.0)

    def _filtered_per_sample(self):
        """Filtered torque for every recorded sample, in display units.

        Filtering is done in the RPM domain, because that is what makes a power
        curve meaningful. To line it up row-for-row with the per-sample log, the
        filtered curve is evaluated at each sample's own RPM. Returns None if
        there is not enough of a curve to interpolate.
        """
        if not self.run_rpm:
            return None
        self._send_run_to_analysis(switch_tab=False)
        result = self._compute_analysis_curve()
        if result is None:
            return None
        grid_rpm, _raw_tq, filt_tq, _raw_hp, _filt_hp = result
        grid = np.asarray(grid_rpm, dtype=float)
        filt = np.asarray(filt_tq, dtype=float)
        good = np.isfinite(grid) & np.isfinite(filt)
        if int(good.sum()) < 2:
            return None
        with self._lock:
            sample_rpm = np.asarray(self.run_rpm, dtype=float)
        return np.interp(sample_rpm, grid[good], filt[good])

    def _write_conditions(self, path, run_file):
        """Record every setting in force for this run, beside the data.

        Written as a loadable profile, so a run's exact setup can be restored
        months later by loading this file - not just read. The CFG lines the
        controller reported are included verbatim, so the firmware's own view is
        preserved even where it differs from what the GUI believes it sent.
        """
        conditions = {
            "_saved": time.strftime("%Y-%m-%d %H:%M:%S"),
            "_run_file": os.path.basename(run_file),
            "_note": ("Test conditions for the run above. This is also a valid "
                      "profile - load it to put the rig back exactly as it was."),
            "_ui_version": UI_VERSION,
            "_fw_version": self.fw_version,
            "_fw_build": self.fw_build,
            "_torque_units": self.units_var.get(),
            "_torque_calibrated": bool(self.recorded_torque_is_nm),
            "_samples": len(self.log_rows),
            "_brake_char_stall": self._char_stall_at,
            "_controller_cfg": [line for _tag, line in list(self.events)
                                if "CFG," in line][-40:],
        }
        conditions.update(self._profile_snapshot())
        with open(path, "w") as f:
            json.dump(conditions, f, indent=2)
        return path

    def _write_run_csv(self, path):
        """Write the recorded run to `path`: everything, raw and filtered.

        Target RPM, actual RPM, torque and horsepower lead the file, with the
        filtered torque and the horsepower derived from it beside their raw
        counterparts, so one file answers both "what did it measure" and "what
        does the curve say". Time_s is first so the file reloads cleanly - the
        loader takes column 0 as its time axis.
        """
        # Torque is written in the unit currently on screen, named in the header
        # so there is never a question. Uncalibrated data stays labelled native.
        unit = self.units_var.get() if self.recorded_torque_is_nm else "native"
        header = [
            "Time_s", "Timestamp_ms", "Target_RPM", "RPM",
            f"Torque_raw_{unit}", f"Torque_filt_{unit}", "HP_raw", "HP_filt",
            "LoadCell_mV", "ADC_Load_mV", "Pressure_mV",
            "Brake_Pos", "Brake_Pct", "Brake_PSI",
            "State", "PID_P", "PID_I", "PID_Output", "Fault_Bits", "Spare_Aux_mV",
            # Running totals, not per-sample flags. Where these step up is
            # exactly where the tach misbehaved and where a reading was
            # estimated rather than measured.
            "Tach_Glitches", "RPM_Estimated",
        ]

        filt = self._filtered_per_sample()      # None if no usable curve

        with self._lock:
            rows = list(self.log_rows)
            tq_native = np.asarray(self.run_torque, dtype=float)
            rpm_arr = np.asarray(self.run_rpm, dtype=float)
        n = min(len(rows), tq_native.size, rpm_arr.size)
        rows = rows[:n]

        tq_raw = np.asarray(self._torque_display(tq_native[:n]), dtype=float)
        hp_raw = self._hp_from_display_torque(tq_raw, rpm_arr[:n])
        if filt is not None and filt.size >= n:
            tq_filt = np.asarray(filt[:n], dtype=float)
            hp_filt = self._hp_from_display_torque(tq_filt, rpm_arr[:n])
        else:
            tq_filt = hp_filt = None

        t0 = float(rows[0][0]) / 1000.0 if rows else 0.0
        with open(path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            for i, r in enumerate(rows):
                # r: ms, rpm, torque, hp, load, adcload, press_mV, brakepos,
                #    target, state, p, i, out, brakepct, psi, faults, spare,
                #    glitches, estimated
                writer.writerow([
                    f"{float(r[0]) / 1000.0 - t0:.4f}", r[0], r[8], r[1],
                    f"{tq_raw[i]:.3f}",
                    f"{tq_filt[i]:.3f}" if tq_filt is not None else "",
                    f"{hp_raw[i]:.2f}",
                    f"{hp_filt[i]:.2f}" if hp_filt is not None else "",
                    r[4], r[5], r[6],
                    r[7], r[13], r[14],
                    r[9], r[10], r[11], r[12], r[15], r[16],
                    r[17] if len(r) > 17 else "",
                    r[18] if len(r) > 18 else "",
                ])
        self.last_saved_run = path
        return len(rows)

    def _autosave_run(self):
        """Write a finished sweep to the run folder with no operator action.

        Back-to-back pulls are the normal way this gets used, and a run that
        only exists in memory until someone remembers to click Save is a run
        that eventually gets lost.
        """
        if not self.cfg_vars["autosave"].get() or not self.log_rows:
            return
        folder = self.cfg_vars["data_dir"].get().strip() or DEFAULT_DATA_DIR
        prefix = self.cfg_vars["run_prefix"].get().strip() or "dyno_run"
        try:
            os.makedirs(folder, exist_ok=True)
            path = os.path.join(
                folder, f"{prefix}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
            n = self._write_run_csv(path)
        except OSError as e:
            self._log_event(f"Auto-save failed: {e}", "err")
            self.last_error = f"Auto-save failed: {e}"
            return
        self._log_event(f"Saved {n} raw rows to {os.path.basename(path)}", "ack")

        # Settings that produced this run, under the same timestamp.
        try:
            cpath = path[:-4] + "_conditions.json"
            self._write_conditions(cpath, path)
            self._log_event(f"Saved test conditions to {os.path.basename(cpath)}", "ack")
        except OSError as e:
            self._log_event(f"Conditions file failed: {e}", "err")

        # Save the filtered curve alongside the raw under the same timestamp, so
        # a run is a matched pair rather than raw-now-filter-later. Also loads
        # the finished run into the Analysis tab.
        try:
            self._send_run_to_analysis(switch_tab=False)
            fpath = path[:-4] + "_filtered.csv"
            m = self._write_filtered_csv(fpath)
            if m:
                self._log_event(f"Saved {m} filtered points to "
                                f"{os.path.basename(fpath)}", "ack")
            else:
                self._log_event("Filtered curve not saved - no usable curve", "err")
        except (OSError, ValueError) as e:
            self._log_event(f"Filtered save failed: {e}", "err")

    def _save_and_restart(self):
        """Save the current pull and clear down ready for the next one."""
        if not self.log_rows:
            messagebox.showinfo("Nothing recorded", "There is no run to save yet.")
            return
        if not self.cfg_vars["autosave"].get() or self.last_saved_run == "":
            self._save_csv()
        self._clear_plot()
        self._log_event("Cleared — ready for the next pull", "ack")

    def _save_csv(self):
        if not self.log_rows:
            messagebox.showinfo("No Data", "No recorded data to save.")
            return
        folder = self.cfg_vars["data_dir"].get().strip() or DEFAULT_DATA_DIR
        try:
            os.makedirs(folder, exist_ok=True)
        except OSError:
            folder = os.path.expanduser("~")
        prefix = self.cfg_vars["run_prefix"].get().strip() or "dyno_run"
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV Files", "*.csv")],
            initialdir=folder,
            initialfile=f"{prefix}_{time.strftime('%Y%m%d_%H%M%S')}.csv",
        )
        if not path:
            return
        try:
            n = self._write_run_csv(path)
            base = path[:-4] if path.lower().endswith(".csv") else path
            self._write_conditions(base + "_conditions.json", path)
            self._write_filtered_csv(base + "_filtered.csv")
        except OSError as e:
            messagebox.showerror("Could not save", str(e))
            return
        messagebox.showinfo("Saved", f"{n} rows saved to:\n{path}")

    # ══════════════════════════════════════════════════════════
    # Analysis & Filtering
    # ══════════════════════════════════════════════════════════
    def _send_run_to_analysis(self, switch_tab=True):
        if not self.run_rpm:
            if switch_tab:
                messagebox.showinfo("No Run", "No recorded run available yet.")
            return
        self.analysis_rpm = np.asarray(self.run_rpm, dtype=float)
        self.analysis_torque = np.asarray(self.run_torque, dtype=float)
        self.analysis_is_nm = self.recorded_torque_is_nm
        self.analysis_label = f"Last recorded run ({len(self.run_rpm)} pts)"
        self.analysis_src_label.config(text=self.analysis_label)
        if switch_tab:
            self.notebook.select(self.analysis_tab)
        self._refresh_analysis()

    def _analysis_load_file(self):
        path = filedialog.askopenfilename(
            title="Load a recorded dyno log / CSV / Excel run for analysis",
            filetypes=[("CSV / log / Excel files", "*.csv *.txt *.xlsx *.xlsm"),
                       ("All files", "*.*")],
        )
        if not path:
            return
        try:
            rec = dsp.load_recording(path)
        except Exception as e:
            messagebox.showerror("Load Error", f"Could not load:\n{path}\n\n{e}")
            return
        if rec.rpm is None or rec.torque is None:
            messagebox.showerror("No RPM/Torque",
                                 f"File has no RPM/Torque channel (format: {rec.fmt}).")
            return
        self.analysis_rpm = rec.rpm
        self.analysis_torque = rec.torque
        self.analysis_is_nm = rec.torque_is_nm
        self.analysis_label = f"{os.path.basename(path)} ({rec.fmt}, {rec.n} pts)"
        self.analysis_src_label.config(text=self.analysis_label)
        self._refresh_analysis()

    def _read_filter_params(self):
        def fnum(key, default):
            try:
                return float(self.filter_param_vars[key].get())
            except ValueError:
                return default
        return dict(
            alpha=fnum("alpha", 0.2),
            window=int(max(2, fnum("window", 20))),
            polyorder=int(max(1, fnum("polyorder", 3))),
            tau=fnum("tau", 50.0),
            degree=int(max(1, fnum("degree", 2))),
        )

    def _compute_analysis_curve(self):
        """Return (grid_rpm, raw_tq_disp, filt_tq_disp, raw_hp, filt_hp) or None.

        Torque is processed in Nm internally, then converted for display.
        HP is derived from torque assuming Nm; it is only a physical horsepower
        when self.analysis_is_nm is True (otherwise it is flagged "uncal").
        """
        if self.analysis_rpm is None or self.analysis_torque is None:
            return None
        rpm = np.asarray(self.analysis_rpm, dtype=float)
        tq_nm = np.asarray(self.analysis_torque, dtype=float)

        def fval(attr, default):
            try:
                return float(getattr(self, attr).get())
            except (ValueError, AttributeError):
                return default
        rpm_min = fval("rpm_min_var", 0.0)
        rpm_max = fval("rpm_max_var", 0.0)        # 0 = auto (no upper gate)

        m = np.isfinite(rpm) & np.isfinite(tq_nm) & (rpm > max(rpm_min, 0.0))
        if rpm_max > 0:
            m &= rpm <= rpm_max
        rpm, tq_nm = rpm[m], tq_nm[m]
        if rpm.size < 3:
            return None

        try:
            step = float(self.rpm_bin_var.get())
        except ValueError:
            step = 25.0
        step = max(1.0, step)

        # ── YourDyno "Run analysis tool" binned method (its own full pipeline) ──
        if self.filter_type_var.get() == dsp.FILTER_YOURDYNO:
            return self._compute_yourdyno_curve(rpm, tq_nm, step)

        # Optional spike removal before binning. Despike RPM (tooth-to-tooth
        # jitter corrupts which bin a sample lands in) and torque (outliers bias
        # the per-bin mean).
        if self.despike_var.get():
            rpm = dsp.hampel_despike(rpm, window=7, n_sigma=3.0)
            tq_nm = dsp.hampel_despike(tq_nm, window=7, n_sigma=3.0)

        grid_rpm, grid_tq = dsp.resample_by_rpm(rpm, tq_nm, rpm_step=step)
        if grid_rpm.size < 3:
            return None

        p = self._read_filter_params()
        filt_tq = dsp.apply_filter(
            self.filter_type_var.get(), grid_tq,
            x=grid_rpm, dt=step, alpha=p["alpha"], window=p["window"],
            polyorder=p["polyorder"], tau=p["tau"], degree=p["degree"])

        raw_hp = dsp.hp_from_torque_nm(grid_tq, grid_rpm)
        filt_hp = dsp.hp_from_torque_nm(filt_tq, grid_rpm)
        return (grid_rpm,
                self._torque_display(grid_tq), self._torque_display(filt_tq),
                raw_hp, filt_hp)

    def _sae_factor(self):
        """Current SAE J607 factor from the panel (1.0 if disabled). Updates label."""
        if not self.sae_apply_var.get():
            self.sae_factor_label.config(text="(off — factor 1.000)")
            return 1.0
        try:
            t = float(self.sae_vars["temp"].get())
            h = float(self.sae_vars["hum"].get())
            pr = float(self.sae_vars["press"].get())
        except ValueError:
            self.sae_factor_label.config(text="(invalid inputs — factor 1.000)")
            return 1.0
        f = dsp.sae_j607_factor(t, h, pr)
        self.sae_factor_label.config(text=f"J607 factor: {f:.4f}")
        return f

    def _compute_yourdyno_curve(self, rpm, tq_nm, step):
        """YourDyno 'Run analysis tool' binned method → same return tuple as
        _compute_analysis_curve. Raw overlay = bin average; filtered = graph-MA
        smoothed; both J607-corrected (factor applies to torque, dimensionless)."""
        def ival(key, default, lo, hi):
            try:
                return int(np.clip(float(self.yd_vars[key].get()), lo, hi))
            except ValueError:
                return default
        gauge = ival("gauge", 3, 0, 10)
        graph = ival("graph", 3, 0, 10)
        spike = ival("spike", 0, 0, 5)
        factor = self._sae_factor()
        res = dsp.yourdyno_binned_filter(rpm, tq_nm, gauge=gauge, graph=graph,
                                         spike=spike, bin_width=step, sae_factor=factor)
        if res is None or res["bin_rpm"].size < 2:
            return None
        grid_rpm = res["bin_rpm"]
        raw_tq = res["bin_torque"]            # bin average × factor (pre-graph)
        filt_tq = res["smoothed_torque"]      # graph MA × factor (final)
        raw_hp = dsp.hp_from_torque_nm(raw_tq, grid_rpm)
        filt_hp = dsp.hp_from_torque_nm(filt_tq, grid_rpm)
        return (grid_rpm,
                self._torque_display(raw_tq), self._torque_display(filt_tq),
                raw_hp, filt_hp)

    def _refresh_analysis(self):
        self.an_ax_t.clear()
        self.an_ax_hp.clear()
        units = self.units_var.get()
        # When torque isn't in Nm (raw ADC counts), HP = counts*RPM/7120.9 is not
        # a physical horsepower — flag both torque and HP as uncalibrated.
        unit_note = "" if self.analysis_is_nm else " (raw counts)"
        hp_note = "" if self.analysis_is_nm else " (uncal)"
        self.an_ax_t.set_xlabel("RPM")
        self.an_ax_t.set_ylabel(f"Torque ({units}){unit_note}", color="tab:blue")
        self.an_ax_hp.set_ylabel(f"HP{hp_note}", color="tab:red")
        self.an_ax_t.grid(True, alpha=0.3)

        result = self._compute_analysis_curve()
        if result is None:
            self.an_ax_t.set_title("Filtered vs Raw — load a dataset")
            self.an_canvas.draw_idle()
            self.analysis_peaks.config(text="")
            return

        grid_rpm, raw_tq, filt_tq, raw_hp, filt_hp = result
        if self.show_raw_var.get():
            self.an_ax_t.plot(grid_rpm, raw_tq, "-", color="tab:blue", alpha=0.25,
                              linewidth=0.9, label="Torque (raw)")
            self.an_ax_hp.plot(grid_rpm, raw_hp, "-", color="tab:red", alpha=0.25,
                               linewidth=0.9, label="HP (raw)")
        self.an_ax_t.plot(grid_rpm, filt_tq, "-", color="tab:blue", linewidth=1.8,
                          label="Torque (filtered)")
        self.an_ax_hp.plot(grid_rpm, filt_hp, "-", color="tab:red", linewidth=1.8,
                           label="HP (filtered)")

        ftype = self.filter_type_var.get()
        self.an_ax_t.set_title(f"Filtered vs Raw — {ftype}")
        lines = self.an_ax_t.get_lines() + self.an_ax_hp.get_lines()
        self.an_ax_t.legend(lines, [l.get_label() for l in lines],
                            loc="upper left", fontsize=8)
        self.an_fig.tight_layout()
        self.an_canvas.draw_idle()

        # Peak callouts (from the filtered curves; NaN-safe for sparse bins)
        if np.isfinite(filt_tq).any() and np.isfinite(filt_hp).any():
            it, ih = int(np.nanargmax(filt_tq)), int(np.nanargmax(filt_hp))
            self.analysis_peaks.config(
                text=(f"Peak Torque: {filt_tq[it]:.1f} {units}{unit_note} @ {grid_rpm[it]:.0f} RPM\n"
                      f"Peak HP:     {filt_hp[ih]:.1f} HP{hp_note}  @ {grid_rpm[ih]:.0f} RPM"))
        else:
            self.analysis_peaks.config(text="")

    def _write_filtered_csv(self, path, result=None):
        """Write the filtered RPM-domain curve. Shared by the export dialog and
        the automatic save, so a run always has raw and filtered side by side."""
        if result is None:
            result = self._compute_analysis_curve()
        if result is None:
            return 0
        grid_rpm, raw_tq, filt_tq, raw_hp, filt_hp = result
        units = self.units_var.get()
        cal_note = "" if self.analysis_is_nm else (
            " WARNING: torque is raw ADC counts (not Nm); Torque/HP columns are uncalibrated")
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([f"# filter={self.filter_type_var.get()} "
                        f"params={self._read_filter_params()} "
                        f"rpm_bin={self.rpm_bin_var.get()} units={units}{cal_note}"])
            w.writerow(["RPM", f"Torque_raw_{units}", f"Torque_filt_{units}",
                        "HP_raw", "HP_filt"])
            for i in range(grid_rpm.size):
                w.writerow([f"{grid_rpm[i]:.1f}", f"{raw_tq[i]:.3f}", f"{filt_tq[i]:.3f}",
                            f"{raw_hp[i]:.2f}", f"{filt_hp[i]:.2f}"])
        return int(grid_rpm.size)

    def _export_filtered(self):
        result = self._compute_analysis_curve()
        if result is None:
            messagebox.showinfo("No Data", "Load a dataset and apply a filter first.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV Files", "*.csv")],
            initialfile=f"dyno_filtered_{time.strftime('%Y%m%d_%H%M%S')}.csv",
        )
        if not path:
            return
        n = self._write_filtered_csv(path, result)
        messagebox.showinfo("Saved", f"{n} points saved to:\n{path}")

    # ── Cleanup ──────────────────────────────────────────────
    def on_close(self):
        self._save_session()
        self.replay_running = False
        self._motor_close()
        self._disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = DynoApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.on_close()


if __name__ == "__main__":
    main()

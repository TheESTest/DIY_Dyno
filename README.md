# DIY Dyno

Control software for a DIY engine dynamometer: a Raspberry Pi 5 operator
interface talking over USB serial to an ESP32-S3 that runs the control loop for
a stepper-actuated hydraulic brake.

| Component | Version | Location |
|---|---|---|
| Raspberry Pi UI | 1.14.0 | [`pi_gui/`](pi_gui/) |
| ESP32-S3 firmware | 1.7.0 | [`esp32_firmware/`](esp32_firmware/) |

Both versions are recorded in the `_conditions.json` file saved beside every
run, so a result can be traced back to the code that produced it. The UI shows
the firmware version it is talking to on the Live Run tab, and flags `unknown`
when the board is too old to answer.

## How it is split

The ESP32 owns anything that has to happen on time: reading the tach, running
the PID, and driving the stepper. The Pi owns everything else — the interface,
recording, filtering and the power-curve maths. They speak a line-based ASCII
protocol over UART0 at 115200 baud, so the link can be driven by hand from a
serial terminal when something needs diagnosing.

### The control loop

The PID input is **RPM error** and its output is **brake position in driver
steps**, so `Kp` is steps of brake per RPM of error. Line pressure is *not*
feedback — it appears only as a safety limit.

Every step figure is derived from three drivetrain settings in the UI, because
those are what change when a DIP switch moves, and a step count that silently
disagrees with the driver reads as a mechanical fault:

| Setting | As built |
|---|---|
| Motor full steps per rev | 200 |
| Driver steps per rev (DIP) | 400 — half stepping |
| Gearbox reduction | 10 : 1 |
| Usable cam angle | 45° |

which gives 4000 steps per cam revolution, 11.111 per degree, 500 across the
usable travel, and 1000 for a quarter cam turn. Changing the driver setting
rescales the travel, and `Kp` has to move with it: applying a new drivetrain
offers to rescale the gains, speed and acceleration by the same factor, because
gains left behind are wrong by exactly that much — and too high means the brake
slams on for a small error.

### RPM conditioning

An inductive pickup on a running engine collects electrical noise, and a bad
reading is not a harmless data point — the controller answers it with brake.
Readings pass through, in order:

1. **Interrupt glitch rejection** — edges closer together than the fastest
   credible tooth are discarded before the control loop ever sees them.
2. **Median of the last N intervals** — rejects isolated outliers.
3. **Validity band** (`RPM_BAND`) — a computed reading outside the range the
   engine can actually run in is a bad reading, not a slow or fast engine. Its
   upper end also sets the interrupt threshold in step 1. Applied after the
   pulse is counted, so a slow engine never reads as a dead pickup.
4. **Ratio gate** and **slew limit** — bound how far a sample may sit from the
   established reading, and how fast the reading may change.
5. **Averaging** over a configurable window.

Optionally (`RPM_EXTRAP`, off by default) an out-of-band reading can be replaced
by a least-squares line projected through the last few *genuine* samples, rather
than holding the last good value and flat-spotting the trace mid-sweep.
Deliberately bounded: estimated values never re-enter the fit, the result is
clamped to the band, and after a set number of consecutive substitutions it
gives up and holds — a straight line carried through a long noise burst would
invent a runaway, and the brake acts on whatever it is told. Every substitution
is counted and reported, and both `Tach_Glitches` and `RPM_Estimated` are
written to the run CSV as running totals, so where they step up is exactly where
the tach misbehaved and where a value was estimated rather than measured.

## Stepper position: open loop, for now

There is **no encoder on the stepper**. The controller commands microsteps and
assumes they arrive, which is exactly the assumption a stall breaks. The
firmware carries a marked TBD block for one — settings (`ENCODER,<0|1>,<cpr>,
<invert>`), a `CFG,ENCODER` readback, two reserved `DATA` fields and a stub
`updateEncoder()` — so fitting the hardware will not change the protocol or the
UI. Nothing fakes a value in the meantime: enabling an encoder with no pins
assigned answers with an error rather than quietly accepting, and the position
field reports `not installed` rather than echoing the commanded position, which
would report an agreement it has not measured.

Until then, **line pressure is the witness** that the motor went where it was
told. During the brake characterisation sweep, once the brake is engaged,
commanded position climbing while pressure stays flat is flagged as a possible
stall. The takeup travel before the pads bite is deliberately ignored — position
rising with no pressure is normal there, and judging it would flag every healthy
sweep. A brake that is already fully applied looks identical to a stalled motor
from here, so this reports a suspicion, not a verdict: the point at which the
sweep stopped being trustworthy is marked on the plot, written to the CSV as
`Stall_Suspected`, and recorded in the run's conditions file.

## Updating a rig from here

The interface can pull its own code from this repository. It fetches to memory,
**compiles every Python file before allowing it to replace anything**, copies
what it replaces into a dated `backup_` folder, and writes through a temporary
name so an interrupted write cannot leave a half-file where the real one was. It
refuses while a run, a characterisation sweep or a replay is going, or while the
controller is in any state but idle.

Firmware is **downloaded only**. Flashing is a separate button that requires the
serial port to be disconnected and a deliberate confirmation, because it reboots
the controller holding the brake. `esp32_firmware/build/firmware.bin` is the
binary it fetches; rebuild and commit it alongside `main.cpp` so the two never
disagree.

## Analysing runs

`pi_gui/dyno_analyze.py` reads runs, sweeps and pulse captures and says what is
wrong with them. It imports nothing from the GUI, so it runs anywhere the data
does.

```sh
python dyno_analyze.py                  # the default run folder
python dyno_analyze.py <file-or-folder> # a specific run
python dyno_analyze.py --github         # fetch data/ from here and analyse it
python dyno_analyze.py --plot           # also write a PNG per file
```

Every check exists because it caught something real. It separates the cases
that look alike in a summary:

| It sees | It concludes |
|---|---|
| every measured channel a constant zero | nothing was connected; not a run at all |
| position a **constant ratio** of the command | the controller was working to a smaller target - the interface and the firmware disagree about the brake range |
| that ratio **degrading** as load builds | the motor is losing steps |
| pressure collapsing while position still rises | the hydraulic let go, not sensor noise |
| `Kp` against the brake range it drives | the error it takes to reach full brake |

It will not recommend a brake range from a sweep that did not finish - a
truncated curve puts "saturation" wherever the actuator happened to stop.

## Publishing runs

Every finished test and every calibration sweep is published to `data/` in this
repository automatically: the run CSV, its conditions file, the filtered curve
and any plot. A run finishes whether or not the network happens to be up, so
anything that cannot be sent is queued to disk and goes with the next attempt
rather than being lost to a red status line.

It needs a token with `Contents: write` on this repository, in
`DYNO_GITHUB_TOKEN` or a local `dyno_github_token.txt` (git-ignored). The token
is read only when uploading and is scrubbed from every error path — it never
reaches a dialog, the event log, a profile or a conditions file.

**This repository is public.** Anything published here is readable by anyone,
including the free-text notes recorded with a run.

## Running it

```sh
cd pi_gui
pip install -r requirements.txt
python3 dyno_gui.py
```

The UI runs without hardware attached: the firmware has a simulation mode, and
`tools/gen_sim_trace.py` produces a synthetic run for exercising the analysis
path. A `SIM` badge is shown whenever the data is not real, and an unidentified
board is treated as simulated until it says otherwise — the safe default is to
doubt the data rather than trust it.

### Tests

```sh
cd pi_gui
for t in test_*.py; do python3 "$t"; done
```

Each file is standalone and prints `FAILURES: none` on success. They build a
real `DynoApp` against Tk, so they need a display (`DISPLAY=:0` on the Pi).

### Firmware

```sh
cd esp32_firmware
pio run                                    # both environments
pio run -e esp32-s3-devkitc-1 -t upload    # live
```

`esp32-s3-devkitc-1-sim` builds the same source with simulation defaulted on.

## Hardware

- ESP32-S3 DevKitC-1. The `opi_opi` memory variant consumes GPIO26–37; UART0
  (43/44) is the link to the Pi.
- Inductive tach pickup, configurable pulses per revolution.
- Load cell through a 0–10 V amplifier into a DFRobot ADS1115 at `0x48`
  (GPIO8/9).
- Brake line pressure sensor, 0–2000 PSI, 0.5–4.5 V, on its own ADS1115 at
  `0x49` (GPIO10/11). Note this sensor wants an 8–16 V supply, not 5 V.
- Stepper through a 10:1 planetary onto the brake cam, with a homing switch.

## Serial protocol

Line based, `\n` terminated, ASCII throughout.

**From the board**

| Frame | Meaning |
|---|---|
| `DATA,…` | 19 fields of live telemetry, 20 Hz |
| `READY,…` | readiness flags: ready, homed, tared, ADC, fault, sim |
| `CFG,…` | configuration readback, in reply to `STATUS` or `VERSION` |
| `ACK,…` / `ERR,…` | command accepted / rejected with a reason |

**To the board** — `STATUS`, `VERSION`, `RPM_BAND,<min>,<max>`,
`RPM_EXTRAP,<0|1>,<points>,<maxrun>`, `RPM_MEDIAN`, `RPM_RATIO`, `RPM_SLEW`,
`RPM_AVG`, `TEETH`, `RATIO`, `PID`, `PID_SWEEP`, `BRAKE`, `BRAKE_RANGE`,
`STEPPER_SPEED`, `STEPPER_ACCEL`, `HOME`, `START`, `STOP`, `TARE` and others —
see the command block in [`esp32_firmware/src/main.cpp`](esp32_firmware/src/main.cpp).

Older firmware is tolerated: the UI treats trailing `DATA` fields as optional,
so a board that predates a field still connects and runs.

## Safety

This drives a hydraulic brake against a running engine. `STOP` ramps the brake
off on a fast linear ramp from any state, a configurable cutoff RPM releases it
entirely at the end of a pull, and a lost tach signal is treated as a fault. The
brake characterisation sweep refuses to run with the engine turning, since
walking the actuator across its full travel would apply full braking.

Treat every default in this repository as a starting point to tune from on your
own rig, not a tuned value.

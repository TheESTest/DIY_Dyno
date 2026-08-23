# DIY Dyno

Control software for a DIY engine dynamometer: a Raspberry Pi 5 operator
interface talking over USB serial to an ESP32-S3 that runs the control loop for
a stepper-actuated hydraulic brake.

| Component | Version | Location |
|---|---|---|
| Raspberry Pi UI | 1.1.0 | [`pi_gui/`](pi_gui/) |
| ESP32-S3 firmware | 1.1.0 | [`esp32_firmware/`](esp32_firmware/) |

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

The PID input is **RPM error** and its output is **brake position in
microsteps**, so `Kp` is microsteps of brake per RPM of error. Line pressure is
*not* feedback — it appears only as a safety limit. The brake geometry is 5000
microsteps per motor revolution through a 10:1 planetary, giving 138.889
microsteps per degree of cam and 6250 microsteps across the 45° of usable
travel.

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

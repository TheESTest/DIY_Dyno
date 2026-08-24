# Recorded runs

Data published from the dyno's Raspberry Pi by the interface's
**Publish latest run** button. Each run appears as a set:

| File | What it holds |
|---|---|
| `dyno_run_<stamp>.csv` | every sample: RPM (all three measurement methods), torque raw and filtered, HP, load cell mV, pressure, brake position and percent, PID terms, fault bits, tach glitches |
| `dyno_run_<stamp>_conditions.json` | every setting in force for that run, and a loadable profile |
| `dyno_run_<stamp>_filtered.csv` | the RPM-binned power curve |
| `brake_char_<stamp>.csv` | a brake characterisation sweep: commanded position, reported position, line pressure |
| `pulses_<stamp>.csv` | a raw pulse capture: every tach edge with its interval |

`Time_s` is the first column of every run CSV, so they reload directly.

Torque is written in whatever unit was on screen, named in the header. An
uncalibrated run says `native` rather than pretending to be Nm or lb-ft.

This repository is public. Anything here is readable by anyone, including the
free-text notes recorded with a run.

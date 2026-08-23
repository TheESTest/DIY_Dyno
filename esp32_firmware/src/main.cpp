// ============================================================
// DIY Engine Dyno — ESP32-S3 Firmware
// Communicates with Raspberry Pi 5 via USB Serial.
// Controls stepper brake, reads RPM sensor, load cell, ADC.
//
// Operator procedure (manual throttle):
//   1. Home stepper, Tare load cell, set parameters
//   2. Press START → brake moves to preload % and WAITS (loop not engaged)
//   3. Operator throttles up; at holdRPM the loop engages and holds that speed
//   4. At WOT, operator presses RELEASE on the Pi GUI
//   5. ESP enters SWEEP: sweeps RPM start→end using the SWEEP gain set
//   6. Sweep completes → hold at endRPM on the HOLD gain set
//   7. Operator slowly throttles down; PID releases brake
//   8. RPM drops below holdRPM → auto-reset to IDLE, brake 0%
//
// Serial protocol (line based, 115200):
//   Pi → ESP  READY? STATUS PING SIM,<0|1> HOME HOME_SET STOP
//             BRAKE,<steps>  BRAKE_REL,<d>  HOLD,<rpm>
//             SWEEP,<start>,<end>,<rate>    START  RELEASE
//             PID,<kp>,<ki>,<kd>            PID_SWEEP,<kp>,<ki>,<kd>
//             TARE  CAL_SCALE,<v>  CAL_ARM,<m>  CAL_MECH,<v>
//             CAL_PRESS,<offset_mV>,<psi_per_mV>   PRESS_LIMIT,<psi>
//             PRESS_DIV,<ratio>  PRESS_PGA,<0-5>
//             TEETH,<n>  RATIO,<r>  BRAKE_RANGE,<min>,<max>  PRELOAD,<pct>
//             INVERT,<0|1>  STEPPER_SPEED,<v>  STEPPER_ACCEL,<v>
//             VERSION (firmware version and build stamp)
//             ENCODER,<0|1>,<cpr>,<invert>   TBD - stored, not yet active
//             BRAKE_SWEEP,<target>,<ms>  one smooth timed traverse
//             RPM_BAND,<min>,<max>  RPM_MEDIAN,<1|3|5|7>  RPM_RATIO,<x>
//             RPM_EXTRAP,<0|1>,<points>,<maxrun>
//             RPM_SOURCE,<0 gap|1 counted|2 revolution>
//             RPM_COUNT_MS,<ms>  counting window length
//             RPM_SLEW,<rpm/s>  RPM_AVG,<n>  TACH_RESET
//             RAMPDOWN_MODE,<0-2>  RAMPDOWN_RATE,<rpm/s>  RAMPDOWN_BRAKE,<%/s>
//             CUTOFF_RPM,<rpm>  THROTTLE_OFF,<pct>  RAMPDOWN  STOP_RATE,<%/s>
//             CAM_MODEL,<0|1|2>  CAM_SPD,<steps/deg>  CAM_LIN,<0|1>
//             CAM_NPTS,<n>  CAM_PT,<i>,<stepPct>,<brakePct>
//   ESP → Pi  DATA,millis,rpm,torque,loadRaw,loadmV,pressmV,brakePos,targetRPM,
//                  state,pidP,pidI,pidOut,brakePct,pressPSI,faultBits,spareAux,tachGlitches,rpmEstimated,encoderCount,encoderOK,rpmCounted,rpmRev
//             READY,ready,homed,tared,adc,sim,pressAdc
//             CFG,<key>[,values]   ACK,<msg>   ERR,<msg>
//   faultBits: bit0 = tach lost, bit1 = brake pressure over limit
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_ADS1115_0_10V.h>
#include <Adafruit_ADS1X15.h>
#include <AccelStepper.h>
#include "sim_trace.h"   // recorded torque-vs-RPM curve for SIM (auto-generated)

// =============================================
// GPIO Pin Definitions
// =============================================
#define PIN_STEPPER_PULSE    4   // Stepper driver PUL- (common-anode wiring)
#define PIN_STEPPER_DIR      5   // Stepper driver DIR- (common-anode wiring)
#define PIN_STEPPER_ENABLE   6   // Stepper driver ENA- (common-anode wiring)
#define PIN_PROXIMITY        7   // Induction proximity sensor (digital input)
#define PIN_I2C_SDA          8   // I2C SDA (DFRobot 0-10V ADC module)
#define PIN_I2C_SCL          9   // I2C SCL (DFRobot 0-10V ADC module)
#define PIN_LIMIT_SWITCH     12  // Limit switch for stepper homing (active LOW)
// Second I2C bus — dedicated ADS1115 for the brake pressure transducer, kept off
// the DFRobot module's bus so a stall on one cannot block the other.
#define PIN_I2C2_SDA         10  // I2C1 SDA (pressure ADS1115)
#define PIN_I2C2_SCL         11  // I2C1 SCL (pressure ADS1115)
// NOTE: Load cell amplifier analog output connects to ADC channel 0

// =============================================
// Configuration Constants
// =============================================
// Firmware version. Bump the minor when the serial protocol changes shape
// (a new DATA field, a renamed command) so a mismatched GUI is diagnosable
// from the log rather than from guesswork about which build is on the board.
#define FW_VERSION "1.6.1"

// ─────────────────────────────────────────────────────────────────────
// STEPPER ENCODER - TBD, HARDWARE NOT FITTED
//
// Nothing here reads a real encoder yet. Position is open loop: the
// controller commands microsteps and assumes they arrive, which is exactly
// the assumption a stall breaks. The settings and the reporting path exist
// now so that fitting the hardware is a matter of filling in updateEncoder()
// and attaching the interrupts, without changing the serial protocol or
// the GUI - DATA already carries the two fields, reporting 0 and not-ok.
//
// When it is fitted:
//   * pick two free GPIO for A/B (GPIO26-37 are consumed by opi_opi
//     memory on this board, and 8/9 and 10/11 are the two I2C buses)
//   * attach both edges of both channels to a quadrature ISR
//   * compare encoderSteps() against stepper.currentPosition() to detect a
//     stall directly, which replaces the GUI's pressure-based inference
//   * decide whether a confirmed stall should fault the run outright
// ─────────────────────────────────────────────────────────────────────
#define ENCODER_PIN_A -1              // TBD: not assigned
#define ENCODER_PIN_B -1              // TBD: not assigned
bool     encoderEnabled = false;      // ENCODER,<0|1>,<cpr>,<invert>
uint32_t encoderCPR     = 4000;       // counts per motor revolution
bool     encoderInvert  = false;
volatile int32_t encoderCount = 0;    // stays 0 until the hardware exists
bool     encoderOK      = false;      // never true without a real encoder

// TBD: no hardware, so there is nothing to read. Deliberately does not fake
// a value from the commanded position - that would report agreement it
// cannot possibly have measured, which is worse than reporting nothing.
static void updateEncoder() {
    encoderOK = false;
}


#define DEFAULT_PULSES_PER_REV 3     // Proximity triggers/rev (runtime: TEETH,<n>)
// Averaging is capped only by RAM (200 floats is 800 bytes). Long windows are
// smooth but slow: 100 pulses on a 3-tooth wheel is 33 revolutions, which at
// 3000 RPM is two thirds of a second of lag inside the control loop.
// 500 floats is 2 kB, which this board can spare. Note the cost is not
// memory but phase lag: this average feeds the PID, so a long window means
// the loop is answering an RPM the engine had some time ago. The GUI shows
// that lag in seconds next to the field.
#define RPM_AVG_MAX          500     // Capacity of the RPM average buffer
#define DEFAULT_RPM_AVG      3       // Pulses averaged (runtime: RPM_AVG,<n>)
#define TORQUE_AVG_SIZE      10      // Rolling average window for torque
#define DATA_REPORT_MS       50      // Data report interval (20 Hz)
#define ADC_READ_MS          50      // ADC read interval (20 Hz, includes load cell)
#define PID_INTERVAL_MS      50      // PID update interval (20 Hz)
#define RPM_TIMEOUT_US       2000000 // 2 seconds with no pulse = 0 RPM (idle)
#define ADC_I2C_ADDR         0x48    // DFRobot Gravity 0-10V ADC (ADS1115)
#define PRESS_ADC_ADDR       0x49    // Pressure ADS1115, ADDR strapped to VDD
#define PRESS_ADC_CHANNEL    0       // Sensor signal on its AIN0

// Tach-loss detection.  While the PID is actively controlling the brake we
// expect pulses far more often than this: at the lowest realistic hold speed
// (~500 RPM) with a single trigger we still see a pulse every 120 ms.  Going
// quiet for longer than this means the signal is gone, NOT that the engine
// stopped that fast — see controlBrake() for why that distinction matters.
#define RPM_FAULT_TIMEOUT_US 300000  // 0.3 s with no pulse while controlling
#define RPM_MEDIAN_MAX       7       // Longest median window on pulse intervals

// Stepper defaults, scaled with the driver change so the physical motion is
// unchanged: 800 steps/s at 400 steps/rev is the same 2 motor rev/s that
// 10000 was at 5000 steps/rev, and still crosses the travel in about 0.6 s.
#define STEPPER_MAX_SPEED    800.0f
#define STEPPER_ACCEL        4000.0f
// Brake travel in DRIVER STEPS. The driver DIP switches are now set to 400
// steps per motor revolution - half stepping on a 200 full-step motor, not
// the 5000 it was set to before:
//   400 steps/rev x 10:1 planetary = 4000 per cam revolution
//   4000 / 360 deg                 = 11.111 steps per degree
//   45 deg of usable cam           = 500 steps
//   a quarter cam turn             = 1000 steps
// The GUI derives all of this from the motor, driver and gearbox figures and
// can push it down, so these are the fallback for an unconfigured board.
#define BRAKE_MAX_STEPS_DEF  500     // Default full-travel limit (runtime: BRAKE_RANGE)
#define CAM_STEPS_PER_DEG_DEF 11.111f
#define HOMING_SPEED         160.0f  // Homing approach speed (driver steps/sec)

// PID defaults, sized against the 500-step travel the current drivetrain gives
// (400 driver steps/rev x 10:1, 45 deg of cam). The loop output IS a step
// position, so Kp is driver steps of brake per RPM of error, and Kp 0.5
// reaches full brake at about 1000 RPM of error - the same authority the old
// 6.25 had against 6250 steps. Move the driver DIP switches and these must be
// rescaled by the same factor, or the loop is wrong by exactly that much.
// Starting points to tune from on the stand, not tuned values.
#define DEFAULT_PID_KP       0.5f
#define DEFAULT_PID_KI       0.8f
#define DEFAULT_PID_KD       0.02f
// Sweep is softer than hold, matching the reference system's split (its 20% /
// 12% pair) — the brake has to catch hard during throttle-up but must not fight
// a near-instantaneous hydraulic during the pull.
#define DEFAULT_SWEEP_KP     0.3f
#define DEFAULT_SWEEP_KI     0.5f
#define DEFAULT_SWEEP_KD     0.01f

// RPM tolerance for hold detection
#define RPM_HOLD_TOLERANCE   0.05f   // ±5% of target RPM
// RPM threshold for auto-reset after sweep complete
// When RPM drops below holdRPM, system resets
#define RPM_RESET_HYSTERESIS 0.10f   // 10% below holdRPM → reset

// =============================================
// SIM ("virtual engine") — bench demo with NO engine attached
// A lumped closed-loop model drives currentRPM/currentTorque so the REAL state
// machine + PID control it and the cam brake stepper visibly moves in real time:
//     dRPM/dt = A*throttle - B*brakeSteps - C*RPM
// Constants tuned (against this firmware's own default PID gains) so HOLD and
// SWEEP are stable and the brake stepper sweeps.  Torque is injected from the
// recorded run's torque-vs-RPM curve (sim_trace.h).  Enable at boot with
// SIM_MODE_DEFAULT or live with the "SIM,1"/"SIM,0" serial command.  When off,
// real-hardware behaviour is completely unchanged.
// =============================================
// Boots OFF.  A board that comes up in SIM produces a completely convincing
// RPM/torque/brake display with no engine attached, which is indistinguishable
// from a working system until you look closely — so real hardware must never be
// the mode you get by accident.  Build the `...-sim` PlatformIO env (which sets
// -DSIM_MODE_DEFAULT=1) for bench demos, or send "SIM,1" at runtime.
#ifndef SIM_MODE_DEFAULT
#define SIM_MODE_DEFAULT     0        // 0 = real hardware; 1 = virtual engine
#endif
#define SIM_ENGINE_A         11250.0f // WOT accel authority (RPM/s at full throttle)
#define SIM_ENGINE_B         6.0f     // brake authority (RPM/s per brake step)
#define SIM_ENGINE_C         1.5f     // drag (1/s); WOT ceiling = A/C ≈ 7500 RPM
#define SIM_WOT_RPM          (SIM_ENGINE_A / SIM_ENGINE_C)
#define SIM_THROTTLE_TAU     1.0f     // s; operator throttle first-order lag
#define SIM_DONE_HOLD_MS     2500     // hold at end RPM, then auto throttle-down

// =============================================
// State Machine
//   IDLE          – waiting; brake released
//   HOMING        – stepper backing off until limit switch
//   MANUAL        – operator manually positions brake
//   HOLD_RPM      – PID holds engine at holdRPM (pre-run)
//   SWEEP         – PID sweeps target RPM from start→end
//   SWEEP_DONE    – hold at endRPM; waiting for throttle-down
// =============================================
enum DynoState {
    STATE_IDLE = 0,
    STATE_HOMING,
    STATE_MANUAL,
    STATE_HOLD_RPM,
    STATE_SWEEP,
    STATE_SWEEP_DONE,
    STATE_RAMP_DOWN          // operator has lifted; bringing the engine down
};

static const char* STATE_NAMES[] = {
    "IDLE", "HOMING", "MANUAL", "HOLD_RPM", "SWEEP", "SWEEP_DONE", "RAMP_DOWN"
};

// =============================================
// Global Objects
// Common-anode wiring: PUL+/DIR+/ENA+ → 5V, signals on - pins
// ESP32 sinks current → inverted pulse/dir logic
// =============================================
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEPPER_PULSE, PIN_STEPPER_DIR);
DFRobot_ADS1115_I2C adc(&Wire, ADC_I2C_ADDR);
bool adcOk = false;

// Brake pressure transducer on its own ADS1115 (second bus, address 0x49).
// Optional: if it is not fitted the rig still runs, the channel simply reports
// nothing and the over-pressure interlock stays disarmed.
Adafruit_ADS1115 pressAdc;
bool  pressAdcOk = false;

// =============================================
// Readiness flags
// =============================================
bool isHomed   = false;
bool isTared   = false;

// =============================================
// ADC task (runs on Core 0, separate from stepper)
// =============================================
volatile bool tareRequested = false;  // set by main core, consumed by ADC task

// =============================================
// RPM Measurement (interrupt-driven)
// =============================================
volatile uint32_t pulseCount      = 0;
volatile uint32_t lastPulseUs     = 0;
volatile uint32_t pulseIntervalUs = 0;
// Anything arriving sooner than this cannot be a tooth at any engine speed we
// accept, so it is electrical noise. Recomputed whenever TEETH or the maximum
// valid RPM changes. The count is reported so the operator can see how much is
// being thrown away rather than having to infer it from a ragged trace.
// A revolution's worth of pulse timestamps. Timing one whole revolution
// cancels tooth-to-tooth spacing error outright, which timing a single
// gap cannot: an unevenly spaced tooth shows up as a fixed pattern that
// no amount of averaging removes, it only smears.
#define PULSE_TS_MAX 64
volatile uint32_t pulseTs[PULSE_TS_MAX] = {0};
volatile uint8_t  pulseTsIdx = 0;

volatile uint32_t minPulseIntervalUs = 2500;
volatile uint32_t tachGlitches       = 0;

float    rpmBuffer[RPM_AVG_MAX];
// uint16_t throughout: the window can now be 500 samples, and a uint8_t
// silently wraps a requested 500 to 244.
uint16_t rpmBufIdx   = 0;
uint16_t rpmBufCount = 0;
float    currentRPM  = 0.0f;      // whichever source drives the loop
// The same tach measured three ways, all reported, so they can be compared
// against each other on the same run rather than across two runs.
//   0 gap      - time between consecutive pulses (fast, tooth-error prone)
//   1 counted  - pulses per fixed window       (smooth, coarse, laggy)
//   2 rev      - time for one whole revolution (tooth error cancels)
float    rpmGap      = 0.0f;
float    rpmCounted  = 0.0f;
float    rpmRev      = 0.0f;
uint8_t  rpmSource   = 1;         // RPM_SOURCE,<0|1|2>, counting by default
// How long each counting window lasts. 100 ms is a 10 Hz update; longer
// windows hold more pulses and read steadier, shorter ones answer sooner.
uint32_t rpmCountWindowMs = 100;  // RPM_COUNT_MS,<ms>
#define RPM_COUNT_MS_MIN 20
#define RPM_COUNT_MS_MAX 5000
uint32_t countWindowMs    = 0;
uint32_t countWindowStart = 0;
uint32_t countWindowTs    = 0;    // pulse timestamp at the window boundary
uint16_t rpmAvgSize  = DEFAULT_RPM_AVG;   // RPM_AVG,<n>

// Live RPM conditioning, ahead of the control loop.  Electrical noise on the
// pickup reads as a genuine RPM change, and the PID answers it by moving the
// brake — which is one source of the oscillation seen on the reference system.
// Left OFF by default to match the settings Matt is running today.
//   0 = off, 1 = median-of-3 on pulse intervals, 2 = slew limit, 3 = both
// Four independent gates, each switchable on its own, applied in this order:
//   1. glitch reject  - an edge sooner than the fastest the engine could turn
//   2. median         - kills an isolated bad interval with no added lag
//   3. ratio gate     - rejects a sample wildly away from what we are seeing
//   4. slew limit     - caps the rate of change
// Measured noise on this rig arrives around 2500 Hz while the real pickup runs
// near 100 Hz, so gate 1 alone removes it; the rest are there for the residue.
// The band the engine actually runs in. Anything computed outside it is not
// a slow or fast engine, it is a bad reading, so it is discarded rather than
// averaged in. The upper end also sets the interrupt glitch threshold.
float    rpmBandMin    = 800.0f;          // RPM_BAND,<min>,<max>
float    rpmBandMax    = 6000.0f;

// Optionally carry the trend through a bad reading instead of holding the
// last value. Strictly bounded: a straight line projected through a long
// noise burst would invent a runaway, and the brake would act on it. After
// rpmExtrapMax consecutive substitutions it gives up and holds, and every
// substituted sample is counted so the operator can see how much of the
// trace was estimated rather than measured.
#define RPM_HIST_MAX 10
bool     rpmExtrapOn   = false;           // RPM_EXTRAP,<0|1>,<points>,<maxrun>
uint8_t  rpmExtrapN    = 4;               // good samples used for the fit
uint8_t  rpmExtrapMax  = 5;               // consecutive substitutions allowed
uint32_t rpmExtrapolated = 0;             // running count, reported live
uint8_t  extrapRun     = 0;
uint8_t  rpmMedianN    = 3;               // RPM_MEDIAN,<1|3|5|7>  (1 = off)
float    rpmRatioGate  = 0.0f;            // RPM_RATIO,<x>  (0 = off)
float    rpmMaxSlew    = 0.0f;            // RPM_SLEW,<RPM/s> (0 = off)
float    driveRatio    = 1.0f;            // RATIO,<r> — engine rev per sensor rev

// =============================================
// Load Cell / Torque
// =============================================
float    torqueBuffer[TORQUE_AVG_SIZE];
uint16_t torqueBufIdx   = 0;
uint16_t torqueBufCount = 0;
float    currentTorque   = 0.0f;
float    loadCellRaw_mV  = 0.0f;  // Raw ADC millivolts from load cell channel
float    loadCellScale   = 1.0f;  // Converts mV to force (N)      — CAL_SCALE
float    leverArm        = 0.3f;  // Lever arm in meters           — CAL_ARM
float    loadCellOffset_mV = 0.0f; // Tare offset in mV
// The load cell sits under a 3-point mount, so it only carries a fraction of
// the true reaction force.  This multiplies the cell's own calibration to get
// back to actual torque; kept separate from loadCellScale so the mechanical
// factor can be re-derived on the stand without disturbing the sensor cal.
float    mechRatio       = 1.0f;  // Mechanical correction factor  — CAL_MECH

// =============================================
// ADC Readings
//   ADS1115 ch1 → load cell amplifier  (stored in loadCellRaw_mV above)
//   ADS1115 ch2 → auxiliary 0-10V in   (brake line pressure transducer)
// =============================================
float spareAux_mV = 0.0f;  // DFRobot ch2 — spare, no longer carries pressure

// =============================================
// Brake line pressure (auxiliary channel)
// Stays inert until CAL_PRESS supplies a non-zero scale, so an uncalibrated
// channel can never trip the interlock on the stand.
// =============================================
float pressurePSI        = 0.0f;
float pressRaw_mV        = 0.0f;   // millivolts AT THE SENSOR (divider undone)
float pressureOffset_mV  = 0.0f;
float pressureScale      = 0.0f;    // PSI per mV; 0 = not calibrated
float pressureLimitPSI   = 1500.0f; // Hose rating — brake stops advancing above this
bool  pressureCalValid   = false;
// If the signal is divided down before the ADC (needed when the ADS1115 runs at
// 3.3 V, since this sensor swings to 4.5 V), this puts the reading back into
// sensor volts so the 500-4500 mV datasheet calibration still applies.
// 1.0 = wired straight through, 2.0 = a 2:1 divider.
float pressDivider       = 1.0f;   // PRESS_DIV,<ratio>
uint8_t pressPgaIdx      = 0;      // PRESS_PGA,<0-5>; 0 = +/-6.144 V

// =============================================
// Runtime configuration (was compile-time)
// =============================================
uint8_t pulsesPerRev   = DEFAULT_PULSES_PER_REV;  // TEETH,<n>
long    brakeMinSteps  = 0;                       // BRAKE_RANGE,<min>,<max>
long    brakeMaxSteps  = BRAKE_MAX_STEPS_DEF;
float   brakePreloadPct = 20.0f;                  // PRELOAD,<pct> — applied on START
bool    dirInverted    = true;                    // INVERT,<0|1> (common-anode default)
// Mirrors of what was handed to AccelStepper, which has no getters for these.
float   stepperMaxSpeed = STEPPER_MAX_SPEED;      // STEPPER_SPEED,<steps/s>
float   stepperAccel    = STEPPER_ACCEL;          // STEPPER_ACCEL,<steps/s^2>

// =============================================
// Cam geometry — stepper position to actual brake application
// The pusher is driven by a cam through a planetary gearbox, so brake effort is
// NOT linear in stepper steps: the same 100 steps near the base circle and near
// full lift do very different things.  This maps between the two.
//   0 = linear   — brake fraction == step fraction (previous behaviour)
//   1 = eccentric — circular cam, lift ∝ (1 - cos θ); θ from CAM_SPD and range
//   2 = table    — measured points, linearly interpolated between
// The mapping always drives the reported brake %, so the operator sees real
// brake application.  CAM_LIN additionally applies the inverse in the control
// path, which linearises the plant for the PID — off by default, because it
// changes what a given gain means.
// =============================================
#define CAM_MAX_PTS 12
// Defaults to linear, not eccentric: a round cam on a lever is close to
// (1 - cos θ) but the lever geometry shifts it, and quietly reporting a curve
// we have not measured would be worse than reporting plain travel. The steps
// per degree below is the real gearing, so switching to the eccentric model —
// or entering a measured table — is a one-click change on the stand.
uint8_t camModel       = 0;                      // CAM_MODEL,<0|1|2>
float   camStepsPerDeg = CAM_STEPS_PER_DEG_DEF;  // CAM_SPD,<steps per degree>
bool    camLinearize   = false;    // CAM_LIN,<0|1>
uint8_t camNPts        = 0;        // CAM_NPTS,<n>
float   camStepPct[CAM_MAX_PTS];   // CAM_PT,<i>,<stepPct>,<brakePct>
float   camBrakePct[CAM_MAX_PTS];

// =============================================
// PID Controller
// =============================================
// Two gain sets, as on the reference system: the HOLD set has to catch the
// engine hard while the operator opens the throttle (otherwise it runs away),
// while the SWEEP set is deliberately softer because the hydraulic brake is
// near-instantaneous and a stiff loop just oscillates.  PID,... sets hold gains;
// PID_SWEEP,... sets sweep gains; the active set follows the state machine.
float pidKp       = DEFAULT_PID_KP;
float pidKi       = DEFAULT_PID_KI;
float pidKd       = DEFAULT_PID_KD;
float sweepKp     = DEFAULT_SWEEP_KP;
float sweepKi     = DEFAULT_SWEEP_KI;
float sweepKd     = DEFAULT_SWEEP_KD;
float actKp       = DEFAULT_PID_KP;   // gains in force this tick (reported in CFG)
float actKi       = DEFAULT_PID_KI;
float actKd       = DEFAULT_PID_KD;
float pidIntegral = 0.0f;
float pidLastErr  = 0.0f;
float pidOutput   = 0.0f;

// =============================================
// Fault state
//   FAULT_TACH     – no tach pulses while the loop is controlling the brake
//   FAULT_PRESSURE – brake line pressure above the configured hose limit
// Faults latch until the condition clears (tach) or the run resets (pressure),
// and are reported both as ERR lines and in the DATA frame.
// =============================================
bool  faultTach     = false;
bool  faultPressure = false;
float heldBrakeSteps = 0.0f;   // brake position frozen at the moment of a fault

// =============================================
// Dyno State
// =============================================
DynoState currentState   = STATE_IDLE;
float targetRPM          = 0.0f;
float holdRPM            = 2000.0f;
float sweepStartRPM      = 2000.0f;
float sweepEndRPM        = 5500.0f;
float sweepRate          = 500.0f;
unsigned long sweepStartTime = 0;
// After START the brake sits at preload and the loop stays out of the way until
// the operator brings the engine up to holdRPM — matching the reference system,
// where the brake "waits" below the trigger speed instead of fighting the
// throttle all the way up from idle.
bool  holdEngaged        = false;

// =============================================
// End of run: bringing the engine back down
// Dumping the brake the instant the operator lifts is what the reference
// system warns against for engine dynos, but holding it on while the engine
// loses power is how you stall it. So the release is a phase of its own with
// selectable behaviour, and a hard floor below which the brake always lets go.
//   0 = release immediately
//   1 = follow a descending RPM target (brake keeps the descent controlled)
//   2 = walk the brake off at a fixed rate, open loop
// =============================================
uint8_t rampDownMode   = 1;        // RAMPDOWN_MODE,<0-2>
float   rampDownRate   = 300.0f;   // RAMPDOWN_RATE,<RPM/s> for mode 1
float   rampBrakeRate  = 40.0f;    // RAMPDOWN_BRAKE,<% of travel per second> mode 2
float   cutoffRPM      = 1200.0f;  // CUTOFF_RPM — below this the brake fully releases
float   throttleOffPct = 50.0f;    // THROTTLE_OFF — torque below this % of peak = lifted
float   peakTorqueRun  = 0.0f;     // running peak torque for that comparison
unsigned long rampStartMs = 0;
float   rampStartRPM   = 0.0f;
// Safety stop: a deliberate quick linear walk to zero rather than an instant
// dump, so the engine is never unloaded in one step at whatever it was holding.
bool    stopRamping    = false;
float   stopRampRate   = 200.0f;   // STOP_RATE,<% of travel per second>

// =============================================
// SIM state (virtual engine)
// =============================================
volatile bool simMode           = SIM_MODE_DEFAULT;  // read by Core-0 ADC task too
float         simRPM            = 0.0f;   // virtual engine speed
float         simThrottle       = 0.0f;   // 0..1, ramps toward the state's target
float         brakeCommandSteps = 0.0f;   // last commanded brake pos (SIM plant input)
unsigned long simStateEnterMs   = 0;
DynoState     simPrevState      = STATE_IDLE;
uint32_t      lastSimUs         = 0;

// =============================================
// Timing
// =============================================
unsigned long lastDataReport = 0;
unsigned long lastADCRead    = 0;
unsigned long lastPIDUpdate  = 0;

// =============================================
// Serial Input Buffer
// =============================================
char    serialBuf[256];
uint8_t serialBufLen = 0;

// =============================================
// ISR — Proximity Sensor Pulse
// =============================================
void IRAM_ATTR onProximityPulse() {
    uint32_t now = micros();
    uint32_t dt  = now - lastPulseUs;
    if (dt < minPulseIntervalUs) {
        // Too soon to be real. Drop it WITHOUT advancing lastPulseUs, so a
        // burst of noise cannot walk the reference forward one glitch at a
        // time and fabricate a plausible-looking pulse train.
        tachGlitches++;
        return;
    }
    pulseIntervalUs = dt;
    lastPulseUs = now;
    pulseCount++;
    pulseTs[pulseTsIdx] = now;
    pulseTsIdx = (uint8_t)((pulseTsIdx + 1) % PULSE_TS_MAX);
}

// Fastest credible tooth spacing, from the engine ceiling and the wheel.
static void updateMinPulseInterval() {
    float ppr = (pulsesPerRev > 0) ? (float)pulsesPerRev : 1.0f;
    float rpm = (rpmBandMax > 100.0f) ? rpmBandMax : 100.0f;
    // Half the theoretical spacing, so a genuinely fast engine is never gated.
    minPulseIntervalUs = (uint32_t)(60000000.0f / (rpm * ppr) * 0.5f);
}

// =============================================
// Pressure ADC gain
// Index 0 (+/-6.144 V) is the default because this sensor reaches 4.5 V when
// wired straight through; the narrower ranges are only usable behind a divider.
// =============================================
static void applyPressureGain() {
    switch (pressPgaIdx) {
        case 1:  pressAdc.setGain(GAIN_ONE);       break;  // +/-4.096 V
        case 2:  pressAdc.setGain(GAIN_TWO);       break;  // +/-2.048 V
        case 3:  pressAdc.setGain(GAIN_FOUR);      break;  // +/-1.024 V
        case 4:  pressAdc.setGain(GAIN_EIGHT);     break;  // +/-0.512 V
        case 5:  pressAdc.setGain(GAIN_SIXTEEN);   break;  // +/-0.256 V
        default: pressAdc.setGain(GAIN_TWOTHIRDS); break;  // +/-6.144 V
    }
}

// =============================================
// Rolling Average Helper
// =============================================
static float addToAvg(float* buf, uint16_t &idx, uint16_t &count,
                      uint16_t maxSize, float value) {
    buf[idx] = value;
    idx = (idx + 1) % maxSize;
    if (count < maxSize) count++;

    float sum = 0.0f;
    for (uint16_t i = 0; i < count; i++) sum += buf[i];
    return sum / (float)count;
}

// =============================================
// RPM Calculation
// =============================================
static uint32_t medianOf(uint32_t *v, uint8_t n) {
    // Insertion sort on a copy; n is at most RPM_MEDIAN_MAX.
    uint32_t a[RPM_MEDIAN_MAX];
    for (uint8_t i = 0; i < n; i++) a[i] = v[i];
    for (uint8_t i = 1; i < n; i++) {
        uint32_t k = a[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && a[j] > k) { a[j + 1] = a[j]; j--; }
        a[j + 1] = k;
    }
    return a[n / 2];
}

// Recent genuine samples, newest first. Extrapolated values are deliberately
// NOT fed back in, or the estimate would compound on its own output.
static uint32_t rpmHistT[RPM_HIST_MAX] = {0};
static float    rpmHistV[RPM_HIST_MAX] = {0};
static uint8_t  rpmHistN = 0;

static void pushRpmHistory(uint32_t t, float v) {
    for (int8_t i = RPM_HIST_MAX - 1; i > 0; i--) {
        rpmHistT[i] = rpmHistT[i - 1];
        rpmHistV[i] = rpmHistV[i - 1];
    }
    rpmHistT[0] = t;
    rpmHistV[0] = v;
    if (rpmHistN < RPM_HIST_MAX) rpmHistN++;
}

// Least-squares line through the recent history, evaluated at this pulse.
// Time is measured relative to the newest sample so the numbers stay small
// and the micros() rollover is handled by the signed difference.
static bool rpmExtrapolate(uint32_t nowUs, float *out) {
    uint8_t n = rpmExtrapN;
    if (n > rpmHistN) n = rpmHistN;
    if (n > RPM_HIST_MAX) n = RPM_HIST_MAX;
    if (n < 2) return false;
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (uint8_t i = 0; i < n; i++) {
        double x = (double)(int32_t)(rpmHistT[i] - rpmHistT[0]) * 1e-6;
        double y = (double)rpmHistV[i];
        sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    double denom = (double)n * sxx - sx * sx;
    if (fabs(denom) < 1e-12) return false;     // samples share a timestamp
    double slope = ((double)n * sxy - sx * sy) / denom;
    double intercept = (sy - slope * sx) / (double)n;
    double x = (double)(int32_t)(nowUs - rpmHistT[0]) * 1e-6;
    double v = intercept + slope * x;
    if (v < rpmBandMin) v = rpmBandMin;        // never leaves the valid band
    if (v > rpmBandMax) v = rpmBandMax;
    *out = (float)v;
    return true;
}

static uint32_t intervalHist[RPM_MEDIAN_MAX] = {0};
static uint8_t  intervalHistN   = 0;
static uint32_t lastSeenPulse   = 0;

// Pulses per window: how many teeth went by, over the time they took.
//
// The time is measured between the pulses themselves rather than between
// the window edges, so the count is always a whole number of intervals and
// the reading does not step. Dividing a whole pulse count by a fixed window
// would quantise the answer to one pulse - about 200 RPM at 1500 on three
// teeth in a 100 ms window, which a control loop would chase. The window
// then only sets how often the reading updates and how many pulses it
// averages over, which is what it is actually for.
static void updateRPMCounted() {
    uint32_t nowMs = millis();
    if (countWindowMs == 0) {
        countWindowMs = nowMs;
        noInterrupts();
        countWindowStart = pulseCount;
        countWindowTs    = lastPulseUs;
        interrupts();
        return;
    }
    if ((nowMs - countWindowMs) < rpmCountWindowMs) return;

    noInterrupts();
    uint32_t c  = pulseCount;
    uint32_t ts = lastPulseUs;
    interrupts();

    uint32_t n = c - countWindowStart;
    uint8_t ppr = pulsesPerRev > 0 ? pulsesPerRev : 1;
    if (n == 0) {
        // No teeth at all this window. That is a stopped engine, which is a
        // reading in its own right, not a bad sample to be filtered out.
        rpmCounted = 0.0f;
    } else {
        uint32_t span = ts - countWindowTs;      // exactly n intervals
        if (span > 0) {
            float v = (float)n * 60000000.0f
                      / ((float)span * (float)ppr) * driveRatio;
            if (v >= rpmBandMin && v <= rpmBandMax) rpmCounted = v;
        }
    }
    countWindowMs    = nowMs;
    countWindowStart = c;
    countWindowTs    = ts;
    if (rpmSource == 1) currentRPM = rpmCounted;
}

static void updateRPM() {
    noInterrupts();
    uint32_t interval  = pulseIntervalUs;
    uint32_t lastPulse = lastPulseUs;
    uint32_t count     = pulseCount;
    interrupts();

    // micros() AFTER the snapshot, and the age compared SIGNED. Read the
    // other way round, a pulse landing between the two reads leaves
    // lastPulse in the future, and the unsigned difference wraps to about
    // 4.3e9 - comfortably past the timeout. That fired a spurious timeout
    // several times a second at idle: the reading dropped to exactly zero
    // for one frame and the averaging buffer was thrown away, while the
    // pickup was working perfectly.
    uint32_t now = micros();
    int32_t age = (int32_t)(now - lastPulse);
    if (age > (int32_t)RPM_TIMEOUT_US) {
        currentRPM    = 0.0f;
        rpmGap        = 0.0f;
        rpmRev        = 0.0f;
        rpmCounted    = 0.0f;
        rpmBufCount   = 0;
        rpmBufIdx     = 0;
        intervalHistN = 0;
        rpmHistN = 0;                  // stale trend must not seed a restart
        extrapRun = 0;
        lastSeenPulse = count;
        return;
    }

    // One sample per pulse. Running this every loop() iteration would refill
    // the averaging buffer in microseconds and average nothing at all.
    if (count == lastSeenPulse || interval == 0) return;
    lastSeenPulse = count;

    // ---- gate 2: median of the last N intervals -------------------------
    uint32_t use = interval;
    uint8_t  win = rpmMedianN;
    if (win > RPM_MEDIAN_MAX) win = RPM_MEDIAN_MAX;
    if (win >= 3) {
        if (!(win & 1)) win--;                       // odd windows only
        for (int8_t i = (int8_t)win - 1; i > 0; i--) intervalHist[i] = intervalHist[i - 1];
        intervalHist[0] = interval;
        if (intervalHistN < win) intervalHistN++;
        if (intervalHistN >= win) use = medianOf(intervalHist, win);
    } else {
        intervalHistN = 0;
    }

    uint8_t ppr = pulsesPerRev > 0 ? pulsesPerRev : 1;
    float instantRPM = 60000000.0f / ((float)use * (float)ppr) * driveRatio;

    // ---- gate 2b: validity band ----------------------------------------
    // Outside the range the engine can actually run in, this is a bad
    // reading, not a measurement. Dropped here rather than in the ISR, so
    // the pulse still counts as the tach being alive and a slow engine is
    // never mistaken for a dead pickup.
    bool estimated = false;
    if (instantRPM < rpmBandMin || instantRPM > rpmBandMax) {
        tachGlitches++;
        float est;
        if (rpmExtrapOn && extrapRun < rpmExtrapMax &&
            rpmExtrapolate(lastPulse, &est)) {
            instantRPM = est;
            estimated = true;
            extrapRun++;
            rpmExtrapolated++;
        } else {
            return;                    // hold the last good value
        }
    }

    // ---- gate 3: ratio gate --------------------------------------------
    // Discard a sample that sits a long way from what we are already seeing.
    // Only once there is an established reading to compare against, so it can
    // never block the engine from being picked up in the first place.
    if (rpmRatioGate > 1.0f && rpmBufCount > 0 && currentRPM > 100.0f) {
        if (instantRPM > currentRPM * rpmRatioGate ||
            instantRPM < currentRPM / rpmRatioGate) {
            tachGlitches++;
            return;
        }
    }

    // ---- gate 4: slew limit --------------------------------------------
    if (rpmMaxSlew > 0.0f && rpmBufCount > 0 && currentRPM > 0.0f) {
        float maxDelta = rpmMaxSlew * ((float)use * 1e-6f);
        float delta = instantRPM - currentRPM;
        if (delta >  maxDelta) instantRPM = currentRPM + maxDelta;
        if (delta < -maxDelta) instantRPM = currentRPM - maxDelta;
    }

    if (!estimated) {
        extrapRun = 0;
        pushRpmHistory(lastPulse, instantRPM);
    }

    // Whole-revolution period: the gap between this pulse and the one a
    // full revolution back. Every tooth contributes exactly once, so their
    // spacing errors cancel instead of alternating.
    if (ppr >= 1 && ppr < PULSE_TS_MAX) {
        noInterrupts();
        uint8_t i = pulseTsIdx;
        uint32_t newest = pulseTs[(uint8_t)((i + PULSE_TS_MAX - 1) % PULSE_TS_MAX)];
        uint32_t older  = pulseTs[(uint8_t)((i + PULSE_TS_MAX - 1 - ppr) % PULSE_TS_MAX)];
        interrupts();
        uint32_t revUs = newest - older;
        if (older != 0 && revUs > 1000 && revUs < 60000000UL) {
            float v = 60000000.0f / (float)revUs * driveRatio;
            if (v >= rpmBandMin && v <= rpmBandMax) {
                rpmRev = v;
                if (rpmSource == 2) currentRPM = rpmRev;
            }
        }
    }

    uint16_t avg = rpmAvgSize;
    if (avg < 1) avg = 1;
    if (avg > RPM_AVG_MAX) avg = RPM_AVG_MAX;
    rpmGap = addToAvg(rpmBuffer, rpmBufIdx, rpmBufCount, avg, instantRPM);
    if (rpmSource == 0) currentRPM = rpmGap;
}

// =============================================
// Tach liveness — has the proximity sensor pulsed recently?
// In SIM there is no real sensor, so the virtual engine always counts as live.
// =============================================
static bool tachIsLive() {
    if (simMode) return true;
    noInterrupts();
    uint32_t lastPulse = lastPulseUs;
    interrupts();
    return (micros() - lastPulse) < RPM_FAULT_TIMEOUT_US;
}

// =============================================
// ADC Reading (DFRobot 0-10V module via ADS1115)
// getValue(1) = load cell amplifier output -> torque
// getValue(2) = auxiliary 0-10V input      -> brake line pressure (AiM MC-327)
// The library numbers the two input terminals 1 and 2. This block used to say
// Ch0/Ch1, which matched neither the library nor the code below.
// =============================================
// ADC task running on Core 0 — reads I2C without blocking stepper
static void adcTask(void* param) {
    (void)param;
    for (;;) {
        if (!adcOk) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

        // Handle tare request from main core
        if (tareRequested) {
            loadCellOffset_mV = adc.getValue(1);
            isTared = true;
            tareRequested = false;
        }

        // Channel 1: Load cell amplifier output → torque
        // Skipped in SIM: torque is injected from the recorded curve there, so we
        // must not let this task overwrite currentTorque/loadCellRaw_mV.
        if (!simMode) {
            float raw = adc.getValue(1);
            loadCellRaw_mV = raw;
            float force  = (raw - loadCellOffset_mV) * loadCellScale;
            float torque = force * leverArm * mechRatio;
            currentTorque = addToAvg(torqueBuffer, torqueBufIdx, torqueBufCount,
                                     TORQUE_AVG_SIZE, torque);
        }

        // Channel 2 of the DFRobot module is now spare — pressure moved to its
        // own ADS1115 on the second bus.
        spareAux_mV = adc.getValue(2);

        // Brake line pressure, from the dedicated ADS1115 at 0x49 on Wire1.
        if (pressAdcOk) {
            int16_t counts = pressAdc.readADC_SingleEnded(PRESS_ADC_CHANNEL);
            // computeVolts() honours whatever gain is set; multiply back up by the
            // divider so pressRaw_mV is always millivolts at the sensor itself.
            pressRaw_mV = pressAdc.computeVolts(counts) * 1000.0f * pressDivider;
            if (pressureCalValid) {
                pressurePSI = (pressRaw_mV - pressureOffset_mV) * pressureScale;
            } else {
                pressurePSI = 0.0f;
            }
        } else {
            pressRaw_mV = 0.0f;
            pressurePSI = 0.0f;
        }

        vTaskDelay(pdMS_TO_TICKS(ADC_READ_MS));
    }
}

// =============================================
// PID — Reverse-acting (controls brake to hold RPM)
// =============================================
// =============================================
// Cam mapping — step fraction <-> brake fraction, both on 0..1
// Forward answers "how much brake is actually applied at this position".
// Inverse answers "what position gives this much brake", used to linearise the
// plant when CAM_LIN is on.  Both fall back to identity if the selected model
// has no usable parameters, so a half-configured cam can never distort control.
// =============================================
static float camFullAngleRad() {
    if (camStepsPerDeg <= 0.0f) return 0.0f;
    float deg = (float)(brakeMaxSteps - brakeMinSteps) / camStepsPerDeg;
    if (deg <= 0.0f) return 0.0f;
    // Past 180 degrees a circular cam starts retracting again — the same
    // over-centre travel the range limit exists to prevent.
    if (deg > 180.0f) deg = 180.0f;
    return deg * 0.01745329252f;
}

static float camForward(float u) {
    u = constrain(u, 0.0f, 1.0f);

    if (camModel == 1) {                       // eccentric: lift ∝ (1 - cos θ)
        float th = camFullAngleRad();
        float denom = 1.0f - cosf(th);
        if (th <= 0.0f || denom < 1e-6f) return u;
        return (1.0f - cosf(u * th)) / denom;
    }

    if (camModel == 2 && camNPts >= 2) {       // measured table
        float x = u * 100.0f;
        if (x <= camStepPct[0]) return camBrakePct[0] * 0.01f;
        for (uint8_t i = 1; i < camNPts; i++) {
            if (x <= camStepPct[i]) {
                float dx = camStepPct[i] - camStepPct[i - 1];
                float t = (dx > 1e-6f) ? (x - camStepPct[i - 1]) / dx : 0.0f;
                return (camBrakePct[i - 1]
                        + t * (camBrakePct[i] - camBrakePct[i - 1])) * 0.01f;
            }
        }
        return camBrakePct[camNPts - 1] * 0.01f;
    }

    return u;                                   // linear
}

static float camInverse(float v) {
    v = constrain(v, 0.0f, 1.0f);

    if (camModel == 1) {
        float th = camFullAngleRad();
        float denom = 1.0f - cosf(th);
        if (th <= 0.0f || denom < 1e-6f) return v;
        return acosf(constrain(1.0f - v * denom, -1.0f, 1.0f)) / th;
    }

    if (camModel == 2 && camNPts >= 2) {
        float y = v * 100.0f;
        if (y <= camBrakePct[0]) return camStepPct[0] * 0.01f;
        for (uint8_t i = 1; i < camNPts; i++) {
            if (y <= camBrakePct[i]) {
                float dy = camBrakePct[i] - camBrakePct[i - 1];
                float t = (dy > 1e-6f) ? (y - camBrakePct[i - 1]) / dy : 0.0f;
                return (camStepPct[i - 1]
                        + t * (camStepPct[i] - camStepPct[i - 1])) * 0.01f;
            }
        }
        return camStepPct[camNPts - 1] * 0.01f;
    }

    return v;
}

static float integralCeiling() {
    float span = (float)(brakeMaxSteps - brakeMinSteps);
    return span / (actKi > 0.001f ? actKi : 1.0f);
}

static float runPID(float setpoint, float measurement, float dt) {
    float error = measurement - setpoint;

    float derivative = (dt > 0.001f) ? (error - pidLastErr) / dt : 0.0f;
    pidLastErr = error;

    // Conditional integration.  The brake is a one-sided actuator — it cannot
    // pull below its minimum — so the integrator is clamped to [0, ceiling] and
    // is not advanced when the output is already hard against a limit and this
    // error would push it further in.  The old two-sided clamp let the integral
    // run to -brakeMax/Ki during a dropout, which kept the brake released for
    // seconds after the signal returned.
    float candidate = constrain(pidIntegral + error * dt, 0.0f, integralCeiling());
    float lo = (float)brakeMinSteps, hi = (float)brakeMaxSteps;
    float unsat = actKp * error + actKi * candidate + actKd * derivative;
    bool pushingIntoLimit = (unsat > hi && error > 0.0f) || (unsat < lo && error < 0.0f);
    if (!pushingIntoLimit) pidIntegral = candidate;

    pidOutput = actKp * error + actKi * pidIntegral + actKd * derivative;
    return constrain(pidOutput, lo, hi);
}

// =============================================
// Select the gain set for the current phase (see the PID globals for why).
// =============================================
static void applyGainsForState() {
    if (currentState == STATE_SWEEP) {
        actKp = sweepKp; actKi = sweepKi; actKd = sweepKd;
    } else {
        actKp = pidKp;   actKi = pidKi;   actKd = pidKd;
    }
}

// =============================================
// Brake control tick — the single path from setpoint to stepper position for
// every closed-loop state, so the safety interlocks cannot be bypassed by one
// state forgetting to apply them.
// =============================================
static void controlBrake(float setpoint, float dt) {
    applyGainsForState();

    // ---- Tach loss ----------------------------------------------------
    // A dead pickup reads as 0 RPM.  Fed to the loop that looks identical to
    // "engine far below target", whose correct answer is to release the brake
    // completely — except the engine is at wide-open throttle and the brake is
    // the only thing holding it.  Freeze the brake where it is instead.
    if (!tachIsLive()) {
        if (!faultTach) {
            faultTach = true;
            heldBrakeSteps = brakeCommandSteps;
            Serial.println("ERR,TACH_LOST");
        }
        stepper.moveTo((long)heldBrakeSteps);
        return;                       // integrator frozen while blind
    }
    if (faultTach) {
        // Bumpless resume: back-calculate the integral so the first controlled
        // output matches the position we were holding, instead of stepping.
        faultTach = false;
        float error = currentRPM - setpoint;
        if (actKi > 0.001f) {
            pidIntegral = constrain((heldBrakeSteps - actKp * error) / actKi,
                                    0.0f, integralCeiling());
        }
        pidLastErr = error;
        Serial.println("ACK,TACH_RECOVERED");
    }

    float brakePos = runPID(setpoint, currentRPM, dt);

    // Optional plant linearisation: treat the loop's output as a brake demand
    // and ask the cam what position delivers it, so a given gain means the same
    // thing at the base circle as it does near full lift.
    if (camLinearize) {
        float span = (float)(brakeMaxSteps - brakeMinSteps);
        if (span > 0.0f) {
            float demand = (brakePos - (float)brakeMinSteps) / span;
            brakePos = (float)brakeMinSteps + camInverse(demand) * span;
        }
    }

    // ---- Brake line over-pressure --------------------------------------
    // Deliberately does NOT dump the brake: that is the runaway case above.
    // Stop advancing, raise the alarm, and let the operator lift off.
    if (pressureCalValid && pressurePSI > pressureLimitPSI) {
        if (!faultPressure) {
            faultPressure = true;
            Serial.println("ERR,PRESSURE_LIMIT");
        }
        if (brakePos > brakeCommandSteps) brakePos = brakeCommandSteps;
    }

    stepper.moveTo((long)brakePos);
    brakeCommandSteps = brakePos;
}

// =============================================
// Stepper Enable / Disable
// Common-anode: ENA active = driver DISABLED (inhibit signal)
// So enable driver = opto OFF (GPIO HIGH), disable = opto ON (GPIO LOW)
// =============================================
// A timed sweep temporarily lowers the speed ceiling so one moveTo() covers
// the whole travel at a steady rate. The normal ceiling is put back as soon
// as the move finishes, or the moment anything else commands the brake -
// leaving it lowered would silently slow a STOP.
bool sweepSpeedActive = false;

static void clearSweepSpeed() {
    if (sweepSpeedActive) {
        stepper.setMaxSpeed(stepperMaxSpeed);
        sweepSpeedActive = false;
    }
}

static void enableStepper(bool en) {
    digitalWrite(PIN_STEPPER_ENABLE, en ? HIGH : LOW);
}

// =============================================
// PID Reset
// =============================================
static void resetPID() {
    pidIntegral = 0.0f;
    pidLastErr  = 0.0f;
    pidOutput   = 0.0f;
}

// =============================================
// Check system readiness
// =============================================
static bool isReady() {
    if (simMode) return true;              // SIM needs no real homing/tare/ADC
    return isHomed && isTared && adcOk;
}

// =============================================
// Send readiness status to RPi
// =============================================
// Field 5 (sim) is appended, not inserted — older GUI builds parse the first
// four fields and ignore the rest.
static void sendReadyStatus() {
    Serial.printf("READY,%d,%d,%d,%d,%d,%d\n",
        isReady() ? 1 : 0,
        isHomed ? 1 : 0,
        isTared ? 1 : 0,
        (adcOk || simMode) ? 1 : 0,
        simMode ? 1 : 0,
        pressAdcOk ? 1 : 0);
}

// =============================================
// Reset to idle — release brake, reset state
// =============================================
static void resetToIdle() {
    currentState = STATE_IDLE;
    enableStepper(true);
    stepper.moveTo(0);       // Release brake fully (below the control floor)
    resetPID();
    targetRPM = 0.0f;
    brakeCommandSteps = 0.0f;
    heldBrakeSteps = 0.0f;
    faultTach = false;       // engine is stopping; a quiet tach is expected now
    faultPressure = false;
    stopRamping = false;
    Serial.println("ACK,RESET_IDLE");
}

// Defined further down with the rest of the end-of-run logic; the RAMPDOWN
// command needs it here.
static void enterRampDown();

// =============================================
// Process a single serial command
// =============================================
static void processCommand(const char* cmd) {
    String s(cmd);
    s.trim();
    if (s.length() == 0) return;

    Serial.print("ACK,");
    Serial.println(s);

    // ---- Heartbeat ----
    if (s == "PING") {
        return;
    }

    // ---- Query readiness ----
    if (s == "READY?") {
        sendReadyStatus();
        return;
    }

    // ---- Toggle SIM (virtual engine) mode ----
    if (s.startsWith("SIM,")) {
        simMode = (s.substring(4).toInt() != 0);
        if (simMode) {
            isHomed = true; isTared = true;      // no real home/tare needed
            currentState = STATE_IDLE;
            simRPM = 0.0f; simThrottle = 0.0f; brakeCommandSteps = 0.0f;
            stepper.setCurrentPosition(0);
            resetPID(); targetRPM = 0.0f;
            faultTach = false; faultPressure = false; holdEngaged = false;
        } else {
            resetToIdle();                       // don't leave a run active on the (dead) real tach
            isHomed = false; isTared = false;    // require real home/tare again
        }
        sendReadyStatus();
        return;
    }

    // ---- Home the stepper (uses limit switch) ----
    if (s == "HOME") {
        if (simMode) {                            // no limit switch needed in SIM
            stepper.setCurrentPosition(0);
            stepper.setMaxSpeed(STEPPER_MAX_SPEED);
            isHomed = true;
            currentState = STATE_IDLE;
            brakeCommandSteps = 0.0f;
            Serial.println("ACK,HOME_COMPLETE");
            return;
        }
        currentState = STATE_HOMING;
        isHomed = false;
        enableStepper(true);
        stepper.setMaxSpeed(HOMING_SPEED);
        // Back off by a few times the real travel rather than a flat -100000.
        // With 250 steps of range that is about two seconds to discover a dead
        // limit switch, instead of the three minutes the old figure took.
        long sweep = (brakeMaxSteps - brakeMinSteps) * 3 + 200;
        stepper.moveTo(stepper.currentPosition() - sweep);
        Serial.printf("[DBG] HOME: limitSW=%d pos=%ld dist=%ld\n",
            digitalRead(PIN_LIMIT_SWITCH), stepper.currentPosition(), stepper.distanceToGo());
        return;
    }

    // ---- Manually mark current position as home ----
    if (s == "HOME_SET") {
        stepper.setCurrentPosition(0);
        stepper.setMaxSpeed(STEPPER_MAX_SPEED);
        isHomed = true;
        currentState = STATE_IDLE;
        return;
    }

    // ---- Emergency stop ----
    if (s == "STOP") {
        // Only ramp when there is load to shed. From IDLE/MANUAL/HOMING there is
        // nothing to protect, so stop means stop.
        bool loaded = (currentState == STATE_SWEEP || currentState == STATE_SWEEP_DONE ||
                       currentState == STATE_RAMP_DOWN ||
                       (currentState == STATE_HOLD_RPM && holdEngaged));
        if (loaded && brakeCommandSteps > (float)brakeMinSteps + 0.5f) {
            stopRamping = true;
            enterRampDown();
            Serial.println("ACK,STOP_RAMP");
        } else {
            resetToIdle();
        }
        return;
    }

    // ---- Absolute brake position ----
    // BRAKE_SWEEP,<target>,<ms> - traverse to target over roughly ms, as one
    // continuous move. Re-commanding a moving target at 20 Hz instead makes
    // AccelStepper decelerate to a stop at every intermediate point, which
    // is a lurch 20 times a second rather than a smooth traverse.
    if (s.startsWith("BRAKE_SWEEP,")) {
        int i1 = s.indexOf(',', 12);
        if (i1 < 0) {
            Serial.println("ERR,BRAKE_SWEEP needs target,ms");
            return;
        }
        long pos = constrain(s.substring(12, i1).toInt(), 0L, brakeMaxSteps);
        long ms  = s.substring(i1 + 1).toInt();
        if (ms < 50) {
            Serial.println("ERR,BRAKE_SWEEP duration must be >= 50 ms");
            return;
        }
        clearSweepSpeed();
        long dist = labs(pos - stepper.currentPosition());
        currentState = STATE_MANUAL;
        enableStepper(true);
        if (dist > 0) {
            float v = (float)dist / ((float)ms / 1000.0f);
            if (v < 1.0f) v = 1.0f;
            // Never faster than the configured ceiling: a short duration is a
            // request, not permission to exceed the machine's limit.
            if (v > stepperMaxSpeed) v = stepperMaxSpeed;
            stepper.setMaxSpeed(v);
            sweepSpeedActive = true;
        }
        stepper.moveTo(pos);
        brakeCommandSteps = (float)pos;
        return;
    }
    if (s.startsWith("BRAKE,")) {
        clearSweepSpeed();
        // Manual moves may go to 0 (fully released) but never past the
        // configured maximum — over-stroking the cam drives it back over centre
        // and can break the linkage.
        long pos = constrain(s.substring(6).toInt(), 0L, brakeMaxSteps);
        currentState = STATE_MANUAL;
        enableStepper(true);
        stepper.moveTo(pos);
        brakeCommandSteps = (float)pos;
        return;
    }

    // ---- Relative brake move ----
    if (s.startsWith("BRAKE_REL,")) {
        long steps = s.substring(10).toInt();
        currentState = STATE_MANUAL;
        enableStepper(true);
        long dest = constrain(stepper.currentPosition() + steps, 0L, brakeMaxSteps);
        stepper.moveTo(dest);
        brakeCommandSteps = (float)dest;
        return;
    }

    // ---- Set hold RPM (before starting a run) ----
    if (s.startsWith("HOLD,")) {
        holdRPM = s.substring(5).toFloat();
        return;
    }

    // ---- Configure sweep parameters ----
    if (s.startsWith("SWEEP,")) {
        int i1 = s.indexOf(',', 6);
        int i2 = s.indexOf(',', i1 + 1);
        if (i1 > 0 && i2 > 0) {
            sweepStartRPM = s.substring(6, i1).toFloat();
            sweepEndRPM   = s.substring(i1 + 1, i2).toFloat();
            sweepRate     = s.substring(i2 + 1).toFloat();
        }
        return;
    }

    // ---- START: begin hold-RPM phase (operator procedure step 2) ----
    if (s == "START") {
        if (!isReady()) {
            Serial.println("ERR,NOT_READY");
            sendReadyStatus();
            return;
        }
        currentState = STATE_HOLD_RPM;
        targetRPM = holdRPM;
        enableStepper(true);
        resetPID();
        faultTach = false;
        faultPressure = false;
        holdEngaged = false;
        peakTorqueRun = 0.0f;      // per-run, or the last pull sets this one's threshold

        // Pre-load the brake so it is already engaging when the throttle comes
        // up, rather than starting from zero and chasing the engine.
        float span = (float)(brakeMaxSteps - brakeMinSteps);
        float preload = (float)brakeMinSteps
                      + span * constrain(brakePreloadPct, 0.0f, 100.0f) / 100.0f;
        stepper.moveTo((long)preload);
        brakeCommandSteps = preload;
        heldBrakeSteps    = preload;
        Serial.println("ACK,HOLD_STARTED");
        return;
    }

    // ---- RELEASE: operator at WOT → begin sweep (step 4) ----
    if (s == "RELEASE") {
        if (currentState != STATE_HOLD_RPM) {
            Serial.println("ERR,NOT_IN_HOLD");
            return;
        }
        currentState   = STATE_SWEEP;
        sweepStartTime = millis();
        targetRPM      = sweepStartRPM;

        // Bumpless handover to the sweep gains.  resetPID() used to zero the
        // integrator here, which dropped the brake to zero at the exact moment
        // the operator was at wide-open throttle waiting for the sweep.
        applyGainsForState();
        pidLastErr = currentRPM - targetRPM;
        if (actKi > 0.001f) {
            pidIntegral = constrain((brakeCommandSteps - actKp * pidLastErr) / actKi,
                                    0.0f, integralCeiling());
        } else {
            pidIntegral = 0.0f;
        }
        Serial.println("ACK,SWEEP_STARTED");
        return;
    }

    // ---- PID tuning ----
    if (s.startsWith("PID,")) {
        int i1 = s.indexOf(',', 4);
        int i2 = s.indexOf(',', i1 + 1);
        if (i1 > 0 && i2 > 0) {
            pidKp = s.substring(4, i1).toFloat();
            pidKi = s.substring(i1 + 1, i2).toFloat();
            pidKd = s.substring(i2 + 1).toFloat();
        }
        return;
    }

    // ---- Tare the load cell (zero via ADC) ----
    if (s == "TARE") {
        if (simMode) {
            isTared = true;                 // nothing to zero against in SIM
            Serial.println("ACK,TARE_DONE");
        } else if (adcOk) {
            tareRequested = true;  // ADC task on Core 0 will handle it
            Serial.println("ACK,TARE_DONE");
        } else {
            Serial.println("ERR,ADC not ready");
        }
        return;
    }

    // ---- Load cell calibration ----
    if (s.startsWith("CAL_SCALE,")) {
        loadCellScale = s.substring(10).toFloat();
        return;
    }
    if (s.startsWith("CAL_ARM,")) {
        leverArm = s.substring(8).toFloat();
        return;
    }
    // Mechanical correction for the 3-point mount — the cell carries only part
    // of the reaction force, so this scales its reading back up to real torque.
    if (s.startsWith("CAL_MECH,")) {
        float v = s.substring(9).toFloat();
        if (v > 0.0f) mechRatio = v;
        else Serial.println("ERR,CAL_MECH must be > 0");
        return;
    }

    // ---- Brake line pressure calibration: CAL_PRESS,<offset_mV>,<psi_per_mV>
    if (s.startsWith("CAL_PRESS,")) {
        int i1 = s.indexOf(',', 10);
        if (i1 > 0) {
            pressureOffset_mV = s.substring(10, i1).toFloat();
            pressureScale     = s.substring(i1 + 1).toFloat();
            pressureCalValid  = (pressureScale != 0.0f);
            if (!pressureCalValid) pressurePSI = 0.0f;
        } else {
            Serial.println("ERR,CAL_PRESS needs offset_mV,psi_per_mV");
        }
        return;
    }
    // Divider ratio between sensor and ADC input (1.0 = wired straight through).
    if (s.startsWith("PRESS_DIV,")) {
        float v = s.substring(10).toFloat();
        if (v > 0.0f) pressDivider = v;
        else Serial.println("ERR,PRESS_DIV must be > 0");
        return;
    }
    // Pressure ADC full scale: 0=+/-6.144V 1=+/-4.096 2=+/-2.048
    //                          3=+/-1.024  4=+/-0.512 5=+/-0.256
    if (s.startsWith("PRESS_PGA,")) {
        int g = s.substring(10).toInt();
        if (g < 0 || g > 5) { Serial.println("ERR,PRESS_PGA must be 0-5"); return; }
        pressPgaIdx = (uint8_t)g;
        applyPressureGain();
        return;
    }
    if (s.startsWith("PRESS_LIMIT,")) {
        float v = s.substring(12).toFloat();
        if (v > 0.0f) pressureLimitPSI = v;
        else Serial.println("ERR,PRESS_LIMIT must be > 0");
        return;
    }

    // ---- Trigger wheel: pulses per revolution ----
    if (s.startsWith("TEETH,")) {
        int n = s.substring(6).toInt();
        if (n >= 1 && n <= 60) {
            pulsesPerRev = (uint8_t)n;
            updateMinPulseInterval();     // tooth spacing changed
            rpmBufCount = 0;              // old samples were scaled differently
            rpmBufIdx   = 0;
        } else {
            Serial.println("ERR,TEETH out of range 1-60");
        }
        return;
    }

    // ---- Drive ratio between the sensor shaft and the crank ----
    if (s.startsWith("RATIO,")) {
        float v = s.substring(6).toFloat();
        if (v > 0.0f) driveRatio = v;
        else Serial.println("ERR,RATIO must be > 0");
        return;
    }

    // ---- Live RPM conditioning: four independent gates ----
    // Gate 1 is the important one here: an edge implying a faster tooth than the
    // engine can physically turn is rejected in the interrupt, before it can
    // reach the control loop and be answered with brake.
    if (s.startsWith("RPM_BAND,")) {
        int i1 = s.indexOf(',', 9);
        if (i1 < 0) {
            Serial.println("ERR,RPM_BAND needs min,max");
            return;
        }
        float lo = s.substring(9, i1).toFloat();
        float hi = s.substring(i1 + 1).toFloat();
        if (lo >= 0.0f && hi > lo + 100.0f) {
            rpmBandMin = lo;
            rpmBandMax = hi;
            updateMinPulseInterval();
        } else {
            Serial.println("ERR,RPM_BAND needs 0 <= min and max > min + 100");
        }
        return;
    }
    // RPM_EXTRAP,<0|1>,<fit points>,<max consecutive>
    if (s.startsWith("RPM_EXTRAP,")) {
        int i1 = s.indexOf(',', 11);
        int i2 = (i1 > 0) ? s.indexOf(',', i1 + 1) : -1;
        rpmExtrapOn = ((i1 > 0 ? s.substring(11, i1) : s.substring(11)).toInt() != 0);
        if (i1 > 0) {
            int n = (i2 > 0 ? s.substring(i1 + 1, i2) : s.substring(i1 + 1)).toInt();
            if (n >= 2 && n <= RPM_HIST_MAX) rpmExtrapN = (uint8_t)n;
            else Serial.printf("ERR,RPM_EXTRAP points must be 2-%d\n", RPM_HIST_MAX);
        }
        if (i2 > 0) {
            int m = s.substring(i2 + 1).toInt();
            if (m >= 1 && m <= 50) rpmExtrapMax = (uint8_t)m;
            else Serial.println("ERR,RPM_EXTRAP max run must be 1-50");
        }
        extrapRun = 0;
        return;
    }
    // RPM_COUNT_MS,<ms> - how long each counting window lasts
    if (s.startsWith("RPM_COUNT_MS,")) {
        long v = s.substring(13).toInt();
        if (v >= RPM_COUNT_MS_MIN && v <= RPM_COUNT_MS_MAX) {
            rpmCountWindowMs = (uint32_t)v;
            countWindowMs = 0;          // restart the window on the new length
        } else {
            Serial.printf("ERR,RPM_COUNT_MS must be %d-%d\n",
                          RPM_COUNT_MS_MIN, RPM_COUNT_MS_MAX);
        }
        return;
    }
    // RPM_SOURCE,<0 gap|1 counted|2 revolution> - which one drives the loop
    if (s.startsWith("RPM_SOURCE,")) {
        int n = s.substring(11).toInt();
        if (n >= 0 && n <= 2) rpmSource = (uint8_t)n;
        else Serial.println("ERR,RPM_SOURCE must be 0, 1 or 2");
        return;
    }
    if (s.startsWith("RPM_MEDIAN,")) {
        int n = s.substring(11).toInt();
        if (n == 1 || n == 3 || n == 5 || n == 7) {
            rpmMedianN = (uint8_t)n;
            intervalHistN = 0;
        } else {
            Serial.println("ERR,RPM_MEDIAN must be 1, 3, 5 or 7");
        }
        return;
    }
    if (s.startsWith("RPM_RATIO,")) {
        float v = s.substring(10).toFloat();
        if (v == 0.0f || v > 1.0f) rpmRatioGate = v;
        else Serial.println("ERR,RPM_RATIO must be 0 (off) or greater than 1");
        return;
    }
    if (s.startsWith("RPM_SLEW,")) {
        float v = s.substring(9).toFloat();
        if (v >= 0.0f) rpmMaxSlew = v;
        else Serial.println("ERR,RPM_SLEW must be >= 0 (0 = off)");
        return;
    }
    // Reset the noise counter, so a change can be judged on a fresh count.
    if (s == "TACH_RESET") {
        tachGlitches = 0;
        rpmExtrapolated = 0;
        return;
    }

    // ---- Pulses folded into the RPM average ----
    if (s.startsWith("RPM_AVG,")) {
        int n = s.substring(8).toInt();
        if (n >= 1 && n <= RPM_AVG_MAX) {
            rpmAvgSize  = (uint16_t)n;
            rpmBufCount = 0;          // old samples belong to the previous window
            rpmBufIdx   = 0;
        } else {
            Serial.printf("ERR,RPM_AVG must be 1-%d\n", RPM_AVG_MAX);
        }
        return;
    }

    // ---- Brake travel range: BRAKE_RANGE,<min>,<max> ----
    if (s.startsWith("BRAKE_RANGE,")) {
        int i1 = s.indexOf(',', 12);
        if (i1 > 0) {
            long lo = s.substring(12, i1).toInt();
            long hi = s.substring(i1 + 1).toInt();
            if (hi > lo && lo >= 0) {
                brakeMinSteps = lo;
                brakeMaxSteps = hi;
            } else {
                Serial.println("ERR,BRAKE_RANGE needs 0 <= min < max");
            }
        } else {
            Serial.println("ERR,BRAKE_RANGE needs min,max");
        }
        return;
    }

    // ---- Cam geometry ----
    if (s.startsWith("CAM_MODEL,")) {
        int m = s.substring(10).toInt();
        if (m < 0 || m > 2) {
            Serial.println("ERR,CAM_MODEL must be 0 (linear) 1 (eccentric) 2 (table)");
            return;
        }
        if (m == 1 && camStepsPerDeg <= 0.0f)
            Serial.println("ERR,CAM_MODEL 1 needs CAM_SPD first — staying linear");
        if (m == 2 && camNPts < 2)
            Serial.println("ERR,CAM_MODEL 2 needs at least 2 points — staying linear");
        camModel = (uint8_t)m;
        return;
    }
    if (s.startsWith("CAM_SPD,")) {
        float v = s.substring(8).toFloat();
        if (v > 0.0f) camStepsPerDeg = v;
        else Serial.println("ERR,CAM_SPD must be > 0");
        return;
    }
    if (s.startsWith("CAM_LIN,")) {
        camLinearize = (s.substring(8).toInt() != 0);
        return;
    }
    if (s.startsWith("CAM_NPTS,")) {
        int n = s.substring(9).toInt();
        if (n >= 0 && n <= CAM_MAX_PTS) camNPts = (uint8_t)n;
        else Serial.printf("ERR,CAM_NPTS must be 0-%d\n", CAM_MAX_PTS);
        return;
    }
    // CAM_PT,<index>,<stepPct>,<brakePct>
    if (s.startsWith("CAM_PT,")) {
        int i1 = s.indexOf(',', 7);
        int i2 = s.indexOf(',', i1 + 1);
        if (i1 < 0 || i2 < 0) {
            Serial.println("ERR,CAM_PT needs index,stepPct,brakePct");
            return;
        }
        int idx = s.substring(7, i1).toInt();
        if (idx < 0 || idx >= CAM_MAX_PTS) {
            Serial.printf("ERR,CAM_PT index must be 0-%d\n", CAM_MAX_PTS - 1);
            return;
        }
        float x = constrain(s.substring(i1 + 1, i2).toFloat(), 0.0f, 100.0f);
        float y = constrain(s.substring(i2 + 1).toFloat(), 0.0f, 100.0f);
        // Step % must increase down the table, or interpolation has no meaning.
        if (idx > 0 && x <= camStepPct[idx - 1]) {
            Serial.println("ERR,CAM_PT step % must increase down the table");
            return;
        }
        camStepPct[idx]  = x;
        camBrakePct[idx] = y;
        if (idx + 1 > camNPts) camNPts = (uint8_t)(idx + 1);
        return;
    }

    // ---- End-of-run behaviour ----
    if (s.startsWith("RAMPDOWN_MODE,")) {
        int m = s.substring(14).toInt();
        if (m < 0 || m > 2) { Serial.println("ERR,RAMPDOWN_MODE must be 0-2"); return; }
        rampDownMode = (uint8_t)m;
        return;
    }
    if (s.startsWith("RAMPDOWN_RATE,")) {
        float v = s.substring(14).toFloat();
        if (v > 0.0f) rampDownRate = v;
        else Serial.println("ERR,RAMPDOWN_RATE must be > 0 RPM/s");
        return;
    }
    if (s.startsWith("RAMPDOWN_BRAKE,")) {
        float v = s.substring(15).toFloat();
        if (v > 0.0f) rampBrakeRate = v;
        else Serial.println("ERR,RAMPDOWN_BRAKE must be > 0 %/s");
        return;
    }
    if (s.startsWith("STOP_RATE,")) {
        float v = s.substring(10).toFloat();
        if (v > 0.0f) stopRampRate = v;
        else Serial.println("ERR,STOP_RATE must be > 0 %/s");
        return;
    }
    if (s.startsWith("CUTOFF_RPM,")) {
        float v = s.substring(11).toFloat();
        if (v >= 0.0f) cutoffRPM = v;
        else Serial.println("ERR,CUTOFF_RPM must be >= 0");
        return;
    }
    if (s.startsWith("THROTTLE_OFF,")) {
        float v = s.substring(13).toFloat();
        if (v > 0.0f && v < 100.0f) throttleOffPct = v;
        else Serial.println("ERR,THROTTLE_OFF must be between 0 and 100 %");
        return;
    }
    // Force the ramp-down by hand, for bench testing without an engine.
    if (s == "RAMPDOWN") {
        if (currentState == STATE_SWEEP || currentState == STATE_SWEEP_DONE ||
            currentState == STATE_HOLD_RPM) {
            enterRampDown();
        } else {
            Serial.println("ERR,RAMPDOWN only from HOLD_RPM, SWEEP or SWEEP_DONE");
        }
        return;
    }

    // ---- Brake preload applied at START (% of range) ----
    if (s.startsWith("PRELOAD,")) {
        brakePreloadPct = constrain(s.substring(8).toFloat(), 0.0f, 100.0f);
        return;
    }

    // ---- Invert stepper direction (planetary gearbox may reverse it) ----
    if (s.startsWith("INVERT,")) {
        dirInverted = (s.substring(7).toInt() != 0);
        stepper.setPinsInverted(dirInverted, true, true);
        return;
    }

    // ---- Sweep-phase PID gains (hold gains stay on the PID, command) ----
    if (s.startsWith("PID_SWEEP,")) {
        int i1 = s.indexOf(',', 10);
        int i2 = s.indexOf(',', i1 + 1);
        if (i1 > 0 && i2 > 0) {
            sweepKp = s.substring(10, i1).toFloat();
            sweepKi = s.substring(i1 + 1, i2).toFloat();
            sweepKd = s.substring(i2 + 1).toFloat();
        } else {
            Serial.println("ERR,PID_SWEEP needs kp,ki,kd");
        }
        return;
    }

    // ---- Stepper tuning ----
    if (s.startsWith("STEPPER_SPEED,")) {
        float v = s.substring(14).toFloat();
        if (v > 0.0f) { stepperMaxSpeed = v; stepper.setMaxSpeed(v); }
        else Serial.println("ERR,STEPPER_SPEED must be > 0");
        return;
    }
    if (s.startsWith("STEPPER_ACCEL,")) {
        float v = s.substring(14).toFloat();
        if (v > 0.0f) { stepperAccel = v; stepper.setAcceleration(v); }
        else Serial.println("ERR,STEPPER_ACCEL must be > 0");
        return;
    }

    // ---- Query configuration ----
    // ENCODER,<0|1>,<counts per rev>,<invert>  - TBD, stored but inert
    if (s.startsWith("ENCODER,")) {
        int i1 = s.indexOf(',', 8);
        int i2 = (i1 > 0) ? s.indexOf(',', i1 + 1) : -1;
        bool want = ((i1 > 0 ? s.substring(8, i1) : s.substring(8)).toInt() != 0);
        if (i1 > 0) {
            long c = (i2 > 0 ? s.substring(i1 + 1, i2)
                             : s.substring(i1 + 1)).toInt();
            if (c >= 1 && c <= 1000000) encoderCPR = (uint32_t)c;
            else Serial.println("ERR,ENCODER counts per rev must be 1-1000000");
        }
        if (i2 > 0) encoderInvert = (s.substring(i2 + 1).toInt() != 0);
        encoderEnabled = want;
        // Say so plainly rather than silently accepting: asking for an
        // encoder that is not wired should not look like it worked.
        if (encoderEnabled && ENCODER_PIN_A < 0) {
            Serial.println("ERR,ENCODER not fitted - setting stored, no pins assigned (TBD)");
        }
        return;
    }
    if (s == "VERSION") {
        Serial.printf("CFG,FW_VERSION,%s,%s %s\n", FW_VERSION, __DATE__, __TIME__);
        return;
    }
    if (s == "STATUS") {
        Serial.printf("CFG,FW_VERSION,%s,%s %s\n", FW_VERSION, __DATE__, __TIME__);
        Serial.printf("CFG,PID,%.4f,%.4f,%.4f\n", pidKp, pidKi, pidKd);
        Serial.printf("CFG,PID_SWEEP,%.4f,%.4f,%.4f\n", sweepKp, sweepKi, sweepKd);
        Serial.printf("CFG,SCALE,%.6f\n", loadCellScale);
        Serial.printf("CFG,ARM,%.4f\n", leverArm);
        Serial.printf("CFG,MECH,%.4f\n", mechRatio);
        Serial.printf("CFG,HOLD,%.1f\n", holdRPM);
        Serial.printf("CFG,SWEEP,%.1f,%.1f,%.1f\n",
                       sweepStartRPM, sweepEndRPM, sweepRate);
        Serial.printf("CFG,TEETH,%u\n", (unsigned)pulsesPerRev);
        Serial.printf("CFG,RATIO,%.4f\n", driveRatio);
        Serial.printf("CFG,RPM_BAND,%.1f,%.1f\n", rpmBandMin, rpmBandMax);
        Serial.printf("CFG,ENCODER,%d,%lu,%d,%d\n", encoderEnabled ? 1 : 0,
                       (unsigned long)encoderCPR, encoderInvert ? 1 : 0,
                       encoderOK ? 1 : 0);
        Serial.printf("CFG,RPM_SOURCE,%u\n", (unsigned)rpmSource);
        Serial.printf("CFG,RPM_COUNT_MS,%lu\n",
                       (unsigned long)rpmCountWindowMs);
        Serial.printf("CFG,RPM_MEDIAN,%u\n", (unsigned)rpmMedianN);
        Serial.printf("CFG,RPM_EXTRAP,%d,%u,%u\n", rpmExtrapOn ? 1 : 0,
                       (unsigned)rpmExtrapN, (unsigned)rpmExtrapMax);
        Serial.printf("CFG,RPM_ESTIMATED,%lu\n", (unsigned long)rpmExtrapolated);
        Serial.printf("CFG,RPM_RATIO,%.2f\n", rpmRatioGate);
        Serial.printf("CFG,RPM_SLEW,%.1f\n", rpmMaxSlew);
        Serial.printf("CFG,TACH_GLITCHES,%lu\n", (unsigned long)tachGlitches);
        Serial.printf("CFG,RPM_AVG,%u\n", (unsigned)rpmAvgSize);
        Serial.printf("CFG,STEPPER,%.1f,%.1f\n", stepperMaxSpeed, stepperAccel);
        Serial.printf("CFG,CAM,%u,%.4f,%d,%u\n", (unsigned)camModel, camStepsPerDeg,
                       camLinearize ? 1 : 0, (unsigned)camNPts);
        for (uint8_t i = 0; i < camNPts; i++)
            Serial.printf("CFG,CAM_PT,%u,%.2f,%.2f\n", (unsigned)i,
                           camStepPct[i], camBrakePct[i]);
        Serial.printf("CFG,BRAKE_RANGE,%ld,%ld\n", brakeMinSteps, brakeMaxSteps);
        Serial.printf("CFG,PRELOAD,%.1f\n", brakePreloadPct);
        Serial.printf("CFG,RAMPDOWN,%u,%.1f,%.1f\n",
                       (unsigned)rampDownMode, rampDownRate, rampBrakeRate);
        Serial.printf("CFG,CUTOFF_RPM,%.1f\n", cutoffRPM);
        Serial.printf("CFG,STOP_RATE,%.1f\n", stopRampRate);
        Serial.printf("CFG,THROTTLE_OFF,%.1f\n", throttleOffPct);
        Serial.printf("CFG,INVERT,%d\n", dirInverted ? 1 : 0);
        Serial.printf("CFG,PRESS,%.4f,%.6f,%d\n",
                       pressureOffset_mV, pressureScale, pressureCalValid ? 1 : 0);
        Serial.printf("CFG,PRESS_LIMIT,%.1f\n", pressureLimitPSI);
        Serial.printf("CFG,PRESS_ADC,%.4f,%u,%d\n",
                       pressDivider, (unsigned)pressPgaIdx, pressAdcOk ? 1 : 0);
        Serial.printf("CFG,SIM,%d\n", simMode ? 1 : 0);
        sendReadyStatus();
        return;
    }

    Serial.print("ERR,Unknown: ");
    Serial.println(s);
}

// =============================================
// Serial Input (line-based, non-blocking)
// =============================================
static void readSerial() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialBufLen > 0) {
                serialBuf[serialBufLen] = '\0';
                processCommand(serialBuf);
                serialBufLen = 0;
            }
        } else if (serialBufLen < sizeof(serialBuf) - 1) {
            serialBuf[serialBufLen++] = c;
        } else {
            serialBufLen = 0;
        }
    }
}

// =============================================
// Data Report — sent to Raspberry Pi at 20 Hz
// Format: DATA,millis,rpm,torque,loadRaw,adc0,adc1,brakePos,targetRPM,state,pidP,pidI,pidD_out
// =============================================
static void sendDataReport() {
    float pTerm = actKp * pidLastErr;
    float iTerm = actKi * pidIntegral;

    // Brake applied, as a percentage — run through the cam mapping so this is
    // actual brake application rather than raw stepper travel.  With the linear
    // model the two are the same, which is the old behaviour.
    long  pos  = stepper.currentPosition();
    float span = (float)(brakeMaxSteps - brakeMinSteps);
    float stepFrac = (span > 0.0f)
        ? constrain(((float)pos - (float)brakeMinSteps) / span, 0.0f, 1.0f) : 0.0f;
    float brakePct = camForward(stepFrac) * 100.0f;

    uint8_t faults = (faultTach ? 0x01 : 0) | (faultPressure ? 0x02 : 0);

    // Fields 1-12 keep their original meaning and order; 13-16 are appended so
    // an older GUI that reads the first 13 fields still parses this cleanly.
    // Field 5 is the load-cell ADC reading. Field 6 now carries the pressure
    // sensor's own millivolts from the dedicated ADS1115, because that is the
    // value the calibration workflow captures; the DFRobot spare moved to 16.
    Serial.printf("DATA,%lu,%.1f,%.2f,%.1f,%.1f,%.1f,%ld,%.1f,%s,%.2f,%.2f,%.2f,%.1f,%.1f,%u,%.1f,%lu,%lu,%ld,%d,%.1f,%.1f\n",
        millis(),
        currentRPM,
        currentTorque,
        loadCellRaw_mV,
        loadCellRaw_mV,
        pressRaw_mV,
        pos,
        targetRPM,
        STATE_NAMES[currentState],
        pTerm,
        iTerm,
        pidOutput,
        brakePct,
        pressurePSI,
        (unsigned)faults,
        spareAux_mV,
        (unsigned long)tachGlitches,
        (unsigned long)rpmExtrapolated,
        (long)encoderCount,          // TBD: 0 until an encoder is fitted
        encoderOK ? 1 : 0,
        rpmCounted,
        rpmRev);
}

// =============================================
// SIM Virtual Engine — integrate one step (bench demo, no real engine)
// Drives currentRPM/currentTorque from a lumped closed-loop model so the real
// state machine + PID control it and the cam brake stepper moves for real:
//     dRPM/dt = A*throttle - B*brakeSteps - C*RPM
// Throttle emulates the operator per state (partial to hold, WOT to sweep, then
// auto throttle-down after the run). Torque is injected from the recorded curve.
// =============================================
static void updateSimEngine(float dt) {
    unsigned long now = millis();
    if (currentState != simPrevState) {
        simPrevState = currentState;
        simStateEnterMs = now;
    }

    // Operator/engine throttle target for the current phase.
    float uTarget;
    switch (currentState) {
        case STATE_HOLD_RPM:                       // hold at holdRPM (partial throttle)
            uTarget = holdRPM / SIM_WOT_RPM;
            break;
        case STATE_SWEEP:                          // WOT; brake controls the sweep
            uTarget = 1.0f;
            break;
        case STATE_SWEEP_DONE:                     // WOT, then auto throttle-down
            uTarget = (now - simStateEnterMs > SIM_DONE_HOLD_MS) ? 0.0f : 1.0f;
            break;
        default:                                   // IDLE / HOMING / MANUAL
            uTarget = 0.0f;
            break;
    }
    uTarget = constrain(uTarget, 0.0f, 1.0f);

    float a = dt / SIM_THROTTLE_TAU;               // first-order throttle lag
    if (a > 1.0f) a = 1.0f;
    simThrottle += (uTarget - simThrottle) * a;

    // Brake plant input = last commanded brake position (steps), clamped.
    float b = constrain(brakeCommandSteps, 0.0f, (float)brakeMaxSteps);

    // Integrate the lumped engine model (forward Euler).
    float dwdt = SIM_ENGINE_A * simThrottle - SIM_ENGINE_B * b - SIM_ENGINE_C * simRPM;
    simRPM += dwdt * dt;
    if (simRPM < 0.0f) simRPM = 0.0f;

    // Publish to the same channels the real sensors would drive.
    currentRPM     = simRPM;
    float tq       = simThrottle * simTorqueForRpm(simRPM);   // injected recorded torque
    currentTorque  = tq;
    loadCellRaw_mV = tq;    // stand-in so the GUI's loadRaw shows the injected signal
}

// =============================================
// End-of-run helpers
// =============================================
static void trackPeakTorque() {
    if (currentTorque > peakTorqueRun) peakTorqueRun = currentTorque;
}

// Has the operator lifted?  Torque collapses the moment the throttle shuts,
// well before the RPM has visibly moved, so it is the earliest honest signal.
// With an uncalibrated or railed load cell there is no usable torque, so fall
// back to the engine dropping clearly below the speed it was being held at.
static bool throttleClosed() {
    if (peakTorqueRun > 1e-3f)
        return currentTorque < peakTorqueRun * (throttleOffPct * 0.01f);
    return targetRPM > 0.0f && currentRPM < targetRPM * 0.90f;
}

static void enterRampDown() {
    currentState  = STATE_RAMP_DOWN;
    rampStartMs   = millis();
    rampStartRPM  = currentRPM;
    // Hand over bumplessly.  Carrying the sweep's accumulated windup into the
    // ramp is exactly what keeps the brake clamped while the engine is dying,
    // and zeroing it instead would dump the brake just as abruptly.
    applyGainsForState();
    float error = currentRPM - rampStartRPM;
    if (actKi > 0.001f)
        pidIntegral = constrain((brakeCommandSteps - actKp * error) / actKi,
                                0.0f, integralCeiling());
    pidLastErr = error;
    Serial.println("ACK,RAMP_DOWN");
}

static void updateRampDown(unsigned long now, float dt) {
    float span    = (float)(brakeMaxSteps - brakeMinSteps);
    float elapsed = (float)(now - rampStartMs) / 1000.0f;

    if (stopRamping) {
        // Safety stop overrides whatever mode is configured: always a straight
        // linear release, no loop involved, so nothing can command more brake.
        float pos = brakeCommandSteps - span * (stopRampRate * 0.01f) * dt;
        if (pos < (float)brakeMinSteps) pos = (float)brakeMinSteps;
        stepper.moveTo((long)pos);
        brakeCommandSteps = pos;
        if (pos <= (float)brakeMinSteps + 0.5f) {
            Serial.println("ACK,STOP_COMPLETE");
            resetToIdle();
        }
        return;
    }

    if (rampDownMode == 1) {
        // Descending target.  If the engine falls faster than the ramp the loop
        // sees a negative error and backs the brake off by itself, so this can
        // hold the descent back but never hold the engine up.
        float target = rampStartRPM - rampDownRate * elapsed;
        if (target < cutoffRPM) target = cutoffRPM;
        targetRPM = target;
        controlBrake(target, dt);
        return;
    }

    // Modes 0 and 2 are open loop: no PID, so nothing can command more brake.
    float pos;
    if (rampDownMode == 2) {
        pos = brakeCommandSteps - span * (rampBrakeRate * 0.01f) * dt;
    } else {
        pos = (float)brakeMinSteps;
    }
    if (pos < (float)brakeMinSteps) pos = (float)brakeMinSteps;
    stepper.moveTo((long)pos);
    brakeCommandSteps = pos;
    if (pos <= (float)brakeMinSteps + 0.5f) {
        Serial.println("ACK,RAMP_DOWN_COMPLETE");
        resetToIdle();
    }
}

// =============================================
// State Machine Update
// =============================================
static void updateStateMachine() {
    unsigned long now = millis();

    // Hard floor: once a run has actually engaged, an engine below the cutoff is
    // finished, and anything still on the brake can only stall it.  Deliberately
    // ahead of every other rule.  Gated on holdEngaged so the brake is not let
    // go the instant START is pressed with the engine still idling.
    bool inRun = (currentState == STATE_SWEEP || currentState == STATE_SWEEP_DONE ||
                  currentState == STATE_RAMP_DOWN ||
                  (currentState == STATE_HOLD_RPM && holdEngaged));
    if (inRun && !faultTach && !stopRamping && currentRPM > 0.0f && currentRPM < cutoffRPM) {
        Serial.println("ACK,CUTOFF_RELEASE");
        resetToIdle();
        return;
    }

    switch (currentState) {

        case STATE_IDLE:
            // If stepper reached position 0, disable
            if (stepper.distanceToGo() == 0 && stepper.currentPosition() == 0) {
                enableStepper(false);
            }
            break;

        case STATE_HOMING: {
            static unsigned long lastHomingDbg = 0;
            // Check limit switch — LOW = switch tripped
            if (digitalRead(PIN_LIMIT_SWITCH) == LOW) {
                stepper.stop();
                stepper.setCurrentPosition(0);
                stepper.setMaxSpeed(STEPPER_MAX_SPEED);
                isHomed = true;
                currentState = STATE_IDLE;
                enableStepper(false);
                Serial.println("ACK,HOME_COMPLETE");
            }
            // Fallback: if stepper ran out of travel without hitting switch
            else if (stepper.distanceToGo() == 0) {
                stepper.setMaxSpeed(STEPPER_MAX_SPEED);
                currentState = STATE_IDLE;
                enableStepper(false);
                Serial.println("ERR,HOME_FAILED_NO_SWITCH");
            }
            // Debug: print homing progress every 500ms
            if (now - lastHomingDbg >= 500) {
                lastHomingDbg = now;
                Serial.printf("[DBG] HOMING: sw=%d pos=%ld dist=%ld spd=%.1f\n",
                    digitalRead(PIN_LIMIT_SWITCH), stepper.currentPosition(),
                    stepper.distanceToGo(), stepper.speed());
            }
        }
            break;

        case STATE_MANUAL:
            break;

        case STATE_HOLD_RPM:
            if ((now - lastPIDUpdate) >= PID_INTERVAL_MS) {
                float dt = (float)(now - lastPIDUpdate) / 1000.0f;
                lastPIDUpdate = now;

                if (!holdEngaged) {
                    // Below the trigger speed the brake just waits at preload —
                    // it must not fight the operator on the way up from idle.
                    if (currentRPM >= holdRPM) {
                        holdEngaged = true;
                        applyGainsForState();
                        // Seed the integrator at the preload position so the
                        // loop takes over from where the brake already is.
                        if (actKi > 0.001f) {
                            pidIntegral = constrain(brakeCommandSteps / actKi,
                                                    0.0f, integralCeiling());
                        }
                        pidLastErr = currentRPM - holdRPM;
                        Serial.println("ACK,HOLD_ENGAGED");
                    } else {
                        stepper.moveTo((long)brakeCommandSteps);
                        break;
                    }
                }
                controlBrake(holdRPM, dt);
            }
            break;

        case STATE_SWEEP:
            if ((now - lastPIDUpdate) >= PID_INTERVAL_MS) {
                float dt = (float)(now - lastPIDUpdate) / 1000.0f;
                lastPIDUpdate = now;

                // Advance target RPM along the sweep
                float elapsed = (float)(now - sweepStartTime) / 1000.0f;
                if (sweepEndRPM > sweepStartRPM) {
                    targetRPM = sweepStartRPM + sweepRate * elapsed;
                    if (targetRPM >= sweepEndRPM) {
                        targetRPM = sweepEndRPM;
                        currentState = STATE_SWEEP_DONE;
                        Serial.println("ACK,SWEEP_COMPLETE");
                    }
                } else {
                    targetRPM = sweepStartRPM - sweepRate * elapsed;
                    if (targetRPM <= sweepEndRPM) {
                        targetRPM = sweepEndRPM;
                        currentState = STATE_SWEEP_DONE;
                        Serial.println("ACK,SWEEP_COMPLETE");
                    }
                }

                controlBrake(targetRPM, dt);
                trackPeakTorque();
            }
            break;

        case STATE_SWEEP_DONE:
            // Continue holding at end RPM via PID
            if ((now - lastPIDUpdate) >= PID_INTERVAL_MS) {
                float dt = (float)(now - lastPIDUpdate) / 1000.0f;
                lastPIDUpdate = now;
                controlBrake(targetRPM, dt);
                trackPeakTorque();

                // The operator lifting is the end of the test, not a fault: hand
                // over to the ramp-down phase rather than dropping the brake.
                // Suppressed while the tach is faulted, since a dead pickup also
                // reads as falling RPM and must not look like a throttle-down.
                if (!faultTach && throttleClosed()) {
                    enterRampDown();
                }
            }
            break;

        case STATE_RAMP_DOWN:
            if ((now - lastPIDUpdate) >= PID_INTERVAL_MS) {
                float dt = (float)(now - lastPIDUpdate) / 1000.0f;
                lastPIDUpdate = now;
                updateRampDown(now, dt);
            }
            break;
    }
}

// =============================================
// setup()
// =============================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("DIY_DYNO_ESP32S3,V1.0");

    // I2C — enable internal pull-ups explicitly
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, INPUT_PULLUP);
    delay(10);
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(100000);
    Wire.setTimeOut(10);  // 10ms timeout per transaction

    // Debug: read raw pin states before scan
    Serial.printf("[I2C] SDA=%d SCL=%d  SDA_level=%d SCL_level=%d\n",
                  PIN_I2C_SDA, PIN_I2C_SCL,
                  digitalRead(PIN_I2C_SDA), digitalRead(PIN_I2C_SCL));
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.printf("[I2C] Found device at 0x%02X\n", addr);
        }
    }
    Serial.println("[I2C] Scan done");

    // Stepper Motor — common-anode wiring: invert step, dir, enable signals
    pinMode(PIN_STEPPER_ENABLE, OUTPUT);
    enableStepper(false);
    stepper.setPinsInverted(dirInverted, true, true);  // dir (runtime), step, enable
    stepper.setMinPulseWidth(5);                  // DM860T needs >=2.5μs pulse
    stepper.setMaxSpeed(STEPPER_MAX_SPEED);
    stepper.setAcceleration(STEPPER_ACCEL);

    // Limit Switch (for homing)
    pinMode(PIN_LIMIT_SWITCH, INPUT_PULLUP);

    // Proximity Sensor (RPM)
    updateMinPulseInterval();
    pinMode(PIN_PROXIMITY, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_PROXIMITY),
                    onProximityPulse, FALLING);

    // HX711 not used — load cell amp connects via ADC channel 0

    // ADS1115 (DFRobot 0-10V ADC) — must call begin() AFTER Wire.begin()
    adcOk = adc.begin();
    if (adcOk) {
        Serial.println("[INIT] DFRobot ADC @ 0x48: OK");
        // Auto-tare load cell from ADC reading
        loadCellOffset_mV = adc.getValue(1);  // DFRobot returns mV directly
        isTared = true;
        Serial.println("[INIT] Load cell auto-tared via ADC ch1");
    } else {
        Serial.println("[INIT] DFRobot ADC @ 0x48: FAIL");
    }

    // ---- Second I2C bus: pressure ADS1115 ----
    Wire1.begin(PIN_I2C2_SDA, PIN_I2C2_SCL);
    Wire1.setClock(100000);
    Wire1.setTimeOut(10);
    Serial.printf("[I2C2] SDA=%d SCL=%d — scanning\n", PIN_I2C2_SDA, PIN_I2C2_SCL);
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire1.beginTransmission(addr);
        if (Wire1.endTransmission() == 0) Serial.printf("[I2C2] Found device at 0x%02X\n", addr);
    }
    applyPressureGain();
    pressAdcOk = pressAdc.begin(PRESS_ADC_ADDR, &Wire1);
    if (pressAdcOk) {
        applyPressureGain();          // re-assert after begin()
        Serial.printf("[INIT] Pressure ADS1115 @ 0x%02X on I2C2: OK\n", PRESS_ADC_ADDR);
    } else {
        Serial.printf("[INIT] Pressure ADS1115 @ 0x%02X on I2C2: NOT FOUND — "
                      "brake pressure unavailable, over-pressure interlock disarmed\n",
                      PRESS_ADC_ADDR);
    }

    // Reassert I2C pins
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(100000);

    // Launch ADC task on Core 0 (stepper runs on Core 1)
    if (adcOk) {
        xTaskCreatePinnedToCore(adcTask, "ADC", 4096, NULL, 1, NULL, 0);
        Serial.println("[INIT] ADC task started on Core 0");
    }

    // SIM mode: no real engine/tach/ADC needed — mark ready so a run can start.
    if (simMode) {
        isHomed = true;
        isTared = true;
        Serial.println("[INIT] *** SIM MODE — VIRTUAL ENGINE. Readings are synthetic. Send SIM,0 for real hardware. ***");
    } else {
        Serial.println("[INIT] LIVE mode — real sensors. Home and tare required before a run.");
    }

    sendReadyStatus();
    Serial.println("[INIT] DIY Dyno ready");
}

// =============================================
// loop()
// =============================================
void loop() {
    unsigned long now = millis();

    stepper.run();
    // Hand the speed ceiling back the moment a timed sweep lands.
    if (sweepSpeedActive && stepper.distanceToGo() == 0) clearSweepSpeed();
    readSerial();

    if (simMode) {
        // Virtual engine: integrate every loop with real elapsed dt (smooth),
        // using the brake command from the previous control tick.
        uint32_t nowUs = micros();
        float dt = (lastSimUs == 0) ? 0.0f : (float)(nowUs - lastSimUs) * 1e-6f;
        lastSimUs = nowUs;
        if (dt > 0.06f) dt = 0.06f;              // clamp after long pauses
        updateSimEngine(dt);
    } else {
        updateRPM();
    updateRPMCounted();
    updateEncoder();
        lastSimUs = 0;                           // reset dt baseline for next SIM entry
    }

    // ADC reads run on Core 0 task (real mode) — no blocking here

    updateStateMachine();

    if ((now - lastDataReport) >= DATA_REPORT_MS) {
        lastDataReport = now;
        sendDataReport();
    }
}

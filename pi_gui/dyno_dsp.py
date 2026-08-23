#!/usr/bin/env python3
"""
dyno_dsp.py — DIY Engine Dyno shared signal-processing + data-loading helpers.

Used by the Raspberry Pi GUI (dyno_gui.py) for:
  • Loading recorded dyno logs / CSVs for "Dummy Data" replay.
  • Filtering torque/RPM curves (live and final) and resampling onto an RPM grid.

Design goals:
  • Pure NumPy only (no SciPy / pandas) so it runs on a stock Raspberry Pi where
    only matplotlib (→ numpy) and pyserial are installed.  The richer offline
    matching tool (Python Scripts/Dyno_Filter_Matcher_*.py) mirrors these filter
    definitions but is free to use SciPy.

Filter definitions are matched bit-for-bit to the team's reference smoothing
workbook (Dyno run 28 May 2025 smooth.xlsx):
  • EMA  : y[0] = a*x[0]  (i.e. previous value seeded to 0); y[k] = a*x[k] + (1-a)*y[k-1]
  • RA   : centered moving average, window=20 → mean(x[k-9 : k+11]) with edge clipping
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from datetime import datetime

import numpy as np

# ── Unit / power constants ───────────────────────────────────────────────────
# HP = Torque[Nm] * RPM / 7120.9   (metric chain: kW = Nm*RPM/9549.296, 1 hp = 0.745699872 kW)
# HP = Torque[lb-ft] * RPM / 5252  (imperial)
HP_NM_CONST   = 7120.9
HP_LBFT_CONST = 5252.0
KW_CONST      = 9549.296
NM_PER_LBFT   = 1.355818
LBFT_PER_NM   = 1.0 / NM_PER_LBFT          # 0.7375621
HP_PER_KW     = 1.0 / 0.745699872          # 1.341022

# Timestamp formats seen across the DIY dyno's own logs, tried in order.
_TS_FORMATS = (
    "%Y-%m-%dT%H:%M:%S.%f",
    "%Y-%m-%dT%H:%M:%S",
    "%Y-%m-%d %H:%M:%S.%f",
    "%Y-%m-%d %H:%M:%S",
)


# ═════════════════════════════════════════════════════════════════════════════
# Power / unit helpers
# ═════════════════════════════════════════════════════════════════════════════
def hp_from_torque_nm(torque_nm, rpm):
    """Horsepower from torque in N·m and engine RPM (array-safe)."""
    t = np.asarray(torque_nm, dtype=float)
    r = np.asarray(rpm, dtype=float)
    return t * r / HP_NM_CONST


def hp_from_torque_lbft(torque_lbft, rpm):
    """Horsepower from torque in lb-ft and engine RPM (array-safe)."""
    t = np.asarray(torque_lbft, dtype=float)
    r = np.asarray(rpm, dtype=float)
    return t * r / HP_LBFT_CONST


def nm_to_lbft(nm):
    return np.asarray(nm, dtype=float) * LBFT_PER_NM


def lbft_to_nm(lbft):
    return np.asarray(lbft, dtype=float) * NM_PER_LBFT


# ═════════════════════════════════════════════════════════════════════════════
# Filters — every function takes a 1-D array and returns a same-length array
# ═════════════════════════════════════════════════════════════════════════════
def _fill_nan(y):
    """Linearly interpolate interior NaN over finite samples; edge-hold the ends.

    Real logs contain occasional bad/blank cells (→ NaN). Without this, a single
    NaN poisons most/all of a filter's output (cumulative sums, IIR recursions,
    polyfit all propagate NaN). All-NaN input → zeros. NaN-free input is returned
    unchanged, so the reference-exact filters are not affected for clean data.
    """
    y = np.asarray(y, dtype=float)
    if y.size == 0:
        return y.copy()
    finite = np.isfinite(y)
    if finite.all():
        return y.copy()
    if not finite.any():
        return np.zeros_like(y)
    idx = np.arange(y.size)
    return np.interp(idx, idx[finite], y[finite])


def ema(x, alpha, init=0.0):
    """Exponential moving average.

    y[k] = alpha*x[k] + (1-alpha)*y[k-1], with y[-1] = init.
    init=0.0 reproduces the reference workbook exactly (seed = 0).
    Pass init=x[0] for steady-state seeding (no startup ramp).
    """
    x = _fill_nan(x)
    if x.size == 0:
        return x.copy()
    alpha = float(np.clip(alpha, 0.0, 1.0))
    y = np.empty_like(x)
    prev = float(init)
    for i in range(x.size):
        prev = alpha * x[i] + (1.0 - alpha) * prev
        y[i] = prev
    return y


def centered_moving_average(x, window):
    """Centered moving average with edge clipping (min_periods=1).

    Matches the reference workbook's RA(window=20): output[k] is the mean of
    x[k-(w//2-1) : k+(w//2+1)] for even w (→ exactly w points centered slightly
    right), or x[k-w//2 : k+w//2+1] for odd w; the slice is clipped to [0, N).
    """
    x = _fill_nan(x)
    n = x.size
    w = int(max(1, round(window)))
    if n == 0 or w <= 1:
        return x.copy()
    half = w // 2
    if w % 2 == 0:          # even window: 9 before, self, 10 after for w=20
        left, right = half - 1, half + 1
    else:                   # odd window: symmetric
        left, right = half, half + 1
    prefix = np.concatenate(([0.0], np.cumsum(x)))   # prefix[i] = sum(x[:i])
    out = np.empty(n)
    for k in range(n):
        lo = max(0, k - left)
        hi = min(n, k + right)
        out[k] = (prefix[hi] - prefix[lo]) / (hi - lo)
    return out


def savitzky_golay(x, window, polyorder):
    """Savitzky–Golay smoothing (NumPy-only, scipy mode='interp' edge handling).

    Interior points use the analytic SG convolution coefficients; the first and
    last (window//2) points are fit with a single boundary polynomial.
    """
    x = _fill_nan(x)
    n = x.size
    w = int(window)
    p = int(polyorder)
    if w % 2 == 0:
        w += 1                      # window must be odd
    if n < w or w <= p or w < 3:
        return x.copy()
    half = w // 2
    # SG coefficients for the smoothed value at the window centre.
    z = np.arange(-half, half + 1, dtype=float)
    A = np.vander(z, p + 1, increasing=True)          # columns: z^0 .. z^p
    # Smoothing coeffs = row of pinv(A) corresponding to the 0th derivative.
    coeffs = np.linalg.pinv(A)[0]                      # length w
    out = np.convolve(x, coeffs[::-1], mode="same")
    # Fix the edges with explicit boundary polynomial fits (mode='interp').
    zi = np.arange(w)
    cl = np.polyfit(zi, x[:w], p)
    out[:half] = np.polyval(cl, zi[:half])
    cr = np.polyfit(zi, x[-w:], p)
    out[-half:] = np.polyval(cr, zi[-half:])
    return out


def lowpass_single_pole(x, dt, tau):
    """First-order (single-pole) IIR low-pass, supports non-uniform dt.

    Mirrors the firmware/V7 "filter time constant" concept.
    alpha_k = dt_k / (tau + dt_k); y[k] = y[k-1] + alpha_k*(x[k]-y[k-1]).

    dt may be a scalar (uniform sampling) or an array of per-sample intervals
    (len == len(x); dt[0] is ignored). tau is the time constant in seconds.
    """
    x = _fill_nan(x)
    n = x.size
    if n == 0:
        return x.copy()
    tau = max(float(tau), 1e-9)
    if np.isscalar(dt):
        dt_arr = np.full(n, float(dt))
    else:
        dt_arr = np.asarray(dt, dtype=float)
    y = np.empty_like(x)
    y[0] = x[0]
    for k in range(1, n):
        d = dt_arr[k] if dt_arr[k] > 0 else (dt_arr[1] if n > 1 else tau)
        a = d / (tau + d)
        y[k] = y[k - 1] + a * (x[k] - y[k - 1])
    return y


def polynomial_smooth(xgrid, ygrid, degree):
    """Least-squares polynomial fit of y vs x, evaluated back on x (SimpleDyno-style).

    Intended for the RPM-domain final curve (x = RPM, y = torque). Non-finite
    samples are interpolated first (np.polyfit silently returns NaN coefficients
    on NaN input rather than raising). Falls back to the raw values if the fit is
    ill-conditioned.
    """
    x = _fill_nan(xgrid)
    y = _fill_nan(ygrid)
    deg = int(max(1, degree))
    if x.size <= deg:
        return y.copy()
    # Normalise x for numerical stability.
    x0, xs = x.mean(), (x.std() or 1.0)
    xn = (x - x0) / xs
    try:
        coeffs = np.polyfit(xn, y, deg)
        return np.polyval(coeffs, xn)
    except (np.linalg.LinAlgError, ValueError):
        return y.copy()


def clamp_spikes(x, max_jump):
    """SimpleDyno-style spike removal: if |x[k]-x[k-1]| > max_jump, hold previous."""
    x = np.asarray(x, dtype=float).copy()
    if max_jump <= 0:
        return x
    for k in range(1, x.size):
        if abs(x[k] - x[k - 1]) > max_jump:
            x[k] = x[k - 1]
    return x


def hampel_despike(x, window=7, n_sigma=3.0):
    """Hampel filter: replace points > n_sigma*MAD from the local median.

    YourDyno-style RPM spike removal (isolated tooth-to-tooth jitter).
    """
    x = np.asarray(x, dtype=float).copy()
    n = x.size
    w = int(max(1, window))
    k = 1.4826  # MAD → std for normal distribution
    half = w // 2
    for i in range(n):
        lo, hi = max(0, i - half), min(n, i + half + 1)
        seg = x[lo:hi]
        med = np.median(seg)
        mad = k * np.median(np.abs(seg - med))
        if mad > 0 and abs(x[i] - med) > n_sigma * mad:
            x[i] = med
    return x


# Names exposed to the GUI filter dropdown.
FILTER_NONE = "None"
FILTER_EMA  = "EMA"
FILTER_MA   = "Centered MA"
FILTER_SG   = "Savitzky-Golay"
FILTER_LP   = "Low-pass (1-pole)"
FILTER_POLY = "Polynomial fit"

# FILTER_YOURDYNO is a full raw→binned pipeline (see yourdyno_binned_filter); it
# is handled specially by callers, not via apply_filter().
FILTER_YOURDYNO = "YourDyno Binned (J1349)"

FILTER_NAMES = [FILTER_YOURDYNO, FILTER_NONE, FILTER_EMA, FILTER_MA,
                FILTER_SG, FILTER_LP, FILTER_POLY]


def apply_filter(name, y, *, x=None, dt=None, alpha=0.2, window=20,
                 polyorder=3, tau=0.15, degree=2):
    """Dispatch helper used by the GUI.

    y is the signal to smooth. For time-domain filters x/dt describe the sample
    spacing; for the polynomial fit x is the RPM grid the curve is plotted against.
    """
    y = np.asarray(y, dtype=float)
    if y.size == 0 or name == FILTER_NONE:
        return y.copy()
    if name == FILTER_EMA:
        return ema(y, alpha, init=float(y[0]))
    if name == FILTER_MA:
        return centered_moving_average(y, window)
    if name == FILTER_SG:
        return savitzky_golay(y, window, polyorder)
    if name == FILTER_LP:
        if dt is None:
            dt = 0.05
        return lowpass_single_pole(y, dt, tau)
    if name == FILTER_POLY:
        if x is None:
            x = np.arange(y.size, dtype=float)
        return polynomial_smooth(x, y, degree)
    return y.copy()


# ═════════════════════════════════════════════════════════════════════════════
# YourDyno "Run analysis tool" filter chain + SAE J607 correction
# Reproduces Run analysis tool V0_1.xlsx ("Filter Tool" sheet). Methodology:
#   Raw → Gauge MA (samples) → 100-RPM bin average (SAE J1349) → Graph MA (bins)
#        → SAE J607 correction → Power.   HP = Torque · RPM / 5252  (lb-ft).
# ═════════════════════════════════════════════════════════════════════════════
def sae_j607_factor(temp_f, humidity_pct=0.0, pressure_inhg=29.92):
    """SAE J607 power-correction factor, matching Run analysis tool V0_1.xlsx.

        CF = 1.18 · (29.92 / P) · sqrt((T + 460) / 537) − 0.18

    T in °F, P in inHg. The workbook also computes the humidity saturation/actual
    vapor pressure (for display) but — like this function — does NOT subtract it
    from P, so pass the dry/absolute barometric pressure. Returns 1.0-ish near
    standard conditions (77 °F, 29.92 inHg).
    """
    pressure_inhg = max(float(pressure_inhg), 1e-6)
    return 1.18 * (29.92 / pressure_inhg) * math.sqrt((float(temp_f) + 460.0) / 537.0) - 0.18


def _centered_mean_hw(y, half_width, ignore_nan=False):
    """Centered moving average by HALF-WIDTH (±hw points), edge-clamped."""
    y = np.asarray(y, dtype=float)
    n = y.size
    hw = int(max(0, half_width))
    if n == 0 or hw == 0:
        return y.copy()
    out = np.empty(n)
    for k in range(n):
        lo, hi = max(0, k - hw), min(n, k + hw + 1)
        seg = y[lo:hi]
        if ignore_nan:
            mask = np.isfinite(seg)
            out[k] = seg[mask].mean() if mask.any() else np.nan
        else:
            out[k] = seg.mean()
    return out


def yourdyno_binned_filter(rpm, torque, *, gauge=3, graph=3, spike=0,
                           bin_width=100.0, sae_factor=1.0,
                           rpm_min=None, rpm_max=None):
    """Reproduce the Run-analysis-tool filter chain on raw (rpm, torque) samples.

    gauge, graph : 0–10 noise-filter levels (Gauge MA on samples uses half-width
                   floor(gauge/2); Graph MA on the binned curve uses half-width
                   = graph bins, matching the workbook's O column).
    spike        : 0–5 pre-clean (Hampel). 0 = off, which reproduces the workbook
                   exactly (the control is defined there but left unwired in V0_1).
    bin_width    : RPM bin size (workbook uses 100).
    sae_factor   : multiply corrected torque (use sae_j607_factor(); 1.0 = off).

    Torque is unit-agnostic (the J607 factor is dimensionless); compute power from
    the returned torque with the constant matching its unit. Returns a dict:
        bin_rpm, bin_torque (bin-average × factor, pre-graph),
        smoothed_torque (final = graph MA × factor), count, std  — or None.
    """
    rpm = np.asarray(rpm, dtype=float)
    torque = np.asarray(torque, dtype=float)
    m = np.isfinite(rpm) & np.isfinite(torque)
    rpm, torque = rpm[m], torque[m]
    if rpm_min is not None:
        sel = rpm >= rpm_min
        rpm, torque = rpm[sel], torque[sel]
    if rpm_max:
        sel = rpm <= rpm_max
        rpm, torque = rpm[sel], torque[sel]
    if rpm.size == 0:
        return None

    # 0) Optional sample spike removal (off by default → matches workbook V0_1)
    if spike and spike > 0:
        torque = hampel_despike(torque, window=2 * int(spike) + 1, n_sigma=3.0)

    # 1) Gauge moving average on samples (in input/time order, half-width floor(gauge/2))
    tq_g = _centered_mean_hw(torque, int(gauge) // 2)

    # 2) Bin to bin_width and average the gauge-filtered torque (J1349 binning)
    bw = float(bin_width)
    bin_id = np.round(rpm / bw).astype(int)          # nearest bin (ROUND(A/bw)*bw)
    ids = np.arange(bin_id.min(), bin_id.max() + 1)
    bin_rpm = ids.astype(float) * bw
    bin_tq = np.full(ids.size, np.nan)
    count = np.zeros(ids.size, dtype=int)
    std = np.full(ids.size, np.nan)
    for i, gid in enumerate(ids):
        sel = bin_id == gid
        c = int(sel.sum())
        count[i] = c
        if c:
            bin_tq[i] = tq_g[sel].mean()
            if c >= 2:
                std[i] = float(np.std(torque[sel], ddof=1))   # raw-torque std (N col)

    # 3) Graph moving average over the binned curve (±graph bins, skip empty bins)
    smoothed = _centered_mean_hw(bin_tq, int(graph), ignore_nan=True)

    # 4) SAE J607 correction (dimensionless multiplier)
    f = float(sae_factor)
    return {
        "bin_rpm": bin_rpm,
        "bin_torque": bin_tq * f,
        "smoothed_torque": smoothed * f,
        "count": count,
        "std": std,
    }


# ═════════════════════════════════════════════════════════════════════════════
# RPM-domain resampling (final-curve presentation)
# ═════════════════════════════════════════════════════════════════════════════
def resample_by_rpm(rpm, torque, rpm_step=25.0, rpm_min=None, rpm_max=None):
    """Bin (rpm, torque) onto a uniform RPM grid (mean per bin, gaps interpolated).

    Returns (grid_rpm, grid_torque). Empty bins between populated ones are
    linearly interpolated; the curve is trimmed to the populated RPM span.
    """
    rpm = np.asarray(rpm, dtype=float)
    torque = np.asarray(torque, dtype=float)
    mask = np.isfinite(rpm) & np.isfinite(torque)
    rpm, torque = rpm[mask], torque[mask]
    if rpm.size == 0:
        return np.array([]), np.array([])
    lo = math.floor((rpm_min if rpm_min is not None else rpm.min()) / rpm_step) * rpm_step
    hi = math.ceil((rpm_max if rpm_max is not None else rpm.max()) / rpm_step) * rpm_step
    if hi <= lo:
        hi = lo + rpm_step
    grid = np.arange(lo, hi + rpm_step, rpm_step)
    sums = np.zeros(grid.size)
    counts = np.zeros(grid.size)
    # round-half-up (np.round is banker's rounding → inconsistent on bin midlines)
    idx = np.clip(np.floor((rpm - lo) / rpm_step + 0.5).astype(int), 0, grid.size - 1)
    np.add.at(sums, idx, torque)
    np.add.at(counts, idx, 1)
    populated = counts > 0
    if populated.sum() < 2:
        return grid[populated], (sums[populated] / np.maximum(counts[populated], 1))
    means = np.full(grid.size, np.nan)
    means[populated] = sums[populated] / counts[populated]
    # Interpolate interior gaps, then trim to populated span.
    first, last = np.where(populated)[0][[0, -1]]
    g = grid[first:last + 1]
    m = means[first:last + 1]
    nan = np.isnan(m)
    if nan.any():
        m[nan] = np.interp(g[nan], g[~nan], m[~nan])
    return g, m


# ═════════════════════════════════════════════════════════════════════════════
# Data loading — auto-detect the DIY dyno's various log formats
# ═════════════════════════════════════════════════════════════════════════════
@dataclass
class Recording:
    """A loaded time-series ready for replay / analysis."""
    t: np.ndarray                       # elapsed seconds from start (monotonic)
    rpm: np.ndarray | None              # engine RPM, or None if not present
    torque: np.ndarray | None           # torque (raw counts or Nm — see torque_is_nm)
    channels: dict = field(default_factory=dict)   # all named columns
    fmt: str = "unknown"
    path: str = ""
    rate_hz: float = 0.0
    dup_fraction: float = 0.0
    torque_is_nm: bool = False          # True if torque is already engineering units
    notes: str = ""

    @property
    def duration(self):
        return float(self.t[-1] - self.t[0]) if self.t.size else 0.0

    @property
    def n(self):
        return int(self.t.size)


def _try_parse_timestamp(token):
    token = token.strip()
    for fmt in _TS_FORMATS:
        try:
            return datetime.strptime(token, fmt)
        except ValueError:
            continue
    return None


def _looks_like_header(first_line, delim):
    """True if the first row is a header (no parseable leading timestamp/number)."""
    parts = [p.strip() for p in first_line.split(delim)]
    if not parts:
        return True
    if _try_parse_timestamp(parts[0]) is not None:
        return False
    try:
        float(parts[0])
        return False        # numeric first cell → data, not header
    except ValueError:
        return True


def _sniff_delimiter(line):
    if "\t" in line and "," not in line:
        return "\t"
    if ";" in line and line.count(";") >= line.count(","):
        return ";"
    return ","


def _clean_number(token):
    """Extract a float from a token that may carry a label like 'RPM: 1234'."""
    token = token.strip()
    if ":" in token:
        token = token.split(":", 1)[1].strip()
    try:
        return float(token)
    except ValueError:
        return math.nan


def _read_text_rows(path):
    """Read a delimited text/CSV log → (header_or_None, rows[list[str]])."""
    with open(path, "r", newline="", encoding="utf-8", errors="replace") as f:
        raw_lines = [ln.rstrip("\n") for ln in f if ln.strip()]
    if not raw_lines:
        raise ValueError(f"{path}: file is empty")
    delim = _sniff_delimiter(raw_lines[0])
    has_header = _looks_like_header(raw_lines[0], delim)
    header = None
    data_lines = raw_lines
    if has_header:
        header = [h.strip() for h in raw_lines[0].split(delim)]
        data_lines = raw_lines[1:]
    rows = [[c.strip() for c in ln.split(delim)] for ln in data_lines]
    return header, rows


def _read_xlsx_rows(path):
    """Read the first worksheet of an .xlsx/.xlsm → (header_or_None, rows).

    Cells are stringified so the rest of load_recording's text pipeline
    (timestamp / number parsing) works unchanged. Requires openpyxl — an
    optional dependency; recorded runs can also be exported to CSV/TXT.
    """
    try:
        import openpyxl
    except ImportError as e:
        raise ValueError(
            f"{path}: reading .xlsx needs openpyxl (pip install openpyxl), "
            "or export the sheet to CSV/TXT first."
        ) from e

    def _cell(v):
        if v is None:
            return ""
        if isinstance(v, datetime):
            return v.isoformat(sep=" ")     # → 'YYYY-MM-DD HH:MM:SS[.ffffff]'
        return str(v)

    wb = openpyxl.load_workbook(path, data_only=True, read_only=True)
    try:
        ws = wb.active
        all_rows = [[_cell(c) for c in row]
                    for row in ws.iter_rows(values_only=True)
                    if row is not None and not all(c is None for c in row)]
    finally:
        wb.close()
    if not all_rows:
        raise ValueError(f"{path}: no data rows")

    # Header row = first row whose first cell is neither a timestamp nor a number.
    header, data = None, all_rows
    first = all_rows[0]
    if first and _try_parse_timestamp(first[0]) is None:
        try:
            float(first[0])
        except (ValueError, TypeError):
            header = [c.strip() for c in first]
            data = all_rows[1:]
    return header, data


def load_recording(path, *, torque_scale=1.0, torque_offset=0.0,
                   torque_is_nm=None, column_map=None):
    """Load a dyno log / CSV / XLSX into a Recording, auto-detecting the format.

    torque_scale / torque_offset: applied to the raw torque column as
        torque_eng = (raw - offset) * scale     (use to map ADC counts → Nm).
    torque_is_nm: override the engineering-units flag (default: inferred).
    column_map: optional dict overriding auto-detect, e.g.
        {"time": 0, "rpm": 1, "torque": 3, "time_is_seconds": True}.

    .xlsx / .xlsm files are read via openpyxl (optional dependency); all other
    extensions are parsed as delimited text with auto-detected delimiter/header.
    """
    if str(path).lower().endswith((".xlsx", ".xlsm")):
        header, rows = _read_xlsx_rows(path)
    else:
        header, rows = _read_text_rows(path)
    if not rows:
        raise ValueError(f"{path}: no data rows")
    ncols = max(len(r) for r in rows)

    # ── Establish the time axis ──────────────────────────────────────────────
    first_cell = rows[0][0]
    ts0 = _try_parse_timestamp(first_cell)
    time_is_seconds = ts0 is None
    times = np.empty(len(rows))
    if time_is_seconds:
        for i, r in enumerate(rows):
            times[i] = _clean_number(r[0]) if r else math.nan
    else:
        base = ts0
        for i, r in enumerate(rows):
            ts = _try_parse_timestamp(r[0]) if r else None
            times[i] = (ts - base).total_seconds() if ts else math.nan
    # Fill any unparseable timestamps by forward-hold.
    for i in range(times.size):
        if math.isnan(times[i]):
            times[i] = times[i - 1] if i else 0.0
    # Enforce a non-decreasing time axis (guards clock resets / out-of-order rows;
    # build_replay_samples' np.searchsorted requires monotonic t).
    times = np.maximum.accumulate(times)

    # ── Determine column semantics ───────────────────────────────────────────
    fmt, rpm_idx, torque_idx, infer_nm = _classify_columns(
        header, ncols, rows, None, time_is_seconds
    )
    if column_map:
        rpm_idx = column_map.get("rpm", rpm_idx)
        torque_idx = column_map.get("torque", torque_idx)

    def col(idx):
        if idx is None:
            return None
        out = np.empty(len(rows))
        for i, r in enumerate(rows):
            out[i] = _clean_number(r[idx]) if idx < len(r) else math.nan
        return out

    rpm = col(rpm_idx)
    torque_raw = col(torque_idx)

    # 5-column format: derive RPM from pulse interval if the RPM column is
    # absent/zero/blank (blank cells parse to NaN, not 0 — test NaN-safely).
    if fmt == "pulse5" and torque_idx is not None:
        interval = col(4)
        rpm_finite = np.isfinite(rpm) if rpm is not None else None
        rpm_missing = (rpm is None or rpm_finite.sum() == 0
                       or np.nanmax(np.abs(rpm[rpm_finite])) == 0)
        if rpm_missing:
            with np.errstate(divide="ignore", invalid="ignore"):
                rpm = np.where(interval > 0, 60.0 / interval, 0.0)

    torque = None
    if torque_raw is not None:
        torque = (torque_raw - torque_offset) * torque_scale

    # torque_is_nm: honor an explicit override; otherwise infer from the header,
    # and treat any applied scale/offset (raw counts → Nm) as engineering units.
    if torque_is_nm is not None:
        is_nm = torque_is_nm
    else:
        is_nm = bool(infer_nm or torque_scale != 1.0 or torque_offset != 0.0)

    # ── Build channel dict for completeness ──────────────────────────────────
    channels = {}
    for j in range(ncols):
        name = (header[j] if header and j < len(header) else f"col{j}")
        if j == 0:
            channels["time_s"] = times
            continue
        channels[name] = col(j)

    rate_hz, dup_frac = _estimate_rate(times)

    return Recording(
        t=times, rpm=rpm, torque=torque, channels=channels,
        fmt=fmt, path=str(path), rate_hz=rate_hz, dup_fraction=dup_frac,
        torque_is_nm=bool(is_nm),
        notes=("time column is elapsed seconds" if time_is_seconds
               else "timestamps converted to elapsed seconds"),
    )


def _classify_columns(header, ncols, rows, delim, time_is_seconds):
    """Return (fmt, rpm_idx, torque_idx, torque_is_nm)."""
    # Header-driven detection (YourDyno ';' export, pressure CSVs, xlsx-style).
    if header:
        low = [h.lower() for h in header]

        def find(*keys):
            for i, h in enumerate(low):
                if any(k in h for k in keys):
                    return i
            return None
        rpm_i = find("rpm")
        tq_i = find("torque")
        is_nm = tq_i is not None and "nm" in low[tq_i]
        if rpm_i is not None and tq_i is not None:
            return "header", rpm_i, tq_i, is_nm
        # Headered but no rpm/torque (e.g. pressure CSV) → no replayable curve.
        return "header-other", rpm_i, tq_i, is_nm

    # Headerless DIY dyno logs: dispatch on column count.
    if ncols == 3:
        # RPM log: Timestamp, RPM(maybe 'RPM:' labelled), Counter
        return "rpmlog3", 1, None, False
    if ncols == 4:
        # Timestamp, RPM, Torque, Brake
        return "pull4", 1, 2, False
    if ncols == 5:
        # Timestamp, RPM, Torque, PPR, PulseInterval
        return "pulse5", 1, 2, False
    if ncols >= 6:
        # Disambiguate control-sim (col4 ≈ thousands) vs old (col4 < 10).
        try:
            c4 = np.nanmedian([_clean_number(r[4]) for r in rows[:200] if len(r) > 4])
        except (ValueError, IndexError):
            c4 = 0.0
        fmt = "ctrlsim6" if (c4 and abs(c4) >= 100) else "old6"
        return fmt, 1, 2, False
    return "unknown", (1 if ncols > 1 else None), (2 if ncols > 2 else None), False


def _estimate_rate(times):
    """Return (rate_hz, duplicate_fraction) from a monotonic-ish time axis."""
    if times.size < 2:
        return 0.0, 0.0
    dt = np.diff(times)
    dup_fraction = float(np.mean(dt <= 0))
    pos = dt[dt > 0]
    if pos.size == 0:
        return 0.0, dup_fraction
    median_dt = float(np.median(pos))
    rate = 1.0 / median_dt if median_dt > 0 else 0.0
    return rate, dup_fraction


def build_replay_samples(rec, target_hz=25.0):
    """Decimate a Recording to ~target_hz evenly in TIME for smooth replay.

    Returns (t, rpm, torque) arrays sampled on a uniform time grid spanning the
    recording, using nearest-sample hold. High-rate logs (kHz) collapse to a
    sane playback cadence while preserving the real time span.
    """
    if rec.rpm is None or rec.torque is None or rec.n < 2:
        raise ValueError("recording has no RPM/torque channel to replay")
    # Defensive: align all channels to a common length.
    m = min(rec.t.size, rec.rpm.size, rec.torque.size)
    t, rpm_src, tq_src = rec.t[:m], rec.rpm[:m], rec.torque[:m]
    # Drop non-finite samples so replay never injects NaN into the live pipeline.
    fin = np.isfinite(t) & np.isfinite(rpm_src) & np.isfinite(tq_src)
    t, rpm_src, tq_src = t[fin], rpm_src[fin], tq_src[fin]
    m = t.size
    if m < 2:
        raise ValueError("recording has too few finite RPM/torque samples to replay")
    t0, t1 = t[0], t[-1]
    span = max(t1 - t0, 1e-3)
    n_out = max(2, int(round(span * target_hz)))
    grid = np.linspace(t0, t1, n_out)
    # nearest original index for each grid time
    idx = np.searchsorted(t, grid).clip(1, t.size - 1)
    left = grid - t[idx - 1]
    right = t[idx] - grid
    idx = np.where(left <= right, idx - 1, idx)
    idx = idx.clip(0, m - 1)
    return grid - t0, rpm_src[idx], tq_src[idx]

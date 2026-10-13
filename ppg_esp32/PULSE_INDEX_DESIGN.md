# Relative optical pulse index x — parameter roles, design decisions, tests

x is a **relative optical pulse index**: the DC-normalised optical pulse compared with a fixed reference taken in a
known unrestricted condition. x = 0.45 means 45 % of the reference normalised optical pulse strength. It is **not**
a percentage of blood flow remaining. It does not certify tourniquet effectiveness, adequate tissue perfusion or
bandage safety. Code: `m14_flow_level.h` (index), `m10_perfusion_monitor.h` (reference capture), `config.h` §8b.

## 1. Available measurements (inspected in the code, firmware as configured now)

Hardware enabled now: MAX30102 (IR + RED), one ADXL345 on the probe (`NUM_ACCEL 1`). **Not enabled**: band
MPU-6050 (supported, `NUM_ACCEL 2`), FSR pressure (`USE_FSR 0`), NIRS (`NIRS_MODE 0`). These are not used.

| Parameter | Definition | Units | Rate | Valid when | Status |
|---|---|---|---|---|---|
| Raw IR / RED | MAX30102 18-bit FIFO | counts | 125 Hz | sensor answering, not clipped | measured |
| Optical DC `D` | DC tracker (0.05 Hz) of raw IR; per pulse = mean over the pulse samples | counts | 125 Hz / per pulse | all DC samples > 0 | measured |
| PPG, VPG, APG | band-passed (0.5–6 Hz), Savitzky-Golay smoothing + 1st/2nd derivative | a.u., a.u./s, a.u./s² | 125 Hz | after 6 s warm-up | measured |
| Amplitude `A` | systolic peak above the pulse's own onset-to-onset line | counts (filtered) | per pulse (~2 beats late) | pulse accepted | measured |
| PI | 100·A/D | % | per pulse | A > 0, D valid | measured |
| Pulse area (AUC) | ∫ over onset→onset above the same line | counts·s | per pulse | accepted | measured |
| HR, pulse interval | tangent-onset to tangent-onset | bpm, s | per pulse | accepted | measured |
| Max upstroke slope | VPG maximum | a.u./s | per pulse | accepted | measured |
| Crest time, PW50 | tangent onset→peak; width at 50 % | s | per pulse | accepted | measured |
| Notch type, APG b/a…e/a, aging index, IPA, AIx, LASI | waveform landmarks | ratio, s, m/s | per pulse | where found | measured, exploratory |
| relativeAmplitude, relativePI | A/A_ref, PI/PI_ref | × ref | per pulse | reference held | measured |
| pulseDetectionRate | detected / expected beats at reference HR over eligible still time (10 s) | ratio | 1 Hz | ≥ 3 s eligible | measured |
| amplitudeCV | sample SD/mean of the last 10 accepted amplitudes | ratio | per pulse | ≥ 3 pulses | measured |
| templateCorrelation | shape correlation with the waveform frozen at **b** | −1..1 | per pulse | reference held | measured |
| SQI + fail flags | 5 s window, 1 s hop: flat, periodicity, skew, kurtosis, entropy, PI, clipping, non-finite | bit mask | 1 Hz | after warm-up | measured |
| Motion: whole-hand energy | probe accelerometer dynamic energy, averaged ~0.2 s | (m/s²)² | 125 Hz | sensor answering | measured |
| Motion: relative (probe − band) | needs the band sensor | (m/s²)² | — | — | **supported, not measured** (`NUM_ACCEL 1`) |
| ADC saturation | raw ≥ clip level | flag | 125 Hz | — | measured |
| Sensor status / acquisition | I2C errors, FIFO overflow, wall-clock sample gap > 1 s | counts, flag | continuous | — | measured |
| LED current | AGC codes RED / IR | ×0.2 mA | 2 Hz decisions | locked from **b** | measured |
| SpO2 | Maxim algorithm on RED/IR | % | 1 Hz | correlation ≥ 0.8 | measured, not used for x |
| Band pressure (FSR), NIRS | — | — | — | — | **supported code only, hardware not enabled** |

## 2. Roles

**Pulse-strength measurement (determines x):** PI, through relativePI only.

**Measurement validity (determines availability):** reference present; probe on tissue (raw IR ≥ threshold);
ADC clipping; warm-up; motion (large and small); band-pressure change (inactive without FSR); sensor answering;
acquisition running (wall clock); pulse freshness; finite PI; the pulse did not overlap motion; optical settings
unchanged since **b** (LED current lock — a forced change clears the reference).

**Supporting diagnostics (reported, never part of x):** relativeAmplitude, relativeDC, relativeAreaPI (and its
ratio to relativePI), relativeSlopePI, crest-time ratio, width-fraction ratio, hrRatio, pulseDetectionRate,
amplitudeCV (with the reference CV), templateCorrelation, SQI failure type, APG / notch features, SpO2.

Overlaps (why they are not independent evidence):
- relativeAmplitude = relativePI × relativeDC, exactly. Amplitude adds nothing beyond PI except the DC change,
  which is reported directly as relativeDC.
- Area, slope and amplitude all scale with the same pulse; area = A·T·(shape fill), slope ≈ A / rise time.
  Their ratios mostly repeat PI; what they add is **shape** information (fill, rise time).
- pulseDetectionRate and hrRatio both move with heart rate.
- templateCorrelation is amplitude-independent by construction (unit-norm shapes): a small, perfectly shaped pulse
  correlates 1.0 and stays a small pulse (tested).
- The legacy m10 "flow score / GOOD-REDUCED-POOR" is the median of four of these overlapping ratios with
  uncalibrated cutoffs. It is kept as report text, labelled a heuristic, and is not used for x or the LEDs.

## 3. Per-pulse features (same definitions for the reference and the current pulse)

- A_i = systolic peak above the straight line between this pulse's two onsets (feet).
- D_i = mean of the raw-IR DC tracker over the same samples (never the zero-centred PPG). Any non-positive DC
  sample makes PI unavailable.
- PI_i = 100·A_i/D_i.
- areaPI_i = 100·area_i/(T_i·D_i), with T_i = the onset-to-onset duration the area was integrated over. This is
  the time-averaged pulsatile fraction. Dividing by T removes the heart-rate dependence of a raw area. It is
  **not** a blood volume in mL.
- slopePI_i = 100·upSlope_i/D_i. It depends on amplitude **and** rise time. It is **not** a blood velocity.
- widthFrac_i = PW50_i/T_i. Crest time is kept in seconds, because systolic timing does not scale with the beat
  length. Its ratio is therefore HR-dependent, and that is stated wherever it is shown.

## 4. Baseline

- Captured **manually**: command **b**, `AUTO_BASELINE 0`, so a restricted condition is never auto-adopted as normal.
- Content: medians of 10 accepted **still** pulses with a finite PI. Stored values: A, PI, HR, D, areaPI,
  slopePI, crest, widthFrac, the amplitude CV of those 10 pulses, the frozen clean waveform, and the LED currents.
- Optical settings: the AGC may adjust during setup and is locked from **b** on. Only ADC clipping can still force
  a change, and that clears the reference.
- Fixed while monitoring. It is cleared by **r**, probe removal, a sensor re-initialisation or an LED-current change.

## 5. Primary estimator

```
relativePI = PI_i / PI_ref        (unclipped, reported)
target     = clamp(relativePI, 0, 1)
x          = target                                   first usable pulse after any UNKNOWN
x          = FL_X_KEEP·x + (1 − FL_X_KEEP)·target     once per NEW accepted pulse, FL_X_KEEP = 0.75
```
- Each pulse is used once (guarded by its onset time). Nothing is smoothed between pulses.
- A pulse without a finite PI is not used. relativeAmplitude is never substituted.

## 6. How the other parameters are used

- **Motion:** large and small motion are both invalid for x (`FL_USE_SMALL_MOTION 0`).
- **SQI:** a failed window is **not** an invalidation by itself, because a genuinely weak pulse makes SQI poor.
  Pulses inside failed windows are already rejected by the pulse analyser. When no pulse arrives, the UNKNOWN reason
  names the failure type:
  - corrupted (clipping / non-finite)
  - weak (flat / low PI)
  - irregular / changed shape
- **Clipping and sensor status** invalidate x.
- **Freshness** is checked with sample timestamps, and acquisition stops are checked with the wall clock.
- **DC / LED current** are reported. A DC change explains a relativeAmplitude-vs-relativePI difference. It does
  not prove stable contact.
- **Consistency check:** the median of relativeAreaPI / relativePI over the last 3 pulses, compared against
  1/1.5 … 1.5 (`FL_SUPPORT_RATIO`, an engineering threshold, not calibrated). Outside that range the report says
  DISAGREE: the pulse shape changed relative to its peak. **x is not changed.**
- **No quality multiplication:** x is never multiplied by a quality score. No overall quality probability is
  reported.

## 7. Multi-parameter estimator?

- The project has no synchronised reference-flow measurements and no labelled validation recordings. The only
  recordings mentioned in the code are motion recordings without a flow reference, and cuff pressure would not be
  ground truth anyway.
- **Decision:** relativePI stays the only strength estimator. No weights, clinical cutoffs or accuracy claims.
- **If reference data become available:**
  - Model a defined target, for example laser-Doppler flow relative to baseline.
  - Use one feature per overlapping group: PI, the shape fill, and HR.
  - Split by participant or session.
  - Compare against relativePI alone.
  - Keep the model behind a separate switch until it is shown to be better.

## 8. Missing and invalid data → UNKNOWN (level 0, all LEDs off, never "zero flow")

| Cause | When |
|---|---|
| no reference, probe off, sensor failure, stopped acquisition (wall clock), filter restart | immediately |
| clipping, warm-up, motion, pressure change | after 1 s of continuous invalid data (`FL_INVALID_GRACE_S`); shorter blips only pause x |
| no fresh pulse | the newest accepted pulse **ended** > 5 s ago (`FL_PULSE_TIMEOUT_S`; includes the ~1-beat analysis delay) |

- Every UNKNOWN clears x, the smoothing state and the diagnostics.
- After invalid data, only a pulse that **started** after it can restore a level.
- A missing pulse is never turned into x = 0.
- UNKNOWN shows as all LEDs **off, not blinking**. The spec asked for a blinking pattern; you chose no blinking, so
  this follows your choice.

## 9. LEDs

- level = min(10, 1 + floor(10·x)): 0 ≤ x < 0.1 → 1, …, 0.9 ≤ x ≤ 1 → 10.
- Hysteresis `FL_HYST` = 0.02.
- LEDs 1..level are lit: GPIO 19, 2, 15, 13, 14, 27, 26, 25, 33, 32 (pins unchanged).

## 10. Teleplot

- Default: the agreed 16 channels.
  - `relativeAmplitude`, `relativePI`, `PulseIndex` (= the actual x) and the diagnostics: 1 Hz.
  - `BloodFlowLevel` (the actual level after hysteresis): 10 Hz.
- Unavailable values are sent as −1 (−2 for the correlation). UNKNOWN clears them, so stale diagnostics never stay.
- `TELEPLOT_SUPPORT 1` adds the exploratory channels: relativeDC, relativeAreaPI, relativeSlopePI, hrRatio,
  areaVsPI, supportAgreement.
- `tools/flow_viewer.py` plots the level step graph with relativeAmplitude, relativePI and x together below it.
  It uses the firmware values; nothing is recalculated.

## 11. Tests (`sh test/run_tests.sh`; on this PC via `python -m ziglang c++`)

| Test | Covers | Result |
|---|---|---|
| T9 | equation; numerical example (x → 0.4555, level 5); exact update rule; no inter-pulse smoothing; a pulse used once; NaN-PI not substituted; all ten levels and band edges (19 values); hysteresis up **and** down at all 9 edges | pass |
| T10 | baseline / half / weak (levels 10 / 6 / 2); weak but perfectly regular (corr 1.0 → identical x to corr 0.2); DC ×2 with the same A (relPI 0.5, relAmp 1, relDC 2); HR change (x unchanged); trend 1.0 → 0.4 (level never rises, CV 0.12) vs random ±30 % (x ≈ 0.5); changed shape (x unchanged, DISAGREE reported); detection rate, CV | pass |
| T11 | stale → UNKNOWN (not 0) and fresh restart; large and small motion (blip paused, > 1 s UNKNOWN, only post-motion pulses restore); clipping; warm-up; probe off; SQI weak / corrupt / shape reasons; invalid PI / A; sensor fault; stopped acquisition; samples lost; reference reset and recapture; m10 reference record (medians, DC, area-PI, CV, LED codes) | pass |
| T12 | real pipeline: A 1000 → 450 (relPI 0.450, level 5); DC ramp ×2 (relPI 0.500, relDC 2.002); HR ramp 75 → 109 bpm (relPI 0.94, hrRatio 1.455, detection rate 1.42); changed shape (relPI 0.93, corr 0.79); weak regular A 120 (level 2, corr 1.000); pulses stop → UNKNOWN after 4.0 s | pass |
| T13 | t/f calibration ignored while off; mapping when `FL_TWO_POINT_CAL 1` | pass |
| sketch test (report + Teleplot) | real `ppg_esp32.ino`: levels per phase; motion paused < 1 s then UNKNOWN; LED current locked after **b** and released after probe removal; BloodFlowLevel = LED level; exactly 16 channels | pass |
| agc_test | unlocked current follows the light; locked current and registers unchanged; clipping overrides the lock | pass |
| ESP32 build | `pio run -e esp32dev` | pass (RAM 24 %, flash 26 %); **not flashed** |

Did the extra parameters help?
- **Validity:** the motion, clipping, sensor, acquisition and freshness checks are what make x available or not.
- **Explanation:** the SQI failure type now explains UNKNOWN (weak vs corrupted vs irregular) but does not change
  availability.
- **Diagnostics only:** area, slope, crest, width, HR, CV, detection rate and correlation describe the pulse and
  expose disagreements (shown in T10 and T12). Without reference data there is no evidence that they would improve
  the strength estimate, so they stay out of x.

**Found and not hidden:**
- An **abrupt** HR jump from 75 to 109 bpm (> 1/0.72 of the beat interval) falls inside the beat detector's
  refractory window (`m08`, `HR_REFRACTORY_FRAC`). The detector then locks at half rate: measured hrRatio 0.73.
  relativePI stayed correct (0.94), but the timing diagnostics are wrong. A gradual change of the same size is
  tracked correctly. T12 prints this as a known limitation.
- The pre-existing `test_ppgnr` check "output differs from the original NLMS" still fails. It is unrelated to x.

**Not validated:**
- No comparison against any flow reference.
- No human or limb recordings with these settings.
- The new defaults have not run on hardware. In the live session on 2026-10-13, about 26 % of samples were
  flagged as motion with one accelerometer. Because small motion is now invalid and invalid data > 1 s gives
  UNKNOWN, expect the LEDs to go off during finger tremor more often than with the session firmware. If that
  happens:
  - set `FL_USE_SMALL_MOTION 1`, or `FL_HOLD_ON_INVALID 1`, or
  - fit the band sensor (`NUM_ACCEL 2`) so that relative motion is measured.

## Optional switches (from the 2026-10-13 live session)

**Changed on user request after the strict version was tried on hardware** ("too sensitive to motion"):
`FL_USE_SMALL_MOTION 1` (small-motion samples no longer invalidate x) and `FL_HOLD_ON_INVALID 1` (the last level is
held up to 20 s while motion / invalid data blocks pulses). The hold is limited to gaps caused by recent motion or
invalid data: a pulse that disappears while the hand is still still becomes UNKNOWN after 5 s (tested in T11 and in
the sketch test). Section 8 above describes the strict rules; set both switches to 0 to return to them.

| Switch | When 1 |
|---|---|
| `AUTO_BASELINE` | **ON now (user request)** - reference captured automatically whenever there is none (boot, probe re-placed, LED-current change), once the LED current has been stable for 4 s. Put the probe on with the limb unrestricted |
| `FL_TWO_POINT_CAL` | t / f tight / free calibration, 8 equal bands between, stored in flash |
| `FL_HOLD_ON_INVALID` | **ON now** - hold the last level up to 20 s while motion / invalid data blocks pulses |
| `FL_NO_PULSE_IS_LOW` | **ON now (user request)** - no accepted pulse for 5 s AND the PPG only noise (RMS < 10 % of the reference amplitude) → level 1 = red LED only, x = 0. A display choice for "no detectable pulse", not a measured zero flow; a strong PPG whose pulses are only rejected is never shown red |
| `FL_MEDIAN3` | target = median of the last 3 relativePI |
| `FL_USE_SMALL_MOTION` | **ON now** - small-motion samples are not invalid |
| `TELEPLOT_SUPPORT` | exploratory supporting channels |

Kept on from the session, because it is a validity fix: the motion energy is averaged over 0.2 s
(`MOTION_ENERGY_TAU_S`). Single-sample tremor spikes no longer count as motion.

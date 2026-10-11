# Changes and analysis

## 1. What was added / changed (old ppg_esp32 -> this version)

| Request | What was done |
|---|---|
| Keep the old architecture | One flat sketch folder; `ppg_esp32.ino` contains `setup()`, `loop()`, `handleSample()`, report printing. No `firmware.h` / namespaces / `src` / `legacy` folders (the 4.37 layout was **not** used). |
| Spike rejection before the band-pass (friend) | `ppg_spike.h` Hampel, 5 samples, k = 5, first stage of `pipeline.h`, and also on RED + IR in the SpO2 branch. Test: spike-caused error 0.07 counts with it vs 265 counts without (T3). |
| NLMS motion cancellation with the accelerometers (friend) | `m06_nlms_anc.h` (manuth `NLMS-ANC.h` = `ppg_anc.h`) + `m06b` references, PPG and references get the same delay + high-pass, adaptation frozen below `ANC_P_MIN`. **Off by default** (`USE_ANC 0`) exactly as advised: enable after recordings show motion is the problem. |
| AUC code | `m09_auc_calculator.h`: your streaming `AucCalculator` (wrap-safe time difference, optional positive-only) + segment `PulseAuc` with one straight onset-to-onset baseline (total = systolic + diastolic). Pulse area is one of the four flow features. |
| minor-noise libraries | `ppg_biquad/dc/spike/savgol/anc/force/sqi/noise.h` added flat (the hardened 4.37 copies); `pipeline.h`, m06, m07 are built on them. Their test `test/test_ppgnr.cpp` passes. |
| updated_libraries-manuth | `wave_detection.h` segmentation (onset-to-onset, one pulse per report), `NLMS-ANC.h` (in m06), two-ADXL345 + FSR settings (in `config.h`). `test_wave.cpp` is superseded by `test/native_test.cpp` T5. |
| Good parts of 4.37 | Wire drivers with burst FIFO reading (the SparkFun 4-sample buffer lost data), non-blocking serial logger, improved peak detector, FSR/motion gating, linear-baseline amplitude, robust `zig()`, baseline/ratio monitoring idea. |
| NIRS pins missing | Pins in `config.h` for both boards (LED1, LED2, PD1, PD2), `m13_nirs.h` drives the LEDs, subtracts dark, computes dHbO2 / dHHb / dHbT, printed every second. |
| Output "name: value" | `printFeature("Systolic amplitude", ...)` -> `Systolic amplitude: 864.43 a.u.`; all 18 requested features + heart rate, pulse interval, max upstroke slope, notch, flow ratios, **Flow status**, SpO2. |
| Are the features enough for "is good flow reaching the limb"? | The morphology list alone gives numbers but no decision. Added: the VPG **max upstroke slope** (derivative = inflow speed), a **baseline**, four **ratios**, a **flow score** and a **Flow status** (GOOD / REDUCED / POOR / NO PULSE), plus motion / pressure gates so a moving or squeezed probe is not read as a flow change. |

Further fixes found while testing:
- Respiratory baseline wander hid weak pulses from the SQI (15 % pulse was called NO PULSE). SQI and beat detection
  now use a 0.5 Hz high-passed copy; features still use the gentler 0.15 Hz signal.
- Onset = intersecting tangent for timing (heart-rate SD 4 -> 0.5 bpm, crest-time SD 40 -> 6 ms on the test signal);
  the foot minimum is still the amplitude / area baseline.
- Peaks are refined to the true PPG maximum before segmentation.
- Inflection point area ratio is now A2/A1 (diastolic / systolic, Wang 2009); old code printed A1/A2.
- On ESP32-S3 the I2C pins are 8/9 (GPIO 22/23 do not exist on the S3). Classic ESP32 keeps 22/23.

## 2. text1 analysis: smoothing, derivative, sampling, Maxim SpO2

### 2.1 Recommended sampling frequency
**125 Hz for the PPG path** (MAX30102: 1000 sps, average 8, read from the FIFO), **25 Hz for the Maxim SpO2 path**
(low-pass 6 Hz then average-of-5 decimation). Both are defined once in `config.h`:
`PPG_FS 125`, `SPO2_DECIMATION 5`, `SPO2_FS = PPG_FS / SPO2_DECIMATION = 25`, and `FreqS` in `spo2_algorithm.h` is
`SPO2_FS`. A `static_assert` stops the build if they disagree.

### 2.2 What "ms" means in smoothing and the recommended value
`ms` is the length of the moving-average window in time; samples = round(ms x fs / 1000). At 125 Hz (8 ms/sample),
measured on a 75 bpm pulse (test T1):

| window | samples | delay | first null | systolic peak kept |
|---|---|---|---|---|
| 40 ms | 5 | 2 samples (16 ms) | 25.0 Hz | 98 % |
| 80 ms | 10 | 4.5 samples (36 ms) | 12.5 Hz | 92 % |
| 120 ms | 15 | 7 samples (56 ms) | 8.3 Hz | 84 % |
| 200 ms | 25 | 12 samples (96 ms) | 5.0 Hz | 65 % |

More window = less noise but a lower, wider, later peak and a filled-in notch (pulse harmonics reach 8-10 Hz).
After the 10 Hz low-pass, **40 ms (5 samples)** is the right moving-average value: it removes the remaining
sample-to-sample noise and distorts almost nothing. 120-200 ms is too much.
Default in this project: **Savitzky-Golay, 15 samples (120 ms), cubic** - it smooths like a long window but a
polynomial fit keeps the peak and notch, and it produces the derivatives from the same fit.

### 2.3 Derivative method
| method | accuracy | noise | cost | delay | memory | real-time PPG |
|---|---|---|---|---|---|---|
| forward (x[n+1]-x[n])/dt | O(dt), half-sample shift | high | 1 sub | 1 sample | 1 value | poor |
| backward (x[n]-x[n-1])/dt (old) | O(dt), half-sample shift; x3 for JPG = 1.5 samples off | high | 1 sub | 0 | 1 value | poor |
| central (x[n+1]-x[n-1])/(2dt) | O(dt^2), centred | lower (gain -> 0 at fs/2) | 1 sub | 1 sample | 2 values | good |
| Savitzky-Golay derivative (15, cubic) | least-squares, centred, all outputs aligned | lowest | 15 MAC per output | 7 samples (56 ms) | 15 values | best |

Measured VPG error on a noisy pulse (T2): backward 9.9 %, central 4.7 %, Savitzky-Golay 1.7 %.
For the 2nd derivative (APG ratios) the difference is decisive: with 0.4 % noise, moving average + differences
gave b/a = 0.34 +- 0.55 (true -2.22), Savitzky-Golay 15 gave -2.22 +- 0.06 (`test_ppgnr` T7).
So central difference is a real improvement over forward/backward, and Savitzky-Golay is the practical
higher-order method on an ESP32 (default). Central differences remain available (`SMOOTH_METHOD SMOOTH_MOVING_AVG`).

**Correct dt** = 1 / PPG_FS = **0.008 s** (the derivatives run on the 125 Hz PPG, not on the 25 Hz SpO2 data).

**Order** raw -> filter/smooth -> derivative, because differentiation multiplies each frequency by 2*pi*f: noise at
10-60 Hz would be amplified 60-400x before reaching the APG. Smoothing first removes it while the pulse (< 10 Hz) passes.
The derivative is only used for PPG analysis (upstroke speed, fiducials), never for blood-flow direction.

### 2.4 FreqS, BUFFER_SIZE and TARGET_BUFFER_SIZE
`FreqS 25`, `BUFFER_SIZE = FreqS*4 = 100` and `TARGET_BUFFER_SIZE 100` are consistent only if the samples given
to the algorithm are at **25 Hz**: 100 / 25 = **4 s**, which Maxim needs to see 3-6 beats (>= 2 valleys).
The old processor fed 125 Hz samples directly: 100 samples = 0.8 s (< 1 beat at 60 bpm), so mostly invalid, and
the MA4 width, valley distance and HR = FreqS*60/interval would all be 5x wrong. At 100 Hz it would be 1 s with
the same problem. **Do not change FreqS to the sensor rate; decimate to 25 Hz instead** (done in m11).
Update interval: old = one result per full buffer, then the buffer was discarded. New = sliding 4 s window,
**new estimate every 1 s**, displayed value = median of the last 5 valid estimates.

### 2.5 Problems found in `spo2_algorithm.h` (all fixed)
1. `n_x_ac = an_x[n_y_dc_max_idx] - n_x_ac;` must be `an_x[n_x_dc_max_idx]`: the IR AC has to be taken at the IR
   peak. With the RED index, IR AC is read at the wrong sample (smaller), R is biased high -> SpO2 biased low.
2. `if (an_ir_valley_locs[k] > BUFFER_SIZE)` lets index 100 through; must be `>= BUFFER_SIZE`.
3. `maxim_peaks_above_min_height` reads `pn_x[i + n_width]` one past the end on a plateau at the end.
4. MA4 left the last 3 samples unsmoothed but they were still used in the threshold and the valley search.
5. `n_ir_buffer_length` ignored (BUFFER_SIZE used everywhere) - a different length means out-of-bounds access.
6. `n_y_ac * n_x_dc_max` in 32 bit can overflow with 18-bit data (AC 10 000 x DC 262 143 > 2^31) - now 64 bit.
7. The "median" averaged the wrong pair for odd counts (count 5 -> averaged elements 1 and 2).
8. Table declared `[184]` with 183 values: index 183 silently returned 0 % - now sized from the data.
9. Minimum valley distance 4 samples (160 ms = 375 bpm) can count dicrotic notches - now FreqS*60/MAX_BPM = 8.
10. Negative AC (no pulse between valleys) gave negative ratios - skipped.
11. `[cite: ..]` text after `#include` lines stops compilation.
Also: valleys come from a 4-point average whose output is centred 1.5 samples later; each valley is now refined to
the raw IR minimum. Test T4: R = 0.4 ... 1.2 returns the table value +- 1 % and HR 71 bpm (true 70.6).

### 2.6 Corrected files
`m04_smoothing.h` (smoothing.h), `derivatives.h` (derivative.h), `spo2_algorithm.h`, `m11_spo2_processor.h`
(class `SpO2Processor`, same `begin() / pushSample(ir, red, quality) / getSpO2() / isSpO2Valid()` interface).

### 2.7 How the complete system works
Every 8 ms the MAX30102 produces an IR + RED pair. The IR goes through spike removal, DC removal, filtering and
Savitzky-Golay smoothing, giving a clean PPG and its derivatives; peaks are found, each full pulse is measured, and
the pulse's amplitude, perfusion index, area and upstroke slope are compared with the baseline to give the flow
status. Accelerometers and band FSRs veto pulses recorded during motion or changing pressure. In parallel the raw
RED + IR are cleaned, reduced to 25 Hz and passed every second to the corrected Maxim algorithm for SpO2. The NIRS
LEDs are switched in turn ten times a second and the change in absorbed light gives oxy- / deoxy-haemoglobin trends.

## 3. Test results (desktop, g++)
- `test/native_test.cpp`: all unit tests pass (smoothing, derivatives, Hampel, Maxim, features vs truth, AUC,
  flow classification, NIRS inversion). Clean pulse: HR 75.00 +- 0.06 bpm, amplitude CV 0.2 %, area CV 0.4 %.
- `test/sketch_test.cpp` (real `.ino` + simulated hardware): every flow phase classified correctly, SpO2 97 %,
  NIRS -12 / +9 uM recovered, FIFO-overflow and probe-removal events handled; all 15 config variants compile
  without warnings (failures only where the scenario does not apply, e.g. plotter mode has no reports).
- `test/test_ppgnr.cpp` (minor-noise suite) and `test/anc_test.cpp` (old ANC test, REL9 0.35 -> 0.73) pass.
- AddressSanitizer + UBSan clean; also builds as C++11 (ESP32 core 2.x).
- Not done: a build with the real ESP32 toolchain and any test on hardware.

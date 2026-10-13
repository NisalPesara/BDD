# ESP32 PPG pipeline + limb blood-flow assessment (SpO2, NIRS, motion, pressure)

Same layout as the old `ppg_esp32`: one sketch folder, `ppg_esp32.ino` holds `setup()` / `loop()` /
`handleSample()`, and every method is a separate header next to it (no sub-folders for libraries).
Sources: Park et al. 2022 *Front. Physiol.* 12:808451 + pyPPG, the **minor-noise** libraries (`ppg_*.h`),
**updated_libraries-manuth** (wave detection, NLMS, two-ADXL345 config) and the useful fixes from **4.37**.
See `CHANGES_AND_ANALYSIS.md` for what changed and the full SpO2 / smoothing / derivative analysis.

> Research prototype, not a validated medical device. It shows whether **pulsatile** blood flow reaches
> the limb compared with a baseline; it cannot measure flow in mL/min. Never use it alone to decide
> tourniquet application or release.

## Hardware and wiring (current)
Board **ESP32 Dev Module** (classic ESP32). Only two sensors are connected now, both on the same I2C bus:

| Sensor | Connection | I2C address |
|---|---|---|
| MAX30102 | SDA -> GPIO 22, SCL -> GPIO 23, VIN -> 3.3 V, GND | 0x57 |
| ADXL345 | SDA -> GPIO 22, SCL -> GPIO 23, **CS -> 3.3 V** (I2C mode), **SDO / "AD0" -> GND** | 0x53 |

No other GPIO is used. FSR, a second (band) ADXL345 and NIRS are prepared but **commented out** in `config.h`
and `ppg_esp32.ino`; their reserved pins for later are FSR 34/35, NIRS photodiodes 32/33, NIRS LEDs 25/26,
band ADXL345 0x1D (SDO -> 3.3 V). To add one: set its switch (`USE_FSR 1`, `NUM_ACCEL 2`, `NIRS_MODE 1`) and
uncomment its block - the build stops with a clear `#error` if a needed line is still commented.
If an ESP32-S3 is selected the build stops too: the S3 has no GPIO 22/23.

## How to use
1. Copy the folder `ppg_esp32/` into your sketchbook (the folder name must match `ppg_esp32.ino`).
2. Arduino IDE: install **esp32 by Espressif**, select Board **ESP32 Dev Module**.
   **No extra libraries** are needed any more (the MAX30102 and ADXL345 are driven directly with `Wire`).
3. `config.h` is already set for MAX30102 + one ADXL345 (`NUM_ACCEL 1`, `USE_FSR 0`, `NIRS_MODE 0`). Check `MAX_LED_CURRENT_CODE`.
4. Upload, open the Serial Monitor at **460800** baud.
5. Put the probe on in a known **unrestricted** condition and keep the limb still. After warm-up (~6 s) send
   **b**: the next 10 accepted still pulses become the fixed **reference** (only **b** captures it: `AUTO_BASELINE 0`,
   so a restricted condition can never silently become "normal"). The LED current is locked from **b** on.
   Until the reference exists the 10 LEDs stay off (UNKNOWN).
6. Watch the 10 LEDs / `BloodFlowLevel` (relative pulse level, below). The report mode (**p**) prints x, why it is
   UNKNOWN when it is, and the supporting diagnostics.
7. Optional (off by default, `FL_TWO_POINT_CAL 1` in `config.h`): **t** / **f** capture a tight / free value and the
   LEDs use 8 equal bands between them; **c** clears it.

Serial commands: **b** reference (flow + NIRS), **r** reset, **p** cycle report / Serial-Plotter / Teleplot, **h** help.

## Relative optical pulse-strength index x (10 LEDs, `m14_flow_level.h`)
x tracks how the PPG pulse changes during limb-circulation restriction, relative to the **b** reference.
**It is not a blood-flow percentage**: x = 0.45 means the normalised optical pulse is 45 % of the reference pulse.
It does not show how much blood flow remains, or that a tourniquet / bandage is safe.

| Step | Calculation |
|------|-------------|
| per accepted **still** pulse | A = systolic peak above the pulse baseline; D = mean raw-IR DC over the same pulse; PI = 100 A / D |
| reference (**b**) | baselineAmplitude = median A, baselinePI = median PI of 10 still pulses; pulse shape frozen; LED current locked |
| per NEW pulse | relativeAmplitude = A / baselineAmplitude, relativePI = PI / baselinePI (unclipped) |
| index | target = clamp(relativePI, 0, 1); x = target (first), then x = 0.75 x + 0.25 target (once per pulse) |
| LEDs | level = min(10, 1 + floor(10 x)), hysteresis 0.02; LEDs 1..level lit - **more LEDs = stronger pulse** |
| invalid data | clipping, warm-up, motion (large **and** small) or pressure change: x is paused; lasting > 1 s (`FL_INVALID_GRACE_S`) -> UNKNOWN. Afterwards only a pulse that **started** after the invalid stretch restores a level |
| UNKNOWN | level 0, **all LEDs off** (no blinking): no reference, probe off, sensor failure, stopped acquisition (at once), invalid data > 1 s, or the newest accepted pulse ended > 5 s ago (the reason is reported, e.g. the SQI failure type) |

A missing pulse is shown as UNKNOWN, never as level 1 / "zero". The reference is cleared by **r**, probe removal,
a sensor re-initialisation or an LED-current change (send **b** again). Example: reference A = 1000, D = 100000
(PI 1 %); now A = 450 (PI 0.45 %) -> relativePI 0.45 -> x approaches 0.45 -> level 5.
Diagnostics, reported but **not part of x**: `relativeAmplitude` (= relativePI x relativeDC), `relativeDC`,
`relativeAreaPI` (area-based strength; its ratio to relativePI is a consistency check - a disagreement is reported,
never averaged into x), `relativeSlopePI`, crest / width ratios, `hrRatio`, `pulseDetectionRate`, `amplitudeCV`,
`templateCorrelation`. Unavailable: -1 (-2 for the correlation). Full parameter roles: **PULSE_INDEX_DESIGN.md**.

**Teleplot channels** (exactly these): `PPG`, `PPG_noANC`, `VPG`, `APG`, `SQI`, `RelMotion`, `HandMotion`, `Moving`
(per sample); `BloodFlowLevel` = the LED level 0..10 (10 Hz); `relativeAmplitude`, `relativePI`, `PulseIndex` (= x),
`pulseDetectionRate`, `amplitudeCV`, `templateCorrelation`, `IR_DC` (1 Hz).
`TELEPLOT_SUPPORT 1` adds `relativeDC`, `relativeAreaPI`, `relativeSlopePI`, `hrRatio`, `areaVsPI`, `supportAgreement`.
**Viewer:** `python tools/flow_viewer.py` (close Teleplot first) draws BloodFlowLevel as a step graph with a shaded
band per level and a separate UNKNOWN row, and below it relativeAmplitude, relativePI and PulseIndex x together
(`--demo` without a board).

## Data flow
```
MAX30102 IR (125 Hz) -> tissue check -> Hampel spike removal -> DC removal (DC kept for PI) -> high-pass 0.15 Hz
      -> [NLMS motion cancellation, refs from both ADXL345 through the same delay + high-pass]  (USE_ANC)
      -> low-pass 10 Hz -> Savitzky-Golay smoothing + VPG + APG (+ JPG)                      = clean PPG
clean PPG -> 0.5 Hz detection copy -> SQI (m07) + systolic peaks (m08)
          -> wave_detection.h: onset/peak/notch/a-e per pulse + features + AUC (m09)
          -> m10: ratios to baseline -> GOOD / REDUCED / POOR / NO PULSE
          -> m14: relative pulse-strength index x -> 10 LEDs + Teleplot BloodFlowLevel
ADXL345 x2 + FSR x2 -> m12: motion gate + band-pressure-stable gate (bad samples never become pulses)
MAX30102 RED + IR raw -> Hampel -> low-pass 6 Hz -> /5 = 25 Hz -> 4 s window, 1 s hop -> corrected Maxim SpO2 (m11)
NIRS LED1 / LED2 / dark (10 frames/s) -> dark-subtracted light -> dOD -> modified Beer-Lambert -> dHbO2, dHHb (m13)
```

## How blood flow is judged (peak detection + derivative)
Per accepted pulse (onset -> systolic peak -> notch -> next onset), compared with the baseline median:

| Feature | From | Meaning for the limb |
|---|---|---|
| Systolic amplitude | peak detection | strength of each pulse |
| Perfusion index | amplitude / DC | strength corrected for light level |
| Pulse area (AUC) | m09, whole pulse | pulse volume (less sensitive to one noisy point) |
| Max upstroke slope | VPG (1st derivative) maximum | how fast blood volume rises = arterial inflow |

Flow score = median of the four ratios. **GOOD** >= 0.70, **REDUCED** 0.30-0.70, **POOR** < 0.30 (confirmed over
3 pulses), **NO PULSE** = no acceptable pulse for 8 s while optical signal, pressure and motion are fine.
Also watch: crest time grows and the dicrotic notch disappears as inflow drops.

## Output (Name: value unit)
```
==================== PULSE #30 ====================
Heart rate: 75.642 bpm
Systolic amplitude: 864.43 a.u.
Perfusion index: 0.72201 %
Pulse width / PW50: 0.13363 s
Pulse area: 163.64 a.u.*s
...                                   (all 18 features of the requested list + upstroke slope)
(b-c-d-e)/a aging index: -3.7063 ratio
Dicrotic notch: present
-- Blood flow vs baseline --
Amplitude ratio: 0.97278 x baseline
Flow score: 0.97286 x baseline
Flow status: GOOD FLOW
SpO2: 97 %
```
`n/a` = could not be measured on this pulse (e.g. Delta T needs a real diastolic peak). The status block every
second adds signal quality, heart rate, SpO2, band forces, motion energy and NIRS dHbO2 / dHHb / dHbT.
`tools/receive_from_esp32.py COM5 120` records everything and writes `session_pulses.csv` / `session_status.csv`.

## Files
| File | What | Origin |
|---|---|---|
| `ppg_esp32.ino` | main sketch: sensors, per-sample flow, reports | old, rewritten |
| `config.h` | all pins (incl. NIRS), rates, thresholds | old + manuth + 4.37 |
| `pipeline.h` | the processing chain | old, rebuilt on the libraries |
| `ppg_biquad.h ppg_dc.h ppg_spike.h ppg_savgol.h ppg_anc.h ppg_force.h ppg_sqi.h ppg_noise.h` | noise-removal libraries | minor-noise (hardened 4.37 copies) |
| `m01_frequency_filters.h` + `filter_coeffs.h` | Park 2022 filter tables (optional `PPG_BANDPASS`) | old |
| `m02` baseline, `m03` moving difference, `m05` amplitude regularisation (kept, unused), `m06b` accel references | | old |
| `m04_smoothing.h` | moving average (O(1), ms -> samples explained) | old + 4.37 |
| `derivatives.h` | Savitzky-Golay and central-difference derivatives, aligned | new |
| `m06_nlms_anc.h` | NLMS with energy gate / leak | manuth `NLMS-ANC.h` -> `ppg_anc.h` |
| `m07_sqi.h` | SQI with fail bitmask | `ppg_sqi.h` + config |
| `m08_peak_detection.h` | prominence + adaptive refractory peaks | 4.37 |
| `wave_detection.h` | per-pulse fiducials and features | manuth + 4.37 + tangent onset |
| `m09_auc_calculator.h` | AUC: segment (linear baseline) + streaming `AucCalculator` | your AUC code + 4.37 |
| `m10_perfusion_monitor.h` | baseline, ratios, flow status | new (idea from 4.37) |
| `m14_flow_level.h` | relative pulse-strength index x, LED level, UNKNOWN handling, diagnostics | new |
| `spo2_algorithm.h`, `m11_spo2_processor.h` | corrected Maxim SpO2, 25 Hz processor | text1 + fixes |
| `m12_measurement_context.h` | FSR pressure + motion gates | 4.37 |
| `m13_nirs.h` | NIRS LED sequencing + Beer-Lambert | new |
| `sensor_drivers.h`, `serial_logger.h` | Wire drivers (FIFO bursts), non-blocking output | 4.37 |
| `test/` | desktop tests (`sh test/run_tests.sh`) | |
| `tools/` | `gen_coeffs.py` (old), `receive_from_esp32.py` (new parser), `flow_viewer.py` (level graph), `calibrate.py` (obsolete) | |

## Tests (desktop, no ESP32 needed)
`sh test/run_tests.sh` (g++). The end-to-end test compiles the real `ppg_esp32.ino` against simulated
MAX30102 / ADXL345 / FSR / NIRS hardware and runs: no probe -> normal flow ('b' reference) -> motion ->
50 % occlusion -> 15 % -> full occlusion -> release -> 400 ms loop stall (FIFO overflow) -> probe removed.
Every status was reported correctly (GOOD 51/51, MOTION 5/5, REDUCED 40/40, POOR 41/41, NO PULSE 10/10,
NO SIGNAL 3/3, no-baseline-after-reseat 16/16); SpO2 97 % for a simulated R = 0.60; NIRS returned the simulated
-12 / +9 uM; injected 25 000-count glitches were removed (unit test: 0.07 counts residual error vs 265 without the Hampel stage). Also clean under AddressSanitizer/UBSan and in C++11 mode.
The same run checks the LED level per phase (full 10, half 5, 15 % -> 2, missing pulse / motion / no reference /
FIFO restart -> UNKNOWN), the LED-current lock, and in Teleplot mode that exactly the 16 channels above are sent.
Unit tests T9-T13 cover x itself (equation, all ten levels + hysteresis both ways, DC / HR / shape changes, trend
vs noise, motion / clipping / SQI / stale / sensor cases, real-pipeline runs); `test/agc_test.cpp` the LED-current
lock. Results and open limitations: PULSE_INDEX_DESIGN.md.

## Important caveats
- **Not compiled for or run on the ESP32 here** (no ESP32 toolchain was reachable). Desktop builds use stubs of
  the Arduino API. Do a first upload and check the boot lines (`MAX30102 found`, both ADXL345 found).
- On an S3 the I2C pins are now **8 / 9** because GPIO 22/23 do not exist on that chip. Re-wire, or change `config.h`.
- `MAX_LED_CURRENT_CODE 0x24` (7.2 mA, old value). On a limb/toe site you may need more (watch `Raw IR` and
  `SENSOR SATURATED`).
- Thresholds (SQI, motion energy, FSR slope, flow ratios) are starting values from simulation: tune on recordings.
- SpO2 uses Maxim's fingertip calibration; NIRS uses literature extinction coefficients, DPF = 6 and 3 cm spacing.
  Both are uncalibrated trends until compared with reference instruments.
- `Inflection point area ratio` is now diastolic / systolic area (A2/A1, Wang 2009). The old code printed the inverse.
- Large-artery stiffness index and the SDPTG aging index are fingertip indices: exploratory on a limb.

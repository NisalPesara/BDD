// ===================== USER SETTINGS =====================
// ONE place for every pin, rate and threshold. Every module reads its settings from here.
#pragma once
#include <stdint.h>
#if defined(ARDUINO_ARCH_ESP32)
#include "sdkconfig.h"        // defines CONFIG_IDF_TARGET_ESP32S3 / CONFIG_IDF_TARGET_ESP32
#endif

// =====================================================================
// 1. BOARD + PINS  (what is wired NOW)
// =====================================================================
// CURRENT HARDWARE: classic ESP32 (ESP32 Dev Module) + MAX30102 + TWO ADXL345
//   MAX30102 : SDA -> GPIO 22, SCL -> GPIO 23, VIN 3.3 V, GND               (I2C 0x57)
//   ADXL345 finger (on the PPG probe): SDA 22, SCL 23, CS -> 3.3 V (= I2C mode),
//              SDO/ALT ADDRESS ("AD0") -> GND  => address 0x53            (I2C 0x53)
//   MPU-6050 band (on the hand / band): SDA 22, SCL 23, VCC 3.3 V, GND,
//              AD0 -> GND (or open)  => address 0x68                      (I2C 0x68)
// No FSR, no NIRS -> no other GPIO is used.
#define BOARD_NAME            "ESP32"
#define I2C_SDA               22
#define I2C_SCL               23
#define I2C_CLOCK_HZ          100000 // was 400000: bus errors with the long wires to the hand (2026-10-13)
#define SERIAL_BAUD           460800 // set the Serial Monitor / Teleplot to this (115200 is too slow for the
                                     // per-sample teleplot stream with the extra motion channels)
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #error "ESP32-S3 selected, but an S3 has no GPIO 22/23. Select 'ESP32 Dev Module', or wire I2C to GPIO 8/9 and change I2C_SDA/I2C_SCL."
#endif

// ---- NOT USED NOW: pins for later hardware (uncomment when that hardware is added) ----
// #define FSR1_PIN           34     // band FSR 1  (ADC1, input-only)        -> also section 4
// #define FSR2_PIN           35     // band FSR 2  (ADC1, input-only)
// #define PPG_ADC_PIN        36     // analog PPG sensor (SENSOR_ANALOG only)
// #define NIRS_PD1_PIN       32     // NIRS photodiode far  (ADC1)            -> also section 5
// #define NIRS_PD2_PIN       33     // NIRS photodiode near (ADC1)
// #define NIRS_LED1_PIN      25     // NIRS LED 730 nm driver (output)
// #define NIRS_LED2_PIN      26     // NIRS LED 850 nm driver (output)

// =====================================================================
// 2. PPG SENSOR (MAX30102)
// =====================================================================
#define SENSOR_ANALOG         1
#define SENSOR_MAX30102       2
#ifndef SENSOR_TYPE   // SENSOR_ANALOG also needs PPG_ADC_PIN (commented above)
#define SENSOR_TYPE           SENSOR_MAX30102
#endif

#define PPG_FS                125    // Hz. 1000 sps internal / average 8 = 125 Hz. dt = 1/125 = 0.008 s
#define MAX30102_ADDR         0x57
#define MAX_LED_CURRENT_CODE  0xFF   // START current, code x 0.2 mA: 0xFF = 51 mA = the MAX30102 maximum.
                                     // With USE_LED_AGC it is lowered only if the ADC clips.
// Automatic LED current control (led_agc.h): keeps the raw DC of RED and IR near AGC_TARGET of full scale,
// i.e. the brightest light that does not clip -> largest pulse amplitude, lowest relative noise.
#ifndef USE_LED_AGC
#define USE_LED_AGC           1
#endif
#define AGC_TARGET            0.70f  // aim: 80 % of the 18-bit range (was 60 %): as bright as possible, below clipping. Wrist A-B test (ADC range 16384 nA):
                                     // sensor noise 227 -> 75 ppm and pulse SNR 8 -> 12 dB from 9 mA to 35 mA
#define AGC_LOW               0.50f  // re-adjust only outside 55..92 % (each change restarts the filters).
                                     // Below 55 % the current goes up (until 51 mA).
#define AGC_HIGH              0.85f  // 2026-10-13: was 0.80 / 0.55 / 0.92. At 51 mA the IR DC sat at 94 % of full
                                     // scale; hand pressure then clipped it, the locked current was forced down and
                                     // the reference cleared (LEDs off). 70 % leaves headroom while locked
#define AGC_MIN_CODE          0x05   // 1 mA
#define AGC_MAX_CODE          0xFF   // 51 mA = datasheet maximum. NOTE: with USE_WIFI 1 this loads the supply so
                                     // much that Wi-Fi current bursts modulate the LEDs -> then use 0xAF (35 mA)
#define AGC_WINDOW_SAMPLES    (PPG_FS / 2)   // decide on a 0.5 s mean
#define AGC_SETTLE_SAMPLES    40     // ignore ~0.3 s after a change (FIFO samples at the old current)
#define MAX_RAW_FULL_SCALE    262143.0f   // 18-bit FIFO
#define FINGER_THRESHOLD      50000.0f    // IR counts: below this = no tissue on the sensor (old value)

// MAX30102 IR is REFLECTED light: it goes DOWN when blood volume goes UP.
// 1 = flip the sign so the systolic peak points up. Verify by plotting (OUTPUT_MODE_PLOTTER).
#ifndef INVERT_SIGNAL
#define INVERT_SIGNAL         1
#endif

// =====================================================================
// 3. ACCELEROMETERS (2x ADXL345) - relative motion: gating + motion cancellation
// =====================================================================
#ifndef NUM_ACCEL
#define NUM_ACCEL             1      // TEMPORARY 2026-10-13: band MPU-6050 wire is loose (drops ~8x/min, corrupt motion). Set back to 2 when fixed. 2 = finger ADXL345 (0x53) + band MPU-6050 (0x68)
#endif
#define ADXL345_FINGER_ADDR   0x53   // SDO -> GND
#define ADXL345_ADDR          ADXL345_FINGER_ADDR   // legacy name
// Three motion levels (whole motion = probe + hand sensor dynamic energy, (m/s^2)^2):
//   below MOTION_SMALL_ENERGY            QUIET : data used
//   MOTION_SMALL .. MOTION_HIGH_ENERGY   SMALL : motion is cancelled, then the PPG ITSELF decides: the data is
//                                                used while its level stays <= ARTIFACT_MOTION_K x the rest level
//   above MOTION_HIGH_ENERGY             HIGH  : data rejected without looking at the PPG
// Calibrated on the 2026-10-12 wrist recordings (cleaned PPG level per second vs motion):
//   < 0.15 -> ~1x rest | 0.15-1.0 -> 1.0-7x (about half clean) | 1-10 -> mostly 2.7-31x, sometimes 1.3x
//   (cancelled well) -> the PPG check decides; > 10 -> always heavily distorted
#define MOTION_SMALL_ENERGY   0.15f
#define MOTION_HIGH_ENERGY    10.0f  // was 1.0: live test 2026-10-12 had motion 2-3 with the cleaned PPG at 1.3x
                                     // (clean) -> wrongly rejected. Up to 10 the PPG check decides
#define MOTION_MAX_ENERGY     MOTION_SMALL_ENERGY   // legacy name
#define MOTION_HOLD_SAMPLES   (PPG_FS / 2)   // stay "moving" for 0.5 s after the last motion sample
#define MOTION_ENERGY_TAU_S   0.2f   // motion energy averaged over ~0.2 s before the thresholds (tremor spikes ignored)

// ---- second (band / hand) accelerometer. Relative motion = finger - band ----
#define BAND_SENSOR_ADXL345   0      // a second ADXL345, SDO -> 3V3 => 0x1D
#define BAND_SENSOR_MPU6050   1      // MPU-6050 (accelerometer part), AD0 -> GND => 0x68  (fitted now)
#ifndef BAND_SENSOR
#define BAND_SENSOR           BAND_SENSOR_MPU6050
#endif
#if BAND_SENSOR == BAND_SENSOR_MPU6050
  #define BAND_ACCEL_ADDR     0x68
  #define BAND_ACCEL_NAME     "MPU-6050 band (0x68)"
#else
  #define BAND_ACCEL_ADDR     0x1D
  #define BAND_ACCEL_NAME     "ADXL345 band (0x1D)"
#endif
static const int   BAND_AXIS_IDX[3] = {0, 1, 2};    // band axes mapped into the finger frame (coarse;
static const float BAND_AXIS_SGN[3] = {1.f, 1.f, 1.f};   // ACC_ALIGN learns the exact rotation)

// Motion gate: relative motion (probe on the skin) and whole motion are both graded SMALL / HIGH (above).
// The 2026-10-12 recording showed whole-hand motion corrupts the PPG too (blood / tissue inertia),
// and the canceller removes ~80-90 % of it, so the PPG itself decides for small motion.
#define MOTION_GATE_RELATIVE  1
#define REL_MOTION_MAX_ENERGY 0.5f   // (m/s^2)^2 |finger - M*band|^2 above this = probe moving on the skin (SMALL)
#define REL_MOTION_HIGH_ENERGY 5.0f  // above this = probe clearly sliding (HIGH, rejected). Below: PPG decides
                                     // Tune: watch >RelMotion in Teleplot while moving the whole hand
                                     // (should stay low) and while pushing the probe (should go high).
// Learned band-to-finger frame alignment (m06b): makes common motion cancel even when the two
// sensors are not mounted parallel. Learns only while the band sensor moves.
#define ACC_ALIGN             1
#define ACC_ALIGN_MU          0.003f // NLMS step: ~3 s of hand motion to converge
#define ACC_ALIGN_MIN_ENERGY  0.3f   // (m/s^2)^2 band dynamic energy needed to learn (above sensor noise)

// Motion cancellation (NLMS, m06 + m06b): removes the part of the PPG that is correlated with the
// RELATIVE motion (finger vs band). Runs after the PPG low-pass; references get the same filters.
#ifndef USE_ANC
#define USE_ANC               1
#endif
#define ANC_MODE_ABS6         0      // finger xyz (+ band xyz if fitted), gravity removed
#define ANC_MODE_REL9         1      // finger acc, RELATIVE acc (finger-band), RELATIVE velocity (2 sensors)
#define ANC_MODE_REL6         2      // RELATIVE acc + RELATIVE velocity only (whole-hand motion ignored)
#define ANC_MODE_RELH9        3      // RELATIVE acc + RELATIVE velocity + HAND acc (best on the 2026-10-12 recording)
#ifndef ANC_MODE
#define ANC_MODE              ANC_MODE_RELH9
#endif
#if ANC_MODE == ANC_MODE_REL9 || ANC_MODE == ANC_MODE_RELH9
  #define ANC_NUM_REFS        9
#else
  #define ANC_NUM_REFS        6
#endif
#define ACC_RAW_CH            6      // finger xyz, band xyz (band = 0 when not fitted)
#define ANC_TAPS              16     // 128 ms of reference history (artifact lags the motion a little)
#define ANC_MU                0.5f   // fast: motion episodes are short. Tuned by replaying the 2026-10-12 recording:
                                     // 90 % of the motion power removed, pulse at rest changed 1 %
#define ANC_P_MIN             4.0f   // reference energy (sum over refs x taps) below this = no motion: freeze.
                                     // Lower values let the canceller learn on sensor noise and eat the pulse
#define ANC_LEAK              0.0f
#define ANC_GUARD_TAU_S       0.25f  // overshoot guard: if the cleaned PPG has more power than the input over this
                                     // window, the input is used (recordings 3+4: 12 % less residual motion)
#define ANC_IDLE_DECAY_S      0.5f   // while frozen the weights fade out (time constant) -> pulse passes untouched
#define ACC_VEL_LEAK_HZ       0.3f
#define ACC_VEL_SCALE         12.0f

// =====================================================================
// 4. FORCE SENSORS (FSR) - NOT USED NOW
// =====================================================================
#define USE_FSR               0      // 0 = no FSR wired. To add them: set 1, uncomment FSR1/2_PIN above and below.
// #define FSR_GATE_PPG       1      // 1: reject pulses while band pressure is changing
// #define FSR_ADC_BITS       12
// #define FSR_OVERSAMPLE     4
// #define FSR_SUPPLY_MV      3300.0f
// #define FSR_FIXED_OHM      10000.0f
// #define FSR_RAIL_MV        3090.0f
// #define FSR_SLOPE_MAX_US_S 100.0f
// #define FORCE_STABLE_SAMPLES (PPG_FS / 2)

// =====================================================================
// 5. NIRS - NOT USED NOW (no GPIO needed until the NIRS hardware exists)
// =====================================================================
#define NIRS_MODE             0      // 0 = off. To add NIRS: set 1, uncomment the NIRS pins above, the block
                                     // below, and the lines marked "NIRS" in ppg_esp32.ino.
// #define NIRS_LED_ACTIVE_HIGH 1
// #define NIRS_FRAME_HZ        10
// #define NIRS_SETTLE_US       400
// #define NIRS_ADC_OVERSAMPLE  8
// #define NIRS_HAS_PD2         1
// #define NIRS_BASELINE_FRAMES 20
// #define NIRS_MIN_MV          20.0f
// #define NIRS_MAX_MV          3090.0f
// #define NIRS_WL1_NM          730
// #define NIRS_WL2_NM          850
// #define NIRS_E_HBO2_WL1      0.390f     // extinction coefficients 1/(mM*cm), Prahl
// #define NIRS_E_HHB_WL1       1.1022f
// #define NIRS_E_HBO2_WL2      1.058f
// #define NIRS_E_HHB_WL2       0.69132f
// #define NIRS_SD_DISTANCE_CM  3.0f
// #define NIRS_DPF             6.0f

// =====================================================================
// 6. SIGNAL PROCESSING
// =====================================================================
#define WARMUP_SECONDS        6      // filter settling; samples before this are flagged invalid

// Spike rejection (Hampel = moving median / MAD), placed BEFORE the band-pass so a single
// glitch (I2C hiccup, brief saturation) is removed instead of being smeared into an IIR tail.
#define HAMPEL_WINDOW         5      // samples (odd, 3..15). Delay = 2 samples = 16 ms
#define HAMPEL_K              5.0f   // replace if |x - median| > K * 1.4826 * MAD
#define HAMPEL_MIN_SIGMA      2.0f   // counts; stops quantisation steps being flagged

#define DC_TRACK_HZ           0.05f  // slow DC level (raw level for perfusion index)
#define PPG_HIGHPASS_HZ       0.5f   // baseline wander. Wrist recording: 72 % of the power was 0.1-0.5 Hz drift
                                     // (breathing / vasomotion); 0.5 Hz removes it and cut wrong beat intervals 40 % -> 11 %.
                                     // Fingertip waveform-shape work: 0.15 Hz keeps the notch better.
#define PPG_LOWPASS_HZ        6.0f   // broadband noise, 4th-order Butterworth. Was 10 Hz (fingertip shape detail);
                                     // a wrist pulse has almost no energy above 5 Hz, so 6 Hz removes noise only

// Band-pass choice. 0 = run-time Butterworth HP + LP above (default, any fs).
// Or one of the m01 FilterMethod values (e.g. FM_CHEBY2_PYPPG = pyPPG default 0.5-12 Hz),
// which then replaces the low-pass stage (tables in filter_coeffs.h, fs = 125 Hz).
#ifndef PPG_BANDPASS
#define PPG_BANDPASS          0
#endif

// Smoothing + derivatives
#define SMOOTH_SAVGOL         0      // Savitzky-Golay: smoothing + VPG + APG from ONE polynomial fit (aligned)
#define SMOOTH_MOVING_AVG     1      // moving average (m04) + central-difference derivatives
#ifndef SMOOTH_METHOD
#define SMOOTH_METHOD         SMOOTH_SAVGOL
#endif
#define SG_HALF               7      // SG window = 2*7+1 = 15 samples = 120 ms at 125 Hz, delay 7 samples
#define SG_ORDER              3      // cubic: keeps peak height and the dicrotic notch
#define SM_PPG_MS             40     // moving-average window (only for SMOOTH_MOVING_AVG): 40 ms = 5 samples

#define USE_BASELINE_SUB      0      // m02 lower-envelope baseline subtraction (usually not needed)
#define USE_AMP_REGULARIZE    0      // m05: DESTROYS amplitude -> must stay 0 for flow/PI/AUC features
#define MOVE_DIFF_SAMPLES     4      // m03 up-slope window

// =====================================================================
// 7. SIGNAL QUALITY (SQI, m07) - thresholds are starting values, tune on your data
// =====================================================================
#define SQI_WINDOW_S          5
#define SQI_HIGHPASS_HZ       0.5f   // SQI + beat detection look at a 0.5 Hz high-passed copy, so respiratory /
                                     // vasomotor baseline wander cannot hide a weak (low-perfusion) pulse.
                                     // Features are still measured on the 0.15 Hz signal (shape preserved).
#define SQI_HOP_S             1
#define SQI_MIN_PERIODICITY   0.35f  // was 0.50 (fingertip); wrist pulses vary more beat to beat (median 0.25-0.4)
#define SQI_MIN_SKEW          -0.4f  // fingertip ~0.3; limb pulses are more symmetric. Was 0.0: on 2026-10-12 still
                                     // windows with periodicity 0.73-0.90 failed ONLY on skew (-0.01 .. -0.34)
#define SQI_KURT_MIN          1.5f
#define SQI_KURT_MAX          8.0f
#define SQI_MIN_ENTROPY       1.5f
#define SQI_MAX_ENTROPY       4.0f
#define SQI_FLAT_SD           0.5f   // counts: window SD below this = flat line
#define SQI_MAX_CLIP_FRAC     0.08f

// =====================================================================
// 8. PULSE / FLOW ASSESSMENT
// =====================================================================
#define MIN_BPM               40.0f
#define MAX_BPM               180.0f
#define HR_MEDIAN_N           5      // heart rate = median of the last 5 beat intervals (m08)
#define HR_MIN_INTERVALS      3      // no HR until 3 consistent intervals exist
#define HR_OUTLIER_FRAC       0.30f  // interval > 30 % from the median = false / missed beat -> ignored
#define HR_RELOCK_REJECTS     3      // 3 rejections in a row = real rhythm change -> restart the history
#define HR_REFRACTORY_FRAC    0.72f  // no new beat within 72 % of the beat interval (was 60 %): the wrist
                                     // diastolic wave at ~66 % of the cycle was counted as a beat (HR 135 vs 92)
// Motion artifact detected from the PPG itself (pipeline.h) - also works without the ADXL345
#define ARTIFACT_K            8.0f   // |signal| > 8 x its running mean level = movement. Normal wrist pulses reach
                                     // 5x; hand movement in the recordings reached 15-20x (4x flagged 12-24 % of clean data)
#define ARTIFACT_HOLD_S       1.0f   // stay "moving" 1 s after the last such sample
#define ARTIFACT_TAU_S        5.0f   // running level time constant (not updated during movement)
#define ARTIFACT_MAX_S        3.0f   // a level lasting > 3 s is accepted as the new normal (not during motion)
// During SMALL motion: cleaned PPG RMS over ~1 s vs its RMS at rest
#define ARTIFACT_MOTION_K     2.0f   // RMS > 2x the rest RMS = distorted -> rejected (calibrated, see section 3)
#define ARTIFACT_RMS_TAU_S    1.0f   // short-term RMS window
#define ARTIFACT_REST_TAU_S   10.0f  // rest RMS reference (updated only while quiet and clean)
// Beat-by-beat check during SMALL motion (wave_detection.h): a pulse that overlaps motion is used only if its
// SHAPE matches the template learned from still pulses and its amplitude is normal. Live test 2026-10-12:
// the RMS check alone kept 2-4x distorted data and the HR read 115-125 instead of ~75.
#define TEMPLATE_POINTS       48     // pulse resampled to this many points
#define TEMPLATE_MIN_PULSES   5      // still pulses needed before motion pulses can be judged
#define TEMPLATE_MIN_CORR     0.90f  // shape correlation with the template (1 = identical)
#define TEMPLATE_AMP_MIN      0.6f   // amplitude vs template: outside 0.6..1.6x during motion = distorted
#define TEMPLATE_AMP_MAX      1.6f
#define BODY_HEIGHT_M         1.70f  // for the large-artery stiffness index (fingertip index; exploratory on a limb)
#define BASELINE_PULSES       10     // accepted pulses whose median becomes the reference ("good flow") level
#ifndef AUTO_BASELINE
#define AUTO_BASELINE         1      // 1 (2026-10-13, user): also captured automatically from the first 10 still pulses
                                     // after boot / 'r' / probe re-placed. Every ESP32 restart (opening the serial port
                                     // with Teleplot / the Serial Monitor resets a DevKit) erased the reference and the
                                     // LEDs stayed off until 'b'. CAUTION: put the probe on with the limb UNRESTRICTED -
                                     // a reference taken while restricted would silently become "normal".
                                     // 0: only command 'b' captures the reference.
#endif
#define AUTO_ARM_SETTLE_MS    4000   // AUTO_BASELINE: capture starts once the probe is on and the LED current has not
                                     // changed for this long (the AGC settles first, then the current is locked)
#define FLOW_GOOD_RATIO       0.70f  // LEGACY report heuristic (m10 flow status), uncalibrated: flow score >= this -> GOOD
#define FLOW_POOR_RATIO       0.30f  // flow score <  this  -> POOR ; in between -> REDUCED. Not used for x / the LEDs.
#define FLOW_CONFIRM_PULSES   3      // status changes only after this many pulses agree
#define PULSE_TIMEOUT_S       8.0f   // no accepted pulse for this long (signal otherwise fine) -> NO PULSE.
                                     // >= SQI window (5 s) + 2-3 beats, so a short motion episode does not trigger it

// =====================================================================
// 8b. RELATIVE OPTICAL PULSE INDEX x -> 10 LEDs + Teleplot (m14_flow_level.h, PULSE_INDEX_DESIGN.md)
// =====================================================================
// x = smoothed clamp(PI / reference PI, 0, 1), one update per NEW accepted still pulse. It is a relative optical
// pulse index, NOT a blood-flow percentage. level = min(10, 1 + floor(10 x)); LEDs 1..level lit -> MORE LEDs =
// STRONGER relative pulsatility. level 0 = UNKNOWN (no reference, probe off, invalid data, no fresh pulse ...):
// all 10 LEDs OFF (no blinking). The LEDs always show exactly the BloodFlowLevel value sent to Teleplot.
// Each LED: GPIO -> 330 ohm -> LED anode, cathode -> GND.
// Pins as wired (2026-10-12). GPIO 2 and 15 are boot strapping pins: an LED + resistor to GND keeps them LOW at
// boot, which is allowed (GPIO 2 low = normal/flash boot OK, GPIO 15 low = boot log off) - never pull them HIGH.
// NOTE: GPIO 32, 33, 25, 26 are also the reserved NIRS pins (section 1) - move those LEDs if NIRS is added.
#ifndef USE_FLOW_LEDS
#define USE_FLOW_LEDS         1
#endif
// LED 1 = BOTTOM (GPIO 19, lit at the lowest x) .. LED 10 = TOP (GPIO 32). Bar: level 1 = 19, level 2 = 19+2, ...
// level 10 = all ten.
static const uint8_t FLOW_LED_PINS[10] = {19,2,15,13,14,27,26,25,33,32};   // LED 1 (bottom) .. LED 10 (top)
#define FLOW_LED_ACTIVE_HIGH  1
#define FL_LED_BAR            0      // LEDs 1..level lit (bar graph)
#define FL_LED_SINGLE         1      // only LED 'level' lit
#define FL_LED_MODE           FL_LED_BAR
// Nominal level edges on x: 0.00 <= x < 0.10 -> level 1, 0.10 <= x < 0.20 -> level 2, ..., 0.90 <= x <= 1 -> level 10
static const float FL_LEVEL_X[9] = {0.10f, 0.20f, 0.30f, 0.40f, 0.50f, 0.60f, 0.70f, 0.80f, 0.90f};
#define FL_HYST               0.02f  // level hysteresis (x units): a level is left only 0.02 beyond its band edges
#ifndef FL_X_KEEP
#define FL_X_KEEP             0.75f  // x = FL_X_KEEP * previous x + (1 - FL_X_KEEP) * target, once per NEW pulse
#endif
#define FL_PULSE_TIMEOUT_S    5.0f   // newest accepted pulse ENDED longer ago than this -> UNKNOWN (never "zero").
                                     // Measured in sample time, so it includes the ~1-beat analysis delay
#define PROBE_OFF_CLEAR_MS    1000   // no tissue this long -> probe removed: reference cleared. A shorter lift
                                     // (motion) is invalid data only, so the reference survives it
#define FL_ACQ_TIMEOUT_MS     1000   // no sample processed this long (wall clock) -> acquisition stopped -> UNKNOWN
#define FL_INVALID_GRACE_S    1.0f   // clipping / warm-up / motion / pressure change lasting this long -> UNKNOWN.
                                     // Shorter blips only pause x (no update) - debounce so tremor flags do not
                                     // flash the LEDs; probe-off / sensor / stopped acquisition act at once
#ifndef FL_USE_SMALL_MOTION
#define FL_USE_SMALL_MOTION   1      // 1 (2026-10-13, user): SMALL motion does not invalidate x - output during small motion.
                                     // 0: small motion is invalid too (strict rule; LEDs went off with finger tremor)
#endif
#define FL_SUPPORT_RATIO      1.5f   // area-based vs PI-based strength (median of 3 pulses) outside 1/1.5 .. 1.5 ->
                                     // "DISAGREE" is reported (x is NOT changed). Engineering threshold, not calibrated
// diagnostics (reported only - NOT part of x)
#define FL_RATE_WINDOW_S      10     // pulseDetectionRate over the last 10 s ...
#define FL_RATE_MIN_S         3.0f   // ... needing at least 3 s of eligible (still, valid, unclipped) time
#define FL_CV_PULSES          10     // amplitudeCV over the last 10 accepted still pulses (at least 3)
// Teleplot
#define FL_TP_LEVEL_MS        100    // BloodFlowLevel sent every 100 ms: its steps stay sharp in the chart
#define FL_TP_SLOW_MS         1000   // the other index / diagnostic channels once per second
#ifndef TELEPLOT_SUPPORT
#define TELEPLOT_SUPPORT      0      // 1: also send relativeDC, relativeAreaPI, relativeSlopePI, hrRatio, areaVsPI
#endif                               // (exploratory panels; off so the default view keeps the agreed 16 channels)

// ---- OPTIONAL modes from the 2026-10-13 live session (they depart from the strict estimator above). ON now:
//      FL_HOLD_ON_INVALID (and FL_USE_SMALL_MOTION above) - the strict rules blanked the LEDs during small motion ----
#ifndef FL_TWO_POINT_CAL
#define FL_TWO_POINT_CAL      0      // 1: commands t / f capture the tight / free value of the uncapped strength
#endif                               //    (stored in flash); levels 2..9 = 8 equal bands between them
#define FL_CAL_PULSES         5      // pulses averaged per calibration point
#define FL_CAL_MIN_SPAN       0.05f  // xFree must exceed xTight by this much, else the calibration is not used
#define FL_CAL_EPS            0.005f // anything this far above xTight already leaves level 1
#define FL_STRENGTH_MAX       3.0f   // cap of the uncapped "strength" used with the calibration (3x the reference)
#ifndef FL_HOLD_ON_INVALID
#define FL_HOLD_ON_INVALID    1      // 1 (2026-10-13, user): invalid data / stale pulses hold the last level up to FL_HOLD_MAX_S
#endif                               //    (no LED change) instead of UNKNOWN
#define FL_HOLD_MAX_S         20.0f
#ifndef FL_NO_PULSE_IS_LOW
#define FL_NO_PULSE_IS_LOW    1      // 1 (2026-10-13, user): stale AND the PPG RMS below FL_WEAK_RMS_FRAC x the reference amplitude for
#endif                               //    FL_PULSE_TIMEOUT_S -> level 1 (red LED only), x = 0: "no detectable pulse, only noise"
                                     //    (a heuristic display choice, not a measured zero flow). 0: UNKNOWN (all off)
#define FL_WEAK_RMS_FRAC      0.10f
#ifndef FL_MEDIAN3
#define FL_MEDIAN3            0      // 1: target = median of the last 3 relativePI values (one odd beat ignored)
#endif

// =====================================================================
// 9. SpO2 (raw RED + IR, Maxim algorithm)
// =====================================================================
#define SPO2_DECIMATION       5                          // 125 Hz -> 25 Hz
#define SPO2_FS               (PPG_FS / SPO2_DECIMATION) // = 25 Hz: the rate the Maxim algorithm is built for
#define SPO2_WINDOW_S         4                          // 100 samples at 25 Hz
#define SPO2_UPDATE_S         1                          // sliding window, new estimate every second
#define SPO2_LOWPASS_HZ       6.0f                       // anti-alias filter before decimation
#define SPO2_MIN_CORRELATION  0.80f                      // RED/IR correlation gate (motion -> low correlation)
#define SPO2_MEDIAN_N         5                          // reported value = median of last N valid estimates

// =====================================================================
// 10. OUTPUT
// =====================================================================
// REPORT : one "Name: value unit" block per accepted pulse + status every second (default)
// PLOTTER: one "name:value,name:value" line per sample for the Arduino Serial Plotter
// TELEPLOT: ">name:value" lines per sample for the Teleplot VS Code extension (+ ">HR:" per pulse)
#define OUTPUT_MODE_REPORT    0
#define OUTPUT_MODE_PLOTTER   1
#define OUTPUT_MODE_TELEPLOT  2
#define OUTPUT_MODE_RAW       3      // 'w': "R,ir,red,fx,fy,fz,bx,by,bz,ppg,preAnc" per sample (recording)
#ifndef OUTPUT_MODE
#define OUTPUT_MODE           OUTPUT_MODE_TELEPLOT
#endif
#define PLOT_EVERY_N          1      // plotter/teleplot: send every Nth sample (5 -> 25 Hz)
#define STATUS_PERIOD_MS      1000
#ifndef REPORT_FIDUCIALS
#define REPORT_FIDUCIALS      0      // 1: also print fiducial indices (onset, peak, notch, a..e) per pulse
#endif
#ifndef DEBUG_STREAM_RAW
#define DEBUG_STREAM_RAW      0      // 1: stream raw IR, RED + 6 accel axes (for offline ANC tests)
#endif

// =====================================================================
// 11. WI-FI -> WEBSITE (tourniquet-monitor server)
// =====================================================================
// The ESP32 creates Wi-Fi WIFI_AP_SSID; join it from the server laptop and start the server there
// (npm start). Data goes to POST http://<laptop>:SERVER_PORT/api/telemetry/hardware-feed.
// Device key + patient ID: device_secrets.h (copy device_secrets.example.h).
#ifndef USE_WIFI
#define USE_WIFI              0      // 1: send to the website. Off by default: its current bursts (beacon every
                                     // 102.4 ms) ripple the supply and put a 3.8 Hz tone into the PPG (85 % of the
                                     // signal at high LED current). With Wi-Fi on, power the sensor from a clean 3.3 V
                                     // (100 uF + 100 nF at the MAX30102 VIN) or lower AGC_MAX_CODE.
#endif
#define WIFI_AP_SSID          "AERO-CUFF-ESP32"
#define WIFI_AP_PASSWORD      "AeroCuffTest42"   // at least 8 characters
#define WIFI_DEVICE_NAME      "ESP32-PPG"
#define SERVER_IP             ""     // "" = use the laptop that joined the ESP32 Wi-Fi; or e.g. "192.168.4.2"
#define SERVER_PORT           3000   // PORT in tourniquet-monitor/.env
#define WIFI_SEND_PERIOD_MS   1000   // one batch per second
#define WIFI_PPG_DECIMATE     2      // waveform to the website at 125/2 = 62.5 Hz (~62 samples per batch)

// =====================================================================
// sanity checks
// =====================================================================
static_assert(PPG_FS == 125, "MAX30102 is configured for 125 Hz (1000 sps / 8). Change sensor setup + SpO2 decimation together.");
static_assert(SPO2_FS * SPO2_DECIMATION == PPG_FS, "PPG_FS must be a multiple of SPO2_DECIMATION");
static_assert(SPO2_FS == 25, "Maxim SpO2 algorithm constants are for 25 Hz");
static_assert(NUM_ACCEL >= 0 && NUM_ACCEL <= 2, "NUM_ACCEL must be 0..2");
static_assert(!USE_ANC || NUM_ACCEL > 0, "ANC needs at least one accelerometer");
static_assert(USE_AMP_REGULARIZE == 0, "Amplitude regularisation destroys amplitude/PI/AUC; keep it off for flow assessment");
static_assert((HAMPEL_WINDOW % 2) == 1 && HAMPEL_WINDOW >= 3 && HAMPEL_WINDOW <= 15, "HAMPEL_WINDOW must be odd 3..15");
#if NIRS_MODE > 0 && !defined(NIRS_LED1_PIN)
  #error "NIRS_MODE > 0: uncomment the NIRS pins (section 1) and settings (section 5) first"
#endif
#if USE_FSR && !defined(FSR1_PIN)
  #error "USE_FSR 1: uncomment FSR1_PIN / FSR2_PIN (section 1) and the FSR settings (section 4) first"
#endif
#if NUM_ACCEL >= 2 && !defined(BAND_ACCEL_ADDR)
  #error "NUM_ACCEL 2: uncomment the band ADXL345 block (section 3) first"
#endif
static_assert(FLOW_POOR_RATIO < FLOW_GOOD_RATIO, "flow thresholds");

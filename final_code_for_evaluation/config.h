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
// CURRENT HARDWARE: classic ESP32 (ESP32 Dev Module) + MAX30102 + ONE ADXL345
//   MAX30102 : SDA -> GPIO 22, SCL -> GPIO 23, VIN 3.3 V, GND               (I2C 0x57)
//   ADXL345  : SDA -> GPIO 22, SCL -> GPIO 23, CS -> 3.3 V (= I2C mode),
//              SDO/ALT ADDRESS ("AD0") -> GND  => address 0x53            (I2C 0x53)
// Nothing else is connected: no FSR, no second accelerometer, no NIRS -> no other GPIO is used.
#define BOARD_NAME            "ESP32"
#define I2C_SDA               22
#define I2C_SCL               23
#define I2C_CLOCK_HZ          400000
#define SERIAL_BAUD           115200 // set the Serial Monitor to this
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
#define MAX_LED_CURRENT_CODE  0x7F   // START current, code x 0.2 mA: 0x7F = 25.4 mA (was 0x24 = 7.2 mA, too dim
                                     // for the wrist). With USE_LED_AGC the current is then adjusted automatically.
// Automatic LED current control (led_agc.h): keeps the raw DC of RED and IR near AGC_TARGET of full scale,
// i.e. the brightest light that does not clip -> largest pulse amplitude, lowest relative noise.
#ifndef USE_LED_AGC
#define USE_LED_AGC           1
#endif
#define AGC_TARGET            0.60f  // aim: 60 % of the 18-bit range. Wrist A-B test (ADC range 16384 nA):
                                     // sensor noise 227 -> 75 ppm and pulse SNR 8 -> 12 dB from 9 mA to 35 mA
#define AGC_LOW               0.40f  // re-adjust only outside 40..85 % (each change restarts the filters).
                                     // Was 25 %: the LEDs stayed dimmer than they could be.
#define AGC_HIGH              0.85f
#define AGC_MIN_CODE          0x05   // 1 mA
#define AGC_MAX_CODE          0xAF   // 35 mA: best pulse SNR in the wrist A-B test. 51 mA (0xFF, datasheet max)
                                     // loads the supply so much that Wi-Fi current bursts modulate the LEDs
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
// 3. ACCELEROMETER (1x ADXL345) - motion gating (pulses during motion are rejected)
// =====================================================================
#ifndef NUM_ACCEL
#define NUM_ACCEL             1      // ONE ADXL345 fitted now (0 = none, 2 = add the band sensor below)
#endif
#define ADXL345_FINGER_ADDR   0x53   // SDO -> GND
#define ADXL345_ADDR          ADXL345_FINGER_ADDR   // legacy name
#define MOTION_MAX_ENERGY     0.5f   // (m/s^2)^2 dynamic acceleration energy: above = "moving", pulses rejected
#define MOTION_HOLD_SAMPLES   (PPG_FS / 2)   // stay "moving" for 0.5 s after the last motion sample

// ---- NOT USED NOW: second (band) ADXL345 - set NUM_ACCEL 2 and uncomment ----
// #define ADXL345_BAND_ADDR  0x1D   // SDO -> 3V3, CS -> 3V3
// static const int   BAND_AXIS_IDX[3] = {0, 1, 2};    // band axes mapped into the finger frame
// static const float BAND_AXIS_SGN[3] = {1.f, 1.f, 1.f};

// Motion cancellation (NLMS, m06 + m06b). OFF: enable only after recordings show that motion really
// corrupts the PPG. Works with the one ADXL345 (use ANC_MODE_ABS6); REL9 needs the band sensor.
#ifndef USE_ANC
#define USE_ANC               0
#endif
#define ANC_MODE_ABS6         0      // finger xyz (+ band xyz if fitted), gravity removed
#define ANC_MODE_REL9         1      // finger acc, RELATIVE acc (finger-band), RELATIVE velocity (2 sensors)
#ifndef ANC_MODE
#define ANC_MODE              ANC_MODE_ABS6
#endif
#if ANC_MODE == ANC_MODE_REL9
  #define ANC_NUM_REFS        9
#else
  #define ANC_NUM_REFS        6
#endif
#define ACC_RAW_CH            6      // finger xyz, band xyz (band = 0 when not fitted)
#define ANC_TAPS              8
#define ANC_MU                0.01f
#define ANC_P_MIN             0.05f
#define ANC_LEAK              0.0f
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
#define SQI_MIN_SKEW          0.0f   // fingertip ~0.3; limb pulses are more symmetric
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
#define ARTIFACT_MAX_S        3.0f   // a level lasting > 3 s is accepted as the new normal
#define BODY_HEIGHT_M         1.70f  // for the large-artery stiffness index (fingertip index; exploratory on a limb)
#define BASELINE_PULSES       10     // accepted pulses whose median becomes the reference ("good flow") level
#ifndef AUTO_BASELINE
#define AUTO_BASELINE         1      // 1: capture the baseline automatically after boot, after 'r' and when the probe is placed back
#endif
#define FLOW_GOOD_RATIO       0.70f  // flow score >= this  -> GOOD
#define FLOW_POOR_RATIO       0.30f  // flow score <  this  -> POOR ; in between -> REDUCED
#define FLOW_CONFIRM_PULSES   3      // status changes only after this many pulses agree
#define PULSE_TIMEOUT_S       8.0f   // no accepted pulse for this long (signal otherwise fine) -> NO PULSE.
                                     // >= SQI window (5 s) + 2-3 beats, so a short motion episode does not trigger it

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
#if NUM_ACCEL >= 2 && !defined(ADXL345_BAND_ADDR)
  #error "NUM_ACCEL 2: uncomment the band ADXL345 block (section 3) first"
#endif
static_assert(FLOW_POOR_RATIO < FLOW_GOOD_RATIO, "flow thresholds");

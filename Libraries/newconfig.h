// ===================== USER SETTINGS =====================
#pragma once

// --- Hardware: ESP32-S3 N16R8 + MAX30102 + 2x ADXL345 + 2x FSR -----------
#define SENSOR_ANALOG    1
#define SENSOR_MAX30102  2
#define SENSOR_TYPE      SENSOR_MAX30102

#define PPG_FS           125    // Hz. MUST equal the fs used in tools/gen_coeffs.py
#define PPG_ADC_PIN      6      // only used in SENSOR_ANALOG mode (moved from 4: GPIO4/5 are now the FSRs)
#define SERIAL_BAUD      460800 // ignored on native USB (USB-CDC) but harmless

// I2C bus shared by MAX30102 (0x57) and two ADXL345 (0x53 and 0x1D).
// GPIO 35/36/37 are used internally by the N16R8 octal PSRAM - never use them.
#define I2C_SDA          22
#define I2C_SCL          23
#define I2C_CLOCK_HZ     400000

// MAX30102 IR is a REFLECTED-light signal: it goes DOWN when blood volume goes UP
// (the paper: PPG is obtained "by inverting the light intensity", Fig. 3).
// 1 = flip the sign so the systolic peak points up. Verify by plotting.
#define INVERT_SIGNAL    1

// --- Motion-artifact cancellation with 2 accelerometers ------------------
// Two ADXL345 on the same I2C bus, told apart by the SDO/ALT ADDRESS pin:
//   SDO -> GND  : address 0x53   (FINGER sensor, next to the MAX30102)
//   SDO -> 3V3  : address 0x1D   (BAND sensor, on the wrist/hand band)
// Never leave SDO floating. CS must be tied to 3V3 on both boards for I2C mode.
#define ADXL345_FINGER_ADDR 0x53
#define ADXL345_BAND_ADDR   0x1D
#define ADXL345_ADDR        ADXL345_FINGER_ADDR   // legacy name (posture sensor in the old sketch)

// NUM_ACCEL = 2 -> motion cancellation (m06/m06b) is on.
// NUM_ACCEL = 1 -> only the finger ADXL345 is fitted: cancellation is off.
#define NUM_ACCEL        2
#define USE_ANC          (NUM_ACCEL >= 2)

// ADXL345 settings (apply in the sketch when initialising each sensor)
#define ACC_RANGE_G      4      // +/-4 g, as in the test sketches. +/-2 g gives finer resolution if motion allows
#define ACC_ODR_HZ       200    // output data rate; bandwidth = ODR/2 = 100 Hz, which also acts as an anti-alias filter
                                // Read BOTH sensors in the same loop pass as each PPG sample so the timestamps align.

// Axis mapping of the BAND sensor into the FINGER sensor's frame:
//   band_in_finger_frame[i] = BAND_AXIS_SGN[i] * band_raw[BAND_AXIS_IDX[i]]
// Mount both boards with parallel axes if you can. Check with DEBUG_STREAM_RAW (see README).
static const int   BAND_AXIS_IDX[3] = {0, 1, 2};
static const float BAND_AXIS_SGN[3] = {1.f, 1.f, 1.f};

// ANC reference modes
#define ANC_MODE_ABS6    0      // 6 refs: finger xyz + band xyz (gravity removed)
#define ANC_MODE_REL9    1      // 9 refs: finger acc xyz, RELATIVE acc xyz (finger-band), RELATIVE velocity xyz
#ifndef ANC_MODE
#define ANC_MODE         ANC_MODE_REL9
#endif
#if ANC_MODE == ANC_MODE_REL9
  #define ANC_NUM_REFS   9
#else
  #define ANC_NUM_REFS   6
#endif
#define ACC_RAW_CH       6      // what the sketch passes to the pipeline: finger xyz, band xyz (m/s^2)
#define ANC_TAPS         8      // delay taps per reference
#define ANC_MU           0.05f
#define ACC_VEL_LEAK_HZ  0.3f   // leaky integrator corner for relative velocity (avoids drift)
#define ACC_VEL_SCALE    12.0f  // scale velocity to a similar magnitude as acceleration (helps NLMS converge)
#define DEBUG_STREAM_RAW 0      // 1: stream raw IR + 6 accel axes instead of the processed signal (for offline tests)

// --- Contact-force sensing: 2x FSR (ESP32 ADC) ----------------------------
// Wiring: FSR in a voltage divider -> unity-gain op-amp buffer -> RC low-pass (~10-20 Hz) -> pin.
// GPIO4 and GPIO5 are ADC1 channels on the ESP32-S3 (ADC2 is unusable while Wi-Fi is on).
// NOTE: the pipeline does not use force yet - the sketch logs/streams it for now.
#define FSR_COUNT        2
#define FSR1_PIN         4
#define FSR2_PIN         5
#define FSR_ADC_BITS     12     // analogReadResolution(12), as in fsr.cpp
#define FSR_OVERSAMPLE   8      // reads averaged per PPG sample (software averaging)

// --- Timing --------------------------------------------------
#define WARMUP_SECONDS   5      // filter settling; samples before this are flagged invalid

// --- Smoothing windows in ms (same idea as pyPPG sm_wins) ----
#define SM_PPG_MS        50
#define SM_VPG_MS        10
#define SM_APG_MS        10
#define SM_JPG_MS        10

// --- SQI (window / hop in seconds) ---------------------------
#define SQI_WINDOW_S     5
#define SQI_HOP_S        1
// Starting thresholds only - TUNE on your own sensor/data.
#define SQI_MIN_PERIODICITY 0.50f   // normalised autocorrelation peak, 0..1
#define SQI_MIN_SKEW        0.30f   // PPG with sharp upstroke has positive skew. If your sensor output is
                                    // INVERTED the sign flips - invert the signal or set to -99 to disable.
#define SQI_KURT_MIN        1.5f
#define SQI_KURT_MAX        8.0f
#define SQI_MIN_ENTROPY     1.5f    // Shannon entropy of amplitude histogram (bits); low = flat/saturated
#define SQI_MAX_ENTROPY     4.0f

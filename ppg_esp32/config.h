// ===================== USER SETTINGS =====================
#pragma once

// --- Hardware: ESP32-S3 N16R8 + MAX30102 + 2 accelerometers -------------
#define SENSOR_ANALOG    1
#define SENSOR_MAX30102  2
#define SENSOR_TYPE      SENSOR_MAX30102

#define PPG_FS           125    // Hz. MUST equal the fs used in tools/gen_coeffs.py
#define PPG_ADC_PIN      4      // only used in SENSOR_ANALOG mode
#define SERIAL_BAUD      460800 // ignored on native USB (USB-CDC) but harmless

// I2C bus shared by MAX30102 (0x57), ADXL345 (0x53) and MPU6050 (0x68).
// GPIO 35/36/37 are used internally by the N16R8 octal PSRAM - never use them.
#define I2C_SDA          22
#define I2C_SCL          23
#define I2C_CLOCK_HZ     400000

// MAX30102 IR is a REFLECTED-light signal: it goes DOWN when blood volume goes UP
// (the paper: PPG is obtained "by inverting the light intensity", Fig. 3).
// 1 = flip the sign so the systolic peak points up. Verify by plotting.
#define INVERT_SIGNAL    1

// --- Motion-artifact cancellation with 2 accelerometers ------------------
// NOTE: Disabled because the current hardware only has a single ADXL345.
// Set to 1 if a second accelerometer (MPU6050) is added.
#define USE_ANC          0
#define MPU6050_ADDR     0x68   // AD0 -> GND (only needed if USE_ANC=1)
#define ADXL345_ADDR     0x53   // SDO/ALT ADDRESS -> GND (use 0x1D if tied to 3V3)
#define ACC_FINGER_IS_MPU 1     // 1: MPU6050 is on the finger/probe, ADXL345 on the band/wrist
                                // 0: the other way round.  "finger" = the sensor closest to the MAX30102.

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

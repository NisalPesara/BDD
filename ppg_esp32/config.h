#pragma once
#include <stdint.h>

// Target: ESP32-S3 DevKitC-1 N16R8. Verify your actual module before wiring.
#define PPG_FS 125
#define PPG_PERIOD_US (1000000UL / PPG_FS)
#define SERIAL_BAUD 460800
#define I2C_SDA 8
#define I2C_SCL 9
#define I2C_CLOCK_HZ 400000
#define MAX30102_ADDR 0x57
// 1000 conversions/s, average 8 => 125 FIFO pairs/s. 118 us is legal at 1000 Hz.
#define MAX_LED_CURRENT_CODE 100 // register code, approximately 20 mA; NOT 100 mA
#define MAX_SPO2_CONFIG 0x35     // ADC range 4096 nA, 1000 Hz, 118 us / 16-bit conversion
#define MAX_FIFO_CONFIG 0x60    // average 8, rollover disabled
#define MAX_RAW_FULL_SCALE 262143.0f // FIFO left-justifies all resolutions to bit 17
#ifndef INVERT_SIGNAL
#define INVERT_SIGNAL 1         // verify pulse polarity on your limb recording
#endif
#define MIN_OPTICAL_COUNTS 1000.0f // presence floor only; tune with covered/uncovered sensor

#ifndef NUM_ACCEL
#define NUM_ACCEL 2             // 0, 1, or 2 fitted ADXL345s
#endif
#ifndef USE_ANC
#define USE_ANC 0               // record references first; enable only after comparison
#endif
#define ADXL345_PROBE_ADDR 0x53
#define ADXL345_REFERENCE_ADDR 0x1D
static const int REFERENCE_AXIS_IDX[3] = {0, 1, 2};
static const float REFERENCE_AXIS_SIGN[3] = {1.f, 1.f, 1.f};
#define ANC_NUM_REFS 6
#define ANC_TAPS 8
#define ANC_MU 0.01f
#define ANC_MIN_ENERGY 0.05f     // placeholder in sum of squared (m/s^2) references
#define MOTION_MAX_ENERGY 0.5f   // per-sample dynamic acceleration energy; tune at rest
#define ACC_MAX_AGE_US 12000UL

#ifndef USE_FSR
#define USE_FSR 1
#endif
#define FSR1_PIN 4
#define FSR2_PIN 5
#define FSR_ADC_BITS 12
#define FSR_SUPPLY_MV 3300.0f
#define FSR_FIXED_OHM 10000.0f
// Both supplied FSRs measure BAND pressure. Do not mistake low band pressure for
// loss of optical-probe contact. Set 0 or 1 ONLY if that FSR is at the PPG probe.
#ifndef CONTACT_FSR_INDEX
#define CONTACT_FSR_INDEX -1
#endif
#define CONTACT_ON_US 5.0f
#define CONTACT_OFF_US 3.0f
#define CONTACT_HIGH_US 500.0f
#define CONTACT_SLOPE_MAX_US_S 100.0f
#define FORCE_STABLE_SAMPLES (PPG_FS / 2)

// NIRS hardware was not identified. Mode 1 logs two independently conditioned,
// continuous ANALOG AFE outputs; it does not drive LEDs or calculate StO2.
#ifndef NIRS_MODE
#define NIRS_MODE 0             // 0 disabled, 1 two analog AFE channels
#endif
#define NIRS_ADC_1_PIN 1
#define NIRS_ADC_2_PIN 2
#define NIRS_ENABLE_PIN 10      // reserved; not driven without a known AFE
#define PPG_ADC_PIN 6           // reserved for a future analog PPG adapter
#define NIRS_LOG_HZ 10

#define WARMUP_SECONDS 6
#define PPG_HIGHPASS_HZ 0.15f
#define PPG_LOWPASS_HZ 10.0f
#define HAMPEL_WINDOW 5
#ifndef SG_HALF
#define SG_HALF 7               // 15 samples = 120 ms at 125 Hz; 9 => 152 ms
#endif
#define SG_ORDER 3
#define SQI_WINDOW_S 5
#define SQI_HOP_S 1
#define SQI_MIN_SKEW 0.0f
#define MIN_BPM 40.0f
#define MAX_BPM 180.0f
#define PULSE_TIMEOUT_US 3000000ULL
#define BASELINE_PULSES 10
#define REDUCED_TREND_RATIO 0.5f // research trend threshold, not a clinical limit
#define TREND_CONFIRM_PULSES 3
#define STREAM_EVERY_N 5        // SAMPLE at 25 Hz; set 1 for full 125 Hz recordings

static_assert(PPG_FS == 125, "Update MAX30102 rate and decimator before changing PPG_FS");
static_assert(NUM_ACCEL >= 0 && NUM_ACCEL <= 2, "NUM_ACCEL must be 0..2");
static_assert(!USE_ANC || NUM_ACCEL > 0, "ANC needs an accelerometer reference");
static_assert(!USE_ANC || ANC_MIN_ENERGY > 0, "Configure a motion-energy gate");
static_assert(CONTACT_FSR_INDEX >= -1 && CONTACT_FSR_INDEX <= 1, "Contact FSR index -1, 0, or 1");
static_assert(USE_FSR || CONTACT_FSR_INDEX == -1, "Contact gating requires FSRs");
static_assert(NIRS_MODE == 0 || NIRS_MODE == 1, "NIRS mode must be disabled or analog");
static_assert(STREAM_EVERY_N >= 1 && STREAM_EVERY_N <= PPG_FS, "Invalid stream rate");

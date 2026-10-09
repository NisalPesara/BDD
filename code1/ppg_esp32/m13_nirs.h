// METHOD 13 - NIRS (near-infrared spectroscopy) tissue oxygenation TREND
//
//  Pins (config.h, chosen per board):  NIRS_LED1_PIN / NIRS_LED2_PIN  -> LED drivers (transistor/MOSFET)
//                                       NIRS_PD1_PIN  -> photodiode TIA output, far (deep) detector, ADC1
//                                       NIRS_PD2_PIN  -> near detector (or AFE channel 2), ADC1
//  NIRS_MODE 1 (LED-multiplexed, non-blocking state machine, NIRS_FRAME_HZ frames/s):
//        LED1 on -> settle -> read PD   LED2 on -> settle -> read PD   both off -> settle -> read DARK
//        light = reading - dark  (removes ambient light and photodiode/TIA offset)
//  NIRS_MODE 2: external front end with two continuous outputs (wavelength 1 on PD1, wavelength 2 on PD2).
//
//  Modified Beer-Lambert law (relative to a baseline I0 taken at start / on 'b'):
//     dOD(wl) = log10(I0 / I) = (eHbO2(wl) * dHbO2 + eHHb(wl) * dHHb) * d * DPF
//  solved for dHbO2 and dHHb (uM) from the two wavelengths. dHbT = dHbO2 + dHHb (blood volume),
//  dHbDiff = dHbO2 - dHHb (oxygenation). During arterial occlusion distal to a tourniquet you
//  typically see dHbO2 falling and dHHb rising. These are CHANGES from baseline, not an absolute
//  StO2 (that needs multi-distance spatially resolved hardware + calibration). DPF and the
//  extinction coefficients in config.h are literature assumptions.
#pragma once
#include <math.h>
#include <stdint.h>
#include "config.h"
#ifdef ARDUINO
#include <Arduino.h>
#endif

struct NirsResult {
  bool  fresh = false, valid = false, baselineReady = false;
  float light1 = NAN, light2 = NAN, dark = NAN;   // far detector, dark-subtracted (mV)
  float near1 = NAN, near2 = NAN;                 // near detector, dark-subtracted (mV)
  float dOD1 = NAN, dOD2 = NAN;
  float dHbO2 = NAN, dHHb = NAN, dHbT = NAN, dHbDiff = NAN;   // uM
};

class NirsMonitor {
public:
  NirsResult r;

  void begin() {
    phase_ = IDLE; frameStartUs_ = 0; started_ = false;
    captureBaseline();
#if NIRS_MODE == 1 && defined(ARDUINO)
    pinMode(NIRS_LED1_PIN, OUTPUT); pinMode(NIRS_LED2_PIN, OUTPUT);
    led(NIRS_LED1_PIN, false); led(NIRS_LED2_PIN, false);
#endif
#if NIRS_MODE >= 1 && defined(ARDUINO)
    analogSetPinAttenuation(NIRS_PD1_PIN, ADC_11db);
    analogSetPinAttenuation(NIRS_PD2_PIN, ADC_11db);
#endif
  }

  void captureBaseline() { n0_ = 0; s01_ = s02_ = 0; r.baselineReady = false; r.dHbO2 = r.dHHb = r.dHbT = r.dHbDiff = NAN; }

  // Call as often as possible (every loop pass). Never blocks longer than one ADC burst.
  void update(uint32_t nowUs) {
    r.fresh = false;
#if NIRS_MODE == 0
    (void)nowUs;
#else
    const uint32_t period = 1000000UL / NIRS_FRAME_HZ;
    if (!started_) { frameStartUs_ = nowUs - period; started_ = true; }
  #if NIRS_MODE == 2
    if ((uint32_t)(nowUs - frameStartUs_) < period) return;
    frameStartUs_ = nowUs;
    v1_ = readMv(NIRS_PD1_PIN); v2_ = readMv(NIRS_PD2_PIN);
    finish(v1_, v2_, 0.0f, NAN, NAN, 0.0f);
  #else
    switch (phase_) {
      case IDLE:
        if ((uint32_t)(nowUs - frameStartUs_) < period) return;
        frameStartUs_ = nowUs; led(NIRS_LED1_PIN, true); tUs_ = nowUs; phase_ = LED1; return;
      case LED1:
        if ((uint32_t)(nowUs - tUs_) < NIRS_SETTLE_US) return;
        v1_ = readMv(NIRS_PD1_PIN); n1_ = NIRS_HAS_PD2 ? readMv(NIRS_PD2_PIN) : NAN;
        led(NIRS_LED1_PIN, false); led(NIRS_LED2_PIN, true); tUs_ = nowUs; phase_ = LED2; return;
      case LED2:
        if ((uint32_t)(nowUs - tUs_) < NIRS_SETTLE_US) return;
        v2_ = readMv(NIRS_PD1_PIN); n2_ = NIRS_HAS_PD2 ? readMv(NIRS_PD2_PIN) : NAN;
        led(NIRS_LED2_PIN, false); tUs_ = nowUs; phase_ = DARK; return;
      case DARK: {
        if ((uint32_t)(nowUs - tUs_) < NIRS_SETTLE_US) return;
        const float dk = readMv(NIRS_PD1_PIN), dkn = NIRS_HAS_PD2 ? readMv(NIRS_PD2_PIN) : 0.0f;
        phase_ = IDLE;
        finish(v1_, v2_, dk, n1_, n2_, dkn);
        return;
      }
    }
  #endif
#endif
  }

  // Pure maths, also used by the desktop test: dark-subtracted light -> dOD -> dHbO2/dHHb.
  void finish(float raw1, float raw2, float dark, float rawNear1, float rawNear2, float darkNear) {
    r.fresh = true;
    r.dark = dark;
    r.light1 = raw1 - dark; r.light2 = raw2 - dark;
    r.near1 = rawNear1 - darkNear; r.near2 = rawNear2 - darkNear;
    r.valid = isfinite(r.light1) && isfinite(r.light2) &&
              r.light1 >= NIRS_MIN_MV && r.light2 >= NIRS_MIN_MV &&
              raw1 < NIRS_MAX_MV && raw2 < NIRS_MAX_MV;
    if (!r.valid) { r.dOD1 = r.dOD2 = r.dHbO2 = r.dHHb = r.dHbT = r.dHbDiff = NAN; return; }
    if (!r.baselineReady) {
      s01_ += r.light1; s02_ += r.light2;
      if (++n0_ >= NIRS_BASELINE_FRAMES) { i01_ = s01_ / n0_; i02_ = s02_ / n0_; r.baselineReady = true; }
      return;
    }
    r.dOD1 = log10f(i01_ / r.light1);
    r.dOD2 = log10f(i02_ / r.light2);
    const float L = NIRS_SD_DISTANCE_CM * NIRS_DPF;
    const float b1 = r.dOD1 / L, b2 = r.dOD2 / L;                        // mM * (1/(mM cm)) units
    const float det = NIRS_E_HBO2_WL1 * NIRS_E_HHB_WL2 - NIRS_E_HHB_WL1 * NIRS_E_HBO2_WL2;
    if (fabsf(det) < 1e-9f) { r.dHbO2 = r.dHHb = NAN; return; }
    r.dHbO2 = 1000.0f * (b1 * NIRS_E_HHB_WL2 - NIRS_E_HHB_WL1 * b2) / det;   // mM -> uM
    r.dHHb  = 1000.0f * (NIRS_E_HBO2_WL1 * b2 - b1 * NIRS_E_HBO2_WL2) / det;
    r.dHbT = r.dHbO2 + r.dHHb;
    r.dHbDiff = r.dHbO2 - r.dHHb;
  }

private:
  enum Phase : uint8_t { IDLE, LED1, LED2, DARK };
  Phase phase_ = IDLE;
  uint32_t frameStartUs_ = 0, tUs_ = 0;
  bool started_ = false;
  float v1_ = 0, v2_ = 0, n1_ = NAN, n2_ = NAN;
  float s01_ = 0, s02_ = 0, i01_ = 1, i02_ = 1; int n0_ = 0;

  static float readMv(int pin) {
#ifdef ARDUINO
    uint32_t s = 0;
    for (int i = 0; i < NIRS_ADC_OVERSAMPLE; i++) s += analogReadMilliVolts(pin);
    return (float)s / NIRS_ADC_OVERSAMPLE;
#else
    (void)pin; return NAN;
#endif
  }
  static void led(int pin, bool on) {
#ifdef ARDUINO
    digitalWrite(pin, (on == (NIRS_LED_ACTIVE_HIGH != 0)) ? HIGH : LOW);
#else
    (void)pin; (void)on;
#endif
  }
};

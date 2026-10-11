// Automatic LED current control (AGC) for the MAX30102.
//
// A fixed LED current cannot suit every site: on the wrist the pulsatile (AC) part is ~10x smaller
// than on a fingertip, so a low current leaves the pulse buried in ADC/photon noise, while a high
// current saturates the ADC on bright skin or a tight band. This loop keeps the raw DC level of
// each LED (RED, IR independently) near AGC_TARGET of full scale - as much light as possible
// without clipping - which maximises the AC amplitude and therefore the signal-to-noise ratio.
//
// A current change makes a step in the raw signal, so changes are rare by design (a wide
// AGC_LOW..AGC_HIGH dead band) and the sketch restarts the filters after each change.
// Typical behaviour: 1-3 adjustments within ~2 s of putting the sensor on, then none.
#pragma once
#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "sensor_drivers.h"

class LedAgc {
public:
  uint8_t code[2] = {MAX_LED_CURRENT_CODE, MAX_LED_CURRENT_CODE};   // [0] RED (LED1), [1] IR (LED2)
  float   dc[2]   = {0, 0};                                         // last window mean, counts

  void begin() {
    code[0] = code[1] = MAX_LED_CURRENT_CODE;
    sum_[0] = sum_[1] = 0; n_ = 0; settle_ = AGC_SETTLE_SAMPLES;
  }
  static float milliamps(uint8_t c) { return c * 0.2f; }
  bool settling() const { return settle_ > 0; }   // true right after a change (and at start)

  // Call for every raw sample pair. Returns true when a current was changed (restart the filters).
  bool process(ppgio::Max30102& sensor, uint32_t ir, uint32_t red) {
    if (settle_) { settle_--; return false; }      // FIFO still holds samples taken at the old current
    sum_[0] += red; sum_[1] += ir;
    if (++n_ < AGC_WINDOW_SAMPLES) return false;
    bool changed = false;
    for (int i = 0; i < 2; i++) {
      dc[i] = float(sum_[i] / n_);
      const float frac = dc[i] / MAX_RAW_FULL_SCALE;
      if (frac >= AGC_LOW && frac <= AGC_HIGH) continue;
      float want = (frac >= 0.97f) ? code[i] * 0.5f                 // clipped: the ratio is unknown, halve
                                   : code[i] * (AGC_TARGET / fmaxf(frac, 1e-3f));
      want = fminf(want, code[i] * 4.0f);                           // at most x4 per step
      const uint8_t c = uint8_t(constrain(lroundf(want), (long)AGC_MIN_CODE, (long)AGC_MAX_CODE));
      if (c != code[i]) { code[i] = c; changed = true; }
    }
    sum_[0] = sum_[1] = 0; n_ = 0;
    if (changed) {
      sensor.setLedCurrents(code[0], code[1]);
      settle_ = AGC_SETTLE_SAMPLES;
    }
    return changed;
  }

private:
  uint64_t sum_[2] = {0, 0};
  uint32_t n_ = 0, settle_ = 0;
};

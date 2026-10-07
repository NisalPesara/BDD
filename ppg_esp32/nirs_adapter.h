#pragma once
#include <Arduino.h>
#include "config.h"

// Contract: these are TWO externally conditioned continuous analog voltages.
// A multiplexed LED/photodiode circuit needs its own sequencer, dark subtraction,
// settling, calibration and wavelength mapping. This adapter does not infer them.
struct NirsReading {
  bool enabled = false, valid = false;
  uint64_t t_us = 0;
  float ch1_mv = NAN, ch2_mv = NAN;
};
inline NirsReading readNirs(uint64_t t_us) {
  NirsReading r; r.t_us = t_us;
#if NIRS_MODE == 1
  r.enabled = true;
  r.ch1_mv = float(analogReadMilliVolts(NIRS_ADC_1_PIN));
  r.ch2_mv = float(analogReadMilliVolts(NIRS_ADC_2_PIN));
  r.valid = r.ch1_mv > 5 && r.ch2_mv > 5 && r.ch1_mv < 3090 && r.ch2_mv < 3090;
#endif
  return r;
}

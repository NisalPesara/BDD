// METHOD 9 - Area under the pulse curve (AUC), trapezoidal rule
//
//  Why it helps for limb blood flow: amplitude only looks at ONE point (the systolic peak);
//  the area integrates the WHOLE pulse (systolic upstroke + diastolic run-off), so it is less
//  sensitive to a single noisy peak and also drops when the diastolic part flattens out under
//  partial occlusion. Units are signal-units x seconds (a.u.*s) - a relative pulse-volume
//  index, NOT blood volume or flow in mL/min.
//
//  Two tools:
//   PulseAuc::integrate()  - used by wave_detection.h on a finished onset-to-onset segment.
//                            Baseline = straight line from this onset to the next onset, the SAME
//                            line for total, systolic and diastolic area, so sys + dia = total.
//                            Signed: an undershoot below the baseline is kept, not hidden.
//   AucCalculator          - streaming version (sample by sample, resets at each beat). Feed it
//                            the PPG with the pulse trough already subtracted. `positiveOnly`
//                            ignores negative trapezoids (dips below the trough).
#pragma once
#include <math.h>
#include <stdint.h>

struct PulseAuc {
  static float integrate(const float* ppg, int count, float fs, int from, int to) {
    if (!ppg || count < 2 || !(fs > 0) || from < 0 || to >= count || to <= from) return NAN;
    const float slope = (ppg[count - 1] - ppg[0]) / (count - 1);
    double area = 0;
    for (int i = from; i < to; ++i) {
      const double a = ppg[i]     - (ppg[0] + slope * i);
      const double b = ppg[i + 1] - (ppg[0] + slope * (i + 1));
      area += (a + b) * 0.5 / fs;
    }
    return float(area);
  }
};

class AucCalculator {
public:
  bool positiveOnly = true;

  AucCalculator() { reset(); }

  // Reset the accumulator at the start of a new heartbeat
  void reset() { accumulated_area = 0.0f; last_t_ms = 0; last_ppg = 0.0f; is_first_sample = true; }

  // Add one sample (trapezoidal rule). Returns the area so far.
  float process(uint32_t t_ms, float ppg) {
    if (is_first_sample) { last_t_ms = t_ms; last_ppg = ppg; is_first_sample = false; return 0.0f; }
    const float dt = (uint32_t)(t_ms - last_t_ms) / 1000.0f;   // unsigned difference: safe across millis() wrap
    const float area = dt * 0.5f * (ppg + last_ppg);
    if (!positiveOnly || area > 0.0f) accumulated_area += area;
    last_t_ms = t_ms; last_ppg = ppg;
    return accumulated_area;
  }

  float get_auc() const { return accumulated_area; }

private:
  float accumulated_area;
  uint32_t last_t_ms;
  float last_ppg;
  bool is_first_sample;
};

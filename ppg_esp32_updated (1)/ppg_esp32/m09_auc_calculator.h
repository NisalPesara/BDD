#pragma once
#include <math.h>

// Pulse AUC in ADC-count seconds. Not volume, velocity, or flow in mL/min.
// Baseline is the SAME straight line between the two pulse onsets for every
// sub-area. The segment must already have passed contact, gap and SQI checks.
struct PulseAuc {
  static float integrate(const float* ppg, int count, float fs, int from, int to) {
    if (!ppg || count < 2 || !(fs > 0) || from < 0 || to >= count || to <= from) return NAN;
    const float slope = (ppg[count - 1] - ppg[0]) / (count - 1);
    double area = 0;
    for (int i = from; i < to; ++i) {
      const double a = ppg[i] - (ppg[0] + slope * i);
      const double b = ppg[i + 1] - (ppg[0] + slope * (i + 1));
      area += (a + b) * 0.5 / fs;
    }
    return float(area); // signed: do not hide undershoot by clipping each trapezoid
  }
};

// METHOD 2 - Baseline-wander removal (Park 2022, "Baseline Wandering")
//  (a) direct removal by high-pass  -> use FM_HP_JOSEPH in m01 (0.15 Hz, Joseph 2014)
//  (b) indirect: estimate baseline and subtract (Timimi 2017 / interpolation idea).
//      Real-time version: baseline = smoothed lower envelope (running minimum
//      over ~1.2 s, then low-passed). Cubic-spline / wavelet+LMS need the whole
//      record (non-causal) and are not suited to a microcontroller stream.
#pragma once
#include <math.h>

struct LowerEnvelopeBaseline {
  static const int MAXW = 256;
  float buf[MAXW];
  int win = 150, idx = 0, filled = 0;
  float base = 0; bool init = false;
  float alpha = 0.02f;                       // baseline smoothing
  void begin(int fs, float windowSec = 1.2f) {
    win = (int)(windowSec * fs); if (win > MAXW) win = MAXW; if (win < 2) win = 2;
    idx = filled = 0; init = false;
  }
  float process(float x) {
    buf[idx] = x; idx = (idx + 1) % win; if (filled < win) filled++;
    float mn = buf[0];
    for (int i = 1; i < filled; i++) if (buf[i] < mn) mn = buf[i];
    if (!init) { base = mn; init = true; } else base += alpha * (mn - base);
    return x - base;
  }
};

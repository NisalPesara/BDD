// METHOD 4 - Moving-average smoothing (pyPPG sm_wins idea)
//
//  ms = the length of the averaging window in TIME. It is converted to samples with
//       w = round(ms * fs / 1000)       e.g. fs = 125 Hz (dt = 8 ms):
//          40 ms ->  5 samples  first spectral null 25.0 Hz, delay 2 samples (16 ms)
//          80 ms -> 10 samples  first null 12.5 Hz,          delay 4.5 samples (36 ms)
//         120 ms -> 15 samples  first null  8.3 Hz,          delay 7 samples (56 ms)
//         200 ms -> 25 samples  first null  5.0 Hz,          delay 12 samples (96 ms)
//  A pulse at 60-120 bpm has useful harmonics up to ~8-10 Hz (the dicrotic notch lives there).
//  A 120-200 ms box already flattens the notch and lowers the systolic peak, so it distorts
//  peak/valley timing. 40 ms (odd, 5 samples) removes sample-to-sample noise left after the
//  10 Hz low-pass with almost no distortion -> recommended (SM_PPG_MS in config.h).
//  The default pipeline uses Savitzky-Golay (ppg_savgol.h) instead, which smooths like a
//  ~120 ms window but keeps peak height; this moving average is the simple alternative.
//
//  Running sum: O(1) per sample; an exact re-sum once per lap removes float drift.
#pragma once
#include <math.h>

struct MovingAverage {
  static const int MAXW = 64;

  // Returns the window length actually used (clamped to 1..MAXW).
  int begin(int fs, int ms) {
    long n = lroundf(ms * fs / 1000.0f);
    w = (n < 1) ? 1 : (n > MAXW ? MAXW : (int)n);
    reset();
    return w;
  }
  void reset() { idx = filled = 0; sum = 0.0f; for (int i = 0; i < MAXW; i++) buf[i] = 0.0f; }

  float process(float x) {
    if (filled < w) { sum += x; ++filled; }
    else            { sum += x - buf[idx]; }
    buf[idx] = x;
    if (++idx >= w) {                       // one lap done: exact re-sum (bounds rounding drift)
      idx = 0;
      float s = 0.0f;
      for (int i = 0; i < filled; i++) s += buf[i];
      sum = s;
    }
    return sum / filled;
  }

  int   window() const { return w; }
  float delaySamples() const { return 0.5f * (w - 1); }   // group delay of a causal box filter

  float buf[MAXW];
  float sum = 0.0f;
  int   w = 1, idx = 0, filled = 0;
};

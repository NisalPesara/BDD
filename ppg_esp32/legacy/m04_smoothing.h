// METHOD 4 - Moving-average smoothing (pyPPG sm_wins: ppg 50 ms, vpg/apg/jpg 10 ms)
#pragma once
struct MovingAverage {
  static const int MAXW = 64;
  float buf[MAXW]; int w = 1, idx = 0, filled = 0;
  void begin(int fs, int ms) {
    w = (int)((ms * fs) / 1000.0f + 0.5f); if (w < 1) w = 1; if (w > MAXW) w = MAXW;
    idx = filled = 0; for (int i = 0; i < MAXW; i++) buf[i] = 0;
  }
  float process(float x) {
    buf[idx] = x; idx = (idx + 1) % w; if (filled < w) filled++;
    float s = 0; for (int i = 0; i < filled; i++) s += buf[i];   // window <= 64: cheap, no drift
    return s / filled;
  }
};

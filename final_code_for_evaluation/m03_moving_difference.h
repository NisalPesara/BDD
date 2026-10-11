// METHOD 3 - Moving-difference filter (Canac et al. 2019)
//  y[n] = x[n] - x[n-W]. Sharpens the PPG up-slope and suppresses slow drift.
//  Used to make systolic-peak / onset detection easier. Pipeline exposes it as a
//  SEPARATE output ("upslope") so it never distorts the clean PPG waveform.
#pragma once
struct MovingDifference {
  static const int MAXW = 32;
  float buf[MAXW]; int w = 4, idx = 0;
  void begin(int windowSamples) { w = windowSamples < 1 ? 1 : (windowSamples > MAXW ? MAXW : windowSamples); idx = 0; for (int i = 0; i < MAXW; i++) buf[i] = 0; }
  float process(float x) { float old = buf[idx]; buf[idx] = x; idx = (idx + 1) % w; return x - old; }
};

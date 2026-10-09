// ppg_spike.h - Hampel (median / MAD) spike remover.
//
//  Handles: single-sample outliers from EMI bursts, I2C glitches and LED/ADC transients.
//  A sample is replaced by the window median when it deviates from it by more than
//  k * 1.4826 * MAD (k = 5: with only 7 samples MAD itself is noisy, k = 3.5 altered ~1 % of clean samples). Smooth pulse shapes often avoid replacement,
//  but a narrow real feature can be changed; inspect raw and filtered data.
//
//  Delay: (W-1)/2 samples. The first W-1 outputs pass through undelayed (start-up only).
//  If you run motion cancellation after this stage, delay the accelerometer references
//  by DELAY samples too (PpgDenoiser does this for you).
//
//  minSigma is in the units of the signal (use roughly 2 x one ADC count): it stops
//  quantisation steps on flat stretches being flagged when MAD is 0.
#pragma once
#include <math.h>

namespace ppgnr {

template <int W = 7>
class Hampel {
  static_assert(W >= 3 && W <= 15 && (W % 2) == 1, "W must be odd, 3..15");
public:
  static const int DELAY = (W - 1) / 2;
  float k = 5.0f;
  float minSigma = 2.0f;
  unsigned long spikes = 0;

  void reset() { idx_ = cnt_ = 0; spikes = 0; }

  float process(float x) {
    buf_[idx_] = x;
    if (++idx_ >= W) idx_ = 0;
    if (cnt_ < W) { if (++cnt_ < W) return x; }
    float t[W];
    for (int i = 0; i < W; i++) t[i] = buf_[i];
    const float med = median(t);
    for (int i = 0; i < W; i++) t[i] = fabsf(buf_[i] - med);
    const float sigma = 1.4826f * median(t);
    const int c = (idx_ + DELAY) % W;                 // idx_ now points at the oldest sample
    float y = buf_[c];
    const float lim = k * (sigma > minSigma ? sigma : minSigma);
    if (fabsf(y - med) > lim) { y = med; buf_[c] = y; spikes++; }
    return y;
  }

private:
  float buf_[W]; int idx_ = 0, cnt_ = 0;
  static float median(float* a) {                     // insertion sort, W <= 15
    for (int i = 1; i < W; i++) { float v = a[i]; int j = i - 1; while (j >= 0 && a[j] > v) { a[j + 1] = a[j]; j--; } a[j + 1] = v; }
    return a[W / 2];
  }
};

}  // namespace ppgnr

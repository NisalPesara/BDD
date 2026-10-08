// ppg_dc.h - DC level tracking, AC/DC normalisation and perfusion index.
//
//  Handles: photodiode dark current, ambient/LED DC offset, and (via AC/DC normalisation)
//  the MULTIPLICATIVE effects of skin pigmentation, tissue thickness and LED intensity.
//  Normalisation cannot remove additive noise; it only makes amplitude comparable.
#pragma once
#include <math.h>
#include "ppg_biquad.h"

namespace ppgnr {

// One-pole low-pass tracking the slow level; process() returns the AC part (x - dc).
struct DcTracker {
  float a = 0.01f, dc = 0.0f; bool init = false;
  void begin(double fs, double fc) { a = (float)(1.0 - exp(-2.0 * kPi * fc / fs)); init = false; dc = 0; }
  float process(float x) {
    if (!init) { dc = x; init = true; }
    dc += a * (x - dc);
    return x - dc;
  }
  float level() const { return dc; }
};

// AC / |DC|: removes gain differences between subjects, sites and LED currents.
inline float acdcNormalize(float ac, float dc) {
  const float d = fabsf(dc);
  return d > 1e-9f ? ac / d : 0.0f;
}

// Sliding-window perfusion index: 100 * (max - min of the AC signal over W samples) / |DC|.
// Recomputed every `hop` samples (cheap O(W) scan). Window should span several beats.
template <int W>
class PerfusionIndex {
public:
  float pi = 0.0f;
  void begin(int hopSamples) { hop_ = hopSamples < 1 ? 1 : hopSamples; idx_ = cnt_ = since_ = 0; pi = 0; }
  float push(float ac, float dc) {
    buf_[idx_] = ac; if (++idx_ >= W) idx_ = 0; if (cnt_ < W) cnt_++;
    if (cnt_ == W && ++since_ >= hop_) {
      since_ = 0;
      float mn = buf_[0], mx = buf_[0];
      for (int i = 1; i < W; i++) { if (buf_[i] < mn) mn = buf_[i]; if (buf_[i] > mx) mx = buf_[i]; }
      pi = fabsf(dc) > 1e-9f ? 100.0f * (mx - mn) / fabsf(dc) : 0.0f;
    }
    return pi;
  }
private:
  float buf_[W]; int idx_ = 0, cnt_ = 0, since_ = 0, hop_ = 1;
};

}  // namespace ppgnr

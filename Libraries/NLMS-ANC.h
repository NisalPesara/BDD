// METHOD 6 - Adaptive noise cancellation with accelerometers (NLMS, multi-reference)
//  d = PPG with motion artifact, ref[r] = zero-mean accelerometer axes
//  e = d - sum_r sum_i w[r][i] * ref_r[n-i]   -> cleaned PPG (error is the output)
#pragma once
#include <string.h>
#include "config.h"

#ifndef ANC_P_MIN
#define ANC_P_MIN 0.0f   // freeze adaptation below this total reference energy (0 = never freeze)
#endif
#ifndef ANC_LEAK
#define ANC_LEAK  0.0f   // leakage gamma, 0 = plain NLMS; try 1e-5..1e-4 if weights drift
#endif

struct NlmsAnc {
  static constexpr int R = ANC_NUM_REFS;
  static constexpr int L = ANC_TAPS;

  float mu  = ANC_MU;
  float eps = 1e-3f;                 // regularizer: set near R*L*(accel noise variance)
  float pmin = ANC_P_MIN;

  NlmsAnc() { reset(); }

  void reset() {
    memset(w_, 0, sizeof w_);
    memset(x_, 0, sizeof x_);
    head_ = 0; p_ = 0.0f;
  }

  // adapt=false lets the caller freeze learning (e.g. when the PPG is flagged bad)
  float process(float d, const float* ref, bool adapt = true) {
    // 1) push new samples (no shifting): head moves backwards, sample stored twice
    head_ = (head_ == 0) ? L - 1 : head_ - 1;
    for (int r = 0; r < R; r++) {
      float* xr = x_[r];
      float old = xr[head_];                    // x_r[n-L], about to be overwritten
      float nw  = ref[r];
      xr[head_] = xr[head_ + L] = nw;
      p_ += nw * nw - old * old;                // recursive energy update
    }
    if (head_ == 0) {                           // once per lap: exact recompute, bounds drift
      float s = 0.0f;
      for (int r = 0; r < R; r++)
        for (int i = 0; i < L; i++) s += x_[r][i] * x_[r][i];
      p_ = s;
    }
    if (p_ < 0.0f) p_ = 0.0f;

    // 2) filter output: window for axis r is the contiguous x_[r][head_ .. head_+L-1]
    float y = 0.0f;
    for (int r = 0; r < R; r++) {
      const float* xr = x_[r] + head_;
      for (int i = 0; i < L; i++) y += w_[r][i] * xr[i];
    }
    float e = d - y;

    // 3) NLMS update, gated on reference energy
    if (adapt && p_ >= pmin) {
      const float g = mu * e / (eps + p_);
      const float a = 1.0f - ANC_LEAK;
      for (int r = 0; r < R; r++) {
        const float* xr = x_[r] + head_;
        for (int i = 0; i < L; i++) w_[r][i] = a * w_[r][i] + g * xr[i];
      }
    }
    return e;
  }

  float energy() const { return p_; }           // useful for choosing pmin / eps

private:
  float w_[R][L];
  float x_[R][2 * L];
  float p_;
  int   head_;
};
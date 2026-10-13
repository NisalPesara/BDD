// ppg_anc.h - Multi-reference NLMS adaptive noise canceller.
//
//  Handles: motion artifact correlated with measured references (accelerometer axes,
//  relative acceleration, and optionally contact-force signals or their slope).
//  d = PPG with artifact, ref[0..R-1] = zero-mean reference samples,
//  e = d - sum_r sum_i w[r][i] * ref_r[n-i]   (e is the cleaned output).
//
//  Improvements over m06_nlms_anc.h (same maths when pmin = 0 and leak = 0):
//   - mirrored circular delay lines (no per-sample shifting)
//   - recursive input energy with an exact refresh once per lap (bounded drift)
//   - adaptation freeze when the references carry no motion (pmin), because NLMS otherwise
//     takes full-size steps on pure sensor noise
//   - optional leakage, and an `adapt` flag so the caller can freeze learning on bad SQI
//  Tuning: eps ~ R*L*(reference noise variance); pmin ~ 3-5x the energy() seen at rest.
//  References must be time-aligned with d (delay them if d went through a delaying stage).
#pragma once
#include <string.h>
#include <math.h>

namespace ppgnr {

template <int R, int L>
class Nlms {
  static_assert(R >= 1 && L >= 1, "R and L must be >= 1");
public:
  float mu = 0.05f;
  float eps = 1e-3f;
  float pmin = 0.0f;       // freeze adaptation when total reference energy < pmin (0 = never)
  float leak = 0.0f;       // weight leakage gamma, 0 = plain NLMS
  float idleDecay = 1.0f;  // weights x this per sample while frozen (p < pmin): 1 = keep, <1 = fade out so the
                           // output returns to the untouched PPG when the motion stops (no stale cancellation)

  Nlms() { reset(); }

  void reset() { memset(w_, 0, sizeof w_); memset(x_, 0, sizeof x_); head_ = 0; p_ = 0.0f; }

  float process(float d, const float* ref, bool adapt = true) {
    if (!ref || !isfinite(d)) { reset(); return isfinite(d) ? d : 0.f; }
    for (int r = 0; r < R; ++r) if (!isfinite(ref[r])) { reset(); return d; }
    head_ = (head_ == 0) ? L - 1 : head_ - 1;
    for (int r = 0; r < R; r++) {
      float* xr = x_[r];
      const float old = xr[head_], nw = ref[r];
      xr[head_] = xr[head_ + L] = nw;
      p_ += nw * nw - old * old;
    }
    if (head_ == 0) {                                  // once per lap: exact recompute
      float s = 0;
      for (int r = 0; r < R; r++) for (int i = 0; i < L; i++) s += x_[r][i] * x_[r][i];
      p_ = s;
    }
    if (p_ < 0.0f) p_ = 0.0f;

    float y = 0;
    for (int r = 0; r < R; r++) {
      const float* xr = x_[r] + head_;
      for (int i = 0; i < L; i++) y += w_[r][i] * xr[i];
    }
    const float e = d - y;

    if (adapt && p_ >= pmin && eps > 0 && mu >= 0 && mu <= 1 && leak >= 0 && leak < 1) {
      const float g = mu * e / (eps + p_), a = 1.0f - leak;
      for (int r = 0; r < R; r++) {
        const float* xr = x_[r] + head_;
        for (int i = 0; i < L; i++) w_[r][i] = a * w_[r][i] + g * xr[i];
      }
    } else if (p_ < pmin && idleDecay < 1.0f) {
      for (int r = 0; r < R; r++) for (int i = 0; i < L; i++) w_[r][i] *= idleDecay;
    }
    return e;
  }

  float energy() const { return p_; }                  // use at rest to choose pmin / eps

private:
  float w_[R][L];
  float x_[R][2 * L];
  float p_;
  int head_;
};

}  // namespace ppgnr

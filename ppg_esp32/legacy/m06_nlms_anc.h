// METHOD 6 - Adaptive noise cancellation with accelerometers
//  (Widrow 1975; Han et al. 2007 NLMS; Foo & Wilson 2006; Poh et al. 2010)
//  d   = PPG containing motion artifact
//  ref = ANC_NUM_REFS accelerometer axes (gravity removed, zero-mean)
//  e   = d - sum_r sum_k w[r][k] * ref_r[n-k]   -> cleaned PPG (error signal is the output)
//  Multi-reference NLMS: the artifact is a linear mix of finger motion in 3D, so using
//  every axis of both accelerometers (not just a magnitude) lets the filter model it.
//  The pulse itself is uncorrelated with the motion, so it is preserved.
#pragma once
#include <string.h>
#include "config.h"
struct NlmsAnc {
  float w[ANC_NUM_REFS][ANC_TAPS], x[ANC_NUM_REFS][ANC_TAPS];
  float mu = ANC_MU, eps = 1e-6f;
  NlmsAnc() { reset(); }
  void reset() { memset(w, 0, sizeof w); memset(x, 0, sizeof x); }
  float process(float d, const float* ref) {
    float y = 0, p = 0;
    for (int r = 0; r < ANC_NUM_REFS; r++) {
      for (int i = ANC_TAPS - 1; i > 0; i--) x[r][i] = x[r][i - 1];
      x[r][0] = ref[r];
      for (int i = 0; i < ANC_TAPS; i++) { y += w[r][i] * x[r][i]; p += x[r][i] * x[r][i]; }
    }
    float e = d - y;
    float g = mu * e / (eps + p);
    for (int r = 0; r < ANC_NUM_REFS; r++)
      for (int i = 0; i < ANC_TAPS; i++) w[r][i] += g * x[r][i];
    return e;
  }
};

// METHOD 6 - Adaptive noise cancellation with accelerometers (NLMS, multi-reference)
//  (Widrow 1975; Han et al. 2007 NLMS; Foo & Wilson 2006; Poh et al. 2010)
//  d   = PPG containing motion artifact
//  ref = ANC_NUM_REFS accelerometer references (from m06b, zero-mean, filtered like the PPG)
//  e   = d - sum_r sum_i w[r][i] * ref_r[n-i]   -> cleaned PPG (the error signal is the output)
//
//  Math lives in ppg_anc.h (ppgnr::Nlms). Improvements over the first m06 (updated_libraries-manuth):
//   - mirrored circular delay lines (no per-sample shifting)
//   - recursive reference energy with an exact refresh once per lap
//   - adaptation FROZEN when the references carry no motion (ANC_P_MIN), otherwise NLMS takes
//     full-size steps on pure sensor noise and slowly corrupts the pulse
//   - optional leakage (ANC_LEAK) and an `adapt` flag so learning stops on bad signal quality
//  Use it only after recordings show that motion really corrupts the PPG; tune ANC_MU and ANC_P_MIN
//  (read energy() at rest, set ANC_P_MIN to 3-5x that value).
#pragma once
#include "config.h"
#include "ppg_anc.h"

struct NlmsAnc : public ppgnr::Nlms<ANC_NUM_REFS, ANC_TAPS> {
  NlmsAnc() { configure(); }
  void configure() { mu = ANC_MU; eps = 1e-3f; pmin = ANC_P_MIN; leak = ANC_LEAK; }
  void begin() { reset(); configure(); }
};

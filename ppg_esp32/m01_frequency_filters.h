// METHOD 1 - Frequency filtering (Park 2022, Table 2 "Frequency filtering")
//  Butterworth / Chebyshev I / Chebyshev II band-pass, high-pass, low-pass.
//  Implemented as cascaded biquads (SOS) - numerically safe in float32.
//  NOTE: pyPPG uses zero-phase filtering (forward-backward). A real-time
//  filter is causal, so it adds phase delay (a few samples). Timing of
//  fiducial points is therefore shifted; see README.
#pragma once
#include <string.h>
#include "filter_coeffs.h"

#define SOS_MAX 8

struct SosFilter {
  const float (*sos)[6] = nullptr;
  int n = 0;
  float z[SOS_MAX][2];
  void set(const float (*s)[6], int sections) { sos = s; n = sections; reset(); }
  void reset() { memset(z, 0, sizeof z); }
  float process(float x) {
    if (!sos) return x;                       // pass-through when unset
    for (int i = 0; i < n; i++) {
      const float* c = sos[i];
      float y = c[0] * x + z[i][0];           // transposed direct form II
      z[i][0] = c[1] * x - c[4] * y + z[i][1];
      z[i][1] = c[2] * x - c[5] * y;
      x = y;
    }
    return x;
  }
};

enum FilterMethod {
  FM_NONE = 0,
  FM_CHEBY2_PYPPG,      // Allen&Murray 2004 / pyPPG   Cheby II 4th, 0.5-12 Hz
  FM_BUTTER_CANAC,      // Canac 2019                  Butter 4th, 0.5-10 Hz
  FM_BUTTER_PRADHAN,    // Pradhan 2019                Butter 4th, 0.5-50 Hz
  FM_CHEBY1_FERRO,      // Ferro 2015                  Cheby I 4th, 0.5-16 Hz
  FM_BUTTER_SUKOR,      // Sukor 2011                  Butter 1st, 0.5-5 Hz
  FM_BUTTER_LIU2020B,   // Liu 2020b                   Butter 2nd, 0.2-10 Hz
  FM_BUTTER_PAPINI,     // Papini 2018                 Butter 3rd, 0.4-10 Hz
  FM_HP_JOSEPH,         // Joseph 2014                 high-pass 0.15 Hz
  FM_LP_LIU2020A,       // Liu 2020a                   low-pass 10 Hz, 2nd
  FM_LP_FISCHER         // Fischer 2017                low-pass 15 Hz, 4th
};


inline bool selectFrequencyFilter(SosFilter& f, FilterMethod m) {
  switch (m) {
    case FM_NONE: f.sos = nullptr; return true;
#ifdef HAVE_CHEBY2_BP_0P5_12_O4
    case FM_CHEBY2_PYPPG:    f.set(CHEBY2_BP_0P5_12_O4, CHEBY2_BP_0P5_12_O4_N); return true;
#endif
#ifdef HAVE_BUTTER_BP_0P5_10_O4
    case FM_BUTTER_CANAC:    f.set(BUTTER_BP_0P5_10_O4, BUTTER_BP_0P5_10_O4_N); return true;
#endif
#ifdef HAVE_BUTTER_BP_0P5_50_O4
    case FM_BUTTER_PRADHAN:  f.set(BUTTER_BP_0P5_50_O4, BUTTER_BP_0P5_50_O4_N); return true;
#endif
#ifdef HAVE_CHEBY1_BP_0P5_16_O4
    case FM_CHEBY1_FERRO:    f.set(CHEBY1_BP_0P5_16_O4, CHEBY1_BP_0P5_16_O4_N); return true;
#endif
#ifdef HAVE_BUTTER_BP_0P5_5_O1
    case FM_BUTTER_SUKOR:    f.set(BUTTER_BP_0P5_5_O1, BUTTER_BP_0P5_5_O1_N); return true;
#endif
#ifdef HAVE_BUTTER_BP_0P2_10_O2
    case FM_BUTTER_LIU2020B: f.set(BUTTER_BP_0P2_10_O2, BUTTER_BP_0P2_10_O2_N); return true;
#endif
#ifdef HAVE_BUTTER_BP_0P4_10_O3
    case FM_BUTTER_PAPINI:   f.set(BUTTER_BP_0P4_10_O3, BUTTER_BP_0P4_10_O3_N); return true;
#endif
#ifdef HAVE_BUTTER_HP_0P15_O2
    case FM_HP_JOSEPH:       f.set(BUTTER_HP_0P15_O2, BUTTER_HP_0P15_O2_N); return true;
#endif
#ifdef HAVE_BUTTER_LP_10_O2
    case FM_LP_LIU2020A:     f.set(BUTTER_LP_10_O2, BUTTER_LP_10_O2_N); return true;
#endif
#ifdef HAVE_BUTTER_LP_15_O4
    case FM_LP_FISCHER:      f.set(BUTTER_LP_15_O4, BUTTER_LP_15_O4_N); return true;
#endif
    default: return false;   // filter not generated for this fs (cutoff >= Nyquist)
  }
}

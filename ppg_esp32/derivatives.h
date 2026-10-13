// Derivative signals used for fiducial detection (Park 2022, Fig. 5; pyPPG vpg/apg/jpg)
//  VPG = 1st derivative (velocity plethysmogram)      units: a.u. / s
//  APG = 2nd derivative (acceleration, a-b-c-d-e waves, Takazawa 1998)   a.u. / s^2
//  JPG = 3rd derivative                                                  a.u. / s^3
//  dt = 1 / PPG_FS = 0.008 s at 125 Hz (the PPG path is NOT decimated; only the SpO2 branch is).
//
//  Order of operations:  raw -> filter / smooth -> derivative.
//  Differentiation multiplies noise by ~2*pi*f, so high-frequency noise must be removed FIRST.
//
//  Methods (f = signal frequency, h = dt):
//   forward  (x[n+1]-x[n])/h      error O(h)   , centred at n+0.5  -> half-sample timing error
//   backward (x[n]-x[n-1])/h      error O(h)   , centred at n-0.5  -> what the OLD code did; cascading it
//                                                three times shifts JPG 1.5 samples from the PPG
//   central  (x[n+1]-x[n-1])/(2h) error O(h^2) , centred at n     -> one sample of delay in real time,
//                                                and its gain falls to 0 at fs/2, so it amplifies the
//                                                highest-frequency noise LESS than forward/backward
//   Savitzky-Golay derivative     least-squares polynomial over 2H+1 samples: smoothing and derivative in
//                                 one step, all outputs centred on the same sample (the "higher-order"
//                                 method that is practical on an ESP32: 3 x 15 multiply-adds per sample)
//
//  This header gives both practical choices, with every output TIME-ALIGNED to the PPG sample it
//  belongs to:
//   DerivSavGol : PPG, VPG, APG from ppg_savgol.h, JPG = central difference of APG.  delay = SG_HALF + 1
//   DerivCentral: input already smoothed (m04 moving average); 5-point central formulas.  delay = 2
#pragma once
#include "ppg_savgol.h"

// ---- single-step formulas (for reference / your own code) ----
inline float derivForward (float xNext, float x, float fs)            { return (xNext - x) * fs; }
inline float derivBackward(float x, float xPrev, float fs)            { return (x - xPrev) * fs; }
inline float derivCentral (float xNext, float xPrev, float fs)        { return (xNext - xPrev) * 0.5f * fs; }

// ---- Savitzky-Golay smoothing + derivatives (default) ----
template <int HALF>
struct DerivSavGol {
  static const int DELAY = HALF + 1;               // samples between input and aligned outputs
  bool begin(float fs, int order) {
    fs_ = fs; n_ = 0;
    for (int i = 0; i < 3; i++) hs_[i] = hv_[i] = ha_[i] = 0;
    return sg_.begin(fs, order);
  }
  // x = low-passed PPG. Returns true when p, v, a, j are valid (window full).
  bool process(float x, float& p, float& v, float& a, float& j) {
    float s, vv, aa;
    const bool full = sg_.process(x, s, vv, aa);
    hs_[2] = hs_[1]; hs_[1] = hs_[0]; hs_[0] = s;
    hv_[2] = hv_[1]; hv_[1] = hv_[0]; hv_[0] = vv;
    ha_[2] = ha_[1]; ha_[1] = ha_[0]; ha_[0] = aa;
    if (!full) { p = x; v = a = j = 0; n_ = 0; return false; }
    // JPG at the middle sample = central difference of APG; PPG/VPG/APG taken at the same sample
    p = hs_[1]; v = hv_[1]; a = ha_[1];
    j = (n_ >= 2) ? derivCentral(ha_[0], ha_[2], fs_) : 0.0f;
    if (n_ < 2) { n_++; return false; }
    return true;
  }
  ppgnr::SavGolDeriv<HALF> sg_;
  float hs_[3], hv_[3], ha_[3], fs_ = 125;
  int n_ = 0;
};

// ---- central differences on an already smoothed signal ----
struct DerivCentral {
  static const int DELAY = 2;
  void begin(float fs) { fs_ = fs; n_ = 0; for (int i = 0; i < 5; i++) h_[i] = 0; }
  bool process(float x, float& p, float& v, float& a, float& j) {
    for (int i = 4; i > 0; i--) h_[i] = h_[i - 1];
    h_[0] = x;                                     // h_[k] = x[n-k]; outputs are centred on n-2
    if (n_ < 5) n_++;
    if (n_ < 5) { p = x; v = a = j = 0; return false; }
    p = h_[2];
    v = (h_[1] - h_[3]) * 0.5f * fs_;
    a = (h_[1] - 2.0f * h_[2] + h_[3]) * fs_ * fs_;
    j = (h_[0] - 2.0f * h_[1] + 2.0f * h_[3] - h_[4]) * 0.5f * fs_ * fs_ * fs_;
    return true;
  }
  float h_[5], fs_ = 125; int n_ = 0;
};

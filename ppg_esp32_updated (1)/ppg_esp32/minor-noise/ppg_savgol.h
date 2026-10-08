// ppg_savgol.h - Savitzky-Golay smoothing + 1st/2nd derivative from ONE window.
//
//  Handles: broadband / quantisation noise, and noise amplification in VPG and APG.
//  A local polynomial of degree `order` is least-squares fitted to the 2*HALF+1 samples and
//  evaluated (or differentiated) at the window centre. Compared with a moving average it keeps
//  peak height and the dicrotic notch, and the three outputs (smooth, d/dt, d2/dt2) are
//  TIME-ALIGNED, unlike cascaded moving-average differences.
//
//  Delay: HALF samples (centred window). Coefficients are computed in double at begin(),
//  so any fs / order works. Derivatives are returned per SECOND (scaled by fs, fs^2).
//  Rule of thumb: window length ~ 60-100 ms, order 3 (order 2 gives the same 2nd derivative).
#pragma once
#include <math.h>

namespace ppgnr {

template <int HALF>
class SavGolDeriv {
  static_assert(HALF >= 2 && HALF <= 20, "HALF must be 2..20");
public:
  static const int W = 2 * HALF + 1;
  static const int DELAY = HALF;
  float c0[W], c1[W], c2[W];                          // smoothing, 1st and 2nd derivative weights

  // order must satisfy 2 <= order <= min(6, W-1). Returns false otherwise.
  bool begin(double fs, int order) {
    if (order < 2 || order > 6 || order > W - 1) return false;
    const int P = order + 1;
    double M[7][14];
    for (int j = 0; j < P; j++) {                      // normal matrix AtA | I
      for (int k = 0; k < P; k++) {
        double s = 0; for (int t = -HALF; t <= HALF; t++) s += pow((double)t, j + k);
        M[j][k] = s; M[j][P + k] = (j == k) ? 1.0 : 0.0;
      }
    }
    for (int col = 0; col < P; col++) {                // Gauss-Jordan with partial pivoting
      int piv = col;
      for (int r = col + 1; r < P; r++) if (fabs(M[r][col]) > fabs(M[piv][col])) piv = r;
      if (fabs(M[piv][col]) < 1e-12) return false;
      if (piv != col) for (int k = 0; k < 2 * P; k++) { double t = M[col][k]; M[col][k] = M[piv][k]; M[piv][k] = t; }
      const double d = M[col][col];
      for (int k = 0; k < 2 * P; k++) M[col][k] /= d;
      for (int r = 0; r < P; r++) if (r != col) {
        const double f = M[r][col];
        for (int k = 0; k < 2 * P; k++) M[r][k] -= f * M[col][k];
      }
    }
    for (int i = 0; i < W; i++) {                      // pinv row r at sample t: sum_k inv[r][k] t^k
      const double t = i - HALF;
      double r0 = 0, r1 = 0, r2 = 0;
      for (int k = 0; k < P; k++) {
        const double tk = pow(t, k);
        r0 += M[0][P + k] * tk; r1 += M[1][P + k] * tk; r2 += M[2][P + k] * tk;
      }
      c0[i] = (float)r0; c1[i] = (float)(r1 * fs); c2[i] = (float)(2.0 * r2 * fs * fs);
    }
    reset();
    return true;
  }

  void reset() { idx_ = cnt_ = 0; for (int i = 0; i < W; i++) buf_[i] = 0; }

  // Feed one sample. Returns true once the window is full; s, v, a then refer to the sample
  // HALF steps in the past. Before that s = x (undelayed), v = a = 0 and false is returned.
  bool process(float x, float& s, float& v, float& a) {
    buf_[idx_] = x; if (++idx_ >= W) idx_ = 0;
    if (cnt_ < W && ++cnt_ < W) { s = x; v = a = 0; return false; }
    float ss = 0, vv = 0, aa = 0;
    int p = idx_;                                      // oldest sample
    for (int i = 0; i < W; i++) {
      const float u = buf_[p]; if (++p >= W) p = 0;
      ss += c0[i] * u; vv += c1[i] * u; aa += c2[i] * u;
    }
    s = ss; v = vv; a = aa;
    return true;
  }

private:
  float buf_[W]; int idx_ = 0, cnt_ = 0;
};

}  // namespace ppgnr

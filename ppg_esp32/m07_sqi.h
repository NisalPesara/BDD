// METHOD 7 - Signal Quality Index (Park 2022, "Signal Quality Index")
//  Feature-based / rule-based SQI computed on a sliding window:
//   - skewness & kurtosis  (Krishnan 2008; Selvaraj 2011; Elgendi 2016)
//   - Shannon entropy      (Selvaraj 2011)            -> flat-line / saturation
//   - perfusion index      (Hartmut Gehring 2002; Cannesson 2008)  AC/DC * 100
//   - periodicity          (Karlen 2012 idea: cross-correlation between beats;
//                           here: normalised autocorrelation peak over the HR range)
//  ML/DL and template+DTW methods (Naeini, Liu, Li&Clifford) are not run on-device.
//  Thresholds in config.h are STARTING POINTS - tune on your own recordings.
#pragma once
#include <math.h>
#include <string.h>
#include "config.h"

struct SqiResult {
  bool  fresh = false;      // true only on the sample where a new result was computed
  bool  good = false;
  float periodicity = 0, skew = 0, kurt = 0, entropy = 0, pi = 0;
};

class SqiWindow {
public:
  static const int W = SQI_WINDOW_S * PPG_FS;
  static const int HOP = SQI_HOP_S * PPG_FS;
  static const int BINS = 16;

  SqiResult last;

  void begin() { idx = 0; count = 0; sinceHop = 0; last = SqiResult(); }

  // x = clean PPG sample, dc = slowly varying raw level (for perfusion index)
  SqiResult push(float x, float dc) {
    buf[idx] = x; idx = (idx + 1) % W;
    if (count < W) count++;
    dcAcc = dc;
    last.fresh = false;
    if (count == W && ++sinceHop >= HOP) { sinceHop = 0; compute(); last.fresh = true; }
    return last;
  }

private:
  float buf[W]; float tmp[W];
  int idx = 0, count = 0, sinceHop = 0; float dcAcc = 0;

  void compute() {
    for (int i = 0; i < W; i++) tmp[i] = buf[(idx + i) % W];      // oldest -> newest
    double m = 0; float mn = tmp[0], mx = tmp[0];
    for (int i = 0; i < W; i++) { m += tmp[i]; if (tmp[i] < mn) mn = tmp[i]; if (tmp[i] > mx) mx = tmp[i]; }
    m /= W;
    double s2 = 0, s3 = 0, s4 = 0;
    for (int i = 0; i < W; i++) { double d = tmp[i] - m; double d2 = d * d; s2 += d2; s3 += d2 * d; s4 += d2 * d2; }
    s2 /= W; s3 /= W; s4 /= W;
    double sd = sqrt(s2);
    if (sd < 1e-9) { last = SqiResult(); last.fresh = true; return; }   // flat line

    last.skew = (float)(s3 / (sd * sd * sd));
    last.kurt = (float)(s4 / (s2 * s2));

    int hist[BINS] = {0};
    float range = mx - mn; if (range < 1e-12f) range = 1e-12f;
    for (int i = 0; i < W; i++) { int b = (int)((tmp[i] - mn) / range * (BINS - 1e-3f)); hist[b]++; }
    float H = 0;
    for (int b = 0; b < BINS; b++) if (hist[b]) { float p = (float)hist[b] / W; H -= p * log2f(p); }
    last.entropy = H;

    last.pi = (fabsf(dcAcc) > 1e-9f) ? (range / fabsf(dcAcc)) * 100.0f : 0.0f;

    // periodicity: max normalised autocorrelation for lags 0.33 s .. 1.5 s (180..40 bpm)
    for (int i = 0; i < W; i++) tmp[i] -= (float)m;
    int lmin = (int)(0.33f * PPG_FS), lmax = (int)(1.5f * PPG_FS);
    float best = 0;
    for (int l = lmin; l <= lmax && l < W / 2; l++) {
      double num = 0, e1 = 0, e2 = 0;
      for (int i = 0; i < W - l; i++) { float a = tmp[i], b = tmp[i + l]; num += a * b; e1 += a * a; e2 += b * b; }
      double den = sqrt(e1 * e2);
      if (den > 1e-12) { float r = (float)(num / den); if (r > best) best = r; }
    }
    last.periodicity = best;

    last.good = (best >= SQI_MIN_PERIODICITY) &&
                (last.skew >= SQI_MIN_SKEW) &&
                (last.kurt >= SQI_KURT_MIN && last.kurt <= SQI_KURT_MAX) &&
                (H >= SQI_MIN_ENTROPY && H <= SQI_MAX_ENTROPY);
  }
};

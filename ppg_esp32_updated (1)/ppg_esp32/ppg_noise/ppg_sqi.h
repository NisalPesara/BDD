// ppg_sqi.h - Signal quality index (rule based), standalone port of the revised m07.
//
//  Handles: DECIDING which windows are unusable (motion, poor contact, clipping, flat line,
//  placement problems) instead of trying to repair them. Window statistics are computed on the
//  last WIN_S seconds every HOP_S seconds:
//    skewness, kurtosis (non-excess), amplitude-histogram entropy, perfusion index,
//    clip fraction = share of samples stuck at the systolic extreme (flat-topped pulses),
//    periodicity = best true local maximum of the normalised autocorrelation of a detrended,
//    decimated (~25 Hz) copy, over lags 0.33 s .. 1.5 s (180..40 bpm).
//  fail bitmask tells you WHICH rule rejected a window.
//
//  Thresholds are runtime members of `th` (defaults are STARTING POINTS for a fingertip;
//  retune on your own limb recordings). Log periodicity and fail on good and bad data first.
#pragma once
#include <math.h>
#include <stdint.h>

namespace ppgnr {

enum SqiFailBits : uint8_t { SQI_F_FLAT = 1, SQI_F_PERIOD = 2, SQI_F_SKEW = 4, SQI_F_KURT = 8, SQI_F_ENTROPY = 16, SQI_F_PI = 32, SQI_F_CLIP = 64, SQI_F_NONFINITE = 128 };

struct SqiThresholds {
  float minPeriodicity = 0.50f;
  float minSkew = 0.30f;            // limb pulses are more symmetric: lower it
  float kurtMin = 1.5f, kurtMax = 8.0f;
  float entropyMin = 1.5f, entropyMax = 4.0f;
  float minPi = 0.0f, maxPi = 1e9f; // percent; gate disabled by default
  float polarity = 1.0f;            // -1 if the cleaned PPG points down (flips skew only)
  float flatSd = 1e-9f;             // absolute, in signal units
  float maxClipFrac = 0.08f;        // share of samples within 1 % of the systolic extreme (max, or min if polarity < 0): flat tops
};

struct SqiOut {
  bool fresh = false, good = false;
  uint8_t fail = 0;
  float periodicity = 0, skew = 0, kurt = 0, entropy = 0, pi = 0, clipFrac = 0;
};

template <int FS, int WIN_S, int HOP_S>
class Sqi {
public:
  static const int W = WIN_S * FS;
  static const int HOP = HOP_S * FS;
  static const int BINS = 16;
  static const int D = (FS >= 50) ? FS / 25 : 1;      // decimation for the autocorrelation
  static const int FD = FS / D;
  static const int N = W / D;
  static const int LMIN = (33 * FD + 50) / 100;
  static const int LMAX = (150 * FD + 50) / 100;
  static const int LO = (LMIN > 2) ? LMIN : 2;
  static const int HI = (LMAX < N / 2 - 2) ? LMAX : N / 2 - 2;
  static_assert(HOP >= 1, "hop must be at least one sample");
  static_assert(HI > LO, "window too short for the lag range");

  SqiThresholds th;
  SqiOut last;

  void begin() { idx_ = 0; count_ = 0; since_ = 0; dcAvg_ = 0; dcSum_ = 0; hasResult_ = false; last = SqiOut(); }

  // x = cleaned PPG sample, dc = slowly varying raw level (for the perfusion index)
  SqiOut push(float x, float dc) {
    if (!isfinite(x) || !isfinite(dc)) {
      begin(); last.fail = SQI_F_NONFINITE; last.fresh = true; return last;
    }
    if (count_ == W) dcSum_ -= dcBuf_[idx_];
    dcBuf_[idx_] = dc; dcSum_ += dc;
    buf_[idx_] = x; if (++idx_ >= W) idx_ = 0;
    if (count_ < W) ++count_;
    dcAvg_ = float(dcSum_ / count_);
    last.fresh = false;
    if (count_ == W && (!hasResult_ || ++since_ >= HOP)) {
      since_ = 0; hasResult_ = true; compute(); last.fresh = true;
    }
    return last;
  }

private:
  float dcBuf_[W]; double dcSum_ = 0; bool hasResult_ = false;
  float buf_[W], dec_[N], Q_[N + 1];
  int idx_ = 0, count_ = 0, since_ = 0; float dcAvg_ = 0;

  float corr(int l) const {
    float num = 0;
    for (int i = 0; i < N - l; i++) num += dec_[i] * dec_[i + l];
    const float e1 = Q_[N - l], e2 = Q_[N] - Q_[l];
    const float den = sqrtf(fmaxf(0, e1)) * sqrtf(fmaxf(0, e2));
    return (den > 1e-12f) ? num / den : 0.0f;
  }

  void compute() {
    const float pivot = buf_[0];
    float sum = 0, mn = buf_[0], mx = buf_[0];
    for (int i = 0; i < W; i++) { const float v = buf_[i]; sum += v - pivot; if (v < mn) mn = v; if (v > mx) mx = v; }
    const float m = pivot + sum / W;
    float s2 = 0, s3 = 0, s4 = 0;
    for (int i = 0; i < W; i++) { const float d = buf_[i] - m, d2 = d * d; s2 += d2; s3 += d2 * d; s4 += d2 * d2; }
    s2 /= W; s3 /= W; s4 /= W;
    const float sd = sqrtf(s2);
    if (!isfinite(sd) || sd <= th.flatSd || s2 <= 1e-18f) { last = SqiOut(); last.fail = SQI_F_FLAT; last.fresh = true; return; }

    last.skew = th.polarity * s3 / (sd * s2);
    last.kurt = s4 / (s2 * s2);

    const float range = mx - mn;
    int hist[BINS] = {0};
    const float inv = (BINS - 1e-3f) / (range > 1e-12f ? range : 1e-12f);
    const float edge = 0.01f * range; int nMax = 0, nMin = 0;
    for (int i = 0; i < W; i++) {
      int b = (int)((buf_[i] - mn) * inv); if (b >= BINS) b = BINS - 1; if (b < 0) b = 0; hist[b]++;
      if (buf_[i] >= mx - edge) nMax++;
      if (buf_[i] <= mn + edge) nMin++;
    }
    last.clipFrac = (float)(th.polarity >= 0.0f ? nMax : nMin) / W;      // systolic side only: a clean pulse foot is naturally flat
    float H = 0;
    for (int b = 0; b < BINS; b++) if (hist[b]) { const float p = (float)hist[b] / W; H -= p * log2f(p); }
    last.entropy = H;
    last.pi = (fabsf(dcAvg_) > 1e-9f) ? (range / fabsf(dcAvg_)) * 100.0f : 0.0f;

    int p = idx_;                                      // idx_ = oldest sample
    for (int k = 0; k < N; k++) {
      float s = 0;
      for (int j = 0; j < D; j++) { s += buf_[p]; if (++p >= W) p = 0; }
      dec_[k] = s * (1.0f / D);
    }
    float dm = 0; for (int k = 0; k < N; k++) dm += dec_[k]; dm /= N;
    const float tc = (N - 1) * 0.5f;
    float sxt = 0; for (int k = 0; k < N; k++) sxt += (k - tc) * (dec_[k] - dm);
    const float slope = sxt / ((float)N * ((float)N * N - 1.0f) / 12.0f);
    Q_[0] = 0;
    for (int k = 0; k < N; k++) { const float v = dec_[k] - dm - slope * (k - tc); dec_[k] = v; Q_[k + 1] = Q_[k] + v * v; }

    float best = 0, rm = corr(LO - 1), r0 = corr(LO);
    for (int l = LO; l <= HI; l++) {
      const float rp = corr(l + 1);
      if (r0 > rm && r0 >= rp) {
        float v = r0;
        const float den = rm - 2.0f * r0 + rp;
        if (den < -1e-12f) { const float dl = 0.5f * (rm - rp) / den; v = r0 - 0.25f * (rm - rp) * dl; if (v > 1.0f) v = 1.0f; }
        if (v > best) best = v;
      }
      rm = r0; r0 = rp;
    }
    last.periodicity = best;

    uint8_t fail = 0;
    if (best < th.minPeriodicity) fail |= SQI_F_PERIOD;
    if (last.skew < th.minSkew) fail |= SQI_F_SKEW;
    if (last.kurt < th.kurtMin || last.kurt > th.kurtMax) fail |= SQI_F_KURT;
    if (H < th.entropyMin || H > th.entropyMax) fail |= SQI_F_ENTROPY;
    if (last.pi < th.minPi || last.pi > th.maxPi) fail |= SQI_F_PI;
    if (last.clipFrac > th.maxClipFrac) fail |= SQI_F_CLIP;
    last.fail = fail; last.good = (fail == 0);
  }
};

// Raw-ADC saturation monitor. Run it on the RAW counts (before any filter): a saturated front end
// cannot be repaired afterwards. frac is a ~window-second running share of samples at/over the limit.
struct RawClipMonitor {
  float hi = 262143.0f * 0.98f, lo = 0.0f, a = 0.01f, frac = 0.0f, trip = 0.01f;
  // fullScale: 262143 for the MAX30102 (18 bit). windowSec: averaging time. trip: fraction that raises saturated().
  void begin(double fs, float fullScale, float margin = 0.98f, double windowSec = 1.0, float tripFrac = 0.01f) {
    hi = fullScale * margin; lo = fullScale * (1.0f - margin); a = (float)(1.0 / (fs * windowSec)); frac = 0; trip = tripFrac;
  }
  float push(float raw) { frac += a * (((raw >= hi) || (raw <= lo)) ? 1.0f - frac : -frac); return frac; }
  bool saturated() const { return frac > trip; }
};

}  // namespace ppgnr

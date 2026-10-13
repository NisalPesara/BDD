// METHOD 8 - Adaptive peak detection (Shin, Lee & Lee 2009 idea, improved)
//  A local maximum is a systolic peak when
//    - its PROMINENCE above the last trough is > 60 % of the running pulse amplitude
//      (the old 50 %-of-amplitude threshold could fire on a strong diastolic wave), and
//    - it is outside an ADAPTIVE refractory period: HR_REFRACTORY_FRAC of the current beat interval
//      (never shorter than the MAX_BPM interval). On the wrist the diastolic wave sits at ~2/3 of
//      the cycle, so the old 60 % let it through as an extra beat.
//  The peak is reported ONE sample late (the next sample confirms the maximum): it is at n-1.
//  Heart rate = 60 / MEDIAN of the last HR_MEDIAN_N accepted beat intervals. An interval more than
//  HR_OUTLIER_FRAC away from that median (a false beat from noise, or a missed beat) is rejected
//  instead of being averaged in - one bad beat no longer moves or blanks the reading (wrist PPG).
//  HR_RELOCK_REJECTS rejections in a row = the rhythm really changed: the history restarts.
//  Samples flagged as artifact (usable = false) neither make beats nor teach the amplitude, and the
//  first interval after an artifact is skipped (its start is unknown) - the history is kept.
//  bpm() is NaN until HR_MIN_INTERVALS consistent intervals exist, and after a 3 s gap.
//  Heuristic beat detector, not an arrhythmia classifier.
#pragma once
#include <math.h>
#include <stdint.h>
#include "config.h"

class AdaptivePeakDetector {
public:
  void begin(int fs) {
    fs_ = fs; n_ = last_ = 0; havePeak_ = false; learned_ = false; gap_ = false;
    p1_ = p2_ = high_ = low_ = trough_ = amp_ = 0;
    nIbi_ = 0; rejects_ = 0;
  }
  // countIbi = false: beats are still detected (pulse segmentation needs them) but their intervals do not
  // enter the heart-rate median (used during small motion, where a peak may be motion).
  bool process(float x, bool usable = true, bool countIbi = true) {
    const uint64_t cur = n_++;
    if (!isfinite(x)) { begin(fs_); return false; }
    if (!usable) {                                       // artifact: keep time, learn nothing
      p2_ = p1_ = trough_ = x; gap_ = havePeak_; return false;
    }
    if (!learned_) {                                     // first 2 s: learn the amplitude
      if (!cur) high_ = low_ = x;
      if (x > high_) high_ = x;
      if (x < low_)  low_  = x;
      if (n_ >= uint64_t(2 * fs_)) { amp_ = high_ - low_; trough_ = low_; learned_ = true; }
      p2_ = p1_; p1_ = x; return false;
    }
    if (x < trough_) trough_ = x;
    const uint64_t at = cur - 1;
    const float minIbi = 60.f / MAX_BPM, maxIbi = 60.f / MIN_BPM;
    const float elapsed = havePeak_ ? float(at - last_) / fs_ : 100.f;
    const float ibi = medianIbi();
    const float refractory = fmaxf(minIbi - 1.f / fs_, ibi > 0 ? HR_REFRACTORY_FRAC * ibi : minIbi);
    if (havePeak_ && elapsed > 2.f) amp_ *= expf(-1.f / (2.f * fs_));   // slowly re-acquire weaker pulses
    const float prominence = p1_ - trough_;
    const bool found = p1_ > p2_ && p1_ >= x && prominence > fmaxf(1e-4f, 0.60f * amp_) &&
                       (gap_ || elapsed >= refractory);
    if (found) {
      if (gap_) gap_ = false;                            // interval spans an artifact: unknown, skip it
      else if (!countIbi) { }                            // motion: keep the history as it is
      else if (havePeak_ && elapsed >= minIbi - 1.f / fs_ && elapsed <= maxIbi + 1.f / fs_) addIbi(elapsed);
      else nIbi_ = 0;                                    // long pause or first beat: no valid interval
      last_ = at; havePeak_ = true;
      amp_ = 0.75f * amp_ + 0.25f * prominence;
      trough_ = p1_;
    }
    p2_ = p1_; p1_ = x;
    return found;
  }
  float bpm() const {
    if (!havePeak_ || n_ - last_ >= uint64_t(3 * fs_) || nIbi_ < HR_MIN_INTERVALS) return NAN;
    return 60.f / medianIbi();
  }
  float amplitude() const { return amp_; }

private:
  static constexpr int N = HR_MEDIAN_N;
  int fs_ = PPG_FS;
  uint64_t n_ = 0, last_ = 0;
  bool havePeak_ = false, learned_ = false, gap_ = false;
  float p1_ = 0, p2_ = 0, high_ = 0, low_ = 0, trough_ = 0, amp_ = 0;
  float ibis_[N] = {0};
  int nIbi_ = 0, rejects_ = 0;

  void addIbi(float v) {
    const float med = medianIbi();
    if (nIbi_ >= HR_MIN_INTERVALS && fabsf(v - med) > HR_OUTLIER_FRAC * med) {
      if (++rejects_ < HR_RELOCK_REJECTS) return;        // outlier: ignore it
      nIbi_ = 0;                                         // persistent change: re-lock on the new rhythm
    }
    rejects_ = 0;
    if (nIbi_ < N) ibis_[nIbi_++] = v;
    else { for (int i = 1; i < N; i++) ibis_[i - 1] = ibis_[i]; ibis_[N - 1] = v; }
  }
  float medianIbi() const {
    if (nIbi_ == 0) return 0;
    float t[N];
    for (int i = 0; i < nIbi_; i++) t[i] = ibis_[i];
    for (int i = 1; i < nIbi_; i++) { const float v = t[i]; int j = i - 1; while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; } t[j + 1] = v; }
    return (nIbi_ & 1) ? t[nIbi_ / 2] : 0.5f * (t[nIbi_ / 2 - 1] + t[nIbi_ / 2]);
  }
};

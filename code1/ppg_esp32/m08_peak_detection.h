// METHOD 8 - Adaptive peak detection (Shin, Lee & Lee 2009 idea, improved)
//  A local maximum is a systolic peak when
//    - its PROMINENCE above the last trough is > 60 % of the running pulse amplitude
//      (the old 50 %-of-amplitude threshold could fire on a strong diastolic wave), and
//    - it is outside an ADAPTIVE refractory period: 60 % of the current beat interval
//      (never shorter than the MAX_BPM interval).
//  The peak is reported ONE sample late (the next sample confirms the maximum): it is at n-1.
//  bpm() is NaN when no recent, plausible interval exists (no stale value survives a gap).
//  Heuristic beat detector, not an arrhythmia classifier.
#pragma once
#include <math.h>
#include <stdint.h>
#include "config.h"

class AdaptivePeakDetector {
public:
  void begin(int fs) {
    fs_ = fs; n_ = last_ = 0; havePeak_ = false; learned_ = false;
    p1_ = p2_ = high_ = low_ = trough_ = amp_ = ibi_ = 0;
  }
  bool process(float x) {
    const uint64_t cur = n_++;
    if (!isfinite(x)) { begin(fs_); return false; }
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
    const float refractory = fmaxf(minIbi - 1.f / fs_, ibi_ > 0 ? 0.60f * ibi_ : minIbi);
    if (havePeak_ && elapsed > 2.f) amp_ *= expf(-1.f / (2.f * fs_));   // slowly re-acquire weaker pulses
    const float prominence = p1_ - trough_;
    const bool found = p1_ > p2_ && p1_ >= x && prominence > fmaxf(1e-4f, 0.60f * amp_) && elapsed >= refractory;
    if (found) {
      if (havePeak_ && elapsed >= minIbi - 1.f / fs_ && elapsed <= maxIbi + 1.f / fs_)
        ibi_ = ibi_ > 0 ? 0.75f * ibi_ + 0.25f * elapsed : elapsed;
      else ibi_ = 0;
      last_ = at; havePeak_ = true;
      amp_ = 0.75f * amp_ + 0.25f * prominence;
      trough_ = p1_;
    }
    p2_ = p1_; p1_ = x;
    return found;
  }
  float bpm() const { return havePeak_ && ibi_ > 0 && n_ - last_ < uint64_t(3 * fs_) ? 60.f / ibi_ : NAN; }
  float amplitude() const { return amp_; }

private:
  int fs_ = PPG_FS;
  uint64_t n_ = 0, last_ = 0;
  bool havePeak_ = false, learned_ = false;
  float p1_ = 0, p2_ = 0, high_ = 0, low_ = 0, trough_ = 0, amp_ = 0, ibi_ = 0;
};

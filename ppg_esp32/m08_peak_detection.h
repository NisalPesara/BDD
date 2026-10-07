#pragma once
#include <math.h>
#include <stdint.h>

// Prominence plus adaptive refractory detection. A candidate must exceed 60%
// of the recent pulse range; this rejects the supplied 45% diastolic-wave case.
// This is a heuristic detector, not an arrhythmia classifier.
class AdaptivePeakDetector {
 public:
  void begin(int fs) {
    fs_ = fs; n_ = last_ = 0; havePeak_ = false; learned_ = false;
    p1_ = p2_ = high_ = low_ = trough_ = amp_ = ibi_ = 0;
  }
  bool process(float x) {
    const uint64_t cur = n_++;
    if (!isfinite(x)) { begin(fs_); return false; }
    if (!learned_) {
      if (!cur) high_ = low_ = x;
      if (x > high_) high_ = x;
      if (x < low_) low_ = x;
      if (n_ >= uint64_t(2 * fs_)) {
        amp_ = high_ - low_; trough_ = low_; learned_ = true;
      }
      p2_ = p1_; p1_ = x; return false;
    }
    if (x < trough_) trough_ = x;
    const uint64_t at = cur - 1;
    const float elapsed = havePeak_ ? float(at - last_) / fs_ : 100.f;
    const float refractory = fmaxf(60.f / 180.f - 1.f / fs_, ibi_ > 0 ? 0.60f * ibi_ : 60.f / 180.f);
    // Slowly re-acquire weaker pulses after a long gap. No stale BPM survives.
    if (havePeak_ && elapsed > 2.f) amp_ *= expf(-1.f / (2.f * fs_));
    const float prominence = p1_ - trough_;
    const bool found = p1_ > p2_ && p1_ >= x && prominence > fmaxf(1e-4f, .60f * amp_)
                       && elapsed >= refractory;
    if (found) {
      if (havePeak_ && elapsed >= 60.f / 180.f - 1.f / fs_ && elapsed <= 60.f / 40.f + 1.f / fs_)
        ibi_ = ibi_ > 0 ? .75f * ibi_ + .25f * elapsed : elapsed;
      else ibi_ = 0;
      last_ = at; havePeak_ = true;
      amp_ = .75f * amp_ + .25f * prominence;
      trough_ = p1_;
    }
    p2_ = p1_; p1_ = x;
    return found;
  }
  float bpm() const {
    return havePeak_ && ibi_ > 0 && n_ - last_ < uint64_t(3 * fs_) ? 60.f / ibi_ : NAN;
  }
 private:
  int fs_ = 125;
  uint64_t n_ = 0, last_ = 0;
  bool havePeak_ = false, learned_ = false;
  float p1_ = 0, p2_ = 0, high_ = 0, low_ = 0, trough_ = 0, amp_ = 0, ibi_ = 0;
};

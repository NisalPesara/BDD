#pragma once
#include <math.h>

template <int MAXW = 64>
class MovingAverage {
  static_assert(MAXW >= 1, "MAXW must be >= 1");

public:
  // Returns the window length (samples) actually used, so the caller can
  // detect clamping (w == MAXW) or collapse (w == 1).
  int begin(int fs, int ms) {
    long n = lroundf(ms * fs / 1000.0f);
    w_ = (n < 1) ? 1 : (n > MAXW ? MAXW : (int)n);
    reset();
    return w_;
  }

  void reset() {
    idx_ = filled_ = 0;
    sum_ = 0.0f;
    for (int i = 0; i < MAXW; i++) buf_[i] = 0.0f;
  }

  float process(float x) {
    if (filled_ < w_) {            // warm-up: window not yet full
      sum_ += x;
      ++filled_;
    } else {                       // steady state: newest in, oldest out
      sum_ += x - buf_[idx_];
    }
    buf_[idx_] = x;

    if (++idx_ >= w_) {            // one full lap completed
      idx_ = 0;
      float s = 0.0f;              // exact re-sum: bounds rounding drift
      for (int i = 0; i < filled_; i++) s += buf_[i];
      sum_ = s;
    }
    return sum_ / filled_;
  }

  int window() const { return w_; }

private:
  float buf_[MAXW];
  float sum_ = 0.0f;
  int w_ = 1, idx_ = 0, filled_ = 0;
};
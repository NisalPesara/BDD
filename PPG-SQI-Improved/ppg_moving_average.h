#pragma once

template <int WindowSamples>
class PpgMovingAverage {
  static_assert(WindowSamples >= 1, "WindowSamples must be >= 1");

 public:
  PpgMovingAverage() { reset(); }

  float process(float input) {
    sum_ -= buffer_[index_];
    buffer_[index_] = input;
    sum_ += input;
    if (++index_ == WindowSamples) index_ = 0;
    if (filled_ < WindowSamples) ++filled_;
    return sum_ / filled_;
  }

  void reset() {
    for (int i = 0; i < WindowSamples; ++i) buffer_[i] = 0.0f;
    sum_ = 0.0f;
    index_ = 0;
    filled_ = 0;
  }

  int window() const { return WindowSamples; }
  int filled() const { return filled_; }

 private:
  float buffer_[WindowSamples];
  float sum_ = 0.0f;
  int index_ = 0;
  int filled_ = 0;
};

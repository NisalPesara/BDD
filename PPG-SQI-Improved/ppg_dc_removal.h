#pragma once

#include <cmath>

class PpgDcRemoval {
 public:
  PpgDcRemoval() = default;

  bool configure(float sampleRateHz, float timeConstantSeconds) {
    if (!std::isfinite(sampleRateHz) ||
        !std::isfinite(timeConstantSeconds) || sampleRateHz <= 0.0f ||
        timeConstantSeconds <= 0.0f) {
      return false;
    }
    alpha_ = std::exp(-1.0f / (sampleRateHz * timeConstantSeconds));
    configured_ = std::isfinite(alpha_) && alpha_ >= 0.0f && alpha_ < 1.0f;
    reset();
    return configured_;
  }

  float process(float input) {
    if (!configured_) return input;
    if (!initialized_) {
      baseline_ = input;
      initialized_ = true;
      return 0.0f;
    }
    baseline_ = alpha_ * baseline_ + (1.0f - alpha_) * input;
    return input - baseline_;
  }

  void reset() {
    baseline_ = 0.0f;
    initialized_ = false;
  }

  float baseline() const { return baseline_; }
  bool configured() const { return configured_; }

 private:
  float alpha_ = 0.0f;
  float baseline_ = 0.0f;
  bool configured_ = false;
  bool initialized_ = false;
};

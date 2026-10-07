#pragma once

#include <cmath>

class PpgBiquad {
 public:
  PpgBiquad() { reset(); }

  bool configureLowPass(float sampleRateHz, float cutoffHz,
                        float q = 0.70710678f) {
    if (!validParameters(sampleRateHz, cutoffHz, q)) return false;
    const float omega = 2.0f * kPi * cutoffHz / sampleRateHz;
    const float cosine = std::cos(omega);
    const float alpha = std::sin(omega) / (2.0f * q);
    const float a0 = 1.0f + alpha;
    set((1.0f - cosine) * 0.5f / a0, (1.0f - cosine) / a0,
        (1.0f - cosine) * 0.5f / a0, -2.0f * cosine / a0,
        (1.0f - alpha) / a0);
    reset();
    return true;
  }

  bool configureHighPass(float sampleRateHz, float cutoffHz,
                         float q = 0.70710678f) {
    if (!validParameters(sampleRateHz, cutoffHz, q)) return false;
    const float omega = 2.0f * kPi * cutoffHz / sampleRateHz;
    const float cosine = std::cos(omega);
    const float alpha = std::sin(omega) / (2.0f * q);
    const float a0 = 1.0f + alpha;
    set((1.0f + cosine) * 0.5f / a0, -(1.0f + cosine) / a0,
        (1.0f + cosine) * 0.5f / a0, -2.0f * cosine / a0,
        (1.0f - alpha) / a0);
    reset();
    return true;
  }

  bool configureBandPass(float sampleRateHz, float centerHz, float q) {
    if (!validParameters(sampleRateHz, centerHz, q)) return false;
    const float omega = 2.0f * kPi * centerHz / sampleRateHz;
    const float alpha = std::sin(omega) / (2.0f * q);
    const float a0 = 1.0f + alpha;
    set(alpha / a0, 0.0f, -alpha / a0, -2.0f * std::cos(omega) / a0,
        (1.0f - alpha) / a0);
    reset();
    return true;
  }

  float process(float input) {
    const float output = b0_ * input + z1_;
    z1_ = b1_ * input - a1_ * output + z2_;
    z2_ = b2_ * input - a2_ * output;
    return output;
  }

  void reset() {
    z1_ = 0.0f;
    z2_ = 0.0f;
  }

 private:
  static constexpr float kPi = 3.14159265358979323846f;
  float b0_ = 1.0f;
  float b1_ = 0.0f;
  float b2_ = 0.0f;
  float a1_ = 0.0f;
  float a2_ = 0.0f;
  float z1_ = 0.0f;
  float z2_ = 0.0f;

  static bool validParameters(float sampleRateHz, float frequencyHz, float q) {
    return std::isfinite(sampleRateHz) && std::isfinite(frequencyHz) &&
           std::isfinite(q) && sampleRateHz > 0.0f && frequencyHz > 0.0f &&
           frequencyHz < sampleRateHz * 0.5f && q > 0.0f;
  }

  void set(float b0, float b1, float b2, float a1, float a2) {
    b0_ = b0;
    b1_ = b1;
    b2_ = b2;
    a1_ = a1;
    a2_ = a2;
  }
};

class PpgButterworth {
 public:
  PpgButterworth() { clear(); }

  bool configureLowPass(float sampleRateHz, float cutoffHz, int order = 2) {
    if (!validOrder(order) || !validCutoff(sampleRateHz, cutoffHz)) return false;
    PpgBiquad first;
    PpgBiquad second;
    const float q1 = order == 4 ? 0.5411961f : 0.70710678f;
    if (!first.configureLowPass(sampleRateHz, cutoffHz, q1)) return false;
    if (order == 4 &&
        !second.configureLowPass(sampleRateHz, cutoffHz, 1.3065630f)) {
      return false;
    }
    stages_[0] = first;
    stages_[1] = second;
    stageCount_ = order == 4 ? 2 : 1;
    return true;
  }

  bool configureHighPass(float sampleRateHz, float cutoffHz, int order = 2) {
    if (!validOrder(order) || !validCutoff(sampleRateHz, cutoffHz)) return false;
    PpgBiquad first;
    PpgBiquad second;
    const float q1 = order == 4 ? 0.5411961f : 0.70710678f;
    if (!first.configureHighPass(sampleRateHz, cutoffHz, q1)) return false;
    if (order == 4 &&
        !second.configureHighPass(sampleRateHz, cutoffHz, 1.3065630f)) {
      return false;
    }
    stages_[0] = first;
    stages_[1] = second;
    stageCount_ = order == 4 ? 2 : 1;
    return true;
  }

  bool configureBandPass(float sampleRateHz, float lowCutoffHz,
                         float highCutoffHz) {
    if (!validCutoff(sampleRateHz, lowCutoffHz) ||
        !validCutoff(sampleRateHz, highCutoffHz) ||
        lowCutoffHz >= highCutoffHz) {
      return false;
    }
    PpgBiquad highPass;
    PpgBiquad lowPass;
    if (!highPass.configureHighPass(sampleRateHz, lowCutoffHz) ||
        !lowPass.configureLowPass(sampleRateHz, highCutoffHz)) {
      return false;
    }
    stages_[0] = highPass;
    stages_[1] = lowPass;
    stageCount_ = 2;
    return true;
  }

  float process(float input) {
    float output = input;
    for (int i = 0; i < stageCount_; ++i) output = stages_[i].process(output);
    return output;
  }

  void reset() {
    for (int i = 0; i < stageCount_; ++i) stages_[i].reset();
  }

  void clear() {
    stageCount_ = 0;
    stages_[0] = PpgBiquad();
    stages_[1] = PpgBiquad();
  }

 private:
  PpgBiquad stages_[2];
  int stageCount_ = 0;

  static bool validOrder(int order) { return order == 2 || order == 4; }

  static bool validCutoff(float sampleRateHz, float cutoffHz) {
    return std::isfinite(sampleRateHz) && std::isfinite(cutoffHz) &&
           sampleRateHz > 0.0f && cutoffHz > 0.0f &&
           cutoffHz < sampleRateHz * 0.5f;
  }
};

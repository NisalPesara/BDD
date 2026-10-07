#pragma once

#include <cmath>

template <int References, int Taps>
class PpgNlmsAnc {
  static_assert(References >= 1, "References must be >= 1");
  static_assert(Taps >= 1, "Taps must be >= 1");

 public:
  PpgNlmsAnc() { reset(); }

  bool configure(float stepSize, float epsilon = 1.0e-6f,
                 float minimumReferenceEnergy = 0.0f,
                 float leakage = 0.0f) {
    if (!std::isfinite(stepSize) || !std::isfinite(epsilon) ||
        !std::isfinite(minimumReferenceEnergy) || !std::isfinite(leakage) ||
        stepSize <= 0.0f || stepSize >= 2.0f || epsilon <= 0.0f ||
        minimumReferenceEnergy < 0.0f || leakage < 0.0f || leakage >= 1.0f) {
      return false;
    }
    stepSize_ = stepSize;
    epsilon_ = epsilon;
    minimumReferenceEnergy_ = minimumReferenceEnergy;
    leakage_ = leakage;
    reset();
    return true;
  }

  float process(float desired, const float (&references)[References],
                bool adapt = true) {
    if (++head_ == Taps) head_ = 0;

    for (int ref = 0; ref < References; ++ref) {
      history_[ref][head_] = references[ref];
    }

    float estimate = 0.0f;
    float energy = 0.0f;
    for (int ref = 0; ref < References; ++ref) {
      int historyIndex = head_;
      for (int tap = 0; tap < Taps; ++tap) {
        const float sample = history_[ref][historyIndex];
        estimate += weights_[ref][tap] * sample;
        energy += sample * sample;
        if (--historyIndex < 0) historyIndex = Taps - 1;
      }
    }
    const float error = desired - estimate;

    if (adapt && energy >= minimumReferenceEnergy_) {
      const float gain = stepSize_ * error / (epsilon_ + energy);
      const float retain = 1.0f - leakage_;
      for (int ref = 0; ref < References; ++ref) {
        int historyIndex = head_;
        for (int tap = 0; tap < Taps; ++tap) {
          weights_[ref][tap] =
              retain * weights_[ref][tap] +
              gain * history_[ref][historyIndex];
          if (--historyIndex < 0) historyIndex = Taps - 1;
        }
      }
    }
    return error;
  }

  void reset() {
    head_ = Taps - 1;
    for (int ref = 0; ref < References; ++ref) {
      for (int tap = 0; tap < Taps; ++tap) {
        weights_[ref][tap] = 0.0f;
        history_[ref][tap] = 0.0f;
      }
    }
  }

 private:
  float weights_[References][Taps];
  float history_[References][Taps];
  float stepSize_ = 0.1f;
  float epsilon_ = 1.0e-6f;
  float minimumReferenceEnergy_ = 0.0f;
  float leakage_ = 0.0f;
  int head_ = Taps - 1;
};

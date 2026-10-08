#pragma once
#include "wave_detection.h"
#include "spo2_algorithm.h"

// Explicit, fixed baseline of accepted pulses. No automatic re-learning during
// compression. A change in optical coupling requires a NEW baseline capture.
class PerfusionMonitor {
 public:
  bool baselineReady = false, capturing = false;
  float amplitudeRatio = NAN, piRatio = NAN, areaRatio = NAN;
  void reset() { *this = PerfusionMonitor(); }
  void captureBaseline() { reset(); capturing = true; }
  void push(const PulseFeatures& f, uint64_t now) {
    if (!f.valid || !(f.amp > 0) || !(f.pi > 0) || !(f.area > 0)) return;
    lastPulse_ = now;
    if (capturing) {
      a_[count_] = f.amp; p_[count_] = f.pi; u_[count_] = f.area;
      if (++count_ >= BASELINE_PULSES) {
        ppgspo2::sort(a_, BASELINE_PULSES); ppgspo2::sort(p_, BASELINE_PULSES);
        ppgspo2::sort(u_, BASELINE_PULSES);
        const int lo = (BASELINE_PULSES - 1)/2, hi = BASELINE_PULSES/2;
        ampBase_ = .5f*(a_[lo]+a_[hi]); piBase_ = .5f*(p_[lo]+p_[hi]); areaBase_ = .5f*(u_[lo]+u_[hi]);
        baselineReady = true; capturing = false;
      }
    }
    if (baselineReady) {
      amplitudeRatio = f.amp / ampBase_; piRatio = f.pi / piBase_; areaRatio = f.area / areaBase_;
      if (amplitudeRatio < REDUCED_TREND_RATIO && piRatio < REDUCED_TREND_RATIO) {
        if (reduced_ < TREND_CONFIRM_PULSES) ++reduced_;
      } else reduced_ = 0;
    }
  }
  void breakContinuity() { if (capturing) count_ = 0; reduced_ = 0; amplitudeRatio = piRatio = areaRatio = NAN; }
  const char* status(const PpgSample& s, uint64_t now) const {
    if (!s.optical) return "NO_OPTICAL_SIGNAL";
    if (s.clipped) return "SATURATED";
    if (!s.contact) return "CHECK_PROBE_CONTACT";
    if (!s.forceStable) return "PRESSURE_CHANGING";
    if (!s.motionQuiet) return "MOTION_OR_REFERENCE_FAULT";
    if (!s.valid) return "WARMUP";
    if (!lastPulse_ || now - lastPulse_ > PULSE_TIMEOUT_US) return "NO_RELIABLE_PULSE";
    if (!s.sqi_good) return "POOR_SIGNAL_QUALITY";
    if (capturing) return "CAPTURING_BASELINE";
    if (!baselineReady) return "PULSE_DETECTED_NO_BASELINE";
    if (reduced_ >= TREND_CONFIRM_PULSES) return "REDUCED_PULSE_TREND";
    return "PULSE_DETECTED";
  }
 private:
  float a_[BASELINE_PULSES] = {}, p_[BASELINE_PULSES] = {}, u_[BASELINE_PULSES] = {};
  float ampBase_ = NAN, piBase_ = NAN, areaBase_ = NAN;
  int count_ = 0, reduced_ = 0;
  uint64_t lastPulse_ = 0;
};

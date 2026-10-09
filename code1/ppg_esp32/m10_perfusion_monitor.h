// METHOD 10 - Limb perfusion (blood-flow) assessment from peak detection + derivative features
//
//  A PPG measures how much the tissue blood VOLUME pulses with each heart beat. It cannot give
//  flow in mL/min, and absolute amplitude depends on LED current, skin, contact and site.
//  What it CAN do reliably is compare the same probe, on the same limb, against itself:
//
//   1. capture a BASELINE: median of BASELINE_PULSES accepted pulses while flow is known to be
//      normal (automatic after boot / 'r' when AUTO_BASELINE = 1, or on command 'b').
//   2. for every new accepted pulse compute ratios to that baseline:
//        amplitude  (systolic peak - onset)                       -> pulse strength
//        PI         (amplitude / DC x 100)                        -> strength corrected for light level
//        area       (AUC, m09)                                    -> whole pulse volume, not one point
//        upSlope    (VPG maximum, derivative)                     -> how fast blood volume rises (inflow)
//      flow score = median of the available ratios (robust to one bad feature).
//   3. classify, confirmed over FLOW_CONFIRM_PULSES pulses:
//        score >= FLOW_GOOD_RATIO              GOOD     blood flow reaches the limb as at baseline
//        FLOW_POOR_RATIO <= score < GOOD       REDUCED  pulsatile flow present but weaker
//        score <  FLOW_POOR_RATIO              POOR     only a weak pulse remains
//        no accepted pulse for PULSE_TIMEOUT_S while the optical signal, pressure and motion are OK
//                                              NO PULSE no pulsatile flow detected (occlusion) - or
//                                                       the probe lost the pulse: check placement
//
//  Research indicator, not a validated medical device. Never use it alone to decide tourniquet
//  application or release. Motion, probe pressure and cold also reduce the PPG.
#pragma once
#include <math.h>
#include <stdint.h>
#include "config.h"
#include "wave_detection.h"

enum FlowState : uint8_t {
  FLOW_NO_SIGNAL = 0, FLOW_SATURATED, FLOW_WARMUP, FLOW_MOTION, FLOW_PRESSURE_CHANGING,
  FLOW_NO_PULSE, FLOW_WAITING_PULSES, FLOW_CAPTURING_BASELINE, FLOW_NO_BASELINE,
  FLOW_GOOD, FLOW_REDUCED, FLOW_POOR
};

inline const char* flowStateText(FlowState s) {
  switch (s) {
    case FLOW_NO_SIGNAL:          return "NO SIGNAL - place the probe";
    case FLOW_SATURATED:          return "SENSOR SATURATED - lower LED current / ambient light";
    case FLOW_WARMUP:             return "WARM-UP";
    case FLOW_MOTION:             return "MOTION - keep the limb still";
    case FLOW_PRESSURE_CHANGING:  return "BAND PRESSURE CHANGING";
    case FLOW_NO_PULSE:           return "NO PULSE - no pulsatile flow detected (occlusion or probe)";
    case FLOW_WAITING_PULSES:     return "SEARCHING FOR PULSE";
    case FLOW_CAPTURING_BASELINE: return "CAPTURING BASELINE";
    case FLOW_NO_BASELINE:        return "PULSE PRESENT - no baseline (send 'b')";
    case FLOW_GOOD:               return "GOOD FLOW";
    case FLOW_REDUCED:            return "REDUCED FLOW";
    case FLOW_POOR:               return "POOR FLOW";
  }
  return "?";
}

class PerfusionMonitor {
public:
  bool  baselineReady = false, capturing = false;
  float ampBase = NAN, piBase = NAN, areaBase = NAN, slopeBase = NAN;
  float ampRatio = NAN, piRatio = NAN, areaRatio = NAN, slopeRatio = NAN, score = NAN;
  FlowState state = FLOW_NO_SIGNAL;

  void begin(bool autoBaseline) {
    baselineReady = false; capturing = autoBaseline; count_ = 0;
    ampBase = piBase = areaBase = slopeBase = NAN;
    clearRatios(); flowClass_ = FLOW_NO_BASELINE; candidate_ = FLOW_NO_BASELINE; agree_ = 0;
    lastPulseMs_ = 0; haveLastPulse_ = false; okSinceMs_ = 0; okRunning_ = false;
    state = FLOW_NO_SIGNAL;
  }
  void captureBaseline() { baselineReady = false; capturing = true; count_ = 0; clearRatios();
                           flowClass_ = candidate_ = FLOW_NO_BASELINE; agree_ = 0; }
  void clearBaseline()   { baselineReady = false; capturing = false; count_ = 0; clearRatios();
                           flowClass_ = candidate_ = FLOW_NO_BASELINE; agree_ = 0; }
  int  baselineCount() const { return count_; }

  // Call for every accepted pulse.
  void push(const PulseFeatures& f, uint32_t nowMs) {
    if (!f.valid || !(f.amp > 0)) return;
    lastPulseMs_ = nowMs; haveLastPulse_ = true;
    if (capturing) {
      a_[count_] = f.amp; p_[count_] = f.pi; u_[count_] = f.area; s_[count_] = f.upSlope;
      if (++count_ >= BASELINE_PULSES) {
        ampBase = median(a_, BASELINE_PULSES); piBase = median(p_, BASELINE_PULSES);
        areaBase = median(u_, BASELINE_PULSES); slopeBase = median(s_, BASELINE_PULSES);
        baselineReady = true; capturing = false;
      }
      return;
    }
    if (!baselineReady) return;
    ampRatio   = ratio(f.amp, ampBase);
    piRatio    = ratio(f.pi, piBase);
    areaRatio  = ratio(f.area, areaBase);
    slopeRatio = ratio(f.upSlope, slopeBase);
    float r[4]; int n = 0;
    if (isfinite(ampRatio))   r[n++] = ampRatio;
    if (isfinite(piRatio))    r[n++] = piRatio;
    if (isfinite(areaRatio))  r[n++] = areaRatio;
    if (isfinite(slopeRatio)) r[n++] = slopeRatio;
    score = n ? median(r, n) : NAN;
    if (!isfinite(score)) return;
    const FlowState c = score >= FLOW_GOOD_RATIO ? FLOW_GOOD : (score >= FLOW_POOR_RATIO ? FLOW_REDUCED : FLOW_POOR);
    if (c == candidate_) { if (agree_ < 255) agree_++; } else { candidate_ = c; agree_ = 1; }
    if (agree_ >= FLOW_CONFIRM_PULSES || flowClass_ == FLOW_NO_BASELINE) flowClass_ = candidate_;
  }

  // Call every sample (cheap). Decides the displayed state.
  FlowState update(const PpgSample& s, uint32_t nowMs) {
    const bool measurable = s.optical && !s.clipped && s.valid && s.motionQuiet && s.forceStable;
    if (measurable) { if (!okRunning_) { okRunning_ = true; okSinceMs_ = nowMs; } }
    else okRunning_ = false;

    if (!s.optical)          return state = FLOW_NO_SIGNAL;
    if (s.clipped)           return state = FLOW_SATURATED;
    if (!s.valid)            return state = FLOW_WARMUP;
    if (!s.motionQuiet)      return state = FLOW_MOTION;
    if (!s.forceStable)      return state = FLOW_PRESSURE_CHANGING;
    const uint32_t ref = (haveLastPulse_ && (int32_t)(lastPulseMs_ - okSinceMs_) > 0) ? lastPulseMs_ : okSinceMs_;
    if ((uint32_t)(nowMs - ref) > (uint32_t)(PULSE_TIMEOUT_S * 1000.0f)) {
      clearRatios();
      return state = FLOW_NO_PULSE;
    }
    if (!haveLastPulse_)     return state = FLOW_WAITING_PULSES;
    if (capturing)           return state = FLOW_CAPTURING_BASELINE;
    if (!baselineReady)      return state = FLOW_NO_BASELINE;
    return state = flowClass_;
  }

  const char* text() const { return flowStateText(state); }

private:
  float a_[BASELINE_PULSES], p_[BASELINE_PULSES], u_[BASELINE_PULSES], s_[BASELINE_PULSES];
  int count_ = 0;
  FlowState flowClass_ = FLOW_NO_BASELINE, candidate_ = FLOW_NO_BASELINE;
  uint8_t agree_ = 0;
  uint32_t lastPulseMs_ = 0, okSinceMs_ = 0;
  bool haveLastPulse_ = false, okRunning_ = false;

  void clearRatios() { ampRatio = piRatio = areaRatio = slopeRatio = score = NAN; }
  static float ratio(float x, float base) { return (isfinite(x) && isfinite(base) && base > 1e-9f) ? x / base : NAN; }
  static float median(const float* x, int n) {           // ignores NaN; n <= BASELINE_PULSES or 4
    float t[BASELINE_PULSES > 4 ? BASELINE_PULSES : 4]; int m = 0;
    for (int i = 0; i < n; i++) if (isfinite(x[i])) t[m++] = x[i];
    if (!m) return NAN;
    for (int i = 1; i < m; i++) { float v = t[i]; int j = i - 1; while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; } t[j + 1] = v; }
    return (m & 1) ? t[m / 2] : 0.5f * (t[m / 2 - 1] + t[m / 2]);
  }
};

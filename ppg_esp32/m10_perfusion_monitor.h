// METHOD 10 - Limb perfusion (blood-flow) assessment from peak detection + derivative features
//
//  A PPG measures how much the tissue blood VOLUME pulses with each heart beat. It cannot give
//  flow in mL/min, and absolute amplitude depends on LED current, skin, contact and site.
//  What it CAN do reliably is compare the same probe, on the same limb, against itself:
//
//   1. capture a BASELINE (= the fixed REFERENCE of m14): median of BASELINE_PULSES accepted STILL pulses
//      with a valid amplitude and PI, in a known unrestricted condition (command 'b'; also after boot / 'r'
//      when AUTO_BASELINE = 1). Pulses that overlapped motion never enter the reference.
//   2. for every new accepted pulse compute ratios to that baseline:
//        amplitude  (systolic peak - onset)                       -> pulse strength
//        PI         (amplitude / DC x 100)                        -> strength corrected for light level
//        area       (AUC, m09)                                    -> whole pulse volume, not one point
//        upSlope    (VPG maximum, derivative)                     -> how fast blood volume rises (inflow)
//      flow score = median of the available ratios (robust to one bad feature).
//      NOTE: the flow score / GOOD-REDUCED-POOR status is a LEGACY ENGINEERING HEURISTIC: the four ratios overlap
//      heavily (all scale with the pulse amplitude) and the cutoffs are not calibrated against any flow reference.
//      It is report text only - the 10 LEDs / Teleplot index x (m14) use relativePI alone.
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
  FLOW_GOOD, FLOW_REDUCED, FLOW_POOR,
  FLOW_LOW_QUALITY                     // rhythmic pulse present, but no pulse passed the feature checks
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
    case FLOW_LOW_QUALITY:        return "PULSE PRESENT - signal quality too low to grade flow";
  }
  return "?";
}

// Per-pulse features normalised so that current and reference pulses can be compared (same definitions for both):
//   D         = 100 A / PI      : the raw-IR DC level of the pulse (counts, from the DC tracker - never the PPG mean)
//   T         = segment length  : onset-to-onset duration of the SAME samples the area was integrated over (s)
//   areaPI    = 100 area / (T D): time-averaged pulsatile fraction (%), i.e. the area-based counterpart of PI. It is
//                                 NOT a blood volume. areaPI / PI = area / (A T) = how "full" the pulse shape is.
//   slopePI   = 100 upSlope / D : maximum upstroke of the DC-normalised pulse (%/s). Depends on amplitude AND rise
//                                 time - not a blood velocity.
//   widthFrac = PW50 / T        : pulse width at half height as a fraction of the beat (removes the heart-rate scale)
struct PulseNorm {
  static float dc(const PulseFeatures& f)        { return (f.amp > 0 && f.pi > 0) ? 100.0f * f.amp / f.pi : NAN; }
  static float period(const PulseFeatures& f)    { return f.len > 1 ? (float)(f.len - 1) / PPG_FS : NAN; }
  static float areaPI(const PulseFeatures& f)    { const float d = dc(f), t = period(f);
                                                   return (d > 0 && t > 0 && isfinite(f.area)) ? 100.0f * f.area / (t * d) : NAN; }
  static float slopePI(const PulseFeatures& f)   { const float d = dc(f);
                                                   return (d > 0 && isfinite(f.upSlope)) ? 100.0f * f.upSlope / d : NAN; }
  static float widthFrac(const PulseFeatures& f) { const float t = period(f); return (t > 0 && isfinite(f.pw50)) ? f.pw50 / t : NAN; }
};

class PerfusionMonitor {
public:
  // Reference values for m14 (medians of the same BASELINE_PULSES still pulses; amplitude CV = sample SD / mean)
  float dcBase = NAN, areaPIBase = NAN, slopePIBase = NAN, crestBase = NAN, widthFracBase = NAN, ampCvBase = NAN;
  bool  baselineReady = false, capturing = false;
  float ampBase = NAN, piBase = NAN, areaBase = NAN, slopeBase = NAN, hrBase = NAN;
  float ampRatio = NAN, piRatio = NAN, areaRatio = NAN, slopeRatio = NAN, score = NAN;
  FlowState state = FLOW_NO_SIGNAL;

  void begin(bool autoBaseline) {
    baselineReady = false; capturing = autoBaseline; count_ = 0;
    ampBase = piBase = areaBase = slopeBase = hrBase = NAN;
    dcBase = areaPIBase = slopePIBase = crestBase = widthFracBase = ampCvBase = NAN;
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
      if (!referencePulse(f)) return;                      // reference: still pulses with a valid A and PI only
      a_[count_] = f.amp; p_[count_] = f.pi; u_[count_] = f.area; s_[count_] = f.upSlope; h_[count_] = f.bpm;
      d_[count_] = PulseNorm::dc(f); ap_[count_] = PulseNorm::areaPI(f); sp_[count_] = PulseNorm::slopePI(f);
      c_[count_] = f.crest; w_[count_] = PulseNorm::widthFrac(f);
      if (++count_ >= BASELINE_PULSES) {
        ampBase = median(a_, BASELINE_PULSES); piBase = median(p_, BASELINE_PULSES);
        areaBase = median(u_, BASELINE_PULSES); slopeBase = median(s_, BASELINE_PULSES);
        hrBase = median(h_, BASELINE_PULSES);
        dcBase = median(d_, BASELINE_PULSES); areaPIBase = median(ap_, BASELINE_PULSES);
        slopePIBase = median(sp_, BASELINE_PULSES); crestBase = median(c_, BASELINE_PULSES);
        widthFracBase = median(w_, BASELINE_PULSES); ampCvBase = cv(a_, BASELINE_PULSES);
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
    // small motion does not run the NO PULSE timer (pulses during motion are often rejected by design)
    const bool measurable = s.optical && !s.clipped && s.valid && s.motionQuiet && !s.smallMotion && s.forceStable;
    if (measurable) { if (!okRunning_) { okRunning_ = true; okSinceMs_ = nowMs; } }
    else okRunning_ = false;

    if (!s.optical)          return state = FLOW_NO_SIGNAL;
    if (s.clipped)           return state = FLOW_SATURATED;
    if (!s.valid)            return state = FLOW_WARMUP;
    if (!s.motionQuiet)      return state = FLOW_MOTION;
    if (!s.forceStable)      return state = FLOW_PRESSURE_CHANGING;
    const uint32_t ref = (haveLastPulse_ && (int32_t)(lastPulseMs_ - okSinceMs_) > 0) ? lastPulseMs_ : okSinceMs_;
    if (s.smallMotion && haveLastPulse_ && (uint32_t)(nowMs - lastPulseMs_) > (uint32_t)(PULSE_TIMEOUT_S * 1000.0f))
      return state = FLOW_MOTION;                       // no verified pulse for a while because of motion
    if (okRunning_ && (uint32_t)(nowMs - ref) > (uint32_t)(PULSE_TIMEOUT_S * 1000.0f)) {
      // No pulse passed the feature checks. That is NO PULSE only if there is no rhythmic pulse either:
      // a consistent heart rhythm whose beats are at least the POOR-flow fraction of the baseline amplitude
      // means the pulse is there and only its quality is too low to grade (seen 2026-10-12: NO PULSE shown
      // with HR 66-69 bpm while the signal settled after a filter restart).
      const bool rhythmic = isfinite(s.bpm) &&
                            (!baselineReady || !(ampBase > 0) || s.beatAmp >= FLOW_POOR_RATIO * ampBase);
      if (rhythmic) return state = FLOW_LOW_QUALITY;
      clearRatios();
      return state = FLOW_NO_PULSE;
    }
    if (!haveLastPulse_)     return state = FLOW_WAITING_PULSES;
    if (capturing)           return state = FLOW_CAPTURING_BASELINE;
    if (!baselineReady)      return state = FLOW_NO_BASELINE;
    return state = flowClass_;
  }

  const char* text() const { return flowStateText(state); }

  // A pulse usable for the reference and for m14's index x: accepted, not overlapping motion, A > 0 and a
  // finite positive PI (amplitude and DC of the same pulse; NaN when the DC was invalid).
  static bool referencePulse(const PulseFeatures& f) {
    return f.valid && !f.duringMotion && isfinite(f.amp) && f.amp > 0.0f && isfinite(f.pi) && f.pi > 0.0f;
  }

private:
  float a_[BASELINE_PULSES], p_[BASELINE_PULSES], u_[BASELINE_PULSES], s_[BASELINE_PULSES], h_[BASELINE_PULSES];
  float d_[BASELINE_PULSES], ap_[BASELINE_PULSES], sp_[BASELINE_PULSES], c_[BASELINE_PULSES], w_[BASELINE_PULSES];
  int count_ = 0;
  FlowState flowClass_ = FLOW_NO_BASELINE, candidate_ = FLOW_NO_BASELINE;
  uint8_t agree_ = 0;
  uint32_t lastPulseMs_ = 0, okSinceMs_ = 0;
  bool haveLastPulse_ = false, okRunning_ = false;

  void clearRatios() { ampRatio = piRatio = areaRatio = slopeRatio = score = NAN; }
  static float ratio(float x, float base) { return (isfinite(x) && isfinite(base) && base > 1e-9f) ? x / base : NAN; }
  static float cv(const float* x, int n) {               // sample SD / mean (n >= 3)
    if (n < 3) return NAN;
    float m = 0; for (int i = 0; i < n; i++) m += x[i]; m /= n;
    if (!(m > 0)) return NAN;
    float v = 0; for (int i = 0; i < n; i++) v += (x[i] - m) * (x[i] - m);
    return sqrtf(v / (n - 1)) / m;
  }
  static float median(const float* x, int n) {           // ignores NaN; n <= BASELINE_PULSES or 4
    float t[BASELINE_PULSES > 4 ? BASELINE_PULSES : 4]; int m = 0;
    for (int i = 0; i < n; i++) if (isfinite(x[i])) t[m++] = x[i];
    if (!m) return NAN;
    for (int i = 1; i < m; i++) { float v = t[i]; int j = i - 1; while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; } t[j + 1] = v; }
    return (m & 1) ? t[m / 2] : 0.5f * (t[m / 2 - 1] + t[m / 2]);
  }
};

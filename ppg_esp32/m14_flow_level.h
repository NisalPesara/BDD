// METHOD 14 - Relative optical pulse index x -> 10-LED bar + Teleplot
//
//  x is a RELATIVE OPTICAL PULSE INDEX: how strong the DC-normalised optical pulse is compared with a FIXED
//  reference taken in a known unrestricted condition. x = 0.45 means 45 % of the reference normalised optical
//  pulse strength. It is NOT a validated percentage of blood flow remaining, it does not show adequate tissue
//  perfusion, and it does not show that a tourniquet / bandage is effective or safe.
//  Design notes, parameter roles and test results: PULSE_INDEX_DESIGN.md.
//
//  REFERENCE (command 'b' only, AUTO_BASELINE 0; m10): medians of BASELINE_PULSES accepted STILL pulses with a
//  finite PI - amplitude A, PI, heart rate, DC, area-PI, slope-PI, crest time, width fraction - plus the amplitude
//  CV of those pulses, the clean waveform (wave_detection.h) and the LED currents in use. Never updated while
//  monitoring. Cleared by 'r', probe removal and any sensor re-initialisation / LED-current change (the sketch
//  locks the LED current from 'b' on).
//
//  PRIMARY ESTIMATOR - determines x and the LEDs. For every NEW accepted, still, valid pulse i:
//     A_i  = systolic peak above the pulse's own onset-to-onset baseline           (counts of the filtered PPG)
//     D_i  = mean raw-IR DC level (DC tracker) over the same samples               (counts, never the PPG mean)
//     PI_i = 100 A_i / D_i                                                         (%)
//     relativePI = PI_i / PI_ref        target = clamp(relativePI, 0, 1)
//     x = target (first usable pulse)   else   x = FL_X_KEEP x + (1 - FL_X_KEEP) target       (once per pulse)
//  A pulse without a finite PI is not used (relativeAmplitude is never substituted). The same pulse is never
//  pushed twice, and nothing is smoothed between pulses.
//
//  AVAILABILITY (level 0 = UNKNOWN, all LEDs off - never "zero flow"):
//     no reference, probe off, sensor fault, acquisition stopped (wall clock), filter restart   -> immediately
//     clipping, warm-up, motion (large AND small, FL_USE_SMALL_MOTION 0), band pressure changing
//                                                                -> after FL_INVALID_GRACE_S of continuous invalid data
//     no fresh accepted pulse: the newest pulse ENDED more than FL_PULSE_TIMEOUT_S ago (sample time; includes the
//                              ~1-beat analysis delay)           -> UNKNOWN with the current reason (e.g. SQI reason)
//  Every UNKNOWN clears x, the smoothing state and the pulse diagnostics. After invalid data only a pulse that
//  STARTED after the invalid stretch can restore a level. A missing pulse is never turned into x = 0.
//
//  SUPPORTING DIAGNOSTICS - reported, never part of x (a small regular pulse stays a small pulse):
//     relativeAmplitude = A_i / A_ref           = relativePI x relativeDC  (NOT independent of relativePI)
//     relativeDC        = D_i / D_ref           optical conditions (contact, ambient, LED) - not proof of contact
//     relativeAreaPI    = areaPI_i / ref        area-based counterpart of relativePI (whole beat, not one point);
//                                               its median ratio to relativePI is the consistency check below
//     relativeSlopePI   = slopePI_i / ref       upstroke of the normalised pulse; depends on amplitude and rise time
//     crest / width     = crest_i / ref, (PW50/T)_i / ref      exploratory morphology; crest also depends on HR
//     hrRatio           = HR_i / HR_ref         a changed HR changes pulseDetectionRate and the timing features
//     pulseDetectionRate, amplitudeCV (vs the reference CV), templateCorrelation (vs the frozen waveform)
//  CONSISTENCY: if median(relativeAreaPI / relativePI) of the last 3 pulses leaves 1/FL_SUPPORT_RATIO ..
//  FL_SUPPORT_RATIO, `support` = DISAGREE and the report says so. x is NOT changed - the disagreement is shown, not
//  averaged away (it means the pulse SHAPE changed relative to its peak: notch loss, a spike, a distorted beat).
//
//  LEVEL: level = min(10, 1 + floor(10 x)), hysteresis FL_HYST; LEDs 1..level lit (more LEDs = stronger pulse).
//
//  OPTIONAL modes from the 2026-10-13 live session (all 0 by default; see config.h section 8b):
//     FL_TWO_POINT_CAL    't' / 'f' capture tight / free strength; levels 2..9 = 8 equal bands between them
//     FL_HOLD_ON_INVALID  hold the last level up to FL_HOLD_MAX_S instead of UNKNOWN during invalid data and while
//                         pulses are missing BECAUSE of recent motion (not when the pulse vanishes with the hand still)
//     FL_NO_PULSE_IS_LOW  stale + PPG RMS < FL_WEAK_RMS_FRAC x reference amplitude -> level 1 (x = 0)
//     FL_MEDIAN3          target = median of the last 3 relativePI values
//  Unavailable values for Teleplot / logs: -1 (non-negative values), -2 (correlation), level 0 (UNKNOWN).
#pragma once
#include <stdint.h>
#include <math.h>
#include "config.h"
#include "pipeline.h"
#include "wave_detection.h"
#include "m10_perfusion_monitor.h"
#include "ppg_sqi.h"

enum FlowUnknown : uint8_t {
  FU_NONE = 0, FU_NO_REFERENCE, FU_NO_SIGNAL, FU_CLIPPED, FU_WARMUP, FU_MOTION, FU_PRESSURE,
  FU_POOR_QUALITY, FU_SENSOR, FU_STOPPED, FU_NO_FRESH_PULSE,
  FU_SQI_CORRUPT,                  // SQI window failed on clipping / non-finite data: acquisition corrupted
  FU_SQI_WEAK,                     // SQI window failed on flat / low-PI: weak or absent pulsatility (or probe)
  FU_SQI_SHAPE                     // SQI window failed on periodicity / skew / kurtosis / entropy: irregular or changed
};

inline const char* flowUnknownText(FlowUnknown r) {
  switch (r) {
    case FU_NONE:           return "measuring";
    case FU_NO_REFERENCE:   return "no reference (send 'b' in the unrestricted condition)";
    case FU_NO_SIGNAL:      return "no tissue on the sensor";
    case FU_CLIPPED:        return "sensor clipping";
    case FU_WARMUP:         return "filter warm-up";
    case FU_MOTION:         return "motion";
    case FU_PRESSURE:       return "band pressure changing";
    case FU_POOR_QUALITY:   return "poor signal quality";
    case FU_SENSOR:         return "sensor failure";
    case FU_STOPPED:        return "acquisition stopped";
    case FU_NO_FRESH_PULSE: return "no fresh accepted pulse";
    case FU_SQI_CORRUPT:    return "signal corrupted (SQI: clipping / invalid samples)";
    case FU_SQI_WEAK:       return "weak or absent pulsatility (SQI: flat / low PI) - not a measured zero";
    case FU_SQI_SHAPE:      return "irregular / changed waveform (SQI: periodicity / shape)";
  }
  return "?";
}

// Level meaning: the band of x it covers (relative to the reference pulse, not blood flow).
inline const char* flowLevelText(int level) {
  static const char* const names[11] = {
    "UNKNOWN", "x below 0.1 of the reference", "x 0.1-0.2", "x 0.2-0.3", "x 0.3-0.4", "x 0.4-0.5", "x 0.5-0.6",
    "x 0.6-0.7", "x 0.7-0.8", "x 0.8-0.9", "x 0.9-1.0 (reference strength)" };
  return (level >= 0 && level <= 10) ? names[level] : "?";
}

// Everything captured with the reference (m10 medians + the optical settings).
struct FlowReference {
  float amp = NAN, pi = NAN, hr = NAN, dc = NAN, areaPI = NAN, slopePI = NAN, crest = NAN, widthFrac = NAN, ampCv = NAN;
  uint8_t ledRed = 0, ledIr = 0;   // LED current codes (x 0.2 mA) locked during capture and monitoring
};
// The reference record from a finished m10 capture (used by the sketch and the tests alike).
inline FlowReference referenceFrom(const PerfusionMonitor& pm, uint8_t ledRed, uint8_t ledIr) {
  FlowReference r;
  r.amp = pm.ampBase; r.pi = pm.piBase; r.hr = pm.hrBase; r.dc = pm.dcBase; r.areaPI = pm.areaPIBase;
  r.slopePI = pm.slopePIBase; r.crest = pm.crestBase; r.widthFrac = pm.widthFracBase; r.ampCv = pm.ampCvBase;
  r.ledRed = ledRed; r.ledIr = ledIr;
  return r;
}

class FlowLevel {
public:
  enum Support : int8_t { SUP_NA = -1, SUP_AGREE = 0, SUP_DISAGREE = 1 };

  // ---- primary outputs ----
  int   level = 0;                 // 0 = UNKNOWN, 1..10
  float x = NAN;                   // relative optical pulse index, 0..1
  float strength = NAN;            // x without the cap at 1 (only for FL_TWO_POINT_CAL)
  float relPI = NAN;               // last used pulse PI / reference PI (unclipped)
  FlowUnknown why = FU_NO_REFERENCE;
  // ---- supporting diagnostics (never part of x) ----
  float relAmp = NAN, relDC = NAN, relAreaPI = NAN, relSlopePI = NAN, crestRatio = NAN, widthRatio = NAN, hrRatio = NAN;
  float areaVsPI = NAN;            // median of relAreaPI / relPI over the last 3 pulses
  Support support = SUP_NA;
  float detRate = NAN, ampCV = NAN, tplCorr = NAN;

  // ---- two-point calibration (FL_TWO_POINT_CAL; NaN = not captured) ----
  enum CalPoint : uint8_t { CAL_NONE = 0, CAL_TIGHT, CAL_FREE };
  float xTight = NAN, xFree = NAN;
  bool calibrated() const {
#if FL_TWO_POINT_CAL
    return isfinite(xTight) && isfinite(xFree) && xFree - xTight >= FL_CAL_MIN_SPAN;
#else
    return false;
#endif
  }
  void startCalibration(CalPoint p) { calMode_ = p; calSum_ = 0.0f; calN_ = 0; }
  void setCalibration(float tight, float fre) { xTight = tight; xFree = fre; if (level > 0) { level = 0; setLevel(); } }
  void clearCalibration() { setCalibration(NAN, NAN); calMode_ = CAL_NONE; }
  CalPoint calibrating() const { return calMode_; }
  int calibrationCount() const { return calN_; }
  CalPoint takeCalibrationDone() { const CalPoint d = calDone_; calDone_ = CAL_NONE; return d; }
  float levelEdge(int k) const { return threshold(k); }   // value at which level k+2 starts (k = 0..8)

  void begin() { clearReference(); clearRate(); sigWhy_ = FU_WARMUP; nowMs_ = freshAfterMs_ = strongMs_ = 0; rms2_ = 0.0f; }

  // Reference cleared (or a new capture started): UNKNOWN until setReference().
  void clearReference() {
    ref_ = FlowReference();
    clearMeasurement(); clearRate();
    why = FU_NO_REFERENCE;
  }
  // m10 finished capturing. The pulse that completed the capture belongs to the reference; only pulses pushed
  // after this call change x.
  void setReference(const FlowReference& r) {
    clearMeasurement();
    ref_ = r;
    if (!(isfinite(r.amp) && r.amp > 0.0f && isfinite(r.pi) && r.pi > 0.0f)) ref_.amp = ref_.pi = NAN;
    if (!(isfinite(r.hr) && r.hr > 0.0f)) ref_.hr = NAN;
    lastPulseEndMs_ = strongMs_ = nowMs_;                 // freshness timer starts at the reference
  }
  void setReference(float amp, float pi, float hr) { FlowReference r; r.amp = amp; r.pi = pi; r.hr = hr; setReference(r); }
  const FlowReference& reference() const { return ref_; }
  // Samples lost (sample gap > FL_ACQ_TIMEOUT_MS, FIFO overflow / filter restart): the level is stale -> UNKNOWN,
  // and only a pulse that starts after this moment can restore it.
  // FL_HOLD_ON_INVALID: a filter restart does not change the optical scale (the reference is kept), so the level is
  // held through the warm-up like any other invalid stretch; only pulses that start after the restart are used.
  void samplesLost() {
    freshAfterMs_ = disturbedMs_ = nowMs_;
#if FL_HOLD_ON_INVALID
    if (level > 0) return;
#endif
    clearMeasurement(); why = hasReference() ? FU_STOPPED : FU_NO_REFERENCE;
  }
  bool  hasReference() const { return isfinite(ref_.amp) && isfinite(ref_.pi); }
  float referenceAmplitude() const { return ref_.amp; }
  float referencePI() const { return ref_.pi; }

  // Every sample. nowMs = sample time (8 ms steps; the pulse onset times use the same clock).
  void sample(const PpgSample& s, uint32_t nowMs) {
    nowMs_ = nowMs;
    if (s.sqi.fresh) sqiFail_ = s.sqi.good ? 0 : s.sqi.fail;
    sigWhy_ = !s.optical ? FU_NO_SIGNAL : s.clipped ? FU_CLIPPED : !s.valid ? FU_WARMUP
            : (!s.motionQuiet || s.smallMotion) ? FU_MOTION : !s.forceStable ? FU_PRESSURE
            : !s.sqi_good ? sqiReason() : FU_NONE;
    // invalid for the quantitative comparison (SQI is NOT here: a weak pulse under restriction makes SQI poor;
    // pulses in failed SQI windows are already rejected by wave_detection.h, which then shows up as staleness)
    invalidNow_ = !s.optical || s.clipped || !s.valid || !s.motionQuiet || !s.forceStable ||
                  (s.smallMotion && !FL_USE_SMALL_MOTION);
    if (invalidNow_) { if (!invalidRun_) { invalidRun_ = true; invalidSinceMs_ = nowMs; } }
    else invalidRun_ = false;
    if (invalidNow_ || s.smallMotion) disturbedMs_ = nowMs;   // any motion / invalid sample (for the hold rule)
    if (!s.optical) {                                     // probe off: at once (hold mode: invalid data; the sketch
      freshAfterMs_ = nowMs;                              // clears the reference after PROBE_OFF_CLEAR_MS)
      if (!FL_HOLD_ON_INVALID) clearMeasurement();
    }
    // PPG strength (RMS, ~1 s) vs the reference amplitude: "weak" = no pulse worth the name is present
    if (s.optical && s.valid) rms2_ += (s.ppg * s.ppg - rms2_) / (float)PPG_FS;
    const bool weak = s.optical && s.valid && !s.clipped && s.motionQuiet && hasReference() &&
                      sqrtf(rms2_) < FL_WEAK_RMS_FRAC * ref_.amp;
    if (!weak) strongMs_ = nowMs;
    const bool eligible = s.valid && s.optical && !s.clipped && s.motionQuiet && !s.smallMotion && s.forceStable;
    if (eligible) { binSamples_[bin_]++; if (s.beat) binBeats_[bin_]++; }
    if (++binN_ >= PPG_FS) {                               // next 1 s bin
      binN_ = 0; bin_ = (bin_ + 1) % FL_RATE_WINDOW_S;
      binSamples_[bin_] = 0; binBeats_[bin_] = 0;
    }
  }

  // Every accepted pulse (call after sample() of the same sample). Returns true when x was updated.
  bool pushPulse(const PulseFeatures& f, uint32_t nowMs) {
    (void)nowMs;
    if (!PerfusionMonitor::referencePulse(f)) return false;   // still pulse with a valid A and a finite PI
    if (!hasReference() || invalidNow_) return false;         // not measurable now
    if ((int32_t)(f.onset_ms - freshAfterMs_) < 0) return false;   // began before / during invalid data
    if (havePulse_ && f.onset_ms == lastOnsetMs_) return false;    // the same pulse twice
    havePulse_ = true; lastOnsetMs_ = f.onset_ms;
    lastPulseEndMs_ = f.len > 1 ? f.onset_ms + (uint32_t)((f.len - 1) * 1000 / PPG_FS) : nowMs_;

    relPI  = f.pi / ref_.pi;
    diagnostics(f);
#if FL_MEDIAN3
    if (!isfinite(x)) nMed_ = 0;
    med_[medHead_] = relPI; medHead_ = (medHead_ + 1) % 3; if (nMed_ < 3) nMed_++;
    const float r = median3(med_, nMed_, medHead_);
#else
    const float r = relPI;
#endif
    const float target = clamp01(r);
    const float sTarget = r < 0.0f ? 0.0f : (r > FL_STRENGTH_MAX ? FL_STRENGTH_MAX : r);
    const bool fresh = !isfinite(x) || lowNoPulse_;        // first usable pulse: x = target (nothing older)
    x        = fresh ? target  : FL_X_KEEP * x + (1.0f - FL_X_KEEP) * target;
    strength = fresh || !isfinite(strength) ? sTarget : FL_X_KEEP * strength + (1.0f - FL_X_KEEP) * sTarget;
    if (lowNoPulse_) { lowNoPulse_ = false; level = 0; }
    lastLevelMs_ = nowMs_;
    calAdd(strength);
    setLevel();
    return true;
  }

  // Every loop. sensorOk: sensor present without fault; acquiring: samples still arriving (wall clock).
  void update(bool sensorOk, bool acquiring) {
    FlowUnknown w = !hasReference() ? FU_NO_REFERENCE : !sensorOk ? FU_SENSOR : !acquiring ? FU_STOPPED
                  : (sigWhy_ == FU_NO_SIGNAL && !FL_HOLD_ON_INVALID) ? FU_NO_SIGNAL : FU_NONE;
    const bool invalidLong = invalidRun_ && (uint32_t)(nowMs_ - invalidSinceMs_) >= (uint32_t)(FL_INVALID_GRACE_S * 1000.0f);
    const bool stale = !isfinite(x) || (int32_t)(nowMs_ - lastPulseEndMs_) > (int32_t)(FL_PULSE_TIMEOUT_S * 1000.0f);
    if (w == FU_NONE && (invalidLong || stale)) {
#if FL_NO_PULSE_IS_LOW
      const uint32_t timeoutMs = (uint32_t)(FL_PULSE_TIMEOUT_S * 1000.0f);
      if (!invalidLong && (uint32_t)(nowMs_ - strongMs_) > timeoutMs && (int32_t)(nowMs_ - lastPulseEndMs_) > (int32_t)timeoutMs) {
        x = strength = 0.0f; level = 1; relPI = NAN; clearDiagnostics();
        if (!lowNoPulse_ && calMode_ == CAL_TIGHT) { calSum_ = 0.0f; calN_ = FL_CAL_PULSES - 1; calAdd(0.0f); }
        lowNoPulse_ = true; lastLevelMs_ = nowMs_;
        why = FU_NONE; detRate = rate();
        return;
      }
#endif
#if FL_HOLD_ON_INVALID
      // Hold (no LED change) while pulses are missing because of motion / invalid data, or because the detector
      // rejects them although the PPG is still strong. NOT held: the PPG itself weak for the whole timeout with the
      // hand still - the pulse is really gone -> UNKNOWN as without the hold.
      const int32_t tmo = (int32_t)(FL_PULSE_TIMEOUT_S * 1000.0f);
      const bool disturbed = invalidLong || (int32_t)(nowMs_ - disturbedMs_) <= tmo;
      const bool ppgStrong = (int32_t)(nowMs_ - strongMs_) <= tmo;
      if ((disturbed || ppgStrong) && level > 0 && (uint32_t)(nowMs_ - lastLevelMs_) <= (uint32_t)(FL_HOLD_MAX_S * 1000.0f)) {
        if (invalidLong) freshAfterMs_ = nowMs_;          // a fresh pulse must start after the invalid data
        why = FU_NONE; detRate = rate();
        return;
      }
#endif
      w = (sigWhy_ != FU_NONE) ? sigWhy_ : (sqiFail_ ? sqiReason() : FU_NO_FRESH_PULSE);
      if (invalidLong) freshAfterMs_ = nowMs_;            // a fresh pulse must start after the invalid data
    }
    why = w;
    if (w != FU_NONE) clearMeasurement();
    detRate = hasReference() ? rate() : NAN;
  }

  // LED pattern (bit i = LED i+1): LEDs 1..level lit steadily, all off while UNKNOWN.
  uint16_t ledMask() const {
    if (level <= 0) return 0;
#if FL_LED_MODE == FL_LED_SINGLE
    return (uint16_t)(1u << (level - 1));
#else
    return (uint16_t)((1u << level) - 1u);
#endif
  }

  // explicit "unavailable" values for Teleplot / logs
  static float outNonNeg(float v) { return isfinite(v) ? v : -1.0f; }
  static float outCorr(float v)   { return isfinite(v) ? v : -2.0f; }
  int supportCode() const { return (int)support; }

private:
  FlowReference ref_;
  FlowUnknown sigWhy_ = FU_WARMUP;
  uint8_t sqiFail_ = 0;
  bool invalidNow_ = true, invalidRun_ = false, havePulse_ = false, lowNoPulse_ = false;
  uint32_t nowMs_ = 0, invalidSinceMs_ = 0, freshAfterMs_ = 0, lastPulseEndMs_ = 0, lastOnsetMs_ = 0;
  uint32_t lastLevelMs_ = 0, strongMs_ = 0, disturbedMs_ = 0;
  float rms2_ = 0.0f;
  CalPoint calMode_ = CAL_NONE, calDone_ = CAL_NONE;
  float calSum_ = 0.0f;
  int   calN_ = 0;
#if FL_MEDIAN3
  float med_[3] = {0, 0, 0};                              // last 3 relativePI values
#endif
  float shp_[3] = {0, 0, 0};                              // last 3 areaVsPI values
  int   nMed_ = 0, medHead_ = 0, nShp_ = 0, shpHead_ = 0;
  float amps_[FL_CV_PULSES];
  int   nAmp_ = 0, ampHead_ = 0;
  uint16_t binSamples_[FL_RATE_WINDOW_S], binBeats_[FL_RATE_WINDOW_S];
  int   bin_ = 0, binN_ = 0;

  FlowUnknown sqiReason() const {
    if (sqiFail_ & (ppgnr::SQI_F_CLIP | ppgnr::SQI_F_NONFINITE)) return FU_SQI_CORRUPT;
    if (sqiFail_ & (ppgnr::SQI_F_FLAT | ppgnr::SQI_F_PI))        return FU_SQI_WEAK;
    if (sqiFail_)                                                 return FU_SQI_SHAPE;
    return FU_POOR_QUALITY;
  }

  void clearDiagnostics() {
    relAmp = relDC = relAreaPI = relSlopePI = crestRatio = widthRatio = hrRatio = areaVsPI = ampCV = tplCorr = NAN;
    support = SUP_NA; nAmp_ = 0; ampHead_ = 0; nShp_ = 0; shpHead_ = 0;
  }
  void clearMeasurement() {
    level = 0; x = strength = relPI = NAN; lowNoPulse_ = false; nMed_ = 0; medHead_ = 0;
    clearDiagnostics();
  }
  void clearRate() {
    for (int i = 0; i < FL_RATE_WINDOW_S; i++) { binSamples_[i] = 0; binBeats_[i] = 0; }
    bin_ = 0; binN_ = 0; detRate = NAN;
  }

  static float ratio(float a, float b) { return (isfinite(a) && isfinite(b) && b > 1e-12f) ? a / b : NAN; }
  void diagnostics(const PulseFeatures& f) {
    amps_[ampHead_] = f.amp; ampHead_ = (ampHead_ + 1) % FL_CV_PULSES; if (nAmp_ < FL_CV_PULSES) nAmp_++;
    relAmp     = f.amp / ref_.amp;
    relDC      = ratio(PulseNorm::dc(f), ref_.dc);
    relAreaPI  = ratio(PulseNorm::areaPI(f), ref_.areaPI);
    relSlopePI = ratio(PulseNorm::slopePI(f), ref_.slopePI);
    crestRatio = ratio(f.crest, ref_.crest);
    widthRatio = ratio(PulseNorm::widthFrac(f), ref_.widthFrac);
    hrRatio    = ratio(f.bpm, ref_.hr);
    tplCorr    = isfinite(f.baseCorr) ? f.baseCorr : NAN;
    ampCV      = cvOf();
    const float s = ratio(relAreaPI, relPI);              // area-based vs peak-based strength
    if (isfinite(s) && s > 0.0f) { shp_[shpHead_] = s; shpHead_ = (shpHead_ + 1) % 3; if (nShp_ < 3) nShp_++; }
    areaVsPI = nShp_ ? median3(shp_, nShp_, shpHead_) : NAN;
    support = !isfinite(areaVsPI) ? SUP_NA
            : (areaVsPI > FL_SUPPORT_RATIO || areaVsPI < 1.0f / FL_SUPPORT_RATIO) ? SUP_DISAGREE : SUP_AGREE;
  }
  static float median3(const float* v, int n, int head) {  // median of the stored values (mean of 2, the one of 1)
    if (n == 1) return v[(head + 2) % 3];
    if (n == 2) return 0.5f * (v[(head + 1) % 3] + v[(head + 2) % 3]);
    const float a = v[0], b = v[1], c = v[2];
    return fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
  }

  void calAdd(float v) {
    if (calMode_ == CAL_NONE) return;
    calSum_ += v;
    if (++calN_ < FL_CAL_PULSES) return;
    const float mean = calSum_ / calN_;
    if (calMode_ == CAL_TIGHT) xTight = mean; else xFree = mean;
    calDone_ = calMode_; calMode_ = CAL_NONE;
    level = 0;                                            // re-quantise with the new range at once
  }

  // Lower edge of level k+2 (k = 0..8). Nominal: 0.1 (k + 1). FL_TWO_POINT_CAL: edge 0 just above xTight, edge 8 =
  // xFree, 8 equal bands between (level 1 only at/below tight, level 10 at/above free).
  float threshold(int k) const {
    if (!calibrated()) return FL_LEVEL_X[k];
    if (k == 0) return xTight + FL_CAL_EPS;
    return xTight + (xFree - xTight) * (float)k / 8.0f;
  }
  // level = 1 + number of thresholds at or below the value (1..10). A level is left only when the value is more
  // than FL_HYST beyond its band: up when value >= upper edge + FL_HYST, down when value < lower edge - FL_HYST.
  void setLevel() {
    const float v = calibrated() ? strength : x;
    if (!isfinite(v)) return;
    int raw = 1;
    for (int i = 0; i < 9; i++) if (v >= threshold(i)) raw = i + 2;
    if (level == 0) { level = raw; return; }
    if (raw == level) return;
    const float lo = (level > 1 ? threshold(level - 2) : 0.0f) - FL_HYST;
    const float hi = (level < 10 ? threshold(level - 1) : 1.0f) + FL_HYST;
    if ((level > 1 && v < lo) || (level < 10 && v >= hi)) level = raw;
  }

  static float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

  float rate() const {                                    // beats found / beats expected (eligible time only)
    if (!(ref_.hr > 0)) return NAN;
    uint32_t smp = 0, beats = 0;
    for (int i = 0; i < FL_RATE_WINDOW_S; i++) if (i != bin_) { smp += binSamples_[i]; beats += binBeats_[i]; }
    const float seconds = (float)smp / PPG_FS;
    if (seconds < FL_RATE_MIN_S) return NAN;
    return (float)beats / (seconds * ref_.hr / 60.0f);
  }
  float cvOf() const {                                    // sample SD / mean
    if (nAmp_ < 3) return NAN;
    float m = 0;
    for (int i = 0; i < nAmp_; i++) m += amps_[i];
    m /= nAmp_;
    if (!(m > 0)) return NAN;
    float v = 0;
    for (int i = 0; i < nAmp_; i++) v += (amps_[i] - m) * (amps_[i] - m);
    return sqrtf(v / (nAmp_ - 1)) / m;
  }
};

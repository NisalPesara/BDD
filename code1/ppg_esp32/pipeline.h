// Complete real-time PPG pre-processing chain (one call per 125 Hz sample).
//
//   raw IR --> tissue/saturation check
//          --> Hampel spike rejection          (ppg_spike.h)   I2C hiccups, brief saturation, EMI
//          --> DC tracker + sign flip          (ppg_dc.h)      DC = raw level for the perfusion index
//          --> high-pass 0.15 Hz               (ppg_biquad.h)  baseline wander, respiration drift
//          --> [NLMS motion cancellation]      (m06 + m06b)    references get the SAME delay + high-pass
//          --> low-pass 10 Hz  (or m01 table)  (ppg_biquad.h / m01)
//          --> [m02 baseline subtraction]      optional
//          --> smoothing + VPG/APG/JPG         (ppg_savgol.h + derivatives.h, or m04 + central diff)
//   clean --> up-slope (m03)
//   clean --> extra 0.5 Hz high-pass (detection copy) --> SQI (m07) | systolic peak + bpm (m08)
//
//  Each stage is a separate library and can be switched in config.h.
//  Why spikes are removed BEFORE the IIR filters: an IIR filter turns a one-sample glitch into a
//  long ringing tail that then lowers the SQI and corrupts PI / AUC of several beats.
#pragma once
#include <stdint.h>
#include <math.h>
#include "config.h"
#include "ppg_noise.h"                   // minor-noise libraries: biquad, dc, spike, savgol, anc, force, sqi
#include "m01_frequency_filters.h"
#include "m02_baseline_removal.h"
#include "m03_moving_difference.h"
#include "m04_smoothing.h"
#include "m06_nlms_anc.h"
#include "m06b_accel_relative.h"
#include "m07_sqi.h"
#include "m08_peak_detection.h"
#include "derivatives.h"

struct PpgSample {
  uint32_t t_ms = 0;        // arrival time of the raw sample
  uint32_t aligned_ms = 0;  // t_ms minus the fixed filter delay (time of the ppg/vpg/apg values)
  float raw = 0;            // raw IR counts
  float dc = 0;             // slow raw level (perfusion-index denominator)
  float ppg = 0;            // purified PPG (systolic peak up)
  float upslope = 0;        // moving-difference output (Canac) - sharp at each upstroke
  float vpg = 0, apg = 0, jpg = 0;
  float det = 0;            // 0.5 Hz high-passed copy used for SQI and beat detection
  float bpm = NAN;
  float ancEnergy = 0;
  bool  optical = false;    // tissue in front of the sensor (raw above FINGER_THRESHOLD)
  bool  clipped = false;    // ADC saturation
  bool  forceStable = true; // band pressure not changing (FSR)
  bool  motionQuiet = true; // accelerometers quiet
  bool  valid = false;      // false during filter warm-up
  bool  sqi_good = false;   // valid AND SQI good AND pressure stable AND no motion
  bool  beat = false;       // systolic peak detected at the PREVIOUS sample
  SqiResult sqi;
};

struct PipelineConfig {
  bool  invert          = (INVERT_SIGNAL != 0);
  bool  useSpike        = true;
  bool  useAnc          = (USE_ANC != 0);
  bool  useBaselineSub  = (USE_BASELINE_SUB != 0);
  int   bandpass        = PPG_BANDPASS;      // 0 = run-time Butterworth, else an m01 FilterMethod
  int   smoothMethod    = SMOOTH_METHOD;
};

class PpgPipeline {
public:
  static const int SPIKE_DELAY = HAMPEL_WINDOW / 2;   // >= 1 (HAMPEL_WINDOW >= 3)

  bool begin(const PipelineConfig& c = PipelineConfig()) {
    cfg = c;
    bool ok = true;
    hampel.reset(); hampel.k = HAMPEL_K; hampel.minSigma = HAMPEL_MIN_SIGMA;
    dc.begin(PPG_FS, DC_TRACK_HZ);
    ok &= hp.begin(PPG_FS, PPG_HIGHPASS_HZ, true);
    ok &= detHp.begin(PPG_FS, SQI_HIGHPASS_HZ, true);
    for (int r = 0; r < ANC_NUM_REFS; r++) ok &= hpRef[r].begin(PPG_FS, PPG_HIGHPASS_HZ, true);
    if (cfg.bandpass == 0) { ok &= lp.begin(PPG_FS, PPG_LOWPASS_HZ, false); sos.sos = nullptr; }
    else                   { ok &= selectFrequencyFilter(sos, (FilterMethod)cfg.bandpass); }
    base.begin(PPG_FS);
    anc.begin(); accRel.begin(); refInit = false;
    for (int i = 0; i < SPIKE_DELAY; i++) for (int r = 0; r < ANC_NUM_REFS; r++) refDelay[i][r] = 0;
    rd = 0;
    smooth.begin(PPG_FS, SM_PPG_MS);
    ok &= sgDer.begin(PPG_FS, SG_ORDER);
    cdDer.begin(PPG_FS);
    mdiff.begin(MOVE_DIFF_SAMPLES);
    sqi.begin();
    clip.begin(PPG_FS, MAX_RAW_FULL_SCALE);
    peaks.begin(PPG_FS);
    n = 0; active = false; hpPrimed = false;
    ok_ = ok;
    return ok;
  }

  // Samples must be uniform and contiguous (the sketch resets the pipeline after a FIFO overflow).
  // acc: ACC_RAW_CH raw accelerometer values (finger xyz, band xyz, m/s^2) or nullptr.
  PpgSample process(float raw, const float* acc, uint32_t t_ms, bool forceStable = true, bool motionQuiet = true) {
    PpgSample s; s.t_ms = t_ms; s.raw = raw;
    s.forceStable = forceStable; s.motionQuiet = motionQuiet;
    s.optical = isfinite(raw) && raw >= FINGER_THRESHOLD;
    if (!ok_ || !s.optical) {                       // nothing to measure: restart cleanly when tissue returns
      if (active) begin(cfg);
      return s;
    }
    active = true;
    clip.push(raw);
    s.clipped = clip.saturated() || raw >= clip.hi;

    // 1) spikes, 2) DC, 3) high-pass
    const float x = cfg.useSpike ? hampel.process(raw) : raw;
    float ac = dc.process(x);
    s.dc = dc.level();
    if (cfg.invert) ac = -ac;
    if (!hpPrimed) { hp.prime(ac); hpPrimed = true; }
    float e = hp.process(ac);

    // 4) motion cancellation (optional)
    if (cfg.useAnc && acc) {
      float r[ANC_NUM_REFS], rr[ANC_NUM_REFS];
      accRel.process(acc, r);                       // m06b: gravity removed, relative motion
      if (cfg.useSpike) {                           // same delay as the Hampel stage (ring of SPIKE_DELAY)
        for (int k = 0; k < ANC_NUM_REFS; k++) { rr[k] = refDelay[rd][k]; refDelay[rd][k] = r[k]; }
        if (++rd >= SPIKE_DELAY) rd = 0;
      } else {
        for (int k = 0; k < ANC_NUM_REFS; k++) rr[k] = r[k];
      }
      if (!refInit) { for (int k = 0; k < ANC_NUM_REFS; k++) hpRef[k].prime(rr[k]); refInit = true; }
      for (int k = 0; k < ANC_NUM_REFS; k++) rr[k] = hpRef[k].process(rr[k]);   // same high-pass as the PPG
      e = anc.process(e, rr, forceStable && !s.clipped);
      s.ancEnergy = anc.energy();
    }

    // 5) low-pass / band-pass, 6) optional baseline subtraction
    float b = (cfg.bandpass == 0) ? lp.process(e) : sos.process(e);
    if (cfg.useBaselineSub) b = base.process(b);

    // 7) smoothing + aligned derivatives
    bool derOk;
    if (cfg.smoothMethod == SMOOTH_SAVGOL) derOk = sgDer.process(b, s.ppg, s.vpg, s.apg, s.jpg);
    else                                   derOk = cdDer.process(smooth.process(b), s.ppg, s.vpg, s.apg, s.jpg);
    s.aligned_ms = t_ms - (uint32_t)lroundf(delaySamples() * 1000.0f / PPG_FS);

    // 8) analysis signals
    s.upslope = mdiff.process(s.ppg);
    if (n < (uint32_t)(WARMUP_SECONDS * PPG_FS)) n++;
    s.valid = derOk && n >= (uint32_t)(WARMUP_SECONDS * PPG_FS) && !s.clipped;
    // wander-free copy for quality + beat detection. The m01 band-pass tables already high-pass at
    // >= 0.2 Hz, so a second high-pass would only distort the pulse (negative skew) - skip it there.
    s.det = (cfg.bandpass == 0) ? detHp.process(s.ppg) : s.ppg;
    s.sqi = sqi.push(s.det, s.dc);                  // fills during warm-up (window 5 s < warm-up 6 s)
    s.sqi_good = s.valid && s.sqi.good && forceStable && motionQuiet;
    if (s.valid) s.beat = peaks.process(s.det);
    s.bpm = s.sqi_good ? peaks.bpm() : NAN;
    return s;
  }

  // Fixed delay (samples) between the raw input and ppg/vpg/apg/jpg (IIR group delay comes on top).
  float delaySamples() const {
    const float sp = cfg.useSpike ? (float)SPIKE_DELAY : 0.0f;
    return sp + ((cfg.smoothMethod == SMOOTH_SAVGOL) ? (float)DerivSavGol<SG_HALF>::DELAY
                                                     : smooth.delaySamples() + DerivCentral::DELAY);
  }
  unsigned long spikesRemoved() const { return hampel.spikes; }
  bool configured() const { return ok_; }

  PipelineConfig cfg;

private:
  ppgnr::Hampel<HAMPEL_WINDOW> hampel;
  ppgnr::DcTracker dc;
  ppgnr::ButterFilter<2> hp, hpRef[ANC_NUM_REFS], detHp;
  ppgnr::ButterFilter<4> lp;
  SosFilter sos;
  LowerEnvelopeBaseline base;
  NlmsAnc anc;
  AccelRelative accRel;
  float refDelay[SPIKE_DELAY][ANC_NUM_REFS];
  int rd = 0; bool refInit = false;
  MovingAverage smooth;
  DerivSavGol<SG_HALF> sgDer;
  DerivCentral cdDer;
  MovingDifference mdiff;
  SqiWindow sqi;
  ppgnr::RawClipMonitor clip;
  AdaptivePeakDetector peaks;
  uint32_t n = 0;
  bool active = false, hpPrimed = false, ok_ = false;
};

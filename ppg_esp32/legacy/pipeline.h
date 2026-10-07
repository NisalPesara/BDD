// Complete real-time PPG pre-processing chain.
//   raw --> [ANC accel] --> DC removal --> frequency filter --> [baseline subtract]
//       --> [amplitude regularisation] --> moving-average smoothing --> PPG (clean)
//   clean --> VPG / APG / JPG,  moving-difference "upslope",  SQI,  optional beat flag
// Each stage is a separate method module (m01..m08) and can be switched off.
#pragma once
#include <stdint.h>
#include <math.h>
#include "config.h"
#include "m01_frequency_filters.h"
#include "m02_baseline_removal.h"
#include "m03_moving_difference.h"
#include "m04_smoothing.h"
#include "m05_amplitude_regularization.h"
#include "m06_nlms_anc.h"
#include "m06b_accel_relative.h"
#include "m07_sqi.h"
#include "m08_peak_detection.h"
#include "derivatives.h"

struct PpgSample {
  uint32_t t_ms = 0;
  float raw = 0;
  float ppg = 0;        // purified PPG
  float upslope = 0;    // moving-difference output (Canac) for onset/peak detection
  float vpg = 0, apg = 0, jpg = 0;
  bool  valid = false;  // false during filter warm-up
  bool  sqi_good = false;
  bool  beat = false;   // built-in detector: peak was at the PREVIOUS sample
  float bpm = 0;
  SqiResult sqi;
};

struct PipelineConfig {
  FilterMethod bandpass   = FM_CHEBY2_PYPPG;   // same as your pyPPG notebook (0.5-12 Hz, order 4)
  bool useHighpass        = false;             // extra Joseph 0.15 Hz HPF (usually redundant with bandpass)
  bool useBaselineSub     = false;             // m02 lower-envelope baseline subtraction
  bool useAmpRegularize   = false;             // m05 (destroys absolute amplitude)
  bool useAnc             = (USE_ANC != 0);    // m06
  bool useBuiltinPeaks    = true;              // m08
  int  moveDiffSamples    = 4;                 // m03 window
};

class PpgPipeline {
public:
  void begin(const PipelineConfig& c = PipelineConfig()) {
    cfg = c;
    selectFrequencyFilter(bp, cfg.bandpass);
    selectFrequencyFilter(hp, cfg.useHighpass ? FM_HP_JOSEPH : FM_NONE);
    base.begin(PPG_FS);
    mdiff.begin(cfg.moveDiffSamples);
    smooth.begin(PPG_FS, SM_PPG_MS);
    amp.begin(PPG_FS);
    der.begin(PPG_FS, SM_VPG_MS, SM_APG_MS, SM_JPG_MS);
    sqi.begin();
    peaks.begin(PPG_FS);
    anc.reset(); accRel.begin();
    n = 0; dcInit = false;
  }

  // refs: ACC_RAW_CH raw accelerometer values in m/s^2: finger x,y,z then band x,y,z (or nullptr)
  PpgSample process(float raw, const float* refs, uint32_t t_ms) {
    PpgSample s; s.t_ms = t_ms; s.raw = raw;
    if (INVERT_SIGNAL) raw = -raw;
    if (!dcInit) { dc = raw; dcInit = true; }
    dc += kDc * (raw - dc);                       // slow DC tracker (~0.05 Hz)
    float x = raw - dc;

    if (cfg.useAnc && refs) {                     // m06 (+ m06b builds the reference signals)
      float r[ANC_NUM_REFS];
      accRel.process(refs, r);
      x = anc.process(x, r);
    }
    x = bp.process(x);                            // m01
    if (cfg.useHighpass) x = hp.process(x);       // m01 (Joseph)
    if (cfg.useBaselineSub) x = base.process(x);  // m02
    if (cfg.useAmpRegularize) x = amp.process(x); // m05
    float p = smooth.process(x);                  // m04

    s.ppg = p;
    s.upslope = mdiff.process(p);                 // m03
    der.process(p, s.vpg, s.apg, s.jpg);          // derivatives

    n++;
    s.valid = (n > (uint32_t)(WARMUP_SECONDS * PPG_FS));
    if (s.valid) {
      SqiResult r = sqi.push(p, dc);              // m07
      s.sqi = r; s.sqi_good = r.good;
      if (cfg.useBuiltinPeaks) { s.beat = peaks.process(p); s.bpm = peaks.bpm(); }   // m08
    }
    return s;
  }

private:
  PipelineConfig cfg;
  SosFilter bp, hp;
  LowerEnvelopeBaseline base;
  MovingDifference mdiff;
  MovingAverage smooth;
  AmplitudeRegularizer amp;
  NlmsAnc anc;
  AccelRelative accRel;
  Derivatives der;
  SqiWindow sqi;
  AdaptivePeakDetector peaks;
  float dc = 0;
  const float kDc = 6.2831853f * 0.05f / PPG_FS;
  bool dcInit = false;
  uint32_t n = 0;
};

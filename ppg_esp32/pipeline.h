#pragma once
#include <math.h>
#include <stdint.h>
#include "config.h"
#include "ppg_noise/ppg_noise.h"
#include "m08_peak_detection.h"

struct PpgSample {
  uint64_t t_us = 0, aligned_us = 0;
  float raw = 0, dc = 0, ppg = 0, vpg = 0, apg = 0;
  float bpm = NAN, ancEnergy = 0;
  bool valid = false, sqi_good = false, beat = false, optical = false;
  bool clipped = false, contact = false, forceStable = false, motionQuiet = false;
  ppgnr::SqiOut sqi;
};

struct PipelineConfig {
  bool useAnc = USE_ANC;
  bool invert = INVERT_SIGNAL;
};

class PpgPipeline {
 public:
  bool begin(const PipelineConfig& cfg = PipelineConfig()) {
    cfg_ = cfg;
    ppgnr::DenoiseConfig c;
    c.fs = PPG_FS; c.invert = cfg.invert;
    c.hpHz = PPG_HIGHPASS_HZ; c.lpHz = PPG_LOWPASS_HZ;
    c.useAnc = cfg.useAnc; c.ancMu = ANC_MU; c.ancPmin = ANC_MIN_ENERGY;
    c.sgOrder = SG_ORDER;
    ok_ = den_.begin(c);
    sqi_.begin(); sqi_.th.minSkew = SQI_MIN_SKEW;
    sqi_.th.flatSd = 0.5f; // counts; a measured electronics-noise floor is preferable
    clip_.begin(PPG_FS, MAX_RAW_FULL_SCALE); clip_.lo = 1.f;
    peaks_.begin(PPG_FS); n_ = 0; active_ = false;
    return ok_;
  }
  // All samples must be uniform and contiguous. Firmware resets on FIFO gaps.
  PpgSample process(float raw, const float* refs, uint64_t t_us,
                    bool contact = true, bool forceStable = true, bool motionQuiet = true) {
    PpgSample s; s.t_us = t_us; s.raw = raw;
    s.optical = isfinite(raw) && raw >= MIN_OPTICAL_COUNTS;
    s.contact = contact; s.forceStable = forceStable; s.motionQuiet = motionQuiet;
    if (!ok_ || !s.optical || !contact || raw >= clip_.hi || raw <= clip_.lo) {
      s.clipped = isfinite(raw) && (raw >= clip_.hi || raw <= clip_.lo);
      if (active_) { begin(cfg_); }
      return s;
    }
    active_ = true; if (n_ < WARMUP_SECONDS * PPG_FS) ++n_; clip_.push(raw); s.clipped = clip_.saturated();
    const auto d = den_.process(raw, refs, forceStable && !s.clipped);
    s.dc = d.dc; s.ppg = d.clean; s.vpg = d.vpg; s.apg = d.apg;
    s.ancEnergy = den_.energy();
    constexpr uint64_t delay = (HAMPEL_WINDOW / 2 + SG_HALF) * PPG_PERIOD_US;
    s.aligned_us = t_us >= delay ? t_us - delay : 0;
    s.valid = d.valid && n_ >= WARMUP_SECONDS * PPG_FS && !s.clipped;
    // Fill SQI while filters settle; the 6-second warm-up exceeds the 5-second window.
    s.sqi = sqi_.push(s.ppg, s.dc);
    s.sqi_good = s.valid && s.sqi.good && forceStable && motionQuiet;
    if (s.valid) s.beat = peaks_.process(s.ppg);
    s.bpm = s.sqi_good ? peaks_.bpm() : NAN;
    return s;
  }
 private:
  PipelineConfig cfg_;
  ppgnr::PpgDenoiser<ANC_NUM_REFS, ANC_TAPS, SG_HALF, HAMPEL_WINDOW> den_;
  ppgnr::Sqi<PPG_FS, SQI_WINDOW_S, SQI_HOP_S> sqi_;
  ppgnr::RawClipMonitor clip_;
  AdaptivePeakDetector peaks_;
  uint32_t n_ = 0;
  bool ok_ = false, active_ = false;
};

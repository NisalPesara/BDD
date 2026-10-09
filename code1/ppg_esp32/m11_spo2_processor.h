// METHOD 11 - SpO2 processor: raw RED + IR (125 Hz) -> Maxim algorithm (25 Hz, 4 s window)
//
//   raw IR, raw RED (125 Hz, DC + AC, never the inverted/filtered/derivative PPG)
//     -> Hampel spike removal on each channel (an I2C glitch would otherwise wreck a 4-s window)
//     -> 4th-order Butterworth low-pass 6 Hz (anti-alias, keeps DC)      ppg_biquad.h
//     -> average every 5 samples = decimate to 25 Hz  (FreqS of the Maxim algorithm)
//     -> sliding window of 100 samples = 4 s, a new estimate every 25 samples = 1 s
//     -> RED/IR correlation gate (motion makes the two channels disagree)
//     -> corrected Maxim algorithm (spo2_algorithm.h)
//     -> reported SpO2 = median of the last SPO2_MEDIAN_N valid estimates (stable display)
//
// Why not feed 125 Hz straight into the 100-sample buffer: 100 samples would then be only 0.8 s
// (less than one beat at 60 bpm, so < 2 valleys -> almost always invalid), and the Maxim constants
// (MA4 = 4 samples, valley distance, HR = FreqS*60/interval) would all be 5x wrong.
//
// The Maxim lookup table is calibrated for Maxim's fingertip reference design, not for this
// probe/site. Treat the value as an uncalibrated estimate; validate against a reference oximeter.
#pragma once
#include <math.h>
#include <stdint.h>
#include "config.h"
#include "ppg_biquad.h"
#include "ppg_spike.h"
#include "spo2_algorithm.h"

class SpO2Processor {
public:
  bool    fresh = false;           // true on the sample where a new estimate was computed
  int32_t lastRaw = -999;          // last single-window estimate (%), -999 = invalid
  int32_t hr = -999;               // Maxim heart rate of the last window (cross-check only)
  float   ratio = NAN;             // R = (ACred/DCred)/(ACir/DCir) of the last window
  float   correlation = NAN;       // RED/IR correlation of the last window

  void begin() {
    irLp_.begin(PPG_FS, SPO2_LOWPASS_HZ, false); redLp_.begin(PPG_FS, SPO2_LOWPASS_HZ, false);
    irSpike_.reset(); redSpike_.reset();
    irSpike_.k = redSpike_.k = HAMPEL_K; irSpike_.minSigma = redSpike_.minSigma = HAMPEL_MIN_SIGMA;
    primed_ = false; dec_ = 0; sumI_ = sumR_ = 0; idx_ = count_ = hop_ = 0; taint_ = 0;
    nHist_ = hHead_ = badWindows_ = 0; reported_ = -999; fresh = false;
    lastRaw = hr = -999; ratio = correlation = NAN;
  }
  void reset() { begin(); }

  // Call for every 125 Hz sample. qualityGood = PPG signal quality (motion / pressure / SQI).
  void pushSample(uint32_t ir, uint32_t red, bool qualityGood) {
    fresh = false;
    if (ir < (uint32_t)FINGER_THRESHOLD || red == 0 ||
        ir >= 0.98f * MAX_RAW_FULL_SCALE || red >= 0.98f * MAX_RAW_FULL_SCALE) {   // no tissue / saturated
      if (primed_ || count_) begin();
      return;
    }
    if (!qualityGood) taint_ = BUFFER_SIZE * SPO2_DECIMATION;  // window containing a bad sample is not used
    else if (taint_ > 0) taint_--;

    const float fi = irSpike_.process((float)ir), fr = redSpike_.process((float)red);
    if (!primed_) { irLp_.prime(fi); redLp_.prime(fr); primed_ = true; }
    sumI_ += irLp_.process(fi);
    sumR_ += redLp_.process(fr);
    if (++dec_ < SPO2_DECIMATION) return;
    ir_[idx_]  = clampCounts(sumI_ / SPO2_DECIMATION);
    red_[idx_] = clampCounts(sumR_ / SPO2_DECIMATION);
    sumI_ = sumR_ = 0; dec_ = 0;
    if (++idx_ >= BUFFER_SIZE) idx_ = 0;
    if (count_ < BUFFER_SIZE) count_++;
    if (count_ < BUFFER_SIZE) return;
    if (++hop_ < FreqS * SPO2_UPDATE_S) return;
    hop_ = 0;
    compute();
  }

  // Reported (median-smoothed) SpO2, or -1 when not available.
  int  getSpO2() const { return reported_ > 0 ? reported_ : -1; }
  bool isSpO2Valid() const { return reported_ >= 70 && reported_ <= 100; }

private:
  ppgnr::ButterFilter<4> irLp_, redLp_;
  ppgnr::Hampel<HAMPEL_WINDOW> irSpike_, redSpike_;
  uint32_t ir_[BUFFER_SIZE], red_[BUFFER_SIZE];
  float sumI_ = 0, sumR_ = 0;
  int dec_ = 0, idx_ = 0, count_ = 0, hop_ = 0;
  int32_t taint_ = 0;
  bool primed_ = false;
  int32_t hist_[SPO2_MEDIAN_N]; int nHist_ = 0, hHead_ = 0, badWindows_ = 0; int32_t reported_ = -999;

  static uint32_t clampCounts(float v) { return v < 1.f ? 1u : (v > MAX_RAW_FULL_SCALE ? (uint32_t)MAX_RAW_FULL_SCALE : (uint32_t)lroundf(v)); }

  void compute() {
    uint32_t ir[BUFFER_SIZE], red[BUFFER_SIZE];
    double mi = 0, mr = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
      const int j = (idx_ + i) % BUFFER_SIZE;                  // oldest -> newest
      ir[i] = ir_[j]; red[i] = red_[j]; mi += ir[i]; mr += red[i];
    }
    mi /= BUFFER_SIZE; mr /= BUFFER_SIZE;
    double sii = 0, srr = 0, sir = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) { const double a = ir[i] - mi, b = red[i] - mr; sii += a * a; srr += b * b; sir += a * b; }
    correlation = (sii > 0 && srr > 0) ? (float)(sir / sqrt(sii * srr)) : NAN;

    int32_t spo2 = -999, hrv = -999, r100 = -999; int8_t sv = 0, hv = 0;
    maxim_heart_rate_and_oxygen_saturation(ir, BUFFER_SIZE, red, &spo2, &sv, &hrv, &hv, &r100);
    hr = hv ? hrv : -999;
    ratio = (r100 > 0) ? r100 / 100.0f : NAN;
    const bool ok = sv && taint_ == 0 && isfinite(correlation) && correlation >= SPO2_MIN_CORRELATION &&
                    spo2 >= 70 && spo2 <= 100;
    lastRaw = ok ? spo2 : -999;
    fresh = true;
    if (!ok) {                                                  // no stale value: clear after N bad windows
      if (++badWindows_ >= SPO2_MEDIAN_N) { reported_ = -999; nHist_ = hHead_ = 0; }
      return;
    }
    badWindows_ = 0;
    hist_[hHead_] = spo2; hHead_ = (hHead_ + 1) % SPO2_MEDIAN_N; if (nHist_ < SPO2_MEDIAN_N) nHist_++;
    int32_t t[SPO2_MEDIAN_N];
    for (int i = 0; i < nHist_; i++) t[i] = hist_[i];
    maxim_sort_ascend(t, nHist_);
    reported_ = (nHist_ & 1) ? t[nHist_ / 2] : (t[nHist_ / 2 - 1] + t[nHist_ / 2] + 1) / 2;
  }
};

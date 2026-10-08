#pragma once
#include <math.h>
#include "config.h"
#include "spo2_algorithm.h"
#include "ppg_noise/ppg_biquad.h"

// Raw paired RED/IR branch: matched 4th-order LP (6 Hz), average/decimate 5,
// 100 samples at 25 Hz (4 seconds), recompute each 25 samples (1 second).
// Never feed derivatives, normalized pulses, or ANC output to this module.
class SpO2Processor {
 public:
  ppgspo2::Result last;
  bool fresh = false;
  void begin() {
    irLp_.begin(PPG_FS, 6.f, false); redLp_.begin(PPG_FS, 6.f, false);
    primed_ = false; count_ = index_ = dec_ = hop_ = 0; sumI_ = sumR_ = 0;
    emitted_ = false; timestamp_ = 0; last = ppgspo2::Result(); fresh = false;
  }
  void reset() { begin(); }
  void pushSample(uint32_t ir, uint32_t red, uint64_t t_us, bool qualityGood) {
    fresh = false;
    if (!qualityGood || ir <= 1 || red <= 1 || ir >= .98f * MAX_RAW_FULL_SCALE ||
        red >= .98f * MAX_RAW_FULL_SCALE ||
        (timestamp_ && t_us - timestamp_ != PPG_PERIOD_US)) {
      if (primed_ || count_ || last.valid) reset();
      return;
    }
    timestamp_ = t_us;
    if (!primed_) { irLp_.prime(float(ir)); redLp_.prime(float(red)); primed_ = true; }
    sumI_ += irLp_.process(float(ir)); sumR_ += redLp_.process(float(red));
    if (++dec_ < PPG_FS / ppgspo2::FS) return;
    ir_[index_] = uint32_t(fmax(1., fmin(262143., sumI_ / dec_)));
    red_[index_] = uint32_t(fmax(1., fmin(262143., sumR_ / dec_)));
    sumI_ = sumR_ = 0; dec_ = 0;
    if (++index_ >= ppgspo2::WINDOW) index_ = 0;
    if (count_ < ppgspo2::WINDOW) ++count_;
    if (count_ == ppgspo2::WINDOW && (!emitted_ || ++hop_ >= ppgspo2::FS)) {
      hop_ = 0; emitted_ = true;
      uint32_t ir[ppgspo2::WINDOW], red[ppgspo2::WINDOW];
      for (int i = 0; i < ppgspo2::WINDOW; ++i) {
        const int j = (index_ + i) % ppgspo2::WINDOW;
        ir[i] = ir_[j]; red[i] = red_[j];
      }
      last = ppgspo2::calculate(ir, red, ppgspo2::WINDOW); fresh = true;
    }
  }
  int getSpO2() const { return last.valid ? last.spo2 : -1; }
  bool isSpO2Valid() const { return last.valid; }
 private:
  ppgnr::ButterFilter<4> irLp_, redLp_;
  uint32_t ir_[ppgspo2::WINDOW], red_[ppgspo2::WINDOW];
  int index_ = 0, count_ = 0, dec_ = 0, hop_ = 0;
  double sumI_ = 0, sumR_ = 0;
  bool primed_ = false, emitted_ = false;
  uint64_t timestamp_ = 0;
};

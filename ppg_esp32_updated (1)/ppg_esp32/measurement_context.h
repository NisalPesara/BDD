#pragma once
#include "config.h"
#include "ppg_noise/ppg_force.h"

struct MeasurementContext {
  float forceUs[2] = {0,0}, forceSlope[2] = {0,0};
  float dynamicEnergy = 0;
  bool stable = false, contact = true, motionQuiet = false;
  void begin() {
    for (int j = 0; j < 2; ++j) fsr_[j].begin(PPG_FS, FSR_ADC_BITS, FSR_FIXED_OHM);
    contact_.reset(); contact_.onForce = CONTACT_ON_US; contact_.offForce = CONTACT_OFF_US;
    contact_.highForce = CONTACT_HIGH_US; contact_.hyst = 20;
    contact_.slopeMax = CONTACT_SLOPE_MAX_US_S; contact_.holdSamples = FORCE_STABLE_SAMPLES;
    quiet_ = 0; accInit_ = false; stable = false; contact = true; motionQuiet = false;
  }
  void process(const float* millivolts, const float* acc, bool accGood) {
    stable = true; contact = true;
#if USE_FSR
    bool changing = false, rail = false;
    for (int j = 0; j < 2; ++j) {
      rail |= !isfinite(millivolts[j]) || millivolts[j] >= 3090.f;
      const float equivalentCounts = millivolts[j] * 4095.f / FSR_SUPPLY_MV;
      forceUs[j] = fsr_[j].process(equivalentCounts); forceSlope[j] = fsr_[j].slope();
      changing |= fabsf(forceSlope[j]) > CONTACT_SLOPE_MAX_US_S;
    }
    if (changing || rail) quiet_ = 0; else if (quiet_ < FORCE_STABLE_SAMPLES) ++quiet_;
    stable = quiet_ >= FORCE_STABLE_SAMPLES;
#if CONTACT_FSR_INDEX >= 0
    const int j = CONTACT_FSR_INDEX;
    contact = contact_.update(forceUs[j], forceSlope[j]) == ppgnr::CONTACT_OK;
    stable = stable && contact_.stable();
#endif
#else
    (void)millivolts;
#endif
    dynamicEnergy = 0;
    if (accGood && acc) {
      for (int i = 0; i < 3 * NUM_ACCEL; ++i) {
        if (!accInit_) gravity_[i] = acc[i];
        gravity_[i] += .02f * (acc[i] - gravity_[i]);
        const float a = acc[i] - gravity_[i]; dynamicEnergy += a*a;
      }
      accInit_ = true;
    }
    // NUM_ACCEL=0 deliberately disables motion assessment; logged in boot settings.
    motionQuiet = NUM_ACCEL == 0 || (accGood && dynamicEnergy < MOTION_MAX_ENERGY);
  }
 private:
  ppgnr::FsrChannel fsr_[2]; ppgnr::ContactMonitor contact_;
  float gravity_[6] = {};
  int quiet_ = 0; bool accInit_ = false;
};

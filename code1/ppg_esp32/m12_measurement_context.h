// METHOD 12 - Measurement context: band pressure (2x FSR) and motion (2x ADXL345)
//
//  A PPG amplitude change is only meaningful as a FLOW change when the probe is still and the
//  pressure on the limb is not changing. This module turns the FSR and accelerometer readings into
//  two gates used by the pipeline:
//    forceStable : |d(conductance)/dt| of both FSRs below FSR_SLOPE_MAX_US_S for FORCE_STABLE_SAMPLES
//    motionQuiet : dynamic (gravity-removed) acceleration energy below MOTION_MAX_ENERGY,
//                  held "moving" for MOTION_HOLD_SAMPLES after the last motion sample
//  FSR physics (ppg_force.h): conductance G = 1/R_fsr is ~proportional to force. Output is in
//  microsiemens (uS) - relative, NOT calibrated force/pressure. Both FSRs sit on the BAND:
//  low band pressure does not mean the optical probe has lost contact.
#pragma once
#include <math.h>
#include "config.h"
#include "ppg_force.h"

struct MeasurementContext {
  float forceUs[2] = {0, 0}, forceSlope[2] = {0, 0};
  float dynamicEnergy = 0;
  bool  forceStable = true, motionQuiet = true, fsrRail = false;

  void begin() {
#if USE_FSR
    for (int j = 0; j < 2; ++j) fsr_[j].begin(PPG_FS, FSR_ADC_BITS, FSR_FIXED_OHM);
    forceStable = !FSR_GATE_PPG;
#else
    forceStable = true;                                  // no FSR fitted: pressure gate off
#endif
    quiet_ = 0; accInit_[0] = accInit_[1] = false; motionHold_ = 0;
    motionQuiet = true; dynamicEnergy = 0; fsrRail = false;
  }

  // millivolts: the two FSR readings (ignored if USE_FSR = 0).
  // acc: finger xyz, band xyz (m/s^2); accOk[j] = sensor j present and read correctly this sample.
  void process(const float* millivolts, const float* acc, const bool* accOk) {
#if USE_FSR
    bool changing = false; fsrRail = false;
    for (int j = 0; j < 2; ++j) {
      fsrRail |= !isfinite(millivolts[j]) || millivolts[j] >= FSR_RAIL_MV;
      const float counts = millivolts[j] * (float)((1 << FSR_ADC_BITS) - 1) / FSR_SUPPLY_MV;
      forceUs[j] = fsr_[j].process(counts);
      forceSlope[j] = fsr_[j].slope();
      changing |= fabsf(forceSlope[j]) > FSR_SLOPE_MAX_US_S;
    }
    if (changing || fsrRail) quiet_ = 0; else if (quiet_ < FORCE_STABLE_SAMPLES) ++quiet_;
    forceStable = !FSR_GATE_PPG || quiet_ >= FORCE_STABLE_SAMPLES;
#else
    (void)millivolts;
    forceStable = true;
#endif
    dynamicEnergy = 0;
    bool any = false;
    for (int s = 0; s < 2; ++s) {
      if (!accOk || !accOk[s] || !acc) continue;
      any = true;
      for (int i = 3 * s; i < 3 * s + 3; ++i) {
        if (!accInit_[s]) gravity_[i] = acc[i];
        gravity_[i] += 0.02f * (acc[i] - gravity_[i]);   // ~0.4 Hz gravity / tilt tracker
        const float a = acc[i] - gravity_[i];
        dynamicEnergy += a * a;
      }
      accInit_[s] = true;
    }
    if (any && dynamicEnergy >= MOTION_MAX_ENERGY) motionHold_ = MOTION_HOLD_SAMPLES;
    else if (motionHold_ > 0) motionHold_--;
    motionQuiet = (motionHold_ == 0);                      // no accelerometer fitted -> no motion gating
  }

private:
  ppgnr::FsrChannel fsr_[2];
  float gravity_[6] = {0, 0, 0, 0, 0, 0};
  int quiet_ = 0, motionHold_ = 0;
  bool accInit_[2] = {false, false};
};

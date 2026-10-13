// METHOD 12 - Measurement context: band pressure (2x FSR) and motion (2x ADXL345)
//
//  A PPG amplitude change is only meaningful as a FLOW change when the probe is still and the
//  pressure on the limb is not changing. This module turns the FSR and accelerometer readings into
//  two gates used by the pipeline:
//    forceStable : |d(conductance)/dt| of both FSRs below FSR_SLOPE_MAX_US_S for FORCE_STABLE_SAMPLES
//    motionQuiet : RELATIVE motion (finger sensor vs band sensor, m06b) below REL_MOTION_MAX_ENERGY
//                  AND whole motion (both sensors) below MOTION_MAX_ENERGY,
//                  held "moving" for MOTION_HOLD_SAMPLES after the last motion sample.
//  FSR physics (ppg_force.h): conductance G = 1/R_fsr is ~proportional to force. Output is in
//  microsiemens (uS) - relative, NOT calibrated force/pressure. Both FSRs sit on the BAND:
//  low band pressure does not mean the optical probe has lost contact.
#pragma once
#include <math.h>
#include "config.h"
#include "ppg_force.h"
#include "m06b_accel_relative.h"

struct MeasurementContext {
  float forceUs[2] = {0, 0}, forceSlope[2] = {0, 0};
  float dynamicEnergy = 0;      // finger + band dynamic energy (whole motion)
  float relEnergy = 0;          // relative motion energy |finger - M*band|^2
  bool  relative = false;       // gate used relative motion this sample (both sensors read)
  bool  smallMotion = false;    // SMALL motion (held): cancel, then let the PPG decide
  bool  forceStable = true, motionQuiet = true, fsrRail = false;

  void begin() {
#if USE_FSR
    for (int j = 0; j < 2; ++j) fsr_[j].begin(PPG_FS, FSR_ADC_BITS, FSR_FIXED_OHM);
    forceStable = !FSR_GATE_PPG;
#else
    forceStable = true;                                  // no FSR fitted: pressure gate off
#endif
    quiet_ = 0; accInit_[0] = accInit_[1] = false; motionHold_ = 0;
    motionQuiet = true; smallMotion = false; smallHold_ = 0;
    dynamicEnergy = relEnergy = 0; energyAvg_ = 0; relative = false; fsrRail = false;
    rel_.begin();                                         // keeps the learned sensor alignment
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
    float e = 0;
    bool any = false;
    for (int s = 0; s < 2; ++s) {
      if (!accOk || !accOk[s] || !acc) { accInit_[s] = false; continue; }   // re-learn gravity when it returns
      any = true;
      for (int i = 3 * s; i < 3 * s + 3; ++i) {
        if (!accInit_[s]) gravity_[i] = acc[i];
        gravity_[i] += 0.02f * (acc[i] - gravity_[i]);   // ~0.4 Hz gravity / tilt tracker
        const float a = acc[i] - gravity_[i];
        e += a * a;
      }
      accInit_[s] = true;
    }
    // averaged over ~MOTION_ENERGY_TAU_S: single-sample tremor spikes (live 2026-10-13: median 0.02, spikes 0.5 every
    // second) flagged ~50-70 % of a still hand as moving; real movement lasts far longer than 0.2 s
    energyAvg_ = any ? energyAvg_ + (e - energyAvg_) / (MOTION_ENERGY_TAU_S * PPG_FS) : 0.0f;
    dynamicEnergy = energyAvg_;
    bool high  = any && dynamicEnergy >= MOTION_HIGH_ENERGY;
    bool small = any && dynamicEnergy >= MOTION_SMALL_ENERGY;
    relative = MOTION_GATE_RELATIVE && acc && accOk && accOk[0] && accOk[1];
    if (relative) {
      float refs[ANC_NUM_REFS];
      rel_.process(acc, refs);
      relEnergy = rel_.relEnergy();
      high  |= relEnergy >= REL_MOTION_HIGH_ENERGY;        // probe sliding on the skin
      small |= relEnergy >= REL_MOTION_MAX_ENERGY;
    } else {
      relEnergy = 0;
    }
    if (high) motionHold_ = MOTION_HOLD_SAMPLES;           // HIGH: data rejected
    else if (motionHold_ > 0) motionHold_--;
    if (small) smallHold_ = MOTION_HOLD_SAMPLES;           // SMALL: the PPG decides (pipeline.h)
    else if (smallHold_ > 0) smallHold_--;
    motionQuiet = (motionHold_ == 0);                      // no accelerometer fitted -> no motion gating
    smallMotion = motionQuiet && smallHold_ > 0;
  }

private:
  ppgnr::FsrChannel fsr_[2];
  AccelRelative rel_;
  float gravity_[6] = {0, 0, 0, 0, 0, 0};
  int quiet_ = 0, motionHold_ = 0, smallHold_ = 0;
  float energyAvg_ = 0;
  bool accInit_[2] = {false, false};
};

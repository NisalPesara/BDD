// ppg_force.h - FSR (force-sensitive resistor) conditioning and contact monitor.
//
//  Handles: ADC noise and mains pickup on the force channels, and turns contact force into
//  something the PPG chain can use:
//    * force()  - smoothed force estimate (conductance based, optionally calibrated)
//    * slope()  - rate of change per second (use it as an extra NLMS reference, or to gate)
//    * ContactMonitor - NONE / OK / HIGH state with hysteresis, plus a "stable" flag
//
//  FSR physics: conductance G = 1/R_fsr is close to proportional to force, resistance is not.
//  So the divider reading is converted to G first. FSRs drift, show hysteresis and are poor
//  at ABSOLUTE force: treat the output as relative unless you calibrate with known weights.
#pragma once
#include <math.h>
#include <stdint.h>
#include "ppg_biquad.h"

namespace ppgnr {

class FsrChannel {
public:
  // fsrOnTop = true : FSR between Vcc and the ADC node, fixed resistor rFixedOhm to GND.
  // fsrOnTop = false: fixed resistor to Vcc, FSR from the node to GND.
  // lpHz: one-pole smoothing corner for force and slope (about 5-10 Hz suits a slow pressure signal).
  void begin(double fs, int adcBits, double rFixedOhm, double lpHz = 5.0, bool fsrOnTop = true) {
    full_ = (float)((1u << adcBits) - 1u);
    rFixed_ = (float)rFixedOhm; top_ = fsrOnTop;
    a_ = (float)(1.0 - exp(-2.0 * kPi * lpHz / fs));
    fs_ = (float)fs; init_ = false; f_ = 0; slope_ = 0; gain_ = 1.0f; offset_ = 0.0f;
  }

  // force = gain * G[microsiemens] + offset. Default gain 1, offset 0 (so force() is G in uS).
  void setCalibration(float gain, float offset) { gain_ = gain; offset_ = offset; }
  // Two known loads: (G1_uS, F1) and (G2_uS, F2).
  bool calibrate2(float g1, float f1, float g2, float f2) {
    if (fabsf(g2 - g1) < 1e-9f) return false;
    gain_ = (f2 - f1) / (g2 - g1); offset_ = f1 - gain_ * g1; return true;
  }

  // Conductance in microsiemens from a raw ADC reading.
  float conductanceUs(float counts) const {
    if (counts < 1.0f) counts = 1.0f;
    if (counts > full_ - 1.0f) counts = full_ - 1.0f;
    const float rf = top_ ? rFixed_ * (full_ / counts - 1.0f) : rFixed_ * counts / (full_ - counts);
    return 1.0e6f / rf;
  }

  // Feed one reading (average several ADC reads per call if you can). Returns smoothed force.
  float process(float counts) {
    const float f = gain_ * conductanceUs(counts) + offset_;
    if (!init_) { f_ = f; init_ = true; slope_ = 0; return f_; }
    const float prev = f_;
    f_ += a_ * (f - f_);
    slope_ += a_ * ((f_ - prev) * fs_ - slope_);
    return f_;
  }
  float force() const { return f_; }
  float slope() const { return slope_; }               // force units per second

private:
  float full_ = 4095.0f, rFixed_ = 10000.0f, a_ = 0.1f, fs_ = 125.0f;
  float f_ = 0, slope_ = 0, gain_ = 1.0f, offset_ = 0.0f;
  bool top_ = true, init_ = false;
};

enum ContactState : uint8_t { CONTACT_NONE = 0, CONTACT_OK = 1, CONTACT_HIGH = 2 };

// Hysteresis state machine on force (units of FsrChannel::force()).
//   NONE -> OK   when force rises above onForce;   OK -> NONE when it falls below offForce (< onForce)
//   OK -> HIGH   when force exceeds highForce;     HIGH -> OK when it falls below highForce - hyst
// stable() is true after |slope| has stayed below slopeMax for holdSamples in a row: the contact
// pressure is stable. This alone does not prove that an amplitude change is due to perfusion.
class ContactMonitor {
public:
  float onForce = 1.0f, offForce = 0.7f, highForce = 50.0f, hyst = 2.0f;
  float slopeMax = 1.0f; int holdSamples = 125;

  ContactState update(float force, float slope) {
    if (!isfinite(force) || !isfinite(slope) || force < offForce) {
      state_ = CONTACT_NONE; quiet_ = 0; return state_;
    }
    switch (state_) {
      case CONTACT_NONE:
        if (force > highForce) state_ = CONTACT_HIGH;
        else if (force > onForce) state_ = CONTACT_OK;
        break;
      case CONTACT_OK:
        if (force < offForce) state_ = CONTACT_NONE;
        else if (force > highForce) state_ = CONTACT_HIGH;
        break;
      case CONTACT_HIGH:
        if (force < highForce - hyst) state_ = CONTACT_OK;
        break;
    }
    if (state_ != CONTACT_OK || fabsf(slope) > slopeMax) quiet_ = 0; else if (quiet_ < 1000000) quiet_++;
    return state_;
  }
  ContactState state() const { return state_; }
  bool stable() const { return quiet_ >= holdSamples; }
  void reset() { state_ = CONTACT_NONE; quiet_ = 0; }
private:
  ContactState state_ = CONTACT_NONE; int quiet_ = 0;
};

}  // namespace ppgnr

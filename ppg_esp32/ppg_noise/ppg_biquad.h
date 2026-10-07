// ppg_biquad.h - Butterworth low-pass / high-pass / band-pass designed AT RUN TIME.
//
//  Replaces the Python-generated coefficient tables: any sampling rate and cutoff,
//  no regeneration step. Even orders 2..8 as cascaded second-order sections (SOS),
//  transposed direct form II, float state (design is done in double).
//
//  Handles: baseline wander / temperature drift / respiration (high-pass),
//           broadband noise, ADC quantisation noise, EMI above the pulse band (low-pass).
//  Causal => adds phase delay (not zero-phase). A high-pass corner close to the heart
//  rate distorts the pulse shape: a lower corner may reduce distortion but does not guarantee notch preservation.
//
//  prime(x0) sets the state to the steady-state response for a constant input, so the
//  start-up transient is removed (high-pass output starts at 0 instead of ringing).
#pragma once
#include <math.h>

namespace ppgnr {

static const double kPi = 3.14159265358979323846;

struct BiquadCoeffs { float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0; };   // a0 normalised to 1

inline BiquadCoeffs biquadLowpass(double fs, double fc, double Q) {
  const double w = 2 * kPi * fc / fs, c = cos(w), s = sin(w), al = s / (2 * Q), a0 = 1 + al;
  BiquadCoeffs k;
  k.b0 = (float)(((1 - c) / 2) / a0); k.b1 = (float)((1 - c) / a0); k.b2 = k.b0;
  k.a1 = (float)((-2 * c) / a0);      k.a2 = (float)((1 - al) / a0);
  return k;
}
inline BiquadCoeffs biquadHighpass(double fs, double fc, double Q) {
  const double w = 2 * kPi * fc / fs, c = cos(w), s = sin(w), al = s / (2 * Q), a0 = 1 + al;
  BiquadCoeffs k;
  k.b0 = (float)(((1 + c) / 2) / a0); k.b1 = (float)((-(1 + c)) / a0); k.b2 = k.b0;
  k.a1 = (float)((-2 * c) / a0);      k.a2 = (float)((1 - al) / a0);
  return k;
}

struct Biquad {
  BiquadCoeffs c;
  float z1 = 0, z2 = 0;
  void set(const BiquadCoeffs& k) { c = k; z1 = z2 = 0; }
  void reset() { z1 = z2 = 0; }
  float dcGain() const { return (c.b0 + c.b1 + c.b2) / (1.0f + c.a1 + c.a2); }
  float process(float x) {
    const float y = c.b0 * x + z1;
    z1 = c.b1 * x - c.a1 * y + z2;
    z2 = c.b2 * x - c.a2 * y;
    return y;
  }
  // Steady state for a constant input x; returns the (constant) output.
  float prime(float x) {
    const float y = dcGain() * x;
    z1 = y - c.b0 * x;
    z2 = c.b2 * x - c.a2 * y;
    return y;
  }
};

// Butterworth of even ORDER (2,4,6,8) as ORDER/2 biquads.
template <int ORDER>
struct ButterFilter {
  static_assert(ORDER >= 2 && ORDER <= 8 && (ORDER % 2) == 0, "ORDER must be 2, 4, 6 or 8");
  static const int N = ORDER / 2;
  Biquad s[N];

  // Returns false (and leaves the filter as a pass-through) if fc is not in (0, fs/2).
  bool begin(double fs, double fc, bool highpass) {
    for (int i = 0; i < N; i++) s[i].set(BiquadCoeffs());
    if (!(fc > 0.0 && fc < fs / 2.0)) return false;
    for (int k = 1; k <= N; k++) {
      const double Q = 1.0 / (2.0 * cos(kPi * (2 * k - 1) / (2.0 * ORDER)));
      s[k - 1].set(highpass ? biquadHighpass(fs, fc, Q) : biquadLowpass(fs, fc, Q));
    }
    return true;
  }
  void reset() { for (int i = 0; i < N; i++) s[i].reset(); }
  float process(float x) { for (int i = 0; i < N; i++) x = s[i].process(x); return x; }
  float prime(float x) { for (int i = 0; i < N; i++) x = s[i].prime(x); return x; }
};

// High-pass (order HP) followed by low-pass (order LP). Set a cutoff to 0 to bypass that side.
template <int HP = 2, int LP = 4>
struct ButterBandpass {
  ButterFilter<HP> hp; ButterFilter<LP> lp;
  bool useHp = false, useLp = false;
  bool begin(double fs, double fLow, double fHigh) {
    bool ok = true;
    useHp = fLow  > 0.0; if (useHp) ok &= hp.begin(fs, fLow, true);
    useLp = fHigh > 0.0; if (useLp) ok &= lp.begin(fs, fHigh, false);
    if (!ok) { useHp = useLp = false; }
    return ok;
  }
  void reset() { hp.reset(); lp.reset(); }
  float process(float x) { if (useHp) x = hp.process(x); if (useLp) x = lp.process(x); return x; }
  void prime(float x) { if (useHp) x = hp.prime(x); if (useLp) lp.prime(x); }
};

}  // namespace ppgnr

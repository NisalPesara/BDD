// ppg_noise.h - umbrella header + ready-made chain (every stage can be switched off).
//
//   raw --> Hampel (spikes) --> DC tracker --> [invert] --> high-pass
//   refs --> (delay to match Hampel) --> the SAME high-pass --------> [NLMS motion ANC]
//       --> low-pass --> Savitzky-Golay (smooth + VPG + APG) --> outputs
//
//  The ANC sits AFTER the high-pass, and the references go through an identical high-pass.
//  Reason (measured on a synthetic limb recording): if the canceller sees the PPG with its slow
//  drift still in it, the drift dominates the adaptation error and the canceller makes the signal
//  WORSE; filtering d and the references identically keeps the artifact path linear.
//
//  Each library is usable alone:
//    ppg_biquad.h  Butterworth HP/LP/BP at run time       (drift, respiration, broadband noise)
//    ppg_dc.h      DC tracker, AC/DC normalise, PI         (offset, pigmentation/thickness gain)
//    ppg_spike.h   Hampel outlier remover                  (EMI bursts, I2C/ADC glitches)
//    ppg_savgol.h  smoothing + aligned derivatives         (noise in VPG/APG)
//    ppg_anc.h     multi-reference NLMS                    (motion; accel and force references)
//    ppg_force.h   FSR conditioning + contact monitor      (contact force, gating)
//    ppg_sqi.h     signal-quality index                    (reject what cannot be repaired)
//
//  Delay: Hampel (W-1)/2 + Savitzky-Golay HALF samples are fixed; the Butterworth stages add
//  frequency-dependent group delay on top. The ANC references are delayed internally by the
//  Hampel delay so they stay aligned with the PPG. References are DC-tracked and high-pass filtered internally.
#pragma once
#include "ppg_biquad.h"
#include "ppg_dc.h"
#include "ppg_spike.h"
#include "ppg_savgol.h"
#include "ppg_anc.h"
#include "ppg_force.h"
#include "ppg_sqi.h"

namespace ppgnr {

struct DenoiseConfig {
  float fs = 125.0f;
  bool  invert = true;                // reflective MAX30102: IR falls when blood volume rises
  bool  useSpike = true;  float spikeK = 5.0f, spikeMinSigma = 2.0f;
  float dcHz = 0.05f;                 // DC tracker corner
  float hpHz = 0.15f;                 // high-pass corner (0 = off); lower corner may reduce notch distortion; validate on recordings
  float lpHz = 10.0f;                 // low-pass corner (0 = off)
  bool  useAnc = false;  float ancMu = 0.01f, ancEps = 1e-3f, ancPmin = 0.0f, ancLeak = 0.0f;
  bool  useSg = true;    int sgOrder = 3;
};

struct DenoiseOut {
  float ac = 0;          // DC-removed, sign-corrected input (before ANC and filters)
  float dc = 0;          // slow raw level (for perfusion index / SQI)
  float clean = 0;       // cleaned PPG
  float vpg = 0, apg = 0;// aligned derivatives (per second, per second^2), 0 if useSg is false
  bool  valid = false;   // false until the Hampel / Savitzky-Golay windows are full
};

template <int NREF = 9, int TAPS = 8, int SGHALF = 5, int HAMPEL_W = 7>
class PpgDenoiser {
public:
  static const int FIXED_DELAY = HAMPEL_W / 2 + SGHALF;     // samples (excludes IIR group delay)

  bool begin(const DenoiseConfig& c) {
    if (!(c.fs > 0) || !isfinite(c.fs) || !(c.dcHz > 0) || c.dcHz >= c.fs / 2 ||
        !(c.ancEps > 0) || c.ancMu < 0 || c.ancMu > 1 || c.ancPmin < 0 ||
        c.ancLeak < 0 || c.ancLeak >= 1) return false;
    cfg = c;
    bool ok = true;
    hampel_.reset(); hampel_.k = c.spikeK; hampel_.minSigma = c.spikeMinSigma;
    dc_.begin(c.fs, c.dcHz);
    for (int r = 0; r < NREF; ++r) dcR_[r].begin(c.fs, c.dcHz);
    useHp_ = c.hpHz > 0.0f; useLp_ = c.lpHz > 0.0f;
    if (useHp_) { ok &= hpD_.begin(c.fs, c.hpHz, true); for (int r = 0; r < NREF; r++) ok &= hpR_[r].begin(c.fs, c.hpHz, true); }
    if (useLp_) ok &= lp_.begin(c.fs, c.lpHz, false);
    refInit_ = false; refsPresent_ = false;
    anc_.reset(); anc_.mu = c.ancMu; anc_.eps = c.ancEps; anc_.pmin = c.ancPmin; anc_.leak = c.ancLeak;
    ok &= sg_.begin(c.fs, c.sgOrder);
    for (int i = 0; i < DLY; i++) for (int r = 0; r < NREF; r++) rdly_[i][r] = 0;
    rd_ = 0; n_ = 0;
    return ok;
  }

  // raw: ADC counts. refs: NREF finite motion reference samples (or nullptr). adapt=false freezes the ANC.
  DenoiseOut process(float raw, const float* refs = nullptr, bool adapt = true) {
    DenoiseOut o;
    const float x = cfg.useSpike ? hampel_.process(raw) : raw;
    float ac = dc_.process(x);
    o.dc = dc_.level();
    if (cfg.invert) ac = -ac;
    o.ac = ac;

    float e = useHp_ ? hpD_.process(ac) : ac;
    if (cfg.useAnc && !refs && refsPresent_) {
      anc_.reset(); refInit_ = false;
      for (int r = 0; r < NREF; ++r) dcR_[r].begin(cfg.fs, cfg.dcHz);
      for (int i = 0; i < DLY; ++i) for (int r = 0; r < NREF; ++r) rdly_[i][r] = 0;
      rd_ = 0;
    }
    refsPresent_ = (refs != nullptr);
    if (cfg.useAnc && refs) {
      float rr[NREF];
      if (cfg.useSpike && DLY > 0) {                         // keep references aligned with the delayed PPG
        for (int r = 0; r < NREF; r++) { rr[r] = rdly_[rd_][r]; rdly_[rd_][r] = refs[r]; }
        if (++rd_ >= DLY) rd_ = 0;
      } else {
        for (int r = 0; r < NREF; r++) rr[r] = refs[r];
      }
      for (int r = 0; r < NREF; ++r) rr[r] = dcR_[r].process(rr[r]);
      if (useHp_) {                                          // same filter as the PPG; prime on the first sample (gravity/tilt)
        if (!refInit_) { for (int r = 0; r < NREF; r++) hpR_[r].prime(rr[r]); refInit_ = true; }
        for (int r = 0; r < NREF; r++) rr[r] = hpR_[r].process(rr[r]);
      }
      e = anc_.process(e, rr, adapt);
    }
    const float b = useLp_ ? lp_.process(e) : e;
    n_++;
    if (cfg.useSg) {
      float s, v, a;
      const bool full = sg_.process(b, s, v, a);
      o.clean = s; o.vpg = v; o.apg = a;
      o.valid = full && (n_ > (unsigned long)HAMPEL_W);
    } else {
      o.clean = b; o.valid = n_ > (unsigned long)HAMPEL_W;
    }
    return o;
  }

  float energy() const { return anc_.energy(); }
  DenoiseConfig cfg;

private:
  static const int DLY = HAMPEL_W / 2;
  Hampel<HAMPEL_W> hampel_;
  DcTracker dc_, dcR_[NREF];
  ButterFilter<2> hpD_, hpR_[NREF];
  ButterFilter<4> lp_;
  bool useHp_ = false, useLp_ = false, refInit_ = false, refsPresent_ = false;
  Nlms<NREF, TAPS> anc_;
  SavGolDeriv<SGHALF> sg_;
  float rdly_[(DLY > 0) ? DLY : 1][NREF];
  int rd_ = 0; unsigned long n_ = 0;
};

}  // namespace ppgnr

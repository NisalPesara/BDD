// Revised from the supplied Libraries/wave_detection.h.
// Onset-to-onset segments, one beat late. SG PPG/VPG/APG share one time center.
// All areas share a linear onset-to-onset baseline. Core values are research
// pulse features, not blood-flow rate. Optional APG ratios are exploratory.
#pragma once
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include "pipeline.h"
#include "config.h"

#include "m09_auc_calculator.h"

struct PulseFeatures {
  bool     valid = false;
  uint64_t onsetAbs = 0;             // absolute sample index of the onset
  uint64_t onset_us = 0;
  int      len = 0;                  // samples from onset to next onset (inclusive)
  int      onset = 0, peak = 0, notch = -1, dia = -1, offset = 0;
  int      u = -1, v = -1, w = -1, a = -1, b = -1, c = -1, d = -1, e = -1, f = -1;
  uint8_t  notchType = 0;            // 0 none, 1 inflection (VPG 'w'), 2 true PPG minimum
  float    bpm = NAN, ppi = NAN, amp = NAN, pi = NAN, crest = NAN, deltaT = NAN, pw50 = NAN;
  float    area = NAN, sysArea = NAN, diaArea = NAN, ipa = NAN, aix = NAN;
  float    bOverA = NAN, cOverA = NAN, dOverA = NAN, eOverA = NAN;
  float    bMinusEOverA = NAN, bMinusCMinusDOverA = NAN;
};

class WaveAnalyzer {
public:
  static constexpr int N      = 1024;                 // ring size (power of 2) = 8.2 s at 125 Hz
  static constexpr int M      = N - 1;
  static constexpr int MAXSEG = 2 * PPG_FS + 2;       // longest accepted pulse (2 s)

  PulseFeatures last;
  int           reportNum = 0;

  void begin() {
    n_ = 0; npk_ = 0; reportNum = 0;
    for (int i = 0; i < N; i++) bad_[i] = 1;
    last = PulseFeatures();
  }

  WaveAnalyzer() { begin(); }

  // Call once per sample. Returns true when a new pulse was analysed.
  bool process(const PpgSample& s) {
    const uint64_t cur = n_++;
    last.valid = false;
    const int k = (int)(cur & M);
    ppg_[k] = s.ppg; vpg_[k] = s.vpg; apg_[k] = s.apg;
    time_[k] = s.aligned_us; dcRing_[k] = s.dc;
    if (s.sqi.fresh && !s.sqi.good) {
      for (int j = 0; j < SQI_WINDOW_S * PPG_FS && j < N; ++j)
        bad_[(cur - j) & M] = 1;
    }
    bad_[k] = (s.valid && s.sqi_good) ? 0 : 1;

    if (!s.beat || !s.valid || cur < 1) return false;

    pk_[0] = pk_[1]; pk_[1] = pk_[2]; pk_[2] = cur - 1;   // m08 reports the peak one sample late
    if (npk_ < 3) npk_++;
    if (npk_ < 3) return false;
    return analyze();
  }

private:
  static constexpr float DT = 1.0f / PPG_FS;

  float    ppg_[N], vpg_[N], apg_[N];
  uint8_t  bad_[N];
  uint64_t time_[N]; float dcRing_[N];
  float    sp_[MAXSEG], sv_[MAXSEG], sa_[MAXSEG];     // local copy of the segment
  uint64_t n_ = 0, pk_[3] = {0, 0, 0};
  int      npk_ = 0, L_ = 0;

  // ---------- helpers ----------
  // Onset of the beat whose systolic peak is pb (previous peak pa):
  //   u = point of maximum upstroke velocity in the second half of the interval,
  //   onset = first sample walking back from u where the PPG stops falling (VPG <= 0).
  // This is robust to the post-systolic undershoot that a 0.5 Hz high-pass leaves, where
  // the global minimum between two peaks is NOT the foot of the pulse.
  uint64_t findOnset(uint64_t pa, uint64_t pb) const {
    const uint64_t mid = pa + (pb - pa) / 2u;
    uint64_t u = mid;
    for (uint64_t i = mid; i <= pb; i++)
      if (vpg_[i & M] > vpg_[u & M]) u = i;
    uint64_t i = u;
    while (i > mid && vpg_[i & M] > 0.0f) i--;
    if (vpg_[i & M] > 0.0f) {                         // no slope sign change: fall back to the minimum
      i = mid;
      for (uint64_t j = mid; j <= u; j++) if (ppg_[j & M] < ppg_[i & M]) i = j;
    }
    return i;
  }

  // First confirmed extremum of x after index s: the running candidate is accepted
  // once the signal retreats from it by more than thr. Returns -1 if none before lim.
  static int zig(const float* x, int s, int lim, bool wantMax, float thr) {
    if (s < 0 || s + 1 > lim) return -1;
    int cand = s + 1;
    for (int i = s + 1; i <= lim; i++) {
      if (wantMax) { if (x[i] > x[cand]) cand = i; else if (x[cand] - x[i] > thr) return cand; }
      else         { if (x[i] < x[cand]) cand = i; else if (x[i] - x[cand] > thr) return cand; }
    }
    return -1;
  }

  float area(int from, int to) const {
    return PulseAuc::integrate(sp_, L_, PPG_FS, from, to);
  }

  static float cross(const float* x, int i, float level) {   // interpolated crossing between i-1 and i
    const float den = x[i] - x[i - 1];
    return (i - 1) + ((fabsf(den) > 1e-12f) ? (level - x[i - 1]) / den : 0.0f);
  }

  // ---------- main analysis of one onset-to-onset segment ----------
  bool analyze() {
    if (pk_[2] <= pk_[1] || pk_[1] <= pk_[0] || pk_[2] - pk_[0] >= N - 2) return false;
    const int i1 = int(pk_[1] - pk_[0]), i2 = int(pk_[2] - pk_[1]);
    if (i1 < 1 || i2 < 1) return false;            // ring no longer holds it
    const float ratio = (float)i2 / (float)i1;
    if (ratio < 0.6f || ratio > 1.6f) return false;               // missed / extra beat

    const uint64_t oA = findOnset(pk_[0], pk_[1]);
    const uint64_t oB = findOnset(pk_[1], pk_[2]);
    const int L = (int)(oB - oA) + 1;
    if (L < (int)(0.3f * PPG_FS) + 1 || L > MAXSEG) return false;
    for (int i = 0; i < L; i++) if (bad_[(oA + i) & M]) return false;   // gap or poor SQI
    L_ = L;
    for (int i = 0; i < L; i++) {
      const int k = (int)((oA + i) & M);
      sp_[i] = ppg_[k]; sv_[i] = vpg_[k]; sa_[i] = apg_[k];
    }

    PulseFeatures f;
    f.onset_us = time_[oA & M];
    f.onsetAbs = oA; f.len = L; f.onset = 0; f.offset = L - 1;

    // --- PPG: systolic peak and amplitude ---
    int pk = 0;
    for (int i = 1; i < L; i++) if (sp_[i] > sp_[pk]) pk = i;
    if (pk <= 0 || pk >= L - 1) return false;
    const float baselineSlope = (sp_[L-1] - sp_[0]) / (L-1);
    const float amp = sp_[pk] - (sp_[0] + baselineSlope * pk);
    if (amp <= 0) return false;
    f.peak = pk; f.amp = amp;

    // --- VPG: u (max upstroke), v (max downstroke), w (first local max after v) ---
    int u = 0;
    for (int i = 1; i <= pk; i++) if (sv_[i] > sv_[u]) u = i;
    int v = u;
    for (int i = u + 1; i < L; i++) if (sv_[i] < sv_[v]) v = i;
    f.u = u; f.v = v;
    f.w = zig(sv_, v, L - 1, true, 0.05f * sv_[u]);

    // --- notch: first PPG minimum (VPG up-crossing) after v that is followed by a diastolic peak ---
    int notch = -1, dia = -1; uint8_t nType = 0;
    for (int i = v + 1; i < L - 1; i++) {
      if (sv_[i - 1] < 0.0f && sv_[i] >= 0.0f) {
        const int cand = i - 1;
        if (cand > (int)(0.85f * (L - 1))) break;                    // ripples near the next foot are not a notch
        for (int j = i; j < L - 1; j++) {
          if (sv_[j - 1] >= 0.0f && sv_[j] < 0.0f) {
            if (sp_[j - 1] - sp_[cand] > 0.02f * amp) { notch = cand; dia = j - 1; nType = 2; }
            break;
          }
        }
        if (nType) break;
      }
    }
    if (!nType && f.w >= 0 && f.w < L - 2) { notch = f.w; nType = 1; }   // inflection fallback
    f.notch = notch; f.dia = dia; f.notchType = nType;

    // --- APG: a, then b c d e f as alternating extrema ---
    int a = 0;
    for (int i = 1; i <= pk; i++) if (sa_[i] > sa_[a]) a = i;
    if (sa_[a] > 0.0f) {
      const float thr = 0.05f * sa_[a];
      f.a = a;
      f.b = zig(sa_, f.a, L - 1, false, thr);
      f.c = zig(sa_, f.b, L - 1, true,  thr);
      f.d = zig(sa_, f.c, L - 1, false, thr);
      f.e = zig(sa_, f.d, L - 1, true,  thr);
      f.f = zig(sa_, f.e, L - 1, false, thr);
    }

    // --- timing ---
    f.ppi   = (L - 1) * DT;
    f.bpm   = 60.0f / f.ppi;
    f.crest = pk * DT;                                              // onset -> systolic peak
    if (dia >= 0) {
      f.deltaT = (dia - pk) * DT;                                   // systolic peak -> diastolic peak

    }

    // --- amplitude / area features (shared linear onset-to-onset baseline) ---
    float dcMean = 0;
    for (int i = 0; i < L; ++i) dcMean += dcRing_[(oA + i) & M];
    dcMean /= L;
    f.pi = dcMean > 1e-6f ? 100.0f * amp / dcMean : NAN;
    {
      float detrended[MAXSEG];
      for (int i = 0; i < L; ++i) detrended[i] = sp_[i] - (sp_[0] + baselineSlope * i);
      const float level = 0.5f * amp;
      int l = -1, r = -1;
      for (int i = 1; i <= pk; i++) if (detrended[i] >= level) { l = i; break; }
      for (int i = pk + 1; i < L; i++) if (detrended[i] <= level) { r = i; break; }
      if (l > 0 && r > 0) f.pw50 = (cross(detrended, r, level) - cross(detrended, l, level)) * DT;
    }
    f.area = area(0, L - 1);
    if (notch > 0) {
      f.sysArea = area(0, notch);
      f.diaArea = area(notch, L - 1);
      if (f.sysArea > 0.0f && f.diaArea > 0.0f) f.ipa = f.sysArea / f.diaArea;   // NaN if undershoot below onset makes an area negative
    }
    if (dia >= 0 && notch >= 0) f.aix = (sp_[dia] - sp_[notch]) / amp;   // (diastolic peak - notch)/amplitude

    // --- SDPTG ratios ---
    const float aV = (f.a >= 0) ? sa_[f.a] : 0.0f;
    if (aV > 1e-6f) {
      if (f.b >= 0) f.bOverA = sa_[f.b] / aV;
      if (f.c >= 0) f.cOverA = sa_[f.c] / aV;
      if (f.d >= 0) f.dOverA = sa_[f.d] / aV;
      if (f.e >= 0) f.eOverA = sa_[f.e] / aV;
      if (f.b >= 0 && f.e >= 0) f.bMinusEOverA = (sa_[f.b] - sa_[f.e]) / aV;
      if (f.b >= 0 && f.c >= 0 && f.d >= 0) f.bMinusCMinusDOverA = (sa_[f.b] - sa_[f.c] - sa_[f.d]) / aV;

    }

    f.valid = true;
    last = f;
    reportNum++;
    return true;
  }

};

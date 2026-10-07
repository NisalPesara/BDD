// ======================================================================
//  WAVE-PROPERTIES FEATURE EXTRACTION (per-pulse analysis) - revised
//
//  Segmentation (fixes the "peak boundary treated as onset" problem):
//    Method 8 flags the systolic PEAK (one sample late). We keep a ring buffer
//    of ppg/vpg/apg and the last three peak positions. When peak k+1 arrives:
//        onsetA = foot of the beat whose peak is peak k   (max-upstroke point, walked back to VPG<=0)
//        onsetB = foot of the beat whose peak is peak k+1
//    and analyse the segment [onsetA .. onsetB]:
//        onset -> upstroke -> systolic peak -> notch -> diastolic wave -> next onset
//    Every report is therefore delayed by one beat.
//
//  A segment is rejected if: any sample in it failed valid/SQI, the two peak
//  intervals differ by more than 0.6x..1.6x (missed/extra beat), the period is
//  outside 0.3..2 s, or the peak is not strictly inside the segment.
//
//  Fiducials are found as ordered local extrema with a prominence threshold:
//    notch  = first PPG minimum after the max-downstroke point (VPG upward zero
//             crossing) that is followed by a real diastolic peak.
//             Fallback: VPG local maximum 'w' (inflection, notchType 1).
//    APG    = a (max), then b, c, d, e, f as alternating zig-zag extrema.
//  Time resolution is 1/PPG_FS (8 ms at 125 Hz). VPG sample n is centred at
//  n-0.5 and APG sample n at n-1, so VPG/APG indices can sit 1 sample after
//  the PPG feature they belong to.
//
//  Output: one CSV line per pulse (set WAVE_VERBOSE 1 for fiducial indices).
//  Built so it also compiles on a desktop (no Arduino.h) for testing.
// ======================================================================
#pragma once
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#ifdef ARDUINO
#include <Arduino.h>
#endif
#include "pipeline.h"
#include "config.h"

// Declared in the sketch - posture accelerometer readings
extern float lastAccelX, lastAccelY, lastAccelZ;

#ifndef WAVE_VERBOSE
#define WAVE_VERBOSE 0
#endif
#ifndef WAVE_BODY_HEIGHT_M
#define WAVE_BODY_HEIGHT_M 1.7f      // default used for the stiffness index
#endif

struct PulseFeatures {
  bool     valid = false;
  uint32_t onsetAbs = 0;             // absolute sample index of the onset
  int      len = 0;                  // samples from onset to next onset (inclusive)
  int      onset = 0, peak = 0, notch = -1, dia = -1, offset = 0;
  int      u = -1, v = -1, w = -1, a = -1, b = -1, c = -1, d = -1, e = -1, f = -1;
  uint8_t  notchType = 0;            // 0 none, 1 inflection (VPG 'w'), 2 true PPG minimum
  float    bpm = NAN, ppi = NAN, amp = NAN, pi = NAN, crest = NAN, deltaT = NAN, pw50 = NAN;
  float    area = NAN, sysArea = NAN, diaArea = NAN, ipa = NAN, aix = NAN, lasi = NAN;
  float    bOverA = NAN, cOverA = NAN, dOverA = NAN, eOverA = NAN;
  float    bMinusEOverA = NAN, bMinusCMinusDOverA = NAN, agingIdx = NAN;
};

class WaveAnalyzer {
public:
  static constexpr int N      = 1024;                 // ring size (power of 2) = 8.2 s at 125 Hz
  static constexpr int M      = N - 1;
  static constexpr int MAXSEG = 2 * PPG_FS + 2;       // longest accepted pulse (2 s)

  bool          printEnabled = true;
  PulseFeatures last;
  int           reportNum = 0;

  void begin() {
    n_ = 0; npk_ = 0; reportNum = 0; dcInit_ = false; dc_ = 0;
    for (int i = 0; i < N; i++) bad_[i] = 1;
    last = PulseFeatures();
  }

  WaveAnalyzer() { begin(); }

  // Call once per sample. Returns true when a new pulse was analysed.
  bool process(const PpgSample& s) {
    const uint32_t cur = n_++;
    const int k = (int)(cur & M);
    ppg_[k] = s.ppg; vpg_[k] = s.vpg; apg_[k] = s.apg;
    bad_[k] = (s.valid && s.sqi_good) ? 0 : 1;

    if (s.valid) {                                    // slow DC tracker (~0.02 Hz) for PI
      if (!dcInit_) { dc_ = s.raw; dcInit_ = true; }
      else dc_ += 0.001f * (s.raw - dc_);
    }
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
  float    sp_[MAXSEG], sv_[MAXSEG], sa_[MAXSEG];     // local copy of the segment
  uint32_t n_ = 0, pk_[3] = {0, 0, 0};
  int      npk_ = 0, L_ = 0;
  float    dc_ = 0; bool dcInit_ = false;

  // ---------- helpers ----------
  // Onset of the beat whose systolic peak is pb (previous peak pa):
  //   u = point of maximum upstroke velocity in the second half of the interval,
  //   onset = first sample walking back from u where the PPG stops falling (VPG <= 0).
  // This is robust to the post-systolic undershoot that a 0.5 Hz high-pass leaves, where
  // the global minimum between two peaks is NOT the foot of the pulse.
  uint32_t findOnset(uint32_t pa, uint32_t pb) const {
    const uint32_t mid = pa + (pb - pa) / 2u;
    uint32_t u = mid;
    for (uint32_t i = mid; i <= pb; i++)
      if (vpg_[i & M] > vpg_[u & M]) u = i;
    uint32_t i = u;
    while (i > mid && vpg_[i & M] > 0.0f) i--;
    if (vpg_[i & M] > 0.0f) {                         // no slope sign change: fall back to the minimum
      i = mid;
      for (uint32_t j = mid; j <= u; j++) if (ppg_[j & M] < ppg_[i & M]) i = j;
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

  float area(int from, int to) const {                // trapezoid above the ONSET value
    if (from < 0 || to <= from || to >= L_) return NAN;
    const float base = sp_[0];
    float s = 0;
    for (int i = from; i < to; i++) s += 0.5f * ((sp_[i] - base) + (sp_[i + 1] - base));
    return s * DT;
  }

  static float cross(const float* x, int i, float level) {   // interpolated crossing between i-1 and i
    const float den = x[i] - x[i - 1];
    return (i - 1) + ((fabsf(den) > 1e-12f) ? (level - x[i - 1]) / den : 0.0f);
  }

  // ---------- main analysis of one onset-to-onset segment ----------
  bool analyze() {
    const int i1 = (int)(pk_[1] - pk_[0]), i2 = (int)(pk_[2] - pk_[1]);
    if ((int)(pk_[2] - pk_[0]) >= N - 2) return false;            // ring no longer holds it
    const float ratio = (float)i2 / (float)i1;
    if (ratio < 0.6f || ratio > 1.6f) return false;               // missed / extra beat

    const uint32_t oA = findOnset(pk_[0], pk_[1]);
    const uint32_t oB = findOnset(pk_[1], pk_[2]);
    const int L = (int)(oB - oA) + 1;
    if (L < (int)(0.3f * PPG_FS) + 1 || L > MAXSEG) return false;
    for (int i = 0; i < L; i++) if (bad_[(oA + i) & M]) return false;   // gap or poor SQI
    L_ = L;
    for (int i = 0; i < L; i++) {
      const int k = (int)((oA + i) & M);
      sp_[i] = ppg_[k]; sv_[i] = vpg_[k]; sa_[i] = apg_[k];
    }

    PulseFeatures f;
    f.onsetAbs = oA; f.len = L; f.onset = 0; f.offset = L - 1;

    // --- PPG: systolic peak and amplitude ---
    int pk = 0;
    for (int i = 1; i < L; i++) if (sp_[i] > sp_[pk]) pk = i;
    if (pk <= 0 || pk >= L - 1) return false;
    const float amp = sp_[pk] - sp_[0];
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
      if (f.deltaT > 1e-6f) f.lasi = WAVE_BODY_HEIGHT_M / f.deltaT;
    }

    // --- amplitude / area features (single baseline = onset value) ---
    f.pi = (fabsf(dc_) > 1e-6f) ? 100.0f * amp / fabsf(dc_) : NAN;
    {
      const float level = sp_[0] + 0.5f * amp;
      int l = -1, r = -1;
      for (int i = 1; i <= pk; i++) if (sp_[i] >= level) { l = i; break; }
      for (int i = pk + 1; i < L; i++) if (sp_[i] <= level) { r = i; break; }
      if (l > 0 && r > 0) f.pw50 = (cross(sp_, r, level) - cross(sp_, l, level)) * DT;
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
      if (f.b >= 0 && f.c >= 0 && f.d >= 0 && f.e >= 0)
        f.agingIdx = (sa_[f.b] - sa_[f.c] - sa_[f.d] - sa_[f.e]) / aV;
    }

    f.valid = true;
    last = f;
    reportNum++;
    if (printEnabled) print(f);
    return true;
  }

  // ---------- compact output: one CSV line per pulse, built in a buffer, sent once ----------
  static void emit(const char* s) {
#ifdef ARDUINO
    Serial.print(s);
#else
    fputs(s, stdout);
#endif
  }

  void print(const PulseFeatures& f) const {
    char b[512];
    if (reportNum == 1) {
      emit("n,bpm,ppi_s,amp,pi_pct,crest_s,deltaT_s,pw50_s,area,sysArea,diaArea,ipa,aix,lasi,"
           "b/a,c/a,d/a,e/a,(b-e)/a,(b-c-d)/a,aging,notchType,ax,ay,az\n");
    }
    snprintf(b, sizeof b,
      "%d,%.1f,%.3f,%.4g,%.3g,%.3f,%.3f,%.3f,%.4g,%.4g,%.4g,%.3f,%.3f,%.2f,"
      "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%.2f,%.2f,%.2f\n",
      reportNum, (double)f.bpm, (double)f.ppi, (double)f.amp, (double)f.pi,
      (double)f.crest, (double)f.deltaT, (double)f.pw50,
      (double)f.area, (double)f.sysArea, (double)f.diaArea, (double)f.ipa, (double)f.aix, (double)f.lasi,
      (double)f.bOverA, (double)f.cOverA, (double)f.dOverA, (double)f.eOverA,
      (double)f.bMinusEOverA, (double)f.bMinusCMinusDOverA, (double)f.agingIdx, (int)f.notchType,
      (double)lastAccelX, (double)lastAccelY, (double)lastAccelZ);
    emit(b);
#if WAVE_VERBOSE
    snprintf(b, sizeof b,
      "  idx(rel. to onset) peak=%d notch=%d dia=%d off=%d | u=%d v=%d w=%d | a=%d b=%d c=%d d=%d e=%d f=%d\n",
      f.peak, f.notch, f.dia, f.offset, f.u, f.v, f.w, f.a, f.b, f.c, f.d, f.e, f.f);
    emit(b);
#endif
  }
};

inline WaveAnalyzer& waveAnalyzer() { static WaveAnalyzer a; return a; }
inline void waveDetectionProcess(const PpgSample& s) { waveAnalyzer().process(s); }

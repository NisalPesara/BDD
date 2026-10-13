// ======================================================================
//  WAVE-PROPERTIES FEATURE EXTRACTION (per-pulse analysis)
//  Segmentation from updated_libraries-manuth, with the 4.37 fixes.
//
//  m08 flags each systolic PEAK (one sample late, on the 0.5 Hz detection copy); each peak is
//  refined to the true PPG maximum once the samples after it exist. A ring buffer keeps
//  ppg/vpg/apg/dc and the last four peak positions. When peak k+2 arrives:
//      onsetA = foot of the beat whose peak is peak k
//      onsetB = foot of the beat whose peak is peak k+1
//  and the complete pulse [onsetA .. onsetB] is analysed:
//      onset -> upstroke -> systolic peak -> notch -> diastolic wave -> next onset
//  so every report is about two beats late (~1.5 s). Segment ends = pulse feet (minimum before the
//  upstroke); heart rate and crest time use the intersecting-tangent onset (stable timing).
//
//  A pulse is rejected if: any sample in it failed valid/SQI (motion, pressure change, poor
//  signal), the two peak intervals differ by more than 0.6x..1.6x (missed/extra beat), the period
//  is outside 0.3..2 s, or the peak is not strictly inside the segment.
//  Pulses that overlap SMALL motion skip the 5 s SQI window (motion spoils whole windows) and are
//  judged beat by beat instead: shape correlation with a template learned from still pulses
//  >= TEMPLATE_MIN_CORR and amplitude within TEMPLATE_AMP_MIN..MAX of the template.
//
//  Fiducials (ordered extrema with a prominence test):
//    notch = first PPG minimum after the max-downstroke point (VPG upward zero crossing) that is
//            followed by a real diastolic peak (notchType 2). Fallback: VPG local max 'w'
//            (inflection, notchType 1). None found: notchType 0.
//    APG   = a (max before the systolic peak), then b, c, d, e, f as alternating extrema.
//  Amplitude and all areas use ONE straight baseline from this onset to the next onset.
//  Time resolution 1/PPG_FS = 8 ms.
//
//  Features useful for LIMB BLOOD FLOW (compare against a baseline, see m10):
//    amp, pi, area (AUC), upSlope (VPG max = how fast blood volume rises), crest time.
//  Under partial occlusion: amplitude, PI, area and upSlope fall, crest time grows, the
//  dicrotic notch disappears (notchType drops to 1 or 0). With no inflow: no pulse at all.
// ======================================================================
#pragma once
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include "config.h"
#include "pipeline.h"
#include "m09_auc_calculator.h"

struct PulseFeatures {
  bool     valid = false;
  uint32_t onset_ms = 0;            // time of the pulse onset (filter delay already removed)
  uint32_t onsetAbs = 0;            // absolute sample index of the onset
  int      len = 0;                 // samples from onset to next onset (inclusive)
  int      onset = 0, peak = 0, notch = -1, dia = -1, offset = 0;   // indices relative to the onset
  int      u = -1, v = -1, w = -1, a = -1, b = -1, c = -1, d = -1, e = -1, f = -1;
  uint8_t  notchType = 0;           // 0 none, 1 inflection (VPG 'w'), 2 true PPG minimum
  bool     duringMotion = false;   // pulse overlapped small motion and passed the template check
  float    tplCorr = NAN;          // shape correlation with the still-pulse template
  float    baseCorr = NAN;         // shape correlation with the template frozen at baseline capture
  float    bpm = NAN, ppi = NAN, amp = NAN, pi = NAN, crest = NAN, deltaT = NAN, pw50 = NAN;
  float    area = NAN, sysArea = NAN, diaArea = NAN, ipa = NAN, aix = NAN, lasi = NAN;
  float    upSlope = NAN;           // VPG at 'u' = max upstroke velocity (a.u./s)
  float    bOverA = NAN, cOverA = NAN, dOverA = NAN, eOverA = NAN;
  float    bMinusEOverA = NAN, bMinusCMinusDOverA = NAN, agingIdx = NAN;
};

class WaveAnalyzer {
public:
  static constexpr int N      = 1024;                 // ring size (power of 2) = 8.2 s at 125 Hz
  static constexpr int M      = N - 1;
  static constexpr int MAXSEG = 2 * PPG_FS + 2;       // longest accepted pulse (2 s = 30 bpm)

  PulseFeatures last;
  int           reportNum = 0;
  uint32_t      rejectedMotion = 0;                  // motion pulses rejected by the template check
  bool templateReady() const { return tplCount_ >= TEMPLATE_MIN_PULSES; }
  // Freeze the current still-pulse template as the clean BASELINE waveform (kept across signal resets).
  bool freezeBaselineTemplate() {
    if (tplCount_ < 1) return false;
    for (int i = 0; i < TEMPLATE_POINTS; i++) baseTpl_[i] = tpl_[i];
    baseTplSet_ = true; return true;
  }
  void clearBaselineTemplate() { baseTplSet_ = false; }
  bool baselineTemplateSet() const { return baseTplSet_; }

  WaveAnalyzer() { begin(); }

  void begin() {
    n_ = 0; npk_ = 0; reportNum = 0;
    for (int i = 0; i < N; i++) { bad_[i] = 1; hard_[i] = 1; mot_[i] = 0; }
    tplCount_ = 0; tplAmp_ = 0; rejectedMotion = 0;
    last = PulseFeatures();
  }

  // Call once per sample. Returns true when a new pulse was analysed (result in `last`).
  bool process(const PpgSample& s) {
    const uint32_t cur = n_++;
    last.valid = false;
    const int k = (int)(cur & M);
    ppg_[k] = s.ppg; vpg_[k] = s.vpg; apg_[k] = s.apg;
    time_[k] = s.aligned_ms; dcRing_[k] = s.dc;
    if (s.sqi.fresh && !s.sqi.good)                       // a bad SQI window invalidates its whole span
      for (int j = 0; j < SQI_WINDOW_S * PPG_FS && j < N; ++j) bad_[(cur - j) & M] = 1;
    bad_[k] = (s.valid && s.sqi_good) ? 0 : 1;
    hard_[k] = (s.valid && s.motionQuiet && s.forceStable && !s.clipped) ? 0 : 1;   // never usable
    mot_[k] = s.smallMotion ? 1 : 0;

    if (!s.beat || !s.valid || cur < 1) return false;
    // m08 reports the peak one sample late, found on the 0.5 Hz detection copy (slightly earlier
    // than the real PPG maximum). Refine the PREVIOUS peak now that the samples after it exist.
    if (npk_ >= 1) pk_[3] = refinePeak(pk_[3], cur);
    pk_[0] = pk_[1]; pk_[1] = pk_[2]; pk_[2] = pk_[3]; pk_[3] = cur - 1;
    if (npk_ < 4) npk_++;
    if (npk_ < 4) return false;
    return analyze();                                     // pulse between peaks pk_[1] and pk_[2]
  }

private:
  static constexpr float DT = 1.0f / PPG_FS;

  float    ppg_[N], vpg_[N], apg_[N], dcRing_[N];
  uint32_t time_[N];
  uint8_t  bad_[N], hard_[N], mot_[N];
  float    tpl_[TEMPLATE_POINTS] = {0};                // unit-norm, zero-mean still-pulse shape
  float    tplAmp_ = 0; int tplCount_ = 0;
  float    baseTpl_[TEMPLATE_POINTS] = {0}; bool baseTplSet_ = false;   // NOT cleared by begin()
  float    sp_[MAXSEG], sv_[MAXSEG], sa_[MAXSEG];     // local copy of the segment
  uint32_t n_ = 0, pk_[4] = {0, 0, 0, 0};
  int      npk_ = 0, L_ = 0;

  // True PPG maximum near a detected peak: search -0.10 .. +0.20 s, never past `limit`.
  uint32_t refinePeak(uint32_t p, uint32_t limit) const {
    const uint32_t back = (uint32_t)(0.10f * PPG_FS), fwd = (uint32_t)(0.20f * PPG_FS);
    const uint32_t lo = p > back ? p - back : 0;
    uint32_t hi = p + fwd; if (hi >= limit) hi = limit - 1;
    uint32_t best = p;
    for (uint32_t i = lo; i <= hi; i++) if (ppg_[i & M] > ppg_[best & M]) best = i;
    return best;
  }

  // Two onset definitions, each used where it is robust:
  //  foot    = minimum before the upstroke    -> segment ends + amplitude/area baseline
  //            (on a flat foot its position jitters, but there the PPG equals the baseline, so
  //             amplitude and area hardly change)
  //  tangent = INTERSECTING TANGENT (Chiu 1991): the tangent at the steepest upstroke point u meets
  //            the horizontal line through the foot -> TIMING (heart rate, crest time). It depends
  //            on the steep upstroke only, so it is stable from beat to beat. Sub-sample (float).
  uint32_t findOnset(uint32_t pa, uint32_t pb, float& tangent) const {
    const uint32_t mid = pa + (pb - pa) / 2u;
    uint32_t u = mid;
    for (uint32_t i = mid; i <= pb; i++) if (vpg_[i & M] > vpg_[u & M]) u = i;
    uint32_t m = mid;
    for (uint32_t j = mid; j <= u; j++) if (ppg_[j & M] < ppg_[m & M]) m = j;
    const float slope = vpg_[u & M];                     // a.u. per second
    tangent = (float)m;
    if (slope > 0.0f && u > m) {
      float t = (float)u - (ppg_[u & M] - ppg_[m & M]) / slope * PPG_FS;   // samples
      if (t < (float)m) t = (float)m;
      if (t > (float)u) t = (float)u;
      tangent = t;
    }
    return m;
  }

  // First confirmed extremum after s: accepted once the signal retreats by more than thr, or
  // (shallow extremum, weak diastolic wave) if it stopped moving MIN_HOLD samples before the end.
  static int zig(const float* x, int s, int lim, bool wantMax, float thr) {
    if (s < 0 || s + 1 > lim) return -1;
    constexpr int MIN_HOLD = 3;
    int cand = s + 1;
    for (int i = s + 1; i <= lim; i++) {
      if (wantMax) { if (x[i] > x[cand]) cand = i; else if (x[cand] - x[i] > thr) return cand; }
      else         { if (x[i] < x[cand]) cand = i; else if (x[i] - x[cand] > thr) return cand; }
    }
    return (lim - cand >= MIN_HOLD) ? cand : -1;
  }

  // Baseline-corrected pulse resampled to TEMPLATE_POINTS, zero mean, unit norm.
  void shapeOf(float baselineSlope, float* out) const {
    float m = 0;
    for (int j = 0; j < TEMPLATE_POINTS; j++) {
      const float x = (float)j * (float)(L_ - 1) / (float)(TEMPLATE_POINTS - 1);
      int i0 = (int)x; if (i0 >= L_ - 1) i0 = L_ - 2;
      const float fr = x - (float)i0;
      const float v0 = sp_[i0] - (sp_[0] + baselineSlope * i0), v1 = sp_[i0 + 1] - (sp_[0] + baselineSlope * (i0 + 1));
      out[j] = v0 + fr * (v1 - v0); m += out[j];
    }
    m /= TEMPLATE_POINTS; float nrm = 0;
    for (int j = 0; j < TEMPLATE_POINTS; j++) { out[j] -= m; nrm += out[j] * out[j]; }
    nrm = sqrtf(nrm); if (nrm > 1e-9f) for (int j = 0; j < TEMPLATE_POINTS; j++) out[j] /= nrm;
  }

  float area(int from, int to) const { return PulseAuc::integrate(sp_, L_, PPG_FS, from, to); }

  static float cross(const float* x, int i, float level) {   // interpolated crossing between i-1 and i
    const float den = x[i] - x[i - 1];
    return (i - 1) + ((fabsf(den) > 1e-12f) ? (level - x[i - 1]) / den : 0.0f);
  }

  bool analyze() {
    // analyse the pulse whose systolic peak is pk_[1]: onset(pk_[0]->pk_[1]) .. onset(pk_[1]->pk_[2])
    if (pk_[2] <= pk_[1] || pk_[1] <= pk_[0] || n_ - pk_[0] >= (uint32_t)(N - 2)) return false;
    const int i1 = int(pk_[1] - pk_[0]), i2 = int(pk_[2] - pk_[1]);
    const float ratio = (float)i2 / (float)i1;
    if (ratio < 0.6f || ratio > 1.6f) return false;             // missed / extra beat

    float tA = 0, tB = 0;                                         // tangent onsets (absolute, sub-sample)
    const uint32_t oA = findOnset(pk_[0], pk_[1], tA);            // feet = segment ends
    const uint32_t oB = findOnset(pk_[1], pk_[2], tB);
    const int L = (int)(oB - oA) + 1;
    if (L < (int)(0.3f * PPG_FS) + 1 || L > MAXSEG) return false;
    bool anyMot = false;
    for (int i = 0; i < L; i++) anyMot |= mot_[(oA + i) & M] != 0;
    if (anyMot) { for (int i = 0; i < L; i++) if (hard_[(oA + i) & M]) return false; }  // judged by template below
    else        { for (int i = 0; i < L; i++) if (bad_[(oA + i) & M]) return false; }  // gap, motion or poor SQI
    L_ = L;
    for (int i = 0; i < L; i++) {
      const int k = (int)((oA + i) & M);
      sp_[i] = ppg_[k]; sv_[i] = vpg_[k]; sa_[i] = apg_[k];
    }

    PulseFeatures f;
    f.onset_ms = time_[oA & M];
    f.onsetAbs = oA; f.len = L; f.onset = 0; f.offset = L - 1;

    // --- systolic peak and amplitude above the onset-to-onset line ---
    int pk = 0;
    for (int i = 1; i < L; i++) if (sp_[i] > sp_[pk]) pk = i;
    if (pk <= 0 || pk >= L - 1) return false;
    const float baselineSlope = (sp_[L - 1] - sp_[0]) / (L - 1);
    const float amp = sp_[pk] - (sp_[0] + baselineSlope * pk);
    if (!(amp > 0) || !isfinite(amp)) return false;
    f.peak = pk; f.amp = amp;

    // --- shape vs the still-pulse template ---
    float shape[TEMPLATE_POINTS];
    shapeOf(baselineSlope, shape);
    float corr = NAN;
    if (tplCount_ > 0) { corr = 0; for (int i = 0; i < TEMPLATE_POINTS; i++) corr += shape[i] * tpl_[i]; }
    f.tplCorr = corr;
    if (baseTplSet_) { float r = 0; for (int i = 0; i < TEMPLATE_POINTS; i++) r += shape[i] * baseTpl_[i]; f.baseCorr = r; }
    if (anyMot) {                                               // motion pulse: must look like a real beat
      const float ar = (tplAmp_ > 0) ? amp / tplAmp_ : 0;
      if (tplCount_ < TEMPLATE_MIN_PULSES || !(corr >= TEMPLATE_MIN_CORR) || ar < TEMPLATE_AMP_MIN || ar > TEMPLATE_AMP_MAX) {
        rejectedMotion++; return false;
      }
      f.duringMotion = true;
    } else {                                                    // still pulse: teaches the template
      const float a = 1.0f / (float)(tplCount_ < 8 ? tplCount_ + 1 : 8);
      float nrm = 0;
      for (int i = 0; i < TEMPLATE_POINTS; i++) { tpl_[i] += a * (shape[i] - tpl_[i]); nrm += tpl_[i] * tpl_[i]; }
      nrm = sqrtf(nrm); if (nrm > 1e-9f) for (int i = 0; i < TEMPLATE_POINTS; i++) tpl_[i] /= nrm;
      tplAmp_ = (tplCount_ == 0) ? amp : tplAmp_ + a * (amp - tplAmp_);
      tplCount_++;
    }

    // --- VPG: u (max upstroke), v (max downstroke), w (first local max after v) ---
    int u = 0;
    for (int i = 1; i <= pk; i++) if (sv_[i] > sv_[u]) u = i;
    int v = u;
    for (int i = u + 1; i < L; i++) if (sv_[i] < sv_[v]) v = i;
    f.u = u; f.v = v; f.upSlope = sv_[u];
    f.w = zig(sv_, v, L - 1, true, 0.05f * sv_[u]);

    // --- dicrotic notch + diastolic peak ---
    int notch = -1, dia = -1; uint8_t nType = 0;
    for (int i = v + 1; i < L - 1; i++) {
      if (sv_[i - 1] < 0.0f && sv_[i] >= 0.0f) {
        const int cand = i - 1;
        if (cand > (int)(0.85f * (L - 1))) break;               // ripples near the next foot are not a notch
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

    // --- APG a..f ---
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
    f.ppi   = (tB - tA) * DT;                                     // tangent-to-tangent interval
    if (!(f.ppi > 0.0f)) f.ppi = (L - 1) * DT;
    f.bpm   = 60.0f / f.ppi;
    f.crest = ((float)pk - (tA - (float)oA)) * DT;                // tangent onset -> systolic peak
    if (dia >= 0) {
      f.deltaT = (dia - pk) * DT;                                 // systolic peak -> diastolic peak
      if (f.deltaT > 1e-6f) f.lasi = BODY_HEIGHT_M / f.deltaT;    // fingertip index; exploratory on a limb
    }

    // --- perfusion index, PW50, areas ---
    // PI = 100 x A / D. A = systolic peak above this pulse's onset-to-onset baseline; D = mean raw-IR DC level
    // (DC tracker, counts - never the zero-centred PPG) over the SAME samples. Any non-positive / non-finite
    // DC sample or a non-finite result leaves PI unavailable (NaN). Clipped samples never reach this point.
    float dcMean = 0; bool dcOk = true;
    for (int i = 0; i < L; ++i) { const float d = dcRing_[(oA + i) & M]; dcOk &= isfinite(d) && d > 0.0f; dcMean += d; }
    dcMean /= L;
    f.pi = (dcOk && dcMean > 1e-6f) ? 100.0f * amp / dcMean : NAN;
    if (!isfinite(f.pi)) f.pi = NAN;
    {
      float det[MAXSEG];
      for (int i = 0; i < L; ++i) det[i] = sp_[i] - (sp_[0] + baselineSlope * i);
      const float level = 0.5f * amp;
      int l = -1, r = -1;
      for (int i = 1; i <= pk; i++) if (det[i] >= level) { l = i; break; }
      for (int i = pk + 1; i < L; i++) if (det[i] <= level) { r = i; break; }
      if (l > 0 && r > 0) f.pw50 = (cross(det, r, level) - cross(det, l, level)) * DT;
    }
    f.area = area(0, L - 1);
    if (notch > 0) {
      f.sysArea = area(0, notch);
      f.diaArea = area(notch, L - 1);
      if (f.sysArea > 0.0f && f.diaArea > 0.0f) f.ipa = f.diaArea / f.sysArea;   // IPA = A2 / A1 (Wang 2009)
    }
    if (dia >= 0 && notch >= 0) f.aix = (sp_[dia] - sp_[notch]) / amp;

    // --- SDPTG (APG) ratios ---
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
    return true;
  }
};

// Old-style entry points (one shared analyzer)
inline WaveAnalyzer& waveAnalyzer() { static WaveAnalyzer a; return a; }
inline bool waveDetectionProcess(const PpgSample& s) { return waveAnalyzer().process(s); }

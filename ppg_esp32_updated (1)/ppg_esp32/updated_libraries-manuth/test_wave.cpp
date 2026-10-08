// Desktop test for the revised WaveAnalyzer.
// Build: g++ -O2 -std=c++17 -I. -I/mnt/project test_wave.cpp -o t && ./t
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <random>
#include <math.h>
float lastAccelX = 0, lastAccelY = 0, lastAccelZ = 9.81f;
#include "wave_detection.h"

static float g(float t, float c, float s) { float z = (t - c) / s; return expf(-0.5f * z * z); }
static float h(float t) {                        // one pulse, time in seconds from its start
  return 1.00f * g(t, 0.17f, 0.070f) + 0.30f * g(t, 0.50f, 0.090f);   // systolic + diastolic wave, notch between
}
static float clean(float t, float T) { int k0 = (int)floorf(t / T); float s = 0; for (int k = k0 - 1; k <= k0 + 1; k++) s += h(t - k * T); return s; }

struct Truth { int onset, peak, notch, dia; };

static int argext(const std::vector<float>& x, int a, int b, bool mx) {
  int r = a; for (int i = a; i <= b; i++) if (mx ? x[i] > x[r] : x[i] < x[r]) r = i; return r;
}

int main() {
  const float fs = PPG_FS, T = 0.85f;
  int fails = 0;

  for (int pass = 0; pass < 2; pass++) {
    const float noise = (pass == 0) ? 0.0f : 0.004f;
    const int Ns = (int)(40 * fs);
    std::mt19937 rng(1); std::normal_distribution<float> nd(0, 1);
    std::vector<float> x(Ns);
    for (int i = 0; i < Ns; i++) x[i] = clean(i / fs, T) + noise * nd(rng);

    // ground truth from the CLEAN waveform
    std::vector<float> xc(Ns);
    for (int i = 0; i < Ns; i++) xc[i] = clean(i / fs, T);
    std::vector<Truth> truth;
    for (int k = 2; k < 40 / T - 2; k++) {
      int t0 = (int)(k * T * fs);
      Truth tr;
      tr.onset = argext(xc, t0 - (int)(0.30 * T * fs), t0 + (int)(0.10 * fs), false);
      tr.peak  = argext(xc, t0 + (int)(0.05 * fs), t0 + (int)(0.30 * fs), true);
      tr.notch = argext(xc, tr.peak + 3, t0 + (int)(0.40 * fs), false);
      tr.dia   = argext(xc, tr.notch + 1, t0 + (int)(0.70 * fs), true);
      truth.push_back(tr);
    }

    // feed samples through smoothing + derivatives + peak detector exactly like pipeline.h
    MovingAverage smooth; smooth.begin(PPG_FS, SM_PPG_MS);
    Derivatives der; der.begin(PPG_FS, SM_VPG_MS, SM_APG_MS, SM_JPG_MS);
    AdaptivePeakDetector pd; pd.begin(PPG_FS);
    WaveAnalyzer wa; wa.printEnabled = false;
    int n = 0, maxOnErr = 0, maxPkErr = 0, maxNotchErr = 0, maxDiaErr = 0, nNotch2 = 0, nDia = 0;
    int nAB = 0, nOrdered = 0, nSign = 0; double sumPW = 0, sumCT = 0, sumAmp = 0;
    for (int i = 0; i < Ns; i++) {
      PpgSample s; s.raw = 50000; s.t_ms = (uint32_t)(i * 1000 / PPG_FS);
      s.ppg = smooth.process(x[i]);
      der.process(s.ppg, s.vpg, s.apg, s.jpg);
      s.valid = (i > 5 * PPG_FS); s.sqi_good = true;
      if (s.valid) s.beat = pd.process(s.ppg);
      if (wa.process(s)) {
        const PulseFeatures& f = wa.last; n++;
        // nearest truth by onset (the 50 ms causal MA delays everything by ~2.5 samples)
        const Truth* best = nullptr; int bd = 1 << 30;
        for (auto& t : truth) { int d = abs((int)f.onsetAbs - t.onset); if (d < bd) { bd = d; best = &t; } }
        const int shift = 3;   // allowed group-delay of the smoother
        int eOn = abs((int)f.onsetAbs - best->onset);
        int ePk = abs((int)(f.onsetAbs + f.peak) - best->peak);
        if (eOn > maxOnErr) maxOnErr = eOn;
        if (ePk > maxPkErr) maxPkErr = ePk;
        if (f.notchType == 2) { nNotch2++; int e = abs((int)(f.onsetAbs + f.notch) - best->notch); if (e > maxNotchErr) maxNotchErr = e; }
        if (f.dia >= 0) { nDia++; int e = abs((int)(f.onsetAbs + f.dia) - best->dia); if (e > maxDiaErr) maxDiaErr = e; }
        if (f.b >= 0) nAB++;
        if (f.bOverA < 0 && f.cOverA > 0) nSign++;
        if (f.a >= 0 && f.b > f.a && f.c > f.b && f.d > f.c && f.e > f.d) nOrdered++;
        sumPW += f.pw50; sumCT += f.crest; sumAmp += f.amp; (void)shift;
      }
    }
    printf("--- pass %d (noise sd %.3f) ---\n", pass, noise);
    printf("pulses analysed        : %d (expected ~%d)\n", n, (int)(35 / T) - 2);
    printf("max |onset err| (samp) : %d\n", maxOnErr);
    printf("max |peak err|  (samp) : %d\n", maxPkErr);
    printf("true notch found       : %d / %d, max err %d samp\n", nNotch2, n, maxNotchErr);
    printf("diastolic peak found   : %d / %d, max err %d samp\n", nDia, n, maxDiaErr);
    printf("APG a<b<c<d<e ordered  : %d / %d   (b/a<0 and c/a>0: %d / %d)\n", nOrdered, n, nSign, n);
    if (n) printf("mean amp %.3f  crest %.3f s  pw50 %.3f s\n", sumAmp / n, sumCT / n, sumPW / n);
    const PulseFeatures& f = wa.last;
    printf("last pulse: ppi %.3f bpm %.1f notchType %d | sys %.4f dia %.4f sum-vs-area diff %.2e | b/a %.2f c/a %.2f d/a %.2f e/a %.2f\n",
           f.ppi, f.bpm, f.notchType, f.sysArea, f.diaArea, (f.sysArea + f.diaArea) - f.area,
           f.bOverA, f.cOverA, f.dOverA, f.eOverA);
    if (n < 30 || maxOnErr > 6 || maxPkErr > 4 || (pass == 0 && (nNotch2 < n * 0.9 || maxNotchErr > 4 || nSign < n))) fails++;
  }

  // ---- gap test: SQI drop inside a pulse must reject only the pulses it touches ----
  {
    const int Ns = (int)(30 * fs);
    MovingAverage smooth; smooth.begin(PPG_FS, SM_PPG_MS);
    Derivatives der; der.begin(PPG_FS, SM_VPG_MS, SM_APG_MS, SM_JPG_MS);
    AdaptivePeakDetector pd; pd.begin(PPG_FS);
    WaveAnalyzer wa; wa.printEnabled = false;
    int n = 0;
    for (int i = 0; i < Ns; i++) {
      PpgSample s; s.raw = 50000; s.ppg = smooth.process(clean(i / fs, T));
      der.process(s.ppg, s.vpg, s.apg, s.jpg);
      s.valid = (i > 5 * PPG_FS);
      s.sqi_good = !(i >= (int)(15 * fs) && i < (int)(15.4 * fs));   // 0.4 s dropout
      if (s.valid) s.beat = pd.process(s.ppg);
      if (wa.process(s)) {
        n++;
        uint32_t a = wa.last.onsetAbs, b = a + wa.last.len - 1;
        if (!(b < (uint32_t)(15 * fs) || a >= (uint32_t)(15.4 * fs))) { printf("FAIL: pulse spans dropout\n"); fails++; }
      }
    }
    printf("--- gap test: %d pulses analysed, none spanning the dropout ---\n", n);
  }

  // ---- full real pipeline smoke test (reflective sensor, inverted, drift + noise) ----
  {
    PpgPipeline pl; pl.begin();
    WaveAnalyzer wa; wa.printEnabled = false;
    std::mt19937 rng(2); std::normal_distribution<float> nd(0, 1);
    const int Ns = (int)(60 * fs);
    PpgSample s_last; int pulses = 0, goodSqi = 0, withNotch = 0, withDia = 0;
    double bpmSum = 0;
    for (int i = 0; i < Ns; i++) {
      const float t = i / fs;
      float ppgTrue = 400.0f * clean(t, T);                              // blood-volume pulse, counts
      float raw = 60000.0f - ppgTrue + 150.0f * sinf(6.2831853f * 0.1f * t) + 3.0f * nd(rng);
      PpgSample s = pl.process(raw, nullptr, (uint32_t)(t * 1000));
      if (s.sqi_good) goodSqi++;
      if (s.sqi.fresh) s_last = s;
      if (wa.process(s)) {
        pulses++; bpmSum += wa.last.bpm;
        if (wa.last.notchType == 2) withNotch++;
        if (wa.last.dia >= 0) withDia++;
      }
    }
    printf("--- full pipeline smoke test (60 s) ---\n");
    printf("samples with sqi_good  : %d / %d\n", goodSqi, Ns);
    printf("pulses analysed        : %d, mean bpm %.1f (true %.1f)\n", pulses, pulses ? bpmSum / pulses : 0.0, 60.0 / T);
    printf("true notch / dia found : %d / %d\n", withNotch, withDia);
    const PulseFeatures& f = wa.last;
    printf("last SQI: period %.2f skew %.2f kurt %.2f entropy %.2f pi %.3f good %d\n", s_last.sqi.periodicity, s_last.sqi.skew, s_last.sqi.kurt, s_last.sqi.entropy, s_last.sqi.pi, (int)s_last.sqi.good);
    printf("last: amp %.1f counts, PI %.3f %%, crest %.3f s, deltaT %.3f s, pw50 %.3f s, ipa %.2f, aix %.2f\n",
           f.amp, f.pi, f.crest, f.deltaT, f.pw50, f.ipa, f.aix);
    if (pulses < 30) fails++;
  }

  printf("\n%s\n", fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED");
  return fails;
}

// Desktop unit tests for the signal-processing libraries (no ESP32 needed).
//   g++ -std=c++17 -O2 -Wall -Wextra -I.. native_test.cpp -o native_test && ./native_test
// T1 smoothing windows   T2 derivative methods   T3 Hampel   T4 corrected Maxim SpO2
// T5 pipeline + wave features vs ground truth     T6 AUC calculators   T7 flow classification
#include <stdio.h>
#include <math.h>
#include <vector>
#include <random>
#include <algorithm>
#include "pipeline.h"
#include "wave_detection.h"
#include "m10_perfusion_monitor.h"
#include "m11_spo2_processor.h"
#if NIRS_MODE > 0
#include "m13_nirs.h"
#endif

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
static double gs(double t, double c, double s) { double z = (t - c) / s; return exp(-0.5 * z * z); }
static double shape(double ph) { return gs(ph, 0.17, 0.07) + 0.35 * gs(ph, 0.50, 0.09); }
// periodic (continuous) pulse train: phase ph in [0,1), neighbours summed so there is no step at the foot
static double pulse(double ph) { return shape(ph - 1) + shape(ph) + shape(ph + 1); }

static void t1() {
  printf("[T1] moving-average window: ms -> samples at %d Hz\n", PPG_FS);
  int ms[] = {40, 80, 120, 200}, want[] = {5, 10, 15, 25};
  for (int i = 0; i < 4; i++) {
    MovingAverage m; int w = m.begin(PPG_FS, ms[i]);
    // attenuation of the systolic-peak height for a 75 bpm pulse
    double peakIn = 0, peakOut = 0;
    for (int n = 0; n < 125 * 10; n++) { double x = pulse(fmod(n / 125.0, 0.8) / 0.8); double y = m.process(x);
      if (n > 125 * 5) { peakIn = std::max(peakIn, x); peakOut = std::max(peakOut, (double)y); } }
    printf("  %3d ms -> %2d samples, delay %4.1f samples (%4.0f ms), first null %5.1f Hz, systolic peak kept %5.1f %%\n",
           ms[i], w, m.delaySamples(), m.delaySamples() * 8.0, (double)PPG_FS / w, 100 * peakOut / peakIn);
    CHECK(w == want[i], "window %d ms gave %d samples", ms[i], w);
  }
}

static void t2() {
  printf("[T2] derivative methods on the smoothed pulse (truth = analytic derivative)\n");
  // analytic pulse x(t) and its derivative; noise 0.2 % of amplitude added before smoothing
  const double fs = PPG_FS, T = 0.8;
  std::mt19937 rng(5); std::normal_distribution<double> nd(0, 0.002);
  auto x  = [&](double t) { return pulse(fmod(t, T) / T); };
  auto dx = [&](double t) { const double h = 1e-5; return (x(t + h) - x(t - h)) / (2 * h); };
  const int N = int(fs * 20);
  std::vector<double> raw(N); for (int n = 0; n < N; n++) raw[n] = x(n / fs) + nd(rng);
  // (a) backward difference after a 40 ms moving average (old method)
  // (b) central difference after the same moving average (DerivCentral)
  // (c) Savitzky-Golay 15/cubic (DerivSavGol, default)
  MovingAverage ma; ma.begin(PPG_FS, 40); DerivCentral cd; cd.begin(fs); DerivSavGol<SG_HALF> sg; sg.begin(fs, SG_ORDER);
  double eB = 0, eC = 0, eS = 0, ref = 0; int cnt = 0; double prev = 0;
  for (int n = 0; n < N; n++) {
    const double m = ma.process((float)raw[n]);
    const double back = (m - prev) * fs; prev = m;
    float p, v, a, j; const bool okc = cd.process((float)m, p, v, a, j); const double vC = v;
    float p2, v2, a2, j2; const bool oks = sg.process((float)raw[n], p2, v2, a2, j2);
    if (n < fs * 2 || !okc || !oks) continue;
    // true timing of each output: MA delay 2, backward diff centred n-0.5, central DELAY 2, SG DELAY
    const double tB = (n - 2 - 0.5) / fs, tC = (n - 2 - DerivCentral::DELAY) / fs, tS = (n - DerivSavGol<SG_HALF>::DELAY) / fs;
    const double tAligned = (n - 2) / fs;  // the sample the old code paired the derivative with (no shift)
    eB += pow(back - dx(tAligned), 2); eC += pow(vC - dx(tC), 2); eS += pow(v2 - dx(tS), 2);
    ref += pow(dx(tS), 2); cnt++; (void)tB;
  }
  const double rB = sqrt(eB / ref), rC = sqrt(eC / ref), rS = sqrt(eS / ref);
  printf("  relative RMS error of VPG: backward (old, as paired) %.3f | central %.3f | Savitzky-Golay %.3f\n", rB, rC, rS);
  CHECK(rC < rB && rS < rB, "central / SG should beat the old backward difference");
  CHECK(rS < 0.2, "SG VPG error too large");
}

static void t3() {
  printf("[T3] Hampel spike removal before the band-pass\n");
  PpgPipeline a, b; PipelineConfig ca, cb; cb.useSpike = false; a.begin(ca); b.begin(cb);
  PpgPipeline c; c.begin(ca);
  double errA = 0, errB = 0; int n0 = 0;
  for (int n = 0; n < 125 * 40; n++) {
    const double t = n / 125.0; double x = 120000 - 900 * pulse(fmod(t, 0.8) / 0.8);
    const double spike = (n % 250 == 100) ? 30000 : 0;
    PpgSample sa = a.process(x + spike, nullptr, n * 8), sb = b.process(x + spike, nullptr, n * 8), sc = c.process(x, nullptr, n * 8);
    if (t > 10) { errA += fabs(sa.ppg - sc.ppg); errB += fabs(sb.ppg - sc.ppg); n0++; }
  }
  printf("  mean |error| caused by spikes: with Hampel %.2f counts, without %.2f counts (pulse 900)\n", errA / n0, errB / n0);
  CHECK(errA < 0.05 * errB, "Hampel should remove almost all spike energy");
}

static void t4() {
  printf("[T4] corrected Maxim SpO2 (25 Hz, 100 samples)\n");
  int ok = 0, tot = 0;
  for (double R : {0.40, 0.50, 0.60, 0.70, 0.80, 1.00, 1.20}) {
    uint32_t ir[BUFFER_SIZE], red[BUFFER_SIZE];
    for (int i = 0; i < BUFFER_SIZE; i++) {
      const double t = i / 25.0, p = pulse(fmod(t + 0.13, 0.85) / 0.85);
      ir[i]  = uint32_t(120000 - 1200 * p);
      red[i] = uint32_t(90000 - R * 1200 * p * 90000 / 120000);
    }
    int32_t spo2, hr, r100; int8_t sv, hv;
    maxim_heart_rate_and_oxygen_saturation(ir, BUFFER_SIZE, red, &spo2, &sv, &hr, &hv, &r100);
    const int idx = int(lround(R * 100)); const int want = idx < SPO2_TABLE_SIZE ? uch_spo2_table[idx] : -1;
    printf("  R=%.2f -> ratio x100=%d SpO2=%d (table %d) HR=%d (true %.0f)\n", R, r100, spo2, want, hr, 60 / 0.85);
    tot++; if (sv && abs(spo2 - want) <= 1 && hv && abs(hr - 71) <= 3) ok++;
  }
  CHECK(ok == tot, "SpO2/HR correct in %d of %d", ok, tot);
  uint32_t z[50] = {0}; int32_t s, h; int8_t a, b;
  maxim_heart_rate_and_oxygen_saturation(z, 50, z, &s, &a, &h, &b);
  CHECK(!a && !b, "wrong buffer length must be rejected");
}

static void t5() {
  printf("[T5] pipeline + wave features vs ground truth (75 bpm)\n");
  for (double drift : {0.0, 300.0}) {
    PpgPipeline p; p.begin(); WaveAnalyzer w;
    std::mt19937 rng(2); std::normal_distribution<float> nd(0, 1);
    std::vector<float> bpm, amp, crest, pw, area, slope; int notch2 = 0;
    for (int n = 0; n < 125 * 60; n++) {
      const double t = n / 125.0;
      const double x = 120000 - 900 * pulse(fmod(t, 0.8) / 0.8) + drift * sin(2 * M_PI * 0.2 * t) + 4 * nd(rng);
      PpgSample s = p.process((float)x, nullptr, n * 8);
      if (w.process(s)) { const PulseFeatures& f = w.last; bpm.push_back(f.bpm); amp.push_back(f.amp); crest.push_back(f.crest);
        pw.push_back(f.pw50); area.push_back(f.area); slope.push_back(f.upSlope); if (f.notchType == 2) notch2++; }
    }
    auto ms = [](std::vector<float>& v, double& m, double& sd) { m = 0; for (float x : v) m += x; m /= v.size(); sd = 0; for (float x : v) sd += (x - m) * (x - m); sd = sqrt(sd / v.size()); };
    double m, sd;
    const double lim = drift > 0 ? 1.0 : 0.2;          // allowed CV scale: breathing wander modulates the pulse
    printf("  respiratory baseline wander %.0f counts (pulse 900): %zu pulses, notch found in %d\n", drift, bpm.size(), notch2);
    CHECK(bpm.size() > 50, "too few pulses");
    ms(bpm, m, sd);   printf("    heart rate   %.2f +- %.2f bpm (true 75)\n", m, sd);  CHECK(fabs(m - 75) < 0.5 && sd < 2, "bpm");
    ms(crest, m, sd); printf("    crest time   %.3f +- %.3f s\n", m, sd);              CHECK(sd < 0.01, "crest jitter");
    ms(pw, m, sd);    printf("    PW50         %.3f +- %.3f s\n", m, sd);
    ms(amp, m, sd);   printf("    amplitude    %.1f +- %.1f counts (CV %.1f %%)\n", m, sd, 100 * sd / m); CHECK(sd / m < 0.05 * lim, "amplitude CV");
    ms(area, m, sd);  printf("    pulse area   %.2f +- %.2f a.u.*s (CV %.1f %%)\n", m, sd, 100 * sd / m); CHECK(sd / m < 0.10 * lim, "area CV");
    ms(slope, m, sd); printf("    upstroke     %.0f +- %.0f a.u./s (CV %.1f %%)\n", m, sd, 100 * sd / m); CHECK(sd / m < 0.05 * lim, "slope CV");
  }
}

static void t6() {
  printf("[T6] AUC: segment (linear baseline) and streaming calculator\n");
  float x[101]; for (int i = 0; i <= 100; i++) x[i] = 10.0f + 0.05f * i + sinf(M_PI * i / 100.0f);   // half sine on a ramp
  const float a = PulseAuc::integrate(x, 101, 100.0f, 0, 100);   // true area of sin over 1 s = 2/pi
  AucCalculator c; for (int i = 0; i <= 100; i++) c.process(i * 10, sinf(M_PI * i / 100.0f));
  printf("  segment AUC %.4f, streaming AUC %.4f, exact %.4f\n", a, c.get_auc(), 2 / M_PI);
  CHECK(fabs(a - 2 / M_PI) < 0.002 && fabs(c.get_auc() - 2 / M_PI) < 0.002, "AUC");
}

static void t7() {
  printf("[T7] flow classification from ratios (baseline 10 pulses, confirm 3)\n");
  PerfusionMonitor pm; pm.begin(true);
  PulseFeatures f; f.valid = true; f.amp = 100; f.pi = 1; f.area = 10; f.upSlope = 1000;
  for (int i = 0; i < 10; i++) pm.push(f, i * 800);
  CHECK(pm.baselineReady, "baseline");
  PpgSample s; s.optical = true; s.valid = true;
  struct { float k; FlowState want; } cases[] = {{1.0f, FLOW_GOOD}, {0.5f, FLOW_REDUCED}, {0.15f, FLOW_POOR}, {0.9f, FLOW_GOOD}};
  uint32_t t = 10000;
  for (auto& c : cases) {
    PulseFeatures g = f; g.amp *= c.k; g.pi *= c.k; g.area *= c.k; g.upSlope *= c.k;
    for (int i = 0; i < 3; i++) { pm.push(g, t); t += 800; }
    FlowState st = pm.update(s, t);
    printf("  %.2f x baseline -> %s (score %.2f)\n", c.k, flowStateText(st), pm.score);
    CHECK(st == c.want, "flow state for %.2f", c.k);
  }
  FlowState st = pm.update(s, t + 9000);
  printf("  no pulse for 9 s -> %s\n", flowStateText(st));
  CHECK(st == FLOW_NO_PULSE, "no pulse");
}

#if NIRS_MODE > 0
static void t8() {
  printf("[T8] NIRS modified Beer-Lambert inversion\n");
  NirsMonitor nm; nm.begin();
  for (int i = 0; i < NIRS_BASELINE_FRAMES; i++) nm.finish(1000 + 60, 1400 + 60, 60, NAN, NAN, 0);
  const double o = -0.010, d = 0.008, L = NIRS_SD_DISTANCE_CM * NIRS_DPF;   // mM
  const double od1 = (NIRS_E_HBO2_WL1 * o + NIRS_E_HHB_WL1 * d) * L, od2 = (NIRS_E_HBO2_WL2 * o + NIRS_E_HHB_WL2 * d) * L;
  nm.finish(float(60 + 1000 * pow(10, -od1)), float(60 + 1400 * pow(10, -od2)), 60, NAN, NAN, 0);
  printf("  dHbO2 %.2f uM (true -10), dHHb %.2f uM (true +8)\n", nm.r.dHbO2, nm.r.dHHb);
  CHECK(fabs(nm.r.dHbO2 + 10) < 0.05 && fabs(nm.r.dHHb - 8) < 0.05, "NIRS inversion");
}

#else
static void t8() { printf("[T8] NIRS not enabled (NIRS_MODE 0) - skipped\n"); }
#endif

int main() {
  t1(); t2(); t3(); t4(); t5(); t6(); t7(); t8();
  printf(fails ? "\nRESULT: %d FAILURE(S)\n" : "\nRESULT: ALL UNIT TESTS PASSED\n", fails);
  return fails ? 1 : 0;
}

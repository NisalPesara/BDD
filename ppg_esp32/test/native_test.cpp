// Desktop unit tests for the signal-processing libraries (no ESP32 needed).
//   g++ -std=c++17 -O2 -Wall -Wextra -I.. native_test.cpp -o native_test && ./native_test
// T1 smoothing windows   T2 derivative methods   T3 Hampel   T4 corrected Maxim SpO2
// T5 pipeline + wave features vs ground truth     T6 AUC calculators   T7 flow classification
// T8 NIRS   T9-T13 relative optical pulse index x (m14): equation, levels, scenarios, availability, pipeline
#include <stdio.h>
#include <math.h>
#include <vector>
#include <random>
#include <algorithm>
#include "pipeline.h"
#include "wave_detection.h"
#include "m10_perfusion_monitor.h"
#include "m11_spo2_processor.h"
#include "m14_flow_level.h"
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

// ---------------- T9-T13: relative optical pulse index x (m14) ----------------
// XRig: 125 Hz samples, one accepted still pulse every 0.8 s (its onset 0.8 s before the report, i.e. the pulse
// ENDS at the report), FlowLevel::update() every sample (the sketch calls it every loop).
// LED pattern expected for a level in the configured mode (single LED or bar)
static uint16_t maskFor(int lv) { return lv <= 0 ? 0 : (FL_LED_MODE == FL_LED_SINGLE ? (uint16_t)(1u << (lv - 1)) : (uint16_t)((1u << lv) - 1u)); }
// reference: A, PI (D = 100 A / PI), HR 75, area-PI for a pulse whose area fills 40 % of A x T
static FlowReference ref1(float amp = 1000.0f, float pi = 1.0f) {
  FlowReference r; r.amp = amp; r.pi = pi; r.hr = 75; r.dc = 100.0f * amp / pi; r.areaPI = 0.4f * pi;
  r.crest = 0.2f; r.widthFrac = 0.3f / 0.8f; r.ampCv = 0.02f; return r;
}

struct XRig {
  FlowLevel fl; uint32_t ms = 0;
  XRig() { fl.begin(); }
  static PpgSample good() {
    PpgSample s; s.optical = true; s.valid = true; s.motionQuiet = true; s.forceStable = true; s.sqi_good = true;
    s.dc = 100000; return s;
  }
  // fill = area / (A x T): pulse-shape fullness (0.4 = the reference shape)
  static PulseFeatures pulse(float amp, float pi, float corr = 0.99f, float fill = 0.4f) {
    PulseFeatures f; f.valid = true; f.amp = amp; f.pi = pi; f.baseCorr = corr; f.bpm = 75;
    f.len = int(0.8f * PPG_FS) + 1; f.area = fill * amp * 0.8f; f.crest = 0.2f; f.pw50 = 0.3f; return f;
  }
  // run `seconds`; a pulse every 0.8 s while makePulse returns true; s0 = the sample state
  template <class F> void run(double seconds, F makePulse, PpgSample s0 = good(), bool sensorOk = true, bool acq = true) {
    const int n = int(seconds * PPG_FS + 0.5);
    for (int i = 0; i < n; i++) {
      ms += 8;
      PpgSample s = s0; s.beat = (ms % 800) == 0;
      fl.sample(s, ms);
      PulseFeatures f;
      if (s.beat && makePulse(f)) { f.onset_ms = ms - 800; fl.pushPulse(f, ms); }
      fl.update(sensorOk, acq);
    }
  }
  void pulses(int count, float amp, float pi, float corr = 0.99f, float fill = 0.4f) {
    run(count * 0.8, [&](PulseFeatures& f) { f = pulse(amp, pi, corr, fill); return true; });
  }
  void quiet(double seconds, PpgSample s0 = good()) { run(seconds, [](PulseFeatures&) { return false; }, s0); }
};

static void t9() {
  printf("[T9] x equation, numerical example, once-per-pulse update, all ten levels, hysteresis\n");
  {   // the numerical example: baseline A = 1000, D = 100000 (PI 1 %); current A = 450, D = 100000 (PI 0.45 %)
    const float D = 100000.0f;
    const float piRef = 100.0f * 1000.0f / D, piNow = 100.0f * 450.0f / D;
    XRig r; r.fl.setReference(ref1(1000.0f, piRef));
    r.pulses(5, 1000.0f, piRef);
    printf("  baseline A=1000 D=100000 -> PI %.2f %% ; current A=450 D=100000 -> PI %.2f %%\n", piRef, piNow);
    CHECK(fabsf(piRef - 1.0f) < 1e-6f && fabsf(piNow - 0.45f) < 1e-6f, "PI formula");
    CHECK(r.fl.level == 10 && fabsf(r.fl.x - 1.0f) < 1e-6f, "baseline strength -> x 1, level 10");
    printf("  x per new pulse at A=450:");
    for (int i = 0; i < 16; i++) { r.pulses(1, 450.0f, piNow); printf(" %.3f", r.fl.x); }
    printf("\n  relativeAmplitude %.3f relativePI %.3f relativeDC %.3f x %.4f level %d LEDs 0x%03X\n",
           r.fl.relAmp, r.fl.relPI, r.fl.relDC, r.fl.x, r.fl.level, r.fl.ledMask());
    CHECK(fabsf(r.fl.relPI - 0.45f) < 1e-5f && fabsf(r.fl.relAmp - 0.45f) < 1e-5f && fabsf(r.fl.relDC - 1.0f) < 1e-5f,
          "relativePI = relativeAmplitude = 0.45, relativeDC 1");
    CHECK(fabsf(r.fl.x - 0.45f) < 0.01f && r.fl.level == 5 && r.fl.ledMask() == maskFor(5), "x -> 0.45, level 5, LED pattern");
  }
  {   // exact update rule, unclipped diagnostics, no smoothing between pulses, a pulse is used once
    XRig m; m.fl.setReference(ref1()); m.pulses(1, 300, 0.3f);
    CHECK(fabsf(m.fl.x - 0.3f) < 1e-6f, "first usable pulse: x = target");
    m.pulses(1, 1500, 1.5f);
    CHECK(fabsf(m.fl.relPI - 1.5f) < 1e-6f && fabsf(m.fl.x - (FL_X_KEEP * 0.3f + (1.0f - FL_X_KEEP) * 1.0f)) < 1e-6f,
          "x = KEEP x + (1 - KEEP) clamp(relativePI); relativePI stays unclipped");
    const float x1 = m.fl.x;
    m.quiet(FL_PULSE_TIMEOUT_S - 1.5);
    CHECK(m.fl.x == x1, "no change without a new pulse (nothing smoothed between pulses)");
    PulseFeatures f = XRig::pulse(100, 0.1f); f.onset_ms = m.ms - 800;
    m.fl.pushPulse(f, m.ms); const float x2 = m.fl.x; const bool again = m.fl.pushPulse(f, m.ms);
    CHECK(x2 != x1 && !again && m.fl.x == x2, "the same pulse is used once");
  }
  {   // missing PI: relativeAmplitude is never substituted
    XRig m; m.fl.setReference(ref1()); m.pulses(3, 500, 0.5f);
    m.run(2.0, [](PulseFeatures& f) { f = XRig::pulse(1000, NAN); return true; });
    printf("  pulses with A = 1000 but no PI: x %.3f (was 0.5)\n", m.fl.x);
    CHECK(fabsf(m.fl.x - 0.5f) < 1e-6f, "NaN-PI pulses do not change x");
  }
  {   // all ten levels and the band edges (fresh start: level from x directly)
    const float xs[] = {0.00f, 0.05f, 0.0999f, 0.10f, 0.15f, 0.25f, 0.35f, 0.45f, 0.4999f, 0.50f, 0.55f, 0.65f, 0.75f, 0.85f,
                        0.8999f, 0.90f, 0.95f, 1.00f, 1.30f};
    const int   lv[] = {1, 1, 1, 2, 2, 3, 4, 5, 5, 6, 6, 7, 8, 9, 9, 10, 10, 10, 10};
    int ok = 0; const int n = sizeof lv / sizeof lv[0];
    printf("  levels:");
    for (int i = 0; i < n; i++) {
      XRig m; m.fl.setReference(ref1());
      m.pulses(3, 1000 * xs[i] + 1e-3f, xs[i] > 0 ? xs[i] : 1e-6f);
      printf(" %.4g->%d", xs[i], m.fl.level);
      if (m.fl.level == lv[i] && m.fl.ledMask() == maskFor(lv[i])) ok++;
      else printf("(want %d)", lv[i]);
    }
    printf("\n");
    CHECK(ok == n, "nominal mapping level = min(10, 1 + floor(10 x)) (%d of %d)", ok, n);
  }
  {   // hysteresis FL_HYST at every edge e = 0.1 .. 0.9, upward and downward
    int ok = 0;
    for (int k = 1; k <= 9; k++) {
      const float e = 0.1f * k;
      XRig h; h.fl.setReference(ref1());
      auto settle = [&](float v) { h.pulses(40, 1000 * v, v); return h.fl.level; };
      const int l0 = settle(e - 0.05f), l1 = settle(e + 0.01f), l2 = settle(e + 0.03f), l3 = settle(e - 0.01f), l4 = settle(e - 0.03f);
      const bool good = l0 == k && l1 == k && l2 == k + 1 && l3 == k + 1 && l4 == k;
      if (good) ok++;
      else printf("  edge %.1f: %d %d %d %d %d (want %d %d %d %d %d)\n", e, l0, l1, l2, l3, l4, k, k, k + 1, k + 1, k);
    }
    printf("  hysteresis %.2f: edge-0.05 -> edge+0.01 (stay) -> edge+0.03 (up) -> edge-0.01 (stay) -> edge-0.03 (down): %d of 9 edges\n",
           FL_HYST, ok);
    CHECK(ok == 9, "hysteresis up and down at all 9 edges");
  }
}

static void t10() {
  printf("[T10] strength scenarios: baseline/half/weak, weak but regular, DC change, HR change, trend vs noise, shape\n");
  {
    XRig r; r.fl.setReference(ref1());
    r.pulses(10, 1000, 1.0f); const int full = r.fl.level;
    r.pulses(30, 500, 0.5f);  const float xh = r.fl.x; const int half = r.fl.level;
    r.pulses(30, 150, 0.15f); const float xw = r.fl.x; const int weak = r.fl.level;
    printf("  baseline -> level %d | half: x %.3f level %d | weak 0.15: x %.3f level %d\n", full, xh, half, xw, weak);
    CHECK(full == 10, "baseline strength"); CHECK(fabsf(xh - 0.5f) < 0.01f && half == 6, "half strength (x 0.5 -> level 6)");
    CHECK(fabsf(xw - 0.15f) < 0.01f && weak == 2, "weak pulse");
  }
  {   // identical weak pulses: perfect shape vs poor shape -> identical x (diagnostics are not in x)
    XRig a, b; a.fl.setReference(ref1()); b.fl.setReference(ref1());
    a.pulses(30, 120, 0.12f, 1.00f);
    b.pulses(30, 120, 0.12f, 0.20f);
    printf("  weak 0.12 pulse: corr 1.00 -> x %.4f level %d (rate %.2f) | corr 0.20 -> x %.4f level %d\n",
           a.fl.x, a.fl.level, a.fl.detRate, b.fl.x, b.fl.level);
    CHECK(a.fl.x == b.fl.x && a.fl.level == b.fl.level && a.fl.x <= 0.125f && a.fl.level == 2,
          "a perfectly regular, well-shaped weak pulse stays a weak pulse");
    CHECK(fabsf(a.fl.tplCorr - 1.0f) < 1e-6f && fabsf(b.fl.tplCorr - 0.2f) < 1e-6f, "templateCorrelation reported");
  }
  {   // DC doubles, amplitude unchanged: PI halves -> x 0.5; relativeAmplitude stays 1 and relativeDC shows why
    XRig r; r.fl.setReference(ref1()); r.pulses(5, 1000, 1.0f);
    r.pulses(30, 1000, 0.5f);
    printf("  DC x2, A unchanged: relativeAmplitude %.3f relativePI %.3f relativeDC %.3f x %.3f level %d\n",
           r.fl.relAmp, r.fl.relPI, r.fl.relDC, r.fl.x, r.fl.level);
    CHECK(fabsf(r.fl.relAmp - 1.0f) < 1e-5f && fabsf(r.fl.relPI - 0.5f) < 1e-5f && fabsf(r.fl.relDC - 2.0f) < 1e-4f,
          "relativeAmplitude = relativePI x relativeDC");
    CHECK(fabsf(r.fl.x - 0.5f) < 0.01f && r.fl.level == 6, "x follows PI, not amplitude");
  }
  {   // HR changes, normalised amplitude unchanged -> x unchanged; hrRatio reports the change
    XRig r; r.fl.setReference(ref1()); r.pulses(5, 1000, 1.0f);
    r.run(8.0, [](PulseFeatures& f) { f = XRig::pulse(1000, 1.0f); f.bpm = 110; return true; });
    printf("  HR 75 -> 110 bpm, PI unchanged: x %.3f level %d hrRatio %.3f\n", r.fl.x, r.fl.level, r.fl.hrRatio);
    CHECK(r.fl.x == 1.0f && r.fl.level == 10 && fabsf(r.fl.hrRatio - 110.0f / 75.0f) < 1e-4f, "HR change does not move x");
  }
  {   // genuine downward trend vs random variation of the same spread
    XRig t; t.fl.setReference(ref1()); t.pulses(5, 1000, 1.0f);
    bool monotone = true; int prev = t.fl.level;
    for (int i = 0; i < 30; i++) { const float k = 1.0f - 0.02f * (i + 1); t.pulses(1, 1000 * k, k);
      if (t.fl.level > prev) monotone = false; prev = t.fl.level; }
    printf("  trend 1.0 -> 0.4 over 30 pulses: x %.3f level %d, amplitudeCV %.3f (reference CV %.3f), levels never rose: %s\n",
           t.fl.x, t.fl.level, t.fl.ampCV, t.fl.reference().ampCv, monotone ? "yes" : "no");
    CHECK(monotone && t.fl.x > 0.4f && t.fl.x < 0.5f && t.fl.ampCV > 0.1f, "trend: x follows (one-pulse lag), CV rises");
    std::mt19937 rng(11); std::normal_distribution<float> nd(0, 1);
    XRig r; r.fl.setReference(ref1()); r.pulses(10, 1000, 1.0f);
    int n = 0, in = 0, changes = 0, prevL = -1; double sx = 0;
    for (int i = 0; i < 300; i++) {
      const float k = fmaxf(0.0f, 0.5f * (1.0f + 0.3f * nd(rng)));
      r.pulses(1, 1000 * k, k);
      if (i >= 20) { n++; sx += r.fl.x; if (r.fl.level >= 4 && r.fl.level <= 7) in++; if (prevL >= 0 && r.fl.level != prevL) changes++; }
      prevL = r.fl.level;
    }
    printf("  random 0.5 +- 30 %%: mean x %.3f, level 4..7 in %d/%d pulses, level changes %d, amplitudeCV %.3f\n",
           sx / n, in, n, changes, r.fl.ampCV);
    CHECK(fabs(sx / n - 0.5) < 0.03 && in >= 0.95 * n, "random variation: x stays near 0.5");
    XRig q; q.fl.setReference(ref1()); float xmax = 0, rmax = 0;
    for (int i = 0; i < 100; i++) { const float k = fmaxf(0.0f, 1.0f + 0.3f * nd(rng)); q.pulses(1, 1000 * k, k);
      xmax = fmaxf(xmax, q.fl.x); rmax = fmaxf(rmax, q.fl.relPI); }
    printf("  random 1.0 +- 30 %%: max relativePI %.2f (unclipped), max x %.3f\n", rmax, xmax);
    CHECK(rmax > 1.0f && xmax <= 1.0f, "x clamped, relativePI unclipped");
  }
  {   // changed shape, same amplitude and PI: x unchanged; area-vs-PI disagreement is REPORTED
    XRig a, b; a.fl.setReference(ref1()); b.fl.setReference(ref1());
    a.pulses(10, 1000, 1.0f, 0.99f, 0.4f);                       // reference shape
    b.pulses(10, 1000, 1.0f, 0.30f, 0.2f);                       // narrower pulse: half the area for the same peak
    printf("  same A/PI: reference shape -> x %.3f areaVsPI %.2f support %d | changed shape -> x %.3f areaVsPI %.2f "
           "support %d corr %.2f\n", a.fl.x, a.fl.areaVsPI, a.fl.supportCode(), b.fl.x, b.fl.areaVsPI, b.fl.supportCode(), b.fl.tplCorr);
    CHECK(a.fl.support == FlowLevel::SUP_AGREE && b.fl.support == FlowLevel::SUP_DISAGREE, "disagreement exposed");
    CHECK(a.fl.x == b.fl.x && b.fl.level == 10, "shape change / disagreement does not change x");
  }
  {   // detection rate and amplitude CV
    XRig r; r.fl.setReference(ref1());                           // 75 bpm reference, the rig beats every 0.8 s
    r.pulses(15, 1000, 1.0f);
    const float rate = r.fl.detRate;
    std::vector<float> amps = {900, 1000, 1100, 950, 1050, 1000, 980, 1020, 900, 1100};
    for (float a : amps) r.pulses(1, a, a / 1000.0f);
    double m = 0, v = 0; for (float a : amps) m += a; m /= amps.size(); for (float a : amps) v += (a - m) * (a - m);
    const double cv = sqrt(v / (amps.size() - 1)) / m;
    printf("  pulseDetectionRate %.3f (want 1.0) | amplitudeCV %.4f (sample SD / mean %.4f)\n", rate, r.fl.ampCV, cv);
    CHECK(fabsf(rate - 1.0f) < 0.12f, "detection rate"); CHECK(fabs(r.fl.ampCV - cv) < 1e-4, "amplitude CV");
  }
}

static void t11() {
  printf("[T11] availability: stale pulses, motion, clipping, probe, SQI reasons, invalid PI, sensor, acquisition, reference\n");
  {   // no fresh pulse: UNKNOWN after FL_PULSE_TIMEOUT_S (never x = 0); a fresh pulse restarts x
    XRig r; r.fl.setReference(ref1()); r.pulses(5, 600, 0.6f);
    const int before = r.fl.level;
    r.quiet(FL_PULSE_TIMEOUT_S - 0.4); const int pre = r.fl.level;
    r.quiet(0.6); const int post = r.fl.level; const FlowUnknown w = r.fl.why; const float xPost = r.fl.x;
    r.pulses(1, 300, 0.3f);
    printf("  level %d -> just before the timeout %d -> just after %d (%s), x %.2f -> fresh pulse 0.3: x %.2f level %d\n",
           before, pre, post, flowUnknownText(w), xPost, r.fl.x, r.fl.level);
#if FL_NO_PULSE_IS_LOW
    // the rig's PPG is 0 = only noise near the baseline -> red LED only (level 1, x 0)
    CHECK(before == 7 && pre == 7 && post == 1 && w == FU_NONE && xPost == 0.0f, "noise-only signal -> level 1 (red only)");
    {   // a STRONG PPG whose pulses are only rejected is not shown as red
      XRig q; q.fl.setReference(ref1()); q.pulses(5, 600, 0.6f);
      PpgSample strong = XRig::good(); strong.ppg = 400;
      q.quiet(FL_PULSE_TIMEOUT_S + 1.0, strong);
      printf("  strong PPG, no accepted pulse for %.0f s -> level %d\n", FL_PULSE_TIMEOUT_S + 1.0, q.fl.level);
      CHECK(q.fl.level != 1, "strong PPG with rejected pulses is not shown as red");
      CHECK(q.fl.ledMask() == (q.fl.level ? maskFor(q.fl.level) : 0), "LED pattern");
    }
    CHECK(maskFor(1) == 1, "level 1 = LED 1 (GPIO 19) only");
#elif !FL_HOLD_ON_INVALID
    CHECK(before == 7 && pre == 7 && post == 0 && w == FU_NO_FRESH_PULSE && !isfinite(xPost), "stale -> UNKNOWN, not zero");
#endif
    CHECK(fabsf(r.fl.x - 0.3f) < 1e-6f && r.fl.level == 4, "the fresh pulse restarts x (no stale blend)");
    CHECK(FlowLevel::outNonNeg(NAN) == -1.0f && FlowLevel::outCorr(NAN) == -2.0f, "unavailable values");
  }
#if FL_HOLD_ON_INVALID
  {   // hold mode: motion keeps the last level (up to FL_HOLD_MAX_S), a pulse vanishing with the hand STILL does not
    PpgSample mv = XRig::good(); mv.motionQuiet = false;
    XRig r; r.fl.setReference(ref1()); r.pulses(5, 800, 0.8f);
    const int lv = r.fl.level;
    r.quiet(6.0, mv); const int held = r.fl.level; const uint16_t leds = r.fl.ledMask();
    r.quiet(FL_HOLD_MAX_S, mv); const int expired = r.fl.level;
    r.pulses(3, 800, 0.8f); const int back = r.fl.level;
    r.quiet(FL_PULSE_TIMEOUT_S + 0.5); const int still = r.fl.level; const FlowUnknown w = r.fl.why;
    printf("  hold: motion 6 s -> level %d (LEDs 0x%03X); > %.0f s -> %d; pulses back -> %d; pulse gone, hand still -> %d (%s)\n",
           held, leds, FL_HOLD_MAX_S, expired, back, still, flowUnknownText(w));
    CHECK(held == lv && leds == maskFor(lv), "motion: last level held");
    CHECK(expired == 0, "hold ends after FL_HOLD_MAX_S");
#if FL_NO_PULSE_IS_LOW   // the rig's PPG is noise only: shown as red (level 1), not held
    CHECK(back == lv && still == 1 && r.fl.x == 0.0f, "no hold when the pulse vanishes with the hand still -> red");
#else
    CHECK(back == lv && still == 0 && w == FU_NO_FRESH_PULSE, "no hold when the pulse vanishes with the hand still");
#endif
    PpgSample sm = XRig::good(); sm.smallMotion = true;
    XRig q; q.fl.setReference(ref1()); q.pulses(3, 800, 0.8f);
    q.run(4.0, [](PulseFeatures& f) { f = XRig::pulse(500, 0.5f); return true; }, sm);
    printf("  small motion with FL_USE_SMALL_MOTION %d: x %.3f level %d\n", FL_USE_SMALL_MOTION, q.fl.x, q.fl.level);
    CHECK(!FL_USE_SMALL_MOTION || (q.fl.relPI == 0.5f && q.fl.level > 0), "small-motion pulses update x");
  }
#else
  {   // motion (large, then small): x paused for blips shorter than the grace time, UNKNOWN after it; only a pulse
      // that STARTED after the motion restores a level
    PpgSample mv = XRig::good(); mv.motionQuiet = false;
    PpgSample sm = XRig::good(); sm.smallMotion = true;
    for (int kind = 0; kind < 2; kind++) {
      XRig r; r.fl.setReference(ref1()); r.pulses(5, 800, 0.8f);
      const int lv = r.fl.level; const float xv = r.fl.x;
      auto strong = [](PulseFeatures& f) { f = XRig::pulse(1000, 1.0f); return true; };
      r.run(FL_INVALID_GRACE_S - 0.3, strong, kind ? sm : mv);
      const bool paused = r.fl.level == lv && r.fl.x == xv;
      r.run(0.5, strong, kind ? sm : mv);
      const int during = r.fl.level; const FlowUnknown w = r.fl.why; const uint16_t leds = r.fl.ledMask();
      r.run(0.8, strong);                                         // first beat after: its pulse began during motion
      const int first = r.fl.level;
      r.run(0.8, strong);
      printf("  %s motion: blip -> level %d x %.2f (paused) | > %.1f s -> level %d (%s) LEDs 0x%03X | after: %d then %d (x %.2f)\n",
             kind ? "small" : "large", lv, xv, FL_INVALID_GRACE_S, during, flowUnknownText(w), leds, first, r.fl.level, r.fl.x);
      CHECK(paused, "%s motion blip: x paused, level kept", kind ? "small" : "large");
      CHECK(during == 0 && w == FU_MOTION && leds == 0, "%s motion -> UNKNOWN, all LEDs off", kind ? "small" : "large");
      CHECK(first == 0 && r.fl.level == 10 && r.fl.x == 1.0f, "only a pulse that started after the motion is used, fresh x");
    }
    {   // pulses overlapping small motion (duringMotion) never reach x
      XRig r; r.fl.setReference(ref1()); r.pulses(5, 800, 0.8f); const float xv = r.fl.x;
      r.run(2.0, [](PulseFeatures& f) { f = XRig::pulse(100, 0.1f); f.duringMotion = true; return true; });
      CHECK(r.fl.x == xv, "pulse that overlapped motion is not used");
    }
    {   // clipping / warm-up
      PpgSample clip = XRig::good(); clip.clipped = true;
      PpgSample warm = XRig::good(); warm.valid = false;
      XRig r; r.fl.setReference(ref1()); r.pulses(5, 800, 0.8f); r.quiet(FL_INVALID_GRACE_S + 0.2, clip);
      XRig u; u.fl.setReference(ref1()); u.pulses(5, 800, 0.8f); u.quiet(FL_INVALID_GRACE_S + 0.2, warm);
      printf("  clipping -> %d (%s); warm-up -> %d (%s)\n", r.fl.level, flowUnknownText(r.fl.why), u.fl.level, flowUnknownText(u.fl.why));
      CHECK(r.fl.level == 0 && r.fl.why == FU_CLIPPED && u.fl.level == 0 && u.fl.why == FU_WARMUP, "clipping / warm-up -> UNKNOWN");
    }
  }
#endif
  {   // probe off: immediately
    XRig r; r.fl.setReference(ref1()); r.pulses(5, 800, 0.8f);
    PpgSample off = XRig::good(); off.optical = false; r.quiet(0.008, off);
    printf("  probe off (one sample) -> level %d (%s)\n", r.fl.level, flowUnknownText(r.fl.why));
#if FL_HOLD_ON_INVALID   // a brief lift is invalid data (held); the sketch clears the reference after PROBE_OFF_CLEAR_MS
    CHECK(r.fl.level > 0, "brief probe lift -> level held");
    PulseFeatures lifted = XRig::pulse(100, 0.1f); lifted.onset_ms = r.ms - 800;
    CHECK(!r.fl.pushPulse(lifted, r.ms), "a pulse overlapping the lift is not used");
#else
    CHECK(r.fl.level == 0 && r.fl.why == FU_NO_SIGNAL && !isfinite(r.fl.x), "probe off -> UNKNOWN at once");
#endif
  }
  {   // SQI: not an invalidation by itself (a weak pulse makes SQI poor); its failure reason explains staleness
    struct { uint8_t fail; FlowUnknown want; const char* name; } cs[] = {
      {ppgnr::SQI_F_FLAT, FU_SQI_WEAK, "flat"}, {ppgnr::SQI_F_PI, FU_SQI_WEAK, "low PI"},
      {ppgnr::SQI_F_CLIP, FU_SQI_CORRUPT, "clip"}, {ppgnr::SQI_F_SKEW | ppgnr::SQI_F_PERIOD, FU_SQI_SHAPE, "skew+period"}};
    for (auto& c : cs) {
      PpgSample poor = XRig::good(); poor.sqi_good = false; poor.sqi.fresh = true; poor.sqi.good = false; poor.sqi.fail = c.fail;
      poor.ppg = 400;                                             // a PPG is present (not noise only -> never red)
      XRig r; r.fl.setReference(ref1()); r.pulses(5, 800, 0.8f);
      r.run(2.0, [](PulseFeatures& f) { f = XRig::pulse(300, 0.3f); return true; }, poor);
      const bool used = r.fl.level > 0 && r.fl.relPI == 0.3f;
      r.quiet(FL_PULSE_TIMEOUT_S + 0.5 + (FL_HOLD_ON_INVALID ? FL_HOLD_MAX_S : 0.0), poor);   // hold mode: after the hold
      printf("  SQI %-11s: pulses still used: %s; stale -> %s\n", c.name, used ? "yes" : "no", flowUnknownText(r.fl.why));
      CHECK(used && r.fl.level == 0 && r.fl.why == c.want, "SQI %s reason", c.name);
    }
  }
  {   // invalid amplitude / PI never reach x
    XRig r; r.fl.setReference(ref1());
    r.run(2.0, [](PulseFeatures& f) { f = XRig::pulse(500, NAN); return true; });
    const int l1 = r.fl.level;
    r.run(0.9, [](PulseFeatures& f) { f = XRig::pulse(0, 0.5f); return true; });
    printf("  NaN-PI pulses -> level %d, zero-amplitude pulses -> level %d\n", l1, r.fl.level);
    CHECK(l1 == 0 && r.fl.level == 0, "invalid PI / amplitude rejected");
  }
  {   // sensor failure / acquisition stopped (wall clock) / samples lost (filter restart)
    XRig r; r.fl.setReference(ref1()); r.pulses(5, 800, 0.8f);
    r.run(0.1, [](PulseFeatures&) { return false; }, XRig::good(), false, true);
    const FlowUnknown a = r.fl.why; const int la = r.fl.level;
    r.pulses(2, 800, 0.8f);
    r.run(0.1, [](PulseFeatures&) { return false; }, XRig::good(), true, false);
    printf("  sensor fault -> %d (%s); acquisition stopped -> %d (%s)\n", la, flowUnknownText(a), r.fl.level, flowUnknownText(r.fl.why));
    CHECK(la == 0 && a == FU_SENSOR && r.fl.level == 0 && r.fl.why == FU_STOPPED, "sensor / acquisition");
    XRig s; s.fl.setReference(ref1()); s.pulses(5, 800, 0.8f);
    s.fl.samplesLost();
    PulseFeatures old = XRig::pulse(1000, 1.0f); old.onset_ms = s.ms - 800;   // began before the restart
    const bool usedOld = s.fl.pushPulse(old, s.ms);
#if FL_HOLD_ON_INVALID   // filter restart: reference unchanged, level held through the warm-up
    CHECK(s.fl.level == 9 && !usedOld, "samples lost -> level held; a pulse from before is not used");
#else
    CHECK(s.fl.level == 0 && !usedOld, "samples lost -> UNKNOWN; a pulse from before is not used");
#endif
  }
  {   // reference reset: cleared -> UNKNOWN and pulses ignored; a new reference starts x from scratch
    XRig r; r.fl.setReference(ref1()); r.pulses(5, 400, 0.4f);
    r.fl.clearReference(); r.pulses(3, 400, 0.4f);
    const int noRef = r.fl.level; const FlowUnknown w = r.fl.why;
    r.fl.setReference(ref1(400, 0.4f)); r.pulses(1, 400, 0.4f);
    printf("  reference cleared -> level %d (%s); new reference = current pulse -> x %.2f level %d\n",
           noRef, flowUnknownText(w), r.fl.x, r.fl.level);
    CHECK(noRef == 0 && w == FU_NO_REFERENCE, "no reference -> UNKNOWN");
    CHECK(fabsf(r.fl.x - 1.0f) < 1e-6f && r.fl.level == 10, "new reference");
  }
  {   // m10 reference capture: 10 accepted STILL pulses with a valid PI; motion / NaN-PI pulses skipped
    PerfusionMonitor pm; pm.begin(false); pm.captureBaseline();
    PulseFeatures f = XRig::pulse(1000, 1.0f), mot = XRig::pulse(5000, 5.0f), nan = XRig::pulse(3000, NAN);
    mot.duringMotion = true;
    for (int i = 0; i < 5; i++) { pm.push(mot, i * 800); pm.push(nan, i * 800); }
    std::vector<float> a;
    for (int i = 0; i < 9; i++) { f.amp = 900 + 20 * i; f.pi = f.amp / 1000.0f; f.area = 0.4f * f.amp * 0.8f; a.push_back(f.amp);
      pm.push(f, 4000 + i * 800); }
    const bool after9 = pm.baselineReady;
    f.amp = 1500; f.pi = 1.5f; f.area = 0.4f * f.amp * 0.8f; a.push_back(f.amp); pm.push(f, 12000);
    double m = 0, v = 0; for (float x : a) m += x; m /= a.size(); for (float x : a) v += (x - m) * (x - m);
    const double cv = sqrt(v / (a.size() - 1)) / m;
    printf("  reference after 9 still pulses: %s; after 10: A %.0f PI %.3f HR %.0f DC %.0f areaPI %.3f ampCV %.4f (want %.4f)\n",
           after9 ? "ready" : "not ready", pm.ampBase, pm.piBase, pm.hrBase, pm.dcBase, pm.areaPIBase, pm.ampCvBase, cv);
    CHECK(!after9 && pm.baselineReady && fabsf(pm.ampBase - 990) < 1e-3f && fabsf(pm.piBase - 0.99f) < 1e-5f &&
          fabsf(pm.hrBase - 75) < 1e-3f, "reference = medians of 10 still pulses");
    CHECK(fabsf(pm.dcBase - 100000) < 1.0f && fabsf(pm.areaPIBase - 0.4f * 0.99f) < 1e-3f && fabs(pm.ampCvBase - cv) < 1e-4,
          "reference DC, area-PI and amplitude CV");
    const FlowReference rf = referenceFrom(pm, 120, 130);
    CHECK(rf.amp == pm.ampBase && rf.pi == pm.piBase && rf.ledRed == 120 && rf.ledIr == 130, "reference record incl. LED currents");
  }
}

// ---- T12: the real pipeline (filters, peaks, features, m10 reference, m14) on simulated pulses ----
// cond(t, A, D, T, altShape): pulse amplitude (counts), raw-IR DC (counts), beat period (s), changed shape
struct PipeOut { float piRef = NAN, relPI = NAN, relAmp = NAN, relDC = NAN, x = NAN, hrRatio = NAN, detRate = NAN,
                 tplCorr = NAN, areaVsPI = NAN; int level = 0; FlowUnknown why = FU_NONE; double unknownAt = -1; };
template <class Cond>
static PipeOut pipeRun(double seconds, Cond cond, double watchFrom = 1e9) {
  PipeOut o; PpgPipeline p; p.begin(); WaveAnalyzer w; PerfusionMonitor pm; pm.begin(false); FlowLevel fl; fl.begin();
  auto alt = [](double ph) { auto g = [](double q) { return gs(q, 0.22, 0.10); }; return g(ph - 1) + g(ph) + g(ph + 1); };
  double lo = 1e9, hi = -1e9, alo = 1e9, ahi = -1e9;
  for (int i = 0; i < 1000; i++) { const double v = pulse(i / 1000.0), a = alt(i / 1000.0);
    lo = std::min(lo, v); hi = std::max(hi, v); alo = std::min(alo, a); ahi = std::max(ahi, a); }
  std::mt19937 rng(4); std::normal_distribution<float> nd(0, 1);
  bool was = false; double ph = 0;
  for (int n = 0; n < int(seconds * 125); n++) {
    const double t = n / 125.0; double A = 1000, D = 100000, T = 0.8; bool shp = false;
    cond(t, A, D, T, shp);
    ph += (1.0 / 125.0) / T; if (ph >= 1.0) ph -= 1.0;
    const double v = shp ? (alt(ph) - alo) / (ahi - alo) : (pulse(ph) - lo) / (hi - lo);
    if (n == 125 * 15) { pm.captureBaseline(); w.clearBaselineTemplate(); fl.clearReference(); }
    PpgSample s = p.process((float)(D - A * v + 2 * nd(rng)), nullptr, n * 8);
    fl.sample(s, n * 8);
    if (w.process(s)) {
      pm.push(w.last, n * 8);
      if (pm.baselineReady && !was) { w.freezeBaselineTemplate(); fl.setReference(referenceFrom(pm, 0, 0)); o.piRef = pm.piBase; }
      else fl.pushPulse(w.last, n * 8);
      was = pm.baselineReady;
    }
    fl.update(true, true);
    if (t >= watchFrom && o.unknownAt < 0 && fl.level == 0) { o.unknownAt = t - watchFrom; o.why = fl.why; }
  }
  o.relPI = fl.relPI; o.relAmp = fl.relAmp; o.relDC = fl.relDC; o.x = fl.x; o.level = fl.level; o.hrRatio = fl.hrRatio;
  o.detRate = fl.detRate; o.tplCorr = fl.tplCorr; o.areaVsPI = fl.areaVsPI;
  if (o.unknownAt < 0) o.why = fl.why;
  return o;
}

static void t12() {
  printf("[T12] through the real pipeline (reference at 15 s in the normal condition, change at 40 s, read at 75 s)\n");
  {
    const PipeOut o = pipeRun(75, [](double t, double& A, double&, double&, bool&) { if (t >= 40) A = 450; });
    printf("  A 1000 -> 450, D 100000: reference PI %.3f %%, relativePI %.3f, x %.3f, level %d\n", o.piRef, o.relPI, o.x, o.level);
    CHECK(fabsf(o.piRef - 1.0f) < 0.1f && fabsf(o.relPI - 0.45f) < 0.02f && fabsf(o.x - 0.45f) < 0.02f && o.level == 5,
          "pipeline numerical example");
  }
  {
    const PipeOut o = pipeRun(75, [](double t, double&, double& D, double&, bool&) {
      if (t >= 40) D = 100000 + 100000 * std::min(1.0, (t - 40) / 10.0); });
    printf("  D 100000 -> 200000 (ramp 40-50 s), A unchanged: relativeAmplitude %.3f relativePI %.3f relativeDC %.3f x %.3f level %d\n",
           o.relAmp, o.relPI, o.relDC, o.x, o.level);
    CHECK(fabsf(o.relAmp - 1.0f) < 0.05f && fabsf(o.relPI - 0.5f) < 0.03f && fabsf(o.relDC - 2.0f) < 0.05f && fabsf(o.x - 0.5f) < 0.03f,
          "pipeline DC change: x follows PI, relativeDC explains it");
  }
  {   // HR rises 75 -> 109 bpm over 10 s (40-50 s), A and D unchanged
    const PipeOut o = pipeRun(75, [](double t, double&, double&, double& T, bool&) {
      if (t >= 40) T = 0.8 - 0.25 * std::min(1.0, (t - 40) / 10.0); });
    printf("  HR 75 -> 109 bpm (ramp 40-50 s), A and D unchanged: relativePI %.3f x %.3f level %d hrRatio %.3f "
           "pulseDetectionRate %.3f\n", o.relPI, o.x, o.level, o.hrRatio, o.detRate);
    CHECK(fabsf(o.relPI - 1.0f) < 0.1f && o.level >= 9 && fabsf(o.hrRatio - 0.8f / 0.55f) < 0.05f,
          "pipeline HR change: x unchanged, hrRatio reported");
    CHECK(fabsf(o.detRate - 0.8f / 0.55f) < 0.15f, "detection rate rises with HR (documented, not a strength change)");
    // KNOWN LIMITATION (m08, reported - not hidden): an ABRUPT jump to < HR_REFRACTORY_FRAC of the beat interval
    // falls inside the refractory window, every other beat is missed and the detector locks at half rate.
    const PipeOut j = pipeRun(75, [](double t, double&, double&, double& T, bool&) { if (t >= 40) T = 0.55; });
    printf("  KNOWN LIMITATION - abrupt HR jump 75 -> 109 bpm: measured hrRatio %.3f (true %.3f), relativePI %.3f, level %d%s\n",
           j.hrRatio, 0.8 / 0.55, j.relPI, j.level,
           fabsf(j.hrRatio - 0.8f / 0.55f) < 0.05f ? " (handled)" : " -> beat detector locked at half rate (m08 refractory)");
  }
  {
    const PipeOut o = pipeRun(75, [](double t, double&, double&, double&, bool& s) { if (t >= 40) s = true; });
    printf("  changed shape (no notch, broader), same A and D: relativePI %.3f x %.3f level %d templateCorrelation %.3f areaVsPI %.2f\n",
           o.relPI, o.x, o.level, o.tplCorr, o.areaVsPI);
    CHECK(fabsf(o.relPI - 1.0f) < 0.1f && o.level >= 9 && o.tplCorr < 0.99f, "pipeline shape change: x unchanged, correlation shows it");
  }
  {
    const PipeOut o = pipeRun(75, [](double t, double& A, double&, double&, bool&) { if (t >= 40) A = 120; });
    printf("  weak regular pulse A 120: relativePI %.3f x %.3f level %d templateCorrelation %.3f\n", o.relPI, o.x, o.level, o.tplCorr);
    CHECK(fabsf(o.relPI - 0.12f) < 0.02f && o.level == 2 && o.tplCorr > 0.95f, "pipeline weak but regular pulse stays weak");
  }
  {   // pulses stop at 60 s: UNKNOWN (not x = 0) once the newest pulse is older than the timeout - incl. analysis delay
    const PipeOut o = pipeRun(75, [](double t, double& A, double&, double&, bool&) { if (t >= 60) A = 0; }, 60.0);
    printf("  pulses stop at 60 s: UNKNOWN after %.2f s (%s), final level %d x %.2f\n", o.unknownAt, flowUnknownText(o.why), o.level, o.x);
#if FL_NO_PULSE_IS_LOW
    CHECK(o.level == 1 && o.x == 0.0f, "pulses stop (noise only) -> level 1, red LED only");
#elif !FL_HOLD_ON_INVALID
    CHECK(o.unknownAt > FL_PULSE_TIMEOUT_S - 1.5 && o.unknownAt < FL_PULSE_TIMEOUT_S + 0.5 && o.level == 0 && !isfinite(o.x),
          "stale after ~FL_PULSE_TIMEOUT_S measured from the pulse END");
#endif
  }
}

static void t13() {
#if FL_TWO_POINT_CAL
  printf("[T13] optional two-point calibration (FL_TWO_POINT_CAL 1)\n");
  XRig r; r.fl.setReference(ref1());
  r.pulses(12, 300, 0.3f); r.fl.startCalibration(FlowLevel::CAL_TIGHT); r.pulses(FL_CAL_PULSES, 300, 0.3f);
  CHECK(r.fl.takeCalibrationDone() == FlowLevel::CAL_TIGHT && fabsf(r.fl.xTight - 0.3f) < 0.01f, "tight captured");
  r.pulses(20, 900, 0.9f); r.fl.startCalibration(FlowLevel::CAL_FREE); r.pulses(FL_CAL_PULSES, 900, 0.9f);
  CHECK(r.fl.takeCalibrationDone() == FlowLevel::CAL_FREE && r.fl.calibrated(), "free captured, calibration in use");
  const float xs[] = {0.10f, 0.29f, 0.33f, 0.40f, 0.62f, 0.86f, 0.95f};
  const int   lv[] = {1,     1,     2,     3,     6,     9,     10};
  for (int i = 0; i < 7; i++) {
    XRig m; m.fl.setReference(ref1()); m.fl.setCalibration(0.3f, 0.9f); m.pulses(12, 1000 * xs[i], xs[i]);
    CHECK(m.fl.level == lv[i], "calibrated mapping x=%.2f -> %d (want %d)", xs[i], m.fl.level, lv[i]);
  }
#else
  printf("[T13] optional two-point calibration: FL_TWO_POINT_CAL 0 (off) - nominal mapping checked in T9\n");
  XRig r; r.fl.setReference(ref1()); r.fl.setCalibration(0.3f, 0.9f); r.pulses(5, 620, 0.62f);
  CHECK(!r.fl.calibrated() && r.fl.level == 7, "stored t/f values are ignored while the option is off");
#endif
}

int main() {
  t1(); t2(); t3(); t4(); t5(); t6(); t7(); t8(); t9(); t10(); t11(); t12(); t13();
  printf(fails ? "\nRESULT: %d FAILURE(S)\n" : "\nRESULT: ALL UNIT TESTS PASSED\n", fails);
  return fails ? 1 : 0;
}

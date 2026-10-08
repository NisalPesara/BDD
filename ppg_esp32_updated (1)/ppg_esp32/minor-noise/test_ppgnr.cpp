// Desktop tests for ppg_noise/*.h
// Build: g++ -O2 -std=c++17 -Wall -I. -Ippg_noise test_ppgnr.cpp -o tn && ./tn
// (uses the project's config.h / m06 / m07 / m08 / wave_detection.h only for comparisons)
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <random>
#include <algorithm>
#include <array>
float lastAccelX = 0, lastAccelY = 0, lastAccelZ = 9.81f;
#include "wave_detection.h"          // pulls in pipeline.h, m06, m07, m08, config.h
#include "ppg_noise.h"

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { fails++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static float gs(float t, float c, float s) { float z = (t - c) / s; return expf(-0.5f * z * z); }
static float hpulse(float t) { return 1.0f * gs(t, 0.17f, 0.07f) + 0.30f * gs(t, 0.50f, 0.09f); }
static float pulseTrain(float t, float T) { int k0 = (int)floorf(t / T); float s = 0; for (int k = k0 - 1; k <= k0 + 1; k++) s += hpulse(t - k * T); return s; }

// ---------------------------------------------------------------- T1 biquad
static void testBiquad() {
  printf("[T1] ppg_biquad.h\n");
  const double fs = 125;
  // gain at the cutoff must be -3.01 dB for a Butterworth, and ~0 dB / -inf at the extremes
  auto gainAt = [&](bool hp, int order, double fc, double f) {
    double peak = 0; const int n = (int)(fs * 60);
    ppgnr::ButterFilter<2> f2; ppgnr::ButterFilter<4> f4;
    if (order == 2) f2.begin(fs, fc, hp); else f4.begin(fs, fc, hp);
    for (int i = 0; i < n; i++) {
      float x = sinf(2 * M_PI * f * i / fs);
      float y = (order == 2) ? f2.process(x) : f4.process(x);
      if (i > n / 2) peak = std::max(peak, (double)fabsf(y));
    }
    return 20 * log10(peak);
  };
  double g1 = gainAt(true, 2, 0.15, 0.15), g2 = gainAt(false, 4, 10.0, 10.0);
  double g3 = gainAt(true, 2, 0.15, 5.0), g4 = gainAt(false, 4, 10.0, 1.0);
  printf("  HP2 0.15 Hz @0.15 Hz: %.2f dB | LP4 10 Hz @10 Hz: %.2f dB | HP@5 Hz: %.3f dB | LP@1 Hz: %.3f dB\n", g1, g2, g3, g4);
  CHECK(fabs(g1 + 3.01) < 0.3 && fabs(g2 + 3.01) < 0.3, "cutoff gain should be -3 dB");
  CHECK(fabs(g3) < 0.05 && fabs(g4) < 0.05, "passband should be flat");

  // float state vs double reference for the hardest case (0.15 Hz high-pass at 125 Hz)
  const double w = 2 * ppgnr::kPi * 0.15 / fs, c = cos(w), s = sin(w), al = s / (2 * 0.70710678), a0 = 1 + al;
  const double B0 = (1 + c) / 2 / a0, B1 = -(1 + c) / a0, B2 = B0, A1 = -2 * c / a0, A2 = (1 - al) / a0;
  double z1 = 0, z2 = 0, maxErr = 0, rms = 0; ppgnr::ButterFilter<2> hf; hf.begin(fs, 0.15, true);
  std::mt19937 rng(3); std::normal_distribution<float> nd(0, 1);
  const int n = (int)(fs * 600);
  for (int i = 0; i < n; i++) {
    float x = 500.0f * sinf(2 * M_PI * 1.2 * i / fs) + 3000.0f + 5.0f * nd(rng);
    double yd = B0 * x + z1; z1 = B1 * x - A1 * yd + z2; z2 = B2 * x - A2 * yd;
    float yf = hf.process(x);
    if (i > 5000) { maxErr = std::max(maxErr, fabs(yf - yd)); rms += yd * yd; }
  }
  rms = sqrt(rms / (n - 5001));
  printf("  float vs double HP 0.15 Hz (10 min, DC 3000): max err %.3f on rms %.1f (%.3f %%)\n", maxErr, rms, 100 * maxErr / rms);
  CHECK(maxErr / rms < 0.02, "float HP drifts from double (%.4f)", maxErr / rms);

  // prime(): constant input must give ~0 high-pass output from the first sample
  ppgnr::ButterFilter<2> hp2; hp2.begin(fs, 0.15, true); hp2.prime(60000.0f);
  float worst = 0; for (int i = 0; i < 500; i++) worst = std::max(worst, fabsf(hp2.process(60000.0f)));
  ppgnr::ButterFilter<2> hp3; hp3.begin(fs, 0.15, true);
  float worstUn = 0; for (int i = 0; i < 500; i++) worstUn = std::max(worstUn, fabsf(hp3.process(60000.0f)));
  printf("  start-up on a 60000 DC step: unprimed peak %.1f, primed peak %.4f\n", worstUn, worst);
  CHECK(worst < 1.0f, "prime() did not remove the transient");
  ppgnr::ButterFilter<4> bad; CHECK(!bad.begin(125, 70.0, false), "fc above Nyquist must be rejected");
}

// ---------------------------------------------------------------- T2 Savitzky-Golay
static void testSavGol() {
  printf("[T2] ppg_savgol.h\n");
  ppgnr::SavGolDeriv<2> sg; sg.begin(1.0, 2);
  const float ref0[5] = {-3 / 35.f, 12 / 35.f, 17 / 35.f, 12 / 35.f, -3 / 35.f};
  const float ref1[5] = {-2 / 10.f, -1 / 10.f, 0, 1 / 10.f, 2 / 10.f};
  const float ref2[5] = {2 / 7.f, -1 / 7.f, -2 / 7.f, -1 / 7.f, 2 / 7.f};
  float e0 = 0, e1 = 0, e2 = 0;
  for (int i = 0; i < 5; i++) { e0 = std::max(e0, fabsf(sg.c0[i] - ref0[i])); e1 = std::max(e1, fabsf(sg.c1[i] - ref1[i])); e2 = std::max(e2, fabsf(sg.c2[i] - ref2[i])); }
  printf("  window 5, order 2 vs published tables: max coefficient error %.2e / %.2e / %.2e\n", e0, e1, e2);
  CHECK(e0 < 1e-6 && e1 < 1e-6 && e2 < 1e-6, "coefficients differ from the standard tables");

  // exact on polynomials: p(t) = 2 + 3t - 0.5t^2 + 0.1t^3 sampled at fs=100 (order 3 reproduces it exactly)
  const double fs = 100; ppgnr::SavGolDeriv<5> s5; s5.begin(fs, 3);
  float s, v, a; double maxS = 0, maxV = 0, maxA = 0;
  for (int i = 0; i < 100; i++) {
    double t = i / fs, tc = (i - 5) / fs;
    float x = (float)(2 + 3 * t - 0.5 * t * t + 0.1 * t * t * t);
    if (s5.process(x, s, v, a)) {
      maxS = std::max(maxS, fabs(s - (2 + 3 * tc - 0.5 * tc * tc + 0.1 * tc * tc * tc)));
      maxV = std::max(maxV, fabs(v - (3 - tc + 0.3 * tc * tc)));
      maxA = std::max(maxA, fabs(a - (-1 + 0.6 * tc)));
    }
  }
  printf("  cubic test (window 11, order 3): max error smooth %.1e, d/dt %.1e, d2/dt2 %.1e\n", maxS, maxV, maxA);
  CHECK(maxS < 1e-3 && maxV < 5e-2 && maxA < 1.0, "derivatives not exact on a cubic (float round-off bound)");
}

// ---------------------------------------------------------------- T3 Hampel
static void testHampel() {
  printf("[T3] ppg_spike.h\n");
  const int fs = 125, n = 40 * fs;
  std::mt19937 rng(5); std::normal_distribution<float> nd(0, 1);
  std::vector<float> clean(n), noisy(n), out(n);
  for (int i = 0; i < n; i++) clean[i] = 60000.0f - 150.0f * pulseTrain(i / (float)fs, 0.85f) + 4.0f * nd(rng);
  noisy = clean;
  int nSpikes = 0; for (int i = 300; i < n - 10; i += 290) { noisy[i] += (nSpikes % 2 ? -1 : 1) * 3000.0f; nSpikes++; }
  ppgnr::Hampel<7> h; ppgnr::Hampel<7> hc;
  for (int i = 0; i < n; i++) out[i] = h.process(noisy[i]);
  // delayed comparison: out[i] corresponds to noisy[i - 3]
  double errSpike = 0, errClean = 0; int cnt = 0;
  for (int i = 20; i < n; i++) { errSpike = std::max(errSpike, (double)fabsf(out[i] - clean[i - 3])); cnt++; }
  // false-positive check on a spike-free signal
  std::vector<float> outc(n); for (int i = 0; i < n; i++) outc[i] = hc.process(clean[i]);
  for (int i = 20; i < n; i++) errClean = std::max(errClean, (double)fabsf(outc[i] - clean[i - 3]));
  printf("  %d injected +/-3000-count spikes: %lu flagged | worst residual after removal %.1f counts (noise sd 4)\n", nSpikes, h.spikes, errSpike);
  printf("  spike-free input: %lu samples altered (false positives), worst change %.1f counts\n", hc.spikes, errClean);
  CHECK((int)h.spikes >= nSpikes, "missed spikes");
  CHECK(errSpike < 40.0, "spike residual too large");
  CHECK(hc.spikes <= (unsigned long)(0.003 * n), "false-positive rate above 0.3 %% (%lu of %d)", hc.spikes, n);
}

// ---------------------------------------------------------------- T4 ANC
static void testAnc() {
  printf("[T4] ppg_anc.h\n");
  const int R = ANC_NUM_REFS, L = ANC_TAPS;
  std::mt19937 rng(7); std::normal_distribution<float> nd(0, 1);
  // (a) equivalence with the original m06 (pmin = 0, leak = 0, same eps)
  NlmsAnc orig; orig.eps = 1e-6f;
  ppgnr::Nlms<ANC_NUM_REFS, ANC_TAPS> mine; mine.mu = ANC_MU; mine.eps = 1e-6f; mine.pmin = 0; mine.leak = 0;
  std::vector<float> ref(R, 0.0f), lp(R, 0.0f);
  double maxDiff = 0, rms = 0; const int n = 20000;
  for (int i = 0; i < n; i++) {
    for (int r = 0; r < R; r++) { lp[r] = 0.9f * lp[r] + 0.3f * nd(rng); ref[r] = lp[r]; }
    float d = 5.0f * sinf(2 * M_PI * 1.2 * i / 125.0) + 3.0f * ref[0] - 2.0f * ref[4] + 0.1f * nd(rng);
    float a = orig.process(d, ref.data()), b = mine.process(d, ref.data());
    maxDiff = std::max(maxDiff, (double)fabsf(a - b)); rms += a * a;
  }
  rms = sqrt(rms / n);
  printf("  equivalence with m06 (R=%d, L=%d, %d samples): max |diff| %.2e on output rms %.3f\n", R, L, n, maxDiff, rms);
  CHECK(maxDiff < 1e-3 * rms, "output differs from the original NLMS");

  // (b) benefit and gating: artifact = linear mix of 3 reference axes, motion in bursts
  struct Res { double before, after; };
  auto run = [&](float pmin) {
    std::mt19937 g(11); std::normal_distribution<float> nn(0, 1);
    ppgnr::Nlms<3, 8> a; a.mu = 0.05f; a.eps = 1e-3f; a.pmin = pmin;
    const int fs = 125, N = 120 * fs; std::vector<float> truth(N), out(N), in(N);
    float lpf[3] = {0, 0, 0}; float hist[3][4] = {{0}};
    for (int i = 0; i < N; i++) {
      float t = i / (float)fs; bool burst = fmodf(t, 20.0f) < 6.0f;
      float r[3]; for (int k = 0; k < 3; k++) { lpf[k] = 0.85f * lpf[k] + 0.3f * nn(g); r[k] = lpf[k] * (burst ? 1.0f : 0.02f); }
      for (int k = 0; k < 3; k++) { for (int j = 3; j > 0; j--) hist[k][j] = hist[k][j - 1]; hist[k][0] = r[k]; }
      float art = 40.0f * hist[0][1] - 25.0f * hist[1][2] + 15.0f * hist[2][0];
      truth[i] = 20.0f * pulseTrain(t, 0.85f);
      in[i] = truth[i] + art + 0.3f * nn(g);
      out[i] = a.process(in[i], r);
    }
    double eb = 0, ea = 0; int c = 0;
    for (int i = 60 * fs; i < N; i++) { float t = i / (float)fs; if (fmodf(t, 20.0f) < 6.0f) { eb += (in[i] - truth[i]) * (in[i] - truth[i]); ea += (out[i] - truth[i]) * (out[i] - truth[i]); c++; } }
    return Res{sqrt(eb / c), sqrt(ea / c)};
  };
  Res r0 = run(0.0f), r1 = run(0.5f);
  printf("  motion bursts, residual artifact rms (counts), pulse rms 14: no cancel %.2f | NLMS %.2f | NLMS + gating %.2f\n", r0.before, r0.after, r1.after);
  CHECK(r1.after < 0.5 * r1.before, "gated NLMS did not reduce the artifact by half");
}

// ---------------------------------------------------------------- T5 force
static void testForce() {
  printf("[T5] ppg_force.h\n");
  const double fs = 125; const int bits = 12; const double rm = 10000.0;
  ppgnr::FsrChannel ch; ch.begin(fs, bits, rm, 5.0, true);
  ppgnr::ContactMonitor cm; cm.onForce = 5; cm.offForce = 3; cm.highForce = 150; cm.hyst = 10; cm.slopeMax = 20; cm.holdSamples = 62;
  std::mt19937 rng(9); std::normal_distribution<float> nd(0, 1);
  // true force F (arbitrary units) -> G = 2 uS per unit -> Rf -> divider -> ADC with noise sd 4 counts
  auto adc = [&](double F) { double G = 2e-6 * F; double rf = (G > 1e-12) ? 1.0 / G : 1e9; double v = rm / (rm + rf); return (float)(4095.0 * v + 4.0 * nd(rng)); };
  struct Seg { double F; int sec; };
  std::vector<Seg> prof = {{0, 3}, {20, 4}, {60, 4}, {200, 3}, {60, 3}, {0, 3}};
  double gainEst = 0; int gc = 0; std::vector<int> states; int stableAtEnd[6] = {0};
  int seg = 0;
  for (auto& s : prof) {
    for (int i = 0; i < s.sec * 125; i++) {
      float f = ch.process(adc(s.F)); auto st = cm.update(f, ch.slope());
      if (i == s.sec * 125 - 1) { states.push_back(st); stableAtEnd[seg] = cm.stable(); if (s.F == 60 || s.F == 20) { gainEst += f / s.F; gc++; } }
    }
    seg++;
  }
  printf("  force/true-force at 20 and 60 units (ideal = 2.0 uS/unit): mean %.2f\n", gainEst / gc);
  printf("  contact state at end of each segment (0 none, 1 ok, 2 high): ");
  for (int s : states) printf("%d ", s);
  printf("\n");
  CHECK(fabs(gainEst / gc - 2.0) < 0.15, "force not proportional to load");
  const int exp[6] = {0, 1, 1, 2, 1, 0};
  for (int i = 0; i < 6; i++) CHECK(states[i] == exp[i], "segment %d state %d expected %d", i, states[i], exp[i]);
  CHECK(stableAtEnd[2] == 1, "should be stable on a held load");
}

// ---------------------------------------------------------------- T6 SQI
static void testSqi() {
  printf("[T6] ppg_sqi.h vs m07\n");
  const int fs = PPG_FS; const int n = 30 * fs;
  std::mt19937 rng(13); std::normal_distribution<float> nd(0, 1);
  struct Sig { const char* name; std::vector<float> x; };
  std::vector<Sig> sigs;
  auto mk = [&](const char* nm, auto fn) { Sig s; s.name = nm; s.x.resize(n); for (int i = 0; i < n; i++) s.x[i] = fn(i / (float)fs, i); sigs.push_back(s); };
  mk("clean pulses", [&](float t, int) { return pulseTrain(t, 0.85f); });
  mk("pulses+0.3Hz wander 0.3", [&](float t, int) { return pulseTrain(t, 0.85f) + 0.3f * sinf(2 * M_PI * 0.3f * t); });
  mk("pulses+0.5Hz wander 0.8", [&](float t, int) { return pulseTrain(t, 0.85f) + 0.8f * sinf(2 * M_PI * 0.5f * t); });
  mk("pulses+noise sd 0.1", [&](float t, int) { return pulseTrain(t, 0.85f) + 0.1f * nd(rng); });
  mk("white noise", [&](float, int) { return nd(rng); });
  mk("flat line", [&](float, int) { return 0.5f; });
  mk("clipped pulses", [&](float t, int) { return std::min(pulseTrain(t, 0.85f), 0.45f); });
  printf("  %-24s | %-22s | %s\n", "signal", "m07 (period / good)", "ppg_sqi (period / good / fail bits / clipFrac)");
  for (auto& s : sigs) {
    SqiWindow o; o.begin(); SqiResult ro;
    ppgnr::Sqi<PPG_FS, SQI_WINDOW_S, SQI_HOP_S> q; q.begin();
    q.th.minPeriodicity = SQI_MIN_PERIODICITY; q.th.minSkew = SQI_MIN_SKEW; q.th.kurtMin = SQI_KURT_MIN; q.th.kurtMax = SQI_KURT_MAX;
    q.th.entropyMin = SQI_MIN_ENTROPY; q.th.entropyMax = SQI_MAX_ENTROPY;
    ppgnr::SqiOut rq;
    for (int i = 0; i < n; i++) { SqiResult a = o.push(s.x[i], 1.0f); if (a.fresh) ro = a; ppgnr::SqiOut b = q.push(s.x[i], 1.0f); if (b.fresh) rq = b; }
    printf("  %-24s | %5.2f / %-13d | %5.2f / %d / 0x%02x / %.3f\n", s.name, ro.periodicity, (int)ro.good, rq.periodicity, (int)rq.good, rq.fail, rq.clipFrac);
  }
  // expected classification of the revised library
  auto classify = [&](int idx) {
    ppgnr::Sqi<PPG_FS, SQI_WINDOW_S, SQI_HOP_S> q; q.begin(); q.th.minSkew = SQI_MIN_SKEW; ppgnr::SqiOut r;
    for (int i = 0; i < n; i++) { auto b = q.push(sigs[idx].x[i], 1.0f); if (b.fresh) r = b; } return r;
  };
  CHECK(classify(0).good, "clean pulses must be good");
  CHECK(classify(1).good, "pulses with moderate slow wander must still be good");
  CHECK(classify(3).good, "pulses with noise sd 0.1 must be good");
  CHECK(!classify(4).good, "white noise must be rejected");
  CHECK(!classify(5).good && (classify(5).fail & ppgnr::SQI_F_FLAT), "flat line must set the FLAT bit");
  CHECK(!classify(6).good && (classify(6).fail & ppgnr::SQI_F_CLIP), "clipped pulses must set the CLIP bit");
  {
    ppgnr::RawClipMonitor cm; cm.begin(125, 262143.0f); bool ever = false, falseAlarm = false;
    for (int i = 0; i < 125 * 20; i++) {
      float t = i / 125.0f; float raw = 120000.0f + 20000.0f * sinf(2 * M_PI * 1.2f * t);
      if (t > 8 && t < 11) raw = std::min(262143.0f, raw + 200000.0f);               // 3 s of ADC saturation
      cm.push(raw);
      if (t < 7.5f && cm.saturated()) falseAlarm = true;
      if (t > 8.5f && t < 11 && cm.saturated()) ever = true;
    }
    printf("  RawClipMonitor: saturation burst detected %d, false alarm before it %d\n", (int)ever, (int)falseAlarm);
    CHECK(ever && !falseAlarm, "RawClipMonitor");
  }
}

// ---------------------------------------------------------------- T7 derivative noise: moving average vs Savitzky-Golay
struct Stat { std::vector<double> v[4]; int n = 0; };
static Stat runDeriv(bool useSg, int half, int order, float noise) {
  Stat st; std::mt19937 rng(1); std::normal_distribution<float> nd(0, 1);
  MovingAverage sm; sm.begin(PPG_FS, SM_PPG_MS); Derivatives der; der.begin(PPG_FS, SM_VPG_MS, SM_APG_MS, SM_JPG_MS);
  AdaptivePeakDetector pd; pd.begin(PPG_FS); WaveAnalyzer wa; wa.printEnabled = false;
  ppgnr::SavGolDeriv<5> sg5; ppgnr::SavGolDeriv<7> sg7; ppgnr::SavGolDeriv<9> sg9;
  if (useSg) { sg5.begin(PPG_FS, order); sg7.begin(PPG_FS, order); sg9.begin(PPG_FS, order); }
  for (int i = 0; i < 60 * PPG_FS; i++) {
    float x = pulseTrain(i / (float)PPG_FS, 0.85f) + noise * nd(rng);
    PpgSample s; s.raw = 5e4; s.valid = i > 5 * PPG_FS; s.sqi_good = true;
    if (!useSg) { s.ppg = sm.process(x); der.process(s.ppg, s.vpg, s.apg, s.jpg); }
    else {
      float a, b, c; bool ok = (half == 5) ? sg5.process(x, a, b, c) : (half == 7) ? sg7.process(x, a, b, c) : sg9.process(x, a, b, c);
      if (!ok) continue;
      s.ppg = a; s.vpg = b; s.apg = c;
    }
    if (s.valid) s.beat = pd.process(s.ppg);
    if (wa.process(s) && wa.last.e >= 0) {
      auto& f = wa.last; st.n++; st.v[0].push_back(f.bOverA); st.v[1].push_back(f.cOverA); st.v[2].push_back(f.dOverA); st.v[3].push_back(f.eOverA);
    }
  }
  return st;
}
static void testDerivNoise() {
  printf("[T7] APG ratios under noise: 50 ms moving average (current) vs Savitzky-Golay\n");
  printf("  %-12s %-26s %6s  %-14s %-14s\n", "noise sd", "method", "pulses", "b/a", "c/a");
  double saneSd = 0, maSd = 0;
  for (float nz : {0.0f, 0.004f, 0.01f}) {
    struct M { const char* nm; bool sg; int half, ord; };
    for (M m : {M{"MA 50ms + diff (current)", false, 0, 0}, M{"SavGol win 11, order 3", true, 5, 3}, M{"SavGol win 15, order 3", true, 7, 3}, M{"SavGol win 19, order 3", true, 9, 3}}) {
      Stat s = runDeriv(m.sg, m.half, m.ord, nz);
      double mu[2] = {0, 0}, sd[2] = {0, 0};
      for (int k = 0; k < 2; k++) { for (double v : s.v[k]) mu[k] += v; if (s.n) mu[k] /= s.n; for (double v : s.v[k]) sd[k] += (v - mu[k]) * (v - mu[k]); if (s.n) sd[k] = sqrt(sd[k] / s.n); }
      printf("  %-12.3f %-26s %6d  %5.2f +/- %-5.2f  %5.2f +/- %-5.2f\n", nz, m.nm, s.n, mu[0], sd[0], mu[1], sd[1]);
      if (nz == 0.004f && !m.sg) maSd = sd[0];
      if (nz == 0.004f && m.sg && m.half == 7) saneSd = sd[0];
    }
  }
  CHECK(saneSd < maSd, "Savitzky-Golay should give a steadier b/a than the moving-average derivative at 0.4 %% noise");
}

// ---------------------------------------------------------------- T8 chain ablation
struct Variant { const char* name; ppgnr::DenoiseConfig cfg; };
struct ChainData { std::vector<float> raw, truth; std::vector<std::array<float, 3>> refs; };
static ChainData makeChainData(bool artifact, bool spikesOn) {
  const int fs = 125, N = 120 * fs;
  std::mt19937 rng(21); std::normal_distribution<float> nd(0, 1);
  ChainData d; d.raw.resize(N); d.truth.resize(N); d.refs.resize(N);
  float lpf[3] = {0, 0, 0}; float hist[3][4] = {{0}}; int spikes = 0;
  for (int i = 0; i < N; i++) {
    float t = i / (float)fs; bool burst = fmodf(t, 25.0f) > 8.0f && fmodf(t, 25.0f) < 18.0f;
    std::array<float, 3> r;
    for (int k = 0; k < 3; k++) { lpf[k] = 0.85f * lpf[k] + 0.3f * nd(rng); r[k] = lpf[k] * (burst ? 1.0f : 0.02f); }
    for (int k = 0; k < 3; k++) { for (int j = 3; j > 0; j--) hist[k][j] = hist[k][j - 1]; hist[k][0] = r[k]; }
    float art = artifact ? 60.0f * hist[0][1] - 40.0f * hist[1][2] + 25.0f * hist[2][0] : 0.0f;   // counts
    d.truth[i] = pulseTrain(t, 0.85f);
    float v = 60000.0f - 150.0f * d.truth[i] - art                                     // reflective: counts fall as volume rises
            + 400.0f * sinf(2 * M_PI * 0.05f * t) + 40.0f * sinf(2 * M_PI * 0.25f * t) + 8.0f * nd(rng);
    if (spikesOn && i % 290 == 150) v += (spikes++ % 2 ? -1 : 1) * 3000.0f;
    d.raw[i] = v; d.refs[i] = r;
  }
  return d;
}
static double chainScore(const ChainData& d, const std::vector<float>& out) {
  const int fs = 125, N = (int)d.raw.size(), i0 = 30 * fs; double best = -1;
  for (int lag = 0; lag <= 60; lag++) {
    double sa = 0, sb = 0; int c = 0;
    for (int i = i0; i < N; i++) { sa += out[i]; sb += d.truth[i - lag]; c++; }
    double ma = sa / c, mb = sb / c, saa = 0, sbb = 0, sab = 0;
    for (int i = i0; i < N; i++) { double a = out[i] - ma, b = d.truth[i - lag] - mb; saa += a * a; sbb += b * b; sab += a * b; }
    best = std::max(best, sab / sqrt(saa * sbb + 1e-30));
  }
  return best;
}
static void testChain() {
  printf("[T8] full chain on a synthetic limb recording (ablation)\n");
  const int N = 120 * 125;
  ChainData noisy = makeChainData(true, true);
  auto base = []() { ppgnr::DenoiseConfig c; c.useSpike = false; c.lpHz = 0; c.useAnc = false; c.useSg = false; return c; };
  std::vector<Variant> vs;
  { auto c = base(); vs.push_back({"DC removal + 0.15 Hz HP only", c}); }
  { auto c = base(); c.useSpike = true; vs.push_back({"+ Hampel spike removal", c}); }
  { auto c = base(); c.useSpike = true; c.lpHz = 10; vs.push_back({"+ 10 Hz low-pass", c}); }
  { auto c = base(); c.useSpike = true; c.lpHz = 10; c.useAnc = true; c.ancPmin = 0.5f; vs.push_back({"+ NLMS motion cancellation", c}); }
  { auto c = base(); c.useSpike = true; c.lpHz = 10; c.useAnc = true; c.ancPmin = 0.5f; c.useSg = true; vs.push_back({"+ Savitzky-Golay (full chain)", c}); }
  printf("  %-34s %s\n", "stage added (cumulative)", "correlation with the true pulse");
  double first = 0, full = 0; int idx = 0;
  for (auto& v : vs) {
    ppgnr::PpgDenoiser<3, 8, 5, 7> d; d.begin(v.cfg);
    std::vector<float> out(N);
    for (int i = 0; i < N; i++) out[i] = d.process(noisy.raw[i], noisy.refs[i].data(), true).clean;
    double r = chainScore(noisy, out);
    printf("  %-34s %.3f\n", v.name, r);
    if (idx == 0) first = r;
    full = r; idx++;
  }
  // ceiling: same chain, but the recording has no motion artifact and no spikes (respiration, drift, noise remain)
  ChainData ideal = makeChainData(false, false);
  { ppgnr::DenoiseConfig c = vs.back().cfg; ppgnr::PpgDenoiser<3, 8, 5, 7> d; d.begin(c); std::vector<float> out(N);
    for (int i = 0; i < N; i++) out[i] = d.process(ideal.raw[i], ideal.refs[i].data(), true).clean;
    double ceil = chainScore(ideal, out);
    printf("  ceiling (no motion, no spikes; 0.25 Hz respiration 40 counts + drift + noise remain): %.3f\n", ceil);
    CHECK(full > first, "full chain should beat DC removal + high-pass alone");
    CHECK(full > 0.9 * ceil, "full chain %.3f is far below the achievable %.3f", full, ceil);
  }
}

int main() {
  testBiquad(); testSavGol(); testHampel(); testAnc(); testForce(); testSqi(); testDerivNoise(); testChain();
  printf("\n%s (%d failed checks)\n", fails ? "SOME TESTS FAILED" : "ALL TESTS PASSED", fails);
  return fails ? 1 : 0;
}

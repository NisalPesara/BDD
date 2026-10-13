// End-to-end desktop test of ppg_esp32.ino (the real sketch, compiled with stubbed Arduino/Wire).
// A simulated limb goes through: no probe -> normal flow ('b' reference at 10 s) -> motion -> partial
// occlusion (50 %) -> severe (15 %) -> full occlusion (no pulse) -> release -> probe removed / back.
// Checks the relative pulse level (m14: LED level per phase, UNKNOWN for motion / missing pulse / no reference),
// the LED-current lock (from 'b' until the reference is cleared),
// SpO2, spike rejection, and per output mode:
//   report   (-DOUTPUT_MODE=0): flow status per phase + "Name: value unit" lines, no Teleplot lines
//   teleplot (default)        : EXACTLY the 16 Teleplot channels, BloodFlowLevel = the LED level
//
// Build + run (from this folder):
//   g++ -std=c++17 -O2 -Wall -Wextra -DOUTPUT_MODE=0 -Istubs -I.. sketch_test.cpp -o sketch_report && ./sketch_report
//   g++ -std=c++17 -O2 -Wall -Wextra -Istubs -I.. sketch_test.cpp -o sketch_teleplot && ./sketch_teleplot
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <random>
#include <string>
#include <map>
#include <set>
#include <vector>
#include <algorithm>
#include "Arduino.h"
#include "Wire.h"
#include "../config.h"

uint64_t stubTimeUs = 0;
int stubPinLevel[64] = {0};
StubSerial Serial;
StubWire Wire;

// ---------------- simulated physiology ----------------
static double gs(double t, double c, double s) { double z = (t - c) / s; return exp(-0.5 * z * z); }
static double shape1(double ph) { return gs(ph, 0.17, 0.07) + 0.35 * gs(ph, 0.50, 0.09); }
static double pulseShape(double ph) { return shape1(ph - 1) + shape1(ph) + shape1(ph + 1); }   // continuous, period 1
static double flowScale(double t) {           // pulsatile fraction of normal
  if (t < 48) return 1.0;
  if (t < 80) return 0.50;                    // partial occlusion
  if (t < 110) return 0.15;                   // severe
  if (t < 135) return 0.0;                    // full occlusion
  return 1.0;                                 // released
}
static const char* expected(double t) {
  if (t < 3) return "NO SIGNAL";
  if (t >= 45 + 2 * MOTION_ENERGY_TAU_S && t < 48) return "MOTION";   // motion energy is averaged over ~0.2 s
  if (t >= 62 && t < 80) return "REDUCED FLOW";
  if (t >= 92 && t < 110) return "POOR FLOW";
  if (t >= 125 && t < 135) return "NO PULSE";
  if ((t >= 30 && t < 45) || (t >= 162 && t < 170)) return "GOOD FLOW";   // 150.0-150.4 s: loop stall
  if (t >= 170.5 && t < 172) return "NO SIGNAL";                           // probe removed 170-172 s
  if (t >= 185) return "PULSE PRESENT";                                    // back on, reference cleared
  return nullptr;                             // transition: not scored
}
// NIRS: occlusion -> dHbO2 falls, dHHb rises (uM)
[[maybe_unused]] static double nirsHbO2(double t) { return (t >= 48 && t < 135) ? -12.0 * fmin(1.0, (t - 48) / 30.0) : 0.0; }
[[maybe_unused]] static double nirsHHb (double t) { return (t >= 48 && t < 135) ?  9.0 * fmin(1.0, (t - 48) / 30.0) : 0.0; }

static std::mt19937 rng(3);
static std::normal_distribution<float> nd(0, 1);

float stubAnalogMv(int pin) {
  const double t = stubTimeUs / 1e6; (void)t;
#if USE_FSR
  if (pin == FSR1_PIN) return 1000.f + 0.3f * nd(rng);
  if (pin == FSR2_PIN) return 1100.f + 0.3f * nd(rng);
#endif
#if NIRS_MODE > 0
  if (pin == NIRS_PD1_PIN || pin == NIRS_PD2_PIN) {
    const double dark = 60, L = NIRS_SD_DISTANCE_CM * NIRS_DPF;
    const double o = nirsHbO2(t) / 1000.0, d = nirsHHb(t) / 1000.0;   // mM
    const double od1 = (NIRS_E_HBO2_WL1 * o + NIRS_E_HHB_WL1 * d) * L, od2 = (NIRS_E_HBO2_WL2 * o + NIRS_E_HHB_WL2 * d) * L;
    double v = dark;
    if (stubPinLevel[NIRS_LED1_PIN]) v += 900.0 * pow(10.0, -od1);
    if (stubPinLevel[NIRS_LED2_PIN]) v += 1300.0 * pow(10.0, -od2);
    if (pin == NIRS_PD2_PIN) v = dark + (v - dark) * 1.8;
    return float(v + 0.5 * nd(rng));
  }
#endif
  (void)pin;
  return 1500.f;
}

#include "../ppg_esp32.ino"

int main() {
  int fails = 0;
  Wire.begin(I2C_SDA, I2C_SCL);
  setup();
  const double T = 0.8;                       // 75 bpm
  const double irDC = 120000, redDC = 90000, irAC = 900;
  const double R = 0.60;                      // -> Maxim table[60] = 97 %
  const double redAC = R * irAC * redDC / irDC;
  uint64_t nextSampleUs = 0; long n = 0; int spikes = 0;
  std::map<std::string, std::map<std::string, int>> score;   // expected -> reported -> count
  size_t scanned = 0;
  double spo2Sum = 0; int spo2N = 0;
  float nirsO2At120 = NAN, nirsHHbAt120 = NAN;
  bool sentB = false;
  int agcLockedAt12 = -1, agcLockedAt40 = -1, agcLockedAt176 = -1;   // optical current locked from 'b' until cleared
  struct Snap { double t; int level; float x; FlowUnknown why; uint16_t leds; };
  std::vector<Snap> snaps;                     // relative pulse level, sampled every 0.5 s
  uint64_t nextSnapUs = 0;

  while (stubTimeUs < 192e6) {
    while (stubTimeUs >= nextSampleUs) {      // the MAX30102 produces one pair every 8 ms
      const double t = n / 125.0;
      const bool tissue = t >= 3.0 && !(t >= 170 && t < 172);
      const double ph = fmod(t, T) / T;
      const double p = flowScale(t) * pulseShape(ph);
      double ir  = irDC  - irAC * p  + 300 * sin(2 * M_PI * 0.2 * t) + 4 * nd(rng);
      double red = redDC - redAC * p + 225 * sin(2 * M_PI * 0.2 * t) + 3 * nd(rng);
      if (tissue && n % 375 == 100) { ir += 25000; spikes++; }             // I2C glitch every 3 s
      if (!tissue) { ir = 3000 + 5 * nd(rng); red = 2000; }
      const bool moving = t >= 45 && t < 48;
      for (int s = 0; s < 2; s++) for (int k = 0; k < 3; k++)
        Wire.acc[s][k] = (k == 2 ? 9.81f : 0.f) + (moving ? 4.0f * sinf(2 * M_PI * 3.0f * t + k + s) : 0.02f * nd(rng));
      if (moving) ir += 2500 * sin(2 * M_PI * 3.0 * t);                    // motion artifact on the PPG
      Wire.push(uint32_t(ir), uint32_t(red));
      n++; nextSampleUs += 8000;
    }
    if (!sentB && stubTimeUs >= 10000000ULL) { Serial.input.push_back('b'); sentB = true; }   // reference, unrestricted
    if (agcLockedAt12 < 0 && stubTimeUs >= 12000000ULL) agcLockedAt12 = agc.locked;
    if (agcLockedAt40 < 0 && stubTimeUs >= 40000000ULL) agcLockedAt40 = agc.locked;
    if (agcLockedAt176 < 0 && stubTimeUs >= 176000000ULL) agcLockedAt176 = agc.locked;
    const bool stalled = stubTimeUs >= 150000000ULL && stubTimeUs < 150400000ULL;   // e.g. a blocking call
    if (!stalled) loop();
    stubTimeUs += 1500;                        // loop period ~1.5 ms
    if (stubTimeUs >= nextSnapUs) {
      snaps.push_back({stubTimeUs / 1e6, flowLevel.level, flowLevel.x, flowLevel.why, flowLevel.ledMask()});
      nextSnapUs += 500000;
    }

    // scan new output for "Flow status:" lines inside STATUS blocks
    const std::string& o = Serial.output;
    size_t pos;
    while ((pos = o.find("Flow status: ", scanned)) != std::string::npos) {
      size_t e = o.find('\n', pos);
      if (e == std::string::npos) break;
      const std::string st = o.substr(pos + 13, e - pos - 13);
      scanned = e;
      const double t = stubTimeUs / 1e6;
      const char* ex = expected(t);
      if (ex) {
        std::string key = st.substr(0, st.find(" -"));
        score[ex][key]++;
      }
    }
    if (stubTimeUs >= 30000000ULL && stubTimeUs < 45000000ULL && (stubTimeUs % 1000000) < 1500 && spo2.getSpO2() > 0) { spo2Sum += spo2.getSpO2(); spo2N++; }
#if NIRS_MODE > 0
    if (stubTimeUs >= 120000000ULL && !isfinite(nirsO2At120)) { nirsO2At120 = nirs.r.dHbO2; nirsHHbAt120 = nirs.r.dHHb; }
#endif
  }
  const std::string& o = Serial.output;

  // ---- relative pulse level (the 10 LEDs) per phase: both output modes ----
  printf("Relative pulse level per phase (simulated pulsatile fraction -> LED level, x):\n");
  struct Phase { const char* name; double t0, t1; int lo, hi; float xlo, xhi; FlowUnknown why; };
  const Phase phases[] = {
    {"before reference (no 'b')", 4, 10, 0, 0, -1, -1, FU_NO_REFERENCE},
    {"full strength 1.0",        30, 45, 10, 10, 0.95f, 1.0f, FU_NONE},
    {"motion < grace time",      45.0, 45.0 + FL_INVALID_GRACE_S - 0.1, 10, 10, 0.95f, 1.0f, FU_NONE},  // x paused, level kept
#if FL_HOLD_ON_INVALID
    {"motion (level held)",      45.0 + FL_INVALID_GRACE_S + 0.3, 48, 10, 10, 0.95f, 1.0f, FU_NONE},  // held, not blanked
#else
    {"motion",                   45.0 + FL_INVALID_GRACE_S + 0.3, 48, 0, 0, -1, -1, FU_MOTION},       // invalid -> UNKNOWN
#endif
    {"half strength 0.5",        65, 80, 5, 6, 0.42f, 0.58f, FU_NONE},
    {"weak 0.15",                95, 110, 2, 2, 0.10f, 0.20f, FU_NONE},
#if FL_NO_PULSE_IS_LOW
    {"missing pulse 0",          118, 135, 1, 1, 0.0f, 0.0f, FU_NONE}, // still, no pulse -> lowest level (red)
#else
    {"missing pulse 0",          118, 135, 0, 0, -1, -1, FU_NONE},     // UNKNOWN, never a measured level
#endif
#if FL_HOLD_ON_INVALID
    {"stall -> FIFO overflow",   150.5, 156, 10, 10, 0.95f, 1.0f, FU_NONE},   // filters restart: level held
#else
    {"stall -> FIFO overflow",   150.5, 156, 0, 0, -1, -1, FU_NONE},   // filters restart: stale level cleared
#endif
    {"released 1.0",             163, 170, 10, 10, 0.95f, 1.0f, FU_NONE},   // fresh pulses after the restart
    {"probe back, no reference", 175, 192, 0, 0, -1, -1, FU_NO_REFERENCE},
  };
  for (const Phase& ph : phases) {
    int cnt = 0, ok = 0, lmin = 99, lmax = -1; float xmin = 9, xmax = -9; std::map<int, int> whyN;
    for (const Snap& sn : snaps) {
      if (sn.t < ph.t0 || sn.t >= ph.t1) continue;
      cnt++; lmin = std::min(lmin, sn.level); lmax = std::max(lmax, sn.level); whyN[sn.why]++;
      if (isfinite(sn.x)) { xmin = std::min(xmin, sn.x); xmax = std::max(xmax, sn.x); }
      bool good = sn.level >= ph.lo && sn.level <= ph.hi;
      if (ph.lo > 0) good = good && isfinite(sn.x) && sn.x >= ph.xlo && sn.x <= ph.xhi &&
                            sn.leds == (FL_LED_MODE == FL_LED_SINGLE ? (uint16_t)(1u << (sn.level - 1))
                                                                     : (uint16_t)((1u << sn.level) - 1u));
      else good = good && !isfinite(sn.x) && (ph.why == FU_NONE || sn.why == ph.why) &&
                  sn.leds == 0;                                                           // all LEDs off
      if (good) ok++;
    }
    char xs[40] = "n/a";
    if (xmax >= xmin) snprintf(xs, sizeof xs, "%.3f..%.3f", xmin, xmax);
    printf("  %-26s level %d..%d  x %-12s %3d/%3d ok |", ph.name, lmin, lmax, xs, ok, cnt);
    for (auto& w : whyN) printf(" %s=%d", flowUnknownText((FlowUnknown)w.first), w.second);
    printf("\n");
    if (cnt == 0 || ok < cnt) { printf("  FAIL: relative pulse level in phase '%s'\n", ph.name); fails++; }
  }
  if (o.find("# EVENT: REFERENCE CAPTURED") == std::string::npos) { printf("  FAIL: reference not captured\n"); fails++; }
  printf("LED current locked: 2 s after 'b' %d, monitoring (40 s) %d, after probe removal (176 s) %d\n",
         agcLockedAt12, agcLockedAt40, agcLockedAt176);
  if (agcLockedAt12 != 1 || agcLockedAt40 != 1 || agcLockedAt176 != 0) { printf("  FAIL: optical-current locking\n"); fails++; }

#if OUTPUT_MODE == OUTPUT_MODE_TELEPLOT
  // ---- Teleplot: exactly these channels ----
  const std::set<std::string> want = {"relativeAmplitude", "relativePI", "PulseIndex", "BloodFlowLevel", "PPG",
    "PPG_noANC", "RelMotion", "HandMotion", "Moving", "SQI", "pulseDetectionRate", "amplitudeCV",
    "templateCorrelation", "IR_DC", "VPG", "APG"};
  std::set<std::string> got; std::map<std::string, int> cnt; int bad = 0, levelLines = 0;
  for (size_t q = 0; q < o.size();) {
    size_t e = o.find('\n', q); if (e == std::string::npos) e = o.size();
    if (o[q] == '>') {
      const std::string line = o.substr(q + 1, e - q - 1);
      const size_t c = line.find(':');
      const std::string name = line.substr(0, c);
      const double v = atof(line.c_str() + c + 1);
      got.insert(name); cnt[name]++;
      if (name == "BloodFlowLevel") { levelLines++; if (v < 0 || v > 10 || v != floor(v)) bad++; }
      if (name == "templateCorrelation" && !(v == -2.0 || (v >= -1.0 && v <= 1.0))) bad++;
      if ((name == "relativeAmplitude" || name == "relativePI" || name == "PulseIndex" || name == "pulseDetectionRate" ||
           name == "amplitudeCV" || name == "IR_DC") && !(v == -1.0 || v >= 0.0)) bad++;
      if (name == "PulseIndex" && v > 1.0) bad++;
    }
    q = e + 1;
  }
  printf("Teleplot channels (%zu):", got.size());
  for (auto& g : got) printf(" %s=%d", g.c_str(), cnt[g]);
  printf("\n");
  if (got != want) {
    printf("  FAIL: Teleplot channel set differs:");
    for (auto& g : got) if (!want.count(g)) printf(" extra '%s'", g.c_str());
    for (auto& w : want) if (!got.count(w)) printf(" missing '%s'", w.c_str());
    printf("\n"); fails++;
  }
  if (levelLines < 1500 || bad) { printf("  FAIL: BloodFlowLevel lines %d, out-of-range values %d\n", levelLines, bad); fails++; }
  {   // the last BloodFlowLevel sent equals the LED level (probe back without a reference -> 0)
    const size_t q = o.rfind(">BloodFlowLevel:"); const int v = atoi(o.c_str() + q + 16);
    printf("Last BloodFlowLevel: %d (LED level %d)\n", v, flowLevel.level);
    if (v != flowLevel.level) { printf("  FAIL: BloodFlowLevel != LED level\n"); fails++; }
  }
#else
  printf("Flow status per phase (expected -> reported counts):\n");
  for (auto& kv : score) {
    int tot = 0, ok = 0;
    for (auto& r : kv.second) { tot += r.second; if (r.first.find(kv.first) == 0) ok += r.second; }
    printf("  %-14s %3d/%3d correct |", kv.first.c_str(), ok, tot);
    for (auto& r : kv.second) printf(" %s=%d", r.first.c_str(), r.second);
    printf("\n");
    if (tot == 0 || ok < 0.8 * tot) { printf("  FAIL: phase %s\n", kv.first.c_str()); fails++; }
  }
  // format check: every feature line is "Name: value unit"
  const char* must[] = {"Systolic amplitude: ", "Perfusion index: ", "Pulse width / PW50: ", "Pulse area: ",
    "Systolic area: ", "Diastolic area: ", "Inflection point area ratio: ", "PPG augmentation index: ",
    "Crest time: ", "Delta T: ", "Large artery stiffness index: ", "b/a: ", "c/a: ", "d/a: ", "e/a: ",
    "(b-e)/a: ", "(b-c-d)/a: ", "(b-c-d-e)/a aging index: ", "Flow status: ", "SpO2: ",
    "Relative pulse level (not blood-flow %): "
#if NIRS_MODE > 0
    , "NIRS dHbO2: "
#endif
  };
  for (const char* m : must) if (o.find(m) == std::string::npos) { printf("  FAIL: missing output '%s'\n", m); fails++; }
  if (o.find("\n>") != std::string::npos) { printf("  FAIL: Teleplot lines in report mode\n"); fails++; }
  (void)score;
#endif

  const unsigned long removed = spikesEarlier + pipeline.spikesRemoved();
  printf("Pulse reports: %lu   spikes injected: %d   spikes removed: %lu\n", (unsigned long)pulsesTotal, spikes, removed);
  if (pulsesTotal < 100) { printf("  FAIL: too few pulse reports\n"); fails++; }
  if (removed < (unsigned long)(0.9 * spikes)) { printf("  FAIL: spikes not removed\n"); fails++; }

  printf("SpO2 mean over t=30-45 s (%d readings):", spo2N); printf(" %s (expected ~97)\n", spo2N ? std::to_string(int(spo2Sum / spo2N)).c_str() : "n/a");
  if (!spo2N || fabs(spo2Sum / spo2N - 97) > 2) { printf("  FAIL: SpO2\n"); fails++; }

#if NIRS_MODE > 0
  printf("NIRS at t=120 s: dHbO2=%.2f uM (sim -12)  dHHb=%.2f uM (sim +9)\n", nirsO2At120, nirsHHbAt120);
  if (!(fabs(nirsO2At120 + 12) < 1.5 && fabs(nirsHHbAt120 - 9) < 1.5)) { printf("  FAIL: NIRS\n"); fails++; }
#else
  (void)nirsO2At120; (void)nirsHHbAt120;
#endif

  if (o.find("# EVENT: FIFO OVERFLOW") == std::string::npos) { printf("  FAIL: FIFO overflow not reported\n"); fails++; }
  if (o.find("# EVENT: NO TISSUE") == std::string::npos) { printf("  FAIL: probe removal not reported\n"); fails++; }
  printf("Events reported:\n");
  for (size_t q = o.find("# EVENT"); q != std::string::npos; q = o.find("# EVENT", q + 1))
    printf("  %s\n", o.substr(q, o.find('\n', q) - q).c_str());

#if OUTPUT_MODE != OUTPUT_MODE_TELEPLOT
  // print one complete pulse report from the normal phase as a sample
  size_t a = o.find("==================== PULSE #30 ");
  if (a != std::string::npos) { size_t b = o.find("=====\n", o.find("SpO2", a)); printf("\n%s\n", o.substr(a, b - a + 6).c_str()); }
  size_t s1 = o.find("---- STATUS  t=70");
  if (s1 != std::string::npos) printf("%s\n", o.substr(s1, o.find("Signal quality:", s1) - s1).c_str());
#endif

  FILE* f = fopen("sketch_output.txt", "w"); if (f) { fputs(o.c_str(), f); fclose(f); }
  printf(fails ? "\nRESULT: %d FAILURE(S)\n" : "\nRESULT: ALL CHECKS PASSED\n", fails);
  return fails ? 1 : 0;
}

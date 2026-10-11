// End-to-end desktop test of ppg_esp32.ino (the real sketch, compiled with stubbed Arduino/Wire).
// A simulated limb goes through: no probe -> normal flow (auto baseline) -> motion -> partial
// occlusion -> severe occlusion -> full occlusion -> release.  Checks the reported flow status,
// SpO2, NIRS direction, spike rejection and that output lines are "Name: value unit".
//
// Build + run (from this folder):
//   g++ -std=c++17 -O2 -Wall -Wextra -Istubs -I.. sketch_test.cpp -o sketch_test && ./sketch_test
#include <stdio.h>
#include <math.h>
#include <random>
#include <string>
#include <map>
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
  if (t >= 45 && t < 48) return "MOTION";
  if (t >= 62 && t < 80) return "REDUCED FLOW";
  if (t >= 92 && t < 110) return "POOR FLOW";
  if (t >= 125 && t < 135) return "NO PULSE";
  if ((t >= 30 && t < 45) || (t >= 162 && t < 170)) return "GOOD FLOW";   // 150.0-150.4 s: loop stall
  if (t >= 170.5 && t < 172) return "NO SIGNAL";                           // probe removed 170-172 s
  if (t >= 185) return "PULSE PRESENT";                                    // back on, baseline cleared
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
    const bool stalled = stubTimeUs >= 150000000ULL && stubTimeUs < 150400000ULL;   // e.g. a blocking call
    if (!stalled) loop();
    stubTimeUs += 1500;                        // loop period ~1.5 ms

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

  printf("Flow status per phase (expected -> reported counts):\n");
  for (auto& kv : score) {
    int tot = 0, ok = 0;
    for (auto& r : kv.second) { tot += r.second; if (r.first.find(kv.first) == 0) ok += r.second; }
    printf("  %-14s %3d/%3d correct |", kv.first.c_str(), ok, tot);
    for (auto& r : kv.second) printf(" %s=%d", r.first.c_str(), r.second);
    printf("\n");
    if (tot == 0 || ok < 0.8 * tot) { printf("  FAIL: phase %s\n", kv.first.c_str()); fails++; }
  }
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

  if (Serial.output.find("# EVENT: FIFO OVERFLOW") == std::string::npos) { printf("  FAIL: FIFO overflow not reported\n"); fails++; }
  if (Serial.output.find("# EVENT: NO TISSUE") == std::string::npos) { printf("  FAIL: probe removal not reported\n"); fails++; }
  printf("Events reported:\n");
  for (size_t q = Serial.output.find("# EVENT"); q != std::string::npos; q = Serial.output.find("# EVENT", q + 1))
    printf("  %s\n", Serial.output.substr(q, Serial.output.find('\n', q) - q).c_str());

  // format check: every feature line is "Name: value unit"
  const std::string& o = Serial.output;
  const char* must[] = {"Systolic amplitude: ", "Perfusion index: ", "Pulse width / PW50: ", "Pulse area: ",
    "Systolic area: ", "Diastolic area: ", "Inflection point area ratio: ", "PPG augmentation index: ",
    "Crest time: ", "Delta T: ", "Large artery stiffness index: ", "b/a: ", "c/a: ", "d/a: ", "e/a: ",
    "(b-e)/a: ", "(b-c-d)/a: ", "(b-c-d-e)/a aging index: ", "Flow status: ", "SpO2: "
#if NIRS_MODE > 0
    , "NIRS dHbO2: "
#endif
  };
  for (const char* m : must) if (o.find(m) == std::string::npos) { printf("  FAIL: missing output '%s'\n", m); fails++; }

  // print one complete pulse report from the normal phase as a sample
  size_t a = o.find("==================== PULSE #30 ");
  if (a != std::string::npos) { size_t b = o.find("=====\n", o.find("SpO2", a)); printf("\n%s\n", o.substr(a, b - a + 6).c_str()); }
  size_t s1 = o.find("---- STATUS  t=12");
  if (s1 != std::string::npos) printf("%s\n", o.substr(s1, o.find("Samples:", s1) - s1).c_str());

  FILE* f = fopen("sketch_output.txt", "w"); if (f) { fputs(o.c_str(), f); fclose(f); }
  printf(fails ? "\nRESULT: %d FAILURE(S)\n" : "\nRESULT: ALL CHECKS PASSED\n", fails);
  return fails ? 1 : 0;
}

// Desktop test of the LED-current control lock (led_agc.h) with the stubbed MAX30102.
//   g++ -std=c++17 -O2 -Wall -Wextra -Istubs -I.. agc_test.cpp -o agc_test && ./agc_test
// Setup (no reference): the current follows the light level. Locked (reference being captured / held): the current
// stays fixed - reference and measurement use the same optical settings - and only ADC clipping still changes it
// (the sketch then clears the reference, see sketch_test.cpp).
#include <stdio.h>
#include "Arduino.h"
#include "Wire.h"
#include "../config.h"
#include "../led_agc.h"

uint64_t stubTimeUs = 0;
int stubPinLevel[64] = {0};
StubSerial Serial;
StubWire Wire;
float stubAnalogMv(int) { return 0; }

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

// feed `windows` AGC windows of a constant raw level (fraction of full scale); returns true if any change happened
static bool feed(LedAgc& agc, ppgio::Max30102& s, float frac, int windows) {
  bool changed = false;
  const uint32_t v = uint32_t(frac * MAX_RAW_FULL_SCALE);
  for (int i = 0; i < windows * (AGC_WINDOW_SAMPLES + AGC_SETTLE_SAMPLES); i++) changed |= agc.process(s, v, v);
  return changed;
}

int main() {
  Wire.begin(0, 0);
  ppgio::Max30102 sensor; sensor.begin();
  printf("[AGC] optical-current lock\n");
  {   // setup: a dim signal raises the current
    LedAgc agc; agc.begin(); agc.code[0] = agc.code[1] = 0x40;
    const bool ch = feed(agc, sensor, 0.30f, 1);
    printf("  unlocked, DC 30 %% of full scale: current 0x40 -> 0x%02X (changed %d)\n", agc.code[1], ch);
    CHECK(ch && agc.code[1] > 0x40, "unlocked: current follows the light level");
  }
  {   // locked: a dim (or bright, unclipped) signal leaves the current alone
    LedAgc agc; agc.begin(); agc.code[0] = agc.code[1] = 0x40; agc.locked = true;
    Wire.regs[0x57][0x0C] = Wire.regs[0x57][0x0D] = 0x5A;        // marker: any LED-register write would replace it
    const bool dim = feed(agc, sensor, 0.30f, 3), bright = feed(agc, sensor, 0.95f, 3);
    printf("  locked, DC 30 %% and 95 %%: current 0x%02X (changed %d / %d)\n", agc.code[1], dim, bright);
    CHECK(!dim && !bright && agc.code[0] == 0x40 && agc.code[1] == 0x40, "locked: current fixed");
    CHECK(Wire.regs[0x57][0x0C] == 0x5A && Wire.regs[0x57][0x0D] == 0x5A, "LED registers not written while locked");
  }
  {   // locked but clipping: the current must drop (and the sketch clears the reference)
    LedAgc agc; agc.begin(); agc.code[0] = agc.code[1] = 0x80; agc.locked = true;
    const bool ch = feed(agc, sensor, 0.99f, 1);
    printf("  locked, clipping (99 %%): current 0x80 -> 0x%02X (changed %d)\n", agc.code[1], ch);
    CHECK(ch && agc.code[1] < 0x80 && Wire.regs[0x57][0x0D] == agc.code[1], "clipping overrides the lock and reaches the sensor");
  }
  printf(fails ? "\nRESULT: %d FAILURE(S)\n" : "\nRESULT: ALL AGC CHECKS PASSED\n", fails);
  return fails ? 1 : 0;
}

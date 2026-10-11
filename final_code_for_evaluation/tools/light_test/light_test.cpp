// MAX30102 light/range A-B test: which ADC range + LED current gives the cleanest pulse on this site?
// Build/flash: pio run -e light_test -t upload   (then flash the firmware again: pio run -t upload)
// For each setting (repeated twice, interleaved) the IR LED current is calibrated to ~55 % of full scale,
// then 12 s of raw IR are streamed as "S,<setting>,<ir>" lines (125 Hz) for offline SNR analysis.
#include <Arduino.h>
#include <Wire.h>
#include "../../sensor_drivers.h"

ppgio::Max30102 ppg;
struct Setting { const char* name; uint8_t cfg; };
static const Setting SETTINGS[] = {
  {"4096nA", 0x35}, {"8192nA", 0x55}, {"16384nA", 0x75},
};

static bool readSamples(uint32_t* ir, int want) {            // blocking read of 'want' IR samples
  int got = 0; uint32_t t0 = millis();
  while (got < want && millis() - t0 < 2000UL + want * 10UL) {
    uint8_t n = 0; bool ovf = false, amb = false;
    if (!ppg.available(n, ovf, amb)) return false;
    if (ovf) { ppg.clearFifo(); continue; }
    if (!n) { delay(2); continue; }
    uint32_t a[32], r[32];
    if (!ppg.readPairs(a, r, n)) return false;
    for (int k = 0; k < n && got < want; k++) ir[got++] = a[k];
  }
  return got == want;
}

static uint8_t calibrate(uint8_t cfg) {
  ppgio::writeReg(MAX30102_ADDR, 0x0A, cfg);
  uint8_t code = 0x20;
  static uint32_t buf[125];
  for (int it = 0; it < 6; it++) {
    ppg.setLedCurrents(code, code); ppg.clearFifo(); delay(150);
    readSamples(buf, 32);                                      // discard settling samples
    if (!readSamples(buf, 62)) return code;
    double m = 0; for (int i = 0; i < 62; i++) m += buf[i]; m /= 62;
    const double frac = m / 262143.0;
    long want = (frac >= 0.97) ? code / 2 : lround(code * 0.55 / (frac > 1e-3 ? frac : 1e-3));
    want = constrain(want, 2L, 255L);
    if (labs(want - code) <= 1) break;
    code = (uint8_t)want;
  }
  return code;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Wire.begin(I2C_SDA, I2C_SCL); Wire.setClock(400000); Wire.setTimeOut(5);
  if (!ppg.begin()) { Serial.println("# MAX30102 not found"); return; }
  Serial.println("# LIGHT TEST start - keep the wrist still");
  static uint32_t ir[125];
  for (int rep = 0; rep < 2; rep++) {
    for (const Setting& s : SETTINGS) {
      const uint8_t code = calibrate(s.cfg);
      Serial.printf("# SET %s cfg=0x%02X led=%.1f mA\n", s.name, s.cfg, code * 0.2f);
      ppg.clearFifo();
      for (int sec = 0; sec < 12; sec++) {
        if (!readSamples(ir, 125)) { Serial.println("# read error"); break; }
        for (int i = 0; i < 125; i++) Serial.printf("S,%s,%lu\n", s.name, (unsigned long)ir[i]);
      }
    }
  }
  Serial.println("# LIGHT TEST done");
}

void loop() { delay(1000); }

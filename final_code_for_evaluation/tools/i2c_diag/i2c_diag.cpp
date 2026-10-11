// I2C wiring diagnostic for the PPG board (MAX30102 + ADXL345).
// Build/flash:  pio run -e i2c_diag -t upload     (then flash the normal firmware again: pio run -t upload)
// Prints, at 115200 baud: idle line levels, a repeated address scan, the ADXL345 ID, and a scan on
// other GPIO pairs (to find a sensor wired to the wrong pins).
#include <Arduino.h>
#include <Wire.h>

static const int SDA_PIN = 22, SCL_PIN = 23;   // as in config.h

static void idleLevels(int sda, int scl) {
  pinMode(sda, INPUT); pinMode(scl, INPUT);     // no internal pull-up: shows the external pull-ups
  delay(5);
  const int a = digitalRead(sda), b = digitalRead(scl);
  pinMode(sda, INPUT_PULLUP); pinMode(scl, INPUT_PULLUP);
  delay(5);
  const int c = digitalRead(sda), d = digitalRead(scl);
  Serial.printf("idle  SDA(GPIO%d): %s / with internal pull-up %s   SCL(GPIO%d): %s / with internal pull-up %s\n",
                sda, a ? "HIGH" : "LOW", c ? "HIGH" : "LOW", scl, b ? "HIGH" : "LOW", d ? "HIGH" : "LOW");
}

static bool ack(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static int readReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return -2;
  return Wire.read();
}

static void scan(int sda, int scl, uint32_t hz, int rounds, bool verbose) {
  Wire.end(); Wire.begin(sda, scl, hz); Wire.setTimeOut(20);
  int hits[128] = {0};
  for (int r = 0; r < rounds; r++)
    for (uint8_t a = 0x08; a < 0x78; a++) if (ack(a)) hits[a]++;
  bool any = false;
  for (int a = 0x08; a < 0x78; a++) if (hits[a]) {
    any = true;
    const char* name = a == 0x57 ? "MAX30102" : a == 0x53 ? "ADXL345 (SDO=GND)" : a == 0x1D ? "ADXL345 (SDO=3V3/floating)" : "?";
    Serial.printf("  SDA=%d SCL=%d %3lu kHz: 0x%02X answered %d/%d  %s\n", sda, scl, (unsigned long)(hz / 1000), a, hits[a], rounds, name);
  }
  if (!any && verbose) Serial.printf("  SDA=%d SCL=%d %3lu kHz: no device answered\n", sda, scl, (unsigned long)(hz / 1000));
}

static void idCheck(int sda, int scl) {
  Wire.end(); Wire.begin(sda, scl, 100000); Wire.setTimeOut(20);
  Serial.printf("IDs with SDA=%d SCL=%d:", sda, scl);
  const int adxl = readReg(0x53, 0x00), max = readReg(0x57, 0xFF), x68 = readReg(0x68, 0x75);
  Serial.printf("  0x53 DEVID=%d (ADXL345=229)  0x57 PART=%d (MAX30102=21)  0x68 reg0x75=%d\n", adxl, max, x68);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n===== I2C DIAGNOSTIC =====");
#ifdef CFG_TEST                  // MAX30102: which SPO2_CONFIG (reg 0x0A) values does the chip keep?
  Wire.begin(SDA_PIN, SCL_PIN, 100000); Wire.setTimeOut(20);
  for (int v : {0x35, 0x36, 0x55, 0x56, 0x75, 0x76, 0x52, 0x4E}) {
    Wire.beginTransmission(0x57); Wire.write(0x0A); Wire.write((uint8_t)v);
    const int w = Wire.endTransmission();
    Serial.printf("SPO2_CFG write 0x%02X (end=%d) -> read 0x%02X\n", v, w, readReg(0x57, 0x0A));
  }
#endif
#ifdef SWAP_FIRST
  idCheck(SCL_PIN, SDA_PIN);     // fresh boot: swapped lines first, no pins left from an earlier test
  idCheck(SDA_PIN, SCL_PIN);
  idCheck(SCL_PIN, SDA_PIN);
#endif
  idleLevels(SDA_PIN, SCL_PIN);

  Serial.println("scan on the configured pins (20 rounds each):");
  scan(SDA_PIN, SCL_PIN, 100000, 20, true);
  scan(SDA_PIN, SCL_PIN, 400000, 20, true);

  Wire.end(); Wire.begin(SDA_PIN, SCL_PIN, 100000); Wire.setTimeOut(20);
  for (uint8_t a : {(uint8_t)0x53, (uint8_t)0x1D}) {
    const int id = readReg(a, 0x00);
    Serial.printf("ADXL345 DEVID at 0x%02X: %s", a, id < 0 ? "no answer\n" : "");
    if (id >= 0) Serial.printf("0x%02X %s\n", id, id == 0xE5 ? "(correct ADXL345)" : "(NOT 0xE5 - other chip / clone)");
  }
  const int mid = readReg(0x57, 0xFF);
  Serial.printf("MAX30102 PART_ID at 0x57: %s", mid < 0 ? "no answer\n" : "");
  if (mid >= 0) Serial.printf("0x%02X %s\n", mid, mid == 0x15 ? "(correct)" : "(unexpected)");

  // Other pin pairs: is a sensor wired to different GPIOs (e.g. the ESP32 default SDA 21 / SCL 22)?
  Serial.println("scan on other GPIO pairs (looking for a sensor on the wrong pins):");
  const int pins[] = {21, 22, 23, 19, 18, 5, 4, 25, 26, 27, 32, 33};
  for (int i : pins) for (int j : pins) {
    if (i == j || (i == SDA_PIN && j == SCL_PIN)) continue;
    scan(i, j, 100000, 2, false);
  }
  Wire.end();
  Serial.println("===== DONE =====");
}

void loop() { delay(1000); }

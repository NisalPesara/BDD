// Wire finder: where is the MPU-6050 (or any I2C sensor) actually connected?
// Build/flash:  pio run -e wire_finder -t upload   (then flash the normal firmware again: pio run -t upload)
// Prints at 115200 baud:
//  1. every candidate GPIO read with the internal PULL-DOWN: a pin that still reads HIGH is held up by an
//     external pull-up resistor = it is wired to an I2C line (sensor boards carry 4.7-10 k pull-ups)
//  2. a software (bit-banged) I2C scan on every pin pair, both orientations: which addresses answer where,
//     plus WHO_AM_I for anything at 0x68 / 0x69. Bit-banging avoids the Wire library keeping old pins attached.
#include <Arduino.h>

static const int PINS[] = {2, 4, 5, 12, 13, 14, 15, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
static const int INPUT_ONLY[] = {34, 35, 36, 39};
static const int N = sizeof(PINS) / sizeof(PINS[0]);
static int SDA_ = 22, SCL_ = 23;

static inline void hi(int p) { pinMode(p, INPUT_PULLUP); }               // open-drain release
static inline void lo(int p) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
static inline void dly() { delayMicroseconds(6); }                       // ~80 kHz

static void start() { hi(SDA_); hi(SCL_); dly(); lo(SDA_); dly(); lo(SCL_); dly(); }
static void stop()  { lo(SDA_); dly(); hi(SCL_); dly(); hi(SDA_); dly(); }
static bool writeByte(uint8_t b) {                                       // true = ACK
  for (int i = 7; i >= 0; i--) { (b >> i) & 1 ? hi(SDA_) : lo(SDA_); dly(); hi(SCL_); dly(); lo(SCL_); }
  hi(SDA_); dly(); hi(SCL_); dly();
  const bool ack = digitalRead(SDA_) == LOW;
  lo(SCL_); dly();
  return ack;
}
static uint8_t readByteNack() {
  uint8_t v = 0; hi(SDA_);
  for (int i = 0; i < 8; i++) { dly(); hi(SCL_); dly(); v = (v << 1) | (digitalRead(SDA_) ? 1 : 0); lo(SCL_); }
  hi(SDA_); dly(); hi(SCL_); dly(); lo(SCL_); dly();                     // NACK
  return v;
}
static bool probe(uint8_t a) { start(); const bool ack = writeByte(a << 1); stop(); return ack; }
static int readReg(uint8_t a, uint8_t reg) {
  start(); if (!writeByte(a << 1) || !writeByte(reg)) { stop(); return -1; }
  start(); if (!writeByte((a << 1) | 1)) { stop(); return -1; }
  const int v = readByteNack(); stop(); return v;
}
static void releaseAll() { for (int i = 0; i < N; i++) pinMode(PINS[i], INPUT); }

static const char* name(uint8_t a) {
  switch (a) { case 0x53: return "ADXL345"; case 0x1D: return "ADXL345(alt)"; case 0x57: return "MAX30102";
               case 0x68: return "MPU-6050"; case 0x69: return "MPU-6050(AD0 high)"; default: return "?"; }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n===== WIRE FINDER =====");

  Serial.println("1) pins held HIGH against the internal pull-down (= external pull-up = I2C line):");
  for (int i = 0; i < N; i++) {
    pinMode(PINS[i], INPUT_PULLDOWN); delay(3);
    int h = 0; for (int k = 0; k < 20; k++) { h += digitalRead(PINS[i]); delayMicroseconds(200); }
    if (h == 20) Serial.printf("   GPIO %d HIGH\n", PINS[i]);
    else if (h > 0) Serial.printf("   GPIO %d unstable (%d/20 high)\n", PINS[i], h);
    pinMode(PINS[i], INPUT);
  }
  for (int p : INPUT_ONLY) {                       // no internal pull-down on 34-39: just report the level
    int h = 0; for (int k = 0; k < 20; k++) { h += digitalRead(p); delayMicroseconds(200); }
    if (h == 20) Serial.printf("   GPIO %d HIGH (input-only pin, no pull-down: may also be floating)\n", p);
  }

  Serial.println("2) I2C scan on every pin pair (SDA, SCL):");
  int found = 0;
  for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) {
    if (i == j) continue;
    releaseAll();
    SDA_ = PINS[i]; SCL_ = PINS[j];
    hi(SDA_); hi(SCL_); delayMicroseconds(50);
    if (!digitalRead(SDA_) || !digitalRead(SCL_)) continue;               // a line is held low: not a bus
    for (uint8_t a = 0x08; a < 0x78; a++) {
      if (!probe(a)) continue;
      found++;
      Serial.printf("   SDA=%2d SCL=%2d  0x%02X %s", SDA_, SCL_, a, name(a));
      if (a == 0x68 || a == 0x69) Serial.printf("  WHO_AM_I=0x%02X", readReg(a, 0x75));
      Serial.println();
    }
  }
  releaseAll();
  if (!found) Serial.println("   nothing answered on any pair");
  Serial.println("===== DONE =====");
}

void loop() { delay(1000); }

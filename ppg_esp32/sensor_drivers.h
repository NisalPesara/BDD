#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// Small checked I2C drivers avoid the 4-slot host ring used by some MAX3010x
// libraries. FIFO ownership stays here; every RED/IR pair is read atomically.
namespace ppgio {
inline bool writeReg(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}
inline bool readRegs(uint8_t address, uint8_t reg, uint8_t* data, size_t n) {
  Wire.beginTransmission(address); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, n) != n) { while (Wire.available()) Wire.read(); return false; }
  for (size_t i = 0; i < n; ++i) data[i] = uint8_t(Wire.read());
  return true;
}
class Max30102 {
 public:
  bool begin() {
    uint8_t id = 0;
    if (!readRegs(MAX30102_ADDR, 0xff, &id, 1) || id != 0x15) return false;
    if (!writeReg(MAX30102_ADDR, 0x09, 0x40)) return false;
    const uint32_t start = millis();
    do {
      delay(1);
      if (!readRegs(MAX30102_ADDR, 0x09, &id, 1)) return false;
    } while ((id & 0x40) && uint32_t(millis() - start) < 100);
    if (id & 0x40) return false;
    if (!writeReg(MAX30102_ADDR, 0x02, 0) || !writeReg(MAX30102_ADDR, 0x03, 0) ||
        !writeReg(MAX30102_ADDR, 0x08, MAX_FIFO_CONFIG) ||
        !writeReg(MAX30102_ADDR, 0x0a, MAX_SPO2_CONFIG) ||
        !writeReg(MAX30102_ADDR, 0x0c, MAX_LED_CURRENT_CODE) ||
        !writeReg(MAX30102_ADDR, 0x0d, MAX_LED_CURRENT_CODE) ||
        !writeReg(MAX30102_ADDR, 0x09, 0x03) || !clearFifo()) return false;
    uint8_t actual = 0;
    return readRegs(MAX30102_ADDR, 0x0a, &actual, 1) && actual == MAX_SPO2_CONFIG;
  }
  bool clearFifo() {
    uint8_t status[2];
    return writeReg(MAX30102_ADDR, 0x04, 0) && writeReg(MAX30102_ADDR, 0x05, 0) &&
           writeReg(MAX30102_ADDR, 0x06, 0) && readRegs(MAX30102_ADDR, 0x00, status, 2);
  }
  bool available(uint8_t& count, bool& overflow, bool& ambientOverflow) {
    uint8_t p[3], status[2];
    if (!readRegs(MAX30102_ADDR, 0x00, status, 2) || !readRegs(MAX30102_ADDR, 0x04, p, 3)) return false;
    count = uint8_t((p[0] - p[2]) & 31);
    overflow = (p[1] & 31) != 0;
    ambientOverflow = (status[0] & 0x20) != 0;
    return true;
  }
  bool readPair(uint32_t& ir, uint32_t& red) {
    uint8_t b[6];
    if (!readRegs(MAX30102_ADDR, 0x07, b, 6)) return false;
    red = ((uint32_t(b[0]) << 16) | (uint32_t(b[1]) << 8) | b[2]) & 0x3ffff;
    ir  = ((uint32_t(b[3]) << 16) | (uint32_t(b[4]) << 8) | b[5]) & 0x3ffff;
    return true;
  }
};
class Adxl345 {
 public:
  bool begin(uint8_t address) {
    addr_ = address; uint8_t id = 0;
    return readRegs(addr_, 0x00, &id, 1) && id == 0xe5 &&
           writeReg(addr_, 0x2d, 0) && writeReg(addr_, 0x31, 0x09) && // full res +/-4g
           writeReg(addr_, 0x2c, 0x0b) && // 200 Hz ODR; software reads latest coherent data
           writeReg(addr_, 0x38, 0) && writeReg(addr_, 0x2d, 0x08);
  }
  // Return false on bus error or if no NEW data is ready; never reuse stale axes.
  bool read(float* xyz) {
    uint8_t st = 0, b[6];
    if (!readRegs(addr_, 0x30, &st, 1) || !(st & 0x80) || !readRegs(addr_, 0x32, b, 6)) return false;
    for (int i = 0; i < 3; ++i) {
      int16_t v = int16_t(uint16_t(b[2*i]) | uint16_t(b[2*i+1]) << 8);
      xyz[i] = float(v) * .0039f * 9.80665f;
    }
    return true;
  }
 private:
  uint8_t addr_ = ADXL345_PROBE_ADDR;
};
} // namespace ppgio

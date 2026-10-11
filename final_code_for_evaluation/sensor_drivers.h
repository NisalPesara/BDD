// Small checked I2C drivers for the MAX30102 and ADXL345 (from 4.37, extended).
//
//  Why not the SparkFun MAX3010x library any more: its check() keeps only a 4-sample host ring,
//  so any loop pause longer than 32 ms (USB printing, NIRS reads) silently overwrote samples and
//  broke the uniform 125 Hz timing the filters and derivatives rely on. Here the 32-sample
//  hardware FIFO is drained in bursts (256 ms of head-room), RED and IR are read as one pair,
//  and the FIFO overflow counter tells the sketch when samples really were lost.
//  No external sensor library is needed (only Wire).
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "config.h"

namespace ppgio {
inline bool writeReg(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address); Wire.write(reg); Wire.write(value);
  return Wire.endTransmission() == 0;
}
inline bool readRegs(uint8_t address, uint8_t reg, uint8_t* data, size_t n) {
  Wire.beginTransmission(address); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, (uint8_t)n) != n) { while (Wire.available()) Wire.read(); return false; }
  for (size_t i = 0; i < n; ++i) data[i] = uint8_t(Wire.read());
  return true;
}

// ---------------------------------------------------------------- MAX30102
// SpO2 mode (RED + IR), ADC range 16384 nA, 1000 sps, 118 us pulse (16 bit), average 8 -> 125 Hz.
// Range was 4096 nA: on the wrist the ADC was 76 % full at only 12.8 mA, so the LEDs could not be driven
// brighter. 16384 nA = 4x head-room -> the LED AGC drives ~4x more current (more photons, less shot noise).
// A longer pulse is NOT possible here: the chip forces 215/411 us down to <= 800 sps (tested: 0x56 reads
// back 0x52), and 1000 sps / 8 is the only way to get the 125 Hz the filters are built for.
class Max30102 {
public:
  static const uint8_t REG_INTR1 = 0x00, REG_FIFO_WR = 0x04, REG_FIFO_OVF = 0x05, REG_FIFO_RD = 0x06,
                       REG_FIFO_DATA = 0x07, REG_FIFO_CFG = 0x08, REG_MODE = 0x09, REG_SPO2_CFG = 0x0A,
                       REG_LED1 = 0x0C, REG_LED2 = 0x0D, REG_PART_ID = 0xFF;
  static const uint8_t FIFO_CFG = 0x60;     // SMP_AVE = 8, rollover off
  static const uint8_t SPO2_CFG = 0x75;     // range 16384 nA (11), 1000 sps (101), 118 us (01)

  bool begin() {
    uint8_t id = 0;
    if (!readRegs(MAX30102_ADDR, REG_PART_ID, &id, 1) || id != 0x15) return false;
    if (!writeReg(MAX30102_ADDR, REG_MODE, 0x40)) return false;            // reset
    const uint32_t start = millis();
    do { delay(1); if (!readRegs(MAX30102_ADDR, REG_MODE, &id, 1)) return false; }
    while ((id & 0x40) && uint32_t(millis() - start) < 100);
    if (id & 0x40) return false;
    if (!writeReg(MAX30102_ADDR, 0x02, 0) || !writeReg(MAX30102_ADDR, 0x03, 0) ||   // no interrupts
        !writeReg(MAX30102_ADDR, REG_FIFO_CFG, FIFO_CFG) ||
        !writeReg(MAX30102_ADDR, REG_SPO2_CFG, SPO2_CFG) ||
        !writeReg(MAX30102_ADDR, REG_LED1, MAX_LED_CURRENT_CODE) ||                  // LED1 = RED
        !writeReg(MAX30102_ADDR, REG_LED2, MAX_LED_CURRENT_CODE) ||                  // LED2 = IR
        !writeReg(MAX30102_ADDR, REG_MODE, 0x03) || !clearFifo()) return false;     // SpO2 mode
    uint8_t actual = 0;
    return readRegs(MAX30102_ADDR, REG_SPO2_CFG, &actual, 1) && actual == SPO2_CFG;
  }
  // LED drive: code x 0.2 mA (0..255 -> 0..51 mA). Used by the automatic LED current control (led_agc.h).
  bool setLedCurrents(uint8_t redCode, uint8_t irCode) {
    return writeReg(MAX30102_ADDR, REG_LED1, redCode) && writeReg(MAX30102_ADDR, REG_LED2, irCode);
  }
  bool clearFifo() {
    uint8_t st[2], ovf = 0;
    if (!writeReg(MAX30102_ADDR, REG_FIFO_WR, 0) || !writeReg(MAX30102_ADDR, REG_FIFO_OVF, 0) ||
        !writeReg(MAX30102_ADDR, REG_FIFO_RD, 0) || !readRegs(MAX30102_ADDR, REG_INTR1, st, 2) ||
        !readRegs(MAX30102_ADDR, REG_FIFO_OVF, &ovf, 1)) return false;
    // A FULL FIFO has WR == RD (looks empty) and the pointer writes above do not clear it:
    // the overflow counter only resets once a sample is popped, so pop the 32 stored samples.
    if (ovf & 31) {
      uint32_t ir[32], red[32];
      if (!readPairs(ir, red, 32)) return false;
    }
    return true;
  }
  // count = unread samples (0..31); overflow = samples were lost; ambient = ambient-light cancellation overflow
  bool available(uint8_t& count, bool& overflow, bool& ambient) {
    uint8_t p[3], st[2];
    if (!readRegs(MAX30102_ADDR, REG_INTR1, st, 2) || !readRegs(MAX30102_ADDR, REG_FIFO_WR, p, 3)) return false;
    count = uint8_t((p[0] - p[2]) & 31);
    overflow = (p[1] & 31) != 0;
    ambient = (st[0] & 0x20) != 0;
    return true;
  }
  // Read n (<= 32) RED/IR pairs, in bursts that fit the 128-byte Wire buffer.
  bool readPairs(uint32_t* ir, uint32_t* red, int n) {
    int done = 0;
    while (done < n) {
      const int chunk = (n - done) > 16 ? 16 : (n - done);
      uint8_t b[16 * 6];
      if (!readRegs(MAX30102_ADDR, REG_FIFO_DATA, b, (size_t)chunk * 6)) return false;
      for (int k = 0; k < chunk; k++) {
        const uint8_t* q = b + 6 * k;
        red[done + k] = ((uint32_t(q[0]) << 16) | (uint32_t(q[1]) << 8) | q[2]) & 0x3FFFF;
        ir[done + k]  = ((uint32_t(q[3]) << 16) | (uint32_t(q[4]) << 8) | q[5]) & 0x3FFFF;
      }
      done += chunk;
    }
    return true;
  }
};

// ---------------------------------------------------------------- ADXL345
// Full resolution +/-4 g (3.9 mg/LSB), 200 Hz output data rate (bandwidth 100 Hz).
class Adxl345 {
public:
  bool present = false;
  bool begin(uint8_t address) {
    addr_ = address; uint8_t id = 0;
    present = readRegs(addr_, 0x00, &id, 1) && id == 0xE5 &&
              writeReg(addr_, 0x2D, 0) &&       // standby while configuring
              writeReg(addr_, 0x31, 0x09) &&    // FULL_RES, +/-4 g
              writeReg(addr_, 0x2C, 0x0B) &&    // 200 Hz
              writeReg(addr_, 0x38, 0) &&       // FIFO bypass: registers always hold the newest sample
              writeReg(addr_, 0x2D, 0x08);      // measure
    return present;
  }
  // Latest acceleration in m/s^2. Returns false on a bus error.
  bool read(float* xyz) {
    uint8_t b[6];
    if (!present || !readRegs(addr_, 0x32, b, 6)) return false;
    for (int i = 0; i < 3; ++i) {
      const int16_t v = int16_t(uint16_t(b[2 * i]) | (uint16_t(b[2 * i + 1]) << 8));
      xyz[i] = float(v) * 0.0039f * 9.80665f;
    }
    return true;
  }
private:
  uint8_t addr_ = ADXL345_FINGER_ADDR;
};
} // namespace ppgio

// Desktop stub of Wire with a simulated MAX30102 (32-sample FIFO) and two ADXL345.
#pragma once
#include "Arduino.h"
#include <vector>
#include <utility>
struct StubWire {
  uint8_t regs[128][256] = {};
  uint8_t address = 0, reg = 0;
  std::vector<uint8_t> tx; std::deque<uint8_t> rx;
  std::deque<std::pair<uint32_t, uint32_t>> fifo;   // (ir, red)
  int overflow = 0; bool fail = false, ambient = false;
  float acc[2][3] = {{0, 0, 9.81f}, {0, 0, 9.81f}};
  bool present53 = true, present1d = true;
  void begin(int, int) { regs[0x57][0xff] = 0x15; regs[0x53][0] = present53 ? 0xe5 : 0; regs[0x1d][0] = present1d ? 0xe5 : 0; }
  void setClock(int) {} void setTimeOut(int) {}
  void beginTransmission(uint8_t a) { address = a; tx.clear(); }
  void write(uint8_t b) { tx.push_back(b); }
  uint8_t endTransmission(bool = true) {
    if (fail) return 4;
    if (!tx.empty()) reg = tx[0];
    if (tx.size() == 2) {
      regs[address][reg] = tx[1];
      if (address == 0x57 && reg == 0x09 && tx[1] == 0x40) regs[address][reg] = 0;   // reset done instantly
      if (address == 0x57 && reg == 0x04) fifo.clear();
      if (address == 0x57 && reg == 0x05) overflow = 0;
    }
    return 0;
  }
  // push one sample into the FIFO (called every 8 ms of simulated time)
  void push(uint32_t ir, uint32_t red) {
    if (fifo.size() >= 32) { overflow++; fifo.pop_front(); }
    fifo.emplace_back(ir, red);
  }
  size_t requestFrom(uint8_t a, uint8_t n) {
    rx.clear(); if (fail) return 0;
    if (a == 0x57 && reg == 0x07) {
      if (n % 6 || fifo.size() < size_t(n / 6)) return 0;
      for (int k = 0; k < n / 6; k++) {
        auto v = fifo.front(); fifo.pop_front();
        for (uint32_t z : {v.second, v.first}) { rx.push_back((z >> 16) & 3); rx.push_back((z >> 8) & 255); rx.push_back(z & 255); }
      }
      return rx.size();
    }
    if ((a == 0x53 || a == 0x1d) && reg == 0x32 && n == 6) {
      const int s = (a == 0x53) ? 0 : 1;
      for (int i = 0; i < 3; i++) { int16_t v = int16_t(lroundf(acc[s][i] / (0.0039f * 9.80665f))); rx.push_back(uint8_t(v & 255)); rx.push_back(uint8_t((uint16_t)v >> 8)); }
      return rx.size();
    }
    for (size_t i = 0; i < n; ++i) {
      const uint8_t r = uint8_t(reg + i); uint8_t v = regs[a][r];
      if (a == 0x57) {
        if (r == 0x00) v = ambient ? 0x20 : 0;
        if (r == 0x04) v = uint8_t(fifo.size() & 31);   // write pointer (read pointer kept at 0)
        if (r == 0x05) v = uint8_t(overflow > 31 ? 31 : overflow);
        if (r == 0x06) v = 0;
      }
      rx.push_back(v);
    }
    return rx.size();
  }
  int available() { return int(rx.size()); }
  int read() { if (rx.empty()) return -1; int x = rx.front(); rx.pop_front(); return x; }
};
extern StubWire Wire;

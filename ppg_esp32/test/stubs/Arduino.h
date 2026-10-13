// Desktop stub of the Arduino/ESP32 API - ONLY for test/sketch_test.cpp (never used on the ESP32).
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string>
#include <deque>
#include <cmath>
#ifndef ARDUINO
#define ARDUINO 10800
#endif
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
constexpr int ADC_11db = 3;
extern uint64_t stubTimeUs;
extern int stubPinLevel[64];
float stubAnalogMv(int pin);                       // provided by the test
template <class T, class L, class H> inline T constrain(T x, L lo, H hi) { return x < (T)lo ? (T)lo : (x > (T)hi ? (T)hi : x); }
inline uint32_t millis() { return uint32_t(stubTimeUs / 1000); }
inline uint32_t micros() { return uint32_t(stubTimeUs); }
inline void delay(uint32_t ms) { stubTimeUs += uint64_t(ms) * 1000; }
inline void analogReadResolution(int) {}
inline void analogSetPinAttenuation(int, int) {}
inline uint32_t analogReadMilliVolts(int pin) { float v = stubAnalogMv(pin); return v < 0 ? 0 : uint32_t(v + 0.5f); }
inline int analogRead(int pin) { return int(stubAnalogMv(pin) * 4095.f / 3300.f); }
inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int v) { if (pin >= 0 && pin < 64) stubPinLevel[pin] = v; }
struct StubSerial {
  std::string output; std::deque<int> input; bool connected = true;
  void begin(int) {}
  explicit operator bool() const { return connected; }
  int availableForWrite() { return connected ? 4096 : 0; }
  size_t write(const uint8_t* p, size_t n) { output.append(reinterpret_cast<const char*>(p), n); return n; }
  int available() { return int(input.size()); }
  int read() { if (input.empty()) return -1; int x = input.front(); input.pop_front(); return x; }
  void println(const char* s) { output += s; output += "\n"; }
};
extern StubSerial Serial;

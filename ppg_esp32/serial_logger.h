#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <string.h>

// Fixed-size queue: USB disconnection or a slow terminal drops log records,
// never waits for room in the acquisition loop. Counts are included in STATUS.
class SerialLogger {
 public:
  uint32_t dropped = 0;
  bool enqueue(const char* line) {
    const size_t n = strlen(line);
    if (n >= CAP - used_) { ++dropped; return false; }
    for (size_t i = 0; i < n; ++i) { buffer_[head_] = line[i]; head_ = (head_ + 1) % CAP; }
    used_ += n; return true;
  }
  void pump() {
    if (!Serial || !used_) return;
    const int available = Serial.availableForWrite();
    if (available <= 0) return;
    size_t n = used_;
    if (n > size_t(available)) n = size_t(available);
    if (n > CAP - tail_) n = CAP - tail_;
    if (n > 128) n = 128;
    const size_t sent = Serial.write(reinterpret_cast<const uint8_t*>(buffer_ + tail_), n);
    tail_ = (tail_ + sent) % CAP; used_ -= sent;
  }
 private:
  static constexpr size_t CAP = 16384;
  char buffer_[CAP]; size_t head_ = 0, tail_ = 0, used_ = 0;
};

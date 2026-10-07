// m09_auc_calculator.h
#pragma once
#include <stdint.h>

class AucCalculator {
public:
  AucCalculator() { reset(); }

  // Resets the area accumulator at the start of a new heartbeat
  void reset() {
    accumulated_area = 0.0f;
    last_t_ms = 0;
    last_ppg = 0.0f;
    is_first_sample = true;
  }

  // Feeds a new sample into the integrator using the Trapezoidal rule
  float process(uint32_t t_ms, float ppg) {
    if (is_first_sample) {
      last_t_ms = t_ms;
      last_ppg = ppg;
      is_first_sample = false;
      return 0.0f;
    }

    // Convert time delta from milliseconds to seconds
    float dt = (t_ms - last_t_ms) / 1000.0f;

    // Trapezoidal Area = width * average height
    // If you want to isolate just the pulsatile volume above the baseline, 
    // ensure the 'ppg' signal passed here has already had its minimum trough value subtracted.
    float area = dt * ((ppg + last_ppg) / 2.0f);
    
    // Only accumulate positive areas to prevent baseline dips from subtracting volume
    if (area > 0) {
      accumulated_area += area;
    }

    last_t_ms = t_ms;
    last_ppg = ppg;

    return accumulated_area;
  }

  // Retrieve the final calculated area for the pulse
  float get_auc() const { return accumulated_area; }

private:
  float accumulated_area;
  uint32_t last_t_ms;
  float last_ppg;
  bool is_first_sample;
};

// METHOD 6b - Reference signals for the motion canceller from TWO accelerometers.
//  Idea: PPG motion artifact is mainly the sensor moving RELATIVE to the skin/tissue. One
//  accelerometer sees only how the whole hand moves; the difference between a finger-side and a
//  band-side sensor is closer to the relative motion at the light sensor. (This two-sensor
//  relative-motion scheme is NOT from Park 2022, which cites single-accelerometer ANC.)
//
//  Input : raw[0..2] finger xyz, raw[3..5] band xyz (already axis-mapped), m/s^2
//  Output: ANC_MODE_ABS6 -> finger xyz, band xyz          (gravity/tilt removed by a 0.4 Hz high-pass)
//          ANC_MODE_REL9 -> finger xyz, rel-acc xyz, rel-velocity xyz
//            rel-acc = finger - band
//            rel-vel = LEAKY integral of rel-acc (a pure integral drifts; leak corner ACC_VEL_LEAK_HZ)
#pragma once
#include <math.h>
#include "config.h"
struct AccelRelative {
  float dc[6], vel[3], leak = 0.98f; bool init = false;
  void begin() { init = false; for (int i = 0; i < 6; i++) dc[i] = 0; for (int i = 0; i < 3; i++) vel[i] = 0;
                 leak = expf(-6.2831853f * ACC_VEL_LEAK_HZ / PPG_FS); }
  void process(const float* raw, float* out) {
    float a[6];
    for (int i = 0; i < 6; i++) {
      if (!init) dc[i] = raw[i];
      dc[i] += 0.02f * (raw[i] - dc[i]);          // remove gravity / slow tilt
      a[i] = raw[i] - dc[i];
    }
    init = true;
#if ANC_MODE == ANC_MODE_REL9
    for (int i = 0; i < 3; i++) {
      float rel = a[i] - a[3 + i];
      vel[i] = vel[i] * leak + rel / PPG_FS;
      out[i] = a[i]; out[3 + i] = rel; out[6 + i] = vel[i] * ACC_VEL_SCALE;
    }
#else
    for (int i = 0; i < 6; i++) out[i] = a[i];
#endif
  }
};

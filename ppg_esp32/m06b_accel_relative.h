// METHOD 6b - Reference signals for the motion canceller from TWO accelerometers.
//  Idea: PPG motion artifact is mainly the sensor moving RELATIVE to the skin/tissue. When the whole
//  hand moves, finger-side and band-side sensors move together and the light path does not change;
//  when the probe moves relative to the hand, the PPG is corrupted. So only the DIFFERENCE between the
//  two sensors is used: common (whole-hand) motion cancels, relative motion remains.
//  (This two-sensor relative-motion scheme is NOT from Park 2022, which cites single-accelerometer ANC.)
//
//  Frame alignment: the two ADXL345 are never mounted exactly parallel, so "finger - band" axis by axis
//  would NOT cancel common motion. A 3x3 matrix M maps band axes into the finger frame and is learned
//  online (NLMS, ACC_ALIGN_MU) while the band sensor moves: it converges to the M that best explains
//  the finger motion from the band motion, i.e. the common (rigid) part. The residual is the relative
//  motion. M is kept across signal resets (only begin(true) clears it).
//
//  Input : raw[0..2] finger xyz, raw[3..5] band xyz (BAND_AXIS_* mapping already applied), m/s^2
//  Output: ANC_MODE_ABS6 -> finger xyz, band xyz          (gravity/tilt removed by a 0.4 Hz high-pass)
//          ANC_MODE_REL9 -> finger xyz, rel-acc xyz, rel-velocity xyz
//          ANC_MODE_REL6 -> rel-acc xyz, rel-velocity xyz (pure relative motion)
//          ANC_MODE_RELH9 -> rel-acc xyz, rel-velocity xyz, hand (band) acc xyz   (default)
//            Recording 2026-10-12: whole-hand motion ALSO corrupts the PPG (blood / tissue inertia in the
//            finger), so the hand acceleration is a reference too. Offline replay: REL6 removed 84 % of
//            the motion power, RELH9 90 %, both leaving the pulse at rest unchanged (<= 1 %).
//            rel-acc = finger - M * band
//            rel-vel = LEAKY integral of rel-acc (a pure integral drifts; leak corner ACC_VEL_LEAK_HZ)
//  relEnergy(): |rel-acc|^2 of the last sample, (m/s^2)^2  - used for the relative-motion gate (m12).
#pragma once
#include <math.h>
#include "config.h"
struct AccelRelative {
  float dc[6], vel[3], rel[3], M[3][3], leak = 0.98f, relE = 0, absE = 0; bool init = false;

  AccelRelative() { begin(true); }
  // clearAlignment = false keeps the learned sensor-to-sensor rotation (signal-only reset)
  void begin(bool clearAlignment = false) {
    init = false; relE = absE = 0;
    for (int i = 0; i < 6; i++) dc[i] = 0;
    for (int i = 0; i < 3; i++) vel[i] = rel[i] = 0;
    leak = expf(-6.2831853f * ACC_VEL_LEAK_HZ / PPG_FS);
    if (clearAlignment) for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) M[r][c] = (r == c) ? 1.0f : 0.0f;
  }

  void process(const float* raw, float* out) {
    float a[6];
    for (int i = 0; i < 6; i++) {
      if (!init) dc[i] = raw[i];
      dc[i] += 0.02f * (raw[i] - dc[i]);          // remove gravity / slow tilt
      a[i] = raw[i] - dc[i];
    }
    init = true;

    // relative acceleration = finger - M * band, then adapt M toward the common (rigid) motion
    const float* f = a; const float* b = a + 3;
    const float pb = b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
    relE = 0; absE = f[0] * f[0] + f[1] * f[1] + f[2] * f[2];
    for (int r = 0; r < 3; r++) {
      rel[r] = f[r] - (M[r][0] * b[0] + M[r][1] * b[1] + M[r][2] * b[2]);
      relE += rel[r] * rel[r];
    }
#if ACC_ALIGN
    if (pb >= ACC_ALIGN_MIN_ENERGY) {               // learn only while the band really moves
      const float g = ACC_ALIGN_MU / (1e-3f + pb);
      for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) M[r][c] += g * rel[r] * b[c];
    }
#endif
    for (int i = 0; i < 3; i++) vel[i] = vel[i] * leak + rel[i] / PPG_FS;

#if ANC_MODE == ANC_MODE_REL9
    for (int i = 0; i < 3; i++) { out[i] = a[i]; out[3 + i] = rel[i]; out[6 + i] = vel[i] * ACC_VEL_SCALE; }
#elif ANC_MODE == ANC_MODE_REL6
    for (int i = 0; i < 3; i++) { out[i] = rel[i]; out[3 + i] = vel[i] * ACC_VEL_SCALE; }
#elif ANC_MODE == ANC_MODE_RELH9
    for (int i = 0; i < 3; i++) { out[i] = rel[i]; out[3 + i] = vel[i] * ACC_VEL_SCALE; out[6 + i] = b[i]; }
#else
    for (int i = 0; i < 6; i++) out[i] = a[i];
#endif
  }
  float relEnergy() const { return relE; }        // relative motion (finger vs band)
  float absEnergy() const { return absE; }        // whole finger-sensor motion
};

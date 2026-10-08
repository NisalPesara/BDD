// Derivative signals used for fiducial detection (Park 2022, Fig. 5; pyPPG vpg/apg/jpg)
//  VPG = 1st derivative (velocity plethysmogram)
//  APG = 2nd derivative (acceleration plethysmogram, a-b-c-d-e waves, Takazawa 1998)
//  JPG = 3rd derivative
//  Units: signal-units per second^k. pyPPG's scaling may differ - rescale if your
//  detector uses absolute thresholds.
#pragma once
#include "m04_smoothing.h"
struct Derivatives {
  int fs = 125; float pP = 0, pV = 0, pA = 0; bool first = true;
  MovingAverage smV, smA, smJ;
  void begin(int fs_, int msV, int msA, int msJ) {
    fs = fs_; pP = pV = pA = 0; first = true;
    smV.begin(fs, msV); smA.begin(fs, msA); smJ.begin(fs, msJ);
  }
  void process(float p, float& v, float& a, float& j) {
    if (first) { pP = p; first = false; }
    v = smV.process((p - pP) * fs);
    a = smA.process((v - pV) * fs);
    j = smJ.process((a - pA) * fs);
    pP = p; pV = v; pA = a;
  }
};

// METHOD 5 - Amplitude regularisation (Kim, Lee & Shin 2019, "amplitude compensation curve")
//  The paper builds a compensation curve from the PPG envelope. Causal, streaming
//  approximation: track the upper envelope with a fast attack / slow decay and divide.
//  WARNING: removes absolute amplitude -> do NOT use if you need systolic amplitude,
//  perfusion index or other amplitude features. Off by default.
#pragma once
#include <math.h>
struct AmplitudeRegularizer {
  float env = 0, decay = 0.999f, eps = 1e-6f;
  void begin(int fs, float decaySeconds = 4.0f) { decay = expf(-1.0f / (fs * decaySeconds)); env = 0; }
  float process(float x) {
    float a = fabsf(x);
    env = (a > env) ? a : env * decay;
    return x / (env > eps ? env : eps);
  }
};

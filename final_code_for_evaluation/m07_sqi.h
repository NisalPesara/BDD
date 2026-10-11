// METHOD 7 - Signal Quality Index (Park 2022, "Signal Quality Index")
//  Rule-based SQI on a sliding window (SQI_WINDOW_S, new result every SQI_HOP_S):
//   - skewness & kurtosis  (Krishnan 2008; Selvaraj 2011; Elgendi 2016)
//   - Shannon entropy      (Selvaraj 2011)  -> flat-line / saturation
//   - perfusion index      (Gehring 2002; Cannesson 2008)  AC/DC * 100
//   - periodicity          (Karlen 2012 idea): best local maximum of the autocorrelation of a
//                           detrended, decimated (~25 Hz) copy, lags 0.33 .. 1.5 s (180..40 bpm)
//   - clip fraction        flat-topped (saturated) pulses
//  `fail` is a bitmask telling WHICH rule rejected the window (SQI_F_* in ppg_sqi.h).
//  The maths is in ppg_sqi.h (minor-noise library); this file binds it to config.h.
#pragma once
#include "config.h"
#include "ppg_sqi.h"

typedef ppgnr::SqiOut SqiResult;       // fresh, good, fail, periodicity, skew, kurt, entropy, pi, clipFrac

class SqiWindow : public ppgnr::Sqi<PPG_FS, SQI_WINDOW_S, SQI_HOP_S> {
public:
  void begin() {
    ppgnr::Sqi<PPG_FS, SQI_WINDOW_S, SQI_HOP_S>::begin();
    th.minPeriodicity = SQI_MIN_PERIODICITY;
    th.minSkew        = SQI_MIN_SKEW;
    th.kurtMin        = SQI_KURT_MIN;   th.kurtMax    = SQI_KURT_MAX;
    th.entropyMin     = SQI_MIN_ENTROPY; th.entropyMax = SQI_MAX_ENTROPY;
    th.flatSd         = SQI_FLAT_SD;
    th.maxClipFrac    = SQI_MAX_CLIP_FRAC;
  }
};

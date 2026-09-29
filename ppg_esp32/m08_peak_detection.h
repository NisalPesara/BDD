// METHOD 8 - Adaptive-threshold peak detection (Shin, Lee & Lee 2009)
//  Threshold tracks the signal amplitude; peaks accepted only outside a refractory
//  period. Robust to slow baseline changes (unlike a fixed-threshold LCM detector).
//  Included as a REFERENCE / helper (beat flag, heart-rate). Your own wave-properties
//  code can replace it. A peak is reported one sample late (needs the next sample to
//  confirm the local maximum): the peak is at sample index (n-1).
#pragma once
struct AdaptivePeakDetector {
  int fs = 125, refractory = 37;
  unsigned long n = 0, lastPeak = 0;
  float p1 = 0, p2 = 0, amp = 0, thr = 0, maxSeen = 0;
  float decay = 0.9998f;
  int learnSamples = 250; bool learned = false;
  float lastIbiSec = 0;

  void begin(int fs_) {
    fs = fs_; refractory = (int)(0.30f * fs); learnSamples = 2 * fs;
    n = lastPeak = 0; p1 = p2 = amp = thr = maxSeen = 0; learned = false; lastIbiSec = 0;
  }
  bool process(float x) {
    n++;
    if (!learned) {                                   // first 2 s: learn amplitude
      if (x > maxSeen) maxSeen = x;
      if ((int)n >= learnSamples) { amp = 0.8f * maxSeen; thr = 0.5f * amp; learned = true; }
      p2 = p1; p1 = x; return false;
    }
    bool found = false;
    bool localMax = (p1 > p2 && p1 >= x);
    if (localMax && p1 > thr && (n - 1 - lastPeak) > (unsigned long)refractory) {
      if (lastPeak) lastIbiSec = (float)(n - 1 - lastPeak) / fs;
      lastPeak = n - 1; found = true;
      amp = 0.75f * amp + 0.25f * p1; thr = 0.5f * amp;
    } else {
      thr *= decay; if (thr < 0.25f * amp) thr = 0.25f * amp;
    }
    p2 = p1; p1 = x;
    return found;
  }
  float bpm() const { return lastIbiSec > 0 ? 60.0f / lastIbiSec : 0; }
};

#pragma once
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "spo2_table.h"

// Bounded revision of the user-supplied Maxim-style valley/ratio algorithm.
// Attribution and upstream license: docs/THIRD_PARTY_NOTICES.md.
// Modifications alter results; this is NOT a validated pulse oximeter.
namespace ppgspo2 {
constexpr int FS = 25, WINDOW = 100;
struct Result {
  bool valid = false, hrValid = false;
  int spo2 = -1;
  float bpm = NAN, ratio = NAN, correlation = NAN;
};
template<class T> inline void sort(T* x, int n) {
  for (int i = 1; i < n; ++i) {
    const T v = x[i]; int j = i;
    while (j > 0 && x[j-1] > v) { x[j] = x[j-1]; --j; }
    x[j] = v;
  }
}
inline Result calculate(const uint32_t* ir, const uint32_t* red, int length) {
  Result out;
  if (!ir || !red || length != WINDOW) return out;
  double mi = 0, mr = 0;
  for (int i = 0; i < WINDOW; ++i) {
    if (!ir[i] || !red[i] || ir[i] > 262143 || red[i] > 262143) return out;
    mi += ir[i]; mr += red[i];
  }
  mi /= WINDOW; mr /= WINDOW;
  double ei = 0, er = 0, cross = 0;
  for (int i = 0; i < WINDOW; ++i) {
    const double a = ir[i] - mi, b = red[i] - mr;
    ei += a*a; er += b*b; cross += a*b;
  }
  if (ei <= 0 || er <= 0) return out;
  out.correlation = float(cross / sqrt(ei*er));
  if (out.correlation < .8f || sqrt(ei/WINDOW)/mi < .0001 || sqrt(er/WINDOW)/mr < .0001)
    return out;
  constexpr int SMOOTH = WINDOW - 4 + 1;
  float x[SMOOTH]; double threshold = 0;
  for (int i = 0; i < SMOOTH; ++i) {
    x[i] = float(mi - (double(ir[i]) + ir[i+1] + ir[i+2] + ir[i+3]) / 4.0);
    threshold += x[i];
  }
  threshold = fmax(30., fmin(60., threshold / SMOOTH));
  int candidates[SMOOTH], nc = 0;
  for (int i = 1; i < SMOOTH - 1;) {
    if (x[i] > threshold && x[i] > x[i-1]) {
      int end = i + 1;
      while (end < SMOOTH && x[end] == x[i]) ++end;
      if (end < SMOOTH && x[i] > x[end]) candidates[nc++] = i;
      i = end;
    } else ++i;
  }
  // Strongest candidates first, then reject neighbors before restoring time order.
  for (int i = 1; i < nc; ++i) {
    int v = candidates[i], j = i;
    while (j > 0 && x[candidates[j-1]] < x[v]) { candidates[j] = candidates[j-1]; --j; }
    candidates[j] = v;
  }
  int valleys[15], nv = 0;
  for (int i = 0; i < nc && nv < 15; ++i) {
    bool near = false;
    for (int j = 0; j < nv; ++j) if (abs(candidates[i] + 1 - valleys[j]) < 8) near = true;
    if (!near) valleys[nv++] = candidates[i] + 1; // center of 4-point smoothing (rounded)
  }
  sort(valleys, nv);
  if (nv < 2) return out;
  out.bpm = 60.f * FS * (nv - 1) / (valleys[nv-1] - valleys[0]);
  out.hrValid = out.bpm >= 40 && out.bpm <= 180;
  double ratios[14]; int nr = 0;
  for (int k = 0; k + 1 < nv; ++k) {
    const int a = valleys[k], b = valleys[k+1], dt = b-a;
    if (dt < 8 || dt > 38) continue;
    int ix = a, iy = a;
    for (int i = a; i <= b; ++i) {
      if (ir[i] > ir[ix]) ix = i;
      if (red[i] > red[iy]) iy = i;
    }
    const double acI = double(ir[ix]) - (double(ir[a]) + (double(ir[b])-ir[a])*(ix-a)/dt);
    const double acR = double(red[iy]) - (double(red[a]) + (double(red[b])-red[a])*(iy-a)/dt);
    // Double intermediates prevent the 18-bit products overflowing int32_t.
    if (acI > 0 && acR > 0 && ir[ix] && red[iy])
      ratios[nr++] = 100.0 * acR * ir[ix] / (acI * red[iy]);
  }
  if (!nr || !out.hrValid) return out;
  sort(ratios, nr);
  const double ratio = nr % 2 ? ratios[nr/2] : .5 * (ratios[nr/2-1] + ratios[nr/2]);
  out.ratio = float(ratio / 100.);
  const int index = int(lround(ratio));
  if (index <= 2 || index >= int(sizeof(table)/sizeof(table[0]))) return out;
  out.spo2 = table[index];
  // 70..100 is an output/calibration-domain gate; it is NOT a 'normal' range.
  out.valid = out.spo2 >= 70 && out.spo2 <= 100;
  return out;
}
} // namespace ppgspo2

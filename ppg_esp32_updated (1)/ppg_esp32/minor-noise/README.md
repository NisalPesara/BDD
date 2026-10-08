# ppg_noise - separate noise-removal libraries for the PPG pipeline

Header-only, no dependency on config.h (everything is a runtime or template parameter), C++11, no heap.
Each header works alone; `ppg_noise.h` includes all and provides a ready-made chain.

| Header | Removes / handles | Notes |
|---|---|---|
| ppg_biquad.h | drift, respiration, broadband noise | Butterworth HP/LP/BP designed at run time (any fs, fc), prime() removes start-up transient |
| ppg_dc.h | DC offset, dark current, gain (pigmentation, thickness) | DC tracker, AC/DC normalise, sliding perfusion index |
| ppg_spike.h | EMI bursts, I2C/ADC glitches | Hampel; delay 3 samples (W=7) |
| ppg_savgol.h | noise in the PPG and its derivatives | smooth + d/dt + d2/dt2, time-aligned, delay HALF samples |
| ppg_anc.h | motion artifact | multi-reference NLMS (accelerometers, force slope) with adaptation gating |
| ppg_force.h | FSR noise, contact state | conductance model, smoothed force + slope, NONE/OK/HIGH + stable flag |
| ppg_sqi.h | whatever cannot be repaired | rule-based SQI with fail bitmask, clip detection, RawClipMonitor for ADC saturation |

## Chain (ppg_noise.h)
```
raw -> Hampel -> DC tracker -> [invert] -> high-pass
refs -> delay -> same high-pass ----------> NLMS ANC -> low-pass -> Savitzky-Golay -> clean, vpg, apg
```
```cpp
#include "ppg_noise.h"
ppgnr::PpgDenoiser<3, 8, 5, 7> den;      // 3 references, 8 taps, SG window 11, Hampel window 7
ppgnr::DenoiseConfig c; c.fs = 125; c.useAnc = true; c.ancPmin = 0.5f;   // tune pmin from den ANC energy at rest
den.begin(c);
// every sample:
ppgnr::DenoiseOut o = den.process(irCounts, refs /*3 zero-mean floats or nullptr*/, sqiGood);
// o.clean, o.vpg, o.apg, o.dc, o.valid
```
References must be time-aligned with the PPG and zero-mean (the chain high-passes them with the same filter as the PPG).

## Test
`g++ -O2 -std=c++17 -Wall -I. -Ippg_noise test_ppgnr.cpp -o tn && ./tn` (needs the project headers for comparisons with m06/m07/m08).

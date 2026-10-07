# PPG Signal Quality Index (SQI)

This folder contains standalone, fixed-memory C++ signal-processing headers
for PPG conditioning and quality estimation. The modules do not depend on
`config.h` or on one another, so callers can select only the stages their
hardware and recordings need.

## Files

- `sqi.hpp` — `SqiWindow<WindowSamples, SampleRateHz>` and configurable
  thresholds in `SqiOptions`.
- `test_sqi.cpp` — clean pulse, noise, flatline, low-perfusion and clipped
  waveform checks.
- `ppg_biquad.h` — runtime-designed RBJ biquad sections and second/fourth-order
  Butterworth low-pass/high-pass plus a cascaded band-pass.
- `ppg_dc_removal.h` — one-pole baseline estimator and DC subtraction.
- `ppg_moving_average.h` — fixed-window O(1)-per-sample smoothing.
- `ppg_nlms_anc.h` — multi-reference normalized-LMS motion-artifact canceller.
- `test_filters.cpp` — synthetic frequency response, baseline, moving average,
  and adaptive-cancellation checks.

## Build and run

With a C++11-or-newer compiler:

```text
g++ -std=c++11 -O2 -Wall -Wextra -pedantic test_sqi.cpp -o test_sqi
./test_sqi
```

On Windows with MinGW, compile each test source to its own executable and run
it in PowerShell:

```text
g++ -std=c++11 -O2 -Wall -Wextra -pedantic test_sqi.cpp -o test_sqi.exe
.\test_sqi.exe
g++ -std=c++11 -O2 -Wall -Wextra -pedantic test_filters.cpp -o test_filters.exe
.\test_filters.exe
```

## Standalone noise-removal modules

### Biquad and Butterworth filters

```cpp
#include "ppg_biquad.h"

PpgButterworth bandPass;
if (!bandPass.configureBandPass(125.0f, 0.2f, 8.0f)) {
  // Reject invalid rates or cutoff frequencies.
}
float filtered = bandPass.process(sample);
```

`PpgBiquad` offers individual low-pass, high-pass, and constant-skirt
band-pass sections. `PpgButterworth` offers second- or fourth-order
low/high-pass filters and a high-pass followed by low-pass band-pass. Cutoffs
are specified in Hz at runtime and must be strictly below Nyquist. Each
successful configure resets filter state. Biquad stages are causal and have
frequency-dependent phase delay; use only as much filtering as the waveform
and feature timing allow.

### DC/baseline removal

```cpp
#include "ppg_dc_removal.h"

PpgDcRemoval baseline;
baseline.configure(125.0f, 2.0f);  // sample rate and baseline time constant
float ac = baseline.process(rawIr);
```

The first sample seeds the baseline and returns zero. Thereafter the one-pole
baseline follows slow drift, and the returned output is input minus baseline.
A longer time constant removes slower drift but preserves more of it in the
output. This high-pass operation can distort pulse shape if set too
aggressively. Before successful configuration, `process()` passes samples
through unchanged.

### Moving-average smoothing

```cpp
#include "ppg_moving_average.h"

PpgMovingAverage<5> smoother;
float smooth = smoother.process(sample);
```

The template fixes memory use at compile time. During startup it averages only
samples received so far. It is a boxcar low-pass with a measurable group delay
of `(WindowSamples - 1) / 2` samples.

### Accelerometer-referenced motion cancellation

```cpp
#include "ppg_nlms_anc.h"

PpgNlmsAnc<6, 16> canceller;  // e.g. two 3-axis accelerometers
canceller.configure(0.05f, 1.0e-3f);
float cleaned = canceller.process(contaminatedPpg, accelReferences, adapt);
```

The reference array must contain `References` simultaneous, finite samples.
Provide gravity-removed, zero-mean motion references where practical. The
returned error is the cleaned PPG estimate. Set `adapt` false to freeze
learning when a separate quality check says the PPG is unreliable. NLMS only
removes motion components correlated with the references; it cannot recover
clipped data or guarantee preservation when the reference contains pulse
information.

All cutoffs, window sizes, and adaptation parameters are starting points, not
validated limb-PPG settings. Validate on recorded signals at the target
sample rate and hardware.

## Integration

Instantiate with compile-time window and sample rate, and set the hop and
thresholds for the sensor and acquisition pipeline:

```cpp
SqiOptions options;
options.hopSamples = 200;  // 2 seconds at 100 Hz
options.minPiPercent = 0.1f;  // example starting point; tune with recordings
SqiWindow<800, 100> sqi(options);

SqiResult result = sqi.push(cleanPpgSample, rawDcLevel);
if (result.fresh) {
  // Consume result.good and the reported feature values.
}
```

The first result is emitted as soon as the window fills; subsequent results
are emitted once per `hopSamples`. The DC input is averaged over the same
window as the signal. PI is window peak-to-peak AC divided by absolute mean DC,
expressed as a percent. All threshold defaults are intentionally permissive;
they are not clinical limits and must be tuned on representative recordings.

Autocorrelation searches lags corresponding to 40–180 bpm. It rejects the
small-lag smoothness peak by looking for the first positive local maximum
after a zero crossing. The calculation still requires a cross-product sum
for each candidate lag; prefix sums make the two overlap-energy terms O(1)
per lag. `push()` computes the features synchronously when a result is due,
so production firmware should measure execution time at its target sample
rate and schedule computation appropriately.

This is signal-processing example code, not a medical device component or
clinical decision rule.

# Integration review and design decisions

## Source scope

Reviewed the uploaded `BDD-main (1).zip`, its `ppg_esp32` firmware and `Libraries`/`PPG-SQI-Improved` proposals, `minor-noise.zip`, the embedded Maxim-style code in `text1.txt`, and the included Park et al. PPG review. The nested noise ZIP was also inspected: all ten files match the outer archive copies byte-for-byte. The directly supplied headers are the integrated baseline. The public GitHub folder and raw configuration URL could not be retrieved. This package therefore makes no claim to include changes made after the uploaded BDD snapshot. No Claude share URL was supplied or found; that research conversation was not reviewed.

## What was changed

| Original or proposed component | Decision and reason |
|---|---|
| `pipeline.h` | Replaced the old ANC-before-filter/Chebyshev/MA path with the modular denoiser and explicit quality gates |
| Proposed `Libraries/newconfig.h` | Incorporated two ADXL345s and FSR allocation; corrected GPIO22/23 to S3 GPIO8/9; do not automatically enable ANC just because two accelerometers exist |
| MAX30102 read loop | Checked paired RED/IR reads directly from the device FIFO; no small hidden library ring; sequence-derived nominal timestamps |
| Missing samples and contact loss | Reset filters, beat history, SpO2 and baseline; never silently skip low-count frames and stitch discontinuous pulses |
| FIFO backlog | Discard a backlog of more than one pair and start a new epoch. Current accelerometer readings cannot be assigned to old optical frames reliably. Discards are counted |
| Serial reports | Replaced many prints per sample with typed CSV records and a bounded queue. Log backpressure drops records and increments a counter |
| `Libraries/smoothing.h` | Kept and tested as `m04_smoothing.h`; amortized O(1), with O(W) re-sums once per window lap. Re-summing bounds accumulated floating-point error, not mathematically zero error |
| `minor-noise/ppg_anc.h` | Integrated mirrored buffers, energy gating and adaptation control; added finite-input protection |
| ANC reference filtering | Matched both DC removal and high-pass, plus the Hampel delay; reset adaptive state when references disappear |
| `m06b` relative acceleration/velocity | Left in legacy. Six absolute axes are simpler to validate; reference velocity is not blood velocity and a second accelerometer is not mandatory |
| `ppg_sqi.h` | Uses decimated detrended autocorrelation, rule bitmask, finite-input rejection, immediate first full-window result and DC averaged over the same window |
| Alternative `sqi.hpp` | Did not duplicate SQI engines. In the supplied version `prefixEnergy_[0]` was not explicitly initialized; the active version writes its prefix origin before every calculation |
| Raw clipping | Checked before spike repair or filtering, so filtering cannot make saturation appear valid; ambient-cancellation overflow also invalidates acquisition |
| Spike rejection | Five-sample Hampel before IIR; may still alter legitimate narrow pulses, so compare against raw data |
| SG derivatives | One cubic least-squares window gives aligned PPG/VPG/APG; removed cascaded finite differences and their unequal delays |
| `m08` peak detector | Prominence threshold plus adaptive refractory period; the supplied 45%-diastolic-wave failure is a regression test; startup supports 40-180 bpm including sample quantization |
| `Libraries/wave_detection.h` | Integrated onset-to-onset segmentation; whole-pulse quality checks, ring bounds and aligned time fields; same baseline for amplitude, width and AUC |
| AUC request | Added `m09_auc_calculator.h`; integrates complete accepted pulses using seconds, with one linear onset-to-onset baseline |
| Clinical-looking indices | Removed assumed-height stiffness and aging-index reporting; APG extrema/ratios remain exploratory, not part of pulse-trend decisions |
| Main purpose | Added explicit baseline capture, pulse timeout, persistent relative-amplitude/PI trend, pressure/motion quality states and experimental SpO2 |
| NIRS | Reserved nonconflicting pins and supplied a disabled-by-default analog adapter; no invented wavelengths, driver or tissue-saturation calibration |
| PlatformIO / Arduino | Self-contained Wire drivers, no MPU6050 dependency; both entry points reference the same firmware |

## Why peaks and derivatives alone are insufficient

A derivative describes the rate of change of the measured optical waveform, not the direction of blood flow. Peak timing primarily measures pulse repetition. Neither alone resolves pulse strength, contact changes, sensor saturation, pressure changes, or the difference between no detectable pulse and a measurement failure.

The active feature set adds pulse amplitude, explicitly defined AC/DC PI, whole-pulse area, width, crest time, pulse presence/timeout, and a fixed within-session baseline. These are **relative optical pulsatility features**. Skin/tissue properties, venous pulsation, ambient light, placement, compression and instrument settings all affect them. Even simultaneous PPG and NIRS measurements need an independently validated interpretation model to establish limb perfusion.

## AUC and the proposed integrator

The supplied timestamp trapezoid loop is a useful numerical idea, but it does not define a complete pulse, its baseline, missing-data handling, or what its units mean. Clipping each negative trapezoid to zero can bias area upward and obscure filter undershoot. Integrating peak-to-peak and labeling the start as an onset further changes the feature.

Here `WaveAnalyzer` finds two onsets and a complete pulse one beat later. A straight line between both onset values is subtracted. `PulseAuc::integrate()` uses trapezoids at `dt=1/125 s`. Total, systolic and diastolic areas all use this same baseline; the sub-areas therefore sum to the total. Areas are signed, in ADC-count seconds. An absent notch leaves its sub-areas unavailable. A positive area is required for baseline/trend updates. AUC is not converted into milliliters or mL/min.

## Sampling, smoothing and derivative choices from text1.txt

`ms` is a time-window specification, not a sampling rate. A moving average generally uses `round(fs * ms / 1000)` samples. A centered SG filter needs an odd number of samples and a polynomial order below that window size.

| Requested window | Moving-average samples at 125 Hz | Practical SG window | Approximate interpretation |
|---|---:|---:|---|
| 40 ms | 5 | 5 | Little smoothing; noisy APG |
| 80 ms | 10 | 11 (88 ms) | Less morphology smoothing, more derivative noise |
| 120 ms | 15 | 15 (120 ms) | Default compromise to investigate |
| 200 ms | 25 | 25 (200 ms) | Can blur short features and timing |

The default is **15-sample cubic SG**, not a 120 ms moving average. Try `SG_HALF=5` for 11 samples or `SG_HALF=9` for 19 (152 ms) on actual data. A longer polynomial window may stabilize derivatives while still changing notch morphology. The team's claimed 15/19-sample b/a results are synthetic observations, not universal optima; this update does not claim to reproduce their exact b/a numbers.

| Derivative method | Accuracy / noise | Streaming cost and alignment |
|---|---|---|
| `(x[n]-x[n-1])/dt` | Backward difference, first-order approximation; amplifies high-frequency noise | O(1), one previous value; effectively centered half a sample earlier |
| Forward difference | Same order/noise; requires a later sample for an estimate at the chosen time | Delayed real-time output |
| Centered difference | Second-order approximation; still noise-sensitive | O(1), small buffer, one-sample delay |
| SG polynomial derivative | Local polynomial fit averages noise and returns derivatives at a common center | O(W) multiply-adds, fixed buffer, 7-sample delay here |

The expression in text1.txt is a backward difference, despite its label. `dt=0.008 s` for this 125 Hz morphology path; derivatives are counts/s and counts/s^2. The separate SpO2 path has a 0.04 s interval after decimation. The active implementation does not compute JPG because it was not needed for the chosen robust features and is more noise-sensitive.

## SpO2 corrections and remaining limits

At 25 Hz, 100 samples span 4 seconds. At 100 Hz they span 1 second; at this firmware's 125 Hz they span 0.8 second. Changing only `FreqS` cannot fix the original buffering and algorithm assumptions.

This version maintains a separate paired raw branch. Both channels receive identical 6 Hz low-pass filtering, followed by average/decimate by five. The 25 Hz data fill a 100-sample window with 25-sample hops. DC is retained and channels stay aligned. This changes the preprocessing of the supplied Maxim-style algorithm and requires validation; it is not a guarantee of improved SpO2 accuracy. The decimator is a practical low-pass, not an ideal brick-wall filter.

Corrections in the supplied code include removal of literal `[cite: ...]` fragments that break C++; bounded plateau look-ahead; exact buffer-length checks; use of only initialized moving-average samples; correct IR own-peak indexing; double-precision products instead of overflowing 32-bit AC/DC products; correct odd/even median; explicit empty-ratio rejection; per-instance processing state; consistent table length; and clearing stale estimates after quality/timing failures. There are 183 explicit values in the supplied table even though its declared size is 184; bounds now use its actual size rather than silently using an extra zero.

The reference-style threshold and lookup calibration are retained as experimental starting points. Added red/IR correlation and amplitude checks reject some unusable windows but are not clinical validation. The 70-100% gate specifies an output domain, not a normal physiological range. Do not report an out-of-domain result as a normal reading or clamp it to 70%. A result from a limb reflectance site is not interchangeable with a validated fingertip oximeter.

## Research claims deliberately not made

The teammate's reported correlations, b/a distributions, timing estimates and motion-removal ceiling are not carried forward as measured results of this integration. Synthetic success cannot demonstrate that motion is fully removed, that notch timing is physiological, or that a hardware system is clinically reliable. The separate validation document records the actual tests run on this package.

## Primary references checked

- Espressif ESP32-S3 GPIO documentation: https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/gpio.html
- Analog Devices MAX30102 datasheet, FIFO averaging, left-justified data, legal rate/pulse-width combinations and register layout: https://www.analog.com/media/en/technical-documentation/data-sheets/max30102.pdf
- Analog Devices ADXL345 datasheet, addresses, full-resolution scale, DATA_READY and I2C reads: https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
- Maxim reference algorithm distributed by SparkFun: https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/blob/master/src/spo2_algorithm.cpp
- Park et al., Photoplethysmogram Analysis and Applications: An Integrative Review, 2022, DOI 10.3389/fphys.2021.808451: https://www.frontiersin.org/journals/physiology/articles/10.3389/fphys.2021.808451/full

Sources checked 2026-10-07. The paper provides background, not validation of this firmware or its thresholds.

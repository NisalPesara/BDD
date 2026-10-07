# Modules and output reference

## Active modules

| File | Job |
|---|---|
| `ppg_esp32.ino` / `src/main.cpp` | Arduino / PlatformIO entry points |
| `firmware.h` | Acquisition, commands, quality, orchestration and reporting |
| `sensor_drivers.h` | Checked MAX30102 paired FIFO and ADXL345 I2C access |
| `config.h` | Board assumptions, pins, rates and tunable starting thresholds |
| `pipeline.h` | Denoising, SQI, warm-up, clipping and peak detection |
| `ppg_noise/ppg_noise.h` | Integrates the small reusable noise modules |
| `ppg_noise/ppg_spike.h` | Hampel transient rejection before IIR filtering |
| `ppg_noise/ppg_dc.h` | Slow DC tracking and AC/DC helpers |
| `ppg_noise/ppg_biquad.h` | Runtime Butterworth SOS design and processing |
| `ppg_noise/ppg_savgol.h` | Aligned PPG, first derivative and second derivative |
| `ppg_noise/ppg_anc.h` | Optional multi-reference NLMS |
| `ppg_noise/ppg_sqi.h` | Quality statistics, autocorrelation and failure bits |
| `m08_peak_detection.h` | Systolic-candidate detector with prominence/refractory checks |
| `wave_detection.h` | Complete-pulse segmentation and feature extraction |
| `m09_auc_calculator.h` | Common-baseline trapezoidal pulse area |
| `measurement_context.h` / `ppg_noise/ppg_force.h` | FSR conductance/stability, optional probe contact and motion assessment |
| `spo2_processor.h` / `spo2_algorithm.h` / `spo2_table.h` | Paired decimation, four-second window and research SpO2 estimate |
| `perfusion_monitor.h` | Explicit baseline, relative feature trends and availability states |
| `nirs_adapter.h` | Optional raw analog AFE readings; disabled by default |
| `serial_logger.h` | Bounded nonwaiting log queue |
| `m04_smoothing.h` | Tested standalone moving-average alternative, not in the default chain |

There are no direct includes of the old `legacy/` headers from active firmware. They are reference copies, not interchangeable replacements for the new APIs. Source breaks from the old API are intentional: `PpgSample` uses microsecond timestamps and exposes aligned SG derivatives; status and CSV formats are new.

## Record types

The first column identifies the record, so mixed records must be split before plotting as a rectangular table. Header records begin `# TYPE,field,...`. Send `h` if the terminal attached after boot. `tools/split_records.py` uses those exact headers and checks row lengths. `nan` means no meaningful floating-point measurement; SpO2 `-1` means unavailable.

- **SAMPLE:** nominal optical sample time, delay-corrected time, original IR/RED, processed PPG/VPG/APG, validity, SQI failure mask/statistics, candidate beat, provisional BPM, FSR voltages/conductance, six acceleration axes, reference-valid flag, motion/ANC energy and contact/stability flags. Raw signals are not replaced by the cleaned ones.
- **PULSE:** one report per accepted complete pulse, one beat late. Includes onset/report timestamps, pulse BPM, amplitude, PI, whole/systolic/diastolic AUC, crest and half-height width, notch type, optional fiducial indices/b:a, and baseline ratios.
- **SPO2:** each computed estimate plus validity, ratio of ratios, red/IR correlation and the algorithm's reference HR. An explicit invalid row is also emitted when a previously valid result is cleared or analysis resets. `algorithm_valid` does not mean medically validated.
- **STATUS:** every second: state, SQI mask, baseline availability, acquisition gaps, known discarded pairs, dropped log records, maximum complete `handlePair` duration in microseconds, and total processed pairs.
- **EVENT:** analysis reset, epoch number and reason. Every reset starts a new `epoch`; never join waveforms across epochs as if contiguous.
- **NIRS:** optional time, raw-valid flag and two millivolt readings; absent in disabled mode. These are not oxygenation values.

`PULSE` fiducial indices are relative to that pulse's onset at the 125 Hz processing rate. `notch_type=0` is absent, 1 is an inflection fallback, 2 is a detected local minimum with a following diastolic peak. An inflection is not mislabeled as a confirmed true notch. The APG extrema are heuristic research features, not guaranteed anatomical landmarks.

## SQI bits

| Decimal bit | Meaning |
|---:|---|
| 1 | Flat / insufficient standard deviation |
| 2 | Insufficient periodicity in configured 40-180 bpm lag range |
| 4 | Skewness below configured minimum |
| 8 | Kurtosis outside configured range |
| 16 | Amplitude-histogram entropy outside configured range |
| 32 | Window PI outside configured range (default gate is permissive) |
| 64 | Excess share of samples near the systolic extreme |
| 128 | Nonfinite input |

Several bits may be set. Warm-up, motion, saturation, contact and acquisition faults are additional gates; a zero SQI mask alone does not imply a valid observation. SQI is recomputed once per second, so a newly emerging artifact may precede its next window result. Immediate acquisition/contact/range checks operate per sample; retrospective bad-window marking prevents affected buffered pulses being emitted, but previously emitted reports cannot be withdrawn. Treat live reports as provisional and inspect stored raw data.

## Status states

| State | Meaning |
|---|---|
| `NO_OPTICAL_SIGNAL` | Raw optical level below the configured floor; cause unresolved |
| `SATURATED` | IR raw range failure |
| `CHECK_PROBE_CONTACT` | Optional designated probe FSR failed its contact range |
| `PRESSURE_CHANGING` | FSR slope or ADC range is unsuitable for stable-feature interpretation |
| `MOTION_OR_REFERENCE_FAULT` | Motion exceeds its threshold or configured references are unavailable |
| `WARMUP` | Filters have not settled |
| `NO_RELIABLE_PULSE` | No accepted complete pulse within three seconds; includes poor-quality/low-pulsatility cases |
| `POOR_SIGNAL_QUALITY` | Recent pulse exists but current quality fails |
| `CAPTURING_BASELINE` | Waiting for ten consecutive accepted pulses |
| `PULSE_DETECTED_NO_BASELINE` | Pulse detected, no relative reference recorded |
| `PULSE_DETECTED` | Pulse detected, baseline available; this is not a sufficient-perfusion claim |
| `REDUCED_PULSE_TREND` | Amplitude and PI below the starting trend ratio for three accepted pulses |
| `MAX30102_INIT_ERROR`, `I2C_ERROR`, `SENSOR_TIMEOUT`, `ACQUISITION_GAP` | Acquisition problem; inspect events/counters |

The pulse detector's provisional BPM and the accepted pulse's onset-interval BPM are distinct values. Candidate beats alone are never sufficient to update the baseline.

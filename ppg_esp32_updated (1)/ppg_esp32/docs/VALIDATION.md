# Validation and hardware work still required

Prepared 2026-10-07. Tests use the **actual active headers**, not a separate model of the signal-processing algorithms.

## Completed here

- Native C++11 compilation with GCC 13.3.0, `-Wall -Wextra -Wpedantic -Werror`.
- 11,516 core assertions: moving average against an explicit reference; exact cubic-polynomial SG derivatives; isolated spike removal; Butterworth cutoff gain; flat and nonfinite SQI; immediate first-window result; FSR hysteresis and release from HIGH; analytic pulse AUC and sub-area additivity.
- Systolic detector with a diastolic wave at 45% of systolic height: approximately 40, 41.38, 60, 70.59, 120, 171.43 and 180 bpm. All produced the expected mean rate within the test tolerance. No stale BPM after a pulse timeout.
- Controlled correlated-artifact NLMS and frozen adaptation below the energy gate.
- 80-second synthetic integrated stream at 125 Hz, with a strong diastolic wave, slow drift, noise and an isolated spike: 82 accepted pulse reports, mean 70.596 bpm for a 70.588 bpm source, 82 notch candidates. This is one constructed waveform, not a general accuracy estimate.
- Entire-pulse rejection across an injected quality dropout; contact-loss invalidation and filter warm-up on return.
- Paired red/IR SpO2 processing: first result at four seconds, seven results over ten seconds, one-second cadence; expected synthetic ratio 0.500 and lookup output 99; flatline, out-of-range input, wrong buffer size, null buffer and tail plateau handling. A synthetic ratio agreement does not validate oxygen saturation accuracy.
- Fixed-baseline capture, persistence threshold and pulse timeout.
- Entire firmware compiled and run against small Arduino/Wire/clock stubs: configuration registers, RED/IR FIFO handling, backlog discard, overflow, I2C error, timeout, optical loss, bounded logging and CSV field counts.
- Compile configurations with zero/one/two accelerometers, ANC enabled, FSR disabled, a designated contact FSR, and analog NIRS enabled.
- AddressSanitizer and UndefinedBehaviorSanitizer on the native code. LeakSanitizer must be disabled in this restricted process environment because it cannot inspect `/proc`; that limitation is not an application leak finding.
- Python recording/splitting tools syntax checked; replay utility compiled, with synthetic full-rate replay and preview-rate rejection checked.

Run `python tests/run_tests.py`. The exact captured console outputs are stored beside this file. The tests are for reproducibility and software regression; they do not establish physiological validity.

Host object sizes: pipeline 7,560 bytes; waveform analyzer 28,832; log queue 16,416; SpO2 processor 984. These are host `sizeof` values, not ESP32 linker RAM usage or measured stack high-water marks. Production DSP uses fixed buffers; framework/I2C/USB allocations are outside that claim.

## Not completed here

**No real ESP32 compilation, flashing, sensor recording, timing benchmark, electrical verification, or clinical validation was performed.** PlatformIO was unavailable and its package download timed out. Mock Arduino compilation checks this firmware's code paths but cannot prove compatibility with the installed ESP32 core, USB stack or board. The intended build configuration is included so the first target build is straightforward.

The Claude research link and NIRS sensor/AFE details were not supplied. Actual tissue-oxygenation acquisition and interpretation remain unfinished until those are known. There is no fabricated NIRS calibration in the code.

## Hardware checks in a useful order

1. Confirm the ESP32-S3 module and breakout specifications. Verify wiring, logic voltage and pull-ups; confirm I2C addresses 0x57, 0x53 and 0x1D. Start without applying clinical compression to a person.
2. Build/upload on the target. With `NUM_ACCEL` matching fitted devices, inspect `CONFIG` and `STATUS`. Measure frame counts against elapsed wall time: the intended FIFO rate is 125 pairs/second. A firmware timestamp is a nominal sample clock, not proof of the sensor oscillator's exact frequency.
3. Inspect raw red/IR for clipping, low counts and ambient changes. Verify polarity. Choose LED current/range from your site and sensor. Every LED, optical geometry or gain change requires a new baseline. Add power decoupling, stable mechanical mounting and an optical shroud before tuning software.
4. Calibrate the FSR voltage-to-conductance model and ADC voltage range; determine which FSR, if any, really measures optical-probe contact. Band pressure cannot substitute for contact verification. Record both voltages during unloaded and changing-pressure conditions.
5. Record raw PPG, six accelerometer axes and both FSR channels at full rate. Confirm gaps, discarded pairs and dropped-log counters stay at zero. Align acceleration axes; characterize rest noise. The ADXL devices run at 200 Hz and supply latest DATA_READY samples near the optical read; this is approximate alignment, not a synchronized instrument.
6. Compare 11-, 15- and 19-sample SG windows against manually reviewed pulse morphology. Tune SQI on independently labeled recordings, including flatline, saturation, contact changes, baseline wander and motion. Do not tune only to increase the number of accepted pulses.
7. Read `STATUS.max_process_us`. It includes reference/FSR reads, DSP, feature work and record formatting within `handlePair`. It excludes separate FIFO polling, USB drain, NIRS reads and framework scheduling; also inspect gaps and measure the whole loop. Leave margin below the 8 ms frame period. No execution-time claim from desktop tests should be substituted for this measurement.
8. Test ANC off/on using the **same recorded data** and known timing, with rest-noise gating. Check amplitude, morphology and false-pulse changes, not only correlation or visual smoothness. The default reports motion-affected features as unavailable even if ANC is enabled.
9. Validate the experimental SpO2 branch against an appropriate reference and measurement-site protocol. Do not treat its lookup-table percentage as a certified measurement.
10. Provide the exact NIRS device/AFE, wavelengths, source-detector geometry, interface, supply/output ranges, LED timing and calibration information. Then implement and validate that adapter separately. An analog signal that stays within ADC range is not necessarily a valid NIRS measurement.

For any human study or interpretation of tourniquet/perfusion conditions, agree on the protocol and reference measurements with your supervising biomedical/clinical team. This package has no actuator outputs and does not decide whether applying, tightening or releasing a tourniquet is appropriate.

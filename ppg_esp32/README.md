# ESP32 PPG purification pipeline

Sources: **Park et al. 2022, *Photoplethysmogram Analysis and Applications: An Integrative Review*,
Front. Physiol. 12:808451** and the pyPPG preprocessing in your `initial.ipynb`.

## Your hardware: ESP32-S3 N16R8 + MAX30102 + MPU6050 + ADXL345
**Arduino IDE settings:** Board *ESP32S3 Dev Module*, Flash *16MB*, **PSRAM = OPI PSRAM** (N16R8 has octal PSRAM; the wrong
setting causes a boot loop), **USB CDC On Boot = Enabled**.

**Wiring (all 3.3 V, one shared I2C bus, SDA = GPIO8, SCL = GPIO9):**
| Device | Address | Note |
|---|---|---|
| MAX30102 | 0x57 | on the finger |
| ADXL345 | 0x53 | SDO/ALT ADDRESS -> GND (0x1D if tied to 3V3; change `ADXL345_ADDR`) |
| MPU6050 | 0x68 | AD0 -> GND |

Never use GPIO 35/36/37 (used by the octal PSRAM).
**Libraries:** SparkFun MAX3010x Pulse and Proximity Sensor Library; Adafruit MPU6050; Adafruit ADXL345; Adafruit Unified Sensor.

### Two-accelerometer relative-motion cancellation (m06 + m06b)
- "Finger" sensor = the one next to the MAX30102; "band" sensor = the one on the wrist/hand band. Set `ACC_FINGER_IS_MPU`.
- `ANC_MODE_REL9` (default): references = finger acceleration, **relative acceleration** (finger - band) and
  **relative velocity** (leaky integral of the relative acceleration, 0.3 Hz corner so it does not drift).
- `ANC_MODE_ABS6`: the plain alternative, the 6 raw axes of both sensors. Compare the two on your own recordings.
- This two-sensor relative scheme is **my implementation of your idea**, not a method from Park 2022 (which cites
  single-accelerometer adaptive cancellation).
- **Mounting matters:** relative motion is only meaningful if both sensors' axes agree. Mount them with parallel axes.
  Check: set `DEBUG_STREAM_RAW 1`, tape/hold both boards rigidly together, shake them, and look at the streamed columns
  (finger xyz vs band xyz). Each axis pair should move together. If an axis is swapped or reversed, fix
  `BAND_AXIS_IDX` / `BAND_AXIS_SGN` in `config.h`. `DEBUG_STREAM_RAW 1` also lets you record raw IR + 6 axes to test
  the canceller offline in Python.
- An accelerometer cannot see every artifact source (contact pressure, tissue deformation, ambient light). Cancellation
  reduces motion artifact; it does not remove it. Keep the SQI on so bad windows are still skipped.

## How to use
1. Copy the folder `ppg_esp32/` into your Arduino sketchbook (folder name must match `ppg_esp32.ino`).
2. Install **esp32 by Espressif** in Boards Manager, apply the settings above, install the libraries.
3. Check `config.h` (pins, `INVERT_SIGNAL`). If fs is not 125 Hz: `cd tools && python gen_coeffs.py <fs>` and re-flash.
4. Put your wave-properties code in `wave_detection.h` (on-device) **or** stream to the PC with `tools/receive_from_esp32.py`.
5. First run: plot the output. If the systolic peaks point DOWN, set `INVERT_SIGNAL 0`. If the IR level is very low/saturated, change the LED current (`0x24`) in the sketch.

## Data flow
```
raw -> [m06 NLMS ANC] -> DC removal -> m01 filter -> [m02 baseline] -> [m05 amp-reg] -> m04 smoothing -> clean PPG
clean PPG -> VPG/APG/JPG (derivatives.h) | upslope (m03) | SQI (m07) | beat flag (m08) -> waveDetectionProcess()
```

## Paper methods -> code
| Paper method (section / reference) | Status | File |
|---|---|---|
| Band-pass Butterworth: Sukor 2011, Liu 2020b, Papini 2018, Pradhan 2019, Canac 2019 | implemented | m01 |
| Chebyshev I (Ferro 2015), Chebyshev II (Allen & Murray 2004 / pyPPG) | implemented | m01 |
| High-pass 0.15 Hz (Joseph 2014); low-pass 10 Hz (Liu 2020a), 15 Hz (Fischer 2017) | implemented | m01 |
| Baseline wandering: HPF (Jang 2014) / estimate-and-subtract (Timimi 2017) | HPF exact; subtraction = causal lower-envelope approximation | m01, m02 |
| Moving-difference filter (Canac 2019) | implemented (separate `upslope` output) | m03 |
| Smoothing windows (pyPPG `sm_wins`) | implemented | m04 |
| Amplitude regularisation (Kim 2019) | causal envelope approximation | m05 |
| Motion artifact: adaptive noise cancellation, NLMS (Widrow 1975, Han 2007, Foo & Wilson 2006, Poh 2010) | implemented; multi-reference (6 axes of 2 accelerometers x 8 taps) | m06 |
| SQI: skewness/kurtosis (Krishnan 2008, Selvaraj 2011, Elgendi 2016), entropy (Selvaraj), perfusion index (Gehring 2002, Cannesson 2008), cross-correlation idea (Karlen 2012) | implemented (rule-based) | m07 |
| Adaptive-threshold peak detection (Shin 2009) | implemented (optional helper) | m08 |
| Derivative PPG: VPG / APG (Takazawa 1998, Fig. 5) | implemented | derivatives.h |
| 64th-order FIR (Selvaraj), DCT filtering (Shin 2010) | not included (offline/block based) | - |
| EMD (Lu 2008), wavelet (Vadrevu 2018, Patterson 2011, Tang 2016), ICA (Kim & Yoo 2006, Lee 2020), Fourier cycle-by-cycle (Reddy 2008) | not on ESP32: non-causal / heavy / need multi-wavelength channels. Do these in Python offline | - |
| Kalman adaptive filter (Seyedtabaii 2008), Wiener filter (Foo & Wilson 2006) | not included | - |
| 32nd-order polynomial detrending (Selvaraj 2011) | not included: numerically unstable in float32; use m02 instead | - |
| Wavelet-scalogram peaks (Scholkmann 2012), ML/DL peak detection & SQI, RNN reconstruction | not included | - |
| Feature extraction of Table 1 (crest time, b/a, c/a, PW50 ...) | **your detection code** | wave_detection.h |

## Important caveats
- **Causal vs zero-phase.** pyPPG filters forward-backward (no delay). Real-time filtering is one-directional, so the
  waveform is delayed and slightly distorted. Timing features (crest time, PPI) keep their relative meaning but absolute
  timestamps shift. For exact pyPPG equivalence, record with `receive_from_esp32.py` (or stream raw) and filter offline.
- **Cheby-II stopband** is 20 dB in the generator (my recollection of pyPPG's setting; not stated in your notebook).
  Ferro's Cheby-I ripple (0.5 dB) is also assumed, since the paper doesn't give it.
- **SQI thresholds are starting values** validated only on a synthetic signal. Tune on your sensor. Skewness sign depends on
  whether your sensor output is inverted.
- **Warm-up**: first `WARMUP_SECONDS` samples are not streamed (filter settling).
- The MAX30102 settings (1000 sps averaged x8 = 125 Hz), the two-MPU6050 wiring and the ESP32-S3 sketch itself have **not** been run on hardware.
- **ANC test (synthetic, two accelerometers; `test/anc_test.cpp`, compile with `-DANC_MODE=0` or `1`).** Correlation with the
  motion-free signal, no cancellation -> with cancellation:

  | Synthetic artifact model | ABS6 | REL9 |
  |---|---|---|
  | proportional to relative velocity | 0.35 -> 0.29 | 0.35 -> 0.60 |
  | proportional to absolute finger acceleration | 0.37 -> 0.73 | 0.37 -> 0.79 |

  REL9 was better in both cases here, and ABS6 cannot model a velocity-type artifact. But these numbers only reflect the
  artifact models I invented; real motion is less linear. Which model is closer to your hand must be checked on real data.
  The filter/SQI/peak headers were compiled and tested natively (`test/`).

## Tests (desktop, optional)
```
cd test
g++ -O2 -std=c++17 -I.. native_test.cpp -o t && ./t > out.csv
```
Result on the synthetic 60 s signal (sign-inverted like a reflective sensor) (72 bpm, drift, noise, 10 s motion burst): 70 beats detected, 24/24 clean windows
accepted, 10/10 motion windows rejected. C++ float32 Cheby-II output matches SciPy within 3.5e-5.

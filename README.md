# 🩺 Smart Tourniquet Perfusion Monitoring System

> **A compact wearable system for monitoring distal blood perfusion during tourniquet and compression-bandage application.**

---

## 📌 Overview

The **Smart Tourniquet Perfusion Monitoring System** is a biomedical engineering project designed to provide a real-time indication of **distal limb perfusion** when a tourniquet, compression bandage, cast, or similar external pressure is applied.

Excessive compression can reduce blood flow beyond the compression site. Our system combines **Photoplethysmography (PPG)**, **pressure sensing**, and **motion sensing** to evaluate the quality of the detected pulse signal and estimate the perfusion condition of the limb.

The system is built around an **ESP32-S3** microcontroller and is intended as a compact, reusable monitoring device for potential clinical and emergency-care applications.

---

## 🎯 Project Objective

The primary objective is to develop a device capable of detecting changes in distal perfusion and providing a simple indication such as:

🟢 **GOOD** — Adequate detected perfusion
🟡 **MODERATE** — Reduced or questionable perfusion
🔴 **POOR** — Significantly reduced/absent detected perfusion

The system also considers **motion artifacts** and **applied pressure** to improve the reliability of the assessment.

---

## 🧠 System Concept

```text
       ┌───────────────────────┐
       │   Tourniquet / Band   │
       └───────────┬───────────┘
                   │
             Compression
                   │
                   ▼
       ┌───────────────────────┐
       │     Distal Limb       │
       │                       │
       │  ┌─────┐  ┌────────┐  │
       │  │ PPG │  │  FSRs  │  │
       │  └──┬──┘  └───┬────┘  │
       │     │         │       │
       │  ┌──┴─────────┴───┐   │
       │  │ Accelerometers  │   │
       │  │   (Motion)      │   │
       │  └────────┬────────┘   │
       └───────────┼────────────┘
                   │
                   ▼
             ┌───────────┐
             │ ESP32-S3  │
             └─────┬─────┘
                   │
          Signal Processing
                   │
                   ▼
        ┌────────────────────┐
        │ Perfusion Analysis │
        └─────────┬──────────┘
                  │
          ┌───────┴───────┐
          ▼               ▼
       Perfusion        Signal
        Status          Quality
```

---

## 🔬 Sensing Approach

### ❤️ Photoplethysmography (PPG)

PPG is the primary sensing technique used to detect pulsatile blood-volume changes in the distal limb.

The system uses a **MAX30102** optical sensor to acquire:

* 🔴 Red-light PPG
* 🌑 Infrared PPG
* Pulse waveform characteristics
* Perfusion-related features

Changes in the PPG waveform can provide information about the quality and strength of distal blood flow.

---

### 📏 Pressure Measurement

Two **Force Sensitive Resistors (FSRs)** are used to monitor applied pressure.

The pressure measurements can provide additional context when interpreting changes in the PPG signal.

```text
        Applied Pressure
               │
               ▼
        ┌─────────────┐
        │    FSR 1    │
        ├─────────────┤
        │    FSR 2    │
        └──────┬──────┘
               │
               ▼
           ESP32-S3
```

Using two sensors also allows us to investigate pressure distribution rather than relying on a single measurement point.

---

### 📐 Motion Artifact Detection

Movement can introduce significant artifacts into PPG measurements.

To address this, the system incorporates **two ADXL345 accelerometers** to detect limb/device motion.

The motion information can be used during signal processing to identify corrupted PPG segments and improve signal-quality assessment.

---

## ⚙️ Signal Processing Pipeline

The proposed processing pipeline is:

```text
Raw PPG
   │
   ▼
Motion Detection
   │
   ▼
Artifact Reduction
   │
   ▼
Baseline Correction
   │
   ▼
Filtering
   │
   ▼
Signal Quality Assessment
   │
   ▼
Feature Extraction
   │
   ▼
Perfusion Assessment
   │
   ▼
GOOD / MODERATE / POOR
```

Potential features include:

* Pulse amplitude
* Pulse width
* Pulse area
* Pulse-to-pulse interval
* Perfusion Index
* Waveform morphology
* First and second derivatives
* Pulse Transit Time (PTT)
* Signal Quality Index (SQI)

Segments with poor signal quality can be rejected or reacquired instead of being used directly for perfusion classification.

---

## 🔧 Hardware

| Component              | Purpose                     |
| ---------------------- | --------------------------- |
| **ESP32-S3**           | Main processing and control |
| **MAX30102**           | PPG acquisition             |
| **2 × ADXL345**        | Motion detection            |
| **2 × FSR**            | Pressure measurement        |
| OLED / Display         | User feedback               |
| Battery / Power Supply | Portable operation          |

---

## 💻 Software

### Development

* Arduino IDE
* C/C++
* Python
* Jupyter Notebook

### Signal Processing & Analysis

The project may use Python-based tools for:

* PPG visualization
* Filtering
* Feature extraction
* Signal-quality analysis
* Algorithm development
* Performance evaluation

---

## 🔌 System Architecture

```text
                 ┌──────────────────────┐
                 │      MAX30102        │
                 │       PPG Sensor     │
                 └──────────┬───────────┘
                            │ I²C
                            │
┌──────────────┐            │            ┌──────────────┐
│   ADXL345    │────────────┐            │    ADXL345   │
│ Accelerometer│            │            │ Accelerometer│
└──────────────┘            │            └──────┬───────┘
                            │                   │
                            ▼                   ▼
                     ┌─────────────────────────────┐
                     │          ESP32-S3            │
                     │                              │
                     │  • Data Acquisition          │
                     │  • Signal Processing         │
                     │  • Motion Detection          │
                     │  • Perfusion Assessment      │
                     └──────────────┬──────────────┘
                                    │
                         ┌──────────┴──────────┐
                         │                     │
                         ▼                     ▼
                   ┌───────────┐         ┌───────────┐
                   │   FSR 1   │         │   FSR 2   │
                   └───────────┘         └───────────┘
```

---

## 🚀 Current Development Status

* [x] ESP32-S3 platform selected
* [x] PPG sensing approach selected
* [x] MAX30102 integration
* [x] Motion sensing approach selected
* [x] ADXL345 selected for motion detection
* [x] Dual-FSR pressure sensing approach
* [ ] Multi-sensor hardware integration
* [ ] PPG preprocessing pipeline
* [ ] Motion-artifact reduction
* [ ] Signal Quality Index implementation
* [ ] Perfusion feature extraction
* [ ] Perfusion classification
* [ ] Integrated prototype testing
* [ ] Validation with controlled pressure/perfusion conditions

---

## 📁 Repository Structure

```text
Smart-Tourniquet/
│
├── 📁 firmware/
│   ├── esp32/
│   └── sensors/
│
├── 📁 signal_processing/
│   ├── filtering/
│   ├── feature_extraction/
│   └── analysis/
│
├── 📁 hardware/
│   ├── schematics/
│   ├── pcb/
│   └── enclosure/
│
├── 📁 data/
│   └── ppg/
│
├── 📁 documentation/
│   ├── reports/
│   └── figures/
│
└── README.md
```

---
# Techniques and sources used in each stage

Main source: **Park J, Seok HS, Kim S-S, Shin H (2022). Photoplethysmogram Analysis and Applications: An Integrative
Review. Front. Physiol. 12:808451. doi:10.3389/fphys.2021.808451.** All papers below are cited in that review, except pyPPG.
"Fidelity" says how closely the code follows the original. The paper is a review and contains no code.

| Stage (file) | Technique / algorithm | Source | Fidelity |
|---|---|---|---|
| Main band-pass (m01) | Chebyshev Type II, 4th order, 0.5-12 Hz | Allen & Murray 2004; settings from pyPPG | Type/order match; 20 dB stopband is an assumption |
| Band-pass alternatives (m01) | Butterworth 4th, 0.5-10 Hz | Canac 2019 | Same |
| | Butterworth 4th, 0.5-50 Hz | Pradhan 2019 | Same |
| | Chebyshev I 4th, 0.5-16 Hz | Ferro 2015 | Same; 0.5 dB ripple assumed |
| | Butterworth 1st, 0.5-5 Hz | Sukor 2011 | Same |
| | Butterworth 2nd, 0.2-10 Hz | Liu 2020b | Same |
| | Butterworth 3rd, 0.4-10 Hz | Papini 2018 | Same |
| High-pass 0.15 Hz (m01) | Butterworth high-pass | Joseph 2014 | Cutoff same; 2nd order is my choice |
| Low-pass (m01) | Butterworth 2nd 10 Hz; 4th 15 Hz | Liu 2020a; Fischer 2017 | Same |
| Filter implementation (m01) | Cascaded biquads (SOS, transposed direct form II) | Standard DSP | Not from the paper |
| Baseline removal, direct (m01) | High-pass filtering | Jang 2014 | Same idea |
| Baseline removal, indirect (m02) | Estimate baseline and subtract | Idea from Timimi 2017 | **My approximation** (running lower envelope) |
| Upslope enhancement (m03) | Moving-difference filter | Canac 2019 | Idea only; formula/window are mine |
| Smoothing (m04) | Moving average (PPG 50 ms, derivatives 10 ms) | pyPPG `sm_wins` | Same as your notebook |
| Amplitude regularisation (m05) | Envelope-based amplitude compensation | Kim, Lee & Shin 2019 | **My causal approximation** |
| Motion cancellation (m06) | NLMS adaptive noise cancellation | Widrow 1975; Han 2007; Foo & Wilson 2006; Poh 2010 | Algorithm same; 8 taps / multi-input form are mine |
| Two-accelerometer references (m06b) | Relative acceleration + leaky-integrated relative velocity | **Not from any paper** | Your idea, implemented by me |
| Skewness, kurtosis (m07) | Statistical-moment quality measures | Krishnan 2008; Selvaraj 2011; Elgendi 2016 | Measures same; thresholds are mine |
| Entropy (m07) | Shannon entropy of amplitude histogram | Selvaraj 2011 | Measure same; 16 bins/thresholds mine |
| Perfusion index (m07) | AC/DC ratio | Hartmut Gehring 2002; Cannesson 2008a | Streamed only, not in the accept/reject rule |
| Periodicity (m07) | Normalised autocorrelation over the heart-rate range | Inspired by Karlen 2012 | **Not their algorithm** (they cross-correlate successive beats) |
| Quality decision rule (m07) | Combined thresholds | Rule-based style of Sukor 2011, Fischer 2017 | **My combination and thresholds** |
| Peak detection (m08) | Adaptive threshold + refractory period | Shin 2009 | Idea same; threshold updates are my design |
| Derivatives | Velocity (1st) and acceleration (2nd) plethysmogram | Takazawa 1998; crest time / delta-T from Alty 2007 | Signals only; feature extraction is your code |
| Third derivative (JPG) | Third derivative | pyPPG | Not covered by the paper |

**Not from any paper:** slow DC tracking, the warm-up period, the 125 Hz sampling scheme, the MAX30102 settings,
the relative-velocity leak corner (0.3 Hz) and scale factor.

## References
- Allen J, Murray A (2004). Effects of filtering on multisite photoplethysmography pulse waveform characteristics. Computers in Cardiology 2004, 485-488.
- Alty SR, Angarita-Jaimes N, Millasseau SC, Chowienczyk PJ (2007). Predicting arterial stiffness from the digital volume pulse waveform. IEEE Trans. Biomed. Eng. 54, 2268-2275.
- Canac N, et al. (2019). Algorithm for reliable detection of pulse onsets in cerebral blood flow velocity signals. Front. Neurol. 10:1072.
- Cannesson M, et al. (2008). Does the pleth variability index indicate the respiratory-induced variation in the plethysmogram and arterial pressure waveforms? Anesth. Analg. 106, 1189-1194.
- Elgendi M (2016). Optimal signal quality index for photoplethysmogram signals. Bioengineering 3:21.
- Ferro BR, Aguilera AR, De La Vara Prieto RF (2015). Automated detection of the onset and systolic peak in the pulse wave using Hilbert transform. Biomed. Signal Process. Control 20, 78-84.
- Fischer C, Glos M, Penzel T, Fietze I (2017). Extended algorithm for real-time pulse waveform segmentation and artifact detection in photoplethysmograms. Somnologie 21, 110-120.
- Foo JYA, Wilson SJ (2006). A computational system to optimise noise rejection in photoplethysmography signals during motion or poor perfusion states. Med. Biol. Eng. Comput. 44, 140-145.
- Han H, Kim M-J, Kim J (2007). Development of real-time motion artifact reduction algorithm for a wearable photoplethysmography. Proc. 29th Annual Int. Conf. IEEE EMBS, 1538-1541.
- Hartmut Gehring M, et al. (2002). The effects of motion artifact and low perfusion on the performance of a new generation of pulse oximeters in volunteers undergoing hypoxemia. Respiratory Care 47, 48-60.
- Jang D-G, Farooq U, Park S-H, Hahn M (2014). A robust method for pulse peak determination in a digital volume pulse waveform with a wandering baseline. IEEE Trans. Biomed. Circuits Syst. 8, 729-737.
- Joseph G, Joseph A, Titus G, Thomas RM, Jose D (2014). Photoplethysmogram (PPG) signal analysis and wavelet de-noising. AICERA/iCMMD, 1-5.
- Karlen W, Kobayashi K, Ansermino JM, Dumont G (2012). Photoplethysmogram signal quality estimation using repeated Gaussian filters and cross-correlation. Physiol. Meas. 33:1617.
- Kim J, Lee J-W, Shin H (2019). Pre-processing of photoplethysmographic waveform for amplitude regularization. J. Electr. Eng. Technol. 14, 1741-1748.
- Krishnan R, Natarajan B, Warren S (2008). Motion artifact reduction in photopleythysmography using magnitude-based frequency domain independent component analysis. Proc. 17th ICCCN, 1-5. (Cited in the review for skewness/kurtosis-based quality.)
- Liu S-H, Li R-X, Wang J-J, Chen W, Su C-H (2020a). Classification of photoplethysmographic signal quality with deep convolution neural networks for accurate measurement of cardiac stroke volume. Appl. Sci. 10:4612.
- Liu S-H, Wang J-J, Chen W, Pan K-L, Su C-H (2020b). Classification of photoplethysmographic signal quality with fuzzy neural network for improvement of stroke volume measurement. Appl. Sci. 10:1476.
- Papini GB, et al. (2018). Sinus or not: a new beat detection algorithm based on a pulse morphology quality index to extract normal sinus rhythm beats from wrist-worn photoplethysmography recordings. Physiol. Meas. 39:115007.
- Park J, Seok HS, Kim S-S, Shin H (2022). Photoplethysmogram Analysis and Applications: An Integrative Review. Front. Physiol. 12:808451.
- Poh M-Z, Swenson NC, Picard RW (2010). Motion-tolerant magnetic earring sensor and wireless earpiece for wearable photoplethysmography. IEEE Trans. Inf. Technol. Biomed. 14, 786-794.
- Pradhan N, Rajan S, Adler A (2019). Evaluation of the signal quality of wrist-based photoplethysmography. Physiol. Meas. 40:065008.
- Selvaraj N, Mendelson Y, Shelley KH, Silverman DG, Chon KH (2011). Statistical approach for the detection of motion/noise artifacts in photoplethysmogram. Proc. Annual Int. Conf. IEEE EMBS, 4972-4975.
- Shin HS, Lee C, Lee M (2009). Adaptive threshold method for the peak detection of photoplethysmographic waveform. Comput. Biol. Med. 39, 1145-1152.
- Sukor JA, Redmond S, Lovell N (2011). Signal quality measures for pulse oximetry through waveform morphology analysis. Physiol. Meas. 32:369.
- Takazawa K, et al. (1998). Assessment of vasoactive agents and vascular aging by the second derivative of photoplethysmogram waveform. Hypertension 32, 365-370.
- Timimi AA, Ali MM, Chellappan K (2017). A novel AMARS technique for baseline wander removal applied to photoplethysmogram. IEEE Trans. Biomed. Circuits Syst. 11, 627-639.
- Widrow B, et al. (1975). Adaptive noise cancelling: principles and applications. Proc. IEEE 63, 1692-1716.
- Goda MA, Charlton PH, Behar JA (2024). pyPPG: a Python toolbox for comprehensive photoplethysmography signal analysis. Physiol. Meas. *(cited from memory, not checked against the source; verify before publishing)*


## 🧪 Testing & Validation

Testing will focus on the relationship between:

**Applied compression → PPG waveform → Signal quality → Perfusion indication**

Testing will evaluate:

* PPG signal stability
* Motion-artifact rejection
* Pressure-sensor response
* Sensor placement
* Changes in PPG amplitude and morphology
* Repeatability
* Classification performance

> ⚠️ **This project is an academic engineering prototype and is not intended for clinical diagnosis or direct medical use.**

---

## 👥 Project Team

**Biomedical Engineering Project**

University of Moratuwa
Department of Biomedical Engineering

### Team Members

* Nisal Pesara
* *Add team members here*

---

## 📜 License

This project is developed for **academic and educational purposes**.

Please contact the project authors before using the hardware design, firmware, or processing algorithms for clinical or commercial applications.

---

## ⭐ Project Vision

The long-term goal is to develop a **compact, non-invasive, data-driven monitoring system** that can provide useful feedback about distal perfusion during compression-based medical interventions.

**Sense → Process → Assess → Inform**

---

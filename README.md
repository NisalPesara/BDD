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

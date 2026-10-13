/*
  ESP32 real-time PPG pipeline + limb blood-flow assessment + SpO2
  Sensors NOW : MAX30102 (IR + RED) + ADXL345 finger (0x53) + MPU-6050 band (0x68)   [classic ESP32, I2C SDA 22 / SCL 23]
                Motion artifact = probe moving RELATIVE to the hand (finger - band): gated + cancelled (NLMS)
  Prepared, commented out until the hardware exists: 2x FSR, NIRS (see config.h)
  Based on: Park et al. 2022, Front. Physiol. 12:808451 + pyPPG preprocessing,
            minor-noise libraries (ppg_*.h), updated_libraries-manuth, 4.37 fixes.

  WHAT HAPPENS TO EVERY SAMPLE (125 Hz, see handleSample() below)
    1. read IR + RED from the MAX30102 FIFO              sensor_drivers.h
    2. read the accelerometer (FSRs later)               m12_measurement_context.h  -> motion gate
    3. PPG pipeline                                      pipeline.h
         spike removal (Hampel) -> DC removal -> high-pass 0.15 Hz -> [motion cancellation]
         -> low-pass 10 Hz -> Savitzky-Golay smoothing + VPG/APG/JPG -> SQI -> systolic peak (m08)
    4. per complete pulse: fiducials + features          wave_detection.h (+ m09 AUC)
    5. blood flow vs baseline: GOOD / REDUCED / POOR / NO PULSE     m10_perfusion_monitor.h
    6. SpO2 from raw RED/IR (decimated to 25 Hz)         m11_spo2_processor.h + spo2_algorithm.h
    7. [NIRS - not used now]                             m13_nirs.h
  Flow is judged from PEAK DETECTION + DERIVATIVE features: amplitude, perfusion index, pulse
  area and the maximum upstroke slope (VPG), each compared with the baseline of the same probe.

  ARDUINO IDE: Board "ESP32S3 Dev Module" (USB CDC On Boot: Enabled, Flash 16MB, PSRAM: OPI PSRAM)
               or "ESP32 Dev Module". No extra libraries are needed (only Wire, built in).
  WIRING NOW (Board: "ESP32 Dev Module"):
    MAX30102 : SDA -> GPIO 22, SCL -> GPIO 23, VIN -> 3.3 V, GND            (0x57)
    ADXL345  : SDA -> GPIO 22, SCL -> GPIO 23, CS -> 3.3 V, SDO -> GND      (0x53)
  LATER (pins reserved in config.h, commented out): FSR 34/35, NIRS PD 32/33, NIRS LEDs 25/26,
    band MPU-6050 0x68 (AD0 -> GND) - or a 2nd ADXL345 0x1D (config.h BAND_SENSOR)

  SERIAL (460800 baud) - commands:  b = capture reference   r = reset   p = plotter/report/teleplot   h = help
                                    t / f = capture TIGHT / FREE hand level for the LEDs   c = clear that calibration
  10 LEDs: relative optical pulse-strength index x vs the 'b' reference (m14) - NOT a blood-flow percentage
  OUTPUT ("Name: value unit"):
    per accepted pulse : PULSE REPORT with all features, flow ratios, flow status, SpO2
    every second       : STATUS block (flow status, signal quality, HR, SpO2, motion)
    plotter mode       : "PPG:..,VPG:..,APG:..,Beat:.." per sample for the Arduino Serial Plotter
    teleplot mode      : exactly the channels listed at teleplotSample() / teleplotIndex()

  Research prototype: not a validated medical device; never use it alone to decide tourniquet use.
*/
#include <Wire.h>
#include "config.h"
#include "sensor_drivers.h"
#include "pipeline.h"
#include "wave_detection.h"
#include "m10_perfusion_monitor.h"
#include "m11_spo2_processor.h"
#include "m12_measurement_context.h"
#include "led_agc.h"
#include "m14_flow_level.h"
// #include "m13_nirs.h"         // NIRS - not used now
#include "serial_logger.h"
#if USE_WIFI
#include "wifi_telemetry.h"
#endif
#if defined(ARDUINO_ARCH_ESP32)
#include <Preferences.h>          // tight / free LED calibration kept in flash
#endif

// ================= SENSORS =================
ppgio::Max30102 ppgSensor;
ppgio::Adxl345  accFinger;
#if NUM_ACCEL >= 2
#if BAND_SENSOR == BAND_SENSOR_MPU6050
ppgio::Mpu6050  accBand;          // band / hand accelerometer - only when NUM_ACCEL 2
#else
ppgio::Adxl345  accBand;
#endif
#endif

// ================= PROCESSING =================
PpgPipeline        pipeline;      // filters, derivatives, SQI, peaks
WaveAnalyzer       wave;          // per-pulse features
PerfusionMonitor   perfusion;     // flow vs baseline
SpO2Processor      spo2;          // SpO2 from raw RED/IR
MeasurementContext context;       // motion gate (+ FSR pressure gate when USE_FSR 1)
LedAgc             agc;           // automatic LED current (USE_LED_AGC)
SerialLogger       out;           // non-blocking serial output
FlowLevel          flowLevel;     // relative pulse-strength index x -> 10 LEDs + Teleplot
uint16_t           ledShown = 0xFFFF;   // last pattern written to the 10 LEDs
bool               baselineWasReady = false;

void setFlowLeds(uint16_t mask) {
#if USE_FLOW_LEDS
  if (mask == ledShown) return;
  ledShown = mask;
  for (int i = 0; i < 10; i++) {
    const bool on = (mask >> i) & 1u;
    digitalWrite(FLOW_LED_PINS[i], (on == (FLOW_LED_ACTIVE_HIGH != 0)) ? HIGH : LOW);
  }
#else
  (void)mask;
#endif
}
// NirsMonitor     nirs;          // NIRS - not used now
#if USE_WIFI
WifiTelemetry      telemetry;     // 1 s batches to the website (network runs on core 0)
#endif

PpgSample latest;
float lastAcc[6] = {0, 0, 0, 0, 0, 0};
float lastFsrMv[2] = {0, 0};
uint32_t autoArmFromMs = 0;       // AUTO_BASELINE: the capture is armed AUTO_ARM_SETTLE_MS after this
uint32_t sampleMs = 0;            // time stamp of the current sample (exactly 8 ms per sample)
uint32_t samplesTotal = 0, fifoOverflows = 0, i2cErrors = 0, lastSampleUs = 0, lastStatusMs = 0;
uint8_t  i2cErrorRun = 0;          // consecutive I2C failures
uint32_t pulsesTotal = 0;          // accepted pulses since boot
uint32_t lastPulseMs = 0;          // sampleMs of the last accepted pulse (0 = none yet)
uint32_t lastSampleWallMs = 0;     // millis() when the last sample was processed (acquisition-stopped check)
unsigned long spikesEarlier = 0;   // spikes removed before the last filter reset
bool sensorReady = false, hadTissue = false;
int  outputMode = OUTPUT_MODE;
const char* fault = nullptr;

// Heart rate to show: still -> median of beat intervals (m08); small motion -> the last pulse that passed
// the template check, if it is recent; otherwise unknown (NAN).
float displayHr() {
  if (isfinite(latest.bpm)) return latest.bpm;
  if (latest.smallMotion && lastPulseMs && sampleMs - lastPulseMs < 3000 && isfinite(wave.last.bpm)) return wave.last.bpm;
  return NAN;
}

// ================= TELEPLOT =================
// The Teleplot stream carries EXACTLY these 16 channels (nothing else starts with '>'):
//   per sample  : PPG, PPG_noANC, VPG, APG, SQI, RelMotion, HandMotion, Moving
//   every 100 ms: BloodFlowLevel = the LED level (0 UNKNOWN, 1..10) = "Relative pulse level - not blood-flow
//                 percentage". Sent often so it plots as a step graph (tools/flow_viewer.py draws it with the
//                 10 level bands and a separate UNKNOWN row)
//   every 1 s   : relativeAmplitude, relativePI, PulseIndex (= x), pulseDetectionRate, amplitudeCV,
//                 templateCorrelation, IR_DC (raw-IR DC level, counts = the PI denominator)
// Unavailable: -1 (non-negative values), -2 (templateCorrelation), 0 (BloodFlowLevel). Every value comes from the
// same FlowLevel state that drives the LEDs (no separate calculation for the graph); UNKNOWN clears them all.
// TELEPLOT_SUPPORT 1 adds (once per second, exploratory): relativeDC, relativeAreaPI, relativeSlopePI, hrRatio,
// areaVsPI (median area-based / PI-based strength) and supportAgreement (-1 n/a, 0 agree, 1 disagree).
void teleplotSample() {
  out.printf(">PPG:%.2f\n>PPG_noANC:%.2f\n>VPG:%.2f\n>APG:%.2f\n>SQI:%d\n>RelMotion:%.3f\n>HandMotion:%.3f\n>Moving:%d\n",
             latest.ppg, latest.preAnc, latest.vpg, latest.apg, latest.sqi_good ? 1 : 0,
             context.relEnergy, context.dynamicEnergy,
             !latest.motionQuiet ? 2 : (latest.smallMotion ? 1 : 0));   // 0 quiet, 1 small motion kept, 2 rejected
}
void teleplotIndex(uint32_t nowMs) {
  static uint32_t lastLevelMs = 0, lastSlowMs = 0;
  if (nowMs - lastLevelMs >= FL_TP_LEVEL_MS) { lastLevelMs = nowMs; out.printf(">BloodFlowLevel:%d\n", flowLevel.level); }
  if (nowMs - lastSlowMs < FL_TP_SLOW_MS) return;
  lastSlowMs = nowMs;
  const bool dcOk = sensorReady && !fault && latest.optical && isfinite(latest.dc) && latest.dc > 0.0f;
  out.printf(">relativeAmplitude:%.3f\n>relativePI:%.3f\n>PulseIndex:%.3f\n>pulseDetectionRate:%.3f\n"
             ">amplitudeCV:%.3f\n>templateCorrelation:%.3f\n>IR_DC:%.0f\n",
             FlowLevel::outNonNeg(flowLevel.relAmp), FlowLevel::outNonNeg(flowLevel.relPI),
             FlowLevel::outNonNeg(flowLevel.x), FlowLevel::outNonNeg(flowLevel.detRate),
             FlowLevel::outNonNeg(flowLevel.ampCV), FlowLevel::outCorr(flowLevel.tplCorr),
             dcOk ? latest.dc : -1.0f);
#if TELEPLOT_SUPPORT
  out.printf(">relativeDC:%.3f\n>relativeAreaPI:%.3f\n>relativeSlopePI:%.3f\n>hrRatio:%.3f\n>areaVsPI:%.3f\n>supportAgreement:%d\n",
             FlowLevel::outNonNeg(flowLevel.relDC), FlowLevel::outNonNeg(flowLevel.relAreaPI),
             FlowLevel::outNonNeg(flowLevel.relSlopePI), FlowLevel::outNonNeg(flowLevel.hrRatio),
             FlowLevel::outNonNeg(flowLevel.areaVsPI), flowLevel.supportCode());
#endif
}

// ================= OUTPUT HELPERS =================
// One feature per line:  "Name: value unit"   (n/a when the feature could not be measured)
void printFeature(const char* name, float value, const char* unit) {
  char b[128];
  if (isfinite(value) && fabsf(value) >= 10000.0f) snprintf(b, sizeof b, "%s: %.0f %s\n", name, (double)value, unit);
  else if (isfinite(value))                        snprintf(b, sizeof b, "%s: %.5g %s\n", name, (double)value, unit);
  else                 snprintf(b, sizeof b, "%s: n/a %s\n", name, unit);
  out.enqueue(b);
}
void printText(const char* name, const char* value) {
  char b[160]; snprintf(b, sizeof b, "%s: %s\n", name, value); out.enqueue(b);
}

void printHelp() {
  out.enqueue("\n# PPG + limb blood-flow monitor (" BOARD_NAME ")\n");
  out.printf("# fs=%d Hz  SDA=%d SCL=%d  accel=%d  ANC=%d  FSR=%d  NIRS mode=%d\n",
             PPG_FS, I2C_SDA, I2C_SCL, NUM_ACCEL, USE_ANC, USE_FSR, NIRS_MODE);
  // out.printf("# NIRS pins: LED1=%d LED2=%d PD1=%d PD2=%d\n", NIRS_LED1_PIN, NIRS_LED2_PIN, NIRS_PD1_PIN, NIRS_PD2_PIN);  // NIRS
#if USE_FSR
  out.printf("# FSR pins: %d %d\n", FSR1_PIN, FSR2_PIN);
#endif
  out.enqueue("# Commands: b = capture reference  r = reset  p = cycle report/plotter/teleplot  w = raw recording  h = help\n");
#if FL_TWO_POINT_CAL
  out.enqueue("#           t = capture TIGHT hand level  f = capture FREE hand level  c = clear t/f\n");
#endif
  out.enqueue("# Send b in a known UNRESTRICTED condition and keep the limb still for 10 pulses.\n");
  out.enqueue("# LEDs: relative optical pulse index x = smoothed PI / reference PI (more LEDs = stronger pulse).\n");
  out.enqueue("# It is NOT a blood-flow percentage and does not show that a tourniquet or bandage is safe.\n");
  out.enqueue("# All LEDs off = UNKNOWN (no reference, probe off, motion, clipping, no fresh pulse) - never zero flow.\n\n");
}

void printIndexDiagnostics();
void printPulseReport(const PulseFeatures& f) {
  out.printf("\n==================== PULSE #%lu ====================\n", (unsigned long)pulsesTotal);
  printFeature("Heart rate", f.bpm, "bpm");
  printFeature("Pulse interval", f.ppi, "s");
  printFeature("Systolic amplitude", f.amp, "a.u.");
  printFeature("Perfusion index", f.pi, "%");
  printFeature("Pulse width / PW50", f.pw50, "s");
  printFeature("Pulse area", f.area, "a.u.*s");
  printFeature("Systolic area", f.sysArea, "a.u.*s");
  printFeature("Diastolic area", f.diaArea, "a.u.*s");
  printFeature("Inflection point area ratio", f.ipa, "ratio");
  printFeature("PPG augmentation index", f.aix, "ratio");
  printFeature("Crest time", f.crest, "s");
  printFeature("Delta T", f.deltaT, "s");
  printFeature("Large artery stiffness index", f.lasi, "m/s");
  printFeature("Max upstroke slope (VPG)", f.upSlope, "a.u./s");
  printFeature("b/a", f.bOverA, "ratio");
  printFeature("c/a", f.cOverA, "ratio");
  printFeature("d/a", f.dOverA, "ratio");
  printFeature("e/a", f.eOverA, "ratio");
  printFeature("(b-e)/a", f.bMinusEOverA, "ratio");
  printFeature("(b-c-d)/a", f.bMinusCMinusDOverA, "ratio");
  printFeature("(b-c-d-e)/a aging index", f.agingIdx, "ratio");
  printText("Dicrotic notch", f.notchType == 2 ? "present" : (f.notchType == 1 ? "inflection only" : "absent"));
#if REPORT_FIDUCIALS
  out.printf("Fiducials (samples from onset): peak=%d notch=%d dia=%d offset=%d u=%d v=%d w=%d a=%d b=%d c=%d d=%d e=%d f=%d\n",
             f.peak, f.notch, f.dia, f.offset, f.u, f.v, f.w, f.a, f.b, f.c, f.d, f.e, f.f);
#endif
  out.enqueue("-- Blood flow vs baseline --\n");
  if (perfusion.baselineReady) {
    printFeature("Amplitude ratio", perfusion.ampRatio, "x baseline");
    printFeature("Perfusion index ratio", perfusion.piRatio, "x baseline");
    printFeature("Pulse area ratio", perfusion.areaRatio, "x baseline");
    printFeature("Upstroke slope ratio", perfusion.slopeRatio, "x baseline");
    printFeature("Flow score", perfusion.score, "x baseline");
  } else if (perfusion.capturing) {
    out.printf("Baseline: capturing %d/%d pulses\n", perfusion.baselineCount(), BASELINE_PULSES);
  } else {
    out.enqueue("Baseline: none (send 'b' while flow is normal)\n");
  }
  printText("Flow status", perfusion.text());
  out.enqueue("  (flow score / status = legacy heuristic: overlapping ratios, uncalibrated cutoffs - not the LEDs)\n");
  printIndexDiagnostics();
  if (spo2.getSpO2() > 0) printFeature("SpO2", (float)spo2.getSpO2(), "%");
  else                    printText("SpO2", "n/a");
  out.enqueue("===================================================\n");
}

// x and what it is made of, then the supporting diagnostics (never part of x).
void printIndexDiagnostics() {
  out.enqueue("-- Relative optical pulse index (LEDs) --\n");
  printFeature("  PulseIndex x", flowLevel.x, "(smoothed clamp(relativePI, 0, 1))");
  printFeature("  relativePI", flowLevel.relPI, "x reference (primary input of x)");
  out.printf("  Availability: %s\n", flowLevel.level ? "measuring" : flowUnknownText(flowLevel.why));
  out.enqueue("-- Supporting diagnostics (not part of x) --\n");
  printFeature("  relativeAmplitude", flowLevel.relAmp, "x reference (= relativePI x relativeDC)");
  printFeature("  relativeDC", flowLevel.relDC, "x reference (optical conditions)");
  printFeature("  relativeAreaPI", flowLevel.relAreaPI, "x reference (area-based strength)");
  printFeature("  areaVsPI", flowLevel.areaVsPI, "median relativeAreaPI / relativePI");
  printText("  Consistency", flowLevel.support == FlowLevel::SUP_DISAGREE
                               ? "DISAGREE - pulse shape changed vs its peak (x not changed; check the waveform)"
                               : flowLevel.support == FlowLevel::SUP_AGREE ? "area and PI agree" : "n/a");
  printFeature("  relativeSlopePI", flowLevel.relSlopePI, "x reference (depends on amplitude + rise time)");
  printFeature("  crest time ratio", flowLevel.crestRatio, "x reference (HR-dependent)");
  printFeature("  width fraction ratio", flowLevel.widthRatio, "x reference (PW50 / beat)");
  printFeature("  hrRatio", flowLevel.hrRatio, "x reference HR");
  printFeature("  pulseDetectionRate", flowLevel.detRate, "of expected at the reference HR (HR changes affect it)");
  printFeature("  amplitudeCV", flowLevel.ampCV, "SD/mean (a real downward trend also raises it)");
  printFeature("  reference amplitudeCV", flowLevel.reference().ampCv, "SD/mean");
  printFeature("  templateCorrelation", flowLevel.tplCorr, "-1..1 vs the reference waveform (shape only)");
}

void printStatus(uint32_t nowMs) {
  out.printf("\n---- STATUS  t=%.1f s ----\n", nowMs / 1000.0);
  if (fault) printText("Sensor", fault);
  printText("Flow status", perfusion.text());
  out.printf("Relative pulse level (not blood-flow %%): %d/10 %s%s%s\n", flowLevel.level, flowLevelText(flowLevel.level),
             flowLevel.level ? "" : " - ", flowLevel.level ? "" : flowUnknownText(flowLevel.why));
  if (flowLevel.hasReference()) {
    const FlowReference& rf = flowLevel.reference();
    out.printf("Reference: amplitude %.4g a.u., PI %.4g %%, DC %.0f, HR %.0f bpm, amplitude CV %.3f, LEDs RED %.1f mA IR %.1f mA (locked)\n",
               (double)rf.amp, (double)rf.pi, (double)rf.dc, (double)rf.hr, (double)rf.ampCv,
               (double)LedAgc::milliamps(rf.ledRed), (double)LedAgc::milliamps(rf.ledIr));
  }
  else if (perfusion.capturing)
    out.printf("Reference: capturing %d/%d still pulses\n", perfusion.baselineCount(), BASELINE_PULSES);
  else out.enqueue("Reference: none (send 'b' in the unrestricted condition)\n");
  printIndexDiagnostics();
  out.printf("Signal quality: %s (fail bits %u: 1 flat 2 periodicity 4 skew 8 kurtosis 16 entropy 32 PI 64 clip 128 invalid)\n",
             latest.sqi_good ? "good" : "poor", (unsigned)latest.sqi.fail);
  printFeature("Periodicity", latest.sqi.periodicity, "0..1");
  printFeature("Heart rate", displayHr(), "bpm");
  printText("Pulse template", wave.templateReady() ? "ready (motion pulses are shape-checked)" : "learning from still pulses");
  out.printf("Motion pulses rejected by shape check: %lu\n", (unsigned long)wave.rejectedMotion);
  if (spo2.getSpO2() > 0) printFeature("SpO2", (float)spo2.getSpO2(), "%"); else printText("SpO2", "n/a");
  printFeature("SpO2 ratio R", spo2.ratio, "ratio");
  printFeature("Raw IR", latest.raw, "counts");
#if USE_FSR
  printFeature("Band force 1", context.forceUs[0], "uS");
  printFeature("Band force 2", context.forceUs[1], "uS");
#endif
#if NUM_ACCEL > 0
  printFeature("Motion energy (whole hand)", context.dynamicEnergy, "(m/s^2)^2");
  if (context.relative) printFeature("Relative motion energy", context.relEnergy, "(m/s^2)^2");
  else                  printText("Relative motion", "n/a (" BAND_ACCEL_NAME " not read) - gating on finger motion");
  printText("Motion gate", !latest.motionQuiet ? "MOVING - data rejected"
                         : latest.smallMotion ? "small motion - cancelled, PPG clean, data used" : "quiet");
#endif
#if USE_ANC
  printFeature("ANC reference energy", latest.ancEnergy, "(m/s^2)^2");
#endif
  // ---- NIRS - not used now (uncomment together with NIRS_MODE 1) ----
  // if (!nirs.r.valid)              printText("NIRS", "no valid light (check LEDs / photodiode / contact)");
  // else if (!nirs.r.baselineReady) printText("NIRS", "capturing baseline");
  // else {
  //   printFeature("NIRS dHbO2", nirs.r.dHbO2, "uM");
  //   printFeature("NIRS dHHb", nirs.r.dHHb, "uM");
  //   printFeature("NIRS dHbT", nirs.r.dHbT, "uM");
  //   printFeature("NIRS dHbDiff", nirs.r.dHbDiff, "uM");
  // }
  out.printf("Samples: %lu  FIFO overflows: %lu  I2C errors: %lu  Spikes removed: %lu  Log lines dropped: %lu\n",
             (unsigned long)samplesTotal, (unsigned long)fifoOverflows, (unsigned long)i2cErrors,
             (unsigned long)(spikesEarlier + pipeline.spikesRemoved()), (unsigned long)out.dropped);
}

// ================= RESETS =================
// Signal-only reset (FIFO overflow, I2C error): filters restart, the baseline is kept.
void resetSignal(const char* why) {
  spikesEarlier += pipeline.spikesRemoved();
  pipeline.begin(); wave.begin(); spo2.begin(); context.begin();
  latest = PpgSample(); sampleMs = millis();
  flowLevel.samplesLost();                                 // filters restart: the last level is stale
  out.printf("# EVENT: %s\n", why);
}
// Full reset ('r' or probe re-seated): the optical coupling changed, so the baseline is invalid.
void resetAll(const char* why, bool armAutoBaseline) {
  resetSignal(why);
  (void)armAutoBaseline;                                   // AUTO_BASELINE: autoReference() re-arms the capture
  perfusion.begin(false);
  wave.clearBaselineTemplate(); flowLevel.begin(); baselineWasReady = false;
  agc.locked = false; autoArmFromMs = millis();
  // nirs.captureBaseline();     // NIRS - not used now
}
// Optical settings changed (LED current, sensor re-initialised): the pulse-strength reference is no longer
// comparable -> cleared; the user captures a new one with 'b'.
void clearReference(const char* why) {
  const bool had = perfusion.baselineReady || perfusion.capturing;
  perfusion.clearBaseline(); wave.clearBaselineTemplate(); flowLevel.clearReference();
  baselineWasReady = false; agc.locked = false; autoArmFromMs = millis();
  if (had) out.printf("# EVENT: REFERENCE CLEARED (%s) - %s\n", why,
                      AUTO_BASELINE ? "re-captured automatically once the signal is steady" : "send 'b' again in the unrestricted condition");
}
// ---- LED calibration: 't' (hand tightened) and 'f' (hand free) -> 10 equal LED steps between them ----
void saveCalibration() {
#if defined(ARDUINO_ARCH_ESP32)
  Preferences pr;
  if (pr.begin("flowcal", false)) { pr.putFloat("tight", flowLevel.xTight); pr.putFloat("free", flowLevel.xFree); pr.end(); }
#endif
}
void loadCalibration() {
#if defined(ARDUINO_ARCH_ESP32) && FL_TWO_POINT_CAL
  Preferences pr;
  if (pr.begin("flowcal", true)) { flowLevel.setCalibration(pr.getFloat("tight", NAN), pr.getFloat("free", NAN)); pr.end(); }
#endif
}
void printCalibration() {
  if (!FL_TWO_POINT_CAL) return;
  out.printf("# LED CALIBRATION: tight x = %.3f, free x = %.3f -> %s\n", (double)flowLevel.xTight, (double)flowLevel.xFree,
             flowLevel.calibrated() ? "IN USE (tight = 1 LED, free = 10, 8 equal bands between)"
             : (isfinite(flowLevel.xTight) && isfinite(flowLevel.xFree)) ? "NOT USED: free must be above tight - capture again"
             : "incomplete (send t with the hand tight, f with the hand free) - fixed thresholds in use");
  out.printf("# LED thresholds (%s): 1 LED below %.3f", flowLevel.calibrated() ? "strength" : "x", (double)flowLevel.levelEdge(0));
  for (int k = 1; k < 9; k++) out.printf(" | %d >= %.3f", k + 1, (double)flowLevel.levelEdge(k - 1));
  out.printf(" | 10 >= %.3f\n", (double)flowLevel.levelEdge(8));
}
void startCalibration(FlowLevel::CalPoint p) {
  const char* name = p == FlowLevel::CAL_TIGHT ? "TIGHT" : "FREE";
  if (!FL_TWO_POINT_CAL) { out.enqueue("# t / f calibration is off (FL_TWO_POINT_CAL 0): LEDs use the nominal 10 x-bands\n"); return; }
  if (!flowLevel.hasReference()) {
    out.printf("# %s capture not possible: no reference yet (keep still until REFERENCE CAPTURED, or send b)\n", name);
    return;
  }
  flowLevel.startCalibration(p);
  out.printf("# %s capture started: keep the hand %s and still for %d pulses\n", name,
             p == FlowLevel::CAL_TIGHT ? "tightened" : "free", FL_CAL_PULSES);
}

// 'b': start a new reference capture (10 accepted still pulses). UNKNOWN until it is complete.
void startReferenceCapture() {
  perfusion.captureBaseline(); wave.clearBaselineTemplate(); flowLevel.clearReference();
  baselineWasReady = false; agc.locked = true;                 // LED current fixed from now on
  out.enqueue("# Reference capture started: unrestricted condition, keep the limb still (10 pulses).\n");
}
// AUTO_BASELINE: (re)start the reference capture whenever there is none - after boot, 'r', probe re-placed, LED
// current change or sensor re-initialisation - but only once the probe is on and the LED current has not changed
// for AUTO_ARM_SETTLE_MS (the capture locks the current; locking it before the AGC settled left it at the clipping
// edge, and a later forced change cleared the reference without ever re-arming the capture).
void autoReference() {
#if AUTO_BASELINE
  if (!sensorReady || !hadTissue || perfusion.baselineReady || perfusion.capturing) return;
  if ((uint32_t)(millis() - autoArmFromMs) < AUTO_ARM_SETTLE_MS) return;
  out.enqueue("# AUTO reference: ");
  startReferenceCapture();
#endif
}

// ================= CONTEXT SENSORS =================
#if USE_FSR
float readFsrMv(int pin) {
  uint32_t s = 0;
  for (int i = 0; i < FSR_OVERSAMPLE; i++) s += analogReadMilliVolts(pin);
  return (float)s / FSR_OVERSAMPLE;
}
#endif

// Loose-wire handling (ppgio::AccelGuard): a failing motion sensor is skipped for 1 s, then re-initialised.
ppgio::AccelGuard accGuard[2];
bool accOkFrom(ppgio::AccelGuard::Result r, int idx, uint8_t addr) {
  if (r == ppgio::AccelGuard::FAILED)
    out.printf("# WARNING: %s motion sensor 0x%02X not answering - check its wires (retry in 1 s)\n",
               idx ? "Band" : "Probe", addr);
  if (r == ppgio::AccelGuard::RECOVERED) out.printf("# %s motion sensor 0x%02X back\n", idx ? "Band" : "Probe", addr);
  return r == ppgio::AccelGuard::OK;
}

void readContext(float* acc, bool* accOk, float* mv) {
  accOk[0] = accOk[1] = false;
  if (NUM_ACCEL >= 1) accOk[0] = accOkFrom(accGuard[0].read(accFinger, ADXL345_FINGER_ADDR, acc), 0, ADXL345_FINGER_ADDR);
#if NUM_ACCEL >= 2
  float band[3] = {0, 0, 0};
  if (accOkFrom(accGuard[1].read(accBand, BAND_ACCEL_ADDR, band), 1, BAND_ACCEL_ADDR)) {
    for (int j = 0; j < 3; j++) acc[3 + j] = BAND_AXIS_SGN[j] * band[BAND_AXIS_IDX[j]];   // into finger frame
    accOk[1] = true;
  }
#endif
  if (!accOk[0]) { acc[0] = acc[1] = acc[2] = 0; }
  if (!accOk[1]) { acc[3] = acc[4] = acc[5] = 0; }
#if USE_FSR
  mv[0] = readFsrMv(FSR1_PIN);
  mv[1] = readFsrMv(FSR2_PIN);
#else
  mv[0] = mv[1] = 0;
#endif
}

// ================= SAMPLE HANDLER =================
void handleSample(uint32_t ir, uint32_t red) {
  sampleMs += 1000 / PPG_FS;                               // 8 ms per FIFO sample
  samplesTotal++;
  { const uint32_t w = millis();                         // a gap > FL_ACQ_TIMEOUT_MS (loop or sensor stalled):
    if (lastSampleWallMs && (uint32_t)(w - lastSampleWallMs) > FL_ACQ_TIMEOUT_MS) flowLevel.samplesLost();   // stale level
    lastSampleWallMs = w; }

  // (2) motion + band pressure
  bool accOk[2];
  readContext(lastAcc, accOk, lastFsrMv);
  context.process(lastFsrMv, lastAcc, accOk);

#if DEBUG_STREAM_RAW
  out.printf("IR:%lu,RED:%lu,fx:%.3f,fy:%.3f,fz:%.3f,bx:%.3f,by:%.3f,bz:%.3f\n", (unsigned long)ir, (unsigned long)red,
             lastAcc[0], lastAcc[1], lastAcc[2], lastAcc[3], lastAcc[4], lastAcc[5]);
#endif

  // (3) PPG pipeline: spikes -> filters -> [ANC] -> smoothing + derivatives -> SQI -> peaks
  // relative-motion modes need BOTH sensors; without the band sensor "relative" would just be finger motion
  const float* refs = (USE_ANC && accOk[0] && (ANC_MODE == ANC_MODE_ABS6 || accOk[1])) ? lastAcc : nullptr;
  latest = pipeline.process((float)ir, refs, sampleMs, context.forceStable, context.motionQuiet, context.smallMotion);

  // probe removed / re-seated: optical coupling changed -> everything incl. the baseline restarts
  // a lift shorter than PROBE_OFF_CLEAR_MS (motion) is invalid data only; longer = removed / re-seated
  static uint32_t noTissueSinceMs = 0;
  if (!latest.optical && hadTissue && !noTissueSinceMs) noTissueSinceMs = sampleMs ? sampleMs : 1;
  if (latest.optical) noTissueSinceMs = 0;
  if (!latest.optical && (uint32_t)(sampleMs - noTissueSinceMs) < PROBE_OFF_CLEAR_MS) {
    // brief lift: keep everything, the sample is invalid for x
  } else if (!latest.optical) {
    noTissueSinceMs = 0;
    if (hadTissue) resetAll(AUTO_BASELINE ? "NO TISSUE ON SENSOR - reference cleared, re-captured from the first pulses when the probe is back"
                                          : "NO TISSUE ON SENSOR - reference cleared, send 'b' again in the unrestricted condition", true);
    hadTissue = false;
  } else {
    hadTissue = true;
  }

  // (4) per-pulse features, (5) flow assessment + relative pulse-strength index
  flowLevel.sample(latest, sampleMs);                      // validity of this sample + detection-rate bins
  if (wave.process(latest)) {
    pulsesTotal++; lastPulseMs = sampleMs;
    perfusion.push(wave.last, sampleMs);
    perfusion.update(latest, sampleMs);                    // so the report shows the new status
    if (perfusion.baselineReady && !baselineWasReady) {    // reference complete: freeze the clean waveform
      wave.freezeBaselineTemplate();
      const FlowReference rf = referenceFrom(perfusion, agc.code[0], agc.code[1]);   // LED currents locked since 'b'
      flowLevel.setReference(rf);
      out.printf("# EVENT: REFERENCE CAPTURED - amplitude %.4g a.u., PI %.4g %%, DC %.0f, HR %.0f bpm, amplitude CV %.3f, "
                 "LEDs RED %.1f mA IR %.1f mA (locked)\n",
                 (double)rf.amp, (double)rf.pi, (double)rf.dc, (double)rf.hr, (double)rf.ampCv,
                 (double)LedAgc::milliamps(rf.ledRed), (double)LedAgc::milliamps(rf.ledIr));
    } else {
      flowLevel.pushPulse(wave.last, sampleMs);            // a NEW pulse after the reference: one x update
    }
    baselineWasReady = perfusion.baselineReady;
    if (outputMode == OUTPUT_MODE_REPORT) printPulseReport(wave.last);
  }
  perfusion.update(latest, sampleMs);

  // (6) SpO2 from the RAW red/IR pair
#if SENSOR_TYPE == SENSOR_MAX30102
  spo2.pushSample(ir, red, latest.sqi_good);
#else
  (void)red;
#endif

  // raw recording: R,ir,red,finger xyz,band xyz,ppg,ppg before ANC
  if (outputMode == OUTPUT_MODE_RAW) {
    out.printf("R,%lu,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f\n", (unsigned long)ir, (unsigned long)red,
               lastAcc[0], lastAcc[1], lastAcc[2], lastAcc[3], lastAcc[4], lastAcc[5], latest.ppg, latest.preAnc);
  }
  // plotter stream
  if (outputMode == OUTPUT_MODE_PLOTTER && (samplesTotal % PLOT_EVERY_N) == 0) {
    out.printf("PPG:%.2f,PPG_noANC:%.2f,VPG/10:%.2f,Beat:%.1f,SQI:%d,RelMotion:%.3f\n", latest.ppg, latest.preAnc,
               latest.vpg * 0.1f, latest.beat ? latest.ppg : 0.0f, latest.sqi_good ? 1 : 0, context.relEnergy);
  }
  if (outputMode == OUTPUT_MODE_TELEPLOT && (samplesTotal % PLOT_EVERY_N) == 0) teleplotSample();
#if USE_WIFI
  if (latest.valid && latest.optical && (samplesTotal % WIFI_PPG_DECIMATE) == 0) telemetry.addSample(latest.ppg);
#endif
}

#if USE_WIFI
// ================= WEBSITE TELEMETRY =================
// Once per second: latest values -> network task. Values that are not measured stay NAN (omitted).
void submitTelemetry(uint32_t nowMs) {
  TelemetrySnapshot s;
  const bool measuring = sensorReady && !fault && latest.valid && latest.optical;
  if (measuring) {
    s.ppg = latest.ppg; s.vpg = latest.vpg; s.apg = latest.apg;
    s.heartRate = displayHr();
    s.motion = !latest.motionQuiet;
    s.quality = !latest.motionQuiet ? "motion" : (latest.sqi_good ? "usable" : "weak_signal");
    if (spo2.isSpO2Valid()) s.spo2 = (float)spo2.getSpO2();
    if (lastPulseMs) {
      s.pulseAgeS = (sampleMs - lastPulseMs) / 1000.0f;
      s.pulseDetected = s.pulseAgeS < 3.0f;
      if (s.pulseDetected) {
        s.perfusionIndex = wave.last.pi;
        s.pulseArea = wave.last.area;
        s.relativeAmplitude = perfusion.ampRatio * 100.0f;     // % of the good-flow baseline
      }
    }
  } else {
    s.quality = (!sensorReady || fault) ? "sensor_fault" : "unavailable";
  }
  telemetry.submit(s);

  // report link changes on serial (the network task cannot use 'out')
  static uint8_t lastStations = 255; static int lastCode = 0; static uint32_t lastIp = 0;
  if (telemetry.stations != lastStations || telemetry.serverIp != lastIp) {
    lastStations = telemetry.stations; lastIp = telemetry.serverIp;
    out.printf("# WIFI: %u device(s) on %s, server %s:%d\n", lastStations, WIFI_AP_SSID,
               lastIp ? IPAddress(lastIp).toString().c_str() : "(waiting for laptop)", SERVER_PORT);
  }
  if (telemetry.lastHttp != lastCode) {
    lastCode = telemetry.lastHttp;
    if (lastCode >= 200 && lastCode < 300) out.printf("# WIFI: website accepting data (HTTP %d)\n", lastCode);
    else if (lastCode == 401) out.enqueue("# WIFI: HTTP 401 - DEVICE_API_KEY differs from tourniquet-monitor/.env\n");
    else if (lastCode == 400) out.enqueue("# WIFI: HTTP 400 - check DEVICE_PATIENT_ID exists on the website\n");
    else if (lastCode == 503) out.enqueue("# WIFI: HTTP 503 - set DEVICE_API_KEY in the server .env\n");
    else out.printf("# WIFI: send failed (%d) - server running? firewall open for port %d?\n", lastCode, SERVER_PORT);
  }
}
#endif

// ================= SETUP =================
void setup() {
  Serial.begin(SERIAL_BAUD);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);      // wait for USB-CDC

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(15);   // a 96-byte FIFO burst takes ~9 ms at 100 kHz
  analogReadResolution(12);
#if USE_FLOW_LEDS
  for (int i = 0; i < 10; i++) pinMode(FLOW_LED_PINS[i], OUTPUT);
  for (int i = 0; i < 10; i++) { setFlowLeds((uint16_t)((1u << (i + 1)) - 1u)); delay(60); }   // LED test: fill
  setFlowLeds(0);
#endif
  flowLevel.begin();
  loadCalibration();
#if USE_FSR
  analogSetPinAttenuation(FSR1_PIN, ADC_11db);
  analogSetPinAttenuation(FSR2_PIN, ADC_11db);
#endif

  printHelp();
  printCalibration();

  // --- MAX30102 ---
  sensorReady = ppgSensor.begin(); agc.begin();
  if (!sensorReady) { fault = "MAX30102 NOT FOUND - check wiring / I2C pins"; out.enqueue("# ERROR: MAX30102 not found\n"); }
  else out.enqueue("# MAX30102 found (125 Hz, RED + IR)\n");

  // --- ADXL345 x2 (missing sensor = warning, its motion gate is simply not used) ---
  if (NUM_ACCEL >= 1) out.enqueue(accFinger.begin(ADXL345_FINGER_ADDR) ? "# ADXL345 finger (0x53) found\n" : "# WARNING: ADXL345 finger (0x53) not found\n");
#if NUM_ACCEL >= 2
  out.enqueue(accBand.begin(BAND_ACCEL_ADDR) ? "# " BAND_ACCEL_NAME " found\n" : "# WARNING: " BAND_ACCEL_NAME " not found\n");
#endif

  if (NUM_ACCEL >= 1 && !accFinger.present) accGuard[0].armRetry();   // keep retrying a missing sensor
#if NUM_ACCEL >= 2
  if (!accBand.present) accGuard[1].armRetry();
#endif

  // --- processing ---
  if (!pipeline.begin()) { fault = "FILTER CONFIGURATION ERROR"; out.enqueue("# ERROR: filter configuration\n"); }
  wave.begin(); spo2.begin(); context.begin(); perfusion.begin(false); agc.locked = false; autoArmFromMs = millis();   // AUTO_BASELINE: see autoReference()
  // nirs.begin();                                        // NIRS - not used now (would set the LED pins as outputs)

#if USE_WIFI
  telemetry.begin();
  out.printf("# WIFI: access point '%s' up at %s - join it from the server laptop\n", WIFI_AP_SSID,
             WiFi.softAPIP().toString().c_str());
#endif
  out.enqueue("# System ready. Place the probe; keep still for the baseline.\n");
  if (sensorReady) ppgSensor.clearFifo();
  sampleMs = millis(); lastSampleUs = micros();
}

// ================= LOOP =================
void loop() {
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  // --- commands ---
  while (Serial.available()) {
    const int c = Serial.read();
    if (c == 'b' || c == 'B') startReferenceCapture();     /* nirs.captureBaseline();  NIRS */
    if (c == 'r' || c == 'R') { resetAll("USER RESET", true); if (sensorReady) ppgSensor.clearFifo(); }
    if (c == 'p' || c == 'P') outputMode = (outputMode + 1) % 3;   // report -> plotter -> teleplot
    if (c == 'w' || c == 'W') outputMode = OUTPUT_MODE_RAW;         // raw recording (offline motion analysis)
    if (c == 'h' || c == 'H') { printHelp(); printCalibration(); }
    if (c == 't' || c == 'T') startCalibration(FlowLevel::CAL_TIGHT);
    if (c == 'f' || c == 'F') startCalibration(FlowLevel::CAL_FREE);
    if (c == 'c' || c == 'C') { flowLevel.clearCalibration(); saveCalibration(); printCalibration(); }
  }

#if SENSOR_TYPE == SENSOR_ANALOG
  // Software-timed sampling for an analog PPG sensor (no RED channel -> no SpO2)
  static uint32_t nextTickUs = micros();
  if ((int32_t)(nowUs - nextTickUs) >= 0) {
    nextTickUs += 1000000UL / PPG_FS;
    handleSample((uint32_t)analogRead(PPG_ADC_PIN) * 64u, 0);
  }
#else
  // --- MAX30102: drain the hardware FIFO (it paces the samples at exactly 125 Hz) ---
  if (sensorReady) {
    uint8_t count = 0; bool overflow = false, ambient = false;
    if (!ppgSensor.available(count, overflow, ambient)) {
      i2cErrors++; fault = "I2C ERROR";
      if (++i2cErrorRun > 50) { sensorReady = false; i2cErrorRun = 0; resetSignal("I2C BUS LOST - retrying sensor");
                                clearReference("sensor lost"); }
    } else if (overflow) {
      fifoOverflows++; ppgSensor.clearFifo();
      resetSignal(ambient ? "AMBIENT LIGHT OVERFLOW + FIFO OVERFLOW (samples lost)" : "FIFO OVERFLOW (samples lost)");
      lastSampleUs = nowUs;
    } else if (count > 0) {
      uint32_t ir[32], red[32];
      if (ppgSensor.readPairs(ir, red, count)) {
        for (int k = 0; k < count; k++) {
#if USE_LED_AGC
          if (agc.process(ppgSensor, ir[k], red[k])) {     // brightness changed: restart the filters
            out.printf("# LED: RED %.1f mA, IR %.1f mA (DC was RED %.0f, IR %.0f counts)\n",
                       LedAgc::milliamps(agc.code[0]), LedAgc::milliamps(agc.code[1]), agc.dc[0], agc.dc[1]);
            resetSignal("LED CURRENT ADJUSTED - filters restart");
            clearReference("LED current changed");         // reference no longer comparable
            autoArmFromMs = millis();                      // (also restarts the settle time without a reference)
          }
          if (agc.settling()) continue;                    // samples still taken at the old current: skip
                                                           // (no clearFifo: a pointer reset can leave the FIFO 'full')
#endif
          handleSample(ir[k], red[k]);
        }
        lastSampleUs = nowUs; fault = nullptr; i2cErrorRun = 0;
      } else {
        i2cErrors++; fault = "I2C ERROR"; ppgSensor.clearFifo(); resetSignal("FIFO READ ERROR");
      }
    } else if ((uint32_t)(nowUs - lastSampleUs) > 200000UL) {   // no sample for 200 ms
      fault = "MAX30102 STOPPED - re-initialising";
      sensorReady = ppgSensor.begin(); agc.begin();
      resetSignal("SENSOR TIMEOUT");
      clearReference("sensor re-initialised");
      lastSampleUs = nowUs;
    }
  } else {
    static uint32_t lastRetryMs = 0;
    if (nowMs - lastRetryMs > 2000) {                       // retry a missing sensor every 2 s
      lastRetryMs = nowMs;
      sensorReady = ppgSensor.begin(); agc.begin();
      if (sensorReady) { fault = nullptr; resetSignal("MAX30102 CONNECTED"); clearReference("sensor re-initialised");
                         lastSampleUs = micros(); }
    }
  }
#endif

  // --- NIRS - not used now (non-blocking LED1 / LED2 / dark sequence) ---
  // nirs.update(nowUs);

  // --- status once per second ---
  if (outputMode == OUTPUT_MODE_REPORT && nowMs - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = nowMs;
    printStatus(nowMs);
  }
  // --- relative pulse-strength index: validity every loop, LEDs = the level (steady, all off = UNKNOWN) ---
  // millis() now, not nowMs: lastSampleWallMs was set AFTER nowMs during this loop, and nowMs - lastSampleWallMs
  // would wrap to ~4e9 -> "acquisition stopped" -> the level was wiped right after every pulse that set it.
  autoReference();
  flowLevel.update(sensorReady && !fault, sensorReady && (uint32_t)(millis() - lastSampleWallMs) <= FL_ACQ_TIMEOUT_MS);
  if (const FlowLevel::CalPoint done = flowLevel.takeCalibrationDone()) {
    out.printf("# %s LEVEL CAPTURED: x = %.3f\n", done == FlowLevel::CAL_TIGHT ? "TIGHT" : "FREE",
               (double)(done == FlowLevel::CAL_TIGHT ? flowLevel.xTight : flowLevel.xFree));
    saveCalibration(); printCalibration();
  }
  static int levelShown = -1;
  if (flowLevel.level != levelShown) {
    levelShown = flowLevel.level;
    if (outputMode == OUTPUT_MODE_REPORT)
      out.printf("# RELATIVE PULSE LEVEL %d/10: %s%s%s\n", flowLevel.level, flowLevelText(flowLevel.level),
                 flowLevel.level ? "" : " - ", flowLevel.level ? "" : flowUnknownText(flowLevel.why));
  }
  setFlowLeds(flowLevel.ledMask());
  if (outputMode == OUTPUT_MODE_TELEPLOT) teleplotIndex(nowMs);

#if USE_WIFI
  static uint32_t lastTelemetryMs = 0;
  if (nowMs - lastTelemetryMs >= WIFI_SEND_PERIOD_MS) { lastTelemetryMs = nowMs; submitTelemetry(nowMs); }
#endif

  out.pump();
}

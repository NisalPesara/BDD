/*
  ESP32 real-time PPG pipeline + limb blood-flow assessment + SpO2
  Sensors NOW : MAX30102 (IR + RED) + 1x ADXL345 (motion)   [classic ESP32, I2C SDA 22 / SCL 23]
  Prepared, commented out until the hardware exists: 2nd ADXL345, 2x FSR, NIRS (see config.h)
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
    band ADXL345 0x1D (SDO -> 3.3 V)

  SERIAL (460800 baud) - commands:  b = capture baseline   r = reset   p = plotter/report   h = help
  OUTPUT ("Name: value unit"):
    per accepted pulse : PULSE REPORT with all features, flow ratios, flow status, SpO2
    every second       : STATUS block (flow status, signal quality, HR, SpO2, motion)
    plotter mode       : "PPG:..,VPG:..,APG:..,Beat:.." per sample for the Arduino Serial Plotter

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
// #include "m13_nirs.h"         // NIRS - not used now
#include "serial_logger.h"

// ================= SENSORS =================
ppgio::Max30102 ppgSensor;
ppgio::Adxl345  accFinger;
#if NUM_ACCEL >= 2
ppgio::Adxl345  accBand;          // second (band) ADXL345 - only when NUM_ACCEL 2
#endif

// ================= PROCESSING =================
PpgPipeline        pipeline;      // filters, derivatives, SQI, peaks
WaveAnalyzer       wave;          // per-pulse features
PerfusionMonitor   perfusion;     // flow vs baseline
SpO2Processor      spo2;          // SpO2 from raw RED/IR
MeasurementContext context;       // motion gate (+ FSR pressure gate when USE_FSR 1)
// NirsMonitor     nirs;          // NIRS - not used now
SerialLogger       out;           // non-blocking serial output

PpgSample latest;
float lastAcc[6] = {0, 0, 0, 0, 0, 0};
float lastFsrMv[2] = {0, 0};
uint32_t sampleMs = 0;            // time stamp of the current sample (exactly 8 ms per sample)
uint32_t samplesTotal = 0, fifoOverflows = 0, i2cErrors = 0, lastSampleUs = 0, lastStatusMs = 0;
uint8_t  i2cErrorRun = 0;          // consecutive I2C failures
uint32_t pulsesTotal = 0;          // accepted pulses since boot
unsigned long spikesEarlier = 0;   // spikes removed before the last filter reset
bool sensorReady = false, hadTissue = false;
int  outputMode = OUTPUT_MODE;
const char* fault = nullptr;

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
  out.enqueue("# Commands: b = capture baseline (good-flow reference)  r = reset  p = cycle report/plotter/teleplot  h = help\n");
  out.enqueue("# Keep the limb still with normal flow while the baseline is captured.\n\n");
}

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
  if (spo2.getSpO2() > 0) printFeature("SpO2", (float)spo2.getSpO2(), "%");
  else                    printText("SpO2", "n/a");
  out.enqueue("===================================================\n");
}

void printStatus(uint32_t nowMs) {
  out.printf("\n---- STATUS  t=%.1f s ----\n", nowMs / 1000.0);
  if (fault) printText("Sensor", fault);
  printText("Flow status", perfusion.text());
  out.printf("Signal quality: %s (fail bits %u)\n", latest.sqi_good ? "good" : "poor", (unsigned)latest.sqi.fail);
  printFeature("Periodicity", latest.sqi.periodicity, "0..1");
  printFeature("Heart rate", latest.bpm, "bpm");
  if (spo2.getSpO2() > 0) printFeature("SpO2", (float)spo2.getSpO2(), "%"); else printText("SpO2", "n/a");
  printFeature("SpO2 ratio R", spo2.ratio, "ratio");
  printFeature("Raw IR", latest.raw, "counts");
#if USE_FSR
  printFeature("Band force 1", context.forceUs[0], "uS");
  printFeature("Band force 2", context.forceUs[1], "uS");
#endif
#if NUM_ACCEL > 0
  printFeature("Motion energy", context.dynamicEnergy, "(m/s^2)^2");
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
  out.printf("# EVENT: %s\n", why);
}
// Full reset ('r' or probe re-seated): the optical coupling changed, so the baseline is invalid.
void resetAll(const char* why, bool armAutoBaseline) {
  resetSignal(why);
  perfusion.begin(armAutoBaseline && AUTO_BASELINE);
  // nirs.captureBaseline();     // NIRS - not used now
}

// ================= CONTEXT SENSORS =================
#if USE_FSR
float readFsrMv(int pin) {
  uint32_t s = 0;
  for (int i = 0; i < FSR_OVERSAMPLE; i++) s += analogReadMilliVolts(pin);
  return (float)s / FSR_OVERSAMPLE;
}
#endif

void readContext(float* acc, bool* accOk, float* mv) {
  accOk[0] = accOk[1] = false;
  if (NUM_ACCEL >= 1 && accFinger.present) accOk[0] = accFinger.read(acc);
#if NUM_ACCEL >= 2
  float band[3] = {0, 0, 0};
  if (accBand.present && accBand.read(band)) {
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

  // (2) motion + band pressure
  bool accOk[2];
  readContext(lastAcc, accOk, lastFsrMv);
  context.process(lastFsrMv, lastAcc, accOk);

#if DEBUG_STREAM_RAW
  out.printf("IR:%lu,RED:%lu,fx:%.3f,fy:%.3f,fz:%.3f,bx:%.3f,by:%.3f,bz:%.3f\n", (unsigned long)ir, (unsigned long)red,
             lastAcc[0], lastAcc[1], lastAcc[2], lastAcc[3], lastAcc[4], lastAcc[5]);
#endif

  // (3) PPG pipeline: spikes -> filters -> [ANC] -> smoothing + derivatives -> SQI -> peaks
  const float* refs = (USE_ANC && accOk[0]) ? lastAcc : nullptr;
  latest = pipeline.process((float)ir, refs, sampleMs, context.forceStable, context.motionQuiet);

  // probe removed / re-seated: optical coupling changed -> everything incl. the baseline restarts
  if (!latest.optical) {
    if (hadTissue) resetAll("NO TISSUE ON SENSOR - baseline cleared, send 'b' (or 'r') when the probe is back", false);
    hadTissue = false;
  } else {
    hadTissue = true;
  }

  // (4) per-pulse features, (5) flow assessment
  if (wave.process(latest)) {
    pulsesTotal++;
    perfusion.push(wave.last, sampleMs);
    perfusion.update(latest, sampleMs);                    // so the report shows the new status
    if (outputMode == OUTPUT_MODE_REPORT) printPulseReport(wave.last);
    if (outputMode == OUTPUT_MODE_TELEPLOT) out.printf(">HR:%.1f\n", wave.last.bpm);
  }
  perfusion.update(latest, sampleMs);

  // (6) SpO2 from the RAW red/IR pair
#if SENSOR_TYPE == SENSOR_MAX30102
  spo2.pushSample(ir, red, latest.sqi_good);
#else
  (void)red;
#endif

  // plotter stream
  if (outputMode == OUTPUT_MODE_PLOTTER && (samplesTotal % PLOT_EVERY_N) == 0) {
    out.printf("PPG:%.2f,VPG/10:%.2f,APG/100:%.2f,Beat:%.1f,SQI:%d\n", latest.ppg, latest.vpg * 0.1f,
               latest.apg * 0.01f, latest.beat ? latest.ppg : 0.0f, latest.sqi_good ? 1 : 0);
  }
  if (outputMode == OUTPUT_MODE_TELEPLOT && (samplesTotal % PLOT_EVERY_N) == 0) {
    out.printf(">PPG:%.2f\n>VPG:%.2f\n>APG:%.2f\n>Beat:%d\n>SQI:%d\n", latest.ppg, latest.vpg, latest.apg,
               latest.beat ? 1 : 0, latest.sqi_good ? 1 : 0);
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(SERIAL_BAUD);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);      // wait for USB-CDC

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(5);
  analogReadResolution(12);
#if USE_FSR
  analogSetPinAttenuation(FSR1_PIN, ADC_11db);
  analogSetPinAttenuation(FSR2_PIN, ADC_11db);
#endif

  printHelp();

  // --- MAX30102 ---
  sensorReady = ppgSensor.begin();
  if (!sensorReady) { fault = "MAX30102 NOT FOUND - check wiring / I2C pins"; out.enqueue("# ERROR: MAX30102 not found\n"); }
  else out.enqueue("# MAX30102 found (125 Hz, RED + IR)\n");

  // --- ADXL345 x2 (missing sensor = warning, its motion gate is simply not used) ---
  if (NUM_ACCEL >= 1) out.enqueue(accFinger.begin(ADXL345_FINGER_ADDR) ? "# ADXL345 finger (0x53) found\n" : "# WARNING: ADXL345 finger (0x53) not found\n");
#if NUM_ACCEL >= 2
  out.enqueue(accBand.begin(ADXL345_BAND_ADDR) ? "# ADXL345 band (0x1D) found\n" : "# WARNING: ADXL345 band (0x1D) not found\n");
#endif

  // --- processing ---
  if (!pipeline.begin()) { fault = "FILTER CONFIGURATION ERROR"; out.enqueue("# ERROR: filter configuration\n"); }
  wave.begin(); spo2.begin(); context.begin(); perfusion.begin(AUTO_BASELINE);
  // nirs.begin();                                        // NIRS - not used now (would set the LED pins as outputs)

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
    if (c == 'b' || c == 'B') { perfusion.captureBaseline(); /* nirs.captureBaseline();  NIRS */
                                out.enqueue("# Baseline capture started: keep the limb still, flow normal.\n"); }
    if (c == 'r' || c == 'R') { resetAll("USER RESET", true); if (sensorReady) ppgSensor.clearFifo(); }
    if (c == 'p' || c == 'P') outputMode = (outputMode + 1) % 3;   // report -> plotter -> teleplot
    if (c == 'h' || c == 'H') printHelp();
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
      if (++i2cErrorRun > 50) { sensorReady = false; i2cErrorRun = 0; resetSignal("I2C BUS LOST - retrying sensor"); }
    } else if (overflow) {
      fifoOverflows++; ppgSensor.clearFifo();
      resetSignal(ambient ? "AMBIENT LIGHT OVERFLOW + FIFO OVERFLOW (samples lost)" : "FIFO OVERFLOW (samples lost)");
      lastSampleUs = nowUs;
    } else if (count > 0) {
      uint32_t ir[32], red[32];
      if (ppgSensor.readPairs(ir, red, count)) {
        for (int k = 0; k < count; k++) handleSample(ir[k], red[k]);
        lastSampleUs = nowUs; fault = nullptr; i2cErrorRun = 0;
      } else {
        i2cErrors++; fault = "I2C ERROR"; ppgSensor.clearFifo(); resetSignal("FIFO READ ERROR");
      }
    } else if ((uint32_t)(nowUs - lastSampleUs) > 200000UL) {   // no sample for 200 ms
      fault = "MAX30102 STOPPED - re-initialising";
      sensorReady = ppgSensor.begin();
      resetSignal("SENSOR TIMEOUT");
      lastSampleUs = nowUs;
    }
  } else {
    static uint32_t lastRetryMs = 0;
    if (nowMs - lastRetryMs > 2000) {                       // retry a missing sensor every 2 s
      lastRetryMs = nowMs;
      sensorReady = ppgSensor.begin();
      if (sensorReady) { fault = nullptr; resetSignal("MAX30102 CONNECTED"); lastSampleUs = micros(); }
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

  out.pump();
}

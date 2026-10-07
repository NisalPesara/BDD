#pragma once
#include <Arduino.h>
#include <esp_timer.h>
#include <stdio.h>
#include "config.h"
#include "sensor_drivers.h"
#include "pipeline.h"
#include "wave_detection.h"
#include "spo2_processor.h"
#include "perfusion_monitor.h"
#include "measurement_context.h"
#include "nirs_adapter.h"
#include "serial_logger.h"

namespace ppgapp {
static ppgio::Max30102 sensor;
static ppgio::Adxl345 accSensor[2];
static PpgPipeline pipeline;
static WaveAnalyzer wave;
static SpO2Processor spo2;
static PerfusionMonitor perfusion;
static MeasurementContext context;
static SerialLogger logger;
static PpgSample latest;
static uint32_t gaps = 0, discarded = 0, epoch = 0, samples = 0;
static uint32_t maxProcessUs = 0;
static uint64_t sampleTime = 0, lastArrival = 0, lastStatus = 0, lastNirs = 0;
static bool sensorReady = false, hadOptical = false;
static const char* fault = "STARTING";
static char line[1400];

inline void resetAnalysis(const char* why) {
  pipeline.begin(); wave.begin(); spo2.begin(); perfusion.reset(); context.begin();
  snprintf(line,sizeof(line),"SPO2,%llu,-1,0,nan,nan,nan\n",(unsigned long long)esp_timer_get_time());
  logger.enqueue(line);
  latest = PpgSample(); sampleTime = 0; hadOptical = false; ++epoch;
  snprintf(line, sizeof(line), "EVENT,%llu,%lu,%s\n", (unsigned long long)esp_timer_get_time(), (unsigned long)epoch, why);
  logger.enqueue(line);
}
inline void headers() {
  logger.enqueue("# Research prototype; no automatic tourniquet control. nan/-1 = unavailable.\n");
  logger.enqueue("# SAMPLE,epoch,t_us,aligned_us,ir,red,ppg,vpg,apg,valid,sqi_good,sqi_fail,periodicity,pi_window,beat,bpm,fsr1_mv,fsr2_mv,fsr1_uS,fsr2_uS,ax,ay,az,rx,ry,rz,refs_valid,motion_energy,anc_energy,contact,force_stable\n");
  logger.enqueue("# PULSE,epoch,onset_us,reported_us,bpm,amplitude_counts,pi_pct,auc_counts_s,sys_auc,dia_auc,crest_s,width50_s,notch_type,notch_index,dia_index,a_index,b_index,c_index,d_index,e_index,f_index,b_over_a,amp_ratio,pi_ratio,area_ratio\n");
  logger.enqueue("# SPO2,t_us,estimate_pct,algorithm_valid,ratio,red_ir_corr,reference_hr\n");
  logger.enqueue("# STATUS,t_us,state,sqi_fail,baseline_ready,gaps,discarded_samples,log_drops,max_process_us,samples_total\n");
  logger.enqueue("# NIRS,t_us,enabled,raw_valid,ch1_mv,ch2_mv\n");
}
inline void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin(I2C_SDA, I2C_SCL); Wire.setClock(I2C_CLOCK_HZ); Wire.setTimeOut(5);
  analogReadResolution(FSR_ADC_BITS);
#if USE_FSR
  analogSetPinAttenuation(FSR1_PIN, ADC_11db); analogSetPinAttenuation(FSR2_PIN, ADC_11db);
#endif
#if NIRS_MODE == 1
  analogSetPinAttenuation(NIRS_ADC_1_PIN, ADC_11db); analogSetPinAttenuation(NIRS_ADC_2_PIN, ADC_11db);
#endif
  headers(); resetAnalysis("BOOT");
  snprintf(line, sizeof(line), "# CONFIG,fs=%d,sda=%d,scl=%d,accel=%d,anc=%d,fsr=%d,contact_fsr=%d,nirs_mode=%d,sg_samples=%d,stream_every=%d\n",
           PPG_FS,I2C_SDA,I2C_SCL,NUM_ACCEL,USE_ANC,USE_FSR,CONTACT_FSR_INDEX,NIRS_MODE,2*SG_HALF+1,STREAM_EVERY_N);
  logger.enqueue(line);
  sensorReady = sensor.begin();
  for (int j = 0; j < NUM_ACCEL; ++j) {
    const bool ok = accSensor[j].begin(j ? ADXL345_REFERENCE_ADDR : ADXL345_PROBE_ADDR);
    if (!ok) logger.enqueue(j ? "# Missing reference ADXL345; readings will be invalid.\n" : "# Missing probe ADXL345; readings will be invalid.\n");
  }
  if (!pipeline.begin()) { sensorReady = false; fault = "FILTER_CONFIGURATION_ERROR"; }
  else fault = sensorReady ? "WARMUP" : "MAX30102_INIT_ERROR";
  sensor.clearFifo(); lastArrival = uint64_t(esp_timer_get_time());
}
inline void handlePair(uint32_t ir, uint32_t red, uint64_t arrival) {
  const uint64_t begin = uint64_t(esp_timer_get_time());
  if (!sampleTime) sampleTime = arrival; else sampleTime += PPG_PERIOD_US;
  float acc[6] = {}, millivolts[2] = {};
  bool referencesGood = true;
  for (int j = 0; j < NUM_ACCEL; ++j) referencesGood &= accSensor[j].read(acc + 3*j);
  float tmp[3] = {acc[3],acc[4],acc[5]};
  for (int j = 0; j < 3; ++j) acc[3+j] = REFERENCE_AXIS_SIGN[j] * tmp[REFERENCE_AXIS_IDX[j]];
  if (uint64_t(esp_timer_get_time()) - begin > ACC_MAX_AGE_US) referencesGood = false;
#if USE_FSR
  millivolts[0] = float(analogReadMilliVolts(FSR1_PIN));
  millivolts[1] = float(analogReadMilliVolts(FSR2_PIN));
#endif
  context.process(millivolts, acc, referencesGood);
  latest = pipeline.process(float(ir), referencesGood && NUM_ACCEL ? acc : nullptr,
                            sampleTime, context.contact, context.stable, context.motionQuiet);
  if ((!latest.optical || !latest.contact || latest.clipped) && hadOptical) {
    const PpgSample reason = latest;
    resetAnalysis("OPTICAL_OR_CONTACT_LOSS"); // reset all histories, including fixed baseline
    latest = reason; sampleTime = arrival;
  }
  hadOptical = latest.optical && latest.contact && !latest.clipped;
  if (!latest.sqi_good) perfusion.breakContinuity();
  const bool pulse = wave.process(latest);
  if (pulse) {
    const auto& f = wave.last; perfusion.push(f, sampleTime);
    snprintf(line, sizeof(line),
      "PULSE,%lu,%llu,%llu,%.2f,%.4f,%.5f,%.6f,%.6f,%.6f,%.4f,%.4f,%u,%d,%d,%d,%d,%d,%d,%d,%d,%.4f,%.4f,%.4f,%.4f\n",
      (unsigned long)epoch,(unsigned long long)f.onset_us,(unsigned long long)sampleTime,
      f.bpm,f.amp,f.pi,f.area,f.sysArea,f.diaArea,f.crest,f.pw50,unsigned(f.notchType),f.notch,f.dia,
      f.a,f.b,f.c,f.d,f.e,f.f,f.bOverA,perfusion.amplitudeRatio,perfusion.piRatio,perfusion.areaRatio);
    logger.enqueue(line);
  }
  // Raw paired channels use their own matched low-pass branch.
  const bool spo2WasValid = spo2.isSpO2Valid();
  spo2.pushSample(ir, red, sampleTime, latest.sqi_good);
  if (spo2.fresh || (spo2WasValid && !spo2.isSpO2Valid())) {
    snprintf(line,sizeof(line),"SPO2,%llu,%d,%d,%.5f,%.4f,%.2f\n",(unsigned long long)sampleTime,
      spo2.getSpO2(),int(spo2.isSpO2Valid()),spo2.last.ratio,spo2.last.correlation,spo2.last.bpm);
    logger.enqueue(line);
  }
  if (++samples % STREAM_EVERY_N == 0) {
    snprintf(line,sizeof(line),
      "SAMPLE,%lu,%llu,%llu,%lu,%lu,%.4f,%.4f,%.4f,%d,%d,%u,%.4f,%.5f,%d,%.2f,%.1f,%.1f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.5f,%.5f,%d,%d\n",
      (unsigned long)epoch,(unsigned long long)latest.t_us,(unsigned long long)latest.aligned_us,
      (unsigned long)ir,(unsigned long)red,latest.ppg,latest.vpg,latest.apg,int(latest.valid),int(latest.sqi_good),
      unsigned(latest.sqi.fail),latest.sqi.periodicity,latest.sqi.pi,int(latest.beat),latest.bpm,
      millivolts[0],millivolts[1],context.forceUs[0],context.forceUs[1],acc[0],acc[1],acc[2],acc[3],acc[4],acc[5],
      int(referencesGood && NUM_ACCEL),context.dynamicEnergy,latest.ancEnergy,int(latest.contact),int(latest.forceStable));
    logger.enqueue(line);
  }
  const uint32_t elapsed = uint32_t(uint64_t(esp_timer_get_time()) - begin);
  if (elapsed > maxProcessUs) maxProcessUs = elapsed;
}
inline void loop() {
  const uint64_t now = uint64_t(esp_timer_get_time());
  if (Serial.available()) {
    const int cmd = Serial.read();
    if (cmd == 'b' || cmd == 'B') { perfusion.captureBaseline(); logger.enqueue("# Baseline requested: waiting for 10 accepted pulses.\n"); }
    if (cmd == 'r' || cmd == 'R') { resetAnalysis("USER_RESET"); sensor.clearFifo(); }
    if (cmd == 'h' || cmd == 'H') headers();
  }
  if (sensorReady) {
    uint8_t count = 0; bool overflow = false, ambient = false;
    if (!sensor.available(count, overflow, ambient)) {
      if (fault != nullptr && strcmp(fault,"I2C_ERROR") != 0) { ++gaps; resetAnalysis("I2C_ERROR"); }
      fault = "I2C_ERROR"; sensor.clearFifo();
    } else if (overflow || ambient || count > 1 || (now - lastArrival > 100000 && count)) {
      ++gaps; discarded += count;
      resetAnalysis(ambient ? "AMBIENT_LIGHT_OVERFLOW" : "FIFO_BACKLOG_OR_OVERFLOW");
      sensor.clearFifo(); lastArrival = now; fault = "ACQUISITION_GAP";
    } else if (count == 1) {
      uint32_t ir = 0, red = 0;
      if (sensor.readPair(ir, red)) { handlePair(ir, red, now); lastArrival = now; fault = nullptr; }
      else { ++gaps; resetAnalysis("FIFO_READ_ERROR"); sensor.clearFifo(); fault = "I2C_ERROR"; }
    } else if (now - lastArrival > 100000 && (!fault || strcmp(fault,"SENSOR_TIMEOUT") != 0)) {
      ++gaps; resetAnalysis("SENSOR_TIMEOUT"); sensor.clearFifo(); fault = "SENSOR_TIMEOUT";
    }
  }
  if (now - lastNirs >= 1000000UL / NIRS_LOG_HZ) {
    lastNirs = now;
#if NIRS_MODE == 1
    const auto n = readNirs(now);
    snprintf(line,sizeof(line),"NIRS,%llu,%d,%d,%.1f,%.1f\n",(unsigned long long)n.t_us,int(n.enabled),int(n.valid),n.ch1_mv,n.ch2_mv);
    logger.enqueue(line);
#endif
  }
  if (now - lastStatus >= 1000000) {
    lastStatus = now;
    snprintf(line,sizeof(line),"STATUS,%llu,%s,%u,%d,%lu,%lu,%lu,%lu,%lu\n",(unsigned long long)now,
      fault ? fault : perfusion.status(latest,now),unsigned(latest.sqi.fail),int(perfusion.baselineReady),
      (unsigned long)gaps,(unsigned long)discarded,(unsigned long)logger.dropped,(unsigned long)maxProcessUs,(unsigned long)samples);
    logger.enqueue(line);
  }
  logger.pump();
  delay(1); // bounded cooperative yield; hardware FIFO continues sampling
}
} // namespace ppgapp

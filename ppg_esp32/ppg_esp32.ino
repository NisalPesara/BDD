/*
  ESP32 real-time PPG purification pipeline + wave-properties analysis
  Sensors : 1x MAX30102 (IR) + ADXL345 accelerometer (posture context)
  Based on: Park et al. 2022, Front. Physiol. 12:808451  +  pyPPG preprocessing

  SIGNAL CHAIN (from the library modules):
    raw -> DC removal -> Chebyshev II bandpass (0.5–12 Hz, order 4, m01)
        -> moving-average smoothing (m04) -> clean PPG
    clean PPG -> VPG/APG/JPG (derivatives.h) | upslope (m03) | SQI (m07)
             | adaptive peak detection (m08) -> wave_detection.h (feature extraction)

  The old code used a Savitzky-Golay 5-point filter and batch processing.
  This version uses the library's Chebyshev II IIR filter (pyPPG default)
  and processes sample-by-sample in real-time, then extracts wave features
  per-pulse in wave_detection.h when a beat boundary is detected.

  ARDUINO IDE SETTINGS (Tools menu)
    Board ................ ESP32S3 Dev Module  (or your ESP32 variant)
    USB CDC On Boot ...... Enabled
  Wiring (3.3 V, I2C):  SDA=GPIO22  SCL=GPIO23
    MAX30102 (0x57), ADXL345 (0x53, SDO=GND)

  Libraries (Library Manager):
    - SparkFun MAX3010x Pulse and Proximity Sensor Library
    - Adafruit ADXL345
    - Adafruit Unified Sensor
  Output: CSV lines  t_ms,ppg,vpg,apg,jpg,sqi_good,beat,bpm
          + periodic wave-analysis reports (per-pulse feature extraction)
*/
#include <Wire.h>
#include "config.h"
#include "pipeline.h"
#include "wave_detection.h"

// ================= SENSOR DRIVERS =================
#if SENSOR_TYPE == SENSOR_MAX30102
  #include "MAX30105.h"          // SparkFun library; also drives the MAX30102
  MAX30105 sensor;
#endif

// Single ADXL345 for posture context (not for ANC — that needs 2 accelerometers)
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_Sensor.h>
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);

// ================= PIPELINE =================
PpgPipeline pipeline;

// ================= POSTURE CONTEXT =================
// Read the single ADXL345 for posture reporting (not used in pipeline filtering).
// The NLMS ANC (m06) requires TWO accelerometers — disabled by default in config.h.
float lastAccelX = 0, lastAccelY = 0, lastAccelZ = 0;
unsigned long lastAccelReadMs = 0;
const unsigned long ACCEL_READ_INTERVAL_MS = 200;  // read at 5 Hz (plenty for posture)

void readPostureAccel() {
  if (millis() - lastAccelReadMs >= ACCEL_READ_INTERVAL_MS) {
    lastAccelReadMs = millis();
    sensors_event_t event;
    adxl.getEvent(&event);
    lastAccelX = event.acceleration.x;
    lastAccelY = event.acceleration.y;
    lastAccelZ = event.acceleration.z;
  }
}

// ================= SAMPLE HANDLER =================
// Feeds each new IR sample through the library pipeline and forwards the
// purified result to wave_detection.h for per-pulse feature extraction.
void handleSample(float raw) {
#if USE_ANC
  // If ANC were enabled, we'd read dual accelerometers here.
  // Currently disabled (single ADXL345 is only for posture context).
  float refs[ACC_RAW_CH] = {0};
  PpgSample s = pipeline.process(raw, refs, millis());
#else
  PpgSample s = pipeline.process(raw, nullptr, millis());
#endif

  // Stream purified signal with variable names
  if (s.valid) {
    Serial.printf("s.t_ms:     %lu\n",   (unsigned long)s.t_ms);
    Serial.printf("s.raw:      %.1f\n",  s.raw);
    Serial.printf("s.ppg:      %.4f\n",  s.ppg);
    Serial.printf("s.vpg:      %.4f\n",  s.vpg);
    Serial.printf("s.apg:      %.4f\n",  s.apg);
    Serial.printf("s.jpg:      %.4f\n",  s.jpg);
    Serial.printf("s.upslope:  %.4f\n",  s.upslope);
    Serial.printf("s.valid:    %d\n",    s.valid ? 1 : 0);
    Serial.printf("s.sqi_good: %d\n",    s.sqi_good ? 1 : 0);
    Serial.printf("s.beat:     %d\n",    s.beat ? 1 : 0);
    Serial.printf("s.bpm:      %.1f\n",  s.bpm);
    Serial.println();
  }

  // Per-pulse wave-properties analysis (implemented in wave_detection.h)
  waveDetectionProcess(s);
}

// ================= HALT HELPER =================
void halt(const char* msg) { while (1) { Serial.println(msg); delay(1000); } }

// ================= SETUP =================
void setup() {
  Serial.begin(SERIAL_BAUD);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);   // wait for USB-CDC

  Serial.println("\n=================================");
  Serial.println("PPG PIPELINE + WAVE ANALYSIS");
  Serial.printf( "Sample rate: %d Hz\n", PPG_FS);
  Serial.printf( "Filter: Chebyshev II 0.5-12 Hz (pyPPG default)\n");
  Serial.println("=================================");
  Serial.println("# t_ms,ppg,vpg,apg,jpg,sqi_good,beat,bpm");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);

  // --- Initialize MAX30102 ---
  Serial.println("Initializing MAX30102...");
#if SENSOR_TYPE == SENSOR_MAX30102
  if (!sensor.begin(Wire, I2C_SPEED_FAST)) halt("# ERROR: MAX30102 not found!");
  Serial.println("MAX30102 found!");
  // 1000 sps internal, 8 samples averaged = 125 Hz output (matches PPG_FS).
  //         LED mA  avg  mode  rate  pulse  adc
  sensor.setup(0x24, 8,   2,    1000, 118,   4096);
  // Raise 0x24 (LED current) if IR level is too low.
  // The averaging of 8 at 1000 sps gives exactly 125 Hz.
#endif

  // --- Initialize ADXL345 (posture context) ---
  Serial.println("Initializing ADXL345...");
  if (!adxl.begin(ADXL345_ADDR)) halt("# ERROR: ADXL345 not found!");
  Serial.println("ADXL345 found!");
  adxl.setRange(ADXL345_RANGE_4_G);

  // --- Configure and start pipeline ---
  // The PipelineConfig defaults use FM_CHEBY2_PYPPG (Chebyshev II, 0.5-12 Hz, 4th order)
  // which replaces the old Savitzky-Golay smoothing filter.
  PipelineConfig cfg;
  // cfg.bandpass = FM_BUTTER_CANAC;     // alternative: Butterworth 0.5-10 Hz
  // cfg.useBaselineSub = true;          // enable if baseline wander is a problem
  // cfg.useAmpRegularize = false;       // destroys absolute amplitude (leave off for features)
  pipeline.begin(cfg);

  Serial.println("\nSystem Ready. Place finger on sensor.");
  Serial.println("Waiting for valid signal + good SQI before analysis...\n");
}

// ================= LOOP =================
void loop() {
  // Read posture context (for the wave-analysis report header)
  readPostureAccel();

#if SENSOR_TYPE == SENSOR_ANALOG
  // Software-timed sampling for analog sensor
  static uint32_t nextTickUs = micros();
  static const uint32_t PERIOD_US = 1000000UL / PPG_FS;
  if ((int32_t)(micros() - nextTickUs) >= 0) {
    nextTickUs += PERIOD_US;
    handleSample((float)analogRead(PPG_ADC_PIN));
  }
#else
  // MAX30102: the sensor FIFO paces samples at 125 Hz (1000 sps / 8 avg)
  sensor.check();
  while (sensor.available()) {
    float ir = (float)sensor.getFIFOIR();
    sensor.nextSample();

    // Check if finger is present (IR > 50000 is a typical threshold)
    if (ir > 50000) {
      handleSample(ir);
    } else {
      // No finger: could optionally print status
      static unsigned long lastNoFingerMsg = 0;
      if (millis() - lastNoFingerMsg > 3000) {
        lastNoFingerMsg = millis();
        Serial.println("# No finger detected. Waiting...");
      }
    }
  }
#endif
}

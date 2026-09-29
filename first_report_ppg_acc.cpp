#include <Wire.h>
#include "MAX30105.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

// ================= HARDWARE PINS =================
#define SDA_PIN 22
#define SCL_PIN 23

// ================= CONFIGURATION =================
#define PPG_MAX_SAMPLES 512
const float SAMPLE_RATE_HZ = 100.0f;
const unsigned long SAMPLE_INTERVAL_MS = 1000 / SAMPLE_RATE_HZ;

// ================= SENSORS =================
MAX30105 particleSensor;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// ================= DATA BUFFERS =================
static float rawPpg[PPG_MAX_SAMPLES];
static float smoothPpg[PPG_MAX_SAMPLES];
static float vpg[PPG_MAX_SAMPLES];
static float apg[PPG_MAX_SAMPLES];
size_t bufferIndex = 0;
unsigned long lastSampleTime = 0;

// ================= PPG SIGNAL PROCESSING NAMESPACE =================
namespace ppg {

struct FiducialPoints {
    int onset, peak, offset, notch;
    int u, v, w;
    int a, b, c, d, e, f;
    int zero1, zero2;
    float crestTimeSeconds;
    float deltaTSeconds;
};

struct FeatureResults {
    float systolicAmplitude;
    float pulseToPulseIntervalSeconds;
    float perfusionIndexPercent;
    float pulseWidth50Seconds;
    float pulseArea;
    float systolicArea;
    float diastolicArea;
    float inflectionPointAreaRatio;
    float augmentationIndex;
    float pulseTransitTimeSeconds;
    float deltaTSeconds;
    float largeArteryStiffnessIndex;
    float bOverA, cOverA, dOverA, eOverA;
    float bMinusEOverA;
    float bMinusCMinusDOverA;
    float agingIndex;
};

void smoothSignal(const float *input, float *output, size_t count) {
    static const float k[5] = {-3.0f, 12.0f, 17.0f, 12.0f, -3.0f};
    if (!input || !output || count == 0) return;
    for (size_t i = 0; i < count; ++i) {
        if (i < 2 || i + 2 >= count) { output[i] = input[i]; continue; }
        float sum = 0.0f;
        for (int j = -2; j <= 2; ++j) sum += k[j + 2] * input[i + j];
        output[i] = sum / 35.0f;
    }
}

void firstDerivative(const float *input, float *output, size_t count, float dt) {
    static const float k[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};
    if (!input || !output || count == 0 || dt <= 0.0f) return;
    for (size_t i = 0; i < count; ++i) {
        if (i < 2 || i + 2 >= count) { output[i] = 0.0f; continue; }
        float sum = 0.0f;
        for (int j = -2; j <= 2; ++j) sum += k[j + 2] * input[i + j];
        output[i] = sum / (10.0f * dt);
    }
}

void secondDerivative(const float *input, float *output, size_t count, float dt) {
    static const float k[5] = {2.0f, -1.0f, -2.0f, -1.0f, 2.0f};
    if (!input || !output || count == 0 || dt <= 0.0f) return;
    for (size_t i = 0; i < count; ++i) {
        if (i < 2 || i + 2 >= count) { output[i] = 0.0f; continue; }
        float sum = 0.0f;
        for (int j = -2; j <= 2; ++j) sum += k[j + 2] * input[i + j];
        output[i] = sum / (7.0f * dt * dt);
    }
}

size_t detectPeaks(const float *signal, size_t count, float fs, int *peaks, size_t capacity) {
    if (!signal || !peaks || count < 3 || fs <= 0.0f) return 0;
    float lo = signal[0], hi = signal[0];
    for (size_t i = 1; i < count; ++i) { if (signal[i] < lo) lo = signal[i]; if (signal[i] > hi) hi = signal[i]; }
    const float threshold = lo + 0.35f * (hi - lo);
    const int refractory = (int)(0.30f * fs);
    int previous = -refractory;
    size_t found = 0;
    for (size_t i = 1; i + 1 < count; ++i) {
        if (signal[i] >= threshold && signal[i] >= signal[i - 1] && signal[i] > signal[i + 1] && (int)i - previous >= refractory) {
            if (found < capacity) peaks[found++] = (int)i;
            previous = (int)i;
        }
    }
    return found;
}

int findOnset(const float *signal, int previousPeak, int peak) {
    if (!signal || previousPeak < 0 || peak <= previousPeak) return -1;
    int result = previousPeak;
    for (int i = previousPeak + 1; i <= peak; ++i) if (signal[i] < signal[result]) result = i;
    return result;
}

int findOffset(const float *signal, int peak, int nextPeak) {
    if (!signal || peak < 0 || nextPeak <= peak) return -1;
    int result = peak;
    for (int i = peak + 1; i <= nextPeak; ++i) if (signal[i] < signal[result]) result = i;
    return result;
}

int findNotch(const float *signal, int peak, int offset) {
    if (!signal || peak < 0 || offset - peak < 2) return -1;
    int result = peak + 1;
    for (int i = peak + 2; i < offset; ++i) if (signal[i] < signal[result]) result = i;
    return result;
}

float systolicAmplitude(const float *signal, int onset, int peak) {
    return signal && onset >= 0 && peak > onset ? signal[peak] - signal[onset] : NAN;
}

float pulseInterval(int previousPeak, int peak, float fs) {
    return previousPeak >= 0 && peak > previousPeak && fs > 0.0f ? (peak - previousPeak) / fs : NAN;
}

float perfusionIndex(const float *signal, size_t count) {
    if (!signal || count == 0) return NAN;
    float lo = signal[0], hi = signal[0], mean = 0.0f;
    for (size_t i = 0; i < count; ++i) { if (signal[i] < lo) lo = signal[i]; if (signal[i] > hi) hi = signal[i]; mean += signal[i]; }
    mean /= (float)count;
    return fabsf(mean) > 1.0e-6f ? 100.0f * (hi - lo) / fabsf(mean) : NAN;
}

float pulseWidth50(const float *signal, int onset, int peak, int offset, float fs) {
    if (!signal || onset < 0 || peak <= onset || offset <= peak || fs <= 0.0f) return NAN;
    const float level = signal[onset] + 0.5f * (signal[peak] - signal[onset]);
    int left = onset, right = offset;
    for (int i = onset; i <= peak; ++i) if (signal[i] >= level) { left = i; break; }
    for (int i = peak; i <= offset; ++i) if (signal[i] <= level) { right = i; break; }
    return (right - left) / fs;
}

float areaBetween(const float *signal, int start, int end, float fs) {
    if (!signal || start < 0 || end <= start || fs <= 0.0f) return NAN;
    const float baseline = signal[start];
    float area = 0.0f;
    for (int i = start; i < end; ++i) area += 0.5f * ((signal[i] - baseline) + (signal[i + 1] - baseline)) / fs;
    return area;
}

float inflectionPointAreaRatio(float systolicArea, float diastolicArea) {
    return fabsf(diastolicArea) > 1.0e-6f ? systolicArea / diastolicArea : NAN;
}

int findU(const float *vpg, int onset, int peak) {
    if (!vpg || peak <= onset) return -1;
    int result = onset;
    for (int i = onset + 1; i <= peak; ++i) if (vpg[i] > vpg[result]) result = i;
    return result;
}

int findV(const float *vpg, int u, int offset) {
    if (!vpg || u < 0 || offset <= u) return -1;
    int result = u;
    for (int i = u + 1; i <= offset; ++i) if (vpg[i] < vpg[result]) result = i;
    return result;
}

int findW(const float *vpg, int v, int offset) {
    if (!vpg || v < 0 || offset - v < 2) return -1;
    for (int i = v + 1; i < offset; ++i) if (vpg[i] >= vpg[i - 1] && vpg[i] > vpg[i + 1]) return i;
    return v;
}

int findDownwardCrossing(const float *vpg, int start, int end, int requested) {
    if (!vpg || start < 0 || end <= start) return -1;
    int seen = 0;
    for (int i = start + 1; i <= end; ++i) if (vpg[i - 1] >= 0.0f && vpg[i] < 0.0f && seen++ == requested) return i;
    return -1;
}

float crestTime(int onset, int crossing, float fs) {
    return onset >= 0 && crossing > onset && fs > 0.0f ? (crossing - onset) / fs : NAN;
}

float deltaT(int firstCrossing, int secondCrossing, float fs) {
    return firstCrossing >= 0 && secondCrossing > firstCrossing && fs > 0.0f ? (secondCrossing - firstCrossing) / fs : NAN;
}

int apgExtremum(const float *apg, int start, int end, bool maximum) {
    if (!apg || start < 0 || end <= start) return -1;
    int result = start;
    for (int i = start + 1; i <= end; ++i) if ((maximum && apg[i] > apg[result]) || (!maximum && apg[i] < apg[result])) result = i;
    return result;
}

int findA(const float *apg, int onset, int peak) { return apgExtremum(apg, onset, peak, true); }
int findB(const float *apg, int a, int midpoint) { return apgExtremum(apg, a, midpoint, false); }
int findC(const float *apg, int b, int e) { return apgExtremum(apg, b, e, true); }
int findD(const float *apg, int c, int e) { return apgExtremum(apg, c, e, false); }
int findE(const float *apg, int peak, int offset) { return apgExtremum(apg, peak, offset, true); }
int findF(const float *apg, int e, int offset) { return apgExtremum(apg, e, offset, false); }

void sdptgRatios(const float *apg, const FiducialPoints &p, FeatureResults *r) {
    if (!apg || !r || p.a < 0 || fabsf(apg[p.a]) <= 1.0e-6f) return;
    if (p.b >= 0) r->bOverA = apg[p.b] / apg[p.a];
    if (p.c >= 0) r->cOverA = apg[p.c] / apg[p.a];
    if (p.d >= 0) r->dOverA = apg[p.d] / apg[p.a];
    if (p.e >= 0) r->eOverA = apg[p.e] / apg[p.a];
    if (p.b >= 0 && p.e >= 0) r->bMinusEOverA = (apg[p.b] - apg[p.e]) / apg[p.a];
    if (p.b >= 0 && p.c >= 0 && p.d >= 0) r->bMinusCMinusDOverA = (apg[p.b] - apg[p.c] - apg[p.d]) / apg[p.a];
    if (p.b >= 0 && p.c >= 0 && p.d >= 0 && p.e >= 0) r->agingIndex = (apg[p.b] - apg[p.c] - apg[p.d] - apg[p.e]) / apg[p.a];
}

float augmentationIndex(const float *signal, int reflection, int notch, int peak) {
    return signal && reflection >= 0 && notch >= 0 && peak >= 0 && fabsf(signal[peak]) > 1.0e-6f ? (signal[reflection] - signal[notch]) / signal[peak] : NAN;
}

float pulseTransitTime(int proximalIndex, int ppgOnset, float fs) {
    return proximalIndex >= 0 && ppgOnset >= proximalIndex && fs > 0.0f ? (ppgOnset - proximalIndex) / fs : NAN;
}

float largeArteryStiffness(float bodyHeightMeters, float dtSeconds) {
    const float height = bodyHeightMeters > 0.0f ? bodyHeightMeters : 1.7f;
    return dtSeconds > 1.0e-6f ? height / dtSeconds : NAN;
}

void analyze(const float *signal, const float *vpg, const float *apg, size_t count, float fs, FiducialPoints *p, FeatureResults *r) {
    if (!signal || !vpg || !apg || !p || !r || count < 8 || fs <= 0.0f) return;
    p->onset = 0; p->peak = 0; p->offset = (int)count - 1;
    for (size_t i = 1; i < count; ++i) if (signal[i] > signal[p->peak]) p->peak = (int)i;
    p->notch = findNotch(signal, p->peak, p->offset);
    p->u = findU(vpg, p->onset, p->peak); p->v = findV(vpg, p->u, p->offset); p->w = findW(vpg, p->v, p->offset);
    p->zero1 = findDownwardCrossing(vpg, p->onset, p->offset, 0); p->zero2 = findDownwardCrossing(vpg, p->onset, p->offset, 1);
    p->crestTimeSeconds = crestTime(p->onset, p->zero1, fs); p->deltaTSeconds = deltaT(p->zero1, p->zero2, fs);
    p->a = findA(apg, p->onset, p->peak); p->b = findB(apg, p->a, (p->peak + p->offset) / 2); p->e = findE(apg, p->peak, p->offset);
    p->c = findC(apg, p->b, p->e); p->d = findD(apg, p->c, p->e); p->f = findF(apg, p->e, p->offset);
    r->systolicAmplitude = systolicAmplitude(signal, p->onset, p->peak);
    r->pulseToPulseIntervalSeconds = count / fs; r->perfusionIndexPercent = perfusionIndex(signal, count);
    r->pulseWidth50Seconds = pulseWidth50(signal, p->onset, p->peak, p->offset, fs);
    r->pulseArea = areaBetween(signal, p->onset, p->offset, fs);
    r->systolicArea = areaBetween(signal, p->onset, p->notch, fs); r->diastolicArea = areaBetween(signal, p->notch, p->offset, fs);
    r->inflectionPointAreaRatio = inflectionPointAreaRatio(r->systolicArea, r->diastolicArea);
    r->augmentationIndex = augmentationIndex(signal, p->e, p->notch, p->peak);
    r->pulseTransitTimeSeconds = NAN; r->deltaTSeconds = p->deltaTSeconds;
    r->largeArteryStiffnessIndex = largeArteryStiffness(1.7f, p->deltaTSeconds);
    r->bOverA = r->cOverA = r->dOverA = r->eOverA = r->bMinusEOverA = r->bMinusCMinusDOverA = r->agingIndex = NAN;
    sdptgRatios(apg, *p, r);
}

} // namespace ppg

static void printFeature(const char *name, float value, const char *unit) {
    Serial.printf("%-38s %10.5f %s\n", name, (double)value, unit);
}

// ================= SETUP =================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Wire.begin(SDA_PIN, SCL_PIN);

    Serial.println("\n=================================");
    Serial.println("PPG BATCH PROCESSOR + ADXL345");
    Serial.println("=================================");

    // Initialize MAX30102
    Serial.println("Initializing MAX30102...");
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("ERROR: MAX30102 not found!");
        while (1);
    }
    Serial.println("MAX30102 found!");
    
    particleSensor.setup(
        60,     // LED brightness
        1,      // Sample averaging
        2,      // Red + IR
        100,    // Sample rate = 100 Hz
        411,    // Pulse width
        4096    // ADC range
    );

    // Initialize ADXL345
    Serial.println("Initializing ADXL345...");
    if (!accel.begin()) {
        Serial.println("ERROR: ADXL345 not found!");
        while (1);
    }
    Serial.println("ADXL345 found!");
    accel.setRange(ADXL345_RANGE_4_G);

    Serial.println("\nSystem Ready. Please place finger on sensor.");
}

// ================= LOOP =================
void loop() {
    unsigned long currentMillis = millis();

    // Sample exactly at 100 Hz to maintain dt integrity for the Savitzky-Golay filters
    if (currentMillis - lastSampleTime >= SAMPLE_INTERVAL_MS) {
        lastSampleTime = currentMillis;

        long irValue = particleSensor.getIR();

        // Check if finger is on sensor
        if (irValue > 50000) {
            
            // Store raw value in buffer
            rawPpg[bufferIndex] = (float)irValue;
            bufferIndex++;

            // Periodically log collection status so you know it hasn't frozen
            if (bufferIndex % 100 == 0) {
                Serial.printf("Collecting data... %d/%d\n", bufferIndex, PPG_MAX_SAMPLES);
            }

            // Once buffer is full, execute analysis pipeline
            if (bufferIndex >= PPG_MAX_SAMPLES) {
                
                // Get ADXL context at time of evaluation
                sensors_event_t event;
                accel.getEvent(&event);

                Serial.println("\n==================================================");
                Serial.println("           5-SECOND BATCH ANALYSIS REPORT         ");
                Serial.println("==================================================");

                // Run DSP pipeline
                ppg::FiducialPoints points = {};
                ppg::FeatureResults results = {};
                
                ppg::smoothSignal(rawPpg, smoothPpg, PPG_MAX_SAMPLES);
                ppg::firstDerivative(smoothPpg, vpg, PPG_MAX_SAMPLES, 1.0f / SAMPLE_RATE_HZ);
                ppg::secondDerivative(smoothPpg, apg, PPG_MAX_SAMPLES, 1.0f / SAMPLE_RATE_HZ);
                ppg::analyze(smoothPpg, vpg, apg, PPG_MAX_SAMPLES, SAMPLE_RATE_HZ, &points, &results);

                // Print outputs
                Serial.printf("ADXL345 Posture: X: %.2f | Y: %.2f | Z: %.2f\n\n", event.acceleration.x, event.acceleration.y, event.acceleration.z);
                
                printFeature("Systolic amplitude", results.systolicAmplitude, "a.u.");
                printFeature("Perfusion index", results.perfusionIndexPercent, "%");
                printFeature("Pulse width / PW50", results.pulseWidth50Seconds, "s");
                printFeature("Pulse area", results.pulseArea, "a.u.*s");
                printFeature("Systolic area", results.systolicArea, "a.u.*s");
                printFeature("Diastolic area", results.diastolicArea, "a.u.*s");
                printFeature("Inflection point area ratio", results.inflectionPointAreaRatio, "ratio");
                printFeature("PPG augmentation index", results.augmentationIndex, "ratio");
                printFeature("Crest time", points.crestTimeSeconds, "s");
                printFeature("Delta T", results.deltaTSeconds, "s");
                printFeature("Large artery stiffness index", results.largeArteryStiffnessIndex, "m/s");
                printFeature("b/a", results.bOverA, "ratio");
                printFeature("c/a", results.cOverA, "ratio");
                printFeature("d/a", results.dOverA, "ratio");
                printFeature("e/a", results.eOverA, "ratio");
                printFeature("(b-e)/a", results.bMinusEOverA, "ratio");
                printFeature("(b-c-d)/a", results.bMinusCMinusDOverA, "ratio");
                printFeature("(b-c-d-e)/a aging index", results.agingIndex, "ratio");
                
                Serial.println("==================================================\n");

                // Reset buffer to collect the next 5-second chunk
                bufferIndex = 0;
            }
        } 
        else {
            // If finger is removed, clear the partially filled buffer
            if (bufferIndex > 0) {
                Serial.println("Finger removed. Buffer cleared. Waiting for finger...");
                bufferIndex = 0;
            }
        }
    }
}

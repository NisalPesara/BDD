#include <Arduino.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

#ifndef PPG_MAX_SAMPLES
#define PPG_MAX_SAMPLES 512
#endif

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

// ==========================================
// TITLE: Signal Smoothing
// TECHNIQUE: Savitzky-Golay Convolution Filtering
// DESCRIPTION: Smooths the PPG using a fixed five-sample polynomial convolution.
// ==========================================
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

// ==========================================
// TITLE: First Derivative
// TECHNIQUE: Savitzky-Golay Differentiation
// DESCRIPTION: Calculates the velocity photoplethysmogram (VPG).
// ==========================================
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

// ==========================================
// TITLE: Second Derivative
// TECHNIQUE: Savitzky-Golay Differentiation
// DESCRIPTION: Calculates the acceleration photoplethysmogram (APG/SDPTG).
// ==========================================
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

// ==========================================
// TITLE: Systolic Peak Detection
// TECHNIQUE: Adaptive Thresholding and Rank Filtering
// DESCRIPTION: Detects local systolic maxima above an amplitude threshold with a refractory period.
// ==========================================
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

// ==========================================
// TITLE: Pulse Onset Detection
// TECHNIQUE: Local Minimum Boundary Isolation
// DESCRIPTION: Finds the minimum signal between two pulse peaks on the rising-side boundary.
// ==========================================
int findOnset(const float *signal, int previousPeak, int peak) {
    if (!signal || previousPeak < 0 || peak <= previousPeak) return -1;
    int result = previousPeak;
    for (int i = previousPeak + 1; i <= peak; ++i) if (signal[i] < signal[result]) result = i;
    return result;
}

// ==========================================
// TITLE: Pulse Offset Detection
// TECHNIQUE: Local Minimum Boundary Isolation
// DESCRIPTION: Finds the minimum signal between the systolic peak and the following peak.
// ==========================================
int findOffset(const float *signal, int peak, int nextPeak) {
    if (!signal || peak < 0 || nextPeak <= peak) return -1;
    int result = peak;
    for (int i = peak + 1; i <= nextPeak; ++i) if (signal[i] < signal[result]) result = i;
    return result;
}

// ==========================================
// TITLE: Dicrotic Notch Detection
// TECHNIQUE: Local Extrema Boundary Isolation
// DESCRIPTION: Finds the lowest post-systolic point before pulse offset.
// ==========================================
int findNotch(const float *signal, int peak, int offset) {
    if (!signal || peak < 0 || offset - peak < 2) return -1;
    int result = peak + 1;
    for (int i = peak + 2; i < offset; ++i) if (signal[i] < signal[result]) result = i;
    return result;
}

// ==========================================
// TITLE: Systolic Amplitude
// TECHNIQUE: Adaptive Thresholding and Rank Filtering
// DESCRIPTION: Calculates peak amplitude relative to pulse onset.
// ==========================================
float systolicAmplitude(const float *signal, int onset, int peak) {
    return signal && onset >= 0 && peak > onset ? signal[peak] - signal[onset] : NAN;
}

// ==========================================
// TITLE: Pulse-to-Pulse Interval
// TECHNIQUE: Peak Fiducial Timing
// DESCRIPTION: Converts consecutive systolic peak distance to cardiac-cycle seconds.
// ==========================================
float pulseInterval(int previousPeak, int peak, float fs) {
    return previousPeak >= 0 && peak > previousPeak && fs > 0.0f ? (peak - previousPeak) / fs : NAN;
}

// ==========================================
// TITLE: Perfusion Index
// TECHNIQUE: AC/DC Ratio Extraction
// DESCRIPTION: Computes peak-to-peak pulsatile AC amplitude divided by mean DC baseline, in percent.
// ==========================================
float perfusionIndex(const float *signal, size_t count) {
    if (!signal || count == 0) return NAN;
    float lo = signal[0], hi = signal[0], mean = 0.0f;
    for (size_t i = 0; i < count; ++i) { if (signal[i] < lo) lo = signal[i]; if (signal[i] > hi) hi = signal[i]; mean += signal[i]; }
    mean /= (float)count;
    return fabsf(mean) > 1.0e-6f ? 100.0f * (hi - lo) / fabsf(mean) : NAN;
}

// ==========================================
// TITLE: Pulse Width PW50
// TECHNIQUE: Full Width at Half Maximum
// DESCRIPTION: Measures the time between 50 percent amplitude crossings.
// ==========================================
float pulseWidth50(const float *signal, int onset, int peak, int offset, float fs) {
    if (!signal || onset < 0 || peak <= onset || offset <= peak || fs <= 0.0f) return NAN;
    const float level = signal[onset] + 0.5f * (signal[peak] - signal[onset]);
    int left = onset, right = offset;
    for (int i = onset; i <= peak; ++i) if (signal[i] >= level) { left = i; break; }
    for (int i = peak; i <= offset; ++i) if (signal[i] <= level) { right = i; break; }
    return (right - left) / fs;
}

// ==========================================
// TITLE: Pulse Area
// TECHNIQUE: Trapezoidal Numerical Integration
// DESCRIPTION: Integrates the baseline-corrected pulse waveform.
// ==========================================
float areaBetween(const float *signal, int start, int end, float fs) {
    if (!signal || start < 0 || end <= start || fs <= 0.0f) return NAN;
    const float baseline = signal[start];
    float area = 0.0f;
    for (int i = start; i < end; ++i) area += 0.5f * ((signal[i] - baseline) + (signal[i + 1] - baseline)) / fs;
    return area;
}

// ==========================================
// TITLE: Inflection Point Area Ratio
// TECHNIQUE: Systolic-to-Diastolic Area Ratio
// DESCRIPTION: Calculates systolic area divided by diastolic area split at the dicrotic notch.
// ==========================================
float inflectionPointAreaRatio(float systolicArea, float diastolicArea) {
    return fabsf(diastolicArea) > 1.0e-6f ? systolicArea / diastolicArea : NAN;
}

// ==========================================
// TITLE: VPG u Point
// TECHNIQUE: Segmented Maximum Bounding
// DESCRIPTION: Finds the maximum VPG value between onset and systolic peak.
// ==========================================
int findU(const float *vpg, int onset, int peak) {
    if (!vpg || peak <= onset) return -1;
    int result = onset;
    for (int i = onset + 1; i <= peak; ++i) if (vpg[i] > vpg[result]) result = i;
    return result;
}

// ==========================================
// TITLE: VPG v Point
// TECHNIQUE: Segmented Minimum Bounding
// DESCRIPTION: Finds the minimum VPG value between u and pulse offset.
// ==========================================
int findV(const float *vpg, int u, int offset) {
    if (!vpg || u < 0 || offset <= u) return -1;
    int result = u;
    for (int i = u + 1; i <= offset; ++i) if (vpg[i] < vpg[result]) result = i;
    return result;
}

// ==========================================
// TITLE: VPG w Point
// TECHNIQUE: Segmented Local Maximum Detection
// DESCRIPTION: Finds the first local maximum after the v point.
// ==========================================
int findW(const float *vpg, int v, int offset) {
    if (!vpg || v < 0 || offset - v < 2) return -1;
    for (int i = v + 1; i < offset; ++i) if (vpg[i] >= vpg[i - 1] && vpg[i] > vpg[i + 1]) return i;
    return v;
}

// ==========================================
// TITLE: VPG Downward Zero Crossing
// TECHNIQUE: Positive-to-Negative Zero-Crossing Detection
// DESCRIPTION: Finds the requested downward VPG zero crossing for crest time and Delta T.
// ==========================================
int findDownwardCrossing(const float *vpg, int start, int end, int requested) {
    if (!vpg || start < 0 || end <= start) return -1;
    int seen = 0;
    for (int i = start + 1; i <= end; ++i) if (vpg[i - 1] >= 0.0f && vpg[i] < 0.0f && seen++ == requested) return i;
    return -1;
}

// ==========================================
// TITLE: Crest Time
// TECHNIQUE: VPG Downward Zero-Crossing Timing
// DESCRIPTION: Measures onset-to-first-downward-VPG-crossing time.
// ==========================================
float crestTime(int onset, int crossing, float fs) {
    return onset >= 0 && crossing > onset && fs > 0.0f ? (crossing - onset) / fs : NAN;
}

// ==========================================
// TITLE: Delta T
// TECHNIQUE: VPG Downward Zero-Crossing Interval
// DESCRIPTION: Measures the interval between the first and second downward VPG zero crossings.
// ==========================================
float deltaT(int firstCrossing, int secondCrossing, float fs) {
    return firstCrossing >= 0 && secondCrossing > firstCrossing && fs > 0.0f ? (secondCrossing - firstCrossing) / fs : NAN;
}

// ==========================================
// TITLE: SDPTG Wave Detector
// TECHNIQUE: Local Extrema Boundary Isolation
// DESCRIPTION: Finds a maximum or minimum APG wave in a bounded interval.
// ==========================================
int apgExtremum(const float *apg, int start, int end, bool maximum) {
    if (!apg || start < 0 || end <= start) return -1;
    int result = start;
    for (int i = start + 1; i <= end; ++i) if ((maximum && apg[i] > apg[result]) || (!maximum && apg[i] < apg[result])) result = i;
    return result;
}

// ==========================================
// TITLE: SDPTG a Wave
// TECHNIQUE: Local Maximum Boundary Isolation
// DESCRIPTION: Finds the early systolic positive a wave.
// ==========================================
int findA(const float *apg, int onset, int peak) { return apgExtremum(apg, onset, peak, true); }

// ==========================================
// TITLE: SDPTG b Wave
// TECHNIQUE: Local Minimum Boundary Isolation
// DESCRIPTION: Finds the early systolic negative b wave.
// ==========================================
int findB(const float *apg, int a, int midpoint) { return apgExtremum(apg, a, midpoint, false); }

// ==========================================
// TITLE: SDPTG c Wave
// TECHNIQUE: Local Maximum Boundary Isolation
// DESCRIPTION: Finds the re-increasing c wave between b and e.
// ==========================================
int findC(const float *apg, int b, int e) { return apgExtremum(apg, b, e, true); }

// ==========================================
// TITLE: SDPTG d Wave
// TECHNIQUE: Local Minimum Boundary Isolation
// DESCRIPTION: Finds the re-decreasing d wave between c and e.
// ==========================================
int findD(const float *apg, int c, int e) { return apgExtremum(apg, c, e, false); }

// ==========================================
// TITLE: SDPTG e Wave
// TECHNIQUE: Local Maximum Boundary Isolation
// DESCRIPTION: Finds the early diastolic positive e wave.
// ==========================================
int findE(const float *apg, int peak, int offset) { return apgExtremum(apg, peak, offset, true); }

// ==========================================
// TITLE: SDPTG f Wave
// TECHNIQUE: Local Minimum Boundary Isolation
// DESCRIPTION: Finds the late negative f wave after e.
// ==========================================
int findF(const float *apg, int e, int offset) { return apgExtremum(apg, e, offset, false); }

// ==========================================
// TITLE: SDPTG Ratios and Aging Indices
// TECHNIQUE: Amplitude Ratio Calculation
// DESCRIPTION: Calculates b/a, c/a, d/a, e/a, (b-e)/a, (b-c-d)/a, and (b-c-d-e)/a.
// ==========================================
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

// ==========================================
// TITLE: PPG Augmentation Index
// TECHNIQUE: Reflection-Wave Amplitude Ratio
// DESCRIPTION: Calculates reflected-minus-notch amplitude normalized by systolic amplitude.
// ==========================================
float augmentationIndex(const float *signal, int reflection, int notch, int peak) {
    return signal && reflection >= 0 && notch >= 0 && peak >= 0 && fabsf(signal[peak]) > 1.0e-6f ? (signal[reflection] - signal[notch]) / signal[peak] : NAN;
}

// ==========================================
// TITLE: Pulse Transit Time
// TECHNIQUE: Proximal-to-Peripheral Fiducial Time Difference
// DESCRIPTION: Calculates synchronized proximal-channel event to PPG-onset delay.
// ==========================================
float pulseTransitTime(int proximalIndex, int ppgOnset, float fs) {
    return proximalIndex >= 0 && ppgOnset >= proximalIndex && fs > 0.0f ? (ppgOnset - proximalIndex) / fs : NAN;
}

// ==========================================
// TITLE: Large Artery Stiffness Index
// TECHNIQUE: Body-Height-to-Delta-T Ratio
// DESCRIPTION: Divides body height by Delta T, using 1.7 m if no height is supplied.
// ==========================================
float largeArteryStiffness(float bodyHeightMeters, float dtSeconds) {
    const float height = bodyHeightMeters > 0.0f ? bodyHeightMeters : 1.7f;
    return dtSeconds > 1.0e-6f ? height / dtSeconds : NAN;
}

// ==========================================
// TITLE: Complete Single-Beat Analysis
// TECHNIQUE: Fiducial Segmentation and Morphological Feature Extraction
// DESCRIPTION: Populates every requested clinical and derivative feature in one fixed-buffer pipeline.
// ==========================================
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

}  // namespace ppg

static float rawPpg[PPG_MAX_SAMPLES];
static float smoothPpg[PPG_MAX_SAMPLES];
static float vpg[PPG_MAX_SAMPLES];
static float apg[PPG_MAX_SAMPLES];

// ==========================================
// TITLE: Serial Feature Output
// TECHNIQUE: Embedded Diagnostic Reporting
// DESCRIPTION: Prints one named feature and its unit to the ESP32 serial monitor.
// ==========================================
static void printFeature(const char *name, float value, const char *unit) {
    Serial.printf("%-38s %10.5f %s\n", name, (double)value, unit);
}

// ==========================================
// TITLE: ESP32 Setup
// TECHNIQUE: Synthetic Biomedical Waveform Validation
// DESCRIPTION: Generates a test pulse, runs the complete analyzer, and reports all features.
// ==========================================
void setup() {
    const float fs = 100.0f;
    const size_t count = PPG_MAX_SAMPLES;
    Serial.begin(115200);
    delay(500);
    for (size_t i = 0; i < count; ++i) {
        const float phase = fmodf((float)i / fs, 0.85f);
        const float systolic = 1.10f * expf(-0.5f * powf((phase - 0.18f) / 0.075f, 2.0f));
        const float notch = -0.18f * expf(-0.5f * powf((phase - 0.43f) / 0.035f, 2.0f));
        const float reflection = 0.38f * expf(-0.5f * powf((phase - 0.56f) / 0.070f, 2.0f));
        rawPpg[i] = 1.0f + 0.01f * sinf(2.0f * PI * 0.2f * ((float)i / fs)) + systolic + notch + reflection;
    }
    ppg::FiducialPoints points = {};
    ppg::FeatureResults results = {};
    ppg::smoothSignal(rawPpg, smoothPpg, count);
    ppg::firstDerivative(smoothPpg, vpg, count, 1.0f / fs);
    ppg::secondDerivative(smoothPpg, apg, count, 1.0f / fs);
    ppg::analyze(smoothPpg, vpg, apg, count, fs, &points, &results);
    Serial.println("\nESP32 PPG Analyzer - single-file report");
    Serial.printf("Fiducials: onset=%d peak=%d notch=%d offset=%d\n", points.onset, points.peak, points.notch, points.offset);
    Serial.printf("VPG: u=%d v=%d w=%d zero1=%d zero2=%d\n", points.u, points.v, points.w, points.zero1, points.zero2);
    Serial.printf("APG: a=%d b=%d c=%d d=%d e=%d f=%d\n", points.a, points.b, points.c, points.d, points.e, points.f);
    printFeature("Systolic amplitude", results.systolicAmplitude, "a.u.");
    printFeature("Pulse-to-pulse interval", results.pulseToPulseIntervalSeconds, "s");
    printFeature("Perfusion index", results.perfusionIndexPercent, "%");
    printFeature("Pulse width / PW50", results.pulseWidth50Seconds, "s");
    printFeature("Pulse area", results.pulseArea, "a.u.*s");
    printFeature("Systolic area", results.systolicArea, "a.u.*s");
    printFeature("Diastolic area", results.diastolicArea, "a.u.*s");
    printFeature("Inflection point area ratio", results.inflectionPointAreaRatio, "ratio");
    printFeature("PPG augmentation index", results.augmentationIndex, "ratio");
    printFeature("Pulse transit time", results.pulseTransitTimeSeconds, "s");
    printFeature("Crest time", points.crestTimeSeconds, "s");
    printFeature("Delta T", results.deltaTSeconds, "s");
    printFeature("Large artery stiffness index", results.largeArteryStiffnessIndex, "m/s");
    printFeature("b/a", results.bOverA, "ratio"); printFeature("c/a", results.cOverA, "ratio");
    printFeature("d/a", results.dOverA, "ratio"); printFeature("e/a", results.eOverA, "ratio");
    printFeature("(b-e)/a", results.bMinusEOverA, "ratio"); printFeature("(b-c-d)/a", results.bMinusCMinusDOverA, "ratio");
    printFeature("(b-c-d-e)/a aging index", results.agingIndex, "ratio");
    Serial.println("PTT is NAN until a synchronized proximal channel is supplied.");
}

// ==========================================
// TITLE: ESP32 Main Loop
// TECHNIQUE: Periodic Embedded Execution
// DESCRIPTION: Holds the completed serial report without allocating memory.
// ==========================================
void loop() {
    delay(1000);
}

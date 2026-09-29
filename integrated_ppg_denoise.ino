#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

// ESP32-S3 + MAX30102 implementation based on:
// https://github.com/rapzyftminji/ppg-hrv-analysis

#define SDA_PIN 22
#define SCL_PIN 23
#define SAMPLE_RATE_HZ 100
#define PPG_MAX_SAMPLES 512
#define DOWNSAMPLE_FACTOR 1
#define STREAM_CLEAN_SIGNAL 1
#define FINGER_THRESHOLD 50000UL

#if DOWNSAMPLE_FACTOR < 1
#error DOWNSAMPLE_FACTOR must be positive
#endif

static const size_t DOWNSAMPLED_CAPACITY = PPG_MAX_SAMPLES / DOWNSAMPLE_FACTOR;
static const size_t DWT_CAPACITY = DOWNSAMPLED_CAPACITY + 127;
static const size_t Q_MAX_TAPS = 510;

MAX30105 particleSensor;

static float rawPpg[PPG_MAX_SAMPLES];
static float normalizedPpg[PPG_MAX_SAMPLES];
static float preProcessedPpg[PPG_MAX_SAMPLES];
static float filterForward[PPG_MAX_SAMPLES];
static float filterReverseInput[PPG_MAX_SAMPLES];
static float filterReverseOutput[PPG_MAX_SAMPLES];
static float downsampleSource[PPG_MAX_SAMPLES];
static float downsampledPpg[DOWNSAMPLED_CAPACITY];
static float dwtSignals[8][DWT_CAPACITY];
static int16_t qCoefficients[8][Q_MAX_TAPS];
static uint16_t qLengths[8];
static int maxima[DWT_CAPACITY];
static int minima[DWT_CAPACITY];
static int peakCandidates[DWT_CAPACITY];
static int detectedPeaks[DWT_CAPACITY];
static size_t sampleCount = 0;

static void initializeQCoefficients() {
  memset(qCoefficients, 0, sizeof(qCoefficients));
  qCoefficients[0][0] = -2;
  qCoefficients[0][1] = 2;
  qLengths[0] = 2;

  static const int16_t binomial[5] = {1, 4, 6, 4, 1};
  for (int level = 2; level <= 8; ++level) {
    const size_t row = (size_t)(level - 1);
    const size_t previousRow = row - 1;
    const size_t dilation = (size_t)1 << (level - 2);
    const size_t previousLength = qLengths[previousRow];
    const size_t length = previousLength + 4 * dilation;

    for (size_t i = 0; i < previousLength; ++i) {
      for (size_t tap = 0; tap < 5; ++tap) {
        int32_t value = (int32_t)qCoefficients[previousRow][i] * binomial[tap];
        if (level == 2) value /= 2;
        qCoefficients[row][i + tap * dilation] += (int16_t)value;
      }
    }
    qLengths[row] = (uint16_t)length;
  }
}

static void applyIir(const float *input, float *output, size_t count,
                     float b0, float b1, float b2, float a1, float a2) {
  float x1 = 0.0f, x2 = 0.0f, y1 = 0.0f, y2 = 0.0f;
  for (size_t i = 0; i < count; ++i) {
    const float x0 = input[i];
    const float y0 = b0 * x0 + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    output[i] = y0;
    x2 = x1;
    x1 = x0;
    y2 = y1;
    y1 = y0;
  }
}

static bool conditionAndDecompose(size_t inputLength, float effectiveFs,
                                  int *selectedLevel, float *selectedPower) {
  if (inputLength == 0 || inputLength > PPG_MAX_SAMPLES || effectiveFs <= 0.0f) {
    return false;
  }

  double mean = 0.0;
  for (size_t i = 0; i < inputLength; ++i) mean += rawPpg[i];
  mean /= inputLength;

  double variance = 0.0;
  for (size_t i = 0; i < inputLength; ++i) {
    const double centered = rawPpg[i] - mean;
    variance += centered * centered;
  }
  double standardDeviation = sqrt(variance / inputLength);
  if (standardDeviation == 0.0) standardDeviation = 1.0;

  for (size_t i = 0; i < inputLength; ++i) {
    normalizedPpg[i] = (float)((rawPpg[i] - mean) / standardDeviation);
  }

  const float hpB[4] = {0.99598683f, -2.98796050f, 2.98796050f, -0.99598683f};
  const float hpA[4] = {0.0f, -2.99195753f, 2.98394736f, -0.99198977f};
  for (size_t i = 0; i < inputLength; ++i) {
    float value = hpB[0] * normalizedPpg[i];
    if (i >= 1) value += hpB[1] * normalizedPpg[i - 1] - hpA[1] * preProcessedPpg[i - 1];
    if (i >= 2) value += hpB[2] * normalizedPpg[i - 2] - hpA[2] * preProcessedPpg[i - 2];
    if (i >= 3) value += hpB[3] * normalizedPpg[i - 3] - hpA[3] * preProcessedPpg[i - 3];
    preProcessedPpg[i] = value;
  }

  if (DOWNSAMPLE_FACTOR > 1) {
    const float cutoff = effectiveFs * 0.5f;
    const float k = tanf(PI * cutoff / SAMPLE_RATE_HZ);
    const float norm = k * k + 1.41421356237f * k + 1.0f;
    const float b0 = k * k / norm;
    const float b1 = 2.0f * k * k / norm;
    const float a1 = 2.0f * (k * k - 1.0f) / norm;
    const float a2 = (k * k - 1.41421356237f * k + 1.0f) / norm;

    applyIir(preProcessedPpg, filterForward, inputLength,
             b0, b1, b0, a1, a2);
    for (size_t i = 0; i < inputLength; ++i) {
      filterReverseInput[i] = filterForward[inputLength - 1 - i];
    }
    applyIir(filterReverseInput, filterReverseOutput, inputLength,
             b0, b1, b0, a1, a2);
    for (size_t i = 0; i < inputLength; ++i) {
      downsampleSource[i] = filterReverseOutput[inputLength - 1 - i];
    }
  } else {
    memcpy(downsampleSource, preProcessedPpg, inputLength * sizeof(float));
  }

  const size_t downsampledLength = inputLength / DOWNSAMPLE_FACTOR;
  for (size_t block = 0; block < downsampledLength; ++block) {
    float sum = 0.0f;
    const size_t start = block * DOWNSAMPLE_FACTOR;
    for (size_t offset = 0; offset < DOWNSAMPLE_FACTOR; ++offset) {
      sum += downsampleSource[start + offset];
    }
    downsampledPpg[block] = sum / DOWNSAMPLE_FACTOR;
  }

  memset(dwtSignals, 0, sizeof(dwtSignals));
  const size_t outputLength = downsampledLength + 127;
  for (int level = 1; level <= 8; ++level) {
    const size_t row = (size_t)(level - 1);
    const size_t delay = ((size_t)1 << (level - 1)) - 1;
    const float scale = level == 1 ? 1.0f : ldexpf(1.0f, 3 * level - 4);
    for (size_t n = 0; n < downsampledLength; ++n) {
      for (size_t tap = 0; tap < qLengths[row] && tap <= n; ++tap) {
        dwtSignals[row][n + delay] +=
            ((float)qCoefficients[row][tap] / scale) * downsampledPpg[n - tap];
      }
    }
  }

  bool foundBandBins = false;
  double bestPower = -1.0;
  int bestLevel = 0;
  for (int level = 1; level <= 8; ++level) {
    double bandPower = 0.0;
    for (size_t bin = 0; bin < outputLength / 2; ++bin) {
      const double frequency = (double)bin * effectiveFs / outputLength;
      if (frequency < 0.8 || frequency > 3.0) continue;
      foundBandBins = true;

      const double omega = 2.0 * PI * bin / outputLength;
      const double coefficient = 2.0 * cos(omega);
      double state1 = 0.0, state2 = 0.0;
      for (size_t i = 0; i < outputLength; ++i) {
        const double state0 = dwtSignals[level - 1][i] + coefficient * state1 - state2;
        state2 = state1;
        state1 = state0;
      }
      double power = state1 * state1 + state2 * state2 - coefficient * state1 * state2;
      if (power < 0.0) power = 0.0;
      bandPower += power;
    }
    if (bandPower > bestPower) {
      bestPower = bandPower;
      bestLevel = level;
    }
  }

  if (!foundBandBins || bestLevel == 0) return false;
  *selectedLevel = bestLevel;
  *selectedPower = (float)bestPower;
  return true;
}

static size_t findExtremePoints(const float *signal, size_t length, float fs,
                                int *outMaxima, int *outMinima,
                                size_t *maximaCountOut, size_t *minimaCountOut) {
  int radius = (int)lroundf(0.15f * fs);
  if (radius < 1) radius = 1;
  size_t maximaCount = 0, minimaCount = 0;
  int i = 1;

  while (i <= (int)length - radius - 1) {
    bool isPeak = true;
    bool isValley = true;
    for (int j = 1; j <= radius; ++j) {
      if (signal[i] <= signal[i + j] || signal[i] <= signal[i - j]) {
        isPeak = false;
        break;
      }
    }
    if (!isPeak) {
      for (int j = 1; j <= radius; ++j) {
        if (signal[i] >= signal[i + j] || signal[i] >= signal[i - j]) {
          isValley = false;
          break;
        }
      }
    }

    if (isPeak) {
      outMaxima[maximaCount++] = i;
      i += radius;
    } else if (isValley) {
      outMinima[minimaCount++] = i;
      i += radius;
    } else {
      ++i;
    }
  }
  *maximaCountOut = maximaCount;
  *minimaCountOut = minimaCount;
  return maximaCount;
}

static size_t detectPeaks(const float *signal, size_t length, float fs,
                          size_t *maximaCountOut, size_t *minimaCountOut) {
  const size_t maximaCount = findExtremePoints(signal, length, fs, maxima, minima,
                                                maximaCountOut, minimaCountOut);
  const int windowLength = (int)lroundf(0.250f * fs);
  const int minDistance = (int)lroundf(0.450f * fs);
  if (windowLength <= 0 || length == 0) return 0;

  const size_t firstWindowLength = min(length, (size_t)lroundf(2.0f * fs));
  float initialMaximum = signal[0];
  for (size_t i = 1; i < firstWindowLength; ++i) {
    if (signal[i] > initialMaximum) initialMaximum = signal[i];
  }
  float threshold1 = 0.1f * initialMaximum;
  float threshold2 = threshold1;
  float threshold3 = threshold1;
  int lastAccepted = -2 * minDistance;
  size_t candidateCount = 0;

  for (int windowStart = 0; windowStart < (int)length; windowStart += windowLength) {
    const int windowEnd = windowStart + windowLength;
    size_t qualifyingCount = 0;
    float maximum = -INFINITY;
    int maximumPosition = -1;

    for (size_t index = 0; index < maximaCount; ++index) {
      const int position = maxima[index];
      if (position < windowStart || position >= windowEnd) continue;
      const float interpolatedThreshold = threshold2 -
          (threshold2 - threshold3) / windowLength * (position - windowStart);
      if (signal[position] > 0.5f * interpolatedThreshold) {
        ++qualifyingCount;
        if (signal[position] > maximum) {
          maximum = signal[position];
          maximumPosition = position;
        }
      }
    }

    if (qualifyingCount > 0) {
      if (maximumPosition - lastAccepted > minDistance) {
        peakCandidates[candidateCount++] = maximumPosition;
        lastAccepted = maximumPosition;
      }
      const float nextThreshold3 = maximum - 0.5f * (maximum - 0.003f) -
                                   0.1f * (maximum - threshold1);
      threshold1 = threshold2;
      threshold2 = maximum;
      threshold3 = nextThreshold3;
    } else {
      const float nextThreshold3 = threshold3 - 0.5f * (threshold3 - 0.003f) -
                                   0.1f * (threshold3 - threshold2);
      threshold1 = threshold2;
      threshold2 = threshold3;
      threshold3 = nextThreshold3;
    }
  }

  const int searchRadius = (int)lroundf(0.200f * fs);
  size_t foundCount = 0;
  for (size_t i = 0; i < candidateCount; ++i) {
    const int start = max(0, peakCandidates[i] - searchRadius);
    const int end = min((int)length, peakCandidates[i] + searchRadius);
    if (start >= end) continue;
    int actualPeak = start;
    for (int j = start + 1; j < end; ++j) {
      if (signal[j] > signal[actualPeak]) actualPeak = j;
    }
    if (foundCount == 0 || actualPeak != detectedPeaks[foundCount - 1]) {
      detectedPeaks[foundCount++] = actualPeak;
    }
  }
  return foundCount;
}

static void processBatch() {
  const float effectiveFs = (float)SAMPLE_RATE_HZ / DOWNSAMPLE_FACTOR;
  int selectedLevel = 0;
  float selectedPower = 0.0f;
  if (!conditionAndDecompose(sampleCount, effectiveFs, &selectedLevel, &selectedPower)) {
    Serial.println("ERROR: No FFT bins in the 0.8-3.0 Hz cardiac band.");
    return;
  }

  const size_t outputLength = sampleCount / DOWNSAMPLE_FACTOR + 127;
  const int delaySamples = ((int)1 << (selectedLevel - 1)) - 1;
  const float *cleanSignal = dwtSignals[selectedLevel - 1];
  size_t maximaCount = 0, minimaCount = 0;
  const size_t peakCount = detectPeaks(cleanSignal, outputLength, effectiveFs,
                                       &maximaCount, &minimaCount);

  double rrTotalMs = 0.0;
  for (size_t i = 1; i < peakCount; ++i) {
    rrTotalMs += (detectedPeaks[i] - detectedPeaks[i - 1]) * 1000.0 / effectiveFs;
  }
  const size_t rrCount = peakCount > 1 ? peakCount - 1 : 0;
  const double meanHr = rrCount > 0 ? 60000.0 * rrCount / rrTotalMs : 0.0;

  Serial.printf("# level=j%d,effective_fs=%.2f,delay_samples=%d,band_power=%.6f\n",
                selectedLevel, effectiveFs, delaySamples, selectedPower);
  Serial.printf("# peaks=%u,mean_hr_bpm=%.2f\n",
                (unsigned)peakCount, meanHr);

#if STREAM_CLEAN_SIGNAL
  Serial.println("time_s,clean_signal");
  for (size_t i = 0; i < outputLength; ++i) {
    const float timeSeconds = i / effectiveFs + delaySamples / effectiveFs;
    Serial.printf("%.4f,%.7f\n", timeSeconds, cleanSignal[i]);
  }
#endif

  Serial.println("rr_index,rr_interval_ms");
  for (size_t i = 1; i < peakCount; ++i) {
    const float intervalMs = (detectedPeaks[i] - detectedPeaks[i - 1]) * 1000.0f / effectiveFs;
    Serial.printf("%u,%.2f\n", (unsigned)i, intervalMs);
  }
  if (peakCount < 2) {
    Serial.println("WARNING: Fewer than two peaks; no RR interval is available.");
  }
  if (selectedPower <= 1.0e-12f) {
    Serial.println("WARNING: Selected cardiac-band power is effectively zero.");
  }
}

void setup() {
  Serial.begin(921600);
  delay(500);
  Wire.begin(SDA_PIN, SCL_PIN);
  initializeQCoefficients();

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("ERROR: MAX30102 not found. Check wiring and power.");
    while (true) delay(100);
  }

  particleSensor.setup(60, 1, 2, SAMPLE_RATE_HZ, 411, 4096);
  Serial.println("ESP32-S3 PPG denoiser ready; place a finger on the MAX30102.");
}

void loop() {
  particleSensor.check();
  while (particleSensor.available()) {
    const uint32_t irValue = particleSensor.getFIFOIR();
    particleSensor.nextSample();

    if (irValue < FINGER_THRESHOLD) {
      if (sampleCount > 0) {
        Serial.println("Finger removed; discarding incomplete batch.");
        sampleCount = 0;
      }
      continue;
    }

    rawPpg[sampleCount++] = (float)irValue;
    if (sampleCount >= PPG_MAX_SAMPLES) {
      processBatch();
      sampleCount = 0;
    }
  }
}
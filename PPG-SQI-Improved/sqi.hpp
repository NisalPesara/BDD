#pragma once

#include <cmath>

struct SqiOptions {
  int hopSamples = 1;
  float minPeriodicity = 0.50f;
  float minSkew = 0.0f;
  float minKurtosis = 1.0f;
  float maxKurtosis = 10.0f;
  float minEntropy = 0.0f;
  float maxEntropy = 4.0f;
  float minPiPercent = 0.0f;
  float maxPiPercent = 100.0f;
};

struct SqiResult {
  bool fresh = false;
  bool good = false;
  float periodicity = 0.0f;
  float skew = 0.0f;
  float kurtosis = 0.0f;
  float entropy = 0.0f;
  float piPercent = 0.0f;
};

template <int WindowSamples, int SampleRateHz>
class SqiWindow {
  static_assert(WindowSamples >= 16, "WindowSamples must be >= 16");
  static_assert(SampleRateHz > 0, "SampleRateHz must be positive");

  static constexpr int kBins = 16;

 public:
  explicit SqiWindow(const SqiOptions& options = SqiOptions())
      : options_(options) {
    begin();
  }

  void begin() {
    index_ = 0;
    count_ = 0;
    samplesSinceResult_ = 0;
    hasResult_ = false;
    result_ = SqiResult();
    for (int i = 0; i < WindowSamples; ++i) {
      signal_[i] = 0.0f;
      dc_[i] = 0.0f;
    }
  }

  SqiResult push(float sample, float dc) {
    signal_[index_] = sample;
    dc_[index_] = dc;
    if (++index_ == WindowSamples) index_ = 0;
    if (count_ < WindowSamples) ++count_;

    result_.fresh = false;
    if (count_ == WindowSamples) {
      if (!hasResult_) {
        compute();
        hasResult_ = true;
        samplesSinceResult_ = 0;
      } else if (++samplesSinceResult_ >= normalizedHop()) {
        compute();
        samplesSinceResult_ = 0;
      }
    }
    return result_;
  }

 private:
  float signal_[WindowSamples];
  float dc_[WindowSamples];
  float ordered_[WindowSamples];
  float prefixEnergy_[WindowSamples + 1];
  SqiOptions options_;
  SqiResult result_;
  int index_ = 0;
  int count_ = 0;
  int samplesSinceResult_ = 0;
  bool hasResult_ = false;

  int normalizedHop() const {
    return options_.hopSamples > 0 ? options_.hopSamples : 1;
  }

  void invalidate() {
    result_ = SqiResult();
    result_.fresh = true;
  }

  void compute() {
    result_ = SqiResult();
    result_.fresh = true;

    float minimum = 0.0f;
    float maximum = 0.0f;
    float dcMean = 0.0f;
    float mean = 0.0f;
    for (int i = 0; i < WindowSamples; ++i) {
      const int source = (index_ + i) % WindowSamples;
      const float value = signal_[source];
      ordered_[i] = value;
      mean += value;
      dcMean += dc_[source];
      if (i == 0 || value < minimum) minimum = value;
      if (i == 0 || value > maximum) maximum = value;
    }
    mean /= WindowSamples;
    dcMean /= WindowSamples;

    float m2 = 0.0f;
    float m3 = 0.0f;
    float m4 = 0.0f;
    for (int i = 0; i < WindowSamples; ++i) {
      const float delta = ordered_[i] - mean;
      const float delta2 = delta * delta;
      m2 += delta2;
      m3 += delta2 * delta;
      m4 += delta2 * delta2;
    }
    m2 /= WindowSamples;
    m3 /= WindowSamples;
    m4 /= WindowSamples;
    if (!(m2 > 1.0e-18f) || !std::isfinite(m2)) {
      invalidate();
      return;
    }

    const float standardDeviation = std::sqrt(m2);
    result_.skew = m3 / (standardDeviation * standardDeviation * standardDeviation);
    result_.kurtosis = m4 / (m2 * m2);

    int histogram[kBins] = {};
    float range = maximum - minimum;
    if (range < 1.0e-12f) range = 1.0e-12f;
    for (int i = 0; i < WindowSamples; ++i) {
      int bin = static_cast<int>((ordered_[i] - minimum) / range * kBins);
      if (bin >= kBins) bin = kBins - 1;
      if (bin < 0) bin = 0;
      ++histogram[bin];
    }
    for (int bin = 0; bin < kBins; ++bin) {
      if (histogram[bin] != 0) {
        const float probability =
            static_cast<float>(histogram[bin]) / WindowSamples;
        result_.entropy -= probability * std::log2(probability);
      }
    }

    result_.piPercent =
        std::fabs(dcMean) > 1.0e-9f ? (range / std::fabs(dcMean)) * 100.0f : 0.0f;

    for (int i = 0; i < WindowSamples; ++i) {
      ordered_[i] -= mean;
      prefixEnergy_[i + 1] =
          prefixEnergy_[i] + ordered_[i] * ordered_[i];
    }

    result_.periodicity = firstPeakPeriodicity();
    result_.good =
        std::isfinite(result_.periodicity) && std::isfinite(result_.skew) &&
        std::isfinite(result_.kurtosis) && std::isfinite(result_.entropy) &&
        std::isfinite(result_.piPercent) &&
        result_.periodicity >= options_.minPeriodicity &&
        result_.skew >= options_.minSkew &&
        result_.kurtosis >= options_.minKurtosis &&
        result_.kurtosis <= options_.maxKurtosis &&
        result_.entropy >= options_.minEntropy &&
        result_.entropy <= options_.maxEntropy &&
        result_.piPercent >= options_.minPiPercent &&
        result_.piPercent <= options_.maxPiPercent;
  }

  float correlationAt(int lag) const {
    const int overlap = WindowSamples - lag;
    float cross = 0.0f;
    for (int i = 0; i < overlap; ++i) {
      cross += ordered_[i] * ordered_[i + lag];
    }
    const float leftEnergy = prefixEnergy_[overlap];
    const float rightEnergy =
        prefixEnergy_[WindowSamples] - prefixEnergy_[lag];
    const float denominator = std::sqrt(leftEnergy * rightEnergy);
    return denominator > 1.0e-12f ? cross / denominator : 0.0f;
  }

  float firstPeakPeriodicity() const {
    int minLag = (SampleRateHz + 2) / 3;  // 180 bpm
    int maxLag = (3 * SampleRateHz) / 2;  // 40 bpm
    if (minLag < 1) minLag = 1;
    if (maxLag >= WindowSamples / 2) maxLag = WindowSamples / 2 - 1;
    if (minLag > maxLag) return 0.0f;

    bool crossedZero = false;
    float previousPrevious = correlationAt(minLag - 1);
    float previous = correlationAt(minLag);
    if (previous <= 0.0f) crossedZero = true;

    for (int lag = minLag + 1; lag <= maxLag; ++lag) {
      const float current = correlationAt(lag);
      if (previous <= 0.0f) crossedZero = true;
      if (crossedZero && previous > 0.0f &&
          previous >= previousPrevious && previous >= current) {
        return previous;
      }
      previousPrevious = previous;
      previous = current;
    }
    return 0.0f;
  }
};

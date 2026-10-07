#include "sqi.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

namespace {

constexpr int kSampleRate = 100;
constexpr int kWindow = 800;
constexpr float kPi = 3.14159265358979323846f;

float pulseSample(int n, float amplitude) {
  const float phase = std::fmod(static_cast<float>(n) / kSampleRate, 0.8f);
  const float distance = phase < 0.12f ? phase : phase - 0.8f;
  const float pulse = std::exp(-(distance * distance) / (2.0f * 0.035f * 0.035f));
  return amplitude * pulse;
}

float noiseSample(unsigned& state) {
  state = state * 1664525u + 1013904223u;
  return (static_cast<float>((state >> 8) & 0xffffu) / 32767.5f) - 1.0f;
}

template <typename Generator>
SqiResult runWindow(SqiWindow<kWindow, kSampleRate>& sqi,
                    Generator generator,
                    float dc = 1000.0f) {
  SqiResult result;
  for (int n = 0; n < kWindow; ++n) {
    result = sqi.push(generator(n), dc);
  }
  assert(result.fresh);
  return result;
}

}  // namespace

int main() {
  SqiOptions options;
  options.hopSamples = 200;
  options.minPeriodicity = 0.55f;
  options.minSkew = 0.2f;
  options.minKurtosis = 1.0f;
  options.maxKurtosis = 20.0f;
  options.minEntropy = 0.5f;
  options.maxEntropy = 4.0f;
  options.minPiPercent = 0.5f;
  options.maxPiPercent = 10.0f;
  SqiWindow<kWindow, kSampleRate> sqi(options);

  const SqiResult clean = runWindow(sqi, [](int n) {
    return pulseSample(n, 10.0f);
  });
  assert(clean.good);
  assert(clean.periodicity >= options.minPeriodicity);
  assert(clean.piPercent >= options.minPiPercent);
  std::printf("clean: periodicity=%.3f skew=%.3f kurtosis=%.3f entropy=%.3f PI=%.3f%%\n",
              clean.periodicity, clean.skew, clean.kurtosis, clean.entropy,
              clean.piPercent);

  int samplesToNext = 0;
  for (int n = kWindow; n < kWindow + options.hopSamples; ++n) {
    if (sqi.push(pulseSample(n, 10.0f), 1000.0f).fresh) ++samplesToNext;
  }
  assert(samplesToNext == 1);

  sqi.begin();
  unsigned randomState = 7;
  const SqiResult noisy = runWindow(sqi, [&randomState](int) {
    return noiseSample(randomState);
  });
  assert(!noisy.good);
  assert(noisy.periodicity < options.minPeriodicity);
  std::printf("noise: periodicity=%.3f good=%d\n", noisy.periodicity,
              noisy.good ? 1 : 0);

  sqi.begin();
  const SqiResult flat = runWindow(sqi, [](int) { return 0.0f; });
  assert(!flat.good);
  assert(flat.fresh);
  assert(flat.periodicity == 0.0f);
  std::printf("flat: rejected\n");

  sqi.begin();
  const SqiResult weakPerfusion = runWindow(
      sqi, [](int n) { return pulseSample(n, 1.0f); }, 1000.0f);
  assert(weakPerfusion.piPercent < options.minPiPercent);
  assert(!weakPerfusion.good);
  std::printf("low PI: PI=%.3f%% good=%d\n", weakPerfusion.piPercent,
              weakPerfusion.good ? 1 : 0);

  sqi.begin();
  const SqiResult clipped = runWindow(sqi, [](int n) {
    const float pulse = pulseSample(n, 10.0f);
    return pulse > 4.0f ? 4.0f : pulse;
  });
  assert(clipped.entropy < clean.entropy);
  std::printf("clipped: entropy=%.3f (clean %.3f)\n", clipped.entropy,
              clean.entropy);

  std::puts("All SQI tests passed.");
  return 0;
}

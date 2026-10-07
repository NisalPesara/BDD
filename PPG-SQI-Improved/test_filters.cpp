#include "ppg_biquad.h"
#include "ppg_dc_removal.h"
#include "ppg_moving_average.h"
#include "ppg_nlms_anc.h"

#include <cassert>
#include <cmath>
#include <cstdio>

namespace {

constexpr float kPi = 3.14159265358979323846f;

float rmsSineThroughLowPass(float frequencyHz) {
  PpgButterworth filter;
  assert(filter.configureLowPass(125.0f, 8.0f, 4));
  float squareSum = 0.0f;
  for (int n = 0; n < 5000; ++n) {
    const float input =
        std::sin(2.0f * kPi * frequencyHz * n / 125.0f);
    const float output = filter.process(input);
    if (n >= 1000) squareSum += output * output;
  }
  return std::sqrt(squareSum / 4000.0f);
}

unsigned nextRandom(unsigned& state) {
  state = state * 1664525u + 1013904223u;
  return state;
}

}  // namespace

int main() {
  PpgBiquad biquad;
  assert(biquad.configureLowPass(125.0f, 8.0f));
  assert(!biquad.configureLowPass(125.0f, 62.5f));
  assert(!biquad.configureHighPass(125.0f, 0.0f));
  const float lowFrequencyRms = rmsSineThroughLowPass(2.0f);
  const float highFrequencyRms = rmsSineThroughLowPass(30.0f);
  assert(lowFrequencyRms > 0.65f);
  assert(highFrequencyRms < lowFrequencyRms * 0.1f);

  PpgButterworth bandPass;
  assert(bandPass.configureBandPass(125.0f, 0.2f, 8.0f));
  float dcOutput = 0.0f;
  for (int n = 0; n < 1000; ++n) dcOutput = bandPass.process(1.0f);
  assert(std::fabs(dcOutput) < 1.0e-3f);

  PpgDcRemoval dcRemoval;
  assert(dcRemoval.configure(125.0f, 1.0f));
  assert(!dcRemoval.configure(125.0f, 0.0f));
  assert(std::fabs(dcRemoval.process(1200.0f)) < 1.0e-6f);
  float dcResidual = 0.0f;
  for (int n = 0; n < 2000; ++n) dcResidual = dcRemoval.process(1200.0f);
  assert(std::fabs(dcResidual) < 1.0e-3f);

  PpgMovingAverage<4> smoother;
  assert(std::fabs(smoother.process(1.0f) - 1.0f) < 1.0e-6f);
  assert(std::fabs(smoother.process(3.0f) - 2.0f) < 1.0e-6f);
  assert(std::fabs(smoother.process(5.0f) - 3.0f) < 1.0e-6f);
  assert(std::fabs(smoother.process(7.0f) - 4.0f) < 1.0e-6f);
  assert(std::fabs(smoother.process(9.0f) - 6.0f) < 1.0e-6f);
  smoother.reset();
  assert(std::fabs(smoother.process(8.0f) - 8.0f) < 1.0e-6f);

  PpgNlmsAnc<1, 8> anc;
  assert(anc.configure(0.1f, 0.01f));
  assert(!anc.configure(2.0f));
  unsigned randomState = 11u;
  float residualSquareSum = 0.0f;
  int residualCount = 0;
  for (int n = 0; n < 30000; ++n) {
    const float reference =
        (static_cast<float>((nextRandom(randomState) >> 8) & 0xffffu) /
         32767.5f) -
        1.0f;
    const float wantedPulse = 0.5f * std::sin(2.0f * kPi * 1.2f * n / 125.0f);
    const float contaminated = wantedPulse + 0.8f * reference;
    const float references[1] = {reference};
    const float cleaned = anc.process(contaminated, references);
    if (n >= 10000) {
      const float residual = cleaned - wantedPulse;
      residualSquareSum += residual * residual;
      ++residualCount;
    }
  }
  const float ancResidualRms =
      std::sqrt(residualSquareSum / residualCount);
  assert(ancResidualRms < 0.12f);

  std::printf("Low-pass RMS: 2 Hz=%.3f, 30 Hz=%.3f\n", lowFrequencyRms,
              highFrequencyRms);
  std::printf("DC step residual after settling: %.6f\n", dcResidual);
  std::printf("NLMS residual RMS after convergence: %.3f\n", ancResidualRms);
  std::puts("All filter tests passed.");
  return 0;
}

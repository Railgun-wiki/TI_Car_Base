#include "Middlewares/encoder_speed.hpp"

namespace {
constexpr float kPi = 3.14159265F;
}
namespace middleware {
WheelSpeed EncoderSpeedEstimator::update(car::EncoderTicks ticks,
                                         std::uint32_t timestampMs) noexcept {
  if (!seeded_) {
    previous_ = ticks;
    previousMs_ = timestampMs;
    seeded_ = true;
    return {0.0F, 0.0F, 0U};
  }
  const std::uint32_t dt = timestampMs - previousMs_;
  if (dt == 0U)
    return {0.0F, 0.0F, 0U};
  const float countsPerWheelTurn = config_.gearRatio *
                                   config_.encoderCountsPerMotorRevolution *
                                   config_.quadratureMultiplier;
  const float metersPerCount =
      (kPi * config_.wheelDiameterMeters) / countsPerWheelTurn;
  const float seconds = static_cast<float>(dt) / 1000.0F;
  const auto left = static_cast<float>(ticks.left - previous_.left) *
                    metersPerCount / seconds;
  const auto right = static_cast<float>(ticks.right - previous_.right) *
                     metersPerCount / seconds;
  previous_ = ticks;
  previousMs_ = timestampMs;
  return {left, right, dt};
}
void EncoderSpeedEstimator::reset() noexcept { seeded_ = false; }
} // namespace middleware

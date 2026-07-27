#include "Middlewares/wheel_speed_controller.hpp"

namespace middleware {
namespace {
std::int16_t limitForwardRise(std::int16_t requested, std::int16_t previous,
                              std::int16_t riseLimit) noexcept {
  if (requested <= previous)
    return requested;
  const auto maximum = static_cast<std::int32_t>(previous) + riseLimit;
  return requested < maximum ? requested : static_cast<std::int16_t>(maximum);
}

std::int16_t toPwmCommand(float value) noexcept {
  if (value > 1000.0F)
    return 1000;
  if (value < -1000.0F)
    return -1000;
  return static_cast<std::int16_t>(value);
}
} // namespace

WheelSpeedController::WheelSpeedController(
    WheelSpeedControllerConfig config) noexcept
    : estimator_(config.encoder), leftPid_(config.leftPid),
      rightPid_(config.rightPid),
      samplePeriodMs_(config.samplePeriodMs == 0U ? 1U
                                                  : config.samplePeriodMs),
      forwardPwmRisePerUpdate_(config.forwardPwmRisePerUpdate < 1
                                   ? 1
                                   : config.forwardPwmRisePerUpdate) {}

car::WheelCommand
WheelSpeedController::update(car::EncoderTicks ticks, std::uint32_t nowMs,
                             car::WheelCommand targetMmPerSecond) noexcept {
  if (!seeded_) {
    measured_ = estimator_.update(ticks, nowMs);
    previousUpdateMs_ = nowMs;
    seeded_ = true;
    return command_;
  }
  if (static_cast<std::uint32_t>(nowMs - previousUpdateMs_) < samplePeriodMs_)
    return command_;

  measured_ = estimator_.update(ticks, nowMs);
  previousUpdateMs_ = nowMs;
  if (measured_.intervalMs == 0U)
    return command_;

  const float dt = static_cast<float>(measured_.intervalMs) / 1000.0F;
  const float leftMeasuredMmPerSecond = measured_.leftMetersPerSecond * 1000.0F;
  const float rightMeasuredMmPerSecond =
      measured_.rightMetersPerSecond * 1000.0F;
  const float leftOutput = leftPid_.update(
      static_cast<float>(targetMmPerSecond.left), leftMeasuredMmPerSecond, dt);
  const float rightOutput =
      rightPid_.update(static_cast<float>(targetMmPerSecond.right),
                       rightMeasuredMmPerSecond, dt);
  // H-question paths only drive forward. On overspeed, coast instead of
  // commanding a reverse pulse that would make a wheel oscillate.
  const auto leftRequested = toPwmCommand(
      targetMmPerSecond.left > 0 && leftOutput < 0.0F ? 0.0F : leftOutput);
  const auto rightRequested = toPwmCommand(
      targetMmPerSecond.right > 0 && rightOutput < 0.0F ? 0.0F : rightOutput);
  command_ = {
      targetMmPerSecond.left > 0
          ? limitForwardRise(leftRequested, command_.left,
                             forwardPwmRisePerUpdate_)
          : leftRequested,
      targetMmPerSecond.right > 0
          ? limitForwardRise(rightRequested, command_.right,
                             forwardPwmRisePerUpdate_)
          : rightRequested};
  return command_;
}

void WheelSpeedController::reset() noexcept {
  estimator_.reset();
  leftPid_.reset();
  rightPid_.reset();
  measured_ = {};
  command_ = {};
  previousUpdateMs_ = 0U;
  seeded_ = false;
}

} // namespace middleware

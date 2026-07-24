#include "Middlewares/line_follower.hpp"
namespace middleware {
namespace {
constexpr float kTurnLimit = 500.0F;
constexpr float kIntegralLimit = 100.0F;
} // namespace
LineFollower::LineFollower(LineFollowerConfig config) noexcept
    : config_(config),
      pid_({config.kp, config.ki, config.kd, kTurnLimit, kIntegralLimit}) {}

car::VehicleCommand LineFollower::update(const car::LineSample &sample,
                                         float dtSeconds) noexcept {
  if (!sample.detected) {
    pid_.reset();
    return {0, 0};
  }
  const float turn =
      pid_.update(0.0F, static_cast<float>(sample.error), dtSeconds);
  return sample.detected ? car::VehicleCommand{config_.cruise,
                                               static_cast<std::int16_t>(turn)}
                         : car::VehicleCommand{0, 0};
}

bool LineFollower::configure(float kp, float ki, float kd,
                             std::int16_t cruise) noexcept {
  if (kp < 0.0F || kp > 300.0F || ki < 0.0F || ki > 30.0F || kd < 0.0F ||
      kd > 100.0F || cruise < 0 || cruise > 500)
    return false;
  config_ = {kp, ki, kd, cruise};
  pid_.configure({kp, ki, kd, kTurnLimit, kIntegralLimit});
  return true;
}
} // namespace middleware

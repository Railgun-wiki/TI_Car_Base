#include "Middlewares/line_follower.hpp"
namespace middleware {
namespace {
constexpr float kTurnLimit = 500.0F;
constexpr float kIntegralLimit = 100.0F;
constexpr float kMaximumDtSeconds = 1.0F;

float clampRatio(float value) noexcept {
  if (value < 0.0F)
    return 0.0F;
  return value > 1.0F ? 1.0F : value;
}

LineFollowerConfig normalize(LineFollowerConfig config) noexcept {
  if (config.holdMs == 0U)
    config.holdMs = 1U;
  if (config.searchTimeoutMs <= config.holdMs)
    config.searchTimeoutMs = config.holdMs + 1U;
  config.holdSpeedRatio = clampRatio(config.holdSpeedRatio);
  config.searchSpeedRatio = clampRatio(config.searchSpeedRatio);
  return config;
}

std::int16_t scaledCruise(std::int16_t cruise, float ratio) noexcept {
  return static_cast<std::int16_t>(static_cast<float>(cruise) * ratio);
}

float clampTurn(float turn, std::int16_t linear) noexcept {
  const float limit = static_cast<float>(linear);
  if (turn > limit)
    return limit;
  return turn < -limit ? -limit : turn;
}
} // namespace
LineFollower::LineFollower(LineFollowerConfig config) noexcept
    : config_(normalize(config)),
      pid_({config_.kp, config_.ki, config_.kd, kTurnLimit, kIntegralLimit}) {}

LineFollowerResult LineFollower::update(const car::LineSample &sample,
                                        float dtSeconds) noexcept {
  const float validDt =
      dtSeconds > 0.0F && dtSeconds <= kMaximumDtSeconds ? dtSeconds : 0.0F;
  if (sample.detected) {
    if (state_ != LineTrackingState::Tracking)
      pid_.reset();
    state_ = LineTrackingState::Tracking;
    lostElapsedMs_ = 0.0F;
    const float turn =
        pid_.update(0.0F, static_cast<float>(sample.error), validDt);
    lastTurn_ = turn;
    if (sample.error != 0)
      lastErrorDirection_ = sample.error > 0 ? 1 : -1;
    return {{config_.cruise, static_cast<std::int16_t>(turn)}, state_};
  }

  if (state_ == LineTrackingState::Tracking) {
    pid_.reset();
    lostElapsedMs_ = 0.0F;
  }
  if (lastErrorDirection_ == 0) {
    state_ = LineTrackingState::Lost;
    return {{0, 0}, state_};
  }

  lostElapsedMs_ += validDt * 1000.0F;
  if (lostElapsedMs_ >= static_cast<float>(config_.searchTimeoutMs)) {
    state_ = LineTrackingState::Lost;
    return {{0, 0}, state_};
  }
  if (lostElapsedMs_ >= static_cast<float>(config_.holdMs)) {
    state_ = LineTrackingState::Searching;
    const std::int16_t linear =
        scaledCruise(config_.cruise, config_.searchSpeedRatio);
    const auto turn =
        static_cast<std::int16_t>(lastErrorDirection_ > 0 ? -linear : linear);
    return {{linear, turn}, state_};
  }

  state_ = LineTrackingState::Holding;
  const std::int16_t linear =
      scaledCruise(config_.cruise, config_.holdSpeedRatio);
  return {{linear, static_cast<std::int16_t>(clampTurn(lastTurn_, linear))},
          state_};
}

bool LineFollower::configure(float kp, float ki, float kd,
                             std::int16_t cruise) noexcept {
  if (kp < 0.0F || kp > 300.0F || ki < 0.0F || ki > 30.0F || kd < 0.0F ||
      kd > 100.0F || cruise < 0 || cruise > 500)
    return false;
  config_.kp = kp;
  config_.ki = ki;
  config_.kd = kd;
  config_.cruise = cruise;
  pid_.configure({kp, ki, kd, kTurnLimit, kIntegralLimit});
  reset();
  return true;
}

void LineFollower::reset() noexcept {
  pid_.reset();
  lostElapsedMs_ = 0.0F;
  lastTurn_ = 0.0F;
  lastErrorDirection_ = 0;
  state_ = LineTrackingState::Lost;
}
} // namespace middleware

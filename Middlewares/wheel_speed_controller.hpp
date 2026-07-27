#pragma once

#include "Middlewares/encoder_speed.hpp"
#include "Middlewares/pid.hpp"

namespace middleware {

struct WheelSpeedControllerConfig final {
  // mm/s targets map to signed PWM commands in the [-1000, 1000] range.
  EncoderSpeedConfig encoder{};
  PidConfig leftPid{1.0F, 25.0F, 0.0F, 1000.0F, 40.0F};
  PidConfig rightPid{1.0F, 25.0F, 0.0F, 1000.0F, 40.0F};
  // The outer controller may run faster, but speed is updated only after this
  // interval so low speeds have enough encoder counts for a useful estimate.
  std::uint32_t samplePeriodMs = 5U;
  // Limit positive PWM growth per speed update to avoid a startup surge.
  std::int16_t forwardPwmRisePerUpdate = 1000;
};

class WheelSpeedController final {
public:
  explicit WheelSpeedController(
      WheelSpeedControllerConfig config = {}) noexcept;

  car::WheelCommand update(car::EncoderTicks ticks, std::uint32_t nowMs,
                           car::WheelCommand targetMmPerSecond) noexcept;
  void reset() noexcept;
  WheelSpeed measured() const noexcept { return measured_; }

private:
  EncoderSpeedEstimator estimator_;
  Pid leftPid_;
  Pid rightPid_;
  WheelSpeed measured_{};
  car::WheelCommand command_{};
  std::uint32_t previousUpdateMs_ = 0U;
  std::uint32_t samplePeriodMs_ = 5U;
  std::int16_t forwardPwmRisePerUpdate_ = 1000;
  bool seeded_ = false;
};

} // namespace middleware

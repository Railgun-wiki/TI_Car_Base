#pragma once

#include "Common/types.hpp"

namespace middleware {

struct EncoderSpeedConfig final {
  // Wheeltec C07A defaults. Verify against the fitted motor/encoder before
  // enabling closed-loop output.
  float wheelDiameterMeters = 0.048F;
  float gearRatio = 28.0F;
  float encoderCountsPerMotorRevolution = 13.0F;
  float quadratureMultiplier = 4.0F;
};

struct WheelSpeed final {
  float leftMetersPerSecond;
  float rightMetersPerSecond;
  std::uint32_t intervalMs;
};

class EncoderSpeedEstimator final {
public:
  explicit EncoderSpeedEstimator(EncoderSpeedConfig config = {}) noexcept
      : config_(config) {}
  WheelSpeed update(car::EncoderTicks ticks,
                    std::uint32_t timestampMs) noexcept;
  void reset() noexcept;

private:
  EncoderSpeedConfig config_;
  car::EncoderTicks previous_{};
  std::uint32_t previousMs_ = 0U;
  bool seeded_ = false;
};

} // namespace middleware

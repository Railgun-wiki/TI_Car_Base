#pragma once
#include "Common/types.hpp"
#include "Middlewares/pid.hpp"
namespace middleware {
enum class LineTrackingState : std::uint8_t {
  Tracking,
  Holding,
  Searching,
  Lost
};

struct LineFollowerResult final {
  car::VehicleCommand command;
  LineTrackingState state;
};

struct LineFollowerConfig final {
  float kp;
  float ki;
  float kd;
  std::int16_t cruise;
  std::uint32_t holdMs;
  std::uint32_t searchTimeoutMs;
  float holdSpeedRatio;
  float searchSpeedRatio;
};
class LineFollower final {
public:
  explicit LineFollower(LineFollowerConfig config) noexcept;
  LineFollowerResult update(const car::LineSample &sample,
                            float dtSeconds) noexcept;
  bool configure(float kp, float ki, float kd, std::int16_t cruise) noexcept;
  LineFollowerConfig config() const noexcept { return config_; }
  void reset() noexcept;

private:
  LineFollowerConfig config_;
  Pid pid_;
  float lostElapsedMs_ = 0.0F;
  float lastTurn_ = 0.0F;
  std::int8_t lastErrorDirection_ = 0;
  LineTrackingState state_ = LineTrackingState::Lost;
};
} // namespace middleware

#pragma once
#include "Common/types.hpp"
#include "Middlewares/pid.hpp"
namespace middleware {
struct LineFollowerConfig final {
  float kp;
  float ki;
  float kd;
  std::int16_t cruise;
};
class LineFollower final {
public:
  explicit LineFollower(LineFollowerConfig config) noexcept;
  car::VehicleCommand update(const car::LineSample &sample,
                             float dtSeconds) noexcept;
  bool configure(float kp, float ki, float kd, std::int16_t cruise) noexcept;
  LineFollowerConfig config() const noexcept { return config_; }
  void reset() noexcept { pid_.reset(); }

private:
  LineFollowerConfig config_;
  Pid pid_;
};
} // namespace middleware

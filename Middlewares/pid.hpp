#pragma once
namespace middleware {
struct PidConfig final {
  float kp;
  float ki;
  float kd;
  float outputLimit;
  float integralLimit;
};
class Pid final {
public:
  explicit Pid(PidConfig config) noexcept : config_(config) {}
  float update(float target, float measured, float dt) noexcept;
  void reset() noexcept;
  void configure(PidConfig config) noexcept;
  PidConfig config() const noexcept { return config_; }

private:
  PidConfig config_;
  float integral_ = 0.0F;
  float previous_ = 0.0F;
  bool hasPrevious_ = false;
};
} // namespace middleware

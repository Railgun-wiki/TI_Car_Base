#include "Middlewares/pid.hpp"
namespace middleware {
float Pid::update(float target, float measured, float dt) noexcept {
  if (dt <= 0.0F)
    return 0.0F;
  const float e = target - measured;
  const float candidate = integral_ + e * dt;
  const float d = hasPrevious_ ? (e - previous_) / dt : 0.0F;
  float boundedIntegral = candidate;
  if (boundedIntegral > config_.integralLimit)
    boundedIntegral = config_.integralLimit;
  if (boundedIntegral < -config_.integralLimit)
    boundedIntegral = -config_.integralLimit;
  integral_ = boundedIntegral;
  previous_ = e;
  hasPrevious_ = true;
  float o = config_.kp * e + config_.ki * integral_ + config_.kd * d;
  if (o > config_.outputLimit)
    o = config_.outputLimit;
  if (o < -config_.outputLimit)
    o = -config_.outputLimit;
  return o;
}
void Pid::reset() noexcept {
  integral_ = 0.0F;
  previous_ = 0.0F;
  hasPrevious_ = false;
}
void Pid::configure(PidConfig config) noexcept {
  config_ = config;
  reset();
}
} // namespace middleware

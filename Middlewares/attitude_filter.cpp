#include "Middlewares/attitude_filter.hpp"
#include <cmath>
namespace {
constexpr float kDegrees = 57.2957795F;
}
namespace middleware {
void AttitudeFilter::reset() noexcept {
  roll_ = {};
  pitch_ = {};
  initialized_ = false;
}
float AttitudeFilter::updateKalman(KalmanAxis &axis, float measured, float rate,
                                   float dt) noexcept {
  axis.angle += dt * (rate - axis.bias);
  axis.p00 += dt * (dt * axis.p11 - axis.p01 - axis.p10 + config_.kalmanQAngle);
  axis.p01 -= dt * axis.p11;
  axis.p10 -= dt * axis.p11;
  axis.p11 += config_.kalmanQBias * dt;
  const float innovation = measured - axis.angle;
  const float s = axis.p00 + config_.kalmanRMeasure;
  const float k0 = axis.p00 / s, k1 = axis.p10 / s;
  axis.angle += k0 * innovation;
  axis.bias += k1 * innovation;
  const float p00 = axis.p00, p01 = axis.p01;
  axis.p00 -= k0 * p00;
  axis.p01 -= k0 * p01;
  axis.p10 -= k1 * p00;
  axis.p11 -= k1 * p01;
  return axis.angle;
}
car::Status AttitudeFilter::update(car::ImuSample &sample, float dt) noexcept {
  if (dt <= 0.0F || dt > 0.1F)
    return car::Status::InvalidArgument;
  const float rollAcc = std::atan2(sample.ay, sample.az) * kDegrees;
  const float pitchAcc =
      std::atan2(-sample.ax,
                 std::sqrt(sample.ay * sample.ay + sample.az * sample.az)) *
      kDegrees;
  if (!initialized_) {
    roll_.angle = rollAcc;
    pitch_.angle = pitchAcc;
    initialized_ = true;
  }
  if (config_.algorithm == AttitudeAlgorithm::Kalman) {
    sample.rollDeg = updateKalman(roll_, rollAcc, sample.gx, dt);
    sample.pitchDeg = updateKalman(pitch_, pitchAcc, sample.gy, dt);
  } else {
    const float w = config_.complementaryGyroWeight;
    roll_.angle = w * (roll_.angle + sample.gx * dt) + (1.0F - w) * rollAcc;
    pitch_.angle = w * (pitch_.angle + sample.gy * dt) + (1.0F - w) * pitchAcc;
    sample.rollDeg = roll_.angle;
    sample.pitchDeg = pitch_.angle;
  }
  sample.yawDeg += sample.gz * dt; // 6-axis yaw is relative and will drift.
  return car::Status::Ok;
}
} // namespace middleware

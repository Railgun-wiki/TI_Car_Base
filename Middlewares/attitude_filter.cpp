#include "Middlewares/attitude_filter.hpp"
#include <cmath>
#include <cstring>
namespace {
constexpr float kDegPerRad = 57.2957795F;
constexpr float kRadPerDeg = 0.0174532925F;
}
namespace middleware {
void AttitudeFilter::reset() noexcept {
  roll_ = {};
  pitch_ = {};
  yaw_ = 0.0F;
  q0_ = 1.0F;
  q1_ = 0.0F;
  q2_ = 0.0F;
  q3_ = 0.0F;
  iEx_ = 0.0F;
  iEy_ = 0.0F;
  iEz_ = 0.0F;
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
  const float rollAcc = std::atan2(sample.ay, sample.az) * kDegPerRad;
  const float pitchAcc =
      std::atan2(-sample.ax,
                 std::sqrt(sample.ay * sample.ay + sample.az * sample.az)) *
      kDegPerRad;
  if (!initialized_) {
    roll_.angle = rollAcc;
    pitch_.angle = pitchAcc;
    initialized_ = true;
  }
  switch (config_.algorithm) {
  case AttitudeAlgorithm::Kalman:
    sample.rollDeg = updateKalman(roll_, rollAcc, sample.gx, dt);
    sample.pitchDeg = updateKalman(pitch_, pitchAcc, sample.gy, dt);
    break;
  case AttitudeAlgorithm::Complementary: {
    const float w = config_.complementaryGyroWeight;
    roll_.angle = w * (roll_.angle + sample.gx * dt) + (1.0F - w) * rollAcc;
    pitch_.angle = w * (pitch_.angle + sample.gy * dt) + (1.0F - w) * pitchAcc;
    sample.rollDeg = roll_.angle;
    sample.pitchDeg = pitch_.angle;
    break;
  }
  case AttitudeAlgorithm::Mahony:
    updateMahony(sample, dt);
    return car::Status::Ok;
  }
  yaw_ += sample.gz * dt; // 6-axis yaw is relative and will drift.
  sample.yawDeg = yaw_;
  return car::Status::Ok;
}

float AttitudeFilter::fastInvSqrt(float x) noexcept {
  // Classic Quake III Q_rsqrt, also used by SJTU-AuTop attitude_solution.c.
  // Cortex-M0+ has no FPU; this is markedly cheaper than 1/sqrtf and the
  // ~1% error is absorbed by accelerometer normalization.
  const float halfx = 0.5F * x;
  float y = x;
  long i;
  std::memcpy(&i, &y, sizeof(i));
  i = 0x5f3759df - (i >> 1);
  std::memcpy(&y, &i, sizeof(y));
  y = y * (1.5F - (halfx * y * y));
  return y;
}

void AttitudeFilter::updateMahony(car::ImuSample &sample, float dt) noexcept {
  const float gx = sample.gx * kRadPerDeg;
  const float gy = sample.gy * kRadPerDeg;
  const float gz = sample.gz * kRadPerDeg;
  const float normSq =
      sample.ax * sample.ax + sample.ay * sample.ay + sample.az * sample.az;
  // Degenerate input (free-fall or dropped frame): disable the accelerometer
  // cross-product correction and integrate gyro only, otherwise
  // fastInvSqrt(0) yields NaN and corrupts the quaternion state.
  const bool accValid = normSq > 1e-9F;
  const float normRecip = accValid ? fastInvSqrt(normSq) : 0.0F;
  const float ax = accValid ? sample.ax * normRecip : 0.0F;
  const float ay = accValid ? sample.ay * normRecip : 0.0F;
  const float az = accValid ? sample.az * normRecip : 1.0F;

  // Estimated gravity direction from current quaternion.
  const float vx = 2.0F * (q1_ * q3_ - q0_ * q2_);
  const float vy = 2.0F * (q0_ * q1_ + q2_ * q3_);
  const float vz = q0_ * q0_ - q1_ * q1_ - q2_ * q2_ + q3_ * q3_;

  // Cross-product error between measured and estimated gravity.
  const float ex = accValid ? (ay * vz - az * vy) : 0.0F;
  const float ey = accValid ? (az * vx - ax * vz) : 0.0F;
  const float ez = accValid ? (ax * vy - ay * vx) : 0.0F;

  const float halfT = 0.5F * dt;
  iEx_ += halfT * ex;
  iEy_ += halfT * ey;
  iEz_ += halfT * ez;

  const float cgx = gx + config_.mahonyKp * ex + config_.mahonyKi * iEx_;
  const float cgy = gy + config_.mahonyKp * ey + config_.mahonyKi * iEy_;
  const float cgz = gz + config_.mahonyKp * ez + config_.mahonyKi * iEz_;

  // First-order quaternion integration (matches attitude_solution.c).
  q0_ += (-q1_ * cgx - q2_ * cgy - q3_ * cgz) * halfT;
  q1_ += (q0_ * cgx + q2_ * cgz - q3_ * cgy) * halfT;
  q2_ += (q0_ * cgy - q1_ * cgz + q3_ * cgx) * halfT;
  q3_ += (q0_ * cgz + q1_ * cgy - q2_ * cgx) * halfT;

  const float qRecip =
      fastInvSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
  q0_ *= qRecip;
  q1_ *= qRecip;
  q2_ *= qRecip;
  q3_ *= qRecip;

  // Quaternion -> Euler (same convention as attitude_solution.c). Yaw stays
  // continuous (no 0..360 wrap) so H-question heading PID input domain is
  // unchanged.
  sample.rollDeg = std::atan2(2.0F * q2_ * q3_ + 2.0F * q0_ * q1_,
                              -2.0F * q1_ * q1_ - 2.0F * q2_ * q2_ + 1.0F) *
                   kDegPerRad;
  sample.pitchDeg =
      std::asin(-2.0F * q1_ * q3_ + 2.0F * q0_ * q2_) * kDegPerRad;
  sample.yawDeg = std::atan2(2.0F * q1_ * q2_ + 2.0F * q0_ * q3_,
                             -2.0F * q2_ * q2_ - 2.0F * q3_ * q3_ + 1.0F) *
                  kDegPerRad;
}
} // namespace middleware

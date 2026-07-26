#pragma once

#include "Common/types.hpp"

namespace middleware {
enum class AttitudeAlgorithm : std::uint8_t { Complementary, Kalman, Mahony };
struct AttitudeFilterConfig final {
  AttitudeAlgorithm algorithm = AttitudeAlgorithm::Complementary;
  float complementaryGyroWeight = 0.98F;
  float kalmanQAngle = 0.001F;
  float kalmanQBias = 0.003F;
  float kalmanRMeasure = 0.03F;
  // Mahony AHRS (SJTU-AuTop attitude_solution.c defaults).
  float mahonyKp = 0.17F;
  float mahonyKi = 0.004F;
};
class AttitudeFilter final {
public:
  explicit AttitudeFilter(AttitudeFilterConfig config = {}) noexcept
      : config_(config) {}
  void reset() noexcept;
  car::Status update(car::ImuSample &sample, float dtSeconds) noexcept;

private:
  struct KalmanAxis final {
    float angle = 0.0F, bias = 0.0F, p00 = 0.0F, p01 = 0.0F, p10 = 0.0F,
          p11 = 0.0F;
  };
  float updateKalman(KalmanAxis &axis, float measured, float rate,
                     float dtSeconds) noexcept;
  void updateMahony(car::ImuSample &sample, float dtSeconds) noexcept;
  static float fastInvSqrt(float x) noexcept;
  AttitudeFilterConfig config_;
  KalmanAxis roll_{}, pitch_{};
  float yaw_ = 0.0F;
  // Mahony AHRS state.
  float q0_ = 1.0F, q1_ = 0.0F, q2_ = 0.0F, q3_ = 0.0F;
  float iEx_ = 0.0F, iEy_ = 0.0F, iEz_ = 0.0F;
  bool initialized_ = false;
};
} // namespace middleware

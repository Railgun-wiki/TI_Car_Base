#pragma once
#include "Common/types.hpp"
#include "Drivers/imu_backend.hpp"
#include "Middlewares/attitude_backend_config.h"

// Compile-time switch for Bmi270::poll output semantics.
// Undefined/0 (default): poll() fills ax..gz only (g, deg/s); Euler left zero
//   and computed upstream by ImuReader + AttitudeFilter (same as Mpu6050).
// 1: poll() additionally runs an on-chip-style Mahony fusion and fills
//   rollDeg/pitchDeg/yawDeg directly, bypassing AttitudeFilter.
#ifndef BMI270_ONBOARD_FUSION
#define BMI270_ONBOARD_FUSION 0
#endif

namespace drivers {

// BMI270 6-axis IMU driver over I2C0. Implements drivers::ImuBackend so the
// software attitude path is chip-agnostic. BMI270 requires an 8192-byte
// config blob (ThirdParty/bmi270/bmi270_config_file.h) burst-written during
// begin(); see Drivers/bmi270.cpp.
class Bmi270 final : public ImuBackend {
public:
  // addr: 0x68 (SDO low, board default) or 0x69 (SDO high). Auto-probes both
  // by reading CHIP_ID (0x00) expecting 0x24 if left at the default of 0.
  explicit Bmi270(std::uint8_t addr = 0U) noexcept : address_(addr) {}
  ~Bmi270() noexcept override = default;
  car::Status begin() noexcept override;
  car::Status poll(car::ImuSample &sample) noexcept override;
  // Blocks for samples*delayMs while averaging static gyro bias. The car must
  // be stationary. Stored bias is subtracted from gx/gy/gz on every subsequent
  // poll; calling again overwrites the previous estimate.
  car::Status calibrateGyroBias(std::uint16_t samples = 200U,
                                std::uint16_t delayMs = 5U) noexcept;
  bool ready() const noexcept override { return ready_; }

private:
  bool ready_ = false;
  std::uint8_t address_ = 0U;
  float gyroBiasX_ = 0.0F;
  float gyroBiasY_ = 0.0F;
  float gyroBiasZ_ = 0.0F;
#if BMI270_ONBOARD_FUSION
  // Mahony AHRS state. dt assumes the ~100 Hz data-ready cadence; verify on
  // hardware and adjust if the INT rate differs.
  float q0_ = 1.0F;
  float q1_ = 0.0F;
  float q2_ = 0.0F;
  float q3_ = 0.0F;
  float iEx_ = 0.0F;
  float iEy_ = 0.0F;
  float iEz_ = 0.0F;
  void updateMahony(car::ImuSample &sample, float dt) noexcept;
  static float fastInvSqrt(float x) noexcept;
#endif
};

} // namespace drivers

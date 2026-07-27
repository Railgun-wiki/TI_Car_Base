#pragma once
#include "Common/types.hpp"
#include "Drivers/imu_backend.hpp"
namespace drivers {

// LSB/(°/s) at ±250 dps; LSB/g at ±2 g. Matches MPU-6000/6050 default range.
constexpr float kAccelSensitivity = 16384.0F;
constexpr float kGyroSensitivity = 131.0F;

class Mpu6050 final : public ImuBackend {
public:
  // addr: 0x68 (AD0 low, board default) or 0x69 (AD0 high). Auto-probes both
  // if left at the default of 0.
  explicit Mpu6050(std::uint8_t addr = 0U) noexcept : address_(addr) {}
  ~Mpu6050() noexcept override = default;
  car::Status begin() noexcept override;
  car::Status poll(car::ImuSample &sample) noexcept override;
  // Blocks for samples*delayMs while averaging static gyro bias. The car must
  // be stationary. Stored bias is subtracted from gx/gy/gz on every subsequent
  // poll; calling again overwrites the previous estimate.
  car::Status calibrateGyroBias(std::uint16_t samples = 200U,
                                std::uint16_t delayMs = 5U) noexcept;
  // Slowly adapts the stored gyro bias from an already bias-corrected sample.
  // The caller must invoke this only after a reliable stationary detection.
  void updateGyroBiasFromStationarySample(const car::ImuSample &sample,
                                          float dtSeconds) noexcept;
  bool ready() const noexcept override { return ready_; }

private:
  bool ready_ = false;
  std::uint8_t address_ = 0U;
  float gyroBiasX_ = 0.0F;
  float gyroBiasY_ = 0.0F;
  float gyroBiasZ_ = 0.0F;
};
} // namespace drivers

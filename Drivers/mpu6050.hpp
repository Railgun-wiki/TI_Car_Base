#pragma once
#include "Common/types.hpp"
namespace drivers {
class Mpu6050 final {
public:
  car::Status begin() noexcept;
  car::Status poll(car::ImuSample &sample) noexcept;
  // Blocks for samples*delayMs while averaging static gyro bias. The car must
  // be stationary. Stored bias is subtracted from gx/gy/gz on every subsequent
  // poll; calling again overwrites the previous estimate.
  car::Status calibrateGyroBias(std::uint16_t samples = 200U,
                                std::uint16_t delayMs = 5U) noexcept;
  bool ready() const noexcept { return ready_; }

private:
  bool ready_ = false;
  float gyroBiasX_ = 0.0F;
  float gyroBiasY_ = 0.0F;
  float gyroBiasZ_ = 0.0F;
};
} // namespace drivers

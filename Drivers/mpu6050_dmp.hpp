#pragma once

#include "Common/types.hpp"
#include "Drivers/imu_backend.hpp"

namespace drivers {

class Mpu6050Dmp final : public ImuBackend {
public:
  ~Mpu6050Dmp() noexcept override = default;
  car::Status begin() noexcept override;
  car::Status poll(car::ImuSample &sample) noexcept override;
  void notifyDataReady() noexcept { due_ = true; }
  bool ready() const noexcept override { return ready_; }

private:
  bool ready_ = false;
  bool due_ = false;
};

} // namespace drivers

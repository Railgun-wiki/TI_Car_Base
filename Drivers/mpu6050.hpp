#pragma once
#include "Common/types.hpp"
namespace drivers {
class Mpu6050 final {
public:
  car::Status begin() noexcept;
  car::Status poll(car::ImuSample &sample) noexcept;
  bool ready() const noexcept { return ready_; }

private:
  bool ready_ = false;
};
} // namespace drivers

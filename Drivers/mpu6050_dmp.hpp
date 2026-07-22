#pragma once

#include "Common/types.hpp"

namespace drivers {

class Mpu6050Dmp final {
public:
  car::Status begin() noexcept;
  car::Status poll(car::ImuSample &sample) noexcept;
  void notifyDataReady() noexcept { due_ = true; }
  bool ready() const noexcept { return ready_; }

private:
  bool ready_ = false;
  bool due_ = false;
};

} // namespace drivers

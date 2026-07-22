#pragma once
#include "Common/types.hpp"
namespace drivers {
class MotorDriver final {
public:
  void stop() noexcept;
  void set(car::WheelCommand command) noexcept;
  car::WheelCommand command() const noexcept { return command_; }

private:
  car::WheelCommand command_{0, 0};
};
} // namespace drivers

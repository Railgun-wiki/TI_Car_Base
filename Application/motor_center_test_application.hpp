#pragma once

#include "Drivers/keypad.hpp"
#include "Drivers/led.hpp"
#include "Drivers/motor_driver.hpp"

namespace app {

// Bench-only motor test. Center is deliberately hold-to-run: releasing it
// stops both motors immediately.
class MotorCenterTestApplication final {
public:
  void init() noexcept;
  void step() noexcept;

private:
  static constexpr std::int16_t kHalfDuty = 500;

  drivers::MotorDriver motor_{};
  drivers::Keypad keys_{};
  drivers::Led leds_{};
  bool running_ = false;
};

} // namespace app

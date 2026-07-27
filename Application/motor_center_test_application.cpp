#include "Application/motor_center_test_application.hpp"

namespace app {

void MotorCenterTestApplication::init() noexcept {
  motor_.stop();
  leds_.setStatus(0U, true);
  leds_.setStatus(1U, false);
  leds_.setStatus(2U, false);
  leds_.setUser(false);
}

void MotorCenterTestApplication::step() noexcept {
  const bool shouldRun = keys_.pressed(car::Key::Center);
  if (shouldRun == running_)
    return;

  running_ = shouldRun;
  if (running_)
    motor_.set({kHalfDuty, kHalfDuty});
  else
    motor_.stop();
  leds_.setUser(running_);
}

} // namespace app

#include "Application/motor_center_test_application.hpp"
#include "BSP/system.hpp"
#include "Config/status_led_config.hpp"

namespace app {

void MotorCenterTestApplication::init() noexcept {
  motor_.stop();
  leds_.setPattern(config::status_led::kMotorIdle);
  leds_.service(bsp::millis());
  leds_.setUser(false);
}

void MotorCenterTestApplication::step() noexcept {
  const std::uint32_t now = bsp::millis();
  const bool shouldRun = keys_.pressed(car::Key::Center);
  if (shouldRun != running_) {
    running_ = shouldRun;
    if (running_)
      motor_.set({kHalfDuty, kHalfDuty});
    else
      motor_.stop();
    leds_.setUser(running_);
  }
  leds_.setPattern(running_ ? config::status_led::kMotorRunning
                            : config::status_led::kMotorIdle);
  leds_.service(now);
}

} // namespace app

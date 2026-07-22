#include "Drivers/motor_driver.hpp"
#include "BSP/motor.hpp"
namespace drivers {

void MotorDriver::stop() noexcept {
  command_ = {0, 0};
  ::bsp::stopMotors();
}

void MotorDriver::set(car::WheelCommand command) noexcept {
  command_ = {car::clampCommand(command.left),
              car::clampCommand(command.right)};
  ::bsp::setMotorDuty(command_.left, command_.right);
}

} // namespace drivers

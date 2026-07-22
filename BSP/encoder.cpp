#include "BSP/encoder.hpp"
#include "ti_msp_dl_config.h"
namespace {
volatile std::int32_t leftTicks = 0, rightTicks = 0;
volatile bool imuDue = false;
std::uint8_t leftState = 0, rightState = 0;
void update(volatile std::int32_t &ticks, std::uint8_t &previous,
            std::uint8_t current) {
  const auto transition = static_cast<std::uint8_t>((previous << 2U) | current);
  if (transition == 1U || transition == 7U || transition == 14U ||
      transition == 8U)
    ++ticks;
  if (transition == 2U || transition == 11U || transition == 13U ||
      transition == 4U)
    --ticks;
  previous = current;
}
void sample() {
  const auto left = static_cast<std::uint8_t>(
      (DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_ENCODER_LEFT_A_PIN)
           ? 2U
           : 0U) |
      (DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_ENCODER_LEFT_B_PIN)
           ? 1U
           : 0U));
  const auto right = static_cast<std::uint8_t>(
      (DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_ENCODER_RIGHT_A_PIN)
           ? 2U
           : 0U) |
      (DL_GPIO_readPins(GPIO_ENCODER_PORT, GPIO_ENCODER_ENCODER_RIGHT_B_PIN)
           ? 1U
           : 0U));
  update(leftTicks, leftState, left);
  update(rightTicks, rightState, right);
}
} // namespace
namespace bsp {
void initializeEncoders() noexcept { sample(); }
car::EncoderTicks encoderTicks() noexcept { return {leftTicks, rightTicks}; }
void resetEncoderTicks() noexcept {
  leftTicks = 0;
  rightTicks = 0;
}
bool consumeImuDataReady() noexcept {
  const bool due = imuDue;
  imuDue = false;
  return due;
}
} // namespace bsp
extern "C" void GPIOB_IRQHandler(void) {
  const std::uint32_t enc =
      GPIO_ENCODER_ENCODER_LEFT_A_PIN | GPIO_ENCODER_ENCODER_LEFT_B_PIN |
      GPIO_ENCODER_ENCODER_RIGHT_A_PIN | GPIO_ENCODER_ENCODER_RIGHT_B_PIN;
  const auto pending = DL_GPIO_getEnabledInterruptStatus(
      GPIOB, enc | GPIO_MPU6050_DATA_READY_MPU6050_INT_PIN);
  if ((pending & enc) != 0U) {
    sample();
    DL_GPIO_clearInterruptStatus(GPIOB, pending & enc);
  }
  if ((pending & GPIO_MPU6050_DATA_READY_MPU6050_INT_PIN) != 0U) {
    imuDue = true;
    DL_GPIO_clearInterruptStatus(GPIOB,
                                 GPIO_MPU6050_DATA_READY_MPU6050_INT_PIN);
  }
}

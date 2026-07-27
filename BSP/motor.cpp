#include "BSP/motor.hpp"
#include "Common/types.hpp"
#include "ti_msp_dl_config.h"
namespace {
constexpr std::uint32_t kPeriod = 8000U;
void setLeftDirection(std::int16_t value) {
  if (value > 0) {
    DL_GPIO_setPins(GPIO_MOTOR_DIR_AIN1_PORT, GPIO_MOTOR_DIR_AIN1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_DIR_AIN2_PORT, GPIO_MOTOR_DIR_AIN2_PIN);
  } else if (value < 0) {
    DL_GPIO_clearPins(GPIO_MOTOR_DIR_AIN1_PORT, GPIO_MOTOR_DIR_AIN1_PIN);
    DL_GPIO_setPins(GPIO_MOTOR_DIR_AIN2_PORT, GPIO_MOTOR_DIR_AIN2_PIN);
  } else {
    DL_GPIO_clearPins(GPIO_MOTOR_DIR_AIN1_PORT, GPIO_MOTOR_DIR_AIN1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_DIR_AIN2_PORT, GPIO_MOTOR_DIR_AIN2_PIN);
  }
}

void setRightDirection(std::int16_t value) {
  if (value > 0) {
    DL_GPIO_setPins(GPIO_MOTOR_DIR_BIN2_PORT, GPIO_MOTOR_DIR_BIN2_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_DIR_BIN1_PORT, GPIO_MOTOR_DIR_BIN1_PIN);
  } else if (value < 0) {
    DL_GPIO_clearPins(GPIO_MOTOR_DIR_BIN2_PORT, GPIO_MOTOR_DIR_BIN2_PIN);
    DL_GPIO_setPins(GPIO_MOTOR_DIR_BIN1_PORT, GPIO_MOTOR_DIR_BIN1_PIN);
  } else {
    DL_GPIO_clearPins(GPIO_MOTOR_DIR_BIN1_PORT, GPIO_MOTOR_DIR_BIN1_PIN);
    DL_GPIO_clearPins(GPIO_MOTOR_DIR_BIN2_PORT, GPIO_MOTOR_DIR_BIN2_PIN);
  }
}
} // namespace
namespace bsp {
void startMotorPwm() noexcept {
  DL_TimerA_startCounter(PWM_MOTOR_INST);
  DL_TimerG_startCounter(PWM_MOTOR_B_INST);
}
void setMotorDuty(std::int16_t left, std::int16_t right) noexcept {
  const auto safeLeft = car::clampCommand(left);
  const auto safeRight = car::clampCommand(right);
  const auto la =
      static_cast<std::uint32_t>(safeLeft < 0 ? -safeLeft : safeLeft);
  const auto ra =
      static_cast<std::uint32_t>(safeRight < 0 ? -safeRight : safeRight);
  // The generated PWM waveform is active while the counter is above compare,
  // so compare=kPeriod is 0% duty and compare=0 is 100% duty.
  DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, kPeriod,
                                   DL_TIMER_CC_0_INDEX);
  DL_TimerG_setCaptureCompareValue(PWM_MOTOR_B_INST, kPeriod,
                                   DL_TIMER_CC_0_INDEX);
  delay_cycles(80U);
  setLeftDirection(safeLeft);
  setRightDirection(safeRight);
  DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST,
                                   kPeriod - la * kPeriod / 1000U,
                                   DL_TIMER_CC_0_INDEX);
  DL_TimerG_setCaptureCompareValue(PWM_MOTOR_B_INST,
                                   kPeriod - ra * kPeriod / 1000U,
                                   DL_TIMER_CC_0_INDEX);
}
void stopMotors() noexcept { setMotorDuty(0, 0); }
} // namespace bsp

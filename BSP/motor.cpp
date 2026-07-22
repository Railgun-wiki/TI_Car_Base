#include "BSP/motor.hpp"
#include "ti_msp_dl_config.h"
namespace {
constexpr std::uint32_t kPeriod = 8000U;
void direction(GPIO_Regs *port, std::uint32_t forward, std::uint32_t reverse,
               std::int16_t value) {
  if (value > 0) {
    DL_GPIO_setPins(port, forward);
    DL_GPIO_clearPins(port, reverse);
  } else if (value < 0) {
    DL_GPIO_clearPins(port, forward);
    DL_GPIO_setPins(port, reverse);
  } else
    DL_GPIO_clearPins(port, forward | reverse);
}
} // namespace
namespace bsp {
void startMotorPwm() noexcept {
  DL_TimerA_startCounter(PWM_MOTOR_INST);
  DL_TimerG_startCounter(PWM_MOTOR_B_INST);
}
void setMotorDuty(std::int16_t left, std::int16_t right) noexcept {
  const auto la = static_cast<std::uint32_t>(left < 0 ? -left : left),
             ra = static_cast<std::uint32_t>(right < 0 ? -right : right);
  DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, 0U, DL_TIMER_CC_0_INDEX);
  DL_TimerG_setCaptureCompareValue(PWM_MOTOR_B_INST, 0U, DL_TIMER_CC_0_INDEX);
  delay_cycles(80U);
  direction(GPIO_MOTOR_DIR_AIN1_PORT, GPIO_MOTOR_DIR_AIN1_PIN,
            GPIO_MOTOR_DIR_AIN2_PIN, left);
  direction(GPIO_MOTOR_DIR_BIN1_PORT, GPIO_MOTOR_DIR_BIN1_PIN,
            GPIO_MOTOR_DIR_BIN2_PIN, right);
  DL_TimerA_setCaptureCompareValue(PWM_MOTOR_INST, la * kPeriod / 1000U,
                                   DL_TIMER_CC_0_INDEX);
  DL_TimerG_setCaptureCompareValue(PWM_MOTOR_B_INST, ra * kPeriod / 1000U,
                                   DL_TIMER_CC_0_INDEX);
}
void stopMotors() noexcept { setMotorDuty(0, 0); }
} // namespace bsp

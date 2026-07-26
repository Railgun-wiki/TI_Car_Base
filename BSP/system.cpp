#include "BSP/system.hpp"
#include "BSP/encoder.hpp"
#include "BSP/motor.hpp"
#include "ti_msp_dl_config.h"
namespace {
volatile std::uint32_t g_millis = 0U;
}
namespace bsp {
void init() noexcept {
  // Keep this ordering in sync with SysConfig output.  UART0 intentionally
  // precedes the I2C controllers so boot diagnostics are observable before
  // probing external devices.
  SYSCFG_DL_initPower();
  SYSCFG_DL_GPIO_init();
  SYSCFG_DL_SYSCTL_init();
  SYSCFG_DL_PWM_MOTOR_init();
  SYSCFG_DL_PWM_MOTOR_B_init();
  SYSCFG_DL_UART_CONSOLE_init();
  SYSCFG_DL_I2C_MPU6050_init();
  SYSCFG_DL_I2C_OLED_init();
  SYSCFG_DL_UART3_MODULE_init();
  SYSCFG_DL_UART2_MODULE_init();
  SYSCFG_DL_UART1_MODULE_init();
  SYSCFG_DL_DMA_init();
  SYSCFG_DL_SYSCTL_CLK_init();
  stopMotors();
  startMotorPwm();
  initializeEncoders();
  (void)SysTick_Config(80000000UL / 1000UL);
}
std::uint32_t millis() noexcept { return g_millis; }
} // namespace bsp
extern "C" void SysTick_Handler(void) { ++g_millis; }

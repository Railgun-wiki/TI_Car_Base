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
  // SysConfig enables the UART peripheral interrupt source but does not enable
  // its NVIC line. The boot logger needs this before its first 17-byte frame.
  NVIC_EnableIRQ(UART_CONSOLE_INST_INT_IRQN);
  SYSCFG_DL_I2C_MPU6050_init();
  SYSCFG_DL_I2C_OLED_init();
  // OLED DMA completion is finalized by I2C1_IRQHandler.
  NVIC_EnableIRQ(I2C_OLED_INST_INT_IRQN);
  SYSCFG_DL_UART3_MODULE_init();
  SYSCFG_DL_UART2_MODULE_init();
  SYSCFG_DL_UART1_MODULE_init();
  SYSCFG_DL_DMA_init();
  // Encoder A/B edges and MPU data-ready share the GPIOB interrupt vector.
  NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
  SYSCFG_DL_SYSCTL_CLK_init();
  stopMotors();
  startMotorPwm();
  initializeEncoders();
  (void)SysTick_Config(80000000UL / 1000UL);
}
std::uint32_t millis() noexcept { return g_millis; }
void delayMs(std::uint32_t ms) noexcept {
  const std::uint32_t start = g_millis;
  while (g_millis - start < ms) {
    // Wait for the next SysTick tick; reads of g_millis are volatile.
  }
}
} // namespace bsp
extern "C" void SysTick_Handler(void) { ++g_millis; }

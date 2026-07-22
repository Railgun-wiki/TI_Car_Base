#include "BSP/system.hpp"
#include "BSP/encoder.hpp"
#include "BSP/motor.hpp"
#include "ti_msp_dl_config.h"
namespace {
volatile std::uint32_t g_millis = 0U;
}
namespace bsp {
void init() noexcept {
  SYSCFG_DL_init();
  stopMotors();
  startMotorPwm();
  initializeEncoders();
  (void)SysTick_Config(80000000UL / 1000UL);
}
std::uint32_t millis() noexcept { return g_millis; }
} // namespace bsp
extern "C" void SysTick_Handler(void) { ++g_millis; }

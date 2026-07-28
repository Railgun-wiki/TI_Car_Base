#include "BSP/indicator.hpp"
#include "ti_msp_dl_config.h"
namespace bsp {
void setStatusLed(std::uint8_t index, bool on) noexcept {
  GPIO_Regs *port = nullptr;
  std::uint32_t pin = 0U;
  switch (index) {
  case 0U:
    port = GPIO_LED_LED1_PORT;
    pin = GPIO_LED_LED1_PIN;
    break;
  case 1U:
    port = GPIO_LED_LED2_PORT;
    pin = GPIO_LED_LED2_PIN;
    break;
  case 2U:
    port = GPIO_LED_LED3_PORT;
    pin = GPIO_LED_LED3_PIN;
    break;
  default:
    return;
  }
  if (on)
    DL_GPIO_clearPins(port, pin);
  else
    DL_GPIO_setPins(port, pin);
}
void setUserLed(bool on) noexcept {
  if (on)
    DL_GPIO_setPins(GPIO_LED_USER_LED_PORT, GPIO_LED_USER_LED_PIN);
  else
    DL_GPIO_clearPins(GPIO_LED_USER_LED_PORT, GPIO_LED_USER_LED_PIN);
}
void setBuzzer(bool on) noexcept {
  if (on)
    DL_GPIO_clearPins(GPIO_BUZZER_PORT, GPIO_BUZZER_BUZZER_PIN);
  else
    DL_GPIO_setPins(GPIO_BUZZER_PORT, GPIO_BUZZER_BUZZER_PIN);
}
} // namespace bsp

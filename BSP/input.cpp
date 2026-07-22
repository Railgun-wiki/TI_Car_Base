#include "BSP/input.hpp"
#include "ti_msp_dl_config.h"
namespace bsp {
std::uint8_t readLineBits() noexcept {
  return static_cast<std::uint8_t>(
      (DL_GPIO_readPins(GPIO_LINE_SENSOR_C1_PORT, GPIO_LINE_SENSOR_C1_PIN)
           ? 1U
           : 0U) |
      (DL_GPIO_readPins(GPIO_LINE_SENSOR_C2_PORT, GPIO_LINE_SENSOR_C2_PIN)
           ? 2U
           : 0U) |
      (DL_GPIO_readPins(GPIO_LINE_SENSOR_C3_PORT, GPIO_LINE_SENSOR_C3_PIN)
           ? 4U
           : 0U) |
      (DL_GPIO_readPins(GPIO_LINE_SENSOR_C4_PORT, GPIO_LINE_SENSOR_C4_PIN)
           ? 8U
           : 0U) |
      (DL_GPIO_readPins(GPIO_LINE_SENSOR_C5_PORT, GPIO_LINE_SENSOR_C5_PIN)
           ? 16U
           : 0U) |
      (DL_GPIO_readPins(GPIO_LINE_SENSOR_C6_PORT, GPIO_LINE_SENSOR_C6_PIN)
           ? 32U
           : 0U) |
      (DL_GPIO_readPins(GPIO_LINE_SENSOR_C7_PORT, GPIO_LINE_SENSOR_C7_PIN)
           ? 64U
           : 0U) |
      (DL_GPIO_readPins(GPIO_LINE_SENSOR_C8_PORT, GPIO_LINE_SENSOR_C8_PIN)
           ? 128U
           : 0U));
}
bool keyPressed(car::Key key) noexcept {
  switch (key) {
  case car::Key::Up:
    return DL_GPIO_readPins(GPIO_KEY_KEY_UP_PORT, GPIO_KEY_KEY_UP_PIN) == 0U;
  case car::Key::Left:
    return DL_GPIO_readPins(GPIO_KEY_KEY_LEFT_PORT, GPIO_KEY_KEY_LEFT_PIN) ==
           0U;
  case car::Key::Down:
    return DL_GPIO_readPins(GPIO_KEY_KEY_DOWN_PORT, GPIO_KEY_KEY_DOWN_PIN) ==
           0U;
  case car::Key::Right:
    return DL_GPIO_readPins(GPIO_KEY_KEY_RIGHT_PORT, GPIO_KEY_KEY_RIGHT_PIN) ==
           0U;
  case car::Key::Center:
    return DL_GPIO_readPins(GPIO_KEY_KEY_CENTER_PORT,
                            GPIO_KEY_KEY_CENTER_PIN) == 0U;
  default:
    return false;
  }
}
} // namespace bsp

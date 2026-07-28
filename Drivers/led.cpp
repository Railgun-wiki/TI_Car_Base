#include "Drivers/led.hpp"
#include "BSP/indicator.hpp"
namespace drivers {

namespace {
bool outputFor(LedMode mode, bool blinkOn) noexcept {
  switch (mode) {
  case LedMode::On:
    return true;
  case LedMode::Blink:
    return blinkOn;
  case LedMode::Off:
    return false;
  }
  return false;
}
} // namespace

void Led::setMode(std::uint8_t index, LedMode mode) noexcept {
  if (index == 0U)
    pattern_.led1 = mode;
  else if (index == 1U)
    pattern_.led2 = mode;
  else if (index == 2U)
    pattern_.led3 = mode;
}

void Led::service(std::uint32_t nowMs) const noexcept {
  const bool blinkOn =
      ((nowMs / VEHICLE_TUNING_LED_BLINK_HALF_PERIOD_MS) & 1U) == 0U;
  ::bsp::setStatusLed(0U, outputFor(pattern_.led1, blinkOn));
  ::bsp::setStatusLed(1U, outputFor(pattern_.led2, blinkOn));
  ::bsp::setStatusLed(2U, outputFor(pattern_.led3, blinkOn));
}

void Led::setUser(bool on) noexcept { ::bsp::setUserLed(on); }
} // namespace drivers

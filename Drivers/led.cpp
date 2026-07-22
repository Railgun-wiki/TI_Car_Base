#include "Drivers/led.hpp"
#include "BSP/indicator.hpp"
namespace drivers {
void Led::setStatus(std::uint8_t index, bool on) noexcept {
  ::bsp::setStatusLed(index, on);
}
void Led::setUser(bool on) noexcept { ::bsp::setUserLed(on); }
} // namespace drivers

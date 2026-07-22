#include "Drivers/keypad.hpp"
#include "BSP/input.hpp"
namespace drivers {
bool Keypad::pressed(car::Key key) const noexcept {
  return ::bsp::keyPressed(key);
}
} // namespace drivers

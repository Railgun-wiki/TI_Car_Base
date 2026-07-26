#include "Drivers/keypad.hpp"
#include "BSP/input.hpp"
#include "BSP/system.hpp"
namespace drivers {
bool Keypad::pressed(car::Key key) noexcept {
  const auto index = static_cast<std::uint8_t>(key);
  if (index >= static_cast<std::uint8_t>(car::Key::None))
    return false;

  State &state = states_[index];
  const bool raw = ::bsp::keyPressed(key);
  const std::uint32_t now = ::bsp::millis();
  if (raw != state.raw) {
    state.raw = raw;
    state.changedAtMs = now;
  }
  if (state.stable != state.raw &&
      static_cast<std::uint32_t>(now - state.changedAtMs) >= kDebounceMs)
    state.stable = state.raw;
  return state.stable;
}
} // namespace drivers

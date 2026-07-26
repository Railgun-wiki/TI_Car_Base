#pragma once
#include "Common/types.hpp"
#include <cstdint>
namespace drivers {
class Keypad final {
public:
  bool pressed(car::Key key) noexcept;

private:
  struct State final {
    bool raw = false;
    bool stable = false;
    std::uint32_t changedAtMs = 0U;
  };
  static constexpr std::uint32_t kDebounceMs = 20U;
  State states_[5]{};
};
} // namespace drivers

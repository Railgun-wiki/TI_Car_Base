#pragma once
#include "Common/types.hpp"
namespace drivers {
class Keypad final {
public:
  bool pressed(car::Key key) const noexcept;
};
} // namespace drivers

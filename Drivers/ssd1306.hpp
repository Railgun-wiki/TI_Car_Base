#pragma once
#include "Common/types.hpp"
namespace drivers {
class Ssd1306 final {
public:
  car::Status begin() noexcept;
  car::Status writeLine(const char *text) noexcept;
  bool ready() const noexcept { return ready_; }

private:
  bool ready_ = false;
};
} // namespace drivers

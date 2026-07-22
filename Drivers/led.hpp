#pragma once
#include <cstdint>
namespace drivers {
class Led final {
public:
  void setStatus(std::uint8_t index, bool on) noexcept;
  void setUser(bool on) noexcept;
};
} // namespace drivers

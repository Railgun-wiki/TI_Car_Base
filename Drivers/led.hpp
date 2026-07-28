#pragma once
#include "Config/vehicle_tuning.hpp"
#include <cstdint>
namespace drivers {

enum class LedMode : std::uint8_t { Off, On, Blink };

struct LedPattern final {
  LedMode led1;
  LedMode led2;
  LedMode led3;
};

class Led final {
public:
  void setPattern(LedPattern pattern) noexcept { pattern_ = pattern; }
  void setMode(std::uint8_t index, LedMode mode) noexcept;
  LedPattern pattern() const noexcept { return pattern_; }
  void service(std::uint32_t nowMs) const noexcept;
  void setUser(bool on) noexcept;

private:
  LedPattern pattern_{LedMode::Off, LedMode::Off, LedMode::Off};
};
} // namespace drivers

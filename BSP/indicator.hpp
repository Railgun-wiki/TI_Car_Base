#pragma once
#include <cstdint>
namespace bsp {
void setStatusLed(std::uint8_t index, bool on) noexcept;
void setUserLed(bool on) noexcept;
void setBuzzer(bool on) noexcept;
} // namespace bsp

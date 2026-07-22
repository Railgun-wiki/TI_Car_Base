#pragma once
#include "Common/types.hpp"
#include <cstdint>
namespace bsp {
std::uint8_t readLineBits() noexcept;
bool keyPressed(car::Key key) noexcept;
} // namespace bsp

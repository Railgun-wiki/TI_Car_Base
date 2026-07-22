#pragma once
#include <cstddef>
namespace bsp {
bool uartTryWrite(const char *data, std::size_t length) noexcept;
}

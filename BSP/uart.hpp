#pragma once
#include <cstddef>
#include <cstdint>
namespace bsp {
bool uartTryWrite(const char *data, std::size_t length) noexcept;
bool uartTryRead(std::uint8_t &byte) noexcept;
std::uint32_t uartRxDroppedBytes() noexcept;
std::uint32_t uartTxDroppedFrames() noexcept;
} // namespace bsp

#pragma once
#include <cstdint>
namespace bsp {
void init() noexcept;
std::uint32_t millis() noexcept;
// Busy-wait via SysTick countdown. Safe before the scheduler starts and in
// init() paths where a blocking delay is acceptable (e.g. MPU calibration).
void delayMs(std::uint32_t ms) noexcept;
} // namespace bsp

#pragma once
#include "Common/types.hpp"
#include <cstddef>
#include <cstdint>
namespace bsp {
car::Status i2cWrite(std::uint8_t bus, std::uint8_t address,
                     const std::uint8_t *data, std::size_t length,
                     std::uint32_t timeoutMs = 5U) noexcept;
car::Status i2cWriteRegister(std::uint8_t bus, std::uint8_t address,
                             std::uint8_t reg, const std::uint8_t *data,
                             std::size_t length,
                             std::uint32_t timeoutMs = 5U) noexcept;
car::Status i2cReadRegister(std::uint8_t bus, std::uint8_t address,
                            std::uint8_t reg, std::uint8_t *data,
                            std::size_t length,
                            std::uint32_t timeoutMs = 5U) noexcept;

// I2C1 is reserved for the OLED. These calls never wait for a transfer; the
// completion state is produced by the I2C1 ISR after its DMA TX channel drains.
car::Status i2cOledWriteDma(std::uint8_t address, const std::uint8_t *data,
                            std::size_t length,
                            std::uint32_t timeoutMs = 5U) noexcept;
car::Status i2cOledDmaStatus() noexcept;
} // namespace bsp

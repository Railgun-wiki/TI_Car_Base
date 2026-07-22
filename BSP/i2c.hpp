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
} // namespace bsp

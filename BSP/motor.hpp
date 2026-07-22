#pragma once
#include <cstdint>
namespace bsp {
void startMotorPwm() noexcept;
void setMotorDuty(std::int16_t left, std::int16_t right) noexcept;
void stopMotors() noexcept;
} // namespace bsp

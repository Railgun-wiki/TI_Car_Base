#pragma once
#include "Common/types.hpp"
#include "Config/vehicle_tuning.hpp"

namespace bsp {
car::EncoderTicks encoderTicks() noexcept;
void resetEncoderTicks() noexcept;
bool consumeImuDataReady() noexcept;
void initializeEncoders() noexcept;
} // namespace bsp

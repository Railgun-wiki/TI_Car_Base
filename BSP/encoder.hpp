#pragma once
#include "Common/types.hpp"

// Set a wheel to -1 when its physical forward rotation decrements its raw
// quadrature count. Keep this convention aligned with positive motor PWM.
#ifndef ENCODER_LEFT_DIRECTION
#define ENCODER_LEFT_DIRECTION 1
#endif

#ifndef ENCODER_RIGHT_DIRECTION
#define ENCODER_RIGHT_DIRECTION -1
#endif

namespace bsp {
car::EncoderTicks encoderTicks() noexcept;
void resetEncoderTicks() noexcept;
bool consumeImuDataReady() noexcept;
void initializeEncoders() noexcept;
} // namespace bsp

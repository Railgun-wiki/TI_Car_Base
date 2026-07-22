#include "Drivers/encoder.hpp"
#include "BSP/encoder.hpp"
namespace drivers {
car::EncoderTicks Encoder::ticks() const noexcept {
  return ::bsp::encoderTicks();
}
void Encoder::reset() noexcept { ::bsp::resetEncoderTicks(); }
} // namespace drivers

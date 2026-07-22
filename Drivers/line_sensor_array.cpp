#include "Drivers/line_sensor_array.hpp"
#include "BSP/input.hpp"
namespace drivers {

car::LineSample LineSensorArray::read() const noexcept {
  const std::uint8_t bits = ::bsp::readLineBits();
  static constexpr std::int8_t kWeights[] = {-7, -5, -3, -1, 1, 3, 5, 7};
  std::int16_t sum = 0;
  std::int16_t count = 0;
  for (std::uint8_t index = 0; index < 8U; ++index) {
    if ((bits & (1U << index)) != 0U) {
      sum += kWeights[index];
      ++count;
    }
  }
  return {bits, static_cast<std::int16_t>(count == 0 ? 0 : sum / count),
          count != 0};
}

} // namespace drivers

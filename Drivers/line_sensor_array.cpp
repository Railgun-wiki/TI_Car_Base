#include "Drivers/line_sensor_array.hpp"
#include "BSP/input.hpp"
#include "Drivers/line_sensor_config.h"
namespace drivers {

car::LineSample LineSensorArray::read() const noexcept {
  const std::uint8_t rawBits = ::bsp::readLineBits();
  std::uint8_t bits = rawBits;
  // 统一约定 bits 中置位 = “压在线上”。若传感器板输出为低电平有效，
  // 在此一次性反转，权重/误差逻辑无需感知极性。
#if LINE_SENSOR_LINE_IS_HIGH == 0
  bits = static_cast<std::uint8_t>(~bits);
#endif
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
          count != 0, rawBits};
}

} // namespace drivers

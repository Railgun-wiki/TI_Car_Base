#include "Middlewares/telemetry.hpp"
#include <cstdio>
namespace middleware {
bool Telemetry::format(char *output, std::size_t capacity,
                       const car::LineSample &line, car::EncoderTicks ticks,
                       bool imuReady, bool oledReady) const noexcept {
  if (output == nullptr || capacity == 0U)
    return false;
  const int n = std::snprintf(
      output, capacity, "line=%02X,e=%d,enc=%ld/%ld,imu=%u,oled=%u\r\n",
      line.bits, line.error, static_cast<long>(ticks.left),
      static_cast<long>(ticks.right), imuReady ? 1U : 0U, oledReady ? 1U : 0U);
  return n > 0 && static_cast<std::size_t>(n) < capacity;
}
} // namespace middleware

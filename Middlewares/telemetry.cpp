#include "Middlewares/telemetry.hpp"

#include <cmath>
#include <cstdio>

namespace middleware {
namespace {
struct Fixed2 final {
  char sign;
  unsigned long whole;
  unsigned long fraction;
};

Fixed2 fixed2(float value) noexcept {
  if (!std::isfinite(value))
    value = 0.0F;
  const long scaled = std::lround(static_cast<double>(value) * 100.0);
  const unsigned long magnitude =
      static_cast<unsigned long>(scaled < 0 ? -scaled : scaled);
  return {scaled < 0 ? '-' : '+', magnitude / 100U, magnitude % 100U};
}
} // namespace

bool Telemetry::formatFrame(char *output, std::size_t capacity,
                            const car::LineSample &line,
                            car::EncoderTicks ticks, const car::ImuSample &imu,
                            car::WheelCommand wheels,
                            const LineFollowerConfig &lineConfig,
                            bool lineEnabled, bool imuReady,
                            std::uint32_t rxDropped,
                            std::uint32_t txDropped) const noexcept {
  if (output == nullptr || capacity == 0U)
    return false;
  const Fixed2 roll = fixed2(imu.rollDeg);
  const Fixed2 pitch = fixed2(imu.pitchDeg);
  const Fixed2 yaw = fixed2(imu.yawDeg);
  const Fixed2 kp = fixed2(lineConfig.kp);
  const Fixed2 ki = fixed2(lineConfig.ki);
  const Fixed2 kd = fixed2(lineConfig.kd);
  const int n = std::snprintf(
      output, capacity,
      "roll:%c%lu.%02lu,pitch:%c%lu.%02lu,yaw:%c%lu.%02lu,"
      "line:%u,error:%d,left:%d,right:%d,enc_l:%ld,enc_r:%ld,"
      "kp:%c%lu.%02lu,ki:%c%lu.%02lu,kd:%c%lu.%02lu,cruise:%d,"
      "run:%u,imu:%u,rx_drop:%lu,tx_drop:%lu\r\n",
      roll.sign, roll.whole, roll.fraction, pitch.sign, pitch.whole,
      pitch.fraction, yaw.sign, yaw.whole, yaw.fraction,
      static_cast<unsigned>(line.bits), static_cast<int>(line.error),
      static_cast<int>(wheels.left), static_cast<int>(wheels.right),
      static_cast<long>(ticks.left), static_cast<long>(ticks.right), kp.sign,
      kp.whole, kp.fraction, ki.sign, ki.whole, ki.fraction, kd.sign, kd.whole,
      kd.fraction, static_cast<int>(lineConfig.cruise), lineEnabled ? 1U : 0U,
      imuReady ? 1U : 0U, static_cast<unsigned long>(rxDropped),
      static_cast<unsigned long>(txDropped));
  return n > 0 && static_cast<std::size_t>(n) < capacity;
}

bool Telemetry::formatConfig(char *output, std::size_t capacity,
                             const LineFollowerConfig &config) const noexcept {
  if (output == nullptr || capacity == 0U)
    return false;
  const Fixed2 kp = fixed2(config.kp);
  const Fixed2 ki = fixed2(config.ki);
  const Fixed2 kd = fixed2(config.kd);
  const int n = std::snprintf(
      output, capacity,
      "config:LINE,kp:%c%lu.%02lu,ki:%c%lu.%02lu,kd:%c%lu.%02lu,"
      "cruise:%d\r\n",
      kp.sign, kp.whole, kp.fraction, ki.sign, ki.whole, ki.fraction, kd.sign,
      kd.whole, kd.fraction, static_cast<int>(config.cruise));
  return n > 0 && static_cast<std::size_t>(n) < capacity;
}
} // namespace middleware

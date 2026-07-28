#pragma once

#include <cstddef>
#include <cstdint>

namespace car {

enum class Status : std::uint8_t {
  Ok,
  Busy,
  Timeout,
  BusError,
  NotReady,
  InvalidArgument,
  DeviceMismatch
};

struct WheelCommand final {
  std::int16_t left;
  std::int16_t right;
};
struct VehicleCommand final {
  std::int16_t linear;
  std::int16_t angular;
};
struct LineSample final {
  // bits is normalized so a set bit always means "sensor is on the line".
  // rawBits preserves the electrical GPIO sample for commissioning.
  std::uint8_t bits;
  std::int16_t error;
  bool detected;
  std::uint8_t rawBits;
};
struct EncoderTicks final {
  std::int32_t left;
  std::int32_t right;
};
struct ImuSample final {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float rollDeg;
  float pitchDeg;
  float yawDeg;
  std::uint32_t timestampMs;
};
enum class Key : std::uint8_t { Up, Left, Down, Right, Center, None };
struct KeyEvent final {
  Key key;
  bool pressed;
  std::uint32_t timestampMs;
};

constexpr std::int16_t clampCommand(std::int32_t value) noexcept {
  return value > 1000
             ? 1000
             : (value < -1000 ? -1000 : static_cast<std::int16_t>(value));
}

} // namespace car

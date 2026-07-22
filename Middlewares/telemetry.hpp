#pragma once
#include "Common/types.hpp"
#include <cstddef>
namespace middleware {
class Telemetry final {
public:
  bool format(char *output, std::size_t capacity, const car::LineSample &line,
              car::EncoderTicks ticks, bool imuReady,
              bool oledReady) const noexcept;
};
} // namespace middleware

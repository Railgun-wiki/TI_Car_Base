#pragma once
#include "Common/types.hpp"
#include "Middlewares/line_follower.hpp"
#include <cstddef>
namespace middleware {
class Telemetry final {
public:
  bool formatFrame(char *output, std::size_t capacity,
                   const car::LineSample &line, car::EncoderTicks ticks,
                   const car::ImuSample &imu, car::WheelCommand wheels,
                   const LineFollowerConfig &lineConfig, bool lineEnabled,
                   bool imuReady, std::uint32_t rxDropped,
                   std::uint32_t txDropped) const noexcept;
  bool formatConfig(char *output, std::size_t capacity,
                    const LineFollowerConfig &lineConfig) const noexcept;
};
} // namespace middleware

#pragma once
#include "Common/types.hpp"
namespace middleware {
struct LineFollowerConfig final {
  float gain;
  std::int16_t cruise;
};
class LineFollower final {
public:
  explicit LineFollower(LineFollowerConfig config) noexcept : config_(config) {}
  car::VehicleCommand update(const car::LineSample &sample) const noexcept;

private:
  LineFollowerConfig config_;
};
} // namespace middleware

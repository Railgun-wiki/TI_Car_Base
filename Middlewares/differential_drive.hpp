#pragma once
#include "Common/types.hpp"
namespace middleware {
struct KinematicsConfig final {
  float trackWidthMeters;
};
class DifferentialDrive final {
public:
  explicit DifferentialDrive(KinematicsConfig config) noexcept
      : config_(config) {}
  car::WheelCommand mix(car::VehicleCommand command) const noexcept;

private:
  KinematicsConfig config_;
};
} // namespace middleware

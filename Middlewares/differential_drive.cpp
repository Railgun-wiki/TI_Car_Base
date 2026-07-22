#include "Middlewares/differential_drive.hpp"
namespace middleware {
car::WheelCommand
DifferentialDrive::mix(car::VehicleCommand command) const noexcept {
  const auto turn = static_cast<std::int32_t>(command.angular *
                                              config_.trackWidthMeters / 0.15F);
  return {car::clampCommand(static_cast<std::int32_t>(command.linear) - turn),
          car::clampCommand(static_cast<std::int32_t>(command.linear) + turn)};
}
} // namespace middleware

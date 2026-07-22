#include "Middlewares/line_follower.hpp"
namespace middleware {
car::VehicleCommand
LineFollower::update(const car::LineSample &sample) const noexcept {
  return sample.detected
             ? car::VehicleCommand{config_.cruise,
                                   static_cast<std::int16_t>(-config_.gain *
                                                             sample.error)}
             : car::VehicleCommand{0, 0};
}
} // namespace middleware

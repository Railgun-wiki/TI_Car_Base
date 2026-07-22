#include "Middlewares/safety_gate.hpp"
namespace middleware {
car::WheelCommand SafetyGate::apply(car::WheelCommand proposal,
                                    bool enabled) const noexcept {
  return enabled ? proposal : car::WheelCommand{0, 0};
}
} // namespace middleware

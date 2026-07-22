#pragma once
#include "Common/types.hpp"
namespace middleware {
class SafetyGate final {
public:
  car::WheelCommand apply(car::WheelCommand proposal,
                          bool enabled) const noexcept;
};
} // namespace middleware

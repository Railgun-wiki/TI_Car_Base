#pragma once
#include "Common/types.hpp"
namespace drivers {
class LineSensorArray final {
public:
  car::LineSample read() const noexcept;
};
} // namespace drivers

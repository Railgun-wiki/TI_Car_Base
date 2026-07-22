#pragma once
#include "Common/types.hpp"
namespace drivers {
class Encoder final {
public:
  car::EncoderTicks ticks() const noexcept;
  void reset() noexcept;
};
} // namespace drivers

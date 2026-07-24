#pragma once

#include <cstddef>
#include <cstdint>

namespace middleware {

enum class VofaCommandType : std::uint8_t {
  None,
  SetLinePid,
  SetCruise,
  GetConfig,
  Invalid
};

struct VofaCommand final {
  VofaCommandType type = VofaCommandType::None;
  float kp = 0.0F;
  float ki = 0.0F;
  float kd = 0.0F;
  std::int16_t cruise = 0;
};

class VofaProtocol final {
public:
  bool consume(std::uint8_t byte, VofaCommand &command) noexcept;

private:
  static constexpr std::size_t kLineCapacity = 80U;
  char line_[kLineCapacity]{};
  std::size_t length_ = 0U;
  bool overflow_ = false;
};

} // namespace middleware

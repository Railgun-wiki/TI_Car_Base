#pragma once
#include "Common/types.hpp"
#include <cstdint>
namespace drivers {
class Ssd1306 final {
public:
  car::Status begin() noexcept;
  // Queues one of the eight 21-character text rows of the 128x64 display.
  // Call service() from a low-priority superloop slot; each invocation issues
  // at most one I2C transaction.
  car::Status writeLine(std::uint8_t row, const char *text) noexcept;
  car::Status service() noexcept;
  bool ready() const noexcept { return ready_; }

private:
  static constexpr std::uint8_t kRows = 8U;
  static constexpr std::uint8_t kCharactersPerRow = 21U;
  static constexpr std::uint8_t kColumns = 128U;
  char lines_[kRows][kCharactersPerRow + 1U]{};
  bool dirty_[kRows]{};
  std::uint8_t activeRow_ = 0U;
  std::uint8_t phase_ = 0U;
  std::uint8_t column_ = 0U;
  std::uint8_t txPacket_[8]{};
  std::uint8_t pendingColumns_ = 0U;
  bool transferPending_ = false;
  bool ready_ = false;
};
} // namespace drivers

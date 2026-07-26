#include "Drivers/ssd1306.hpp"
#include "BSP/i2c.hpp"
namespace {
car::Status command(std::uint8_t commandByte) noexcept {
  const std::uint8_t packet[] = {0x00U, commandByte};
  return bsp::i2cWrite(1U, 0x3CU, packet, sizeof(packet));
}
const std::uint8_t *glyph(char character) noexcept {
  static constexpr std::uint8_t kGlyphs[][5] = {{0, 0, 0, 0, 0},
                                                {0x3E, 0x51, 0x49, 0x45, 0x3E},
                                                {0, 0x42, 0x7F, 0x40, 0},
                                                {0x42, 0x61, 0x51, 0x49, 0x46},
                                                {0x21, 0x41, 0x45, 0x4B, 0x31},
                                                {0x18, 0x14, 0x12, 0x7F, 0x10},
                                                {0x27, 0x45, 0x45, 0x45, 0x39},
                                                {0x3C, 0x4A, 0x49, 0x49, 0x30},
                                                {0x01, 0x71, 0x09, 0x05, 0x03},
                                                {0x36, 0x49, 0x49, 0x49, 0x36},
                                                {0x06, 0x49, 0x49, 0x29, 0x1E},
                                                {0, 0x36, 0x36, 0, 0},
                                                {0x08, 0x08, 0x08, 0x08, 0x08},
                                                {0x7E, 0x11, 0x11, 0x11, 0x7E},
                                                {0x7F, 0x49, 0x49, 0x49, 0x36},
                                                {0x3E, 0x41, 0x41, 0x41, 0x22},
                                                {0x7F, 0x41, 0x41, 0x22, 0x1C},
                                                {0x7F, 0x49, 0x49, 0x49, 0x41},
                                                {0x7F, 0x09, 0x09, 0x09, 0x01},
                                                {0x3E, 0x41, 0x49, 0x49, 0x7A},
                                                {0x7F, 0x08, 0x08, 0x08, 0x7F},
                                                {0, 0x41, 0x7F, 0x41, 0},
                                                {0x20, 0x40, 0x41, 0x3F, 0x01},
                                                {0x7F, 0x08, 0x14, 0x22, 0x41},
                                                {0x7F, 0x40, 0x40, 0x40, 0x40},
                                                {0x7F, 0x02, 0x0C, 0x02, 0x7F},
                                                {0x7F, 0x04, 0x08, 0x10, 0x7F},
                                                {0x3E, 0x41, 0x41, 0x41, 0x3E},
                                                {0x7F, 0x09, 0x09, 0x09, 0x06},
                                                {0x3E, 0x41, 0x51, 0x21, 0x5E},
                                                {0x7F, 0x09, 0x19, 0x29, 0x46},
                                                {0x46, 0x49, 0x49, 0x49, 0x31},
                                                {0x01, 0x01, 0x7F, 0x01, 0x01},
                                                {0x3F, 0x40, 0x40, 0x40, 0x3F},
                                                {0x1F, 0x20, 0x40, 0x20, 0x1F},
                                                {0x7F, 0x20, 0x18, 0x20, 0x7F},
                                                {0x63, 0x14, 0x08, 0x14, 0x63},
                                                {0x03, 0x04, 0x78, 0x04, 0x03},
                                                {0x61, 0x51, 0x49, 0x45, 0x43}};
  if (character >= '0' && character <= '9')
    return kGlyphs[1U + static_cast<std::uint8_t>(character - '0')];
  if (character == ':')
    return kGlyphs[11U];
  if (character == '-')
    return kGlyphs[12U];
  if (character >= 'A' && character <= 'Z')
    return kGlyphs[13U + static_cast<std::uint8_t>(character - 'A')];
  return kGlyphs[0U];
}
} // namespace

namespace drivers {

car::Status Ssd1306::begin() noexcept {
  // 128x64 SSD1306 power-up sequence. 0x8D/0x14 (charge pump) is decisive:
  // without it every command ACKs but the glass stays dark.
  static constexpr std::uint8_t kInit[] = {
      0xAE,       // display off
      0xD5, 0x80, // clock divide / oscillator
      0xA8, 0x3F, // multiplex 1/64
      0xD3, 0x00, // display offset
      0x40,       // start line
      0x8D, 0x14, // charge pump enable (required for panel light)
      0x20, 0x02, // page addressing (matches service())
      0xA1,       // segment remap
      0xC8,       // COM scan direction
      0xDA, 0x12, // COM pins (128x64)
      0x81, 0xCF, // contrast
      0xD9, 0xF1, // precharge period
      0xDB, 0x40, // VCOMH deselect
      0xA4,       // display from RAM
      0xA6,       // normal (non-inverted)
      0xAF        // display on
  };
  for (const std::uint8_t commandByte : kInit) {
    if (command(commandByte) != car::Status::Ok)
      return car::Status::BusError;
  }
  for (std::uint8_t row = 0U; row < kRows; ++row)
    dirty_[row] = true;
  ready_ = true;
  return car::Status::Ok;
}

car::Status Ssd1306::writeLine(std::uint8_t row, const char *text) noexcept {
  if (!ready_ || text == nullptr || row >= kRows)
    return car::Status::NotReady;
  std::uint8_t index = 0U;
  while (index < kCharactersPerRow && text[index] != '\0') {
    lines_[row][index] = text[index];
    ++index;
  }
  while (index < kCharactersPerRow)
    lines_[row][index++] = ' ';
  lines_[row][kCharactersPerRow] = '\0';
  dirty_[row] = true;
  return car::Status::Ok;
}

car::Status Ssd1306::service() noexcept {
  if (!ready_)
    return car::Status::NotReady;
  if (transferPending_) {
    const car::Status status = bsp::i2cOledDmaStatus();
    if (status == car::Status::Busy)
      return status;
    transferPending_ = false;
    if (status != car::Status::Ok)
      return status;
    if (phase_ < 3U) {
      ++phase_;
    } else {
      column_ = static_cast<std::uint8_t>(column_ + pendingColumns_);
      if (column_ >= kColumns) {
        dirty_[activeRow_] = false;
        phase_ = 0U;
        column_ = 0U;
      }
    }
    return car::Status::Ok;
  }
  if (!dirty_[activeRow_]) {
    for (std::uint8_t row = 0U; row < kRows; ++row) {
      const std::uint8_t candidate = (activeRow_ + row + 1U) % kRows;
      if (dirty_[candidate]) {
        activeRow_ = candidate;
        phase_ = 0U;
        column_ = 0U;
        break;
      }
    }
    // Nothing to refresh this cycle — the framebuffer is fully synced. Return
    // Ok (not Busy) so callers can distinguish "idle" from "transfer pending".
    if (!dirty_[activeRow_])
      return car::Status::Ok;
  }

  if (phase_ < 3U) {
    const std::uint8_t commands[] = {
        static_cast<std::uint8_t>(0xB0U + activeRow_), 0x00U, 0x10U};
    txPacket_[0] = 0x00U;
    txPacket_[1] = commands[phase_];
    const car::Status status = bsp::i2cOledWriteDma(0x3CU, txPacket_, 2U, 2U);
    if (status != car::Status::Ok)
      return status;
    transferPending_ = true;
    return car::Status::Busy;
  }

  txPacket_[0] = 0x40U;
  const std::uint8_t count = static_cast<std::uint8_t>(
      kColumns - column_ > 7U ? 7U : kColumns - column_);
  for (std::uint8_t index = 0U; index < count; ++index) {
    const std::uint8_t pixel = static_cast<std::uint8_t>(column_ + index);
    if (pixel >= kCharactersPerRow * 6U) {
      txPacket_[1U + index] = 0U;
      continue;
    }
    const std::uint8_t glyphColumn = pixel % 6U;
    txPacket_[1U + index] =
        glyphColumn == 5U ? 0U
                          : glyph(lines_[activeRow_][pixel / 6U])[glyphColumn];
  }
  const car::Status status =
      bsp::i2cOledWriteDma(0x3CU, txPacket_, count + 1U, 2U);
  if (status != car::Status::Ok)
    return status;
  pendingColumns_ = count;
  transferPending_ = true;
  return car::Status::Busy;
}

} // namespace drivers

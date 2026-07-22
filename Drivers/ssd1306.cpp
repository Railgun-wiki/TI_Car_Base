#include "Drivers/ssd1306.hpp"
#include "BSP/i2c.hpp"
namespace {
car::Status command(std::uint8_t commandByte) noexcept {
  const std::uint8_t packet[] = {0x00U, commandByte};
  return bsp::i2cWrite(1U, 0x3CU, packet, sizeof(packet));
}
const std::uint8_t *glyph(char character) noexcept {
  static constexpr std::uint8_t kBlank[] = {0, 0, 0, 0, 0};
  static constexpr std::uint8_t kA[] = {0x7E, 0x11, 0x11, 0x11, 0x7E};
  static constexpr std::uint8_t kC[] = {0x3E, 0x41, 0x41, 0x41, 0x22};
  static constexpr std::uint8_t kD[] = {0x7F, 0x41, 0x41, 0x22, 0x1C};
  static constexpr std::uint8_t kE[] = {0x7F, 0x49, 0x49, 0x49, 0x41};
  static constexpr std::uint8_t kI[] = {0, 0x41, 0x7F, 0x41, 0};
  static constexpr std::uint8_t kM[] = {0x7F, 2, 12, 2, 0x7F};
  static constexpr std::uint8_t kO[] = {0x3E, 0x41, 0x41, 0x41, 0x3E};
  static constexpr std::uint8_t kR[] = {0x7F, 9, 25, 41, 70};
  static constexpr std::uint8_t kU[] = {0x3F, 0x40, 0x40, 0x40, 0x3F};
  static constexpr std::uint8_t kY[] = {7, 8, 0x70, 8, 7};
  switch (character) {
  case 'A':
    return kA;
  case 'C':
    return kC;
  case 'D':
    return kD;
  case 'E':
    return kE;
  case 'I':
    return kI;
  case 'M':
    return kM;
  case 'O':
    return kO;
  case 'R':
    return kR;
  case 'U':
    return kU;
  case 'Y':
    return kY;
  default:
    return kBlank;
  }
}
} // namespace

namespace drivers {

car::Status Ssd1306::begin() noexcept {
  static constexpr std::uint8_t kInit[] = {0xAE, 0x20, 0x00, 0xA8, 0x3F, 0xAF};
  for (const std::uint8_t commandByte : kInit) {
    if (command(commandByte) != car::Status::Ok)
      return car::Status::BusError;
  }
  ready_ = true;
  return car::Status::Ok;
}

car::Status Ssd1306::writeLine(const char *text) noexcept {
  if (!ready_ || text == nullptr)
    return car::Status::NotReady;
  if (command(0xB0U) != car::Status::Ok || command(0x00U) != car::Status::Ok ||
      command(0x10U) != car::Status::Ok)
    return car::Status::BusError;

  std::uint8_t packet[8] = {0x40U};
  std::size_t dataLength = 0U;
  while (*text != '\0') {
    const std::uint8_t *pixels = glyph(*text++);
    for (std::uint8_t column = 0U; column < 6U; ++column) {
      packet[1U + dataLength++] = column == 5U ? 0U : pixels[column];
      if (dataLength == 7U) {
        const car::Status status =
            bsp::i2cWrite(1U, 0x3CU, packet, sizeof(packet));
        if (status != car::Status::Ok)
          return status;
        dataLength = 0U;
      }
    }
  }
  return dataLength == 0U ? car::Status::Ok
                          : bsp::i2cWrite(1U, 0x3CU, packet, dataLength + 1U);
}

} // namespace drivers

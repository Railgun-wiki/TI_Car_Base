#include "Drivers/mpu6050.hpp"
#include "BSP/i2c.hpp"
namespace {
constexpr std::uint8_t kAddress = 0x68U;
constexpr std::uint8_t kWhoAmI = 0x75U;
constexpr std::uint8_t kAccelXoutH = 0x3BU;
} // namespace

namespace drivers {

car::Status Mpu6050::begin() noexcept {
  std::uint8_t id = 0U;
  const car::Status status =
      ::bsp::i2cReadRegister(0U, kAddress, kWhoAmI, &id, 1U);
  ready_ = status == car::Status::Ok && id == 0x68U;
  return status != car::Status::Ok
             ? status
             : (ready_ ? car::Status::Ok : car::Status::DeviceMismatch);
}

car::Status Mpu6050::poll(car::ImuSample &sample) noexcept {
  std::uint8_t raw[14]{};
  const car::Status status =
      ::bsp::i2cReadRegister(0U, kAddress, kAccelXoutH, raw, sizeof(raw));
  if (status != car::Status::Ok)
    return status;
  const auto readInt16 = [](const std::uint8_t *bytes) {
    return static_cast<std::int16_t>(
        (static_cast<std::uint16_t>(bytes[0]) << 8U) | bytes[1]);
  };
  sample = {readInt16(raw) / 16384.0F,
            readInt16(raw + 2) / 16384.0F,
            readInt16(raw + 4) / 16384.0F,
            readInt16(raw + 8) / 131.0F,
            readInt16(raw + 10) / 131.0F,
            readInt16(raw + 12) / 131.0F,
            0.0F,
            0.0F,
            0.0F,
            0U};
  return car::Status::Ok;
}

} // namespace drivers

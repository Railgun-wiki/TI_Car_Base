#include "Drivers/mpu6050.hpp"
#include "BSP/i2c.hpp"
#include "BSP/system.hpp"
namespace {
constexpr std::uint8_t kAddress = 0x68U;
constexpr std::uint8_t kWhoAmI = 0x75U;
constexpr std::uint8_t kPowerMgmt1 = 0x6BU;
constexpr std::uint8_t kAccelXoutH = 0x3BU;
} // namespace

namespace drivers {

car::Status Mpu6050::begin() noexcept {
  // The MPU powers up asleep. DMP initialization wakes it internally, whereas
  // the raw/software-filter backend must do so explicitly.
  constexpr std::uint8_t kWake = 0x00U;
  car::Status status =
      ::bsp::i2cWriteRegister(0U, kAddress, kPowerMgmt1, &kWake, 1U);
  if (status != car::Status::Ok)
    return status;
  std::uint8_t id = 0U;
  status = ::bsp::i2cReadRegister(0U, kAddress, kWhoAmI, &id, 1U);
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
            readInt16(raw + 8) / 131.0F - gyroBiasX_,
            readInt16(raw + 10) / 131.0F - gyroBiasY_,
            readInt16(raw + 12) / 131.0F - gyroBiasZ_,
            0.0F,
            0.0F,
            0.0F,
            0U};
  return car::Status::Ok;
}

car::Status Mpu6050::calibrateGyroBias(std::uint16_t samples,
                                       std::uint16_t delayMs) noexcept {
  if (samples == 0U)
    return car::Status::InvalidArgument;
  float sumX = 0.0F, sumY = 0.0F, sumZ = 0.0F;
  for (std::uint16_t i = 0U; i < samples; ++i) {
    std::uint8_t raw[14]{};
    if (::bsp::i2cReadRegister(0U, kAddress, kAccelXoutH, raw, sizeof(raw)) !=
        car::Status::Ok)
      return car::Status::BusError;
    const auto readInt16 = [](const std::uint8_t *bytes) {
      return static_cast<std::int16_t>(
          (static_cast<std::uint16_t>(bytes[0]) << 8U) | bytes[1]);
    };
    sumX += readInt16(raw + 8) / 131.0F;
    sumY += readInt16(raw + 10) / 131.0F;
    sumZ += readInt16(raw + 12) / 131.0F;
    const std::uint32_t start = bsp::millis();
    while (static_cast<std::uint32_t>(bsp::millis() - start) < delayMs) {
    }
  }
  gyroBiasX_ = sumX / static_cast<float>(samples);
  gyroBiasY_ = sumY / static_cast<float>(samples);
  gyroBiasZ_ = sumZ / static_cast<float>(samples);
  return car::Status::Ok;
}

} // namespace drivers

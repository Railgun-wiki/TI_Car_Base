#include "Drivers/mpu6050.hpp"
#include "BSP/i2c.hpp"
#include "BSP/system.hpp"
#include <initializer_list>
namespace {
constexpr std::uint8_t kDefaultAddress = 0x68U;
constexpr std::uint8_t kAltAddress = 0x69U;
constexpr std::uint8_t kWhoAmI = 0x75U;
constexpr std::uint8_t kPowerMgmt1 = 0x6BU;
// CONFIG (DLPF) and SMPLRT_DIV configure the raw/software-filter backend.
// The DMP backend reconfigures these internally, so writes are skipped when
// the application selected ATTITUDE_BACKEND_DMP.
constexpr std::uint8_t kSmplrtDiv = 0x19U;
constexpr std::uint8_t kConfig = 0x1AU;
constexpr std::uint8_t kGyroConfig = 0x1BU;
constexpr std::uint8_t kAccelConfig = 0x1CU;
constexpr std::uint8_t kAccelXoutH = 0x3BU;

car::Status writeReg(std::uint8_t addr, std::uint8_t reg, std::uint8_t value) {
  return ::bsp::i2cWriteRegister(0U, addr, reg, &value, 1U);
}
} // namespace

namespace drivers {

car::Status Mpu6050::begin() noexcept {
  // Resolve the I2C address: use the explicit override if non-zero, otherwise
  // probe the two legal MPU-6050 addresses (AD0 low/high).
  if (address_ == 0U) {
    for (const std::uint8_t probe : {kDefaultAddress, kAltAddress}) {
      std::uint8_t id = 0U;
      if (::bsp::i2cReadRegister(0U, probe, kWhoAmI, &id, 1U) ==
              car::Status::Ok &&
          id == 0x68U) {
        address_ = probe;
        break;
      }
    }
    if (address_ == 0U) {
      ready_ = false;
      return car::Status::DeviceMismatch;
    }
  }

  // The MPU powers up asleep. DMP initialization wakes it internally, whereas
  // the raw/software-filter backend must do so explicitly.
  constexpr std::uint8_t kWake = 0x00U;
  car::Status status = writeReg(address_, kPowerMgmt1, kWake);
  if (status != car::Status::Ok)
    return status;
  std::uint8_t id = 0U;
  status = ::bsp::i2cReadRegister(0U, address_, kWhoAmI, &id, 1U);
  if (status != car::Status::Ok)
    return status;
  if (id != 0x68U) {
    ready_ = false;
    return car::Status::DeviceMismatch;
  }

  // Apply sane sensor defaults for the raw/software-filter backend. DLPF=3
  // (44 Hz accel/42 Hz gyro bandwidth) suppresses aliasing; SMPLRT_DIV=9
  // yields ~100 Hz, matching the 10 ms poll cadence assumed by AttitudeFilter.
  // Ranges stay at ±2 g / ±250 dps, so poll() sensitivities are unchanged.
  status = writeReg(address_, kConfig, 0x03U);
  if (status != car::Status::Ok)
    return status;
  status = writeReg(address_, kSmplrtDiv, 0x09U);
  if (status != car::Status::Ok)
    return status;
  (void)writeReg(address_, kGyroConfig, 0x00U);  // ±250 dps
  (void)writeReg(address_, kAccelConfig, 0x00U); // ±2 g

  ready_ = true;
  return car::Status::Ok;
}

car::Status Mpu6050::poll(car::ImuSample &sample) noexcept {
  std::uint8_t raw[14]{};
  const car::Status status =
      ::bsp::i2cReadRegister(0U, address_, kAccelXoutH, raw, sizeof(raw));
  if (status != car::Status::Ok)
    return status;
  const auto readInt16 = [](const std::uint8_t *bytes) {
    return static_cast<std::int16_t>(
        (static_cast<std::uint16_t>(bytes[0]) << 8U) | bytes[1]);
  };
  // raw[6..7] = TEMP_OUT_H/L (0x41/0x42); unused by the current pipeline.
  sample = {readInt16(raw) / kAccelSensitivity,
            readInt16(raw + 2) / kAccelSensitivity,
            readInt16(raw + 4) / kAccelSensitivity,
            readInt16(raw + 8) / kGyroSensitivity - gyroBiasX_,
            readInt16(raw + 10) / kGyroSensitivity - gyroBiasY_,
            readInt16(raw + 12) / kGyroSensitivity - gyroBiasZ_,
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
    if (::bsp::i2cReadRegister(0U, address_, kAccelXoutH, raw, sizeof(raw)) !=
        car::Status::Ok)
      return car::Status::BusError;
    const auto readInt16 = [](const std::uint8_t *bytes) {
      return static_cast<std::int16_t>(
          (static_cast<std::uint16_t>(bytes[0]) << 8U) | bytes[1]);
    };
    sumX += readInt16(raw + 8) / kGyroSensitivity;
    sumY += readInt16(raw + 10) / kGyroSensitivity;
    sumZ += readInt16(raw + 12) / kGyroSensitivity;
    bsp::delayMs(static_cast<std::uint32_t>(delayMs));
  }
  gyroBiasX_ = sumX / static_cast<float>(samples);
  gyroBiasY_ = sumY / static_cast<float>(samples);
  gyroBiasZ_ = sumZ / static_cast<float>(samples);
  return car::Status::Ok;
}

void Mpu6050::updateGyroBiasFromStationarySample(const car::ImuSample &sample,
                                                 float dtSeconds) noexcept {
  if (dtSeconds <= 0.0F || dtSeconds > 0.1F)
    return;
  // A slow 20-second time constant tracks thermal drift without reacting to
  // individual samples. sample.g* already has the current bias removed, so
  // adding its residual moves the stored bias toward the stationary reading.
  constexpr float kBiasTimeConstantSeconds = 20.0F;
  const float gain = dtSeconds / kBiasTimeConstantSeconds;
  gyroBiasX_ += sample.gx * gain;
  gyroBiasY_ += sample.gy * gain;
  gyroBiasZ_ += sample.gz * gain;
}

} // namespace drivers

#include "Drivers/bmi270.hpp"
#include "BSP/i2c.hpp"
#include "BSP/system.hpp"
#include "ThirdParty/bmi270/bmi270_config_file.h"
#include <cmath>
#include <cstring>
#include <initializer_list>

namespace {
// BMI270 I2C address options (SDO low/high). Same addresses as MPU6050; the
// chips are distinguished by CHIP_ID, not by address.
constexpr std::uint8_t kDefaultAddress = 0x68U;
constexpr std::uint8_t kAltAddress = 0x69U;

// Register map (BMI270 datasheet). CHIP_ID is at 0x00 and returns 0x24.
constexpr std::uint8_t kChipIdReg = 0x00U;
constexpr std::uint8_t kExpectedChipId = 0x24U;
constexpr std::uint8_t kAccXLsb = 0x0CU;  // accel X/Y/Z LSB-first, 6 bytes
constexpr std::uint8_t kGyroXLsb = 0x12U; // gyro X/Y/Z LSB-first, 6 bytes
constexpr std::uint8_t kInternalStatus = 0x21U;
constexpr std::uint8_t kAccConf = 0x40U;
constexpr std::uint8_t kAccRange = 0x41U;
constexpr std::uint8_t kGyrConf = 0x42U;
constexpr std::uint8_t kGyrRange = 0x43U;
constexpr std::uint8_t kInitCtrl = 0x59U;
constexpr std::uint8_t kInitData = 0x5EU;
constexpr std::uint8_t kIfConf = 0x6BU;
constexpr std::uint8_t kNvConf = 0x70U;
constexpr std::uint8_t kPwrConf = 0x7CU;
constexpr std::uint8_t kCmd = 0x7DU;

// Range selections: ±2 g (16384 LSB/g), ±2000 dps (16.4 LSB/(deg/s)).
// Accel uses ±2 g for tilt sensitivity; gyro uses ±2000 dps to tolerate car
// vibration without saturating. NOTE: differs from MPU6050's ±250 dps by 8x,
// so AttitudeFilter tuning may need re-scaling on hardware.
constexpr std::uint8_t kAccRange2g = 0x00U;
constexpr std::uint8_t kGyrRange2000dps = 0x00U;
// ODR = 100 Hz (0x08) for both, matching AttitudeFilter's 10 ms poll cadence.
// 3 dB filter enabled via the filter-performance bits (0x20 | 0x80).
constexpr std::uint8_t kAccConfValue = 0x08U | 0x20U | 0x80U;
constexpr std::uint8_t kGyrConfValue = 0x08U | 0x20U | 0x80U;

// Sensitivities for the selected ranges.
constexpr float kAccelSensitivity = 16384.0F; // LSB/g at ±2 g
constexpr float kGyroSensitivity = 16.4F;     // LSB/(deg/s) at ±2000 dps

// The 8192-byte config blob takes ~200 ms to burst-write at 400 kHz; the BSP
// default 5 ms timeout cannot cover it. Use a generous bound.
constexpr std::uint32_t kConfigLoadTimeoutMs = 500U;

car::Status writeReg(std::uint8_t addr, std::uint8_t reg, std::uint8_t value) {
  return ::bsp::i2cWriteRegister(0U, addr, reg, &value, 1U);
}
} // namespace

namespace drivers {

car::Status Bmi270::begin() noexcept {
  // Resolve the I2C address: explicit override, else probe both legal BMI270
  // addresses via CHIP_ID. Same addresses as MPU6050 but distinguished by ID.
  if (address_ == 0U) {
    for (const std::uint8_t probe : {kDefaultAddress, kAltAddress}) {
      std::uint8_t id = 0U;
      if (::bsp::i2cReadRegister(0U, probe, kChipIdReg, &id, 1U) ==
              car::Status::Ok &&
          id == kExpectedChipId) {
        address_ = probe;
        break;
      }
    }
    if (address_ == 0U) {
      ready_ = false;
      return car::Status::DeviceMismatch;
    }
  }

  // Disable advanced power-save, then load the 8192-byte config RAM blob.
  ::bsp::delayMs(100);
  car::Status status = writeReg(address_, kPwrConf, 0x00U);
  if (status != car::Status::Ok)
    return status;
  ::bsp::delayMs(2);

  status = writeReg(address_, kInitCtrl, 0x00U);
  if (status != car::Status::Ok)
    return status;
  // Burst-write the full config file. The BSP fills the I2C TX FIFO in chunks,
  // so no driver-side segmentation is needed; only the timeout must cover the
  // whole transfer.
  status = ::bsp::i2cWriteRegister(0U, address_, kInitData, bmi270_config_file,
                                   8192U, kConfigLoadTimeoutMs);
  if (status != car::Status::Ok)
    return status;
  status = writeReg(address_, kInitCtrl, 0x01U);
  if (status != car::Status::Ok)
    return status;
  ::bsp::delayMs(40);

  // INTERNAL_STATUS nonzero signals the config was accepted.
  std::uint8_t internalStatus = 0U;
  status = ::bsp::i2cReadRegister(0U, address_, kInternalStatus,
                                  &internalStatus, 1U);
  if (status != car::Status::Ok)
    return status;
  if (internalStatus == 0U) {
    ready_ = false;
    return car::Status::BusError;
  }

  // Enable accel + gyro + temp, then apply I2C-mode interface and range/ODR.
  (void)writeReg(address_, kCmd, 0x0EU);    // ACC_EN | GYR_EN | TEMP_EN
  (void)writeReg(address_, kNvConf, 0x00U); // I2C mode (SPI bit off)
  (void)writeReg(address_, kIfConf, 0x00U);
  (void)writeReg(address_, kGyrRange, kGyrRange2000dps);
  (void)writeReg(address_, kGyrConf, kGyrConfValue);
  (void)writeReg(address_, kAccRange, kAccRange2g);
  (void)writeReg(address_, kAccConf, kAccConfValue);
  ::bsp::delayMs(10);

  // Final CHIP_ID sanity check.
  std::uint8_t id = 0U;
  status = ::bsp::i2cReadRegister(0U, address_, kChipIdReg, &id, 1U);
  if (status != car::Status::Ok)
    return status;
  if (id != kExpectedChipId) {
    ready_ = false;
    return car::Status::DeviceMismatch;
  }

  ready_ = true;
  return car::Status::Ok;
}

car::Status Bmi270::poll(car::ImuSample &sample) noexcept {
  // BMI270 data registers are little-endian, LSB-first (unlike MPU6050's
  // big-endian MSB-first layout).
  std::uint8_t acc[6]{};
  std::uint8_t gyr[6]{};
  car::Status status =
      ::bsp::i2cReadRegister(0U, address_, kAccXLsb, acc, sizeof(acc));
  if (status != car::Status::Ok)
    return status;
  status = ::bsp::i2cReadRegister(0U, address_, kGyroXLsb, gyr, sizeof(gyr));
  if (status != car::Status::Ok)
    return status;

  const auto readInt16LE = [](const std::uint8_t *bytes) {
    return static_cast<std::int16_t>(
        static_cast<std::uint16_t>(bytes[0]) |
        (static_cast<std::uint16_t>(bytes[1]) << 8U));
  };

  sample = {readInt16LE(acc) / kAccelSensitivity,
            readInt16LE(acc + 2) / kAccelSensitivity,
            readInt16LE(acc + 4) / kAccelSensitivity,
            readInt16LE(gyr) / kGyroSensitivity - gyroBiasX_,
            readInt16LE(gyr + 2) / kGyroSensitivity - gyroBiasY_,
            readInt16LE(gyr + 4) / kGyroSensitivity - gyroBiasZ_,
            0.0F,
            0.0F,
            0.0F,
            0U};
#if BMI270_ONBOARD_FUSION
  // Drives roll/pitch/yaw from the just-read sample. dt matches the ~100 Hz
  // data-ready cadence; adjust if the INT rate is changed on hardware.
  updateMahony(sample, 0.01F);
#else
  // Euler left zero; computed upstream by ImuReader + AttitudeFilter.
#endif
  return car::Status::Ok;
}

car::Status Bmi270::calibrateGyroBias(std::uint16_t samples,
                                      std::uint16_t delayMs) noexcept {
  if (samples == 0U)
    return car::Status::InvalidArgument;
  float sumX = 0.0F, sumY = 0.0F, sumZ = 0.0F;
  for (std::uint16_t i = 0U; i < samples; ++i) {
    std::uint8_t gyr[6]{};
    if (::bsp::i2cReadRegister(0U, address_, kGyroXLsb, gyr, sizeof(gyr)) !=
        car::Status::Ok)
      return car::Status::BusError;
    const auto readInt16LE = [](const std::uint8_t *bytes) {
      return static_cast<std::int16_t>(
          static_cast<std::uint16_t>(bytes[0]) |
          (static_cast<std::uint16_t>(bytes[1]) << 8U));
    };
    sumX += readInt16LE(gyr) / kGyroSensitivity;
    sumY += readInt16LE(gyr + 2) / kGyroSensitivity;
    sumZ += readInt16LE(gyr + 4) / kGyroSensitivity;
    ::bsp::delayMs(static_cast<std::uint32_t>(delayMs));
  }
  gyroBiasX_ = sumX / static_cast<float>(samples);
  gyroBiasY_ = sumY / static_cast<float>(samples);
  gyroBiasZ_ = sumZ / static_cast<float>(samples);
  return car::Status::Ok;
}

#if BMI270_ONBOARD_FUSION
float Bmi270::fastInvSqrt(float x) noexcept {
  // Quake III Q_rsqrt; Cortex-M0+ has no FPU, so avoid sqrtf. The ~1% error
  // is absorbed by accelerometer normalization. memcpy avoids strict-aliasing
  // UB (matches AttitudeFilter's approach).
  long i;
  std::memcpy(&i, &x, sizeof(i));
  i = 0x5f3759df - (i >> 1);
  float y;
  std::memcpy(&y, &i, sizeof(y));
  y = y * (1.5F - (0.5F * x * y * y));
  return y;
}

void Bmi270::updateMahony(car::ImuSample &sample, float dt) noexcept {
  const float kRadPerDeg = 0.0174532925F;
  const float kDegPerRad = 57.2957795F;
  const float gx = sample.gx * kRadPerDeg;
  const float gy = sample.gy * kRadPerDeg;
  const float gz = sample.gz * kRadPerDeg;
  const float normSq =
      sample.ax * sample.ax + sample.ay * sample.ay + sample.az * sample.az;
  // Degenerate input (free-fall or dropped frame): integrate gyro only.
  const bool accValid = normSq > 1e-9F;
  const float normRecip = accValid ? fastInvSqrt(normSq) : 0.0F;
  const float ax = accValid ? sample.ax * normRecip : 0.0F;
  const float ay = accValid ? sample.ay * normRecip : 0.0F;
  const float az = accValid ? sample.az * normRecip : 1.0F;

  // Estimated gravity direction from the current quaternion.
  const float vx = 2.0F * (q1_ * q3_ - q0_ * q2_);
  const float vy = 2.0F * (q0_ * q1_ + q2_ * q3_);
  const float vz = q0_ * q0_ - q1_ * q1_ - q2_ * q2_ + q3_ * q3_;

  // Cross-product error between measured and estimated gravity.
  const float ex = accValid ? (ay * vz - az * vy) : 0.0F;
  const float ey = accValid ? (az * vx - ax * vz) : 0.0F;
  const float ez = accValid ? (ax * vy - ay * vx) : 0.0F;

  const float halfT = 0.5F * dt;
  // PI feedback (SJTU-AuTop defaults; may need re-tuning for ±2000 dps range).
  constexpr float kKp = 0.17F;
  constexpr float kKi = 0.004F;
  iEx_ += halfT * ex;
  iEy_ += halfT * ey;
  iEz_ += halfT * ez;
  const float ddx = kKp * ex + kKi * iEx_;
  const float ddy = kKp * ey + kKi * iEy_;
  const float ddz = kKp * ez + kKi * iEz_;

  // First-order quaternion integration of corrected gyro.
  const float dq0 = halfT * (-q1_ * gx - q2_ * gy - q3_ * gz);
  const float dq1 = halfT * (q0_ * gx + q2_ * gz - q3_ * gy);
  const float dq2 = halfT * (q0_ * gy - q1_ * gz + q3_ * gx);
  const float dq3 = halfT * (q0_ * gz + q1_ * gy - q2_ * gx);
  q0_ += dq0 - (ddx * q1_ + ddy * q2_ + ddz * q3_) * halfT;
  q1_ += dq1 + (ddx * q0_ + ddy * q3_ - ddz * q2_) * halfT;
  q2_ += dq2 - (ddx * q3_ - ddy * q0_ + ddz * q1_) * halfT;
  q3_ += dq3 + (ddx * q2_ - ddy * q1_ - ddz * q0_) * halfT;
  const float recip =
      fastInvSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
  q0_ *= recip;
  q1_ *= recip;
  q2_ *= recip;
  q3_ *= recip;

  // Euler angles (yaw kept continuous to match AttitudeFilter's convention).
  sample.rollDeg = std::atan2(2.0F * (q0_ * q1_ + q2_ * q3_),
                              1.0F - 2.0F * (q1_ * q1_ + q2_ * q2_)) *
                   kDegPerRad;
  sample.pitchDeg =
      std::asin(
          std::fmax(-1.0F, std::fmin(1.0F, 2.0F * (q0_ * q2_ - q3_ * q1_)))) *
      kDegPerRad;
  sample.yawDeg = std::atan2(2.0F * (q0_ * q3_ + q1_ * q2_),
                             1.0F - 2.0F * (q2_ * q2_ + q3_ * q3_)) *
                  kDegPerRad;
}
#endif // BMI270_ONBOARD_FUSION

} // namespace drivers

#include "Drivers/mpu6050_dmp.hpp"

#include <cmath>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wextern-c-compat"
extern "C" {
#include "ThirdParty/eMPL/inv_mpu.h"
#include "ThirdParty/eMPL/inv_mpu_dmp_motion_driver.h"
}
#pragma clang diagnostic pop

namespace {
constexpr float kQ30 = 1073741824.0F;
constexpr float kDegrees = 57.2957795F;
unsigned short orientationScalar() noexcept {
  // Vehicle frame: X forward, Y left, Z up. Confirm after physical mounting.
  return 0U | (1U << 3U) | (2U << 6U);
}
} // namespace

namespace drivers {

car::Status Mpu6050Dmp::begin() noexcept {
  ready_ = false;
  due_ = false;
  if (mpu_init(nullptr) != 0 ||
      mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0 ||
      mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0 ||
      mpu_set_sample_rate(100U) != 0 || mpu_set_int_level(0U) != 0 ||
      mpu_set_int_latched(0U) != 0 || dmp_load_motion_driver_firmware() != 0 ||
      dmp_set_orientation(orientationScalar()) != 0 ||
      dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_CAL_GYRO |
                         DMP_FEATURE_GYRO_CAL) != 0 ||
      dmp_set_fifo_rate(100U) != 0 ||
      dmp_set_interrupt_mode(DMP_INT_CONTINUOUS) != 0 ||
      mpu_set_dmp_state(1U) != 0)
    return car::Status::BusError;
  ready_ = true;
  return car::Status::Ok;
}

car::Status Mpu6050Dmp::poll(car::ImuSample &sample) noexcept {
  if (!ready_)
    return car::Status::NotReady;
  if (!due_)
    return car::Status::Busy;
  due_ = false;
  short interrupt = 0;
  if (mpu_get_int_status(&interrupt) != 0)
    return car::Status::BusError;
  if ((static_cast<unsigned short>(interrupt) & MPU_INT_STATUS_FIFO_OVERFLOW) !=
      0U)
    return mpu_reset_fifo() == 0 ? car::Status::Busy : car::Status::BusError;

  short gyro[3]{}, accel[3]{}, sensors = 0;
  long quaternion[4]{};
  unsigned long timestamp = 0U;
  unsigned char more = 0U;
  do {
    if (dmp_read_fifo(gyro, accel, quaternion, &timestamp, &sensors, &more) !=
        0)
      return car::Status::Busy;
  } while (more != 0U);
  if ((static_cast<unsigned short>(sensors) & INV_WXYZ_QUAT) == 0U)
    return car::Status::Busy;

  const float q0 = static_cast<float>(quaternion[0]) / kQ30;
  const float q1 = static_cast<float>(quaternion[1]) / kQ30;
  const float q2 = static_cast<float>(quaternion[2]) / kQ30;
  const float q3 = static_cast<float>(quaternion[3]) / kQ30;
  sample = {static_cast<float>(accel[0]) / 16384.0F,
            static_cast<float>(accel[1]) / 16384.0F,
            static_cast<float>(accel[2]) / 16384.0F,
            static_cast<float>(gyro[0]) / 16.4F,
            static_cast<float>(gyro[1]) / 16.4F,
            static_cast<float>(gyro[2]) / 16.4F,
            std::atan2(2.0F * (q0 * q1 + q2 * q3),
                       1.0F - 2.0F * (q1 * q1 + q2 * q2)) *
                kDegrees,
            std::asin(2.0F * (q0 * q2 - q3 * q1)) * kDegrees,
            std::atan2(2.0F * (q0 * q3 + q1 * q2),
                       1.0F - 2.0F * (q2 * q2 + q3 * q3)) *
                kDegrees,
            static_cast<std::uint32_t>(timestamp)};
  return car::Status::Ok;
}

} // namespace drivers

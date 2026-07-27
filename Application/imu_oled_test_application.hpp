#pragma once

#include "Drivers/led.hpp"
#include "Drivers/mpu6050.hpp"
#include "Drivers/ssd1306.hpp"
#include "Middlewares/attitude_backend_config.h"
#include "Middlewares/attitude_filter.hpp"
#include "Middlewares/imu_reader.hpp"

#if ATTITUDE_CONFIG_BACKEND != ATTITUDE_BACKEND_MAHONY
#error "APP_IMU_OLED_TEST requires ATTITUDE_BACKEND_MAHONY"
#endif

namespace app {

class ImuOledTestApplication final {
public:
  void init() noexcept;
  void step() noexcept;

private:
  void updateImu(std::uint32_t now) noexcept;
  void refreshOled(std::uint32_t now) noexcept;
  void writeMeasurements() noexcept;

  drivers::Led leds_{};
  drivers::Mpu6050 imu_{};
  middleware::ImuReader imuReader_{};
  middleware::AttitudeFilter attitude_{{middleware::AttitudeAlgorithm::Mahony}};
  drivers::Ssd1306 oled_{};
  car::ImuSample sample_{};
  std::uint32_t lastImuMs_ = 0U;
  std::uint32_t lastOledTextMs_ = 0U;
  std::uint32_t lastOledServiceMs_ = 0U;
  bool imuReady_ = false;
  bool gyroCalibrated_ = false;
  bool oledReady_ = false;
};

} // namespace app

#pragma once
#include "Drivers/active_buzzer.hpp"
#include "Drivers/encoder.hpp"
#include "Drivers/keypad.hpp"
#include "Drivers/led.hpp"
#include "Drivers/line_sensor_array.hpp"
#include "Drivers/motor_driver.hpp"
#include "Drivers/mpu6050.hpp"
#include "Drivers/mpu6050_dmp.hpp"
#include "Drivers/ssd1306.hpp"
#include "Middleware/Attitude/attitude_backend_config.h"
#include "Middlewares/attitude_filter.hpp"
#include "Middlewares/differential_drive.hpp"
#include "Middlewares/line_follower.hpp"
#include "Middlewares/safety_gate.hpp"
#include "Middlewares/telemetry.hpp"

namespace app {
class CarApplication final {
public:
  void init() noexcept;
  void step() noexcept;

private:
  drivers::MotorDriver motor_{};
  drivers::LineSensorArray line_{};
  drivers::Encoder encoder_{};
  drivers::Keypad keys_{};
  drivers::Led leds_{};
  drivers::ActiveBuzzer buzzer_{};
  drivers::Mpu6050 rawImu_{};
  drivers::Mpu6050Dmp dmpImu_{};
  middleware::AttitudeFilter softwareAttitude_{
#if ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_KALMAN
      {middleware::AttitudeAlgorithm::Kalman}
#else
      {middleware::AttitudeAlgorithm::Complementary}
#endif
  };
  drivers::Ssd1306 oled_{};
  middleware::LineFollower follower_{{45.0F, 180}};
  middleware::DifferentialDrive drive_{{0.15F}};
  middleware::SafetyGate gate_{};
  middleware::Telemetry telemetry_{};
  std::uint32_t lastLineMs_ = 0U;
  std::uint32_t lastTelemetryMs_ = 0U;
  std::uint32_t lastHeartbeatMs_ = 0U;
  std::uint32_t centerSinceMs_ = 0U;
  bool centerWasPressed_ = false;
  bool imuReady_ = false;
#if ATTITUDE_CONFIG_BACKEND != ATTITUDE_BACKEND_DMP
  std::uint32_t lastImuMs_ = 0U;
#endif
  bool oledReady_ = false;
  bool heartbeat_ = false;
  car::LineSample lineSample_{};
};
} // namespace app

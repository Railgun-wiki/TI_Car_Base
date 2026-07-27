#pragma once
#include "Drivers/active_buzzer.hpp"
#include "Drivers/encoder.hpp"
#include "Drivers/keypad.hpp"
#include "Drivers/led.hpp"
#include "Drivers/line_sensor_array.hpp"
#include "Drivers/motor_driver.hpp"
#include "Drivers/ssd1306.hpp"
#include "Middlewares/attitude_backend_config.h"
#include "Middlewares/attitude_filter.hpp"
#include "Middlewares/differential_drive.hpp"
#include "Middlewares/line_follower.hpp"
#include "Middlewares/safety_gate.hpp"
#include "Middlewares/telemetry.hpp"
#include "Middlewares/vofa_protocol.hpp"
#if ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_DMP
#include "Drivers/mpu6050_dmp.hpp"
#elif ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_BMI270
#include "Drivers/bmi270.hpp"
#include "Drivers/imu_backend.hpp"
#include "Middlewares/imu_reader.hpp"
#else
#include "Drivers/imu_backend.hpp"
#include "Drivers/mpu6050.hpp"
#include "Middlewares/imu_reader.hpp"
#endif

namespace app {
class CarApplication final {
public:
  void init() noexcept;
  void step() noexcept;

private:
  void processKeyInteraction() noexcept;
  void refreshOled(std::uint32_t now) noexcept;
  drivers::MotorDriver motor_{};
  drivers::LineSensorArray line_{};
  drivers::Encoder encoder_{};
  drivers::Keypad keys_{};
  drivers::Led leds_{};
  drivers::ActiveBuzzer buzzer_{};
#if ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_DMP
  drivers::Mpu6050Dmp dmpImu_{};
#elif ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_BMI270
  drivers::Bmi270 rawImu_{};
#if !BMI270_ONBOARD_FUSION
  middleware::ImuReader imuReader_{};
#endif
#else
  drivers::Mpu6050 rawImu_{};
  middleware::ImuReader imuReader_{};
#endif
  middleware::AttitudeFilter softwareAttitude_{
#if ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_KALMAN
      {middleware::AttitudeAlgorithm::Kalman}
#elif ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_MAHONY
      {middleware::AttitudeAlgorithm::Mahony}
#else
      {middleware::AttitudeAlgorithm::Complementary}
#endif
  };
  drivers::Ssd1306 oled_{};
  middleware::LineFollower follower_{{45.0F, 0.0F, 0.0F, 180}};
  middleware::DifferentialDrive drive_{{0.15F}};
  middleware::SafetyGate gate_{};
  middleware::Telemetry telemetry_{};
  middleware::VofaProtocol vofa_{};
  car::WheelCommand lineWheelCommand_{};
  car::ImuSample imuSample_{};
  std::uint32_t lastLineMs_ = 0U;
  std::uint32_t lastTelemetryMs_ = 0U;
  std::uint32_t lastHeartbeatMs_ = 0U;
  std::uint32_t lastOledTextMs_ = 0U;
  std::uint32_t lastOledServiceMs_ = 0U;
  std::uint32_t centerSinceMs_ = 0U;
  bool centerWasPressed_ = false;
  bool leftWasPressed_ = false;
  bool rightWasPressed_ = false;
  bool downWasPressed_ = false;
  bool imuReady_ = false;
  bool oledReady_ = false;
  bool heartbeat_ = false;
  bool lineFollowEnabled_ = false;
  car::LineSample lineSample_{};
};
} // namespace app

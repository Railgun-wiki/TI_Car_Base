#pragma once

#include <array>

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
#include "Middlewares/h_question_race.hpp"
#include "Middlewares/line_follower.hpp"
#include "Middlewares/pid.hpp"
#include "Middlewares/safety_gate.hpp"
#include "Middlewares/wheel_speed_controller.hpp"
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

class HQuestionApplication final {
public:
  void init() noexcept;
  void step() noexcept;

private:
  enum class StartupState : std::uint8_t {
    Tone,
    BootLog,
    WaitBootLog,
    UartLog,
    WaitUartLog,
    ScanI2c0,
    I2c0Log,
    WaitI2c0Log,
    ScanI2c1,
    I2c1Log,
    WaitI2c1Log,
    MpuStartLog,
    WaitMpuStartLog,
    MpuInit,
    MpuCalibrate,
    MpuResultLog,
    WaitMpuResultLog,
    OledStartLog,
    WaitOledStartLog,
    OledInit,
    OledResultLog,
    WaitOledResultLog,
    ReadyLog,
    WaitReadyLog,
    Ready,
  };
  struct I2cScan final {
    std::array<std::uint8_t, 8U> addresses{};
    std::uint8_t count = 0U;
    std::uint16_t timeouts = 0U;
    std::uint16_t errors = 0U;
    std::uint32_t elapsedMs = 0U;
    bool contains(std::uint8_t address) const noexcept;
  };

  void startupStep(std::uint32_t now) noexcept;
  void scanI2c(std::uint8_t bus, I2cScan &scan) noexcept;
  bool queueStartupLog(const char *text) noexcept;
  void formatScanLog(std::uint8_t bus, const I2cScan &scan) noexcept;
  void formatDeviceLog(const char *device, bool expectedPresent,
                       bool initialized) noexcept;
  void updateDeviceLeds() noexcept;
  void configureRace() noexcept;
  void updateImu() noexcept;
  void processKeys(std::uint32_t now) noexcept;
  void updateControl(std::uint32_t now) noexcept;
  void updateUserLed(std::uint32_t now) noexcept;
  void refreshOled(std::uint32_t now) noexcept;
  void submitMenu(const middleware::HRaceSnapshot &race) noexcept;
  void submitRun(const middleware::HRaceSnapshot &race) noexcept;

  middleware::RaceConfig raceConfig_{};
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
  middleware::LineFollower lineFollower_{{30.0F, 2.0F, 15.0F, 50}};
  middleware::Pid headingPid_{{10.0F, 1.0F, 3.0F, 60.0F, 50.0F}};
  middleware::WheelSpeedController speedController_{{
      {0.048F, 1.0F, 1040.0F, 1.0F},
      {2.0F, 1.0F, 0.0F, 250.0F, 20.0F},
      {2.0F, 1.0F, 0.0F, 250.0F, 20.0F},
      50U,
  }};
  middleware::DifferentialDrive drive_{{0.15F}};
  middleware::SafetyGate gate_{};
  middleware::HQuestionRace race_{};
  car::LineSample lineSample_{};
  car::ImuSample imuSample_{};
  car::WheelCommand proposal_{};
  std::uint32_t lastControlMs_ = 0U;
  std::uint32_t lastUserLedPulseMs_ = 0U;
  std::uint32_t lastOledTextMs_ = 0U;
  std::uint32_t lastOledServiceMs_ = 0U;
  std::uint32_t startupSinceMs_ = 0U;
  StartupState startupState_ = StartupState::Tone;
  I2cScan i2c0Scan_{};
  I2cScan i2c1Scan_{};
  char startupLog_[128]{};
  bool centerWasPressed_ = false;
  bool leftWasPressed_ = false;
  bool rightWasPressed_ = false;
  bool imuReady_ = false;
  bool oledReady_ = false;
  bool userLedOn_ = false;
  bool autoStartPending_ = true;
};

} // namespace app

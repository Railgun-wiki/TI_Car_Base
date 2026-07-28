#pragma once

#include <array>

#include "Application/h_question_program.hpp"
#include "Config/status_led_config.hpp"
#include "Config/vehicle_tuning.hpp"
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
  void submitMenu(const HRaceSnapshot &race) noexcept;
  void submitRun(const HRaceSnapshot &race) noexcept;

  HQuestionProgramConfig raceConfig_{};
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
  middleware::LineFollower lineFollower_{
      {H_QUESTION_LINE_KP, H_QUESTION_LINE_KI, H_QUESTION_LINE_KD,
       H_QUESTION_ARC_SPEED_MM_PER_SECOND, VEHICLE_TUNING_LINE_PREDICT_MS,
       VEHICLE_TUNING_LINE_SEARCH_TIMEOUT_MS,
       VEHICLE_TUNING_LINE_PREDICT_SPEED_RATIO,
       VEHICLE_TUNING_LINE_SEARCH_SPEED_RATIO,
       VEHICLE_TUNING_LINE_ERROR_FILTER_ALPHA,
       VEHICLE_TUNING_LINE_TURN_SLEW_PER_SECOND,
       VEHICLE_TUNING_LINE_DIRECTION_MEMORY_MS,
       VEHICLE_TUNING_LINE_CORNER_CONFIRM_MS,
       VEHICLE_TUNING_LINE_CORNER_MEMORY_MS,
       VEHICLE_TUNING_LINE_CORNER_TIMEOUT_MS,
       VEHICLE_TUNING_LINE_CORNER_SPEED_RATIO,
       VEHICLE_TUNING_LINE_CORNER_SIDE_MIN_COUNT,
       VEHICLE_TUNING_LINE_CORNER_SIDE_DOMINANCE,
       VEHICLE_TUNING_LINE_REACQUIRE_CONFIRM_MS}};
  middleware::Pid headingPid_{
      {H_QUESTION_HEADING_KP, H_QUESTION_HEADING_KI, H_QUESTION_HEADING_KD,
       static_cast<float>(H_QUESTION_STRAIGHT_SPEED_MM_PER_SECOND -
                          H_QUESTION_MINIMUM_WHEEL_SPEED_MM_PER_SECOND),
       H_QUESTION_HEADING_INTEGRAL_LIMIT}};
  middleware::WheelSpeedController speedController_{{
      {VEHICLE_TUNING_WHEEL_DIAMETER_METERS, VEHICLE_TUNING_GEAR_RATIO,
       VEHICLE_TUNING_ENCODER_COUNTS_PER_MOTOR_REVOLUTION,
       VEHICLE_TUNING_ENCODER_QUADRATURE_MULTIPLIER},
      {H_QUESTION_SPEED_KP, H_QUESTION_SPEED_KI, H_QUESTION_SPEED_KD,
       H_QUESTION_SPEED_PWM_LIMIT, H_QUESTION_SPEED_INTEGRAL_LIMIT},
      {H_QUESTION_SPEED_KP, H_QUESTION_SPEED_KI, H_QUESTION_SPEED_KD,
       H_QUESTION_SPEED_PWM_LIMIT, H_QUESTION_SPEED_INTEGRAL_LIMIT},
      H_QUESTION_SPEED_SAMPLE_PERIOD_MS,
      H_QUESTION_SPEED_PWM_RISE_PER_UPDATE,
  }};
  middleware::DifferentialDrive drive_{{H_QUESTION_TRACK_WIDTH_METERS}};
  middleware::SafetyGate gate_{};
  HQuestionProgram race_{raceConfig_};
  car::LineSample lineSample_{};
  middleware::LineTrackingState lineTrackingState_ =
      middleware::LineTrackingState::Lost;
  car::ImuSample imuSample_{};
  car::WheelCommand proposal_{};
  std::uint32_t lastControlMs_ = 0U;
  std::uint32_t lastOuterControlMs_ = 0U;
  std::uint32_t lastLineFollowerMs_ = 0U;
  std::uint32_t lastControlLogMs_ = 0U;
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

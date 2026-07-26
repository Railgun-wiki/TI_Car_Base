#pragma once

#include "Drivers/active_buzzer.hpp"
#include "Drivers/encoder.hpp"
#include "Drivers/keypad.hpp"
#include "Drivers/led.hpp"
#include "Drivers/line_sensor_array.hpp"
#include "Drivers/motor_driver.hpp"
#include "Drivers/mpu6050_dmp.hpp"
#include "Drivers/ssd1306.hpp"
#include "Middlewares/differential_drive.hpp"
#include "Middlewares/h_question_race.hpp"
#include "Middlewares/line_follower.hpp"
#include "Middlewares/pid.hpp"
#include "Middlewares/safety_gate.hpp"

namespace app {

class HQuestionApplication final {
public:
  void init() noexcept;
  void step() noexcept;

private:
  void updateImu() noexcept;
  void processKeys(std::uint32_t now) noexcept;
  void updateControl(std::uint32_t now) noexcept;
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
  drivers::Mpu6050Dmp imu_{};
  drivers::Ssd1306 oled_{};
  middleware::LineFollower lineFollower_{{45.0F, 0.0F, 0.0F, 250}};
  middleware::Pid headingPid_{{18.0F, 0.0F, 0.2F, 350.0F, 50.0F}};
  middleware::DifferentialDrive drive_{{0.15F}};
  middleware::SafetyGate gate_{};
  middleware::HQuestionRace race_{};
  car::LineSample lineSample_{};
  car::ImuSample imuSample_{};
  car::WheelCommand proposal_{};
  std::uint32_t lastControlMs_ = 0U;
  std::uint32_t lastOledTextMs_ = 0U;
  std::uint32_t lastOledServiceMs_ = 0U;
  bool centerWasPressed_ = false;
  bool leftWasPressed_ = false;
  bool rightWasPressed_ = false;
  bool imuReady_ = false;
  bool oledReady_ = false;
};

} // namespace app

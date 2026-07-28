#include "Application/h_question_application.hpp"

#include "BSP/encoder.hpp"
#include "BSP/i2c.hpp"
#include "BSP/system.hpp"
#include "BSP/uart.hpp"

namespace app {
namespace {
constexpr std::uint32_t kControlPeriodMs = H_QUESTION_CONTROL_PERIOD_MS;
constexpr std::uint32_t kOuterControlPeriodMs =
    H_QUESTION_OUTER_CONTROL_PERIOD_MS;
constexpr std::uint32_t kControlLogPeriodMs = H_QUESTION_CONTROL_LOG_PERIOD_MS;

void constrainForwardTarget(std::int16_t &target) noexcept {
  if (target <= 0) {
    target = 0;
  } else if (target < H_QUESTION_MINIMUM_WHEEL_SPEED_MM_PER_SECOND) {
    target = H_QUESTION_MINIMUM_WHEEL_SPEED_MM_PER_SECOND;
  }
}

void appendText(char *destination, std::uint8_t &length,
                const char *source) noexcept {
  while (*source != '\0' && length < 126U)
    destination[length++] = *source++;
}
void appendHex(char *destination, std::uint8_t &length,
               std::uint8_t value) noexcept {
  constexpr char kDigits[] = "0123456789ABCDEF";
  destination[length++] = kDigits[(value >> 4U) & 0x0FU];
  destination[length++] = kDigits[value & 0x0FU];
}
void appendUnsigned(char *destination, std::uint8_t &length,
                    std::uint32_t value) noexcept {
  char digits[10]{};
  std::uint8_t count = 0U;
  do {
    digits[count++] = static_cast<char>('0' + value % 10U);
    value /= 10U;
  } while (value != 0U);
  while (count != 0U)
    destination[length++] = digits[--count];
}

void appendSigned(char *destination, std::uint8_t &length,
                  std::int32_t value) noexcept {
  if (value < 0) {
    destination[length++] = '-';
    appendUnsigned(destination, length,
                   static_cast<std::uint32_t>(-(value + 1)) + 1U);
    return;
  }
  appendUnsigned(destination, length, static_cast<std::uint32_t>(value));
}

void textNumber(char *text, std::uint8_t offset, std::uint16_t value) noexcept {
  text[offset] = static_cast<char>('0' + (value / 100U) % 10U);
  text[offset + 1U] = static_cast<char>('0' + (value / 10U) % 10U);
  text[offset + 2U] = static_cast<char>('0' + value % 10U);
}

const char *programText(HProgram program) noexcept {
  switch (program) {
  case HProgram::Requirement1:
    return "P1 A-B";
  case HProgram::Requirement2:
    return "P2 LINE 500CM";
  case HProgram::Requirement3:
    return "P3 A-C-B-D-A";
  case HProgram::Requirement4:
    return "P4 P3 X4";
  }
  return "P?";
}

drivers::LedPattern raceLedPattern(const HRaceSnapshot &race) noexcept {
  switch (race.state) {
  case HRaceState::Menu:
    return config::status_led::kRaceMenu;
  case HRaceState::Countdown:
    return config::status_led::kRaceCountdown;
  case HRaceState::Running:
    return config::status_led::kRaceRunning;
  case HRaceState::Checkpoint:
    return config::status_led::kRaceCheckpoint;
  case HRaceState::Finished:
    return config::status_led::kRaceFinished;
  case HRaceState::Fault:
    return config::status_led::kRaceFault;
  }
  return config::status_led::kDevicesNone;
}
} // namespace

bool HQuestionApplication::I2cScan::contains(
    std::uint8_t address) const noexcept {
  for (std::uint8_t index = 0U; index < count; ++index)
    if (addresses[index] == address)
      return true;
  return false;
}

void HQuestionApplication::init() noexcept {
  motor_.stop();
  leds_.setUser(false);
  leds_.setPattern(config::status_led::kStartup);
  buzzer_.set(true);
  startupSinceMs_ = bsp::millis();
  leds_.service(startupSinceMs_);
  startupState_ = StartupState::Tone;
}

bool HQuestionApplication::queueStartupLog(const char *text) noexcept {
  std::size_t length = 0U;
  while (text[length] != '\0')
    ++length;
  return bsp::uartTryWrite(text, length);
}

void HQuestionApplication::scanI2c(std::uint8_t bus, I2cScan &scan) noexcept {
  scan = {};
  const std::uint32_t start = bsp::millis();
  for (std::uint8_t address = 0x08U; address <= 0x77U; ++address) {
    const car::Status status = bsp::i2cProbe(bus, address);
    if (status == car::Status::Ok) {
      if (scan.count < scan.addresses.size())
        scan.addresses[scan.count++] = address;
    } else if (status == car::Status::Timeout) {
      ++scan.timeouts;
    } else {
      ++scan.errors;
    }
  }
  scan.elapsedMs = bsp::millis() - start;
}

void HQuestionApplication::formatScanLog(std::uint8_t bus,
                                         const I2cScan &scan) noexcept {
  std::uint8_t length = 0U;
  appendText(startupLog_, length, "I2C");
  startupLog_[length++] = static_cast<char>('0' + bus);
  appendText(startupLog_, length, " SCAN ACK:");
  if (scan.count == 0U) {
    appendText(startupLog_, length, " NONE");
  } else {
    for (std::uint8_t index = 0U; index < scan.count; ++index) {
      appendText(startupLog_, length, " 0x");
      appendHex(startupLog_, length, scan.addresses[index]);
    }
  }
  appendText(startupLog_, length, " TIMEOUT=");
  appendUnsigned(startupLog_, length, scan.timeouts);
  appendText(startupLog_, length, " ERROR=");
  appendUnsigned(startupLog_, length, scan.errors);
  appendText(startupLog_, length, " MS=");
  appendUnsigned(startupLog_, length, scan.elapsedMs);
  appendText(startupLog_, length, "\r\n");
  startupLog_[length] = '\0';
}

void HQuestionApplication::formatDeviceLog(const char *device,
                                           bool expectedPresent,
                                           bool initialized) noexcept {
  std::uint8_t length = 0U;
  appendText(startupLog_, length, device);
  if (!expectedPresent)
    appendText(startupLog_, length, " SKIP NOT FOUND\r\n");
  else if (initialized)
    appendText(startupLog_, length, " OK\r\n");
  else
    appendText(startupLog_, length, " FAIL\r\n");
  startupLog_[length] = '\0';
}

void HQuestionApplication::configureRace() noexcept {
  const float maxTurn =
      static_cast<float>(H_QUESTION_STRAIGHT_SPEED_MM_PER_SECOND -
                         H_QUESTION_MINIMUM_WHEEL_SPEED_MM_PER_SECOND);
  headingPid_.configure({H_QUESTION_HEADING_KP, H_QUESTION_HEADING_KI,
                         H_QUESTION_HEADING_KD, maxTurn,
                         H_QUESTION_HEADING_INTEGRAL_LIMIT});
  const auto lineConfig = lineFollower_.config();
  (void)lineFollower_.configure(lineConfig.kp, lineConfig.ki, lineConfig.kd,
                                H_QUESTION_ARC_SPEED_MM_PER_SECOND);
  race_ = HQuestionProgram{raceConfig_};
  speedController_.reset();
}

void HQuestionApplication::updateDeviceLeds() noexcept {
  const std::uint8_t devices = static_cast<std::uint8_t>(imuReady_) +
                               static_cast<std::uint8_t>(oledReady_);
  if (devices == 0U)
    leds_.setPattern(config::status_led::kDevicesNone);
  else if (devices == 1U)
    leds_.setPattern(config::status_led::kDevicesOne);
  else
    leds_.setPattern(config::status_led::kDevicesTwo);
}

void HQuestionApplication::startupStep(std::uint32_t now) noexcept {
  switch (startupState_) {
  case StartupState::Tone:
    if (static_cast<std::uint32_t>(now - startupSinceMs_) <
        VEHICLE_TUNING_STARTUP_TONE_MS)
      return;
    buzzer_.set(false);
    leds_.setPattern(config::status_led::kDevicesNone);
    startupState_ = StartupState::BootLog;
    return;
  case StartupState::BootLog:
    if (queueStartupLog("BOOT H-QUESTION\r\n"))
      startupState_ = StartupState::WaitBootLog;
    return;
  case StartupState::WaitBootLog:
    if (bsp::uartTxIdle())
      startupState_ = StartupState::UartLog;
    return;
  case StartupState::UartLog:
    if (queueStartupLog("UART0 READY 115200 8N1\r\n"))
      startupState_ = StartupState::WaitUartLog;
    return;
  case StartupState::WaitUartLog:
    if (bsp::uartTxIdle())
      startupState_ = StartupState::ScanI2c0;
    return;
  case StartupState::ScanI2c0:
    scanI2c(0U, i2c0Scan_);
    startupState_ = StartupState::I2c0Log;
    return;
  case StartupState::I2c0Log:
    formatScanLog(0U, i2c0Scan_);
    if (queueStartupLog(startupLog_))
      startupState_ = StartupState::WaitI2c0Log;
    return;
  case StartupState::WaitI2c0Log:
    if (bsp::uartTxIdle())
      startupState_ = StartupState::ScanI2c1;
    return;
  case StartupState::ScanI2c1:
    scanI2c(1U, i2c1Scan_);
    startupState_ = StartupState::I2c1Log;
    return;
  case StartupState::I2c1Log:
    formatScanLog(1U, i2c1Scan_);
    if (queueStartupLog(startupLog_))
      startupState_ = StartupState::WaitI2c1Log;
    return;
  case StartupState::WaitI2c1Log:
    if (bsp::uartTxIdle())
      startupState_ = StartupState::MpuStartLog;
    return;
  case StartupState::MpuStartLog:
    if (i2c0Scan_.contains(0x68U)) {
#if ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_DMP
      if (queueStartupLog("MPU 0x68 INIT DMP...\r\n"))
#elif ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_BMI270
      if (queueStartupLog("BMI270 0x68 INIT...\r\n"))
#else
      if (queueStartupLog("MPU 0x68 INIT SW FILTER...\r\n"))
#endif
        startupState_ = StartupState::WaitMpuStartLog;
    } else if (queueStartupLog("MPU 0x68 SKIP NOT FOUND\r\n"))
      startupState_ = StartupState::OledStartLog;
    return;
  case StartupState::WaitMpuStartLog:
    if (bsp::uartTxIdle())
      startupState_ = StartupState::MpuInit;
    return;
  case StartupState::MpuInit:
#if ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_DMP
    imuReady_ = dmpImu_.begin() == car::Status::Ok;
    startupState_ = StartupState::MpuResultLog;
#else
    imuReady_ = rawImu_.begin() == car::Status::Ok;
#if ATTITUDE_CONFIG_BACKEND != ATTITUDE_BACKEND_BMI270 || !BMI270_ONBOARD_FUSION
    imuReader_.reset();
    softwareAttitude_.reset();
#endif
    if (imuReady_) {
      startupState_ = StartupState::MpuCalibrate;
      return;
    }
    startupState_ = StartupState::MpuResultLog;
#endif
    return;
  case StartupState::MpuCalibrate: {
#if ATTITUDE_CONFIG_BACKEND != ATTITUDE_BACKEND_DMP
    // Blocks ~1 s while averaging gyro bias. The car must be stationary.
    // Failure downgrades imuReady_ so the H-question gate refuses to start.
    // Unreachable in the DMP build (MpuInit skips straight to MpuResultLog),
    // hence the guard.
    const car::Status status = rawImu_.calibrateGyroBias();
    softwareAttitude_.reset();
    if (status != car::Status::Ok)
      imuReady_ = false;
#endif
    startupState_ = StartupState::MpuResultLog;
    return;
  }
  case StartupState::MpuResultLog:
    updateDeviceLeds();
#if ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_DMP
    formatDeviceLog("MPU 0x68 INIT DMP", true, imuReady_);
#elif ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_BMI270
    formatDeviceLog("BMI270 0x68 INIT", true, imuReady_);
#else
    formatDeviceLog("MPU 0x68 INIT SW FILTER", true, imuReady_);
#endif
    if (queueStartupLog(startupLog_))
      startupState_ = StartupState::WaitMpuResultLog;
    return;
  case StartupState::WaitMpuResultLog:
    if (bsp::uartTxIdle())
      startupState_ = StartupState::OledStartLog;
    return;
  case StartupState::OledStartLog:
    if (i2c1Scan_.contains(0x3CU)) {
      if (queueStartupLog("OLED 0x3C INIT...\r\n"))
        startupState_ = StartupState::WaitOledStartLog;
    } else if (queueStartupLog("OLED 0x3C SKIP NOT FOUND\r\n"))
      startupState_ = StartupState::ReadyLog;
    return;
  case StartupState::WaitOledStartLog:
    if (bsp::uartTxIdle())
      startupState_ = StartupState::OledInit;
    return;
  case StartupState::OledInit:
    oledReady_ = oled_.begin() == car::Status::Ok;
    startupState_ = StartupState::OledResultLog;
    return;
  case StartupState::OledResultLog:
    updateDeviceLeds();
    formatDeviceLog("OLED 0x3C INIT", true, oledReady_);
    if (queueStartupLog(startupLog_))
      startupState_ = StartupState::WaitOledResultLog;
    return;
  case StartupState::WaitOledResultLog:
    if (bsp::uartTxIdle())
      startupState_ = StartupState::ReadyLog;
    return;
  case StartupState::ReadyLog: {
    updateDeviceLeds();
    configureRace();
    const std::uint8_t devices = static_cast<std::uint8_t>(imuReady_) +
                                 static_cast<std::uint8_t>(oledReady_);
    std::uint8_t length = 0U;
    appendText(startupLog_, length,
               devices == 2U ? "BOOT READY DEVICES="
                             : "BOOT DEGRADED DEVICES=");
    appendUnsigned(startupLog_, length, devices);
    appendText(startupLog_, length, "\r\n");
    startupLog_[length] = '\0';
    if (queueStartupLog(startupLog_))
      startupState_ = StartupState::WaitReadyLog;
    return;
  }
  case StartupState::WaitReadyLog:
    if (bsp::uartTxIdle()) {
      startupState_ = StartupState::Ready;
      if (autoStartPending_) {
        autoStartPending_ = false;
#if H_QUESTION_AUTO_START_PROGRAM != 0
        if (imuReady_) {
          race_.select(
              static_cast<HProgram>(H_QUESTION_AUTO_START_PROGRAM - 1));
          headingPid_.reset();
          lineFollower_.reset();
          speedController_.reset();
          race_.start(now, encoder_.ticks(), imuSample_.yawDeg);
        }
#endif
      }
    }
    return;
  case StartupState::Ready:
    return;
  }
}

void HQuestionApplication::updateImu() noexcept {
  if (!imuReady_ || !bsp::consumeImuDataReady())
    return;
  car::ImuSample sample{};
#if ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_DMP
  dmpImu_.notifyDataReady();
  const car::Status status = dmpImu_.poll(sample);
#elif ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_BMI270 &&                    \
    BMI270_ONBOARD_FUSION
  const car::Status status = rawImu_.poll(sample);
#else
  const car::Status status =
      imuReader_.step(rawImu_, softwareAttitude_, bsp::millis(), sample);
#endif
  if (status == car::Status::Ok) {
    imuSample_ = sample;
  } else if (status != car::Status::Busy) {
    imuReady_ = false;
    race_.fail();
    motor_.stop();
    buzzer_.set(true);
  }
}

void HQuestionApplication::processKeys(std::uint32_t now) noexcept {
  const bool left = keys_.pressed(car::Key::Left);
  const bool right = keys_.pressed(car::Key::Right);
  const bool center = keys_.pressed(car::Key::Center);
  const bool leftPressed = left && !leftWasPressed_;
  const bool rightPressed = right && !rightWasPressed_;
  const bool centerPressed = center && !centerWasPressed_;
  leftWasPressed_ = left;
  rightWasPressed_ = right;
  centerWasPressed_ = center;

  const auto race = race_.snapshot();
  if (race.state == HRaceState::Menu) {
    if (leftPressed)
      race_.selectPrevious();
    if (rightPressed)
      race_.selectNext();
    if (centerPressed && imuReady_) {
      headingPid_.reset();
      lineFollower_.reset();
      speedController_.reset();
      race_.start(now, encoder_.ticks(), imuSample_.yawDeg);
    }
    return;
  }
  if (centerPressed) {
    race_.cancel();
    motor_.stop();
    headingPid_.reset();
    lineFollower_.reset();
    speedController_.reset();
  }
}

void HQuestionApplication::updateControl(std::uint32_t now) noexcept {
  if (static_cast<std::uint32_t>(now - lastControlMs_) < kControlPeriodMs)
    return;
  lastControlMs_ = now;
  lineSample_ = line_.read();
  const auto race =
      race_.update(now, encoder_.ticks(), imuSample_.yawDeg, lineSample_);
  if (race.state != HRaceState::Running) {
    motor_.set(gate_.apply({0, 0}, false));
    headingPid_.reset();
    lineFollower_.reset();
    speedController_.reset();
    lineTrackingState_ = middleware::LineTrackingState::Lost;
    lastLineFollowerMs_ = 0U;
    return;
  }

  if (static_cast<std::uint32_t>(now - lastOuterControlMs_) >=
      kOuterControlPeriodMs) {
    lastOuterControlMs_ = now;
    car::VehicleCommand command{};
    if (race.segmentType == HSegmentType::Arc) {
      const std::uint32_t elapsedMs =
          lastLineFollowerMs_ == 0U
              ? kOuterControlPeriodMs
              : static_cast<std::uint32_t>(now - lastLineFollowerMs_);
      lastLineFollowerMs_ = now;
      const auto result = lineFollower_.update(
          lineSample_, static_cast<float>(elapsedMs) / 1000.0F);
      lineTrackingState_ = result.state;
      command = result.command;
      if (result.state == middleware::LineTrackingState::Lost) {
        race_.fail();
        proposal_ = {};
        motor_.stop();
        speedController_.reset();
        return;
      }
    } else {
      lastLineFollowerMs_ = 0U;
      const float turn = headingPid_.update(
          race.targetYawDeg, imuSample_.yawDeg,
          static_cast<float>(kOuterControlPeriodMs) / 1000.0F);
      command = {H_QUESTION_STRAIGHT_SPEED_MM_PER_SECOND,
                 static_cast<std::int16_t>(turn)};
    }
    proposal_ = drive_.mix(command);
    const bool applyMinimum =
        race.segmentType != HSegmentType::Arc ||
        lineTrackingState_ == middleware::LineTrackingState::Tracking;
    if (applyMinimum) {
      constrainForwardTarget(proposal_.left);
      constrainForwardTarget(proposal_.right);
    }
  }

  motor_.set(gate_.apply(
      speedController_.update(encoder_.ticks(), now, proposal_), imuReady_));
  if (static_cast<std::uint32_t>(now - lastControlLogMs_) < kControlLogPeriodMs)
    return;
  lastControlLogMs_ = now;
  const auto ticks = encoder_.ticks();
  const auto speed = speedController_.measured();
  const auto pwm = motor_.command();
  std::uint8_t length = 0U;
  appendText(startupLog_, length, "CTRL T=");
  appendSigned(startupLog_, length, ticks.left);
  appendText(startupLog_, length, ",");
  appendSigned(startupLog_, length, ticks.right);
  appendText(startupLog_, length, " V=");
  appendSigned(startupLog_, length,
               static_cast<std::int32_t>(speed.leftMetersPerSecond * 1000.0F));
  appendText(startupLog_, length, ",");
  appendSigned(startupLog_, length,
               static_cast<std::int32_t>(speed.rightMetersPerSecond * 1000.0F));
  appendText(startupLog_, length, " REF=");
  appendSigned(startupLog_, length, proposal_.left);
  appendText(startupLog_, length, ",");
  appendSigned(startupLog_, length, proposal_.right);
  appendText(startupLog_, length, " PWM=");
  appendSigned(startupLog_, length, pwm.left);
  appendText(startupLog_, length, ",");
  appendSigned(startupLog_, length, pwm.right);
  appendText(startupLog_, length, "\r\n");
  (void)bsp::uartTryWrite(startupLog_, length);
}

void HQuestionApplication::updateUserLed(std::uint32_t now) noexcept {
  if (userLedOn_ && static_cast<std::uint32_t>(now - lastUserLedPulseMs_) >=
                        VEHICLE_TUNING_USER_LED_PULSE_MS) {
    userLedOn_ = false;
    leds_.setUser(false);
  }
  if (!userLedOn_ && static_cast<std::uint32_t>(now - lastUserLedPulseMs_) >=
                         VEHICLE_TUNING_USER_LED_PERIOD_MS) {
    userLedOn_ = true;
    lastUserLedPulseMs_ = now;
    leds_.setUser(true);
  }
}

void HQuestionApplication::submitMenu(const HRaceSnapshot &race) noexcept {
  (void)oled_.writeLine(0U, "H QUESTION MENU");
  (void)oled_.writeLine(1U, programText(race.program));
  (void)oled_.writeLine(2U, "LEFT RIGHT SELECT");
  (void)oled_.writeLine(3U, imuReady_ ? "CENTER START" : "IMU NOT READY");
}

void HQuestionApplication::submitRun(const HRaceSnapshot &race) noexcept {
  char row0[] = "RUN P0 S0/0 L0";
  char row1[] = "D:000/000 CM";
  char row2[] = "Y:000 TARGET";
  row0[5] =
      static_cast<char>('0' + static_cast<std::uint8_t>(race.program) + 1U);
  row0[8] = static_cast<char>('0' + race.segmentNumber + 1U);
  row0[10] = static_cast<char>('0' + race.segmentCount);
  row0[13] = static_cast<char>('0' + race.lap + 1U);
  textNumber(row1, 2U, static_cast<std::uint16_t>(race.distanceCm));
  textNumber(row1, 6U, static_cast<std::uint16_t>(race.targetDistanceCm));
  const float yaw =
      imuSample_.yawDeg < 0.0F ? -imuSample_.yawDeg : imuSample_.yawDeg;
  textNumber(row2, 2U, static_cast<std::uint16_t>(yaw));
  (void)oled_.writeLine(0U, row0);
  (void)oled_.writeLine(1U, row1);
  (void)oled_.writeLine(2U, row2);
  if (race.state == HRaceState::Countdown)
    (void)oled_.writeLine(3U, "CENTER CANCEL");
  else if (race.state == HRaceState::Finished)
    (void)oled_.writeLine(3U, "DONE CENTER MENU");
  else if (race.state == HRaceState::Fault)
    (void)oled_.writeLine(3U, "FAULT CENTER MENU");
  else
    (void)oled_.writeLine(3U, "CENTER STOP");
}

void HQuestionApplication::refreshOled(std::uint32_t now) noexcept {
  if (!oledReady_)
    return;
  if (static_cast<std::uint32_t>(now - lastOledTextMs_) >=
      H_QUESTION_OLED_TEXT_PERIOD_MS) {
    lastOledTextMs_ = now;
    const auto race = race_.snapshot();
    if (race.state == HRaceState::Menu)
      submitMenu(race);
    else
      submitRun(race);
  }
  if (static_cast<std::uint32_t>(now - lastOledServiceMs_) >=
      H_QUESTION_OLED_SERVICE_PERIOD_MS) {
    lastOledServiceMs_ = now;
    const car::Status status = oled_.service();
    if (status != car::Status::Ok && status != car::Status::Busy)
      oledReady_ = false;
  }
}

void HQuestionApplication::step() noexcept {
  const std::uint32_t now = bsp::millis();
  updateUserLed(now);
  leds_.service(now);
  if (startupState_ != StartupState::Ready) {
    startupStep(now);
    if (oledReady_)
      refreshOled(now);
    return;
  }
  updateImu();
  processKeys(now);
  updateControl(now);
  leds_.setPattern(raceLedPattern(race_.snapshot()));
  leds_.service(now);
  refreshOled(now);
}

} // namespace app

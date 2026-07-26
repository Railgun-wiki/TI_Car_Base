#include "Application/h_question_application.hpp"

#include "BSP/encoder.hpp"
#include "BSP/i2c.hpp"
#include "BSP/system.hpp"
#include "BSP/uart.hpp"

namespace app {
namespace {
constexpr std::uint32_t kControlPeriodMs = 5U;
constexpr std::uint32_t kUserLedPeriodMs = 1000U;
constexpr std::uint32_t kUserLedPulseMs = 50U;
constexpr std::uint32_t kStartupToneMs = 30U;

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

void textNumber(char *text, std::uint8_t offset, std::uint16_t value) noexcept {
  text[offset] = static_cast<char>('0' + (value / 100U) % 10U);
  text[offset + 1U] = static_cast<char>('0' + (value / 10U) % 10U);
  text[offset + 2U] = static_cast<char>('0' + value % 10U);
}

const char *programText(middleware::HProgram program) noexcept {
  switch (program) {
  case middleware::HProgram::Requirement1:
    return "P1 A-B";
  case middleware::HProgram::Requirement2:
    return "P2 A-B-C-D-A";
  case middleware::HProgram::Requirement3:
    return "P3 A-C-B-D-A";
  case middleware::HProgram::Requirement4:
    return "P4 P3 X4";
  }
  return "P?";
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
  leds_.setStatus(0U, true);
  leds_.setStatus(1U, true);
  leds_.setStatus(2U, true);
  buzzer_.set(true);
  startupSinceMs_ = bsp::millis();
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
  headingPid_.configure({raceConfig_.headingKp, raceConfig_.headingKi,
                         raceConfig_.headingKd, 350.0F, 50.0F});
  const auto lineConfig = lineFollower_.config();
  (void)lineFollower_.configure(lineConfig.kp, lineConfig.ki, lineConfig.kd,
                                raceConfig_.arcCruise);
  race_ = middleware::HQuestionRace{raceConfig_};
}

void HQuestionApplication::updateDeviceLeds() noexcept {
  leds_.setStatus(0U, imuReady_ || oledReady_);
  leds_.setStatus(1U, imuReady_ && oledReady_);
  leds_.setStatus(2U, false);
}

void HQuestionApplication::startupStep(std::uint32_t now) noexcept {
  switch (startupState_) {
  case StartupState::Tone:
    if (static_cast<std::uint32_t>(now - startupSinceMs_) < kStartupToneMs)
      return;
    buzzer_.set(false);
    leds_.setStatus(0U, false);
    leds_.setStatus(1U, false);
    leds_.setStatus(2U, false);
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
#else
    imuReady_ = rawImu_.begin() == car::Status::Ok;
    softwareAttitude_.reset();
#endif
    startupState_ = StartupState::MpuResultLog;
    return;
  case StartupState::MpuResultLog:
    updateDeviceLeds();
#if ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_DMP
    formatDeviceLog("MPU 0x68 INIT DMP", true, imuReady_);
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
    if (bsp::uartTxIdle())
      startupState_ = StartupState::Ready;
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
#else
  const car::Status status = rawImu_.poll(sample);
  const std::uint32_t now = bsp::millis();
  const std::uint32_t dtMs = lastImuMs_ == 0U ? 10U : now - lastImuMs_;
  lastImuMs_ = now;
  sample.timestampMs = now;
  if (status == car::Status::Ok)
    (void)softwareAttitude_.update(sample, static_cast<float>(dtMs) / 1000.0F);
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
  if (race.state == middleware::HRaceState::Menu) {
    if (leftPressed)
      race_.selectPrevious();
    if (rightPressed)
      race_.selectNext();
    if (centerPressed && imuReady_) {
      headingPid_.reset();
      lineFollower_.reset();
      race_.start(now, encoder_.ticks(), imuSample_.yawDeg);
    }
    return;
  }
  if (centerPressed) {
    race_.cancel();
    motor_.stop();
    headingPid_.reset();
    lineFollower_.reset();
  }
}

void HQuestionApplication::updateControl(std::uint32_t now) noexcept {
  const auto race = race_.update(now, encoder_.ticks(), imuSample_.yawDeg,
                                 lineSample_.detected);
  if (race.state != middleware::HRaceState::Running) {
    motor_.set(gate_.apply({0, 0}, false));
    return;
  }
  if (static_cast<std::uint32_t>(now - lastControlMs_) < kControlPeriodMs)
    return;
  lastControlMs_ = now;
  lineSample_ = line_.read();

  car::VehicleCommand command{};
  if (race.segmentType == middleware::HSegmentType::Arc) {
    command = lineFollower_.update(
        lineSample_, static_cast<float>(kControlPeriodMs) / 1000.0F);
  } else {
    const float turn =
        headingPid_.update(race.targetYawDeg, imuSample_.yawDeg,
                           static_cast<float>(kControlPeriodMs) / 1000.0F);
    command = {raceConfig_.straightCruise, static_cast<std::int16_t>(turn)};
  }
  proposal_ = drive_.mix(command);
  motor_.set(gate_.apply(proposal_, imuReady_));
}

void HQuestionApplication::updateUserLed(std::uint32_t now) noexcept {
  if (userLedOn_ && static_cast<std::uint32_t>(now - lastUserLedPulseMs_) >=
                        kUserLedPulseMs) {
    userLedOn_ = false;
    leds_.setUser(false);
  }
  if (!userLedOn_ && static_cast<std::uint32_t>(now - lastUserLedPulseMs_) >=
                         kUserLedPeriodMs) {
    userLedOn_ = true;
    lastUserLedPulseMs_ = now;
    leds_.setUser(true);
  }
}

void HQuestionApplication::submitMenu(
    const middleware::HRaceSnapshot &race) noexcept {
  (void)oled_.writeLine(0U, "H QUESTION MENU");
  (void)oled_.writeLine(1U, programText(race.program));
  (void)oled_.writeLine(2U, "LEFT RIGHT SELECT");
  (void)oled_.writeLine(3U, imuReady_ ? "CENTER START" : "IMU NOT READY");
}

void HQuestionApplication::submitRun(
    const middleware::HRaceSnapshot &race) noexcept {
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
  if (race.state == middleware::HRaceState::Countdown)
    (void)oled_.writeLine(3U, "CENTER CANCEL");
  else if (race.state == middleware::HRaceState::Finished)
    (void)oled_.writeLine(3U, "DONE CENTER MENU");
  else if (race.state == middleware::HRaceState::Fault)
    (void)oled_.writeLine(3U, "FAULT CENTER MENU");
  else
    (void)oled_.writeLine(3U, "CENTER STOP");
}

void HQuestionApplication::refreshOled(std::uint32_t now) noexcept {
  if (!oledReady_)
    return;
  if (static_cast<std::uint32_t>(now - lastOledTextMs_) >= 100U) {
    lastOledTextMs_ = now;
    const auto race = race_.snapshot();
    if (race.state == middleware::HRaceState::Menu)
      submitMenu(race);
    else
      submitRun(race);
  }
  if (static_cast<std::uint32_t>(now - lastOledServiceMs_) >= 2U) {
    lastOledServiceMs_ = now;
    const car::Status status = oled_.service();
    if (status != car::Status::Ok && status != car::Status::Busy)
      oledReady_ = false;
  }
}

void HQuestionApplication::step() noexcept {
  const std::uint32_t now = bsp::millis();
  updateUserLed(now);
  if (startupState_ != StartupState::Ready) {
    startupStep(now);
    if (oledReady_)
      refreshOled(now);
    return;
  }
  updateImu();
  processKeys(now);
  updateControl(now);
  refreshOled(now);
}

} // namespace app

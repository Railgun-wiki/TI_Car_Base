#include "Application/h_question_application.hpp"

#include "BSP/encoder.hpp"
#include "BSP/system.hpp"

namespace app {
namespace {
constexpr std::uint32_t kControlPeriodMs = 5U;

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

void HQuestionApplication::init() noexcept {
  motor_.stop();
  leds_.setStatus(0U, true);
  buzzer_.set(true);
  const std::uint32_t start = bsp::millis();
  while (static_cast<std::uint32_t>(bsp::millis() - start) < 30U) {
  }
  buzzer_.set(false);

  imuReady_ = imu_.begin() == car::Status::Ok;
  oledReady_ = oled_.begin() == car::Status::Ok;
  headingPid_.configure({raceConfig_.headingKp, raceConfig_.headingKi,
                         raceConfig_.headingKd, 350.0F, 50.0F});
  const auto lineConfig = lineFollower_.config();
  (void)lineFollower_.configure(lineConfig.kp, lineConfig.ki, lineConfig.kd,
                                raceConfig_.arcCruise);
  race_ = middleware::HQuestionRace{raceConfig_};
  leds_.setStatus(1U, imuReady_);
  leds_.setStatus(2U, oledReady_);
}

void HQuestionApplication::updateImu() noexcept {
  if (!imuReady_ || !bsp::consumeImuDataReady())
    return;
  imu_.notifyDataReady();
  car::ImuSample sample{};
  const car::Status status = imu_.poll(sample);
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
  updateImu();
  processKeys(now);
  updateControl(now);
  refreshOled(now);
}

} // namespace app

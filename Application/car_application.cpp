#include "Application/car_application.hpp"
#include "BSP/encoder.hpp"
#include "BSP/system.hpp"
#include "BSP/uart.hpp"

namespace app {
namespace {
const char *lineStateText(middleware::LineTrackingState state) noexcept {
  switch (state) {
  case middleware::LineTrackingState::Tracking:
    return "LINE:TRACK";
  case middleware::LineTrackingState::Holding:
    return "LINE:HOLD";
  case middleware::LineTrackingState::Searching:
    return "LINE:SEARCH";
  case middleware::LineTrackingState::Lost:
    return "LINE:LOST";
  }
  return "LINE:LOST";
}

drivers::LedPattern
lineLedPattern(bool enabled, middleware::LineTrackingState state) noexcept {
  if (!enabled)
    return config::status_led::kLineDisabled;
  switch (state) {
  case middleware::LineTrackingState::Tracking:
    return config::status_led::kLineTracking;
  case middleware::LineTrackingState::Holding:
    return config::status_led::kLineHolding;
  case middleware::LineTrackingState::Searching:
    return config::status_led::kLineSearching;
  case middleware::LineTrackingState::Lost:
    return config::status_led::kLineLost;
  }
  return config::status_led::kDevicesNone;
}

void sendText(const char *text) noexcept {
  if (text == nullptr)
    return;
  std::size_t length = 0U;
  while (text[length] != '\0')
    ++length;
  (void)bsp::uartTryWrite(text, length);
}

void formatCruise(char *text, std::int16_t cruise) noexcept {
  text[0] = 'S';
  text[1] = 'P';
  text[2] = 'D';
  text[3] = ':';
  text[4] = static_cast<char>('0' + cruise / 100);
  text[5] = static_cast<char>('0' + (cruise / 10) % 10);
  text[6] = static_cast<char>('0' + cruise % 10);
  text[7] = '\0';
}
} // namespace

void CarApplication::init() noexcept {
  motor_.stop();
  leds_.setPattern(config::status_led::kStartup);
  leds_.service(bsp::millis());
  buzzer_.set(true);
  const std::uint32_t start = bsp::millis();
  while (static_cast<std::uint32_t>(bsp::millis() - start) <
         VEHICLE_TUNING_STARTUP_TONE_MS) {
  }
  buzzer_.set(false);
#if ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_DMP
  imuReady_ = dmpImu_.begin() == car::Status::Ok;
#else
  imuReady_ = rawImu_.begin() == car::Status::Ok;
  // Static gyro bias calibration. The car must be stationary during init();
  // a failure downgrades imuReady_ so downstream consumers refuse to start,
  // mirroring the H-question startup-state handling.
  if (imuReady_ && rawImu_.calibrateGyroBias() != car::Status::Ok)
    imuReady_ = false;
#if ATTITUDE_CONFIG_BACKEND != ATTITUDE_BACKEND_BMI270 || !BMI270_ONBOARD_FUSION
  imuReader_.reset();
  softwareAttitude_.reset();
#endif
#endif
  oledReady_ = oled_.begin() == car::Status::Ok;
  const std::uint8_t devices = static_cast<std::uint8_t>(imuReady_) +
                               static_cast<std::uint8_t>(oledReady_);
  if (devices == 0U)
    leds_.setPattern(config::status_led::kDevicesNone);
  else if (devices == 1U)
    leds_.setPattern(config::status_led::kDevicesOne);
  else
    leds_.setPattern(config::status_led::kDevicesTwo);
  leds_.service(bsp::millis());
}

void CarApplication::processKeyInteraction() noexcept {
  const bool left = keys_.pressed(car::Key::Left);
  const bool right = keys_.pressed(car::Key::Right);
  const bool down = keys_.pressed(car::Key::Down);
  const bool leftPressed = left && !leftWasPressed_;
  const bool rightPressed = right && !rightWasPressed_;
  const bool downPressed = down && !downWasPressed_;
  leftWasPressed_ = left;
  rightWasPressed_ = right;
  downWasPressed_ = down;

  // Configuration is deliberately unavailable while the safety gate permits
  // motion. This makes the five-way keypad useful without weakening the
  // CENTER + UP hold-to-run interlock.
  if (lineFollowEnabled_ || (!leftPressed && !rightPressed && !downPressed))
    return;

  const auto config = follower_.config();
  std::int16_t cruise = config.cruise;
  if (leftPressed && cruise >= LINE_FOLLOW_CRUISE_STEP)
    cruise = static_cast<std::int16_t>(cruise - LINE_FOLLOW_CRUISE_STEP);
  if (rightPressed &&
      cruise <= LINE_FOLLOW_MAX_CRUISE - LINE_FOLLOW_CRUISE_STEP)
    cruise = static_cast<std::int16_t>(cruise + LINE_FOLLOW_CRUISE_STEP);
  if (downPressed)
    cruise = LINE_FOLLOW_CRUISE;
  if (cruise != config.cruise)
    (void)follower_.configure(config.kp, config.ki, config.kd, cruise);
}

void CarApplication::refreshOled(std::uint32_t now) noexcept {
  if (!oledReady_)
    return;
  if (static_cast<std::uint32_t>(now - lastOledTextMs_) >=
      LINE_FOLLOW_OLED_TEXT_PERIOD_MS) {
    lastOledTextMs_ = now;
    char cruise[8]{};
    formatCruise(cruise, follower_.config().cruise);
    (void)oled_.writeLine(0U, lineFollowEnabled_ ? "MODE:RUN" : "MODE:HOLD");
    (void)oled_.writeLine(1U, cruise);
    (void)oled_.writeLine(2U, lineStateText(lineTrackingState_));
    (void)oled_.writeLine(3U, imuReady_ ? "IMU:OK" : "IMU:ERR");
  }
  if (static_cast<std::uint32_t>(now - lastOledServiceMs_) >=
      LINE_FOLLOW_OLED_SERVICE_PERIOD_MS) {
    lastOledServiceMs_ = now;
    const car::Status status = oled_.service();
    if (status != car::Status::Ok && status != car::Status::Busy)
      oledReady_ = false;
  }
}

void CarApplication::step() noexcept {
  const std::uint32_t now = bsp::millis();
  if (static_cast<std::uint32_t>(now - lastLineMs_) >=
      LINE_FOLLOW_CONTROL_PERIOD_MS) {
    const std::uint32_t elapsedMs =
        lastLineMs_ == 0U ? LINE_FOLLOW_CONTROL_PERIOD_MS
                          : static_cast<std::uint32_t>(now - lastLineMs_);
    lastLineMs_ = now;
    lineSample_ = line_.read();
    const auto result =
        follower_.update(lineSample_, static_cast<float>(elapsedMs) / 1000.0F);
    lineTrackingState_ = result.state;
    lineWheelCommand_ = drive_.mix(result.command);
  }
  if (imuReady_ && bsp::consumeImuDataReady()) {
    car::ImuSample sample{};
    car::Status status = car::Status::NotReady;
#if ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_DMP
    dmpImu_.notifyDataReady();
    status = dmpImu_.poll(sample);
#elif ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_BMI270 &&                    \
    BMI270_ONBOARD_FUSION
    status = rawImu_.poll(sample);
#else
    status = imuReader_.step(rawImu_, softwareAttitude_, now, sample);
#endif
    // Empty FIFO around the interrupt edge is normal; a transport/FIFO error is
    // not and must return the car to its stopped state.
    if (status != car::Status::Ok && status != car::Status::Busy) {
      imuReady_ = false;
      motor_.stop();
      buzzer_.set(true);
    } else if (status == car::Status::Ok) {
      imuSample_ = sample;
    }
  }
  const bool center = keys_.pressed(car::Key::Center);
  if (center && !centerWasPressed_)
    centerSinceMs_ = now;
  centerWasPressed_ = center;
  // Keep the enable action deliberate: CENTER arms the car, while UP is the
  // hold-to-run control. Releasing either key stops immediately.
  const bool armed =
      center && static_cast<std::uint32_t>(now - centerSinceMs_) >=
                    LINE_FOLLOW_ENABLE_HOLD_MS;
  lineFollowEnabled_ = armed && keys_.pressed(car::Key::Up);
  motor_.set(gate_.apply(lineWheelCommand_, lineFollowEnabled_));
  processKeyInteraction();

  std::uint8_t rxByte = 0U;
  middleware::VofaCommand vofaCommand{};
  while (bsp::uartTryRead(rxByte)) {
    if (!vofa_.consume(rxByte, vofaCommand))
      continue;
    if (vofaCommand.type == middleware::VofaCommandType::GetConfig) {
      char response[96]{};
      if (telemetry_.formatConfig(response, sizeof(response),
                                  follower_.config()))
        sendText(response);
    } else if (lineFollowEnabled_) {
      sendText("err:RUNNING\r\n");
    } else if (vofaCommand.type == middleware::VofaCommandType::SetLinePid) {
      const auto config = follower_.config();
      if (follower_.configure(vofaCommand.kp, vofaCommand.ki, vofaCommand.kd,
                              config.cruise))
        sendText("ack:LINE\r\n");
      else
        sendText("err:RANGE\r\n");
    } else if (vofaCommand.type == middleware::VofaCommandType::SetCruise) {
      const auto config = follower_.config();
      if (follower_.configure(config.kp, config.ki, config.kd,
                              vofaCommand.cruise))
        sendText("ack:CRUISE\r\n");
      else
        sendText("err:RANGE\r\n");
    } else {
      sendText("err:FORMAT\r\n");
    }
  }
  if (static_cast<std::uint32_t>(now - lastHeartbeatMs_) >=
      LINE_FOLLOW_HEARTBEAT_PERIOD_MS) {
    lastHeartbeatMs_ = now;
    heartbeat_ = !heartbeat_;
    leds_.setUser(heartbeat_);
  }
  if (static_cast<std::uint32_t>(now - lastTelemetryMs_) >=
      LINE_FOLLOW_TELEMETRY_PERIOD_MS) {
    lastTelemetryMs_ = now;
    char text[224]{};
    if (telemetry_.formatFrame(
            text, sizeof(text), lineSample_, encoder_.ticks(), imuSample_,
            motor_.command(), follower_.config(), lineFollowEnabled_, imuReady_,
            bsp::uartRxDroppedBytes(), bsp::uartTxDroppedFrames()))
      sendText(text);
  }
  leds_.setPattern(lineLedPattern(lineFollowEnabled_, lineTrackingState_));
  leds_.service(now);
  refreshOled(now);
}
} // namespace app

#include "Application/car_application.hpp"
#include "BSP/encoder.hpp"
#include "BSP/system.hpp"
#include "BSP/uart.hpp"

namespace app {
namespace {
void sendText(const char *text) noexcept {
  if (text == nullptr)
    return;
  std::size_t length = 0U;
  while (text[length] != '\0')
    ++length;
  (void)bsp::uartTryWrite(text, length);
}
} // namespace

void CarApplication::init() noexcept {
  motor_.stop();
  leds_.setStatus(0U, true);
  buzzer_.set(true);
  const std::uint32_t start = bsp::millis();
  while (static_cast<std::uint32_t>(bsp::millis() - start) < 30U) {
  }
  buzzer_.set(false);
#if ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_DMP
  imuReady_ = dmpImu_.begin() == car::Status::Ok;
#else
  imuReady_ = rawImu_.begin() == car::Status::Ok;
  softwareAttitude_.reset();
#endif
  oledReady_ = oled_.begin() == car::Status::Ok;
  leds_.setStatus(1U, imuReady_);
  leds_.setStatus(2U, oledReady_);
}
void CarApplication::step() noexcept {
  const std::uint32_t now = bsp::millis();
  if (static_cast<std::uint32_t>(now - lastLineMs_) >= 5U) {
    lastLineMs_ = now;
    lineSample_ = line_.read();
    const car::VehicleCommand command = follower_.update(lineSample_, 0.005F);
    lineWheelCommand_ = drive_.mix(command);
  }
  if (imuReady_ && bsp::consumeImuDataReady()) {
    car::ImuSample sample{};
    car::Status status = car::Status::NotReady;
#if ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_DMP
    dmpImu_.notifyDataReady();
    status = dmpImu_.poll(sample);
#else
    status = rawImu_.poll(sample);
    const std::uint32_t dtMs = lastImuMs_ == 0U ? 10U : now - lastImuMs_;
    lastImuMs_ = now;
    sample.timestampMs = now;
    if (status == car::Status::Ok)
      status =
          softwareAttitude_.update(sample, static_cast<float>(dtMs) / 1000.0F);
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
      center && static_cast<std::uint32_t>(now - centerSinceMs_) >= 500U;
  lineFollowEnabled_ = armed && keys_.pressed(car::Key::Up);
  motor_.set(gate_.apply(lineWheelCommand_, lineFollowEnabled_));

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
  if (static_cast<std::uint32_t>(now - lastHeartbeatMs_) >= 500U) {
    lastHeartbeatMs_ = now;
    heartbeat_ = !heartbeat_;
    leds_.setUser(heartbeat_);
  }
  if (static_cast<std::uint32_t>(now - lastTelemetryMs_) >= 50U) {
    lastTelemetryMs_ = now;
    char text[224]{};
    if (telemetry_.formatFrame(
            text, sizeof(text), lineSample_, encoder_.ticks(), imuSample_,
            motor_.command(), follower_.config(), lineFollowEnabled_, imuReady_,
            bsp::uartRxDroppedBytes(), bsp::uartTxDroppedFrames()))
      sendText(text);
    if (oledReady_) {
      const char *status = "HOLD CENTER+UP";
      if (lineFollowEnabled_)
        status = lineSample_.detected ? "LINE FOLLOWING" : "LINE LOST";
      (void)oled_.writeLine(status);
    }
  }
}
} // namespace app

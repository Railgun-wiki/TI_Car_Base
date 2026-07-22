#include "Application/car_application.hpp"
#include "BSP/encoder.hpp"
#include "BSP/system.hpp"
#include "BSP/uart.hpp"

namespace app {
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
    }
  }
  const bool center = keys_.pressed(car::Key::Center);
  if (center && !centerWasPressed_)
    centerSinceMs_ = now;
  centerWasPressed_ = center;
  // Demo only: long-hold CENTER is required, UP/DOWN choose direction, release
  // or timeout stops.
  const bool armed =
      center && static_cast<std::uint32_t>(now - centerSinceMs_) >= 500U;
  car::WheelCommand demo{0, 0};
  if (armed && keys_.pressed(car::Key::Up))
    demo = {180, 180};
  if (armed && keys_.pressed(car::Key::Down))
    demo = {-180, -180};
  motor_.set(gate_.apply(demo, armed));
  if (static_cast<std::uint32_t>(now - lastHeartbeatMs_) >= 500U) {
    lastHeartbeatMs_ = now;
    heartbeat_ = !heartbeat_;
    leds_.setUser(heartbeat_);
  }
  if (static_cast<std::uint32_t>(now - lastTelemetryMs_) >= 50U) {
    lastTelemetryMs_ = now;
    char text[96]{};
    if (telemetry_.format(text, sizeof(text), lineSample_, encoder_.ticks(),
                          imuReady_, oledReady_)) {
      for (char *p = text; *p != '\0'; p += 4U) {
        char part[4]{};
        std::size_t i = 0;
        for (; i < 4U && p[i] != '\0'; ++i)
          part[i] = p[i];
        if (!bsp::uartTryWrite(part, i))
          break;
      }
    }
    if (oledReady_)
      (void)oled_.writeLine(imuReady_ ? "CAR READY" : "IMU ERROR");
  }
}
} // namespace app

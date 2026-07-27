#include "Application/imu_oled_test_application.hpp"

#include <cstdio>

#include "BSP/system.hpp"

namespace app {
namespace {
constexpr std::uint32_t kImuPeriodMs = 10U;
constexpr std::uint32_t kOledTextPeriodMs = 500U;
constexpr std::uint32_t kOledServicePeriodMs = 2U;

bool elapsed(std::uint32_t now, std::uint32_t &last,
             std::uint32_t periodMs) noexcept {
  if (static_cast<std::uint32_t>(now - last) < periodMs)
    return false;
  last = now;
  return true;
}
} // namespace

void ImuOledTestApplication::init() noexcept {
  leds_.setUser(false);
  imuReady_ = imu_.begin() == car::Status::Ok;
  if (imuReady_) {
    gyroCalibrated_ = imu_.calibrateGyroBias() == car::Status::Ok;
    if (!gyroCalibrated_)
      imuReady_ = false;
  }
  if (imuReady_) {
    imuReader_.reset();
    attitude_.reset();
  }

  oledReady_ = oled_.begin() == car::Status::Ok;
  leds_.setStatus(1U, imuReady_);
  leds_.setStatus(2U, oledReady_);
  if (oledReady_)
    writeMeasurements();
}

void ImuOledTestApplication::updateImu(std::uint32_t now) noexcept {
  if (!imuReady_ || !elapsed(now, lastImuMs_, kImuPeriodMs))
    return;

  car::ImuSample sample{};
  const car::Status status = imuReader_.step(imu_, attitude_, now, sample);
  if (status == car::Status::Ok) {
    sample_ = sample;
    return;
  }
  if (status != car::Status::Busy) {
    imuReady_ = false;
    gyroCalibrated_ = false;
    leds_.setStatus(1U, false);
  }
}

void ImuOledTestApplication::writeMeasurements() noexcept {
  char line[21]{};
  (void)std::snprintf(line, sizeof(line), "IMU:%s CAL:%s",
                      imuReady_ ? "OK" : "ERR", gyroCalibrated_ ? "OK" : "ERR");
  (void)oled_.writeLine(0U, line);

  (void)std::snprintf(line, sizeof(line), "R:%+05.1f P:%+05.1f",
                      static_cast<double>(sample_.rollDeg),
                      static_cast<double>(sample_.pitchDeg));
  (void)oled_.writeLine(1U, line);
  (void)std::snprintf(line, sizeof(line), "Y:%+05.1f",
                      static_cast<double>(sample_.yawDeg));
  (void)oled_.writeLine(2U, line);
  (void)std::snprintf(line, sizeof(line), "AX:%+05.2f AY:%+05.2f",
                      static_cast<double>(sample_.ax),
                      static_cast<double>(sample_.ay));
  (void)oled_.writeLine(3U, line);
  (void)std::snprintf(line, sizeof(line), "AZ:%+05.2f",
                      static_cast<double>(sample_.az));
  (void)oled_.writeLine(4U, line);
  (void)std::snprintf(line, sizeof(line), "GX:%+05.1f GY:%+05.1f",
                      static_cast<double>(sample_.gx),
                      static_cast<double>(sample_.gy));
  (void)oled_.writeLine(5U, line);
  (void)std::snprintf(line, sizeof(line), "GZ:%+05.1f",
                      static_cast<double>(sample_.gz));
  (void)oled_.writeLine(6U, line);
  (void)oled_.writeLine(7U, "MAHONY 100HZ");
}

void ImuOledTestApplication::refreshOled(std::uint32_t now) noexcept {
  if (!oledReady_)
    return;
  if (elapsed(now, lastOledTextMs_, kOledTextPeriodMs))
    writeMeasurements();
  if (!elapsed(now, lastOledServiceMs_, kOledServicePeriodMs))
    return;

  const car::Status status = oled_.service();
  if (status != car::Status::Ok && status != car::Status::Busy) {
    oledReady_ = false;
    leds_.setStatus(2U, false);
  }
}

void ImuOledTestApplication::step() noexcept {
  const std::uint32_t now = bsp::millis();
  updateImu(now);
  refreshOled(now);
}

} // namespace app

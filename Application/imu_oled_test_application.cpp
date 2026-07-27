#include "Application/imu_oled_test_application.hpp"

#include <cmath>
#include <cstdio>

#include "BSP/system.hpp"
#include "BSP/uart.hpp"

namespace app {
namespace {
constexpr std::uint32_t kImuPeriodMs = 10U;
constexpr std::uint32_t kOledTextPeriodMs = 500U;
constexpr std::uint32_t kOledServicePeriodMs = 2U;
constexpr std::uint32_t kTelemetryPeriodMs = 50U;
constexpr std::uint32_t kGyroWarmupMs = 2000U;
constexpr std::uint16_t kGyroCalibrationSamples = 1000U;
constexpr std::uint16_t kGyroCalibrationIntervalMs = 5U;
constexpr std::uint32_t kStationaryDwellMs = 1000U;
constexpr float kStationaryAccelMinG = 0.98F;
constexpr float kStationaryAccelMaxG = 1.02F;
constexpr float kStationaryGyroMaxDps = 1.0F;

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
  leds_.setStatus(0U, true);

  // Bring up and visibly refresh the display before the deliberately blocking
  // warm-up and bias calibration, so the required stationary period is clear.
  oledReady_ = oled_.begin() == car::Status::Ok;
  if (oledReady_)
    showStartupStatus("IMU:WARMUP", "CAL:WAIT");

  imuReady_ = imu_.begin() == car::Status::Ok;
  if (imuReady_) {
    // MPU6050 bias changes during the first seconds after wake-up. Let the
    // sensor settle, then average five seconds of stationary data instead of
    // immediately using the normal application's one-second calibration.
    bsp::delayMs(kGyroWarmupMs);
    if (oledReady_)
      showStartupStatus("IMU:CALIBRATE", "CAL:RUNNING");
    gyroCalibrated_ =
        imu_.calibrateGyroBias(kGyroCalibrationSamples,
                               kGyroCalibrationIntervalMs) == car::Status::Ok;
    if (!gyroCalibrated_)
      imuReady_ = false;
  }
  if (imuReady_) {
    imuReader_.reset();
    attitude_.reset();
  }

  leds_.setStatus(1U, imuReady_);
  leds_.setStatus(2U, oledReady_);
  if (oledReady_)
    writeMeasurements();
}

void ImuOledTestApplication::showStartupStatus(const char *imuStatus,
                                               const char *calStatus) noexcept {
  (void)oled_.writeLine(0U, imuStatus);
  (void)oled_.writeLine(1U, "KEEP STILL");
  (void)oled_.writeLine(2U, calStatus);
  (void)oled_.writeLine(3U, "IMU TEST");
  serviceOledFor(500U);
}

void ImuOledTestApplication::serviceOledFor(std::uint32_t durationMs) noexcept {
  const std::uint32_t start = bsp::millis();
  while (oledReady_ &&
         static_cast<std::uint32_t>(bsp::millis() - start) < durationMs) {
    const car::Status status = oled_.service();
    if (status != car::Status::Ok && status != car::Status::Busy) {
      oledReady_ = false;
      break;
    }
    bsp::delayMs(2U);
  }
}

void ImuOledTestApplication::updateImu(std::uint32_t now) noexcept {
  if (!imuReady_ || !elapsed(now, lastImuMs_, kImuPeriodMs))
    return;

  car::ImuSample sample{};
  const car::Status status = imuReader_.step(imu_, attitude_, now, sample);
  if (status == car::Status::Ok) {
    sample_ = sample;
    stationary_ = stationary(sample_, now);
    if (stationary_)
      imu_.updateGyroBiasFromStationarySample(sample_, 0.01F);
    return;
  }
  if (status != car::Status::Busy) {
    imuReady_ = false;
    gyroCalibrated_ = false;
    stationary_ = false;
    leds_.setStatus(1U, false);
  }
}

bool ImuOledTestApplication::stationary(const car::ImuSample &sample,
                                        std::uint32_t now) noexcept {
  const float accelMagnitude = std::sqrt(
      sample.ax * sample.ax + sample.ay * sample.ay + sample.az * sample.az);
  const bool candidate = accelMagnitude >= kStationaryAccelMinG &&
                         accelMagnitude <= kStationaryAccelMaxG &&
                         std::fabs(sample.gx) <= kStationaryGyroMaxDps &&
                         std::fabs(sample.gy) <= kStationaryGyroMaxDps &&
                         std::fabs(sample.gz) <= kStationaryGyroMaxDps;
  if (!candidate) {
    stationarySinceMs_ = 0U;
    return false;
  }
  if (stationarySinceMs_ == 0U)
    stationarySinceMs_ = now;
  return static_cast<std::uint32_t>(now - stationarySinceMs_) >=
         kStationaryDwellMs;
}

void ImuOledTestApplication::publishTelemetry(std::uint32_t now) noexcept {
  if (!elapsed(now, lastTelemetryMs_, kTelemetryPeriodMs))
    return;
  char frame[224]{};
  if (telemetry_.formatImuFrame(frame, sizeof(frame), sample_, stationary_,
                                gyroCalibrated_, bsp::uartTxDroppedFrames())) {
    std::size_t length = 0U;
    while (frame[length] != '\0')
      ++length;
    (void)bsp::uartTryWrite(frame, length);
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
  publishTelemetry(now);
}

} // namespace app

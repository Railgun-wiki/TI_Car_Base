#include "Middlewares/imu_reader.hpp"
namespace middleware {

void ImuReader::reset() noexcept { lastMs_ = 0U; }

car::Status ImuReader::step(drivers::ImuBackend &imu,
                            middleware::AttitudeFilter &filter,
                            std::uint32_t nowMs, car::ImuSample &out) noexcept {
  car::Status status = imu.poll(out);
  const std::uint32_t dtMs = lastMs_ == 0U ? 10U : nowMs - lastMs_;
  lastMs_ = nowMs;
  out.timestampMs = nowMs;
  if (status == car::Status::Ok)
    status = filter.update(out, static_cast<float>(dtMs) / 1000.0F);
  return status;
}

} // namespace middleware

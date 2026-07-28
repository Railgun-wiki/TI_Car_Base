#pragma once
#include "Common/types.hpp"
#include "Middlewares/pid.hpp"
namespace middleware {
// Direction is abstract until probe numbering and motor polarity are confirmed
// on the real car. PositiveSide/NegativeSide map to error sign of LineSample.
enum class LineDirection : std::int8_t {
  None = 0,
  NegativeSide = -1,
  PositiveSide = 1
};

enum class LineTrackingState : std::uint8_t {
  Tracking,
  CornerArmed,
  Predicting,
  Searching,
  Cornering,
  Lost
};

struct LineFollowerResult final {
  car::VehicleCommand command;
  LineTrackingState state;
};

struct LineFollowerConfig final {
  float kp;
  float ki;
  float kd;
  std::int16_t cruise;
  std::uint32_t predictMs;
  std::uint32_t searchTimeoutMs;
  float predictSpeedRatio;
  float searchSpeedRatio;
  float errorFilterAlpha;
  float turnSlewPerSecond;
  std::uint32_t directionMemoryMs;
  std::uint32_t cornerConfirmMs;
  std::uint32_t cornerMemoryMs;
  std::uint32_t cornerTimeoutMs;
  float cornerSpeedRatio;
  std::uint8_t cornerSideMinCount;
  std::uint8_t cornerSideDominance;
  std::uint32_t reacquireConfirmMs;
};

class LineFollower final {
public:
  explicit LineFollower(LineFollowerConfig config) noexcept;
  LineFollowerResult update(const car::LineSample &sample,
                            float dtSeconds) noexcept;
  bool configure(float kp, float ki, float kd, std::int16_t cruise) noexcept;
  LineFollowerConfig config() const noexcept { return config_; }
  void reset() noexcept;

private:
  LineFollowerResult makeResult(std::int16_t linear, std::int16_t angular,
                                LineTrackingState state) noexcept;
  LineFollowerResult lostResult() noexcept;
  void rememberTracking(const car::LineSample &sample, float filteredError,
                        float turn, float dtMs) noexcept;
  void ageMemory(float dtMs) noexcept;
  bool directionReliable() const noexcept;
  bool cornerDirectionFresh() const noexcept;
  bool cornerConfirmed() const noexcept;
  LineFollowerConfig config_;
  Pid pid_;
  float stateElapsedMs_ = 0.0F;
  float filteredError_ = 0.0F;
  bool hasFilteredError_ = false;
  float lastTurn_ = 0.0F;
  float lastCommandTurn_ = 0.0F;
  float lastDtSeconds_ = 0.0F;
  float previousRawError_ = 0.0F;
  float errorTrend_ = 0.0F;
  float memoryAgeMs_ = 0.0F;
  float cornerCandidateMs_ = 0.0F;
  float cornerMemoryAgeMs_ = 0.0F;
  float reacquireMs_ = 0.0F;
  std::uint8_t lastBits_ = 0U;
  LineDirection lastDirection_ = LineDirection::None;
  LineDirection cornerDirection_ = LineDirection::None;
  LineTrackingState state_ = LineTrackingState::Lost;
};
} // namespace middleware

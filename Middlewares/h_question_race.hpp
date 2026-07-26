#pragma once

#include "Common/types.hpp"

namespace middleware {

enum class HProgram : std::uint8_t {
  Requirement1,
  Requirement2,
  Requirement3,
  Requirement4
};
enum class HRaceState : std::uint8_t {
  Menu,
  Countdown,
  Running,
  Checkpoint,
  Finished,
  Fault
};
enum class HSegmentType : std::uint8_t { Arc, Gap, Diagonal };

struct HSegment final {
  HSegmentType type;
  float distanceCm;
  float relativeHeadingDeg;
};

struct RaceConfig final {
  // Initial values inherited from the C07A reference setup. They must be
  // calibrated against the fitted wheel, gearbox and encoder hardware.
  float wheelDiameterMeters = 0.065F;
  float gearRatio = 28.0F;
  float encoderCountsPerMotorRevolution = 13.0F;
  float quadratureMultiplier = 2.0F;
  float arcLengthCm = 125.6F;
  float gapLengthCm = 100.0F;
  float diagonalLengthCm = 128.0F;
  float headingAtoCDeg = -38.7F;
  float headingBtoDDeg = -38.7F;
  std::int16_t arcCruise = 250;
  std::int16_t straightCruise = 350;
  float headingKp = 18.0F;
  float headingKi = 0.0F;
  float headingKd = 0.2F;
  std::uint32_t countdownMs = 3000U;
  std::uint32_t checkpointPauseMs = 200U;
};

struct HRaceSnapshot final {
  HRaceState state;
  HProgram program;
  HSegmentType segmentType;
  std::uint8_t segmentNumber;
  std::uint8_t segmentCount;
  std::uint8_t lap;
  std::uint8_t lapCount;
  float distanceCm;
  float targetDistanceCm;
  float targetYawDeg;
};

class HQuestionRace final {
public:
  explicit HQuestionRace(RaceConfig config = {}) noexcept;

  void selectNext() noexcept;
  void selectPrevious() noexcept;
  void start(std::uint32_t now, car::EncoderTicks ticks, float yawDeg) noexcept;
  void cancel() noexcept;
  void fail() noexcept;
  HRaceSnapshot update(std::uint32_t now, car::EncoderTicks ticks, float yawDeg,
                       bool lineDetected) noexcept;
  HRaceSnapshot snapshot() const noexcept;

private:
  void beginSegment(car::EncoderTicks ticks, float yawDeg) noexcept;
  float distanceCm(car::EncoderTicks ticks) const noexcept;
  float targetDistanceCm() const noexcept;
  float targetRelativeHeadingDeg() const noexcept;
  const HSegment &segment() const noexcept;
  std::uint8_t segmentCount() const noexcept;
  std::uint8_t lapCount() const noexcept;

  RaceConfig config_;
  HProgram program_ = HProgram::Requirement1;
  HRaceState state_ = HRaceState::Menu;
  std::uint8_t completedSegments_ = 0U;
  std::uint32_t stateSinceMs_ = 0U;
  car::EncoderTicks segmentStartTicks_{};
  float segmentYawOriginDeg_ = 0.0F;
  float lastDistanceCm_ = 0.0F;
};

} // namespace middleware

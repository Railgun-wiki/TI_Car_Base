#pragma once

#include "Common/types.hpp"

// Percentage of the original H-question speed targets. Use lower values for
// commissioning, then restore 200 for normal running.
#ifndef H_QUESTION_BASE_SPEED_PERCENT
#define H_QUESTION_BASE_SPEED_PERCENT 200
#endif

#if H_QUESTION_BASE_SPEED_PERCENT < 1 || H_QUESTION_BASE_SPEED_PERCENT > 200
#error "H_QUESTION_BASE_SPEED_PERCENT must be in the range 1..200"
#endif

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
  // 48 mm wheel, 28:1 gearbox, 13-line encoder and x4 quadrature decode.
  // These are initial values and require an on-car distance calibration.
  float wheelDiameterMeters = 0.048F;
  float gearRatio = 28.0F;
  float encoderCountsPerMotorRevolution = 13.0F;
  float quadratureMultiplier = 4.0F;
  float arcLengthCm = 125.6F;
  float gapLengthCm = 100.0F;
  float diagonalLengthCm = 128.0F;
  float headingAtoCDeg = -38.7F;
  float headingBtoDDeg = -38.7F;
  std::int16_t arcSpeedMmPerSecond = 50 * H_QUESTION_BASE_SPEED_PERCENT / 100;
  std::int16_t straightSpeedMmPerSecond =
      100 * H_QUESTION_BASE_SPEED_PERCENT / 100;
  std::int16_t minimumWheelSpeedMmPerSecond =
      40 * H_QUESTION_BASE_SPEED_PERCENT / 100;
  float headingKp = 10.0F;
  float headingKi = 1.0F;
  float headingKd = 3.0F;
  std::uint32_t countdownMs = 3000U;
  std::uint32_t checkpointPauseMs = 200U;
  float endpointArmRatio = 0.80F;
  std::uint8_t minimumLineSensors = 2U;
  std::uint8_t endpointConfirmTicks = 2U;
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
  void select(HProgram program) noexcept;
  void start(std::uint32_t now, car::EncoderTicks ticks, float yawDeg) noexcept;
  void cancel() noexcept;
  void fail() noexcept;
  HRaceSnapshot update(std::uint32_t now, car::EncoderTicks ticks, float yawDeg,
                       car::LineSample line) noexcept;
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
  std::uint8_t endpointConfirmCount_ = 0U;
};

} // namespace middleware

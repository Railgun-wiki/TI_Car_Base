#include "Middlewares/h_question_race.hpp"

namespace {
constexpr float kPi = 3.14159265F;

std::uint8_t countActiveSensors(std::uint8_t bits) noexcept {
  std::uint8_t count = 0U;
  while (bits != 0U) {
    count += static_cast<std::uint8_t>(bits & 1U);
    bits >>= 1U;
  }
  return count;
}

constexpr middleware::HSegment kPath1[] = {
    {middleware::HSegmentType::Gap, 0.0F, 0.0F}};
constexpr middleware::HSegment kPath2[] = {
    {middleware::HSegmentType::Arc, 500.0F, 0.0F},
};
constexpr middleware::HSegment kPath3[] = {
    {middleware::HSegmentType::Diagonal, 0.0F, -38.7F},
    {middleware::HSegmentType::Arc, 0.0F, 0.0F},
    {middleware::HSegmentType::Diagonal, 0.0F, -38.7F},
    {middleware::HSegmentType::Arc, 0.0F, 0.0F},
};
} // namespace

namespace middleware {

HQuestionRace::HQuestionRace(RaceConfig config) noexcept : config_(config) {}

void HQuestionRace::selectNext() noexcept {
  if (state_ != HRaceState::Menu)
    return;
  program_ =
      static_cast<HProgram>((static_cast<std::uint8_t>(program_) + 1U) % 4U);
}

void HQuestionRace::selectPrevious() noexcept {
  if (state_ != HRaceState::Menu)
    return;
  program_ =
      static_cast<HProgram>((static_cast<std::uint8_t>(program_) + 3U) % 4U);
}

void HQuestionRace::select(HProgram program) noexcept {
  if (state_ == HRaceState::Menu)
    program_ = program;
}

void HQuestionRace::start(std::uint32_t now, car::EncoderTicks ticks,
                          float yawDeg) noexcept {
  if (state_ != HRaceState::Menu && state_ != HRaceState::Finished)
    return;
  completedSegments_ = 0U;
  segmentStartTicks_ = ticks;
  segmentYawOriginDeg_ = yawDeg;
  stateSinceMs_ = now;
  state_ = HRaceState::Countdown;
}

void HQuestionRace::cancel() noexcept {
  completedSegments_ = 0U;
  state_ = HRaceState::Menu;
}

void HQuestionRace::fail() noexcept { state_ = HRaceState::Fault; }

HRaceSnapshot HQuestionRace::update(std::uint32_t now, car::EncoderTicks ticks,
                                    float yawDeg,
                                    car::LineSample line) noexcept {
  if (state_ == HRaceState::Countdown &&
      static_cast<std::uint32_t>(now - stateSinceMs_) >= config_.countdownMs) {
    state_ = HRaceState::Running;
    beginSegment(ticks, yawDeg);
  } else if (state_ == HRaceState::Running) {
    const HSegment &current = segment();
    const float traveled = distanceCm(ticks);
    const float targetDistance = targetDistanceCm();
    lastDistanceCm_ = traveled;
    const bool distanceReached = traveled >= targetDistance;
    const bool sensorArmed =
        traveled >= targetDistance * config_.endpointArmRatio;
    const bool sensorReached =
        current.distanceCm <= 0.0F && sensorArmed &&
        (current.type == HSegmentType::Arc
             ? !line.detected
             : countActiveSensors(line.bits) >= config_.minimumLineSensors);
    if (distanceReached) {
      endpointConfirmCount_ = 0U;
    } else if (sensorReached) {
      if (endpointConfirmCount_ < config_.endpointConfirmTicks)
        ++endpointConfirmCount_;
    } else {
      endpointConfirmCount_ = 0U;
    }
    if (distanceReached ||
        endpointConfirmCount_ >= config_.endpointConfirmTicks) {
      ++completedSegments_;
      stateSinceMs_ = now;
      state_ = completedSegments_ >= segmentCount() * lapCount()
                   ? HRaceState::Finished
                   : HRaceState::Checkpoint;
    }
  } else if (state_ == HRaceState::Checkpoint &&
             static_cast<std::uint32_t>(now - stateSinceMs_) >=
                 config_.checkpointPauseMs) {
    state_ = HRaceState::Running;
    beginSegment(ticks, yawDeg);
  }
  return snapshot();
}

HRaceSnapshot HQuestionRace::snapshot() const noexcept {
  const std::uint8_t count = segmentCount();
  const std::uint8_t current = count == 0U ? 0U : completedSegments_ % count;
  const HSegment &currentSegment = segment();
  return {state_,
          program_,
          currentSegment.type,
          current,
          count,
          static_cast<std::uint8_t>(completedSegments_ / count),
          lapCount(),
          lastDistanceCm_,
          targetDistanceCm(),
          segmentYawOriginDeg_ + targetRelativeHeadingDeg()};
}

void HQuestionRace::beginSegment(car::EncoderTicks ticks,
                                 float yawDeg) noexcept {
  segmentStartTicks_ = ticks;
  segmentYawOriginDeg_ = yawDeg;
  lastDistanceCm_ = 0.0F;
  endpointConfirmCount_ = 0U;
}

float HQuestionRace::distanceCm(car::EncoderTicks ticks) const noexcept {
  const float countsPerWheelTurn = config_.gearRatio *
                                   config_.encoderCountsPerMotorRevolution *
                                   config_.quadratureMultiplier;
  const float cmPerCount =
      (kPi * config_.wheelDiameterMeters * 100.0F) / countsPerWheelTurn;
  const std::int32_t left = ticks.left - segmentStartTicks_.left;
  const std::int32_t right = ticks.right - segmentStartTicks_.right;
  const float averagedCounts =
      static_cast<float>((left < 0 ? -left : left) +
                         (right < 0 ? -right : right)) /
      2.0F;
  return averagedCounts * cmPerCount;
}

float HQuestionRace::targetDistanceCm() const noexcept {
  if (segment().distanceCm > 0.0F)
    return segment().distanceCm;
  switch (segment().type) {
  case HSegmentType::Arc:
    return config_.arcLengthCm;
  case HSegmentType::Gap:
    return config_.gapLengthCm;
  case HSegmentType::Diagonal:
    return config_.diagonalLengthCm;
  }
  return 0.0F;
}

float HQuestionRace::targetRelativeHeadingDeg() const noexcept {
  const std::uint8_t index = completedSegments_ % segmentCount();
  if ((program_ == HProgram::Requirement3 ||
       program_ == HProgram::Requirement4) &&
      index == 0U)
    return config_.headingAtoCDeg;
  if ((program_ == HProgram::Requirement3 ||
       program_ == HProgram::Requirement4) &&
      index == 2U)
    return config_.headingBtoDDeg;
  return segment().relativeHeadingDeg;
}

const HSegment &HQuestionRace::segment() const noexcept {
  const std::uint8_t index = completedSegments_ % segmentCount();
  switch (program_) {
  case HProgram::Requirement1:
    return kPath1[index];
  case HProgram::Requirement2:
    return kPath2[index];
  case HProgram::Requirement3:
  case HProgram::Requirement4:
    return kPath3[index];
  }
  return kPath1[0];
}

std::uint8_t HQuestionRace::segmentCount() const noexcept {
  switch (program_) {
  case HProgram::Requirement1:
    return 1U;
  case HProgram::Requirement2:
    return 1U;
  case HProgram::Requirement3:
  case HProgram::Requirement4:
    return 4U;
  }
  return 1U;
}

std::uint8_t HQuestionRace::lapCount() const noexcept {
  return program_ == HProgram::Requirement4 ? 4U : 1U;
}

} // namespace middleware

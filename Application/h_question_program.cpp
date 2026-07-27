#include "Application/h_question_program.hpp"

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
} // namespace

namespace app {

HQuestionProgram::HQuestionProgram(HQuestionProgramConfig config) noexcept
    : config_(config) {}

void HQuestionProgram::selectNext() noexcept {
  if (state_ != HRaceState::Menu)
    return;
  program_ =
      static_cast<HProgram>((static_cast<std::uint8_t>(program_) + 1U) % 4U);
}

void HQuestionProgram::selectPrevious() noexcept {
  if (state_ != HRaceState::Menu)
    return;
  program_ =
      static_cast<HProgram>((static_cast<std::uint8_t>(program_) + 3U) % 4U);
}

void HQuestionProgram::select(HProgram program) noexcept {
  if (state_ == HRaceState::Menu)
    program_ = program;
}

void HQuestionProgram::start(std::uint32_t now, car::EncoderTicks ticks,
                             float yawDeg) noexcept {
  if (state_ != HRaceState::Menu && state_ != HRaceState::Finished)
    return;
  completedSegments_ = 0U;
  segmentStartTicks_ = ticks;
  segmentYawOriginDeg_ = yawDeg;
  stateSinceMs_ = now;
  state_ = HRaceState::Countdown;
}

void HQuestionProgram::cancel() noexcept {
  completedSegments_ = 0U;
  state_ = HRaceState::Menu;
}

void HQuestionProgram::fail() noexcept { state_ = HRaceState::Fault; }

HRaceSnapshot HQuestionProgram::update(std::uint32_t now, car::EncoderTicks ticks,
                                       float yawDeg, car::LineSample line) noexcept {
  if (state_ == HRaceState::Countdown &&
      static_cast<std::uint32_t>(now - stateSinceMs_) >= config_.countdownMs) {
    state_ = HRaceState::Running;
    beginSegment(ticks, yawDeg);
  } else if (state_ == HRaceState::Running) {
    const Segment current = segment();
    const float traveled = distanceCm(ticks);
    lastDistanceCm_ = traveled;
    const bool distanceReached = traveled >= current.distanceCm;
    const bool sensorReached =
        current.useSensorEndpoint &&
        traveled >= current.distanceCm * config_.endpointArmRatio &&
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

HRaceSnapshot HQuestionProgram::snapshot() const noexcept {
  const std::uint8_t count = segmentCount();
  const std::uint8_t current = completedSegments_ % count;
  const Segment currentSegment = segment();
  return {state_,
          program_,
          currentSegment.type,
          current,
          count,
          static_cast<std::uint8_t>(completedSegments_ / count),
          lapCount(),
          lastDistanceCm_,
          currentSegment.distanceCm,
          segmentYawOriginDeg_ + currentSegment.relativeHeadingDeg};
}

void HQuestionProgram::beginSegment(car::EncoderTicks ticks,
                                    float yawDeg) noexcept {
  segmentStartTicks_ = ticks;
  segmentYawOriginDeg_ = yawDeg;
  lastDistanceCm_ = 0.0F;
  endpointConfirmCount_ = 0U;
}

float HQuestionProgram::distanceCm(car::EncoderTicks ticks) const noexcept {
  const float countsPerWheelTurn = config_.gearRatio *
                                   config_.encoderCountsPerMotorRevolution *
                                   config_.quadratureMultiplier;
  const float cmPerCount =
      (kPi * config_.wheelDiameterMeters * 100.0F) / countsPerWheelTurn;
  const std::int32_t left = ticks.left - segmentStartTicks_.left;
  const std::int32_t right = ticks.right - segmentStartTicks_.right;
  const float averagedCounts =
      static_cast<float>((left < 0 ? -left : left) + (right < 0 ? -right : right)) /
      2.0F;
  return averagedCounts * cmPerCount;
}

HQuestionProgram::Segment HQuestionProgram::segment() const noexcept {
  const std::uint8_t index = completedSegments_ % segmentCount();
  switch (program_) {
  case HProgram::Requirement1:
    return {HSegmentType::Gap, config_.p1DistanceCm, 0.0F, true};
  case HProgram::Requirement2:
    return {HSegmentType::Arc, config_.p2DistanceCm, 0.0F, false};
  case HProgram::Requirement3:
  case HProgram::Requirement4:
    switch (index) {
    case 0U:
      return {HSegmentType::Diagonal, config_.diagonalDistanceCm,
              config_.headingAtoCDeg, true};
    case 1U:
      return {HSegmentType::Arc, config_.arcDistanceCm, 0.0F, true};
    case 2U:
      return {HSegmentType::Diagonal, config_.diagonalDistanceCm,
              config_.headingBtoDDeg, true};
    default:
      return {HSegmentType::Arc, config_.arcDistanceCm, 0.0F, true};
    }
  }
  return {HSegmentType::Gap, config_.p1DistanceCm, 0.0F, true};
}

std::uint8_t HQuestionProgram::segmentCount() const noexcept {
  return (program_ == HProgram::Requirement3 ||
          program_ == HProgram::Requirement4)
             ? 4U
             : 1U;
}

std::uint8_t HQuestionProgram::lapCount() const noexcept {
  return program_ == HProgram::Requirement4 ? 4U : 1U;
}

} // namespace app

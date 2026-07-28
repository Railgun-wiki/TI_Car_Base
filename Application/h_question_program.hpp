#pragma once

#include "Common/types.hpp"
#include "Config/vehicle_tuning.hpp"

namespace app {

enum class HProgram : std::uint8_t {
  Requirement1,
  Requirement2,
  Requirement3,
  Requirement4,
};
enum class HRaceState : std::uint8_t {
  Menu,
  Countdown,
  Running,
  Checkpoint,
  Finished,
  Fault,
};
enum class HSegmentType : std::uint8_t { Arc, Gap, Diagonal };

struct HQuestionProgramConfig final {
  float wheelDiameterMeters = VEHICLE_TUNING_WHEEL_DIAMETER_METERS;
  float gearRatio = VEHICLE_TUNING_GEAR_RATIO;
  float encoderCountsPerMotorRevolution =
      VEHICLE_TUNING_ENCODER_COUNTS_PER_MOTOR_REVOLUTION;
  float quadratureMultiplier = VEHICLE_TUNING_ENCODER_QUADRATURE_MULTIPLIER;
  float p1DistanceCm = H_QUESTION_P1_DISTANCE_CM;
  float p2DistanceCm = H_QUESTION_P2_DISTANCE_CM;
  float arcDistanceCm = H_QUESTION_ARC_DISTANCE_CM;
  float diagonalDistanceCm = H_QUESTION_DIAGONAL_DISTANCE_CM;
  float headingAtoCDeg = H_QUESTION_HEADING_A_TO_C_DEG;
  float headingBtoDDeg = H_QUESTION_HEADING_B_TO_D_DEG;
  float endpointArmRatio = H_QUESTION_ENDPOINT_ARM_RATIO;
  std::uint8_t minimumLineSensors = H_QUESTION_MINIMUM_LINE_SENSORS;
  std::uint32_t endpointConfirmMs = H_QUESTION_ENDPOINT_CONFIRM_MS;
  std::uint32_t countdownMs = H_QUESTION_COUNTDOWN_MS;
  std::uint32_t checkpointPauseMs = H_QUESTION_CHECKPOINT_PAUSE_MS;
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

class HQuestionProgram final {
public:
  explicit HQuestionProgram(HQuestionProgramConfig config = {}) noexcept;

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
  struct Segment final {
    HSegmentType type;
    float distanceCm;
    float relativeHeadingDeg;
    bool useSensorEndpoint;
  };

  void beginSegment(car::EncoderTicks ticks, float yawDeg) noexcept;
  float distanceCm(car::EncoderTicks ticks) const noexcept;
  Segment segment() const noexcept;
  std::uint8_t segmentCount() const noexcept;
  std::uint8_t lapCount() const noexcept;

  HQuestionProgramConfig config_;
  HProgram program_ = HProgram::Requirement1;
  HRaceState state_ = HRaceState::Menu;
  std::uint8_t completedSegments_ = 0U;
  std::uint32_t stateSinceMs_ = 0U;
  car::EncoderTicks segmentStartTicks_{};
  float segmentYawOriginDeg_ = 0.0F;
  float lastDistanceCm_ = 0.0F;
  std::uint32_t endpointCandidateSinceMs_ = 0U;
  bool endpointCandidateActive_ = false;
};

} // namespace app

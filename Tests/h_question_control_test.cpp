#if defined(H_QUESTION_HOST_TEST)

#include <cassert>

#include "Application/h_question_program.hpp"
#include "Config/status_led_config.hpp"
#include "Config/vehicle_tuning.hpp"
#include "Drivers/line_sensor_array.hpp"
#include "Middlewares/line_follower.hpp"
#include "Middlewares/wheel_speed_controller.hpp"

namespace {
std::uint8_t gHostLineBits = 0U;
}

namespace bsp {
std::uint8_t readLineBits() noexcept { return gHostLineBits; }
} // namespace bsp

namespace {

void testEndpointConfirmation() {
  app::HQuestionProgram race{};
  race.start(0U, {}, 0.0F);
  (void)race.update(3000U, {}, 0.0F, {});
  assert(race.snapshot().state == app::HRaceState::Running);

  const car::EncoderTicks armedTicks{8000, 8000};
  const car::LineSample checkpointLine{0x03U, 0, true, 0x03U};
  (void)race.update(3005U, armedTicks, 0.0F, checkpointLine);
  assert(race.snapshot().state == app::HRaceState::Running);
  (void)race.update(3044U, armedTicks, 0.0F, checkpointLine);
  assert(race.snapshot().state == app::HRaceState::Running);
  (void)race.update(3045U, armedTicks, 0.0F, checkpointLine);
  assert(race.snapshot().state == app::HRaceState::Finished);
}

void testDistanceFallbackAndFourLaps() {
  app::HQuestionProgram race{};
  race.select(app::HProgram::Requirement4);
  race.start(0U, {}, 0.0F);
  (void)race.update(3000U, {}, 0.0F, {});

  std::int32_t ticks = 0;
  for (std::uint8_t segment = 0U; segment < 16U; ++segment) {
    ticks += 15000;
    (void)race.update(3005U + segment * 205U, {ticks, ticks}, 0.0F, {});
    if (segment == 15U)
      break;
    assert(race.snapshot().state == app::HRaceState::Checkpoint);
    (void)race.update(3205U + segment * 205U, {ticks, ticks}, 0.0F, {});
    assert(race.snapshot().state == app::HRaceState::Running);
  }
  assert(race.snapshot().state == app::HRaceState::Finished);
}

void testRequirement2Is500CmLineFollow() {
  app::HQuestionProgram race{};
  race.select(app::HProgram::Requirement2);
  race.start(0U, {}, 0.0F);
  (void)race.update(3000U, {}, 0.0F, {});
  const auto started = race.snapshot();
  assert(started.state == app::HRaceState::Running);
  assert(started.segmentType == app::HSegmentType::Arc);
  assert(started.segmentCount == 1U);
  assert(started.targetDistanceCm == H_QUESTION_P2_DISTANCE_CM);

  // A loss of the line after the 80% distance gate cannot end this test early.
  const car::EncoderTicks armedTicks{44000, 44000};
  (void)race.update(3005U, armedTicks, 0.0F, {});
  assert(race.snapshot().state == app::HRaceState::Running);

  (void)race.update(3010U, {50000, 50000}, 0.0F, {});
  assert(race.snapshot().state == app::HRaceState::Finished);
}

void testArcEndpointRequiresContinuousLoss() {
  app::HQuestionProgram race{};
  race.select(app::HProgram::Requirement3);
  race.start(0U, {}, 0.0F);
  (void)race.update(3000U, {}, 0.0F, {});

  (void)race.update(3005U, {15000, 15000}, 0.0F, {});
  assert(race.snapshot().state == app::HRaceState::Checkpoint);
  (void)race.update(3205U, {15000, 15000}, 0.0F, {});
  assert(race.snapshot().state == app::HRaceState::Running);
  assert(race.snapshot().segmentType == app::HSegmentType::Arc);

  const car::EncoderTicks armedArcTicks{25000, 25000};
  (void)race.update(3210U, armedArcTicks, 0.0F, {});
  assert(race.snapshot().state == app::HRaceState::Running);
  (void)race.update(3249U, armedArcTicks, 0.0F, {});
  assert(race.snapshot().state == app::HRaceState::Running);
  (void)race.update(3250U, armedArcTicks, 0.0F, {});
  assert(race.snapshot().state == app::HRaceState::Checkpoint);
}

void testEndpointConfirmationHandlesTimeWrap() {
  app::HQuestionProgramConfig config{};
  config.countdownMs = 0U;
  app::HQuestionProgram race{config};
  constexpr std::uint32_t kStart = 0xFFFFFF00U;
  race.start(kStart, {}, 0.0F);
  (void)race.update(kStart, {}, 0.0F, {});
  const car::EncoderTicks armedTicks{8000, 8000};
  const car::LineSample checkpointLine{0x03U, 0, true, 0x03U};
  (void)race.update(0xFFFFFFEBU, armedTicks, 0.0F, checkpointLine);
  assert(race.snapshot().state == app::HRaceState::Running);
  (void)race.update(20U, armedTicks, 0.0F, checkpointLine);
  assert(race.snapshot().state == app::HRaceState::Finished);
}

void testCentralTuningConfig() {
  const app::HQuestionProgramConfig race{};
  assert(race.wheelDiameterMeters == VEHICLE_TUNING_WHEEL_DIAMETER_METERS);
  assert(race.gearRatio == VEHICLE_TUNING_GEAR_RATIO);
  assert(race.quadratureMultiplier ==
         VEHICLE_TUNING_ENCODER_QUADRATURE_MULTIPLIER);
  assert(race.p2DistanceCm == H_QUESTION_P2_DISTANCE_CM);
  assert(VEHICLE_TUNING_LINE_SENSOR_LINE_IS_HIGH == 1);
  assert(H_QUESTION_ARC_SPEED_MM_PER_SECOND == 200);
  assert(H_QUESTION_STRAIGHT_SPEED_MM_PER_SECOND == 250);
  assert(H_QUESTION_MINIMUM_WHEEL_SPEED_MM_PER_SECOND == 100);
  assert(H_QUESTION_SPEED_KP == 2.5F);
}

void testLineSensorPreservesRawBits() {
  drivers::LineSensorArray sensor{};
  gHostLineBits = 0x81U;
  const car::LineSample centered = sensor.read();
  assert(centered.rawBits == 0x81U);
  assert(centered.bits == 0x81U);
  assert(centered.detected);
  assert(centered.error == 0);

  gHostLineBits = 0U;
  const car::LineSample lost = sensor.read();
  assert(lost.rawBits == 0U);
  assert(lost.bits == 0U);
  assert(!lost.detected);
}

void testStatusLedConfiguration() {
  using drivers::LedMode;
  const auto predicting = config::status_led::kLinePredicting;
  assert(predicting.led1 == LedMode::On);
  assert(predicting.led2 == LedMode::Blink);
  assert(predicting.led3 == LedMode::Off);

  const auto cornerArmed = config::status_led::kLineCornerArmed;
  assert(cornerArmed.led1 == LedMode::Blink);
  assert(cornerArmed.led2 == LedMode::Blink);
  assert(cornerArmed.led3 == LedMode::Off);

  const auto finished = config::status_led::kRaceFinished;
  assert(finished.led1 == LedMode::On);
  assert(finished.led2 == LedMode::Off);
  assert(finished.led3 == LedMode::On);
}

void testSpeedControllerReset() {
  middleware::WheelSpeedController controller{{
      {0.048F, 28.0F, 13.0F, 4.0F},
      {4.0F, 4.0F, 0.0F, 250.0F, 20.0F},
      {4.0F, 4.0F, 0.0F, 250.0F, 20.0F},
      50U,
  }};
  const car::WheelCommand target{10, 10};
  assert(controller.update({}, 0U, target).left == 0);
  assert(controller.update({}, 5U, target).left == 0);
  const auto command = controller.update({}, 50U, target);
  assert(command.left > 0 && command.left <= 250);
  assert(controller.update({}, 55U, target).left == command.left);
  controller.reset();
  assert(controller.update({}, 60U, target).left == 0);
}

void testForwardTargetNeverReverses() {
  middleware::WheelSpeedController controller{{
      {0.048F, 28.0F, 13.0F, 4.0F},
      {4.0F, 4.0F, 0.0F, 250.0F, 20.0F},
      {4.0F, 4.0F, 0.0F, 250.0F, 20.0F},
      50U,
  }};
  const car::WheelCommand target{10, 10};
  (void)controller.update({}, 0U, target);
  const car::EncoderTicks overspeedTicks{1000, 1000};
  const auto command = controller.update(overspeedTicks, 50U, target);
  assert(command.left == 0 && command.right == 0);
}

void testForwardPwmRiseLimit() {
  middleware::WheelSpeedController controller{{
      {0.048F, 28.0F, 13.0F, 4.0F},
      {0.05F, 2.0F, 0.0F, 160.0F, 80.0F},
      {0.05F, 2.0F, 0.0F, 160.0F, 80.0F},
      50U,
      2,
  }};
  const car::WheelCommand target{100, 100};
  (void)controller.update({}, 0U, target);
  assert(controller.update({}, 50U, target).left == 2);
  assert(controller.update({}, 100U, target).left == 4);
}

middleware::LineFollowerConfig makeLineConfig() noexcept {
  return {30.0F,
          0.0F,
          0.0F,
          100,
          50U,
          300U,
          0.50F,
          0.35F,
          0.70F,
          4000.0F,
          200U,
          15U,
          100U,
          300U,
          0.35F,
          3U,
          2U,
          10U};
}

void testLineFollowerRecoveryStates() {
  middleware::LineFollower follower{makeLineConfig()};
  const auto tracking = follower.update({0x20U, 2, true, 0x20U}, 0.005F);
  assert(tracking.state == middleware::LineTrackingState::Tracking);
  assert(tracking.command.linear == 100);
  assert(tracking.command.angular < 0);

  const auto predicting = follower.update({}, 0.005F);
  assert(predicting.state == middleware::LineTrackingState::Predicting);
  assert(predicting.command.linear == 50);
  assert(predicting.command.angular <= 0);

  const auto stillPredicting = follower.update({}, 0.044F);
  assert(stillPredicting.state == middleware::LineTrackingState::Predicting);
  const auto searching = follower.update({}, 0.002F);
  assert(searching.state == middleware::LineTrackingState::Searching);
  assert(searching.command.linear == 35);
  // Steering changes are slew-limited when Predicting hands control to search.
  assert(searching.command.angular < 0);
  assert(searching.command.angular >= -50);

  const auto lost = follower.update({}, 0.250F);
  assert(lost.state == middleware::LineTrackingState::Lost);
  assert(lost.command.linear == 0 && lost.command.angular == 0);

  const auto reacquired = follower.update({0x08U, -1, true, 0x08U}, 0.005F);
  assert(reacquired.state == middleware::LineTrackingState::Tracking);
  assert(reacquired.command.linear == 100);
  assert(reacquired.command.angular > 0);
}

void testLineFollowerDoesNotGuessDirection() {
  middleware::LineFollower follower{makeLineConfig()};
  const auto noHistory = follower.update({}, 0.005F);
  assert(noHistory.state == middleware::LineTrackingState::Lost);
  assert(noHistory.command.linear == 0 && noHistory.command.angular == 0);

  (void)follower.update({0x18U, 0, true, 0x18U}, 0.005F);
  const auto centeredLoss = follower.update({}, 0.005F);
  assert(centeredLoss.state == middleware::LineTrackingState::Lost);

  (void)follower.update({0x20U, 2, true, 0x20U}, 0.005F);
  const auto invalidLargeDt = follower.update({}, 1000.0F);
  assert(invalidLargeDt.state == middleware::LineTrackingState::Predicting);
  const auto invalidNegativeDt = follower.update({}, -1.0F);
  assert(invalidNegativeDt.state == middleware::LineTrackingState::Predicting);
}

void testLineFollowerCornerArmAndTurn() {
  middleware::LineFollower follower{makeLineConfig()};
  // Positive-side wide black 0xE0 (bits 5,6,7) held for >=15 ms.
  for (int i = 0; i < 4; ++i) {
    const auto armed = follower.update({0xE0U, 5, true, 0xE0U}, 0.005F);
    if (i >= 2) {
      assert(armed.state == middleware::LineTrackingState::CornerArmed);
      assert(armed.command.linear == 35);
    }
  }
  const auto cornering = follower.update({}, 0.005F);
  assert(cornering.state == middleware::LineTrackingState::Cornering);
  assert(cornering.command.linear == 35);
  assert(cornering.command.angular == -35);

  // Center ordinary line reacquires promptly.
  const auto back = follower.update({0x18U, 0, true, 0x18U}, 0.005F);
  assert(back.state == middleware::LineTrackingState::Tracking);
}

void testLineFollowerRejectsAmbiguousWideBlack() {
  middleware::LineFollower follower{makeLineConfig()};
  (void)follower.update({0x18U, 0, true, 0x18U}, 0.005F);
  for (int i = 0; i < 5; ++i) {
    const auto cross = follower.update({0xFFU, 0, true, 0xFFU}, 0.005F);
    assert(cross.state == middleware::LineTrackingState::Tracking);
  }
  // Full white after ambiguous pattern with only zero error history → Lost.
  const auto lost = follower.update({}, 0.005F);
  assert(lost.state == middleware::LineTrackingState::Lost);
}

void testLineFollowerSingleFrameWideBlackIsNotCorner() {
  middleware::LineFollower follower{makeLineConfig()};
  (void)follower.update({0x20U, 2, true, 0x20U}, 0.005F);
  // One 5 ms frame of wide black is below the 15 ms confirm window.
  const auto oneFrame = follower.update({0xE0U, 5, true, 0xE0U}, 0.005F);
  assert(oneFrame.state == middleware::LineTrackingState::Tracking);
  const auto predicting = follower.update({}, 0.005F);
  assert(predicting.state == middleware::LineTrackingState::Predicting);
}

void testLineFollowerReacquireDoesNotReverseSteeringInOneTick() {
  middleware::LineFollower follower{makeLineConfig()};
  (void)follower.update({0x20U, 2, true, 0x20U}, 0.005F);
  const auto predicting = follower.update({}, 0.005F);
  assert(predicting.command.angular < 0);

  // Reacquire on the opposite side.  The filter must start from this sample,
  // but the command slew guard must prevent an immediate steering reversal.
  const auto reacquired = follower.update({0x08U, -2, true, 0x08U}, 0.005F);
  assert(reacquired.state == middleware::LineTrackingState::Tracking);
  assert(reacquired.command.angular <= 0);
}

} // namespace

int main() {
  testEndpointConfirmation();
  testDistanceFallbackAndFourLaps();
  testRequirement2Is500CmLineFollow();
  testArcEndpointRequiresContinuousLoss();
  testEndpointConfirmationHandlesTimeWrap();
  testSpeedControllerReset();
  testForwardTargetNeverReverses();
  testForwardPwmRiseLimit();
  testCentralTuningConfig();
  testLineSensorPreservesRawBits();
  testStatusLedConfiguration();
  testLineFollowerRecoveryStates();
  testLineFollowerDoesNotGuessDirection();
  testLineFollowerCornerArmAndTurn();
  testLineFollowerRejectsAmbiguousWideBlack();
  testLineFollowerSingleFrameWideBlackIsNotCorner();
  testLineFollowerReacquireDoesNotReverseSteeringInOneTick();
}

#endif

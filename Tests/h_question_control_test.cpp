#if defined(H_QUESTION_HOST_TEST)

#include <cassert>

#include "Application/h_question_program.hpp"
#include "Config/status_led_config.hpp"
#include "Config/vehicle_tuning.hpp"
#include "Middlewares/line_follower.hpp"
#include "Middlewares/wheel_speed_controller.hpp"

namespace {

void testEndpointConfirmation() {
  app::HQuestionProgram race{};
  race.start(0U, {}, 0.0F);
  (void)race.update(3000U, {}, 0.0F, {});
  assert(race.snapshot().state == app::HRaceState::Running);

  const car::EncoderTicks armedTicks{8000, 8000};
  const car::LineSample checkpointLine{0x03U, 0, true};
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
  const car::LineSample checkpointLine{0x03U, 0, true};
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
}

void testStatusLedConfiguration() {
  using drivers::LedMode;
  const auto holding = config::status_led::kLineHolding;
  assert(holding.led1 == LedMode::On);
  assert(holding.led2 == LedMode::Blink);
  assert(holding.led3 == LedMode::Off);

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

void testLineFollowerRecoveryStates() {
  middleware::LineFollower follower{
      {30.0F, 0.0F, 0.0F, 100, 150U, 600U, 0.50F, 0.35F}};
  const auto tracking = follower.update({0x20U, 2, true}, 0.005F);
  assert(tracking.state == middleware::LineTrackingState::Tracking);
  assert(tracking.command.linear == 100);
  assert(tracking.command.angular < 0);

  const auto holding = follower.update({}, 0.005F);
  assert(holding.state == middleware::LineTrackingState::Holding);
  assert(holding.command.linear == 50);
  assert(holding.command.angular == -50);

  const auto stillHolding = follower.update({}, 0.144F);
  assert(stillHolding.state == middleware::LineTrackingState::Holding);
  const auto searching = follower.update({}, 0.002F);
  assert(searching.state == middleware::LineTrackingState::Searching);
  assert(searching.command.linear == 35);
  assert(searching.command.angular == -35);

  const auto lost = follower.update({}, 0.450F);
  assert(lost.state == middleware::LineTrackingState::Lost);
  assert(lost.command.linear == 0 && lost.command.angular == 0);

  const auto reacquired = follower.update({0x08U, -1, true}, 0.005F);
  assert(reacquired.state == middleware::LineTrackingState::Tracking);
  assert(reacquired.command.linear == 100);
  assert(reacquired.command.angular > 0);
}

void testLineFollowerDoesNotGuessDirection() {
  middleware::LineFollower follower{
      {30.0F, 0.0F, 0.0F, 100, 150U, 600U, 0.50F, 0.35F}};
  const auto noHistory = follower.update({}, 0.005F);
  assert(noHistory.state == middleware::LineTrackingState::Lost);
  assert(noHistory.command.linear == 0 && noHistory.command.angular == 0);

  (void)follower.update({0x18U, 0, true}, 0.005F);
  const auto centeredLoss = follower.update({}, 0.005F);
  assert(centeredLoss.state == middleware::LineTrackingState::Lost);

  (void)follower.update({0x20U, 2, true}, 0.005F);
  const auto invalidLargeDt = follower.update({}, 1000.0F);
  assert(invalidLargeDt.state == middleware::LineTrackingState::Holding);
  const auto invalidNegativeDt = follower.update({}, -1.0F);
  assert(invalidNegativeDt.state == middleware::LineTrackingState::Holding);
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
  testStatusLedConfiguration();
  testLineFollowerRecoveryStates();
  testLineFollowerDoesNotGuessDirection();
}

#endif

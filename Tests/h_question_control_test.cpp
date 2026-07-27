#if defined(H_QUESTION_HOST_TEST)

#include <cassert>

#include "Application/h_question_program.hpp"
#include "Config/vehicle_tuning.hpp"
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
  (void)race.update(3010U, armedTicks, 0.0F, checkpointLine);
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

void testCentralTuningConfig() {
  const app::HQuestionProgramConfig race{};
  assert(race.wheelDiameterMeters == VEHICLE_TUNING_WHEEL_DIAMETER_METERS);
  assert(race.gearRatio == VEHICLE_TUNING_GEAR_RATIO);
  assert(race.quadratureMultiplier ==
         VEHICLE_TUNING_ENCODER_QUADRATURE_MULTIPLIER);
  assert(race.p2DistanceCm == H_QUESTION_P2_DISTANCE_CM);
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

} // namespace

int main() {
  testEndpointConfirmation();
  testDistanceFallbackAndFourLaps();
  testRequirement2Is500CmLineFollow();
  testSpeedControllerReset();
  testForwardTargetNeverReverses();
  testForwardPwmRiseLimit();
  testCentralTuningConfig();
}

#endif

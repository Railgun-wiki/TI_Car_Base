#if defined(H_QUESTION_HOST_TEST)

#include <cassert>

#include "Middlewares/h_question_race.hpp"
#include "Middlewares/wheel_speed_controller.hpp"

namespace {

void testEndpointConfirmation() {
  middleware::HQuestionRace race{};
  race.start(0U, {}, 0.0F);
  (void)race.update(3000U, {}, 0.0F, {});
  assert(race.snapshot().state == middleware::HRaceState::Running);

  const car::EncoderTicks armedTicks{5600, 5600};
  const car::LineSample checkpointLine{0x03U, 0, true};
  (void)race.update(3005U, armedTicks, 0.0F, checkpointLine);
  assert(race.snapshot().state == middleware::HRaceState::Running);
  (void)race.update(3010U, armedTicks, 0.0F, checkpointLine);
  assert(race.snapshot().state == middleware::HRaceState::Checkpoint);
}

void testDistanceFallbackAndFourLaps() {
  middleware::HQuestionRace race{};
  race.select(middleware::HProgram::Requirement4);
  race.start(0U, {}, 0.0F);
  (void)race.update(3000U, {}, 0.0F, {});

  std::int32_t ticks = 0;
  for (std::uint8_t segment = 0U; segment < 16U; ++segment) {
    ticks += 10000;
    (void)race.update(3005U + segment * 205U, {ticks, ticks}, 0.0F, {});
    if (segment == 15U)
      break;
    assert(race.snapshot().state == middleware::HRaceState::Checkpoint);
    (void)race.update(3205U + segment * 205U, {ticks, ticks}, 0.0F, {});
    assert(race.snapshot().state == middleware::HRaceState::Running);
  }
  assert(race.snapshot().state == middleware::HRaceState::Finished);
}

void testRequirement2Is500CmLineFollow() {
  middleware::HQuestionRace race{};
  race.select(middleware::HProgram::Requirement2);
  race.start(0U, {}, 0.0F);
  (void)race.update(3000U, {}, 0.0F, {});
  const auto started = race.snapshot();
  assert(started.state == middleware::HRaceState::Running);
  assert(started.segmentType == middleware::HSegmentType::Arc);
  assert(started.segmentCount == 1U);
  assert(started.targetDistanceCm == 500.0F);

  // A loss of the line after the 80% distance gate cannot end this test early.
  const car::EncoderTicks armedTicks{44000, 44000};
  (void)race.update(3005U, armedTicks, 0.0F, {});
  assert(race.snapshot().state == middleware::HRaceState::Running);

  (void)race.update(3010U, {50000, 50000}, 0.0F, {});
  assert(race.snapshot().state == middleware::HRaceState::Finished);
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
}

#endif

#include "Middlewares/line_follower.hpp"
namespace middleware {
namespace {
constexpr float kTurnLimit = 500.0F;
constexpr float kIntegralLimit = 100.0F;
constexpr float kMaximumDtSeconds = 1.0F;
// bits[0..3] = NegativeSide, bits[4..7] = PositiveSide (matches weight signs).
constexpr std::uint8_t kNegativeSideMask = 0x0FU;
constexpr std::uint8_t kPositiveSideMask = 0xF0U;
// Center-ish adjacent pair patterns used for clean reacquire after cornering.
constexpr std::uint8_t kCenterAdjacentMask = 0x3CU; // bits 2..5

float clampRatio(float value) noexcept {
  if (value < 0.0F)
    return 0.0F;
  return value > 1.0F ? 1.0F : value;
}

float clampAlpha(float value) noexcept {
  if (value <= 0.0F)
    return 0.01F;
  return value > 1.0F ? 1.0F : value;
}

std::uint8_t popcount8(std::uint8_t value) noexcept {
  std::uint8_t count = 0U;
  while (value != 0U) {
    count = static_cast<std::uint8_t>(count + (value & 1U));
    value = static_cast<std::uint8_t>(value >> 1);
  }
  return count;
}

LineFollowerConfig normalize(LineFollowerConfig config) noexcept {
  if (config.predictMs == 0U)
    config.predictMs = 1U;
  if (config.searchTimeoutMs <= config.predictMs)
    config.searchTimeoutMs = config.predictMs + 1U;
  if (config.cornerConfirmMs == 0U)
    config.cornerConfirmMs = 1U;
  if (config.cornerMemoryMs == 0U)
    config.cornerMemoryMs = 1U;
  if (config.cornerTimeoutMs == 0U)
    config.cornerTimeoutMs = 1U;
  if (config.directionMemoryMs == 0U)
    config.directionMemoryMs = 1U;
  if (config.reacquireConfirmMs == 0U)
    config.reacquireConfirmMs = 1U;
  if (config.cornerSideMinCount == 0U)
    config.cornerSideMinCount = 1U;
  if (config.cornerSideDominance == 0U)
    config.cornerSideDominance = 1U;
  config.predictSpeedRatio = clampRatio(config.predictSpeedRatio);
  config.searchSpeedRatio = clampRatio(config.searchSpeedRatio);
  config.cornerSpeedRatio = clampRatio(config.cornerSpeedRatio);
  config.errorFilterAlpha = clampAlpha(config.errorFilterAlpha);
  if (config.turnSlewPerSecond != config.turnSlewPerSecond ||
      config.turnSlewPerSecond <= 0.0F ||
      config.turnSlewPerSecond > 1.0e30F)
    config.turnSlewPerSecond = 1.0F;
  return config;
}

std::int16_t scaledCruise(std::int16_t cruise, float ratio) noexcept {
  return static_cast<std::int16_t>(static_cast<float>(cruise) * ratio);
}

float clampTurn(float turn, std::int16_t linear) noexcept {
  const float limit = static_cast<float>(linear >= 0 ? linear : -linear);
  if (turn > limit)
    return limit;
  return turn < -limit ? -limit : turn;
}

bool isFinite(float value) noexcept {
  return value == value && value <= 1.0e30F && value >= -1.0e30F;
}

LineDirection directionFromError(std::int16_t error) noexcept {
  if (error > 0)
    return LineDirection::PositiveSide;
  if (error < 0)
    return LineDirection::NegativeSide;
  return LineDirection::None;
}

// Angular sign for search/corner: keep the same convention as PID
// (positive error → negative angular → DifferentialDrive left faster).
std::int16_t searchAngular(LineDirection direction,
                           std::int16_t linear) noexcept {
  if (direction == LineDirection::PositiveSide)
    return static_cast<std::int16_t>(-linear);
  if (direction == LineDirection::NegativeSide)
    return linear;
  return 0;
}

bool isAmbiguousWideBlack(std::uint8_t bits) noexcept {
  if (bits == 0xFFU)
    return true;
  const std::uint8_t negative = popcount8(
      static_cast<std::uint8_t>(bits & kNegativeSideMask));
  const std::uint8_t positive = popcount8(
      static_cast<std::uint8_t>(bits & kPositiveSideMask));
  // Both sides wide, or nearly balanced large black area → cross/T/horizontal.
  if (negative >= 3U && positive >= 3U)
    return true;
  if (negative >= 2U && positive >= 2U &&
      (negative > positive ? negative - positive : positive - negative) < 2U)
    return true;
  return false;
}

LineDirection detectCornerCandidate(std::uint8_t bits, std::uint8_t minCount,
                                    std::uint8_t dominance) noexcept {
  if (isAmbiguousWideBlack(bits))
    return LineDirection::None;
  const std::uint8_t negative = popcount8(
      static_cast<std::uint8_t>(bits & kNegativeSideMask));
  const std::uint8_t positive = popcount8(
      static_cast<std::uint8_t>(bits & kPositiveSideMask));
  if (negative >= minCount &&
      negative >= static_cast<std::uint8_t>(positive + dominance))
    return LineDirection::NegativeSide;
  if (positive >= minCount &&
      positive >= static_cast<std::uint8_t>(negative + dominance))
    return LineDirection::PositiveSide;
  return LineDirection::None;
}

bool isOrdinaryLinePattern(std::uint8_t bits) noexcept {
  if (bits == 0U || isAmbiguousWideBlack(bits))
    return false;
  const std::uint8_t count = popcount8(bits);
  if (count == 1U)
    return true;
  if (count != 2U)
    return false;
  static constexpr std::uint8_t kAdjacentPairs[] = {
      0x03U, 0x06U, 0x0CU, 0x18U, 0x30U, 0x60U, 0xC0U};
  for (const auto pair : kAdjacentPairs) {
    if (bits == pair)
      return true;
  }
  return false;
}

bool isCenterAdjacentPattern(std::uint8_t bits) noexcept {
  if (!isOrdinaryLinePattern(bits))
    return false;
  // Immediate reacquire when the hit is near the array center.
  return (bits & kCenterAdjacentMask) != 0U &&
         (bits & static_cast<std::uint8_t>(~kCenterAdjacentMask)) == 0U;
}
} // namespace

LineFollower::LineFollower(LineFollowerConfig config) noexcept
    : config_(normalize(config)),
      pid_({config_.kp, config_.ki, config_.kd, kTurnLimit, kIntegralLimit}) {}

LineFollowerResult LineFollower::makeResult(std::int16_t linear,
                                            std::int16_t angular,
                                            LineTrackingState state) noexcept {
  state_ = state;
  if (state == LineTrackingState::Lost) {
    lastCommandTurn_ = 0.0F;
    return {{0, 0}, state};
  }
  const float requested = static_cast<float>(angular);
  const float maxStep = config_.turnSlewPerSecond * lastDtSeconds_;
  float limited = requested;
  if (maxStep > 0.0F) {
    const float minimum = lastCommandTurn_ - maxStep;
    const float maximum = lastCommandTurn_ + maxStep;
    if (limited < minimum)
      limited = minimum;
    if (limited > maximum)
      limited = maximum;
  }
  lastCommandTurn_ = limited;
  return {{linear, static_cast<std::int16_t>(limited)}, state};
}

LineFollowerResult LineFollower::lostResult() noexcept {
  pid_.reset();
  reacquireMs_ = 0.0F;
  return makeResult(0, 0, LineTrackingState::Lost);
}

bool LineFollower::directionReliable() const noexcept {
  return lastDirection_ != LineDirection::None &&
         memoryAgeMs_ <= static_cast<float>(config_.directionMemoryMs);
}

bool LineFollower::cornerDirectionFresh() const noexcept {
  return cornerDirection_ != LineDirection::None &&
         (state_ == LineTrackingState::Cornering ||
          cornerMemoryAgeMs_ <= static_cast<float>(config_.cornerMemoryMs));
}

bool LineFollower::cornerConfirmed() const noexcept {
  return cornerDirectionFresh() &&
         cornerCandidateMs_ >= static_cast<float>(config_.cornerConfirmMs);
}

void LineFollower::ageMemory(float dtMs) noexcept {
  if (lastDirection_ != LineDirection::None)
    memoryAgeMs_ += dtMs;
  // Once Cornering has started the direction is latched for the whole turn;
  // only age the pre-white corner arm window outside Cornering.
  if (cornerDirection_ != LineDirection::None &&
      state_ != LineTrackingState::Cornering)
    cornerMemoryAgeMs_ += dtMs;
  if (memoryAgeMs_ > static_cast<float>(config_.directionMemoryMs)) {
    lastDirection_ = LineDirection::None;
    memoryAgeMs_ = 0.0F;
  }
  if (state_ != LineTrackingState::Cornering &&
      cornerMemoryAgeMs_ > static_cast<float>(config_.cornerMemoryMs)) {
    cornerDirection_ = LineDirection::None;
    cornerMemoryAgeMs_ = 0.0F;
    cornerCandidateMs_ = 0.0F;
  }
}

void LineFollower::rememberTracking(const car::LineSample &sample,
                                    float filteredError, float turn,
                                    float dtMs) noexcept {
  lastBits_ = sample.bits;
  lastTurn_ = turn;
  errorTrend_ = filteredError - previousRawError_;
  previousRawError_ = filteredError;

  const LineDirection fromError = directionFromError(sample.error);
  if (fromError != LineDirection::None) {
    lastDirection_ = fromError;
    memoryAgeMs_ = 0.0F;
  } else if (lastDirection_ != LineDirection::None) {
    // A centred line is valid tracking data, but it is not turn evidence.
    // Do not keep a stale side preference indefinitely while tracking centre.
    memoryAgeMs_ += dtMs;
    if (memoryAgeMs_ > static_cast<float>(config_.directionMemoryMs)) {
      lastDirection_ = LineDirection::None;
      memoryAgeMs_ = 0.0F;
    }
  }

  const LineDirection candidate = detectCornerCandidate(
      sample.bits, config_.cornerSideMinCount, config_.cornerSideDominance);
  if (candidate != LineDirection::None) {
    if (candidate == cornerDirection_)
      cornerCandidateMs_ += dtMs;
    else {
      cornerDirection_ = candidate;
      cornerCandidateMs_ = dtMs;
    }
    cornerMemoryAgeMs_ = 0.0F;
  } else if (isAmbiguousWideBlack(sample.bits)) {
    // Cross / horizontal / T: do not latch a corner direction.
    cornerDirection_ = LineDirection::None;
    cornerCandidateMs_ = 0.0F;
    cornerMemoryAgeMs_ = 0.0F;
  } else if (cornerConfirmed()) {
    // Already confirmed: keep the latch briefly while the pattern normalizes.
    cornerMemoryAgeMs_ += dtMs;
  } else {
    // Unconfirmed single-frame / short noise must not arm a corner.
    cornerDirection_ = LineDirection::None;
    cornerCandidateMs_ = 0.0F;
    cornerMemoryAgeMs_ = 0.0F;
  }
}

LineFollowerResult LineFollower::update(const car::LineSample &sample,
                                        float dtSeconds) noexcept {
  if (!isFinite(dtSeconds) || !isFinite(static_cast<float>(sample.error)))
    return lostResult();

  const float validDt =
      dtSeconds > 0.0F && dtSeconds <= kMaximumDtSeconds ? dtSeconds : 0.0F;
  lastDtSeconds_ = validDt;
  const float dtMs = validDt * 1000.0F;

  // ---- Detected path ----
  if (sample.detected) {
    const float rawError = static_cast<float>(sample.error);
    const bool recovering = state_ == LineTrackingState::Predicting ||
                            state_ == LineTrackingState::Searching ||
                            state_ == LineTrackingState::Lost;
    if (recovering) {
      // Full white means no position measurement; initialise from the newly
      // observed line instead of blending it with the pre-loss error.
      filteredError_ = rawError;
      hasFilteredError_ = true;
    } else if (!hasFilteredError_) {
      filteredError_ = rawError;
      hasFilteredError_ = true;
    } else {
      filteredError_ = config_.errorFilterAlpha * rawError +
                       (1.0F - config_.errorFilterAlpha) * filteredError_;
    }

    // Cornering reacquire needs a clean ordinary line, not any black bit.
    if (state_ == LineTrackingState::Cornering) {
      const bool clean = isOrdinaryLinePattern(sample.bits);
      if (clean) {
        reacquireMs_ += dtMs;
        const bool confirmed =
            reacquireMs_ >= static_cast<float>(config_.reacquireConfirmMs) ||
            isCenterAdjacentPattern(sample.bits);
        if (confirmed) {
          pid_.reset();
          filteredError_ = rawError;
          hasFilteredError_ = true;
          reacquireMs_ = 0.0F;
          stateElapsedMs_ = 0.0F;
          state_ = LineTrackingState::Tracking;
          // Fall through into Tracking PID this cycle.
        } else {
          ageMemory(dtMs);
          stateElapsedMs_ += dtMs;
          if (stateElapsedMs_ >= static_cast<float>(config_.cornerTimeoutMs) ||
              !cornerDirectionFresh())
            return lostResult();
          const std::int16_t linear =
              scaledCruise(config_.cruise, config_.cornerSpeedRatio);
          return makeResult(linear, searchAngular(cornerDirection_, linear),
                            LineTrackingState::Cornering);
        }
      } else {
        // Wide black, noise, or still on the corner body — keep turning.
        reacquireMs_ = 0.0F;
        ageMemory(dtMs);
        stateElapsedMs_ += dtMs;
        if (stateElapsedMs_ >= static_cast<float>(config_.cornerTimeoutMs) ||
            !cornerDirectionFresh())
          return lostResult();
        const std::int16_t linear =
            scaledCruise(config_.cruise, config_.cornerSpeedRatio);
        return makeResult(linear, searchAngular(cornerDirection_, linear),
                          LineTrackingState::Cornering);
      }
    }

    // From Predicting/Searching/Lost/CornerArmed: any detection returns to
    // Tracking (CornerArmed may also stay armed below).
    if (recovering) {
      pid_.reset();
      stateElapsedMs_ = 0.0F;
      reacquireMs_ = 0.0F;
    }

    const float turn = pid_.update(0.0F, filteredError_, validDt);
    rememberTracking(sample, filteredError_, turn, dtMs);

    const LineDirection liveCandidate = detectCornerCandidate(
        sample.bits, config_.cornerSideMinCount, config_.cornerSideDominance);
    const bool cornerLive =
        cornerConfirmed() && liveCandidate == cornerDirection_ &&
        !isAmbiguousWideBlack(sample.bits);

    if (cornerLive) {
      // Still PID-steer, but reduce cruise; latch direction for later full-white.
      stateElapsedMs_ = 0.0F;
      const std::int16_t linear =
          scaledCruise(config_.cruise, config_.cornerSpeedRatio);
      const float limited = clampTurn(turn, linear);
      return makeResult(linear, static_cast<std::int16_t>(limited),
                        LineTrackingState::CornerArmed);
    }

    // Candidate faded back to ordinary line while armed → Tracking.
    if (state_ == LineTrackingState::CornerArmed &&
        isOrdinaryLinePattern(sample.bits)) {
      stateElapsedMs_ = 0.0F;
      return makeResult(config_.cruise, static_cast<std::int16_t>(turn),
                        LineTrackingState::Tracking);
    }

    // Armed but still somewhat wide / transitional: keep reduced speed while
    // the confirmed latch remains fresh.
    if (state_ == LineTrackingState::CornerArmed && cornerConfirmed()) {
      const std::int16_t linear =
          scaledCruise(config_.cruise, config_.cornerSpeedRatio);
      const float limited = clampTurn(turn, linear);
      return makeResult(linear, static_cast<std::int16_t>(limited),
                        LineTrackingState::CornerArmed);
    }

    stateElapsedMs_ = 0.0F;
    return makeResult(config_.cruise, static_cast<std::int16_t>(turn),
                      LineTrackingState::Tracking);
  }

  // ---- Full white path ----
  // Do not filter bits/lineDetected: full white is immediate.
  ageMemory(dtMs);
  reacquireMs_ = 0.0F;

  // Entering white from CornerArmed with latched corner → Cornering.
  if (state_ == LineTrackingState::CornerArmed && cornerConfirmed()) {
    pid_.reset();
    stateElapsedMs_ = 0.0F;
    const std::int16_t linear =
        scaledCruise(config_.cruise, config_.cornerSpeedRatio);
    return makeResult(linear, searchAngular(cornerDirection_, linear),
                      LineTrackingState::Cornering);
  }

  // Already cornering on white.
  if (state_ == LineTrackingState::Cornering) {
    stateElapsedMs_ += dtMs;
    if (stateElapsedMs_ >= static_cast<float>(config_.cornerTimeoutMs) ||
        !cornerDirectionFresh())
      return lostResult();
    const std::int16_t linear =
        scaledCruise(config_.cruise, config_.cornerSpeedRatio);
    return makeResult(linear, searchAngular(cornerDirection_, linear),
                      LineTrackingState::Cornering);
  }

  // Fresh loss from Tracking (or after corner arm expired without confirmation).
  if (state_ == LineTrackingState::Tracking ||
      state_ == LineTrackingState::CornerArmed) {
    pid_.reset();
    stateElapsedMs_ = 0.0F;
    // Only a confirmed pre-white corner latch may force Cornering.
    if (cornerConfirmed()) {
      const std::int16_t linear =
          scaledCruise(config_.cruise, config_.cornerSpeedRatio);
      return makeResult(linear, searchAngular(cornerDirection_, linear),
                        LineTrackingState::Cornering);
    }
    if (!directionReliable())
      return lostResult();
    // Fall into Predicting this cycle.
    state_ = LineTrackingState::Predicting;
  }

  if (state_ == LineTrackingState::Lost)
    return lostResult();

  if (!directionReliable())
    return lostResult();

  stateElapsedMs_ += dtMs;

  if (stateElapsedMs_ >= static_cast<float>(config_.searchTimeoutMs))
    return lostResult();

  if (stateElapsedMs_ >= static_cast<float>(config_.predictMs) ||
      state_ == LineTrackingState::Searching) {
    const std::int16_t linear =
        scaledCruise(config_.cruise, config_.searchSpeedRatio);
    return makeResult(linear, searchAngular(lastDirection_, linear),
                      LineTrackingState::Searching);
  }

  // Predicting: keep last turn trend, reduced speed, no reverse of turn sign.
  const std::int16_t linear =
      scaledCruise(config_.cruise, config_.predictSpeedRatio);
  float turn = clampTurn(lastTurn_, linear);
  // Disallow reversing relative to lastDirection during predict.
  if (lastDirection_ == LineDirection::PositiveSide && turn > 0.0F)
    turn = 0.0F;
  if (lastDirection_ == LineDirection::NegativeSide && turn < 0.0F)
    turn = 0.0F;
  return makeResult(linear, static_cast<std::int16_t>(turn),
                    LineTrackingState::Predicting);
}

bool LineFollower::configure(float kp, float ki, float kd,
                             std::int16_t cruise) noexcept {
  if (kp < 0.0F || kp > 300.0F || ki < 0.0F || ki > 30.0F || kd < 0.0F ||
      kd > 100.0F || cruise < 0 || cruise > 500)
    return false;
  config_.kp = kp;
  config_.ki = ki;
  config_.kd = kd;
  config_.cruise = cruise;
  pid_.configure({kp, ki, kd, kTurnLimit, kIntegralLimit});
  reset();
  return true;
}

void LineFollower::reset() noexcept {
  pid_.reset();
  stateElapsedMs_ = 0.0F;
  filteredError_ = 0.0F;
  hasFilteredError_ = false;
  lastTurn_ = 0.0F;
  lastCommandTurn_ = 0.0F;
  lastDtSeconds_ = 0.0F;
  previousRawError_ = 0.0F;
  errorTrend_ = 0.0F;
  memoryAgeMs_ = 0.0F;
  cornerCandidateMs_ = 0.0F;
  cornerMemoryAgeMs_ = 0.0F;
  reacquireMs_ = 0.0F;
  lastBits_ = 0U;
  lastDirection_ = LineDirection::None;
  cornerDirection_ = LineDirection::None;
  state_ = LineTrackingState::Lost;
}
} // namespace middleware

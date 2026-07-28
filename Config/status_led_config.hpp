#pragma once

#include "Drivers/led.hpp"

namespace config {
namespace status_led {

using drivers::LedMode;
using drivers::LedPattern;

// LED2 is the center lamp. LED1 and LED3 are mirror-equivalent side lamps:
// swapping LED1/LED3 preserves the semantic state.
constexpr LedPattern kStartup{LedMode::Blink, LedMode::Blink, LedMode::Blink};
constexpr LedPattern kDevicesNone{LedMode::Off, LedMode::Off, LedMode::Off};
constexpr LedPattern kDevicesOne{LedMode::On, LedMode::Off, LedMode::Off};
constexpr LedPattern kDevicesTwo{LedMode::On, LedMode::On, LedMode::Off};

constexpr LedPattern kLineDisabled{LedMode::On, LedMode::Off, LedMode::On};
constexpr LedPattern kLineTracking{LedMode::On, LedMode::On, LedMode::On};
constexpr LedPattern kLineHolding{LedMode::On, LedMode::Blink, LedMode::Off};
constexpr LedPattern kLineSearching{LedMode::Blink, LedMode::On, LedMode::Off};
constexpr LedPattern kLineLost{LedMode::Off, LedMode::Blink, LedMode::Off};

constexpr LedPattern kRaceMenu{LedMode::On, LedMode::On, LedMode::Off};
constexpr LedPattern kRaceCountdown{LedMode::On, LedMode::Blink, LedMode::Off};
constexpr LedPattern kRaceRunning{LedMode::On, LedMode::On, LedMode::On};
constexpr LedPattern kRaceCheckpoint{LedMode::Blink, LedMode::On, LedMode::Off};
constexpr LedPattern kRaceFinished{LedMode::On, LedMode::Off, LedMode::On};
constexpr LedPattern kRaceFault{LedMode::Off, LedMode::Blink, LedMode::Off};

constexpr LedPattern kMotorIdle{LedMode::On, LedMode::Off, LedMode::On};
constexpr LedPattern kMotorRunning{LedMode::On, LedMode::On, LedMode::On};

} // namespace status_led
} // namespace config

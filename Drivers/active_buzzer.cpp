#include "Drivers/active_buzzer.hpp"
#include "BSP/indicator.hpp"
namespace drivers {
void ActiveBuzzer::set(bool on) noexcept { ::bsp::setBuzzer(on); }
} // namespace drivers

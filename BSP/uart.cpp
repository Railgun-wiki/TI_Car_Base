#include "BSP/uart.hpp"
#include "ti_msp_dl_config.h"
namespace bsp {
bool uartTryWrite(const char *data, std::size_t length) noexcept {
  if (data == nullptr || length == 0U || length > 4U ||
      DL_UART_Main_isBusy(UART_CONSOLE_INST))
    return false;
  DL_UART_Main_fillTXFIFO(UART_CONSOLE_INST,
                          reinterpret_cast<const std::uint8_t *>(data), length);
  return true;
}
} // namespace bsp

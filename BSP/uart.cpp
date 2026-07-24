#include "BSP/uart.hpp"
#include "ti_msp_dl_config.h"

namespace {
constexpr std::size_t kTxCapacity = 256U;
constexpr std::size_t kRxCapacity = 128U;
std::uint8_t g_tx[kTxCapacity]{};
std::uint8_t g_rx[kRxCapacity]{};
volatile std::size_t g_txHead = 0U;
volatile std::size_t g_txTail = 0U;
volatile std::size_t g_rxHead = 0U;
volatile std::size_t g_rxTail = 0U;
volatile std::uint32_t g_rxDropped = 0U;
volatile std::uint32_t g_txDropped = 0U;

std::size_t next(std::size_t index, std::size_t capacity) noexcept {
  return (index + 1U) % capacity;
}

void serviceTx() noexcept {
  while (g_txTail != g_txHead &&
         !DL_UART_Main_isTXFIFOFull(UART_CONSOLE_INST)) {
    DL_UART_Main_transmitData(UART_CONSOLE_INST, g_tx[g_txTail]);
    g_txTail = next(g_txTail, kTxCapacity);
  }
  if (g_txTail == g_txHead)
    DL_UART_Main_disableInterrupt(UART_CONSOLE_INST, DL_UART_INTERRUPT_TX);
}

void serviceRx() noexcept {
  while (!DL_UART_Main_isRXFIFOEmpty(UART_CONSOLE_INST)) {
    const std::uint8_t byte = DL_UART_Main_receiveData(UART_CONSOLE_INST);
    const std::size_t nextHead = next(g_rxHead, kRxCapacity);
    if (nextHead == g_rxTail) {
      ++g_rxDropped;
    } else {
      g_rx[g_rxHead] = byte;
      g_rxHead = nextHead;
    }
  }
}
} // namespace

namespace bsp {
bool uartTryWrite(const char *data, std::size_t length) noexcept {
  if (data == nullptr || length == 0U || length >= kTxCapacity)
    return false;

  const std::uint32_t interruptState = __get_PRIMASK();
  __disable_irq();
  std::size_t free = 0U;
  for (std::size_t cursor = g_txHead; next(cursor, kTxCapacity) != g_txTail;
       cursor = next(cursor, kTxCapacity))
    ++free;
  if (length > free) {
    ++g_txDropped;
    if (interruptState == 0U)
      __enable_irq();
    return false;
  }
  for (std::size_t index = 0U; index < length; ++index) {
    g_tx[g_txHead] = static_cast<std::uint8_t>(data[index]);
    g_txHead = next(g_txHead, kTxCapacity);
  }
  DL_UART_Main_enableInterrupt(UART_CONSOLE_INST, DL_UART_INTERRUPT_TX);
  serviceTx();
  if (interruptState == 0U)
    __enable_irq();
  return true;
}

bool uartTryRead(std::uint8_t &byte) noexcept {
  if (g_rxTail == g_rxHead)
    return false;
  byte = g_rx[g_rxTail];
  g_rxTail = next(g_rxTail, kRxCapacity);
  return true;
}

std::uint32_t uartRxDroppedBytes() noexcept { return g_rxDropped; }
std::uint32_t uartTxDroppedFrames() noexcept { return g_txDropped; }
} // namespace bsp

extern "C" void UART_CONSOLE_INST_IRQHandler(void) {
  switch (DL_UART_Main_getPendingInterrupt(UART_CONSOLE_INST)) {
  case DL_UART_MAIN_IIDX_RX:
    serviceRx();
    break;
  case DL_UART_MAIN_IIDX_TX:
    serviceTx();
    break;
  default:
    break;
  }
}

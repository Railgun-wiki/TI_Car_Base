#include "BSP/i2c.hpp"
#include "BSP/system.hpp"
#include "ti_msp_dl_config.h"
namespace {
struct OledDmaTransfer final {
  volatile bool busy = false;
  volatile car::Status status = car::Status::Ok;
  volatile std::uint32_t startedAtMs = 0U;
  volatile std::uint32_t timeoutMs = 0U;
};

OledDmaTransfer g_oledDma{};

I2C_Regs *instance(std::uint8_t bus) {
  return bus == 0U ? I2C_MPU6050_INST : I2C_OLED_INST;
}
bool idle(I2C_Regs *i, std::uint32_t timeout) {
  const auto start = bsp::millis();
  while ((DL_I2C_getControllerStatus(i) & DL_I2C_CONTROLLER_STATUS_IDLE) == 0U)
    if (static_cast<std::uint32_t>(bsp::millis() - start) >= timeout)
      return false;
  return true;
}
car::Status result(I2C_Regs *i) {
  return (DL_I2C_getControllerStatus(i) & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U
             ? car::Status::BusError
             : car::Status::Ok;
}

void finishOledDma(car::Status status) noexcept {
  DL_DMA_disableChannel(DMA, DMA_OLED_TX_CHAN_ID);
  g_oledDma.status = status;
  g_oledDma.busy = false;
}
} // namespace
namespace bsp {
car::Status i2cProbe(std::uint8_t bus, std::uint8_t address,
                     std::uint32_t timeout) noexcept {
  I2C_Regs *i = instance(bus);
  if (!idle(i, timeout))
    return car::Status::Timeout;

  DL_I2C_resetControllerTransfer(i);
  DL_I2C_flushControllerTXFIFO(i);
  DL_I2C_startControllerTransfer(i, address, DL_I2C_CONTROLLER_DIRECTION_TX,
                                 0U);
  delay_cycles(3U); // MSPM0 I2C_ERR_13.
  const auto start = millis();
  while ((DL_I2C_getControllerStatus(i) & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U)
    if (static_cast<std::uint32_t>(millis() - start) >= timeout)
      return car::Status::Timeout;
  return result(i);
}

car::Status i2cWrite(std::uint8_t bus, std::uint8_t address,
                     const std::uint8_t *data, std::size_t length,
                     std::uint32_t timeout) noexcept {
  if (data == nullptr || length == 0U || length > 8U)
    return car::Status::InvalidArgument;
  I2C_Regs *i = instance(bus);
  if (!idle(i, timeout))
    return car::Status::Timeout;
  DL_I2C_fillControllerTXFIFO(i, data, length);
  DL_I2C_startControllerTransfer(i, address, DL_I2C_CONTROLLER_DIRECTION_TX,
                                 length);
  delay_cycles(3U); // MSPM0 I2C_ERR_13.
  const auto start = millis();
  while ((DL_I2C_getControllerStatus(i) & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U)
    if (static_cast<std::uint32_t>(millis() - start) >= timeout)
      return car::Status::Timeout;
  return result(i);
}
car::Status i2cWriteRegister(std::uint8_t bus, std::uint8_t address,
                             std::uint8_t reg, const std::uint8_t *data,
                             std::size_t length,
                             std::uint32_t timeout) noexcept {
  if (data == nullptr || length == 0U || length > 0x0ff0U)
    return car::Status::InvalidArgument;
  I2C_Regs *i = instance(bus);
  if (!idle(i, timeout))
    return car::Status::Timeout;

  DL_I2C_resetControllerTransfer(i);
  DL_I2C_flushControllerTXFIFO(i);
  DL_I2C_fillControllerTXFIFO(i, &reg, 1U);
  std::size_t queued = DL_I2C_fillControllerTXFIFO(
      i, data, static_cast<std::uint16_t>(length > 7U ? 7U : length));
  DL_I2C_startControllerTransfer(i, address, DL_I2C_CONTROLLER_DIRECTION_TX,
                                 static_cast<std::uint16_t>(length + 1U));
  delay_cycles(3U); // MSPM0 I2C_ERR_13.

  const auto start = millis();
  while (queued < length) {
    if ((DL_I2C_getControllerStatus(i) & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U)
      return car::Status::BusError;
    const std::size_t available = DL_I2C_getControllerTXFIFOCounter(i);
    if (available != 0U) {
      const std::size_t remaining = length - queued;
      const std::size_t chunk = remaining < available ? remaining : available;
      const auto wrote = DL_I2C_fillControllerTXFIFO(
          i, data + queued, static_cast<std::uint16_t>(chunk));
      queued += wrote;
    }
    if (static_cast<std::uint32_t>(millis() - start) >= timeout)
      return car::Status::Timeout;
  }
  while ((DL_I2C_getControllerStatus(i) & DL_I2C_CONTROLLER_STATUS_BUSY) != 0U)
    if (static_cast<std::uint32_t>(millis() - start) >= timeout)
      return car::Status::Timeout;
  return result(i);
}
car::Status i2cReadRegister(std::uint8_t bus, std::uint8_t address,
                            std::uint8_t reg, std::uint8_t *data,
                            std::size_t length,
                            std::uint32_t timeout) noexcept {
  if (data == nullptr || length == 0U)
    return car::Status::InvalidArgument;
  const auto status = i2cWrite(bus, address, &reg, 1U, timeout);
  if (status != car::Status::Ok)
    return status;
  I2C_Regs *i = instance(bus);
  if (!idle(i, timeout))
    return car::Status::Timeout;
  DL_I2C_startControllerTransfer(i, address, DL_I2C_CONTROLLER_DIRECTION_RX,
                                 length);
  const auto start = millis();
  for (std::size_t n = 0; n < length; ++n) {
    while (DL_I2C_isControllerRXFIFOEmpty(i))
      if (static_cast<std::uint32_t>(millis() - start) >= timeout)
        return car::Status::Timeout;
    data[n] = DL_I2C_receiveControllerData(i);
  }
  return result(i);
}

car::Status i2cOledWriteDma(std::uint8_t address, const std::uint8_t *data,
                            std::size_t length,
                            std::uint32_t timeout) noexcept {
  if (data == nullptr || length == 0U || length > 8U)
    return car::Status::InvalidArgument;
  if (g_oledDma.busy)
    return car::Status::Busy;
  if ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
       DL_I2C_CONTROLLER_STATUS_IDLE) == 0U)
    return car::Status::Busy;

  DL_I2C_resetControllerTransfer(I2C_OLED_INST);
  DL_I2C_flushControllerTXFIFO(I2C_OLED_INST);
  DL_DMA_disableChannel(DMA, DMA_OLED_TX_CHAN_ID);
  DL_DMA_setSrcAddr(
      DMA, DMA_OLED_TX_CHAN_ID,
      static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(data)));
  DL_DMA_setDestAddr(
      DMA, DMA_OLED_TX_CHAN_ID,
      static_cast<std::uint32_t>(
          reinterpret_cast<std::uintptr_t>(&I2C_OLED_INST->MASTER.MTXDATA)));
  DL_DMA_setTransferSize(DMA, DMA_OLED_TX_CHAN_ID,
                         static_cast<std::uint16_t>(length));

  g_oledDma.status = car::Status::Busy;
  g_oledDma.startedAtMs = millis();
  g_oledDma.timeoutMs = timeout;
  g_oledDma.busy = true;
  DL_DMA_enableChannel(DMA, DMA_OLED_TX_CHAN_ID);
  DL_I2C_startControllerTransfer(I2C_OLED_INST, address,
                                 DL_I2C_CONTROLLER_DIRECTION_TX, length);
  delay_cycles(3U); // MSPM0 I2C_ERR_13.
  return car::Status::Ok;
}

car::Status i2cOledDmaStatus() noexcept {
  if (g_oledDma.busy &&
      static_cast<std::uint32_t>(millis() - g_oledDma.startedAtMs) >=
          g_oledDma.timeoutMs) {
    DL_I2C_resetControllerTransfer(I2C_OLED_INST);
    finishOledDma(car::Status::Timeout);
  }
  return g_oledDma.status;
}
} // namespace bsp

extern "C" void I2C1_IRQHandler(void) {
  // OLED I2C (I2C1) TX-complete path. RX is handled in the controller-mode
  // polling code above via DL_I2C_fillControllerTXFIFO; this handler only
  // finalizes DMA-driven OLED writes. Intentionally narrow scope.
  switch (DL_I2C_getPendingInterrupt(I2C_OLED_INST)) {
  case DL_I2C_IIDX_CONTROLLER_TX_DONE:
    if (g_oledDma.busy)
      finishOledDma(result(I2C_OLED_INST));
    break;
  case DL_I2C_IIDX_CONTROLLER_NACK:
  case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
    if (g_oledDma.busy)
      finishOledDma(car::Status::BusError);
    break;
  default:
    break;
  }
}

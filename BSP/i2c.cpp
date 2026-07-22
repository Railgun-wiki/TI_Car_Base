#include "BSP/i2c.hpp"
#include "BSP/system.hpp"
#include "ti_msp_dl_config.h"
namespace {
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
} // namespace
namespace bsp {
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
} // namespace bsp

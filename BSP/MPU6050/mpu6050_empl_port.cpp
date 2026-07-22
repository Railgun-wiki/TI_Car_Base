#include "BSP/MPU6050/mpu6050_empl_port.h"

#include "BSP/i2c.hpp"
#include "BSP/system.hpp"

extern "C" int MPU6050_eMPL_i2cWrite(unsigned char slaveAddress,
                                     unsigned char registerAddress,
                                     unsigned char length,
                                     const unsigned char *data) {
  return bsp::i2cWriteRegister(0U, slaveAddress, registerAddress, data, length,
                               20U) == car::Status::Ok
             ? 0
             : -1;
}

extern "C" int MPU6050_eMPL_i2cRead(unsigned char slaveAddress,
                                    unsigned char registerAddress,
                                    unsigned char length, unsigned char *data) {
  return bsp::i2cReadRegister(0U, slaveAddress, registerAddress, data, length,
                              20U) == car::Status::Ok
             ? 0
             : -1;
}

extern "C" void MPU6050_eMPL_delayMs(unsigned long milliseconds) {
  const std::uint32_t start = bsp::millis();
  while (static_cast<std::uint32_t>(bsp::millis() - start) < milliseconds) {
  }
}

extern "C" void MPU6050_eMPL_getMs(unsigned long *milliseconds) {
  if (milliseconds != nullptr)
    *milliseconds = static_cast<unsigned long>(bsp::millis());
}

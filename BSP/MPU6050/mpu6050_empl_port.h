#ifndef TI_CAR_BASE_MPU6050_EML_PORT_H_
#define TI_CAR_BASE_MPU6050_EML_PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

int MPU6050_eMPL_i2cWrite(unsigned char slaveAddress,
                          unsigned char registerAddress, unsigned char length,
                          const unsigned char *data);
int MPU6050_eMPL_i2cRead(unsigned char slaveAddress,
                         unsigned char registerAddress, unsigned char length,
                         unsigned char *data);
void MPU6050_eMPL_delayMs(unsigned long milliseconds);
void MPU6050_eMPL_getMs(unsigned long *milliseconds);

#define MPU6050_EMPL_LOG_INFO(...)                                             \
  do {                                                                         \
  } while (0)
#define MPU6050_EMPL_LOG_ERROR(...)                                            \
  do {                                                                         \
  } while (0)

#ifdef __cplusplus
}
#endif

#endif

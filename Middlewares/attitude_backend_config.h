#ifndef ATTITUDE_BACKEND_CONFIG_H
#define ATTITUDE_BACKEND_CONFIG_H

#include "Config/build_config.h"

// 姿态后端选择。软件滤波路径 (COMPLEMENTARY/KALMAN/MAHONY/BMI270) 只依赖
// drivers::ImuBackend 接口 (见 Drivers/imu_backend.hpp)，经 ImuReader 驱动
// AttitudeFilter；DMP 路径直接使用 drivers::Mpu6050Dmp，产出 Euler。
// 切换 IMU 芯片只需为新驱动实现 ImuBackend，不改 Application 滤波路径。
// BMI270 后端用 drivers::Bmi270；定义 BMI270_ONBOARD_FUSION=1 可让驱动内置
// Mahony 直接产出 Euler，绕过 AttitudeFilter (见 Drivers/bmi270.hpp)。

#if ATTITUDE_CONFIG_BACKEND == ATTITUDE_BACKEND_DMP
#define ATTITUDE_FILTER_IMPL_DMP
#define EMPL_TARGET_MSPM0
#define EMPL_SENSOR_MPU6050
#endif

#endif

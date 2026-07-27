#pragma once
#include "Common/types.hpp"
namespace drivers {

// 通用 IMU 后端接口。软件姿态滤波路径只依赖本接口，不直接依赖具体芯片驱动。
// begin/poll/ready 是所有 IMU 后端共有的最小面；芯片专有的初始化标定
// (如 Mpu6050::calibrateGyroBias) 和数据就绪锁存 (如
// Mpu6050Dmp::notifyDataReady) 留在具体类，由 Application 经编译期
// ATTITUDE_CONFIG_BACKEND 分支调用。
class ImuBackend {
public:
  virtual ~ImuBackend() noexcept = default;
  virtual car::Status begin() noexcept = 0;
  virtual car::Status poll(car::ImuSample &sample) noexcept = 0;
  virtual bool ready() const noexcept = 0;
};

} // namespace drivers

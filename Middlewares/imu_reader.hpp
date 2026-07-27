#pragma once
#include "Common/types.hpp"
#include "Drivers/imu_backend.hpp"
#include "Middlewares/attitude_filter.hpp"
namespace middleware {

// 把“poll -> 计算 dt -> AttitudeFilter.update”的运行时采样路径封装起来，
// 让 Application 的软件滤波分支只依赖 ImuBackend 接口，不命名具体芯片。
// 不在本类内做 ready()/data-ready 门控：那仍由 Application 用 imuReady_ 与
// bsp::consumeImuDataReady() 控制，行为与原内联实现一致。
class ImuReader final {
public:
  // 清空 dt 基准，使下一次 step() 用首样本 10 ms fallback。
  void reset() noexcept;
  // 读取一帧并驱动姿态滤波；返回最终状态（poll 非 Ok 时直接返回，跳过
  // update）。
  car::Status step(drivers::ImuBackend &imu, middleware::AttitudeFilter &filter,
                   std::uint32_t nowMs, car::ImuSample &out) noexcept;

private:
  std::uint32_t lastMs_ = 0U;
};

} // namespace middleware

# 控制与姿态 Middlewares

所有 Middleware 仅依赖 `Common/types.hpp` 与标准 C++，可脱离开发板测试。

| 模块 | 输入 | 输出/作用 |
| --- | --- | --- |
| `Pid` | target、measured、dt | 积分限幅、首样本无微分冲击的受限输出。 |
| `DifferentialDrive` | `VehicleCommand` | 左右 `WheelCommand`。 |
| `LineFollower` | `LineSample` | 巡线命令建议，不直接控制电机。 |
| `EncoderSpeedEstimator` | ticks、timestamp | 左右轮 m/s。 |
| `AttitudeFilter` | 原始 `ImuSample`、dt | 互补或 Kalman roll/pitch。 |
| `SafetyGate` | 命令、enabled | 默认输出零。 |
| `Telemetry` | 状态 DTO | 固定缓冲格式化。 |

## 姿态选择

在 `Middleware/Attitude/attitude_backend_config.h` 设置：

```c
#define ATTITUDE_CONFIG_BACKEND ATTITUDE_BACKEND_DMP
// ATTITUDE_BACKEND_COMPLEMENTARY
// ATTITUDE_BACKEND_KALMAN
```

DMP 使用 MPU 内部姿态；软件后端使用原始驱动与 `AttitudeFilter`。三种后端都是 6-axis，yaw 均为相对值。

`EncoderSpeedConfig` 初值为 Wheeltec C07A 的 65 mm、28:1、13 CPR、2x decode。确认本车方向、轮径、倍率和 PID 前，`SafetyGate` 不得允许闭环写入电机。

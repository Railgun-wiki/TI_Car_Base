# 控制与姿态 Middlewares

所有 Middleware 仅依赖 `Common/types.hpp` 与标准 C++，可脱离开发板测试。

| 模块 | 输入 | 输出/作用 |
| --- | --- | --- |
| `Pid` | target、measured、dt | 积分限幅、首样本无微分冲击的受限输出。 |
| `DifferentialDrive` | `VehicleCommand` | 左右 `WheelCommand`。 |
| `LineFollower` | `LineSample`、5 ms dt | 可运行时配置的 PID 巡线命令，不直接控制电机。 |
| `EncoderSpeedEstimator` | ticks、timestamp | 左右轮 m/s。 |
| `AttitudeFilter` | 原始 `ImuSample`、dt | 互补或 Kalman roll/pitch。 |
| `SafetyGate` | 命令、enabled | 默认输出零。 |
| `Telemetry` | 状态 DTO | 固定缓冲格式化。 |
| `VofaProtocol` | UART RX byte stream | 组帧并解析 VOFA+ 文本调参命令。 |

## 姿态选择

在 `Middlewares/attitude_backend_config.h` 设置：

```c
#define ATTITUDE_CONFIG_BACKEND ATTITUDE_BACKEND_DMP
// ATTITUDE_BACKEND_COMPLEMENTARY
// ATTITUDE_BACKEND_KALMAN
```

DMP 使用 MPU 内部姿态；软件后端使用原始驱动与 `AttitudeFilter`。三种后端都是 6-axis，yaw 均为相对值。

`EncoderSpeedConfig` 初值为 Wheeltec C07A 的 65 mm、28:1、13 CPR、2x decode。确认本车方向、轮径、倍率和 PID 前，`SafetyGate` 不得允许闭环写入电机。

## VOFA+ 调参

`LineFollower` 默认使用 `kp=45, ki=0, kd=0, cruise=180`。串口命令只在循线
未 enable 时生效，成功后 PID 状态会 reset，避免旧积分或微分状态污染新参数。

| 命令 | 范围 | 响应 |
| --- | --- | --- |
| `SET,LINE,kp,ki,kd` | kp 0..300，ki 0..30，kd 0..100 | `ack:LINE` |
| `SET,CRUISE,value` | 0..500 | `ack:CRUISE` |
| `GET,CONFIG` | 无参数 | 当前参数命名帧 |

命令使用 ASCII，末尾必须是 `\n` 或 `\r\n`。数值支持普通十进制，不支持科学
计数法。运行中修改返回 `err:RUNNING`，越界返回 `err:RANGE`。

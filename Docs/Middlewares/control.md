# 控制与姿态 Middlewares

所有 Middleware 仅依赖 `Common/types.hpp` 与标准 C++，可脱离开发板测试。

| 模块 | 输入 | 输出/作用 |
| --- | --- | --- |
| `Pid` | target、measured、dt | 积分限幅、首样本无微分冲击的受限输出。 |
| `DifferentialDrive` | `VehicleCommand` | 左右 `WheelCommand`。 |
| `LineFollower` | `LineSample`、5 ms dt | 可运行时配置的 PID 巡线命令，不直接控制电机。 |
| `EncoderSpeedEstimator` | ticks、timestamp | 左右轮 m/s。 |
| `AttitudeFilter` | 原始 `ImuSample`、dt | 互补 / Kalman / Mahony AHRS roll/pitch/yaw。 |
| `SafetyGate` | 命令、enabled | 默认输出零。 |
| `Telemetry` | 状态 DTO | 固定缓冲格式化。 |
| `VofaProtocol` | UART RX byte stream | 组帧并解析 VOFA+ 文本调参命令。 |

## 姿态选择

在 `Middlewares/attitude_backend_config.h` 设置：

```c
#define ATTITUDE_CONFIG_BACKEND ATTITUDE_BACKEND_COMPLEMENTARY
// ATTITUDE_BACKEND_DMP
// ATTITUDE_BACKEND_KALMAN
// ATTITUDE_BACKEND_MAHONY
```

DMP 使用 MPU 内部姿态；软件后端使用原始驱动与 `AttitudeFilter`。四个后端都是 6-axis，yaw 均为相对值。
Mahony 默认参数 Kp=0.17 / Ki=0.004 取自 SJTU-AuTop `attitude_solution.c`，通过 `rawImu_.calibrateGyroBias()` 做启动时静态零偏标定（约 1 s，需静止），运行时再由 Ki 持续收敛；无温度补偿。

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
# H 题赛道状态机

`HQuestionRace` 将 H 题的四条路径表示为纯 Middleware 状态机，不直接控制电机或
读取 GPIO。它输出当前段类型、目标距离、相对 yaw 目标、圈数与状态；
`HQuestionApplication` 决定使用巡线或 heading PID，并通过 `SafetyGate` 发布命令。

状态为菜单、倒计时、运行、顶点暂停、完成和故障。运行段在目标距离 60% 后启用灰度
辅助顶点判断，达到目标距离必然结束该段。所有物理量在 `RaceConfig` 统一管理，默认
参数仅用于构建和初始调试，必须实车标定。

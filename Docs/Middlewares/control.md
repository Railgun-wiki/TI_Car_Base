# 控制与姿态 Middlewares

所有 Middleware 仅依赖 `Common/types.hpp` 与标准 C++，可脱离开发板测试；`ImuReader` 额外依赖 `Drivers/imu_backend.hpp` 接口（不依赖具体芯片）。

| 模块 | 输入 | 输出/作用 |
| --- | --- | --- |
| `Pid` | target、measured、dt | 积分限幅、首样本无微分冲击的受限输出。 |
| `DifferentialDrive` | `VehicleCommand` | 左右 `WheelCommand`。 |
| `LineFollower` | `LineSample`、5 ms dt | 可运行时配置的 PID 巡线命令，不直接控制电机。 |
| `EncoderSpeedEstimator` | ticks、timestamp | 左右轮 m/s。 |
| `WheelSpeedController` | 左右目标 mm/s、ticks、timestamp | 两路受限速度 PID，输出 `[-1000,1000]` PWM 命令。 |
| `AttitudeFilter` | 原始 `ImuSample`、dt | 互补 / Kalman / Mahony AHRS roll/pitch/yaw。 |
| `ImuReader` | `ImuBackend&`、`AttitudeFilter&`、`nowMs` | 封装 poll → dt → filter，让软件滤波路径不命名具体芯片。 |
| `SafetyGate` | 命令、enabled | 默认输出零。 |
| `Telemetry` | 状态 DTO | 固定缓冲格式化。 |
| `VofaProtocol` | UART RX byte stream | 组帧并解析 VOFA+ 文本调参命令。 |

`ImuReader::step()` 依次：调 `imu.poll(out)` → 算 `dtMs`（`lastMs_==0` 时首样本 fallback 10 ms，否则 `nowMs - lastMs_`）→ 更新 `lastMs_` 与 `out.timestampMs` → `poll==Ok` 时 `filter.update(out, dt_s)` 并返回其状态。`reset()` 清 `lastMs_=0`。本类不做 `ready()`/data-ready 门控，那仍由 Application 用 `imuReady_` 与 `bsp::consumeImuDataReady()` 控制，行为与原内联实现一致。状态：已构建，待实机验证。

## 姿态选择

在 `Middlewares/attitude_backend_config.h` 设置：

```c
#define ATTITUDE_CONFIG_BACKEND ATTITUDE_BACKEND_COMPLEMENTARY
// ATTITUDE_BACKEND_DMP
// ATTITUDE_BACKEND_KALMAN
// ATTITUDE_BACKEND_MAHONY
```

DMP 使用 MPU 内部姿态；软件后端使用原始驱动与 `AttitudeFilter`。四个后端都是 6-axis，yaw 均为相对值。
Mahony 默认参数 Kp=0.17 / Ki=0.004 取自 SJTU-AuTop `attitude_solution.c`，通过 `rawImu_.calibrateGyroBias()` 做启动时静态零偏标定（约 1 s，需静止），运行时再由 Ki 持续收敛；无温度补偿。重力反馈前对加速度使用同参考工程的 `alpha=0.3` 一阶低通，以降低车体振动导致的 roll/pitch 误修正；陀螺积分仍使用每帧实测 `dt`，不照搬参考工程固定 1 ms 周期。

`AttitudeFilter::update()` 拒绝 `dt<=0` 或 `dt>0.1 s` 的样本。Mahony 路径在加速度范数近零（自由落体或丢帧）时禁用加速度叉乘修正、仅积分陀螺，并将重力参考置为单位向量，避免 `fastInvSqrt(0)` 产生 NaN 污染四元数；`fastInvSqrt` 的浮点↔整数重解释改用 `memcpy` 以避免严格别名 UB。

H 题速度闭环当前使用 48 mm 轮径、1456 counts/轮、50 ms 更新周期，以及 `Kp=2, Ki=1,
Kd=0`、250 PWM 输出限幅；其中 1456 来自 28:1 减速比、13 线编码器的四倍频正交解码。确认
本车方向、轮径、倍率和 PID 前，
`SafetyGate` 不得允许闭环写入电机。

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
# H 题应用边界

P1–P4 路径、倒计时和端点状态机属于 `Application/h_question_program.*`，而非
Middleware。Middleware 仅提供可复用的 PID、循线和速度闭环；Application 决定使用
巡线或 heading PID，并通过 `SafetyGate` 发布命令。所有 H 题调试参数集中于
`Config/vehicle_tuning.hpp`，默认值仅用于构建和初始调试，必须实车标定。

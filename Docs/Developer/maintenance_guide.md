# 固件维护手册

## 1. 范围与当前状态

本手册描述当前 `TI_Car_Base` 的代码维护、构建、标定和上板验收规则。它不替代
`TI_Car_Base.syscfg`：时钟、引脚、PWM、I2C、UART 和 GPIO 的唯一配置来源始终是
SysConfig。

当前默认构建已通过 CCS 编译；下列项目仍必须实机确认：电机方向、灰度极性、编码器
方向/倍率、MPU INT 极性与安装轴向、OLED 控制器与 I2C 上拉、所有闭环参数。

## 2. 目录与依赖规则

| 目录 | 内容 | 可以依赖 | 不可以依赖 |
| --- | --- | --- | --- |
| `BSP/` | SysConfig/DriverLib 的板级实现、ISR、时基、I2C/UART/PWM/GPIO | `Common/`、DriverLib、SysConfig 生成头 | Driver、Middleware、Application |
| `Drivers/` | 电机、传感器、显示、按键等设备语义 | `BSP/`、`Common/` | Application、算法实现、SysConfig 宏 |
| `Middlewares/` | PID、速度估算、运动学、巡线、姿态、telemetry、安全门 | `Common/`、标准 C++ | BSP、DriverLib、GPIO |
| `Application/` | 初始化、周期调度、台架 Demo、设备装配 | 所有上层公开接口 | 寄存器和 SysConfig 宏 |
| `ThirdParty/eMPL/` | InvenSense MPU6050 DMP vendor 源码 | 仅经 target port 使用 | 业务与板级逻辑 |

`TI_Car_Base.cpp` 是唯一 `main()`。对象静态组合；禁止 `new/delete`、异常、
RTTI 与热路径虚函数。

## 3. 硬件基线

| 功能 | 现有配置 | 维护约束 |
| --- | --- | --- |
| 系统时钟 | HFXT 40 MHz、LFXT 32.768 kHz、MCLK 80 MHz | 修改时必须复核 flash wait state、I2C/UART 时序。 |
| TB6612 | PA28/PB20 PWM 10 kHz；方向 PA13/PB26/PB9/PB7 | 改变方向前先归零 PWM；上电必须停止。 |
| 灰度 | C1..C8：PA31/PA12/PB8/PA27/PB0/PA30/PB21/PB10 | 在 `LineSensorArray` 统一极性。 |
| 编码器 | 左 PB23/PB12；右 PB4/PB5；双边沿 GPIO ISR | ISR 仅更新 ticks，不计算速度/PID。 |
| MPU6050 | I2C0 PA0/PA1 400 kHz；INT PB11 | ISR 仅置 data-ready；I2C/FIFO 仅在主循环。 |
| OLED | I2C1 PB2/PB3 400 kHz | 无设备或超时必须降级，不得阻塞控制。 |
| UART0 | PA10/PA11，115200 8N1 | telemetry 允许丢弃，不得阻塞。 |
| 五向键 | PA14/PA15/PA17/PB25/PB24，active-low | Driver 负责去抖语义。 |
| LED/蜂鸣器 | LED1/2/3 active-low；PB22 active-high；PA21 蜂鸣器 active-low | PA21 为有源蜂鸣器，禁止配置 PWM。 |

W25Q128 的 PB6..PB9、板载 PB21 key 和 PA18 BSL 不纳入本固件，原因是与已确认的小车
引脚或启动维护功能冲突。

## 4. 公开数据与设备接口

跨层只使用 `Common/types.hpp` 的值对象：

| 类型 | 单位/语义 |
| --- | --- |
| `WheelCommand` | 左/右轮归一化指令，范围 -1000..1000。 |
| `VehicleCommand` | 线速度建议与转向建议；由运动学转为 `WheelCommand`。 |
| `LineSample` | 八位逻辑灰度、加权误差、是否检测到线。 |
| `EncoderTicks` | ISR 维护的有符号软件正交计数。 |
| `ImuSample` | accel 为 g、gyro 为 deg/s、姿态为 degree、timestamp 为 ms。 |
| `Status` | 非异常错误路径；调用者必须处理非 `Ok`。 |

设备调用使用 `begin()`、`poll()`、`read()`、`set()`、`stop()` 等
`noexcept` 接口。只有 `Application` 能将命令送至 `MotorDriver`。

## 5. 姿态后端

在 `Middleware/Attitude/attitude_backend_config.h` 选择一个后端：

```c
#define ATTITUDE_CONFIG_BACKEND ATTITUDE_BACKEND_DMP
// ATTITUDE_BACKEND_COMPLEMENTARY
// ATTITUDE_BACKEND_KALMAN
```

| 后端 | 数据路径 | 适用场景 | 限制 |
| --- | --- | --- | --- |
| DMP（默认） | MPU FIFO -> eMPL -> quaternion/Euler | 正常实车运行 | 6-axis yaw 仍为相对值。 |
| Complementary | 原始 accel + gyro | 快速验证、最小资源 | yaw 积分漂移。 |
| Kalman | 每轴 angle+bias 二状态 | 对比调参与噪声测试 | 仅 roll/pitch 融合，yaw 积分漂移。 |

`ThirdParty/eMPL` 不得修改 firmware 数组、FIFO parser 或寄存器算法。允许的
portability patch 仅限：添加 `EMPL_TARGET_MSPM0` 分支、Target 编译包围和
`BSP/MPU6050/mpu6050_empl_port.*` 的外部映射。

## 6. 调度与安全状态

- SysTick 提供 1 ms 单调时间。
- 编码器 ISR 只更新 ticks；MPU PB11 ISR 只置 data-ready。
- 主循环以 200 Hz 读取灰度/按键、按 IMU data-ready 读取姿态、20 Hz 输出 telemetry；
  OLED 为低优先级。
- 台架 Demo：CENTER 连续按住 500 ms 后，UP/ DOWN 以低占空比正/反转；松手停车。
- 初始化失败、DMP/I2C/FIFO 错误均调用 `MotorDriver::stop()`。软件滤波或巡线算法
  默认只计算；`SafetyGate` 不允许自动闭环直接驱动电机。

## 7. 标定清单

以下 Wheeltec 初值只用于起步，位于 `EncoderSpeedConfig`：

| 参数 | 初值 | 验证方法 |
| --- | ---: | --- |
| 轮径 | 0.065 m | 多圈滚动距离 / 圈数。 |
| 减速比 | 28:1 | 轴与轮的转数比。 |
| 编码器 CPR | 13 | 查电机规格并以示波器/计数验证。 |
| 解码倍率 | 2 | 与 GPIO 边沿解码实测 ticks 对齐。 |

完成上述项目后，依次确认左右正方向、速度单位、PID 输出极性和 PWM 最小起转占空比。
未确认前禁止解除 `SafetyGate` 的自动控制限制。

## 8. 修改、格式化与构建

1. 外设改动只能使用 CCS SysConfig；禁止手改 `.syscfg`、`.cproject` 或生成物。
2. C/C++ 修改后仅格式化本次文件：

```sh
/Library/Developer/CommandLineTools/usr/bin/clang-format -i <changed-files>
```

3. 使用 CCS 的 `buildProject` 构建，不能直接调用 `make/gmake`。
4. 改变姿态后端后，至少构建选中的后端；提交前默认 DMP 必须能构建。
5. 任何通信或控制改动都应补充 host-side 测试；当前工程尚未建立 `Tests/` target，
   因而不能把 CCS 构建等同于算法验收。

## 9. 上板验收顺序

1. 断开电机或抬空车轮，上电确认 PWM 为零、电机无动作。
2. 验证 LED、蜂鸣器、UART0 状态输出和 CENTER 长按保护。
3. 分别验证左右轮低占空比正反转；记录实际方向。
4. 验证 8 路灰度的“线/底”逻辑与加权误差方向。
5. 推动轮子并检查编码器 ticks 正负与每圈计数。
6. 检查 MPU `WHO_AM_I`、PB11 中断、姿态轴向和 DMP FIFO overflow 恢复。
7. 断开 OLED，确认控制和 telemetry 继续运行。
8. 仅在完成标定后，单独启用速度 PI，再单独验证巡线输出。

每一步都应记录日期、固件 commit、接线/供电状态、参数和结果；失败时记录 UART 帧及
复现步骤。编译成功不等于硬件通过。

## 10. Git 与变更记录

- 使用 Conventional Commits；按可独立构建的功能边界拆分提交。
- 不提交 `Debug/`、`Release/`、SysConfig 生成物或 IDE 缓存。
- 修改硬件映射时，提交必须同时包含 SysConfig（经 CCS 保存）、BSP/Driver 与文档。
- 未经明确要求不得 push。

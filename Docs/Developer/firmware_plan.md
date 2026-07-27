# 小车固件方案（第一阶段）

## 目标与范围

本工程面向 MSPM0G3507 差速小车。第一阶段目标是在不引入 RTOS 的前提下，完成安全电机驱动、八路灰度巡线、MPU6050/ICM I2C 采样、OLED 硬件 I2C 显示、UART telemetry，以及为编码器闭环预留清晰边界。

已配置的硬件基线为：外部 HFXT 40 MHz + LFXT 32.768 kHz、以 HFXT 为 SYSPLL 参考的 80 MHz MCLK、TB6612 PWM 10 kHz、MPU6050 `I2C0/PA0/PA1` 400 kHz、OLED `I2C1/PB2/PB3` 400 kHz、四个 UART 115200。具体引脚和参数以 `TI_Car_Base.syscfg` 为准。

## 架构决策

采用与 `MPU6050-F4` 参考工程相同的静态组合思路，但使用 MSPM0 SysConfig 与 DriverLib：

```text
Application -> Middlewares -> Drivers -> BSP -> SysConfig / DriverLib
```

| 层 | 职责 | 禁止事项 |
| --- | --- | --- |
| `BSP` | 适配固定的 I2C0、UART0、TIMA1、GPIO、timer/interrupt；只在此层包含 DriverLib。 | 业务状态机、姿态或巡线算法。 |
| `Drivers` | 提供 `MotorDriver`、`Mpu6050`、`LineSensorArray`、未来的 `Encoder` 等设备语义。 | 让上层读取寄存器或依赖具体 GPIO 端口。 |
| `Middlewares` | PID、差速运动学、巡线策略、姿态滤波、安全策略、telemetry 格式化。 | 调用 DriverLib、SysConfig 宏、RTOS API。 |
| `Application` | 初始化顺序、运行状态机和周期编排。 | 绕过 Driver 直接操作硬件。 |

### 静态 OOP

使用 C++ 的值对象和静态组合，而不是动态框架：

- 对象在静态存储期或栈上创建；禁止 `new/delete`、异常、RTTI 和热路径虚函数。
- constructor 不做 I/O；由明确的 `init()` / `configure()` 执行硬件初始化。
- 跨层数据使用单位明确的 DTO，例如 `LineSample`、`ImuSample`、`VehicleCommand` 和 `WheelSpeed`。
- 对外部 C ISR 使用薄 `extern "C"` 转发入口；ISR 不持有业务逻辑。

这一方式保留了 OOP 的 ownership 与可测试边界，同时避免堆、析构顺序和动态派发带来的不可预测性。

### DMP Target 与标定初值

`ThirdParty/eMPL/` 为 InvenSense Motion Driver 的 vendor boundary。只允许在
其平台选择区增加 `EMPL_TARGET_MSPM0`，映射 `i2c_read/write`、
`delay_ms`、`get_ms` 和 no-op log 到 `BSP/MPU6050`；不得修改 DMP
firmware、FIFO parser、寄存器流程或姿态算法。DMP 使用 100 Hz、6-axis
quaternion 和 calibrated gyro；yaw 是相对航向，不能视为绝对航向。

`Middlewares/attitude_backend_config.h` 的
`ATTITUDE_CONFIG_BACKEND` 默认使用 `ATTITUDE_BACKEND_COMPLEMENTARY`，也可选
`ATTITUDE_BACKEND_DMP`、`ATTITUDE_BACKEND_KALMAN` 与 `ATTITUDE_BACKEND_MAHONY`。
三者都输出相同的 `ImuSample`；软件滤波使用 MPU6050 原始采样，不编译
eMPL 主体。互补滤波适合轻量、快速验证，Kalman 为每轴 angle+bias 二状态，
Mahony 为四元数 + PI 修正陀螺零偏（参考 SJTU-AuTop）；三者都只有 6-axis 相对 yaw，
Mahony 的 yaw 稳定性优于纯积分。

`EncoderSpeedEstimator` 的初值来自 Wheeltec C07A：65 mm 轮径、28:1
减速比、13 CPR、2x 软件解码。该配置仅作起点，必须以本车实测值替换。闭环
算法可以计算和上报，但 `SafetyGate` 默认禁止它写入电机，直至完成方向、
极性、轮径、减速比、编码器倍率和 PID 参数标定。

### IMU 后端抽象

软件姿态滤波路径只依赖 `Drivers/imu_backend.hpp` 的 `drivers::ImuBackend`
抽象接口（`begin/poll/ready`），不直接依赖 `Mpu6050` 具体类。采样路径
（poll → dt → `AttitudeFilter.update`）封装在 `Middlewares/imu_reader.{hpp,cpp}`
的 `ImuReader::step()`，它取 `ImuBackend&` 参数。为其它 6 轴 IMU（如 ICM 系列）
接入软件滤波只需新增一个 `ImuBackend` 实现，不必改动 Application 滤波路径。

芯片专有方法不进接口：`Mpu6050::calibrateGyroBias`（启动期静态零偏标定）和
`Mpu6050Dmp::notifyDataReady`（eMPL FIFO due 锁存）留在具体类，由 Application
经编译期 `ATTITUDE_CONFIG_BACKEND` 分支调用。`Mpu6050`、`Mpu6050Dmp` 均实现
`ImuBackend`。DMP 仍是编译期可选后端，与软件路径不做运行时多态；DMP 路径不经
`ImuReader`。因软件路径的 IMU 是值成员，应用头文件在非 DMP 构建下仍需 include
`mpu6050.hpp`（C++ 值语义要求完整类型），但运行时滤波逻辑已与芯片解耦。

`Bmi270`（`ATTITUDE_BACKEND_BMI270`）是此抽象的第二个具体实现，验证了"换 IMU
只加驱动"的目标：I2C0 上替换 MPU6050，驱动实现 `ImuBackend` + 自有
`calibrateGyroBias`，Application 仅加 `#elif` 分支，`ImuReader`/`AttitudeFilter`
零改动。BMI270 特有：初始化必须 burst-write 8192 字节 vendor config blob
（`ThirdParty/bmi270/`），且 config 加载事务的 I2C timeout 需放宽至 ~500 ms。
`BMI270_ONBOARD_FUSION` 宏让 `poll()` 改为驱动内置 Mahony 直出 Euler、绕过
`AttitudeFilter`——这是芯片专有的旁路，不破坏接口契约。

## 当前代码边界与循线 Demo

源码按 `BSP/`、`Drivers/`、`Middlewares/`、`Application/` 组织。`BSP/` 与 `Drivers/` 并列：前者是板级实现，独占 DriverLib/SysConfig，并按 `system`、`i2c`、`uart`、`motor`、`input`、`indicator`、`encoder` 拆分；后者是设备语义 Driver，例如 `motor_driver.*`、`mpu6050.*`。每个设备或功能独占同名 `.hpp + .cpp`。`Middlewares` 只处理 `VehicleCommand`、`WheelCommand`、`LineSample`、`EncoderTicks`、`ImuSample` 等值对象。

`TI_Car_Base.cpp` 是唯一入口：用 SysTick 产生 1 ms tick，并运行 cooperative
`CarApplication::step()`。巡线 PID 以固定 5 ms 周期产生差速命令，只有
CENTER 长按解锁且持续按住 UP 时，`SafetyGate` 才允许命令进入电机。

UART0 每 50 ms 输出 VOFA+ FireWater 命名帧，并在主循环处理停车状态下的循线
PID/cruise 调参；RX/TX ISR 只搬运固定 ring buffer。初始化或 IMU 通信失败会触发
停车，显示通信失败则降级运行。

## 参考工程的取舍

### MPU6050-F4

作为主要结构参考：其 `Application` 静态组合 BSP、Driver 与 Middleware；Driver 不依赖 HAL；ISR 仅通知，I2C 和姿态处理在任务/主循环上下文执行。MSPM0 工程应复用这些接口边界、状态机和 host-side 测试理念，而不能复用 STM32 HAL handle、TIM2、EXTI callback 或 FreeRTOS 资源统计实现。

### Wheeltec 小车与 MPU6050 示例

借鉴 TB6612 安全驱动、四路循迹、编码器速度 PI 和 MPU6050 寄存器操作的功能定义。不得将其把巡线、控制、I2C 和日志集中在 ISR 的实现方式直接带入本工程。

### 逐飞 MSPM0G3507 库

作为 DriverLib 用法、80 MHz 时钟和外设能力的参考，不直接引入。其通用库通过枚举、全局外设表和直接寄存器访问适配多种资源组合，与本工程的 SysConfig 单一配置来源不匹配；并且其源文件声明 GPL-3.0，直接复用会引入相应许可证义务。

本工程所谓“静态配置”是只为本车固化实际资源：I2C0/I2C1、四个 UART、两路 PWM、既定方向 GPIO 和传感器 GPIO，而不是保留通用 pin enum 或全局 callback table。

## 无 RTOS 调度

第一阶段采用 `timer/interrupt + cooperative superloop`。这比过早引入 RTOS 更易测量时序、占用更少 RAM，也能保持与未来 RTOS 的迁移边界。

```text
timer ISR (1 kHz)  -> 累加/锁存编码器数据，置 control_due
MPU PB11 ISR         -> 清中断标志并置 imu_due；I2C 读取在主循环
UART ISR           -> 仅搬运 RX/TX ring buffer

main superloop
  -> Application::process()       // 短操作、状态机、UART 命令
  -> Application::controlStep()   // control_due: 1 kHz
  -> Application::lineStep()      // line_due: 200–500 Hz
  -> Application::imuStep()       // imu_due: data-ready 驱动
  -> Application::telemetryStep() // 10–50 Hz，低优先级
  -> Application::oledStep()      // 低优先级、分段发送，不能影响控制周期
```

ISR 只能做有界且非阻塞的工作：置 flag、读取/锁存计数或写入 ring buffer。禁止在 ISR 执行 I2C、`printf`、姿态解算、完整 PID 或动态分配。

所有主循环工作必须可快速返回。两条硬件 I2C 的传输都必须有超时与失败路径；OLED `I2C1` 刷新只能在低优先级 `oledStep()` 中分段运行，禁止进入 ISR 或 1 kHz 控制路径。UART telemetry 必须允许丢弃低优先级帧，不能阻塞电机控制。

## 控制与安全边界

- 电机方向变更：先将 PWM 设为安全值，经过最短死区，再更新 AIN/BIN，最后恢复 PWM。
- `MotorDriver::stop()` 必须在启动失败、通信超时、传感器故障和显式停车时可重复调用。
- 巡线驱动层统一原始 GPIO 极性；策略层只消费逻辑化的 `LineSample`。
- MPU data-ready 接至 `PB11`，配置为上升沿 GPIO interrupt；ISR 仅置位读取通知，不能在 ISR 中执行 I2C。实机须确认 data-ready 极性。
- W25Q128 Flash 的 `PB7/PB8/PB9` 分别与 TB6612/灰度引脚冲突，板载 `PB21` 用户按键与 `C7` 冲突；两者当前不启用，以保留完整小车外设。`PA18` BSL 按键按用户决定保持禁用，避免把启动维护信号混入应用输入。TM2027 五向按键使用独立的 `PA14/PA15/PA17/PB25/PB24`，保持 `GPIO_KEY` active-low polling 配置。
- 三路 LED 阳极上拉至 3.3 V，`GPIO_LED` 为 active-low output；上电 high（熄灭），只能通过 Driver 提供的语义化接口控制。
- 有源蜂鸣器由 `PA21` 经 S8550 PNP 高边驱动，`GPIO_BUZZER.BUZZER` 为 active-low GPIO：上电 high（关闭）、low（开启）。它不是无源蜂鸣器，不能使用 PWM 音调接口。
- 编码器已配置为四个双边沿 GPIO interrupt，由 Driver 进行软件正交解码；速度闭环与里程计仍须先按实测轮径、减速比和最高脉冲率校验方向、溢出与控制参数。

## 迁入 RTOS 的条件

仅当以下任一情况经过测量确认时，再引入 FreeRTOS：

- cooperative superloop 无法在控制周期预算内完成；
- UART/无线、记录、显示等低优先级工作造成明显的控制抖动；
- 存在多个独立的、可能阻塞的通信工作流，且无法通过状态机拆分。

迁入时保持 `BSP`、`Drivers`、`Middlewares` API 不变，仅将 `Application` 的 `step()` 调度替换为 task notification、queue 或 event。建议的优先级顺序是 MotorControl > IMU > LineFollow > Command > Telemetry。

## 验证路线

1. 构建并确认 SysConfig 无 errors/warnings。
2. 实机验证 `MotorDriver::stop()`、单轮正反转与 PWM 占空比范围。
3. 验证八路巡线逻辑极性与采样频率。
4. 验证 MPU6050 WHO_AM_I、地址、data-ready 极性与 I2C 超时恢复。
5. 验证 UART 帧完整性和 telemetry 不影响 1 kHz 控制。
6. 验证 OLED 的外部 3.3 V 上拉、起停序列和刷新分段不会影响 1 kHz 控制。
7. 验证编码器软件解码后，再引入速度 PI；记录轮径、减速比、编码器分辨率和控制参数单位。

“构建成功”只证明源码和配置一致，不构成电机、传感器或接线已经验收的证据。

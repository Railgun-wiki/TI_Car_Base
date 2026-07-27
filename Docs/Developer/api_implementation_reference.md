# TI 车固件 API 与实现参考

本文档面向需要阅读、扩展或调试 `TI_Car_Base` 的开发者。它说明**当前源码已经实现的接口和行为**，而非未来接口设计；外设实例、引脚和时钟参数仍以 `TI_Car_Base.syscfg` 为唯一来源。

## 1. 分层、对象所有权与调用链

工程以静态组合方式运行，没有堆分配、RTOS 任务、异常或运行时多态：

```text
main()
  └─ bsp::init()
  └─ app::CarApplication::init() / step()
       ├─ Drivers: 将设备语义映射为 BSP 操作
       ├─ Middlewares: 纯算法、格式化与安全裁决
       └─ BSP: DriverLib / SysConfig / ISR / 时基
            └─ MSPM0 外设
```

`CarApplication` 独占 `MotorDriver`，因此只有 Application 能把最终的轮端命令
写到 PWM。`LineFollower`、`Pid` 与 `DifferentialDrive` 已接入循线 Demo；
`EncoderSpeedEstimator` 仍只计算数据，编码器速度闭环尚未接入。

数据方向如下：

```text
GPIO gray -> LineSensorArray -> LineSample -> LineFollower -> VehicleCommand
encoder ISR -> Encoder -> EncoderTicks -> EncoderSpeedEstimator -> WheelSpeed
MPU I2C/FIFO -> Mpu6050/Mpu6050Dmp/Bmi270 (all implement ImuBackend) -> ImuSample -> ImuReader+AttitudeFilter (software backend); DMP or Bmi270+FUSION -> ImuSample with Euler directly
line-follow wheel proposal -> SafetyGate -> MotorDriver -> BSP PWM + TB6612
```

所有公开接口均为 `noexcept`。失败不通过异常上抛，而是以 `car::Status` 返回；调用方必须显式检查。

## 2. 公共数据模型（`Common/types.hpp`）

| 类型 | 字段 | 单位与约定 |
| --- | --- | --- |
| `WheelCommand` | `left`, `right` | 归一化轮端命令，合法范围 `-1000..1000`；正负对应方向，具体正向须实机确认。 |
| `VehicleCommand` | `linear`, `angular` | 车辆级建议量；当前 `DifferentialDrive` 以归一化整数混合，不是 SI 速度/角速度。 |
| `LineSample` | `bits`, `error`, `detected` | 8 个逻辑灰度位、加权横向误差、是否至少命中一个传感器。位极性和 C1..C8 空间顺序须实机确认。 |
| `EncoderTicks` | `left`, `right` | 软件正交解码的有符号累计计数；`int32_t` 长时运行可能回绕。 |
| `ImuSample` | `a*`, `g*`, `roll/pitch/yawDeg`, `timestampMs` | 加速度为 g，角速度为 deg/s，姿态为 degree，时间戳为 ms。 |
| `Key` | `Up/Left/Down/Right/Center/None` | 五向键的逻辑键值。 |
| `KeyEvent` | `key`, `pressed`, `timestampMs` | 预留的事件 DTO；当前 Keypad 未产生该事件。 |

`car::clampCommand(int32_t)` 是轮端命令的统一限幅函数，所有进入 `MotorDriver::set()` 的值都会再次通过它裁剪。

### `Status` 返回值

| 值 | 含义 | 典型处理 |
| --- | --- | --- |
| `Ok` | 操作完成 | 消费输出数据。 |
| `Busy` | 当前没有可消费的数据，但不是故障 | 例如 DMP data-ready 边缘附近 FIFO 暂空；等待下一次事件。 |
| `Timeout` | BSP I2C 等待超时 | 停止相关控制并记录/重试；当前应用会使 IMU 进入失败状态。 |
| `BusError` | I2C 或 DMP 底层错误 | 进入安全停止状态。 |
| `NotReady` | 未初始化或前置条件不满足 | 调用 `begin()` 后再使用。 |
| `InvalidArgument` | 空指针、长度、时间间隔等参数无效 | 修正调用方参数。 |
| `DeviceMismatch` | 设备响应但身份不符 | 检查地址、芯片型号和接线。 |

## 3. Application：`app::CarApplication`

### 接口

```cpp
void init() noexcept;
void step() noexcept;
```

这是唯一的应用装配器和 cooperative superloop 调度器。对象是 `main()` 栈上的值对象；没有对外的状态修改接口，外部入口只能反复调用 `step()`。

### `init()` 的实际顺序

1. 调用 `motor_.stop()`，把 PWM 和方向置入安全停止状态；
2. 点亮状态 LED1、鸣叫约 30 ms 作为上电提示；
3. 按编译期 `ATTITUDE_CONFIG_BACKEND` 初始化 DMP 或原始 MPU6050；软件后端同时 `reset()` 姿态滤波器；
4. 初始化 SSD1306；失败只令 `oledReady_ = false`，不会阻断主循环；
5. LED2 显示 IMU 就绪状态，LED3 显示 OLED 就绪状态。

前提是 `bsp::init()` 已先完成 SysConfig、PWM、编码器和 SysTick 初始化。`init()` 内的 30 ms 忙等只发生在启动期，不能迁入周期路径。

### `step()` 的调度与安全行为

| 条件/周期 | 动作 | 失败行为 |
| --- | --- | --- |
| 每 5 ms | 读取灰度，以固定 dt 执行巡线 PID 和差速混合 | 无命中时 PID reset 并发布零建议；灰度极性需实测。 |
| `consumeImuDataReady()` 为真 | 软件后端调 `ImuReader::step()`（内部 poll → dt → `AttitudeFilter::update`）；DMP 后端直接 `notifyDataReady`+`poll`；BMI270+`BMI270_ONBOARD_FUSION` 直接 `poll`（内置 Mahony） | 除 `Ok`、`Busy` 外，置 `imuReady_ = false`、停止电机并打开蜂鸣器。 |
| CENTER 连按至少 500 ms | 解锁循线 Demo | 未解锁时 `SafetyGate` 始终输出零。 |
| 解锁且 UP 按住 | 允许巡线轮端建议通过 `SafetyGate` | 松开 CENTER/UP 或丢线时立即写入零。 |
| 每 500 ms | 翻转用户 LED | 仅表示 superloop 活着。 |
| 每 50 ms | 格式化 VOFA+ telemetry，并更新第一行 OLED | UART ring 空间不足时丢弃完整帧；OLED 单次失败目前未清除 `oledReady_`。 |

当前 Demo 已接入巡线 PID 和差速混合，速度 PI 尚未接入。候选命令始终先通过
`SafetyGate`；实车运行前仍需完成《维护手册》的方向、灰度和 PID 标定。

## 4. Middleware：可脱板测试的算法层

该层不包含 DriverLib、SysConfig 或 GPIO 依赖，只依赖 `Common/types.hpp` 和标准库。

### `middleware::Pid`

```cpp
Pid(PidConfig config) noexcept;
float update(float target, float measured, float dtSeconds) noexcept;
void reset() noexcept;
void configure(PidConfig config) noexcept;
PidConfig config() const noexcept;
```

`PidConfig` 包含 `kp`、`ki`、`kd`、`outputLimit` 和 `integralLimit`。`update()` 计算 `e = target - measured`，积分候选值先限幅，再计算 P/I/D，最终输出再按 `outputLimit` 限幅。首次调用将 D 项设为 0，避免初始微分冲击；`dt <= 0` 直接返回 0，且不更新历史状态。

`configure()` 替换参数并 reset 积分/微分历史，`config()` 返回当前参数副本。这里的
积分限幅是基础 anti-windup；它没有根据输出饱和反算积分。若将 PID 用于电机速度
闭环，应以固定控制周期调用，并用实测的方向与单位定义 target/measured。

### `middleware::DifferentialDrive`

```cpp
DifferentialDrive(KinematicsConfig config) noexcept;
car::WheelCommand mix(car::VehicleCommand command) const noexcept;
```

`KinematicsConfig::trackWidthMeters` 参与转向缩放。实现计算：

```text
turn  = angular * trackWidthMeters / 0.15
left  = clamp(linear - turn)
right = clamp(linear + turn)
```

这是一种归一化命令混合器，而非严格的 `v ± ω·track/2` SI 运动学模型；`0.15` 是当前归一化基准。若上层改为 m/s 与 rad/s，必须同时重定义输入、输出标定和该公式。

### `middleware::LineFollower`

```cpp
LineFollower(LineFollowerConfig config) noexcept;
car::VehicleCommand update(const car::LineSample& sample, float dtSeconds) noexcept;
bool configure(float kp, float ki, float kd, int16_t cruise) noexcept;
```

检测到线时，以 `target=0`、`measured=sample.error` 和固定 5 ms dt 执行受限
PID，输出 `{cruise, turn}`；丢线时 reset PID 并输出 `{0, 0}`。当前默认参数是
`kp=45, ki=0, kd=0, cruise=180`。运行时配置范围为 kp 0..300、ki 0..30、
kd 0..100、cruise 0..500；配置成功会 reset PID 状态。误差与转向符号仍必须
通过传感器位序和电机方向实机确认。

### `middleware::EncoderSpeedEstimator`

```cpp
EncoderSpeedEstimator(EncoderSpeedConfig config = {}) noexcept;
WheelSpeed update(car::EncoderTicks ticks, uint32_t timestampMs) noexcept;
void reset() noexcept;
```

第一次 `update()` 只播种上一帧 tick/timestamp，返回 0 速度和 `intervalMs=0`。之后：

```text
countsPerWheelTurn = gearRatio * encoderCountsPerMotorRevolution * quadratureMultiplier
metersPerCount     = π * wheelDiameterMeters / countsPerWheelTurn
speed              = (currentTicks - previousTicks) * metersPerCount / dtSeconds
```

若 `dt == 0`，返回零速度且不推进历史样本。默认参数是 Wheeltec C07A 的暂定值（65 mm、28:1、13 CPR、2x），不能直接作为闭环标定结果。

### `middleware::AttitudeFilter`

```cpp
AttitudeFilter(AttitudeFilterConfig config = {}) noexcept;
void reset() noexcept;
car::Status update(car::ImuSample& sample, float dtSeconds) noexcept;
```

它使用原始 MPU 数据计算 `rollAcc = atan2(ay, az)`、`pitchAcc = atan2(-ax, sqrt(ay²+az²))`，首次有效样本用加速度角初始化。`dt` 必须在 `(0, 0.1]` 秒内，否则返回 `InvalidArgument`。

| `AttitudeAlgorithm` | 原理 | 输出限制 |
| --- | --- | --- |
| `Complementary`（默认） | `angle = w * (angle + gyroRate * dt) + (1-w) * accelAngle`；默认 `w=0.98` | roll/pitch 融合，yaw 仅陀螺积分。 |
| `Kalman` | 每轴维护 `angle`、`bias` 和 2×2 协方差；预测后用加速度角创新量更新 | 同样只有 roll/pitch 融合；参数为 `QAngle/QBias/RMeasure`。 |
| `Mahony` | 四元数 AHRS，加速度叉乘修正 + PI 反馈；默认 `Kp=0.17 / Ki=0.004` | roll/pitch/yaw 全输出；yaw 为相对值，无磁力计必漂移。 |

Mahony 路径在加速度范数近零（自由落体或丢帧）时禁用叉乘修正、仅积分陀螺，并将重力参考置为单位向量，避免 `fastInvSqrt(0)` 产生 NaN 污染四元数；`fastInvSqrt` 的浮点↔整数重解释改用 `memcpy` 以避免严格别名 UB。

无磁力计时，`yawDeg += gz * dt` 必然漂移。DMP 后端不调用本类。

### `middleware::SafetyGate`

```cpp
car::WheelCommand apply(car::WheelCommand proposal, bool enabled) const noexcept;
```

`enabled == false` 时无条件返回 `{0, 0}`，否则原样返回 proposal；它不负责限幅，限幅由 `MotorDriver` 保证。这个类故意很小，用来把“算法给出建议”与“是否有权驱动电机”分开。

### `middleware::Telemetry`

```cpp
bool formatFrame(..., const car::ImuSample& imu, car::WheelCommand wheels,
                 const LineFollowerConfig& lineConfig, ...) const noexcept;
bool formatConfig(char* output, size_t capacity,
                  const LineFollowerConfig& lineConfig) const noexcept;
```

`formatFrame()` 输出 VOFA+ FireWater 命名 ASCII 帧，包含姿态、灰度、轮端命令、
编码器、PID、cruise、运行状态和 UART drop counter；`formatConfig()` 输出当前
循线配置。空指针、零容量或缓冲区不足时返回 `false`。该类只格式化，不访问
UART，也不保存协议状态；Application 的周期帧使用 224-byte 栈缓冲区。

### `middleware::VofaProtocol`

它以固定 80-byte 行缓冲消费 UART byte stream，只在收到 `\n` 后产生命令。
支持 `SET,LINE,kp,ki,kd`、`SET,CRUISE,value` 和 `GET,CONFIG`；普通十进制由
轻量解析器处理，不支持科学计数法。行过长、空行和未知命令返回 `Invalid`。

## 5. Drivers：设备语义层

Driver 层可依赖 BSP，但不向上泄漏 DriverLib 或 SysConfig 宏。

### 执行器与人机接口

| 类与接口 | 当前实现 | 使用约束 |
| --- | --- | --- |
| `MotorDriver::set(WheelCommand)` | 将左右命令限幅到 ±1000，调用 `bsp::setMotorDuty()`，并保存最后实际命令。 | 指令写入即生效；调用者应先走 `SafetyGate`。 |
| `MotorDriver::stop()` | 缓存清零并调用 `bsp::stopMotors()`；可重复调用。 | 上电、初始化失败、IMU 错误与显式停车均应调用。 |
| `MotorDriver::command() const` | 返回最近一次已限幅命令。 | 不读取 PWM 寄存器，也不表示电机真实转速。 |
| `ActiveBuzzer::set(bool)` | 代理至 `bsp::setBuzzer()`。 | 板上是有源、active-low 蜂鸣器，只支持开/关。 |
| `Led::setStatus(uint8_t, bool)` | 代理三个状态 LED；索引 0/1/其他分别映射 LED1/LED2/LED3。 | 调用方应仅使用 0..2；越界目前会落到 LED3。 |
| `Led::setUser(bool)` | 控制独立用户 LED。 | 用于心跳。 |
| `Keypad::pressed(Key) const` | 读取 active-low GPIO，按下返回 true。 | 当前没有软件消抖、边沿事件或长按管理；Application 处理 CENTER 长按。 |

### 线传感与编码器

| 类与接口 | 当前实现 | 使用约束 |
| --- | --- | --- |
| `LineSensorArray::read() const` | 读取 8-bit GPIO，按权重 `[-7,-5,-3,-1,1,3,5,7]` 求命中传感器的算术平均。 | 无命中时 `detected=false,error=0`。原始电平是否表示“在线上”必须实测。 |
| `Encoder::ticks() const` | 返回 BSP ISR 维护的左右累计 tick。 | 当前读取为两个 `volatile int32_t` 的直接快照；若需严格一致快照，应在 BSP 改为临界区/原子方案。 |
| `Encoder::reset()` | 清零 BSP 左右累计 tick。 | 与 ISR 并发时可能产生边界竞争；应在安全停止或明确同步点调用。 |

### MPU6050 原始数据：`drivers::Mpu6050`

实现 `drivers::ImuBackend`（`begin/poll/ready` 为虚方法，虚析构 `= default`）；`calibrateGyroBias` 是非虚具体方法，仅启动期调用。

```cpp
explicit Mpu6050(std::uint8_t addr = 0U) noexcept;
car::Status begin() noexcept;
car::Status poll(car::ImuSample& sample) noexcept;
bool ready() const noexcept;
```

构造函数可选指定 I2C 地址（`0x68` AD0 低 / `0x69` AD0 高）；默认 `0` 表示交给 `begin()` 自动探测。`begin()` 在地址为 `0` 时依次探测 `0x68`、`0x69`，以 `WHO_AM_I (0x75)` 返回值 `0x68` 确认；探测失败返回 `DeviceMismatch` 并保持 `ready_=false`。确认设备后唤醒电源（`PWR_MGMT_1=0`），并对软件后端写入采样默认值：`CONFIG` DLPF=3（44 Hz accel / 42 Hz gyro 带宽）、`SMPLRT_DIV=9`（约 100 Hz，匹配 `AttitudeFilter` 假设的 10 ms 轮询）、`GYRO_CONFIG`=±250 dps、`ACCEL_CONFIG`=±2 g。DMP 后端会自行重配这些寄存器，故默认值写入仅对软件后端生效。`poll()` 从 `ACCEL_XOUT_H (0x3B)` 连续读取 14 byte，按 ±2 g（`kAccelSensitivity=16384` LSB/g）和 ±250 dps（`kGyroSensitivity=131` LSB/(deg/s)）转换；`TEMP_OUT`（`0x41/0x42`）两字节在 burst 中跳过、当前管线不输出。roll/pitch/yaw 和 timestamp 初始为 0，需由 Application 及 `AttitudeFilter` 填充。

注意：`poll()` 当前不检查 `ready_`，因此正确用法是仅在 `begin()==Ok` 后调用；这也是该 Driver 的调用契约。

### MPU6050 DMP：`drivers::Mpu6050Dmp`

同样实现 `drivers::ImuBackend`；`notifyDataReady` 是非虚具体方法，由 Application 在消费 data-ready 后调用。

```cpp
car::Status begin() noexcept;
car::Status poll(car::ImuSample& sample) noexcept;
void notifyDataReady() noexcept;
bool ready() const noexcept;
```

`begin()` 通过 eMPL 完成 MPU 初始化、accel+gyro 传感器/FIFO、100 Hz sample rate、DMP firmware、车辆坐标系 orientation、6-axis quaternion、校准 gyro 与连续中断模式配置。任一步失败统一返回 `BusError`，成功后才置 `ready_`。

`notifyDataReady()` 只设置内部 `due_`，不做 I2C；它应由主循环在消费 `PB11` ISR 通知后调用。`poll()` 的状态机：

1. 未 ready 返回 `NotReady`；未 due 返回 `Busy`；
2. 读取 MPU interrupt status；FIFO overflow 时 reset FIFO，并以 `Busy` 表示恢复完成、等待下一包；
3. 循环读取 FIFO 直到最后一包，只在含 `INV_WXYZ_QUAT` 时产出结果；
4. Q30 quaternion 转 float，并以标准欧拉角公式计算 roll/pitch/yaw；accel 按 16384、gyro 按 16.4 进行缩放。

`orientationScalar()` 采用车体坐标 `X` 前、`Y` 左、`Z` 上的映射假设。传感器安装方向不同会使姿态轴和符号错误，必须实机复核。6-axis DMP 的 yaw 是相对航向，不是绝对方向。

### BMI270：`drivers::Bmi270`

实现 `drivers::ImuBackend`；`calibrateGyroBias` 是非虚具体方法，仅启动期调用。由 `ATTITUDE_BACKEND_BMI270` 选中，I2C0 上替换 MPU6050（同地址 `0x68/0x69`，靠 `CHIP_ID (0x00)`=`0x24` 区分）。

```cpp
explicit Bmi270(std::uint8_t addr = 0U) noexcept;
car::Status begin() noexcept override;
car::Status poll(car::ImuSample &sample) noexcept override;
car::Status calibrateGyroBias(std::uint16_t samples = 200U,
                              std::uint16_t delayMs = 5U) noexcept;
bool ready() const noexcept override;
```

`begin()` 先探测 `CHIP_ID`，再 disable advanced power-save，然后 burst-write 8192 字节 vendor config blob（`ThirdParty/bmi270/bmi270_config_file.h`）到 `INIT_DATA (0x5E)`（`INIT_CTRL` 0→1 包夹），读 `INTERNAL_STATUS (0x21)` 校验非零；config 写入传 `timeoutMs=500`（400 kHz 下约 200 ms）。随后写 `CMD=0x0E`（acc/gyro/temp en）、`NV_CONF=0`（I2C 模式）、量程/ODR：±2 g、±2000 dps、100 Hz、3 dB 滤波。注意 gyro ±2000 dps 的 `kGyroSensitivity=16.4` 是 MPU6050 ±250 dps（131）的 1/8，`AttitudeFilter` 参数实车可能需重标定。

`poll()` 连读 accel 6B（`0x0C`）+ gyro 6B（`0x12`），小端 LSB-first（与 MPU6050 大端不同），除灵敏度并减零偏。`BMI270_ONBOARD_FUSION`（默认 0）为 0 时 Euler 留 0、走 `ImuReader`+`AttitudeFilter`；为 1 时内置 Mahony 直接填 Euler、绕过 `AttitudeFilter`（dt 假设 ~100 Hz，实车按 data-ready 率复核）。

### OLED：`drivers::Ssd1306`

```cpp
car::Status begin() noexcept;
car::Status writeLine(const char* text) noexcept;
bool ready() const noexcept;
```

该实现固定走 I2C1、地址 `0x3C`。`begin()` 顺序写入最小初始化命令 `{AE,20,00,A8,3F,AF}`；任一命令失败返回 `BusError`，成功置 ready。`writeLine()` 仅写 page 0，从列 0 开始，内置 5×7 字模目前只覆盖 `A/C/D/E/I/M/O/R/U/Y`，其它字符显示为空白；数据以控制字节 `0x40`、每次最多 7 byte 像素分包写入。

因此它是状态提示显示，不是通用 SSD1306 字库或 framebuffer 驱动。传入空指针、未 `begin()` 成功时返回 `NotReady`。

## 6. BSP：硬件实现与 ISR 约束

BSP 是唯一可包含 `ti_msp_dl_config.h` 的层。下面接口不应由 Middleware 直接调用。

### 系统、时钟与时间

```cpp
void bsp::init() noexcept;
uint32_t bsp::millis() noexcept;
void bsp::delayMs(uint32_t ms) noexcept;
extern "C" void SysTick_Handler(void);
```

`init()` 调用 `SYSCFG_DL_init()`，先停止电机、启动两路 PWM、采样编码器初态，并以 80 MHz 配置 1 kHz SysTick。`SysTick_Handler()` 仅递增 `volatile g_millis`。`millis()` 是单调的 32-bit 毫秒计数，所有周期比较均使用无符号减法，因此可跨回绕工作。`delayMs()` 基于 `g_millis` 忙等倒数，可在调度器启动前和安全 `init()` 路径（如 MPU 静态零偏标定）使用；不得在 ISR 或实时控制回路调用。

### I2C

```cpp
Status i2cWrite(uint8_t bus, uint8_t address, const uint8_t* data, size_t length,
                uint32_t timeoutMs = 5) noexcept;
Status i2cWriteRegister(uint8_t bus, uint8_t address, uint8_t reg,
                        const uint8_t* data, size_t length,
                        uint32_t timeoutMs = 5) noexcept;
Status i2cReadRegister(uint8_t bus, uint8_t address, uint8_t reg,
                       uint8_t* data, size_t length,
                       uint32_t timeoutMs = 5) noexcept;
```

`bus==0` 选择 MPU I2C，其他值选择 OLED I2C；因此它不是通用总线号校验 API。`i2cWrite()` 受 8-byte 硬件 TX FIFO 限制，只允许 `1..8` byte。`i2cWriteRegister()` 先送寄存器地址，再在总线传输期间持续补充 FIFO，允许最多 `0x0ff0` byte，供 DMP firmware 写入。`i2cReadRegister()` 先写寄存器地址，再启动读传输并逐字节排空 RX FIFO。

所有接口在开始前等待 controller idle，并在传输中以 `millis()` 超时；底层 error 映射为 `BusError`。每次大写入在 start 后执行 `delay_cycles(3)`，用于规避 MSPM0 `I2C_ERR_13`。当前未实现 SDA/SCL 卡低的 GPIO bus recovery，超时后由上层进入安全停止。

### 电机 PWM 与方向

```cpp
void startMotorPwm() noexcept;
void setMotorDuty(int16_t left, int16_t right) noexcept;
void stopMotors() noexcept;
```

`setMotorDuty()` 将符号解释为方向、绝对值解释为占空比比例。实现顺序是：两个 PWM compare 先置 0 → 延迟 80 cycles → 更新 TB6612 AIN/BIN → 写入 `abs(command) * 8000 / 1000` compare。这个顺序避免换向时直接带载反转；BSP 假设上层已经限幅，所以不要直接传入超出 ±1000 的值。`stopMotors()` 即 `setMotorDuty(0, 0)`，同时清方向引脚。

### 输入、指示和 UART

```cpp
uint8_t readLineBits() noexcept;
bool keyPressed(car::Key key) noexcept;
void setStatusLed(uint8_t index, bool on) noexcept;
void setUserLed(bool on) noexcept;
void setBuzzer(bool on) noexcept;
bool uartTryWrite(const char* data, size_t length) noexcept;
bool uartTryRead(uint8_t& byte) noexcept;
```

`readLineBits()` 把 C1..C8 读取值放到 bit0..bit7，未转换灰度极性。`keyPressed()` 只支持五个实体键，`None` 返回 false，且按键为 active-low。状态 LED 和蜂鸣器是 active-low，用户 LED 是 active-high；这些电平细节被封装在 BSP/Driver 内。

UART 使用 256-byte TX 和 128-byte RX 静态 ring buffer。`uartTryWrite()` 只在
完整帧可入队时返回 true，否则丢弃整帧并增加 TX drop counter；`uartTryRead()`
从 RX ring 取单字节。UART ISR 只搬运 FIFO，命令解析和 telemetry 格式化均在
Application 主循环。

### 编码器与 MPU data-ready 中断

```cpp
void initializeEncoders() noexcept;
car::EncoderTicks encoderTicks() noexcept;
void resetEncoderTicks() noexcept;
bool consumeImuDataReady() noexcept;
extern "C" void GPIOB_IRQHandler(void);
```

编码器使用两相双边沿 GPIO 中断。`GPIOB_IRQHandler()` 对任一编码器边沿读取两个相位，拼为 2-bit 当前状态，并用 `(previous << 2) | current` 的 16 种跳变表判断：`1/7/14/8` 加一，`2/11/13/4` 减一，其余（保持或非法跳变）不计数。`initializeEncoders()` 在开中断后的基准点采样状态，避免首边沿误计。

同一 IRQ 同时处理 MPU PB11 data-ready：ISR 仅置 `imuDue=true` 并清中断。`consumeImuDataReady()` 读取并清除这个 flag，因此多个未处理 data-ready 会合并为一次通知；这是当前单标志策略的预期行为。ISR 内不得加入 I2C、DMP、PID、OLED、UART 或动态分配。

## 7. 姿态后端编译选择

在 `Middlewares/attitude_backend_config.h` 设置：

```c
#define ATTITUDE_CONFIG_BACKEND ATTITUDE_BACKEND_COMPLEMENTARY
// ATTITUDE_BACKEND_DMP
// ATTITUDE_BACKEND_KALMAN
```

| 后端 | Driver | 后续处理 | 适用性 |
| --- | --- | --- | --- |
| DMP | `Mpu6050Dmp` | eMPL FIFO quaternion 直接转 Euler | MCU 开销小。 |
| Complementary（默认） | `Mpu6050` | `AttitudeFilter(Complementary)` | 调试与快速验证。 |
| Kalman | `Mpu6050` | `AttitudeFilter(Kalman)` | 参数对比与噪声评估。 |

三个后端均依赖 MPU data-ready 通知；当前原始 `Mpu6050::poll()` 虽由 data-ready 触发，但本身读取寄存器而非 FIFO。切换后端至少应构建目标后端，并检查 DMP vendor source 的编译门是否与配置保持一致。

## 8. 扩展时的推荐接入顺序

以“自动巡线 + 速度闭环”为例，建议保持现有职责边界：

1. 在 Application 的固定周期读取 `Encoder::ticks()`，用 `EncoderSpeedEstimator` 得到 `WheelSpeed`；
2. 以固定 5 ms 调用 `LineFollower::update(lineSample_, 0.005F)`，并将结果交给 `DifferentialDrive::mix()`；
3. 左右轮各用一个 `Pid`，把目标/实测速度转换为 `WheelCommand`；
4. 在完成所有安全判定后，通过 `SafetyGate::apply()`；
5. 最后且只在 Application 中调用 `MotorDriver::set()`。

不要让 Middleware 直接调用 BSP，也不要在 `GPIOB_IRQHandler()` 中计算速度、PID 或 I2C。这样日后从 superloop 迁移到 RTOS 时，主要替换调度方式，而不需要重写算法与设备接口。

## 9. 当前实现边界清单

- SysConfig 描述资源事实；本文档不复制或替代其完整引脚/寄存器配置。
- 线传感极性、DMP 安装轴、左右电机正方向、编码器符号/倍率、轮径和 PID 均为待实机验证项。
- Keypad 尚无消抖，OLED 尚无通用字库/多页 framebuffer。
- I2C 超时会返回，但尚无物理总线恢复；DMP FIFO overflow 仅 reset FIFO 后等待下一包。
- `Pid`、`LineFollower` 和 `DifferentialDrive` 已接入循线 Demo；编码器速度闭环
  尚未接入。

这些限制是当前安全分阶段策略的一部分。新增功能时应同步更新本文档、对应模块文档与上板验收记录。

# 设备 Drivers

| Driver | 公开语义 | 关键边界 |
| --- | --- | --- |
| `MotorDriver` | 有符号 `WheelCommand`、可重复 `stop()` | 指令限幅至 ±1000；方向安全在 BSP。 |
| `Encoder` | 读取/复位 `EncoderTicks` | 不在 Driver 内计算速度。 |
| `LineSensorArray` | 8-bit `LineSample`、加权 error | 灰度极性必须实机确认。 |

`LineSensorArray` 的 `read()` 把 `bsp::readLineBits()` 的原始 8 bit 归一为“置位 = 压在线上”，再做 `-7..+7` 加权求平均。灰度模块指示 LED 默认“贴白亮、贴黑灭”，但 OUT 引脚电平是否与指示灯同相取决于传感器板，必须在 `Drivers/line_sensor_config.h` 按 `LINE_SENSOR_LINE_IS_HIGH`（默认 `1`：高电平 = 压线）实机确认；翻转极性只改该宏，不改正文权重逻辑。

## `ImuBackend` 接口

`Drivers/imu_backend.hpp` 定义通用 IMU 抽象基类 `drivers::ImuBackend`，只暴露 `begin/poll/ready` 三个方法。软件姿态滤波路径（Complementary/Kalman/Mahony）只依赖本接口，不直接依赖具体芯片驱动；为其它 6 轴 IMU（如 ICM 系列）接入软件滤波只需新增一个 `ImuBackend` 实现。

芯片专有的行为不进接口：`Mpu6050::calibrateGyroBias`（启动期静态零偏标定）、`Mpu6050::updateGyroBiasFromStationarySample`（由 Application 静止检测保护的慢速温漂跟踪）和 `Mpu6050Dmp::notifyDataReady`（eMPL FIFO 读取前的 due 锁存）留在各自具体类，由 Application 经编译期 `ATTITUDE_CONFIG_BACKEND` 分支调用。`Mpu6050` 与 `Mpu6050Dmp` 均实现 `ImuBackend`，保持类型关系对称；虚析构 `= default`，工程带 `-fno-rtti -fno-exceptions`，禁止 `dynamic_cast`/`throw`。

## `Bmi270`（BMI270 后端）

`Drivers/bmi270.{hpp,cpp}` 是 BMI270 6 轴 IMU 的 I2C 驱动，实现 `drivers::ImuBackend`，由 `ATTITUDE_BACKEND_BMI270` 选中。与 `Mpu6050` 同处 I2C0、同地址 `0x68/0x69`（SDO 低/高），靠 `CHIP_ID (0x00)`=`0x24` 区分（MPU6050 是 `WHO_AM_I 0x75`=`0x68`）；两者不能同板共存，BMI270 是替换而非并存。

BMI270 初始化必须先 burst-write 8192 字节 vendor config blob（`ThirdParty/bmi270/bmi270_config_file.h`，逐字节不改）到 `INIT_DATA (0x5E)`，在 `INIT_CTRL (0x59)` 的 0/1 之间，再读 `INTERNAL_STATUS (0x21)` 校验非零。该写入由 BSP 的 `i2cWriteRegister` 分段填 FIFO 完整传输，驱动传 `timeoutMs=500`（400 kHz 下约 200 ms，BSP 默认 5 ms 不够）。默认量程 ±2 g（`kAccelSensitivity=16384` LSB/g）、±2000 dps（`kGyroSensitivity=16.4` LSB/(°/s)）——gyro 量程是 MPU6050 ±250 dps 的 8 倍，`AttitudeFilter` 参数实车可能需重标定。ODR 100 Hz 匹配 10 ms 轮询。数据寄存器小端 LSB-first（accel `0x0C`、gyro `0x12`，各 6 字节），与 MPU6050 大端布局不同。

`BMI270_ONBOARD_FUSION` 宏（默认 0）切换 `poll()` 语义：未定义/0 时只填 `ax..gz`（g、deg/s），Euler 留 0，走 `ImuReader`+`AttitudeFilter`（与 Mpu6050 一致）；为 1 时 `poll()` 内置 Mahony（移植自参考工程，含退化输入保护）直接填 `rollDeg/pitchDeg/yawDeg`，绕过 `AttitudeFilter`，dt 假设 ~100 Hz data-ready。状态：已构建，待实机验证（CHIP_ID、config 接受、量程/ODR、轴向、I2C 上拉）。
| `Keypad` | active-low 稳定按键状态 | polling + 20 ms 去抖；长按/按下沿逻辑在 Application。 |
| `Led` / `ActiveBuzzer` | 不暴露 GPIO 极性 | 蜂鸣器是 active-low 有源器件。 |
| `Ssd1306` | 128x64 的八行缓存文本与分段刷新 | I2C1 DMA 一次一包；调用方不得在 ISR 刷新。`service()` 帧缓冲已完全同步（无脏行）时返回 `Ok`（空闲），有待处理传输时返回 `Busy`，以便区分"空闲"与"传输中"。 |

`Mpu6050` 实现 `ImuBackend`，是原始 14-byte burst 驱动，输出 accel（g）和 gyro（deg/s），供软件姿态后端使用。构造函数接受可选 I2C 地址（`0x68` AD0 低 / `0x69` AD0 高）；默认 `0` 时 `begin()` 依次探测两个合法地址并按 WHO_AMI（`0x68`）确认。`begin()` 在 raw/software-filter 后端下显式唤醒并写入采样默认值：DLPF=3（44 Hz accel / 42 Hz gyro 带宽，抑制混叠）、`SMPLRT_DIV=9`（约 100 Hz，匹配 `AttitudeFilter` 假设的 10 ms 轮询）、量程保持 ±2 g / ±250 dps，因此 `poll()` 灵敏度（`kAccelSensitivity=16384`、`kGyroSensitivity=131`）不变。`temp_out` 字节（`0x41/0x42`）当前管线未使用。DMP 后端会自行重配这些寄存器，故上述写入仅对软件后端生效。

`Mpu6050Dmp` 实现 `ImuBackend`，初始化 MPU FIFO、100 Hz DMP、6-axis quaternion、calibrated gyro，并转换为 `ImuSample`（`poll` 直接产出 Euler）。DMP 与原始驱动互斥，Application 只初始化选中的一个；DMP 路径不经 `ImuReader`，由 Application 直接调用 `notifyDataReady` + `poll`。

`ThirdParty/eMPL` 来自 InvenSense Motion Driver。只允许加入 `EMPL_TARGET_MSPM0` target 分支、外围编译门和 port 映射；禁止修改 DMP firmware、FIFO parser 与算法。

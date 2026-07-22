# 小车固件方案（第一阶段）

## 目标与范围

本工程面向 MSPM0G3507 差速小车。第一阶段目标是在不引入 RTOS 的前提下，完成安全电机驱动、八路灰度巡线、MPU6050/ICM I2C 采样、OLED 硬件 I2C 显示、UART telemetry，以及为编码器闭环预留清晰边界。

已配置的硬件基线为：SYSPLL 80 MHz、TB6612 PWM 10 kHz、MPU6050 `I2C0/PA0/PA1` 400 kHz、OLED `I2C1/PB2/PB3` 400 kHz、四个 UART 115200。具体引脚和参数以 `TI_Car_Base.syscfg` 为准。

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

## 参考工程的取舍

### MPU6050-F4

作为主要结构参考：其 `Application` 静态组合 BSP、Driver 与 Middleware；Driver 不依赖 HAL；ISR 仅通知，I2C 和姿态处理在任务/主循环上下文执行。MSPM0 工程应复用这些接口边界、状态机和 host-side 测试理念，而不能复用 STM32 HAL handle、TIM2、EXTI callback 或 FreeRTOS 资源统计实现。

### Wheeltec 小车与 MPU6050 示例

借鉴 TB6612 安全驱动、四路循迹、编码器速度 PI 和 MPU6050 寄存器操作的功能定义。不得将其把巡线、控制、I2C 和日志集中在 ISR 的实现方式直接带入本工程。

### 逐飞 MSPM0G3507 库

作为 DriverLib 用法、80 MHz 时钟和外设能力的参考，不直接引入。其通用库通过枚举、全局外设表和直接寄存器访问适配多种资源组合，与本工程的 SysConfig 单一配置来源不匹配；并且其源文件声明 GPL-3.0，直接复用会引入相应许可证义务。

本工程所谓“静态配置”是只为本车固化实际资源：I2C0、四个 UART、两路 PWM、PB2/PB3 的 software I2C、既定方向 GPIO 和传感器 GPIO，而不是保留通用 pin enum 或全局 callback table。

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
- TM2027 五向按键 Common 接地，`GPIO_KEY` 配置为 active-low polling；按键 debounce 属于 Driver，不应让 Application 直接读取 GPIO。
- 三路 LED 阳极上拉至 3.3 V，`GPIO_LED` 为 active-low output；上电 high（熄灭），只能通过 Driver 提供的语义化接口控制。
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

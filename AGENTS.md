# TI_Car_Base 开发指南

## 目标与分层

本工程面向 MSPM0G3507 差速小车，参考 Wheeltec C07A 的 TB6612、四路巡线和 MPU6050 示例；实现按以下边界组织：

`Hardware -> Driver/BSP -> Middleware -> Application`

- `Hardware`：只放板级引脚、电机、循迹和传感器适配。
- `Driver/BSP`：封装 `ti_msp_dl_config.h` 和 DriverLib，不能让 Application 直接操作寄存器。
- `Middleware`：姿态、循迹策略、速度闭环与 telemetry；保持可脱离硬件单测。
- `Application`：任务调度和小车业务状态机。

新增代码应采用该目录结构；不要把 Wheeltec 示例整体复制进根目录，也不要修改 SysConfig 生成物。

## 配置唯一来源

- 引脚、时钟、PWM、I2C、UART 与 GPIO 的唯一来源是 `TI_Car_Base.syscfg`。
- 必须通过 CCS SysConfig MCP/GUI 修改 `.syscfg`；禁止直接编辑，也禁止编辑 `Debug/ti_msp_dl_config.*`、`Debug/device.*` 或 `Debug/Event.dot`。
- `Debug/` 为构建和 SysConfig 生成输出，不纳入版本控制。
- 工程目前使用 SysConfig `1.28.0`；变更版本前需先说明兼容性影响。

## 已配置硬件接口

| 功能 | 外设/引脚 | 约束 |
| --- | --- | --- |
| TB6612 PWMA/PWMB | `TIMA1 CCP0/CCP1`: `PB2` / `PB3` | `PWM_MOTOR`，4 kHz，初始占空比 0%。先写方向再更新 PWM；停止时停止计数器并拉低输出。 |
| TB6612 AIN1/AIN2 | `PA13` / `PA14` | `GPIO_MOTOR_DIR`，初始低。 |
| TB6612 BIN1/BIN2 | `PA16` / `PA17` | `GPIO_MOTOR_DIR`，初始低。 |
| 四路循迹 | `PA27`, `PA12`, `PB16`, `PB17` | `GPIO_LINE_SENSOR`：`IR_DH1..4`，普通输入；信号极性应在驱动层统一转换。 |
| MPU6050 I2C | `I2C0`: SDA `PA0`，SCL `PA1` | `I2C_MPU6050` controller，400 kHz，外部 3.3 V 上拉必需。地址取决于 AD0：低=`0x68`，高=`0x69`。 |
| MPU6050 INT | `PB11` | `GPIO_MPU6050_DATA_READY` 上升沿中断；ISR 只置位 data-ready flag，I2C 读取在任务上下文完成。 |
| 调试/telemetry | `UART0`: TX `PA10`，RX `PA11` | `UART_CONSOLE`，115200 8N1，TX FIFO interrupt。该引脚为 LP-MSPM0G3507 XDS110 backchannel。 |
| SWD | `PA19`, `PA20` | 调试保留，禁止分配给应用。 |

## 关键实现约束

- I2C FIFO 仅 8 bytes；超过 FIFO 深度的事务必须采用中断/分段填充。开始传输前等待总线 idle；轮询路径需保留 I2C_ERR_13 的短延时。
- PWM 定时器不会由 SysConfig 自动启动。应用初始化后显式调用 `DL_TimerA_startCounter(PWM_MOTOR_INST)`。
- 方向变更必须先将两路 PWM 置为安全状态，等待最短死区，再更新 `AIN/BIN`；禁止运行时切换方向脚造成桥臂直通。
- UART TX ISR 只负责搬运 FIFO；禁止在 ISR 内调用 `printf`、日志或 I2C。
- 硬件信号电平、PWM 频率、MPU6050 INT 极性与实际车板一致性必须实机验证；编译成功不代表接线正确。

## 验证与 Git

- 每次改动 `.syscfg` 后，检查 SysConfig 无 errors/warnings，再使用 CCS `buildProject` 构建。
- 不提交 `Debug/`、`Release/`、`.clangd`、IDE 缓存或 SDK 生成物。
- 提交前只暂存当前任务文件，使用 Conventional Commit；未经明确要求不得 push。

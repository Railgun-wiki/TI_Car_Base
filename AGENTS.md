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
| TB6612 PWMA | `TIMA1 CCP0`: `PA28` | `PWM_MOTOR`，10 kHz，初始占空比 0%。 |
| TB6612 PWMB | `TIMG12 CCP0`: `PB20` | `PWM_MOTOR_B`，10 kHz，初始占空比 0%。两路 PWM 使用不同 timer。 |
| TB6612 AIN1/AIN2 | `PA13` / `PB26` | `GPIO_MOTOR_DIR`，初始低。 |
| TB6612 BIN1/BIN2 | `PB9` / `PB7` | `GPIO_MOTOR_DIR`，初始低。 |
| 八路灰度循迹 | `PA31`, `PA12`, `PB8`, `PA27`, `PB0`, `PA30`, `PB21`, `PB10` | `GPIO_LINE_SENSOR`：`C1..C8`，普通输入；信号极性应在驱动层统一转换。 |
| 左编码器 | `PB23` / `PB12` | `GPIO_ENCODER`：`ENCODER_LEFT_A/B`，双边沿 GPIO interrupt，软件正交解码。 |
| 右编码器 | `PB4` / `PB5` | `GPIO_ENCODER`：`ENCODER_RIGHT_A/B`，双边沿 GPIO interrupt，软件正交解码。 |
| MPU6050 I2C | `I2C0`: SDA `PA0`，SCL `PA1` | `I2C_MPU6050` controller，400 kHz，外部 3.3 V 上拉必需。地址取决于 AD0：低=`0x68`，高=`0x69`。 |
| OLED I2C | `I2C1`: SCL `PB2`，SDA `PB3` | `I2C_OLED` controller，400 kHz；使用独立硬件 I2C，不与 MPU 的 I2C0 竞争。外部 3.3 V 上拉必需。 |
| MPU6050/ICM INT | `PB11` | `GPIO_MPU6050_DATA_READY.MPU6050_INT`，上升沿 GPIO interrupt。ISR 只置位 data-ready flag；I2C 读取在主循环执行。 |
| TM2027 五向按键 | UP `PA14`、LEFT `PA15`、DOWN `PA17`、RIGHT `PB25`、CENTER `PB24` | `GPIO_KEY`，input + internal pull-up，Common 接地，按下为 low；第一阶段 polling，Driver 层做 debounce。 |
| 板载状态 LED | LED1 `PB14`、LED2 `PB18`、LED3 `PA22` | `GPIO_LED`，push-pull output，初始 high（熄灭）。LED 阳极上拉至 3.3 V，low 点亮。 |
| 板载用户 LED | `PB22` | `GPIO_LED.USER_LED`，push-pull output，初始 low（熄灭）；高电平点亮。 |
| 有源蜂鸣器 | `PA21` | `GPIO_BUZZER.BUZZER`，push-pull output，初始 high（关闭）。S8550 PNP 高边驱动，GPIO low 开启；无需 PWM。 |
| 调试/telemetry | `UART0`: TX `PA10`，RX `PA11` | `UART_CONSOLE`，115200 8N1，TX FIFO interrupt。该引脚为 LP-MSPM0G3507 XDS110 backchannel。 |
| 扩展 UART | UART1 `PA8/PA9`；UART2 `PB15/PB16`；UART3 `PA26/PB13` | `UART1_MODULE`、`UART2_MODULE`、`UART3_MODULE`，均为 115200 8N1、FIFO。 |
| SWD | `PA19`, `PA20` | 调试保留，禁止分配给应用。 |

## 关键实现约束

- HFXT 使用外部 40 MHz 晶振（`PA5/PA6`），LFXT 使用外部 32.768 kHz 晶振（`PA3/PA4`）；SYSPLL 以 HFXT 为参考，CPU、MCLK 与 ULPCLK 保持 80 MHz。禁止在未重新核对 Flash wait states、UART/I2C 时序和功耗的情况下改动时钟树。
- I2C FIFO 仅 8 bytes；超过 FIFO 深度的事务必须采用中断/分段填充。开始传输前等待总线 idle；轮询路径需保留 I2C_ERR_13 的短延时。
- PWM 定时器不会由 SysConfig 自动启动。应用初始化后显式调用 `DL_TimerA_startCounter(PWM_MOTOR_INST)` 和 `DL_TimerG_startCounter(PWM_MOTOR_B_INST)`。
- 方向变更必须先将两路 PWM 置为安全状态，等待最短死区，再更新 `AIN/BIN`；禁止运行时切换方向脚造成桥臂直通。
- UART TX ISR 只负责搬运 FIFO；禁止在 ISR 内调用 `printf`、日志或 I2C。
- OLED 使用 `I2C1` 的有超时非阻塞/分段传输；禁止在 ISR 或 1 kHz 电机控制路径中刷新显示。
- `GPIO_MPU6050_DATA_READY` ISR 必须先清中断标志、再置位采样通知；禁止在 ISR 内执行 I2C、姿态解算或日志。
- 有源蜂鸣器通过 `GPIO_BUZZER.BUZZER` 控制：`Set` 为关闭、`Clear` 为开启；禁止将其改配为 PWM，以免引入无意义的开关噪声。
- W25Q128 Flash（`PB6/PB7/PB8/PB9`）与板载 `PB21` 用户按键不纳入本固件：前者与 TB6612/灰度引脚冲突，后者与 `C7` 冲突。`PA18` BSL 按键也按用户决定保持禁用，以免影响启动维护行为。除非硬件改线或重新确认启动流程，禁止添加 SPI、PA18 或 PB21 配置。TM2027 五向按键不受影响，应保持 `GPIO_KEY` 配置。
- 编码器 GPIO ISR 必须读取两相的当前状态并更新有符号计数；禁止在 ISR 内计算速度、执行 PID 或打印。输入滤波与解码方向需按实测最高脉冲率和车轮方向验证。
- 硬件信号电平、PWM 频率、MPU6050 INT 极性与实际车板一致性必须实机验证；编译成功不代表接线正确。

## 验证与 Git

- 每次改动 `.syscfg` 后，检查 SysConfig 无 errors/warnings，再使用 CCS `buildProject` 构建。
- 不提交 `Debug/`、`Release/`、`.clangd`、IDE 缓存或 SDK 生成物。
- 提交前只暂存当前任务文件，使用 Conventional Commit；未经明确要求不得 push。

## C/C++ 格式化

- 修改或新增 C/C++ 源码后，提交前必须使用项目可用的
  `clang-format -i` 格式化对应的 `.c`、`.cpp`、`.h`、`.hpp` 文件。
- 格式化范围仅限本次修改的源码；不得用格式化制造无关差异。
- 格式化后必须重新构建；格式化不替代编译或实机验证。

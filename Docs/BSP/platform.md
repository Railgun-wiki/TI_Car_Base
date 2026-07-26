# BSP 平台适配

对应 `BSP/`。BSP 是唯一包含 `ti_msp_dl_config.h` 与 DriverLib 的层。

| 文件 | 职责 |
| --- | --- |
| `system.*` | 手工排序的 SysConfig 初始化、80 MHz SysTick 1 ms、PWM 启动、`delayMs()` 忙等延时。 |
| `i2c.*` | I2C0 轮询超时事务和仅启动期使用的 address-only probe；I2C1 OLED 异步 DMA 发送；支持 DMP firmware 的流式 FIFO 写入。 |
| `uart.*` | UART0 RX/TX interrupt、固定 ring buffer、整帧 best-effort enqueue 与 TX-drain 查询。 |
| `motor.*` | TB6612 方向、PWM、死区与停止。 |
| `input.*` / `indicator.*` | 灰度、按键、LED、蜂鸣器原始电平。 |
| `encoder.*` | 软件正交计数和 MPU data-ready ISR 分发。 |
| `MPU6050/mpu6050_empl_port.*` | eMPL 的 I2C、delay、单调时间 port。 |

`SysTick_Handler` 仅递增毫秒计数。`GPIOB_IRQHandler` 只更新 tick、清中断并置
IMU flag；`UART0_IRQHandler` 只在硬件 FIFO 与 256-byte TX / 128-byte RX
ring buffer 之间搬运数据；`I2C1_IRQHandler` 仅收尾 OLED 的 DMA 写入（TX Done /
NACK / arbitration-lost），I2C1 controller 的 RX 仍由轮询路径处理。ISR 内禁止
I2C、DMP、PID、OLED、UART 格式化和动态分配。TX ring 空间不足时丢弃完整帧，RX
ring 满时丢弃新 byte，并分别累计 drop counter。

`bsp::delayMs()` 基于 `g_millis` 的 SysTick 倒数忙等，可在调度器启动前和安全
的 `init()` 路径（如 MPU 静态零偏标定）中使用；不得在中断或实时控制回路里调用。

`bsp::init()` 不直接调用生成的 `SYSCFG_DL_init()`，而是按生成函数手工排列为
Power/GPIO/SYSCTL/PWM → UART0 → I2C0/I2C1 → 其余 UART/DMA/clock。这样 UART0
可在外设探测前输出启动日志。**若 SysConfig 配置发生变更，必须核对生成的
`SYSCFG_DL_init()` 并同步此手工列表，确保没有漏掉新增初始化函数。**

`i2cProbe()` 以 0-byte controller quick command 扫描地址，默认 timeout 为 1 ms；
它只供 Application 的启动自检调用，运行态不扫描总线。启动日志每一帧都等待
`uartTxIdle()`，避免 256-byte TX ring 满而丢失诊断内容。

MSPM0 I2C FIFO 为 8 byte。MPU 的 I2C0 保持轮询：`i2cWriteRegister()` 在单次
事务中补充 TX FIFO，供 eMPL 写 DMP memory；读取持续排空 RX FIFO。每次事务检查
idle、错误和 timeout，并执行 I2C_ERR_13 短延时。

OLED 的 I2C1 使用 SysConfig 配置的 TX FIFO DMA Event 1。`i2cOledWriteDma()`
只在 controller idle 时配置 DMA 源地址、`MTXDATA` 目的地址和传输长度，随后立即
返回；`I2C1_IRQHandler` 只在 TX Done、NACK 或 arbitration-lost 时记录最终状态并
关闭 DMA channel。调用者必须在 DMA 完成前保持源 buffer 有效，并轮询
`i2cOledDmaStatus()`；它在超时后复位 controller 并返回 `Timeout`。物理 SCL/SDA
被拉低后的 GPIO bus-recovery 尚未实现。

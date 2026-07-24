# BSP 平台适配

对应 `BSP/`。BSP 是唯一包含 `ti_msp_dl_config.h` 与 DriverLib 的层。

| 文件 | 职责 |
| --- | --- |
| `system.*` | SysConfig 初始化、80 MHz SysTick 1 ms、PWM 启动。 |
| `i2c.*` | I2C0/I2C1 同步超时事务；支持 DMP firmware 的流式 FIFO 写入。 |
| `uart.*` | UART0 RX/TX interrupt、固定 ring buffer 和整帧 best-effort enqueue。 |
| `motor.*` | TB6612 方向、PWM、死区与停止。 |
| `input.*` / `indicator.*` | 灰度、按键、LED、蜂鸣器原始电平。 |
| `encoder.*` | 软件正交计数和 MPU data-ready ISR 分发。 |
| `MPU6050/mpu6050_empl_port.*` | eMPL 的 I2C、delay、单调时间 port。 |

`SysTick_Handler` 仅递增毫秒计数。`GPIOB_IRQHandler` 只更新 tick、清中断并置
IMU flag；`UART0_IRQHandler` 只在硬件 FIFO 与 256-byte TX / 128-byte RX
ring buffer 之间搬运数据。ISR 内禁止 I2C、DMP、PID、OLED、UART 格式化和动态
分配。TX ring 空间不足时丢弃完整帧，RX ring 满时丢弃新 byte，并分别累计 drop
counter。

MSPM0 I2C FIFO 为 8 byte。`i2cWriteRegister()` 在单次事务中补充 TX FIFO，供 eMPL 写 DMP memory；读取持续排空 RX FIFO。每次事务检查 idle、错误和 timeout，并执行 I2C_ERR_13 短延时。物理 SCL/SDA 被拉低后的 GPIO bus-recovery 尚未实现。

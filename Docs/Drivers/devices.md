# 设备 Drivers

| Driver | 公开语义 | 关键边界 |
| --- | --- | --- |
| `MotorDriver` | 有符号 `WheelCommand`、可重复 `stop()` | 指令限幅至 ±1000；方向安全在 BSP。 |
| `Encoder` | 读取/复位 `EncoderTicks` | 不在 Driver 内计算速度。 |
| `LineSensorArray` | 8-bit `LineSample`、加权 error | 灰度极性必须实机确认。 |
| `Keypad` | active-low 稳定按键状态 | polling + 20 ms 去抖；长按/按下沿逻辑在 Application。 |
| `Led` / `ActiveBuzzer` | 不暴露 GPIO 极性 | 蜂鸣器是 active-low 有源器件。 |
| `Ssd1306` | 四行缓存文本与分段刷新 | I2C1 DMA 一次一包；调用方不得在 ISR 刷新。 |

`Mpu6050` 是原始 14-byte burst 驱动，输出 accel（g）和 gyro（deg/s），供软件姿态后端使用。

`Mpu6050Dmp` 初始化 MPU FIFO、100 Hz DMP、6-axis quaternion、calibrated gyro，并转换为 `ImuSample`。DMP 与原始驱动互斥，Application 只初始化选中的一个。

`ThirdParty/eMPL` 来自 InvenSense Motion Driver。只允许加入 `EMPL_TARGET_MSPM0` target 分支、外围编译门和 port 映射；禁止修改 DMP firmware、FIFO parser 与算法。

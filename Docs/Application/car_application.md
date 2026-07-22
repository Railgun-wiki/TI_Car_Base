# CarApplication

对应 `Application/car_application.*`。`CarApplication` 是唯一的设备装配和电机命令发布层；它不直接包含 DriverLib 或 SysConfig 宏。

## 初始化

1. 显式 `MotorDriver::stop()`；
2. LED/蜂鸣器给出上电提示；
3. 按已选姿态后端初始化 `Mpu6050Dmp` 或原始 `Mpu6050`；
4. 初始化 OLED；设备不存在时仅降级显示；
5. 未就绪设备不得解除电机安全状态。

## 周期工作

| 工作 | 节奏 | 规则 |
| --- | --- | --- |
| 灰度采样 | 200 Hz | 只更新 `LineSample`。 |
| IMU | PB11 data-ready | 主循环读取；ISR 只置标志。 |
| telemetry | 20 Hz | UART 满时丢弃该帧。 |
| LED 心跳 | 2 Hz 翻转 | 表示 superloop 仍运行。 |
| OLED | 低优先级 | 不得影响电机/IMU 路径。 |

CENTER 连续按住 500 ms 才 armed；UP/DOWN 为两轮低占空比正/反转，松手停车。该 Demo 不代表自动巡线或速度闭环已启用。IMU transport/FIFO 错误会停止电机；`Busy`（尚无完整 FIFO 包）不视为故障。

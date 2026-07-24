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
| 灰度采样/巡线 PID | 200 Hz | 固定 dt=5 ms，更新轮端建议。 |
| IMU | PB11 data-ready | 主循环读取；ISR 只置标志。 |
| VOFA+ telemetry | 20 Hz | UART ring 满时丢弃完整帧。 |
| VOFA+ command | 主循环轮询 | 仅停车时更新循线参数。 |
| LED 心跳 | 2 Hz 翻转 | 表示 superloop 仍运行。 |
| OLED | 低优先级 | 不得影响电机/IMU 路径。 |

## 循线 Demo

该应用参考 `11_PID_car` 的“灰度加权位置 → 转向修正 → 左右轮差速”控制链，
但通过现有分层实现：

`LineSensorArray -> LineFollower -> DifferentialDrive -> SafetyGate -> MotorDriver`

- CENTER 连续按住 500 ms 后 armed，继续同时按住 CENTER + UP 才运行；
- 松开 CENTER 或 UP 时立即发布零轮速命令；
- 8 路灰度均未检测到线时，`LineFollower` 发布零命令并停车；
- 默认巡航命令为 180，PID 为 `45/0/0`；这些参数只完成编译验证，
  必须根据实际车速、赛道曲率和传感器高度进行实车标定；
- 当前 Demo 是开环 PWM 循线，不包含参考工程的编码器速度 PID 闭环。

IMU transport/FIFO 错误会记录 IMU 不可用并触发一次停车；`Busy`（尚无完整
FIFO 包）不视为故障。纯循线控制不依赖 IMU，OLED 会优先显示循线状态。

## VOFA+ 输出

UART0 使用 `115200 8N1`。20 Hz FireWater 命名帧包含 `roll/pitch/yaw`、
灰度 bits/error、左右轮命令、编码器 ticks、PID、cruise、运行/IMU 状态和
RX/TX drop counter。姿态单位为 degree；6-axis yaw 是相对航向，会漂移。

调参命令和范围见 `Docs/Middlewares/control.md`。串口 RX/TX ISR 只搬运固定
ring buffer，命令解析、参数更新和帧格式化都在主循环。

## 验证状态

- 已构建：2026-07-25 使用 CCS `buildProject` 完成 Debug 构建，无 errors/warnings；
- 待实机验证：灰度高/低电平极性、左右传感器顺序、电机正方向、急弯差速、
  丢线停车距离以及 CENTER + UP 松手停车时延。

# CarApplication

对应 `Application/car_application.*`。`CarApplication` 是唯一的设备装配和电机命令发布层；它不直接包含 DriverLib 或 SysConfig 宏。

## 初始化

普通循线 PID、巡航速度、按键步进和调度周期统一由
`Config/vehicle_tuning.hpp` 的 `LINE_FOLLOW_*` 宏配置；三灯组合统一定义在
`Config/status_led_config.hpp`，Application 不保存重复默认值。

1. 显式 `MotorDriver::stop()`；
2. LED/蜂鸣器给出上电提示；
3. 按已选姿态后端初始化：DMP 分支直接 `Mpu6050Dmp::begin()`；软件滤波分支 `Mpu6050::begin()` + `calibrateGyroBias()`（具体类，启动期一次性）+ `ImuReader::reset()`；
4. 初始化 OLED；设备不存在时仅降级显示；
5. 未就绪设备不得解除电机安全状态。

## 周期工作

| 工作 | 节奏 | 规则 |
| --- | --- | --- |
| 灰度采样/巡线 PID | 目标 200 Hz | 使用 `millis()` 实测 elapsed time 更新轮端建议。 |
| IMU | PB11 data-ready | 主循环读取；ISR 只置标志。软件滤波分支经 `ImuReader::step(rawImu_, softwareAttitude_, now, sample)` 驱动（`rawImu_` 以 `ImuBackend&` 上溯），DMP 分支直接 `notifyDataReady` + `poll`。 |
| VOFA+ telemetry | 20 Hz | UART ring 满时丢弃完整帧。 |
| VOFA+ command | 主循环轮询 | 仅停车时更新循线参数。 |
| LED 心跳 | 2 Hz 翻转 | 表示 superloop 仍运行。 |
| OLED | 100 ms 更新文本、2 ms 推进一包 | I2C1 DMA；每次 `step()` 不等待传输完成。 |

## OLED 与五向按键交互

- OLED 四行显示 `MODE`、`SPD`、`LINE`、`IMU`；字符缓存由 Driver 保存，显示
  数据以最多 8 byte 的 I2C1 DMA 包发送。
- CENTER 连续按住 500 ms 后，仍必须同时按住 UP 才可运行；该安全语义不因 UI
  功能改变。
- 停车时，LEFT/RIGHT 分别以 10 调整巡线 cruise，范围 `0..500`；DOWN 恢复
  默认值 180。运行时忽略这三个调参键，避免误操作改变电机输出。
- `Keypad` 在 Driver 层以 20 ms 去抖，故按键按下和释放最多额外引入 20 ms
  软件确认延迟；CENTER/UP 松开后仍由 `SafetyGate` 输出零轮速。

## 循线 Demo

该应用参考 `11_PID_car` 的“灰度加权位置 → 转向修正 → 左右轮差速”控制链，
但通过现有分层实现：

`LineSensorArray -> LineFollower -> DifferentialDrive -> SafetyGate -> MotorDriver`

- CENTER 连续按住 500 ms 后 armed，继续同时按住 CENTER + UP 才运行；
- 松开 CENTER 或 UP 时立即发布零轮速命令；
- 8 路灰度均未检测到线时，`LineFollower` 先保持最近转向 150 ms，再按最近
  误差方向低速搜索；总丢线时间达到 600 ms 后发布零命令。按键仍保持使能时，
  重新检测到线会自动恢复 Tracking；
- OLED 第三行显示 `TRACK/HOLD/SEARCH/LOST`，便于实车确认恢复阶段；
- 三灯状态：未使能=`两边亮/中间熄`，Tracking=`全亮`，
  Holding=`边灯亮/中间闪`，Searching=`中间亮/边灯闪`，Lost=`仅中间闪`；
  左右边灯的镜像组合含义相同；
- 默认巡航命令为 180，PID 为 `45/0/0`，均由 `LINE_FOLLOW_*` 宏配置；这些参数只完成编译验证，
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

## 阻塞审计

| 位置 | 最长影响 | 当前处置 |
| --- | --- | --- |
| `CarApplication::init()` 上电蜂鸣器 | 30 ms，仅启动 | 保留；电机已先停止。 |
| eMPL/DMP 初始化与 I2C0 轮询 | 单次事务 20 ms；仅初始化或 MPU data-ready 后 | 按当前需求保留 MPU 轮询；通信错误会停机。 |
| `BSP/i2c.cpp` 的 I2C0 FIFO 等待 | `timeoutMs`，默认 5 ms、DMP port 为 20 ms | 仅 MPU 使用；不在 ISR 执行。 |
| UART RX 命令 drain | 与已积压字节数成正比 | 当前无单轮预算；高吞吐命令流可能延长一次 superloop。 |
| OLED I2C1 刷新 | 无等待；单次仅启动 DMA | 已消除运行态 busy-wait；DMA timeout 为 2 ms，ISR 只置最终状态。 |

因此当前控制周期的主要残余风险是 MPU 轮询与无预算 UART RX drain，而不是 OLED。
若实机测得 5 ms 巡线周期抖动，下一步应将 MPU 轮询改为非阻塞状态机，或限制每轮
UART 解析字节数；两者都不应放入 ISR。

## 验证状态

- 已构建：2026-07-28 使用 SysConfig CLI 1.28.0 校验，并在临时 CCS 配置中以
  `APP_ACTIVE=0` 完成 Debug 全量构建；0 errors，唯一 warning 为 linker 对项目
  默认 `0x800` heap 的提示。Flash 使用 27,304 B，SRAM 使用 3,139 B；
- 待实机验证：灰度高/低电平极性、左右传感器顺序、电机正方向、急弯差速、
  丢线保持/搜索方向、600 ms 最大盲行距离、自动重捕获、CENTER + UP 松手停车
  时延、OLED DMA 刷新和按键去抖。

# HQuestionApplication

`HQuestionApplication` 是 H 题四程序的独立 App；它与 `CarApplication` 共用
Drivers、BSP 和 Middleware，但不复用其业务状态或按键调速逻辑。编译期选择由
`Application/app_selector.hpp` 的 `APP_ACTIVE` 决定，默认 `APP_H_QUESTION`。

## 程序与交互

| 程序 | 路径 |
| --- | --- |
| P1 | A→B |
| P2 | 巡线 500 cm |
| P3 | A→C→B→D→A |
| P4 | P3 连续 4 圈 |

- 菜单：LEFT/RIGHT 循环选择程序，CENTER 开始 3 秒倒计时；IMU 未就绪时禁止启动。
- 倒计时、运行、顶点暂停、完成或故障：CENTER 立即停车并回到菜单。
- 运行中不响应方向键，故程序切换必须先停机。

## 上电自检与诊断

`HQuestionApplication` 的启动流程是一个独立状态机。它先停车、关闭 `PB22`，点亮
LED1–LED3 并鸣叫 30 ms；鸣叫结束后三盏状态 LED 同时熄灭。随后 UART0（115200 8N1）
已经就绪，按顺序输出 `BOOT`、I2C0/I2C1 的 `0x08..0x77` ACK 列表及 timeout/error
统计。每一条日志都在 TX ring 与硬件 FIFO 排空后才进入下一阶段。

仅当扫描到 I2C0 `0x68` 才初始化 MPU；默认使用原始 MPU6050 加 complementary software
filter（可通过 `ATTITUDE_CONFIG_BACKEND` 切回 DMP 或 Kalman）。软件滤波分支的 IMU 成员为具体
`Mpu6050`（供 `calibrateGyroBias`），但采样路径经 `ImuReader::step()` 以 `ImuBackend&` 驱动
`AttitudeFilter`，不命名芯片；DMP 分支直接 `notifyDataReady` + `poll`，不经 `ImuReader`。仅当扫描到 I2C1 `0x3C`
才初始化 OLED。每个设备完成初始化便立即更新 LED：LED1 表示至少一个设备成功，LED2
表示两个设备都成功，LED3 始终熄灭。最终
日志为 `BOOT READY DEVICES=2` 或 `BOOT DEGRADED DEVICES=n`；在这之前不会运行菜单、
按键启动或电机控制。DMP 初始化仍是启动期唯一较长的同步操作。

`PB22` 用户 LED 每隔 1 s 点亮 50 ms，用于确认 superloop 仍在执行。该心跳只比较
`millis()` 时间戳，不使用阻塞延时，且在自检期间也保持运行。

## 控制和标定

`HQuestionRace` 是不依赖硬件的状态机。直线/对角段按编码器平均绝对 tick 计算距离，
并以相对 yaw 做 heading PID；弧线段使用 `LineFollower`。每段超过目标距离 80% 后
启用灰度辅助顶点判断：直线/对角需至少两路检测到黑线，弧线需全白，且提前触发需连续
两个 5 ms 控制周期；目标距离为最终兜底。

`RaceConfig` 的起始值使用 48 mm 轮径、28:1 减速比、13 线编码器和四倍频，即 1456
counts/轮，场地长度为直线 100 cm、弧线 125.6 cm、对角 128 cm。直线/对角目标 200 mm/s，
弧线目标 100 mm/s，任何一轮目标最低 80 mm/s。P2 是固定 500 cm 的巡线测试，仅由里程
完成，避免短暂丢线误触发终点。`WheelSpeedController` 每 50 ms 使用编码器闭环输出 PWM；
当前速度 PID 为 `Kp=2, Ki=1, Kd=0`，输出限幅为 700。这些均待实车标定。
特别是编码器正方向、tick→厘米比例、yaw 正负方向和灰度极性未验证前，禁止将路径完成
视为赛道验收。

默认使用 LEFT/RIGHT 选择程序、CENTER 启动。可在
`Application/h_question_application.cpp` 中设置 `H_QUESTION_AUTO_START_PROGRAM`：`0` 为菜单，
`1..4` 在上电自检及 IMU 就绪后自动启动对应要求。

## 验证状态

- 已构建：2026-07-26，CCS Debug 下 `APP_H_QUESTION` 与 `APP_LINE_FOLLOW`
  分支均构建成功。
- 待实机验证：启动日志顺序、四种 I2C 设备在/缺失组合、timeout 恢复、状态 LED
  表示，以及倒计时取消、运行急停、四路径顶点判断、P4 圈数、DMA OLED 刷新和所有
  标定参数。

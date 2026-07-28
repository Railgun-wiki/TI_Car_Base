# HQuestionApplication

`HQuestionApplication` 是 H 题四程序的独立 App；它与 `CarApplication` 共用
Drivers、BSP 和 Middleware，但不复用其业务状态或按键调速逻辑。编译期选择由
`Config/build_config.h` 的 `APP_ACTIVE` 决定，默认 `APP_H_QUESTION`；
`Application/app_selector.hpp` 仅保留兼容入口。

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
才初始化 OLED。每个设备完成初始化便立即更新 LED：一个设备成功时点亮一盏边灯，
两个设备都成功时点亮中间灯和一盏边灯；另一侧的镜像组合含义相同。最终
日志为 `BOOT READY DEVICES=2` 或 `BOOT DEGRADED DEVICES=n`；在这之前不会运行菜单、
按键启动或电机控制。DMP 初始化仍是启动期唯一较长的同步操作。

`PB22` 用户 LED 每隔 1 s 点亮 50 ms，用于确认 superloop 仍在执行。该心跳只比较
`millis()` 时间戳，不使用阻塞延时，且在自检期间也保持运行。

比赛状态灯使用以下规范：菜单=`边灯+中间灯亮`，倒计时=`边灯亮/中间灯闪`，
运行=`三灯全亮`，checkpoint=`中间灯亮/一盏边灯闪`，完成=`两边灯亮/中间灯熄`，
故障=`仅中间灯闪`。左右边灯互换不改变状态含义。

## 控制和标定

`HQuestionProgram` 是 Application 内不依赖硬件的状态机。直线/对角段按编码器平均绝对 tick 计算距离，
并以相对 yaw 做 heading PID；弧线段使用 `LineFollower`。每段超过目标距离 80% 后
启用灰度辅助顶点判断：直线/对角需至少两路检测到黑线，弧线需全白，且提前触发需连续
40 ms；目标距离为最终兜底。弧线在端点窗口之前丢线时，先保持最近转向 150 ms，
再低速搜索；总丢线时间达到 600 ms 仍未找回则进入 `Fault`。P2 不使用丢线端点，
所以持续丢线只会恢复或故障，不会提前完成 500 cm。

所有调试与标定参数集中在 `Config/vehicle_tuning.hpp`，应用选择、姿态后端和
BMI270 fusion 开关集中在 `Config/build_config.h`，均可通过编译选项 `-D`
覆盖。LED 状态组合集中在 `Config/status_led_config.hpp`。起始值使用 48 mm
轮径、28:1 减速比、13 线编码器和四倍频，即 1456
counts/轮，场地长度为直线 100 cm、弧线 125.6 cm、对角 128 cm。直线/对角目标 200 mm/s，
弧线目标 100 mm/s；正常 Tracking 和直线控制的正轮速目标最低 80 mm/s，零或负目标
钳为零，恢复阶段不应用该最低轮速。P2 是固定 500 cm 的巡线测试，仅由里程
完成，避免短暂丢线误触发终点。`WheelSpeedController` 每 50 ms 使用编码器闭环输出 PWM；
当前速度 PID 为 `Kp=2, Ki=1, Kd=0`，输出限幅为 250。这些均待实车标定。
特别是编码器正方向、tick→厘米比例、yaw 正负方向和灰度极性未验证前，禁止将路径完成
视为赛道验收。

默认使用 LEFT/RIGHT 选择程序、CENTER 启动。可在
`Config/vehicle_tuning.hpp` 中设置 `H_QUESTION_AUTO_START_PROGRAM`：`0` 为菜单，
`1..4` 在上电自检及 IMU 就绪后自动启动对应要求。

## 验证状态

- 已构建：2026-07-28，SysConfig CLI 1.28.0 校验通过；CCS Debug 下默认
  `APP_H_QUESTION` 与临时 `APP_ACTIVE=0` 普通循线分支均完成全量构建。两者均
  0 errors，唯一 warning 为 linker 对默认 `0x800` heap 的提示。H题 Flash 使用
  30,680 B，SRAM 使用 3,011 B。
- 待实机验证：启动日志顺序、四种 I2C 设备在/缺失组合、timeout 恢复、状态 LED
  表示，以及倒计时取消、运行急停、四路径顶点判断、P4 圈数、DMA OLED 刷新和所有
  标定参数。

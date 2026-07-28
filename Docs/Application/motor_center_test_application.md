# Center Motor Test Application

`APP_MOTOR_CENTER_TEST` 是双电机台架验证应用，当前由
由 `Config/build_config.h` 的 `APP_ACTIVE` 选中。它不初始化 IMU、OLED、循迹或 UART
业务逻辑；系统初始化仍会将两路 PWM 和电机方向设为安全停止状态。

按住 TM2027 五向按键的 Center 超过 20 ms 消抖时间后，应用向两个轮子发送
`{500, 500}`，即逻辑正方向的 50% PWM 命令。松开 Center 会立刻调用 `stop()`，
将命令置为 `{0, 0}`。这是 hold-to-run 设计，避免一次按键后持续驱动车辆。

用户 LED 在电机运行时点亮；待机时两边状态灯亮且中间灯熄，按住 Center
运行时三灯全亮。首次实机测试应使车轮
悬空，并确认两侧物理前进方向；如一侧反转，应在电机驱动/接线层校正，不应在测试 App
中使用相反的逻辑命令掩盖问题。

# IMU OLED Test Application

`APP_IMU_OLED_TEST` 是针对 `MPU6050 + ATTITUDE_BACKEND_MAHONY` 的独立硬件
验证应用。它不启动电机、循迹或 UART 控制逻辑，只轮询 MPU6050、运行 Mahony
姿态解算并以低优先级 DMA 刷新 OLED。

由 `Config/build_config.h` 的 `APP_ACTIVE` 选择该 App。OLED 以完整 `128x64` 的八行
文本区域一次显示 IMU 状态、`CAL` 上电零偏校准状态、roll/pitch/yaw、`AX`/`AY`/`AZ`
（单位 g）以及 `GX`/`GY`/`GZ`（单位 deg/s）。

显示值每 500 ms 刷新；OLED 传输每 2 ms 调用一次 `service()`，以遵守 I2C1 DMA
分段、非 ISR 的约束。采样为 100 Hz 定时轮询，而非依赖 `MPU6050_INT`，因此可单独
验证 I2C 原始读数与 Mahony 输出。OLED 在 MPU 预热前初始化并显示 `KEEP STILL` 状态。
为减小 MPU6050 上电热漂，测试 App 会先静置预热 2 秒，
再执行 1000 次、间隔 5 ms 的陀螺零偏校准（约 5 秒）；期间必须保持车辆静止，`CAL:OK`
表示该过程完成。Mahony 重力反馈使用 SJTU-AuTop 同款
`alpha=0.3` 加速度一阶低通；界面仍显示未滤波的原始加速度，便于观察振动。

UART0 同时以 20 Hz 输出 VOFA+ FireWater 命名帧：`roll`、`pitch`、`yaw`、纯加速度重力角
`roll_acc`/`pitch_acc`、`ax..az`、`gx..gz`、`stationary`、`cal` 和 `tx_drop`。在持续静止
1 秒（加速度模长 0.98–1.02 g，
三轴校准后角速度均不超过 1 °/s）后，测试 App 会以 20 秒时间常数慢速更新 gyro bias，
用于跟踪温漂；只要检测到运动便立即冻结更新。

启动期间三灯闪烁；无设备就绪时全熄，一个设备就绪时点亮一盏边灯，IMU 与 OLED
均就绪时点亮中间灯和一盏边灯。左右边灯镜像等价，不再用固定边灯区分设备。
构建成功仅证明软件集成正确；需在实机上验证 MPU 安装轴向、I2C 上拉和 OLED 显示内容。

# IMU OLED Test Application

`APP_IMU_OLED_TEST` 是针对 `MPU6050 + ATTITUDE_BACKEND_MAHONY` 的独立硬件
验证应用。它不启动电机、循迹或 UART 控制逻辑，只轮询 MPU6050、运行 Mahony
姿态解算并以低优先级 DMA 刷新 OLED。

`Application/app_selector.hpp` 当前默认选择该 App。OLED 以完整 `128x64` 的八行
文本区域一次显示 IMU 状态、`CAL` 上电零偏校准状态、roll/pitch/yaw、`AX`/`AY`/`AZ`
（单位 g）以及 `GX`/`GY`/`GZ`（单位 deg/s）。

显示值每 500 ms 刷新；OLED 传输每 2 ms 调用一次 `service()`，以遵守 I2C1 DMA
分段、非 ISR 的约束。采样为 100 Hz 定时轮询，而非依赖 `MPU6050_INT`，因此可单独
验证 I2C 原始读数与 Mahony 输出。启动阶段会执行约 1 秒的陀螺零偏校准，期间必须保持
车辆静止；`CAL:OK` 表示该过程完成。

状态 LED2 表示 IMU 初始化与运行状态，LED3 表示 OLED 初始化与运行状态。构建成功仅证明
软件集成正确；需在实机上验证 MPU 安装轴向、I2C 上拉和 OLED 显示内容。

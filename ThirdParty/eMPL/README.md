# eMPL vendor boundary

本目录保存 InvenSense Motion Driver 的 MPU6050 DMP firmware、memory table 与 FIFO parser。工程代码不应在此目录加入板级 I2C、GPIO 或应用 helper；MSPM0 适配统一位于 `BSP/MPU6050/mpu6050_empl_port.*`。

## Local portability patch

- vendor 源文件统一转换为 UTF-8，便于现代工具链处理；
- 移除硬编码 `board.h`、`bsp_mpu6050.h` 与 MSP430 target；
- 增加 `EMPL_TARGET_MSPM0`，仅映射 `i2c_read/write`、`delay_ms`、`get_ms` 和 no-op log；
- 使用局部 `EMPL_SENSOR_MPU6050` target 宏，避免上游 `MPU6050` 宏与工程公开 `MPU6050` typedef 冲突；
- 由 `attitude_backend_config.h` 派生 MSPM0 target；未选择 DMP 时 vendor translation unit 编译为空，便于 Keil/CCS 将全部源码保留在同一 target；
- 禁用原包末尾追加的板级 `mpu_dmp_init/get_data` helper，改由工程 driver 封装；
- DMP feature bit 始终引用上游 header，不在工程内重复定义。

升级 vendor 包时，应先恢复官方原始文件，再逐项重放上述 patch，避免把平台实现混入第三方源码。

## Source note

本目录来自工程采用的 Keil DMP 发布包，该包未附带单独的 `License.txt`。工程保留 vendor 源文件原有版权头，不补写或猜测额外许可文本。

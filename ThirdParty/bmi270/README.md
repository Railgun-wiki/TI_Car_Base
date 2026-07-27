# BMI270 vendor boundary

本目录保存 BMI270 的 vendor 配置 RAM blob (`bmi270_config_file.h`),8192 字节,在 `Bmi270::begin()` 中 burst-write 到 `INIT_DATA (0x5E)`。工程代码不应修改本目录文件;BMI270 的板级适配与寄存器序列统一位于 `Drivers/bmi270.{hpp,cpp}`。

## 来源

`gitcode.com/Benjamin123456/bmi270`(`Extend/inc/BMI270_config.h`)。该仓库是 BMI270 的 STM32 HAL/标准库示例。本工程仅引入 config blob;寄存器初始化序列与 I2C 适配为本工程自行编写,不使用该仓库的 SPI 驱动代码。

## 边界

- **禁止修改 blob 的任何字节**。它是 Bosch 供应商固件配置数据,逐字节写入芯片 INIT RAM。
- 头文件 include guard 已从上游 `BMI270_CONFIG_H` 重命名为 `BMI270_CONFIG_FILE_H`,避免与本工程未来可能的配置头冲突;数组内容未改动。

## 升级

若需更新 blob 版本,用新版 Bosch 官方 `bmi270_config_file` 整体替换本文件数组内容,不混入平台代码。

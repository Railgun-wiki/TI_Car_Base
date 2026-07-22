# 开发者文档

本目录说明工程结构、开发约束和验证方法。功能模块的实现完成后，应在对应文档中补充接口、单位、时序预算及实机验证证据。

## 文档入口

- [maintenance_guide.md](maintenance_guide.md)：当前代码的维护、标定、构建和验收手册。
- [firmware_plan.md](firmware_plan.md)：当前推荐的固件演进方案。

模块接口说明位于上级 `Docs/Application`、`Docs/BSP`、`Docs/Drivers` 与 `Docs/Middlewares`，不在本目录重复维护。

## 基本原则

- `TI_Car_Base.syscfg` 是引脚、时钟、PWM、I2C、UART 和 GPIO 的唯一配置来源。
- `BSP` 是唯一可以直接依赖 `ti_msp_dl_config.h` 和 DriverLib 的层。
- 算法模块必须能脱离开发板进行 host-side 单元测试。
- 任何硬件通信、传感器方向、控制参数和时间预算都必须实机验证。

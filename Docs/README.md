# TI_Car_Base 文档

本文档目录记录工程决策、模块契约和实机验证结论；它不替代 `TI_Car_Base.syscfg`。引脚、时钟和外设参数始终以 SysConfig 为唯一来源。

## 入口

- [Application/car_application.md](Application/car_application.md)：应用初始化、调度和台架 Demo。
- [BSP/platform.md](BSP/platform.md)：DriverLib/SysConfig 板级实现与 ISR/I2C 约束。
- [Drivers/devices.md](Drivers/devices.md)：设备接口、MPU 原始与 DMP 驱动边界。
- [Middlewares/control.md](Middlewares/control.md)：控制、速度估算与姿态后端。
- [Developer/maintenance_guide.md](Developer/maintenance_guide.md)：维护、构建、姿态切换、标定和上板验收的操作手册。
- [Developer/firmware_plan.md](Developer/firmware_plan.md)：小车固件的分层、静态 OOP、无 RTOS 调度与演进方案。
- [Developer/api_implementation_reference.md](Developer/api_implementation_reference.md)：按类与 BSP 接口说明当前实现原理、调用契约、状态返回与扩展接入方式。

## 状态标记

- **已配置**：已在 SysConfig 或源码中配置并通过构建。
- **待实机验证**：尚未以实际车板、模块与接线确认；构建成功不代表硬件行为已验证。
- **规划**：架构方向，不表示当前已经实现。

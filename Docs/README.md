# TI_Car_Base 文档

本文档目录记录工程决策、模块契约和实机验证结论；它不替代 `TI_Car_Base.syscfg`。引脚、时钟和外设参数始终以 SysConfig 为唯一来源。

## 入口

- [Developer/firmware_plan.md](Developer/firmware_plan.md)：小车固件的分层、静态 OOP、无 RTOS 调度与演进方案。

## 状态标记

- **已配置**：已在 SysConfig 或源码中配置并通过构建。
- **待实机验证**：尚未以实际车板、模块与接线确认；构建成功不代表硬件行为已验证。
- **规划**：架构方向，不表示当前已经实现。

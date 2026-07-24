# TI_Car_Base 文档

本文档目录记录工程决策、模块契约和实机验证结论；它不替代 `TI_Car_Base.syscfg`。引脚、时钟和外设参数始终以 SysConfig 为唯一来源。

## 文档树

| 目录 | 主入口 | 内容边界 |
| --- | --- | --- |
| `Application/` | [car_application.md](Application/car_application.md) | 当前应用初始化、调度、循线 Demo 和 VOFA+ 行为。 |
| `BSP/` | [platform.md](BSP/platform.md) | DriverLib/SysConfig 板级实现、ISR 与通信约束。 |
| `Drivers/` | [devices.md](Drivers/devices.md) | 设备语义接口、MPU 原始与 DMP 驱动边界。 |
| `Middlewares/` | [control.md](Middlewares/control.md) | PID、巡线、姿态、telemetry 与 VOFA+ 协议。 |
| `Developer/` | [README.md](Developer/README.md) | 维护、API、架构决策、验证与演进文档索引。 |
| `Datasheet/` | [电赛天猛星拓展板v2_copy.md](Datasheet/电赛天猛星拓展板v2_copy.md) | 硬件资料摘录和待核对项；不替代 SysConfig。 |

## 阅读路径

- 使用、调参和上板验收：从
  [maintenance_guide.md](Developer/maintenance_guide.md) 开始。
- 阅读当前源码契约：查看
  [api_implementation_reference.md](Developer/api_implementation_reference.md)。
- 理解架构选择与后续演进：查看
  [firmware_plan.md](Developer/firmware_plan.md)。

## 维护规则

- 当前行为和公开接口写入对应模块文档或 API 参考，不写入未来方案。
- 可操作步骤、参数范围和验收顺序只在维护手册维护。
- 尚未实现的方向只写入固件方案，并明确标记为“规划”。
- 修改目录、公开接口、协议、默认参数或验证状态时，同步更新本索引和受影响文档。

## 状态标记

- **已构建**：源码与 SysConfig 已通过 CCS 构建，不代表硬件行为正确。
- **已配置**：已写入 SysConfig 或源码，但不必然已经构建或实机确认。
- **待实机验证**：尚未以实际车板、模块与接线确认；构建成功不代表硬件行为已验证。
- **规划**：架构方向，不表示当前已经实现。

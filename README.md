# TI_Car_Base

MSPM0G3507 差速小车的无 RTOS C++ 固件。工程采用静态组合：

`Application -> Middlewares -> Drivers / BSP -> SysConfig / DriverLib`

它默认使用 MPU6050 DMP；可在
`Middlewares/attitude_backend_config.h` 编译期切换为互补或
Kalman 软件滤波。引脚、时钟和外设实例以 `TI_Car_Base.syscfg` 为唯一来源。

完整导航见 [文档树](Docs/README.md)。开发、标定、上板验收和故障定位请从
[维护手册](Docs/Developer/maintenance_guide.md) 开始；当前接口见
[API 参考](Docs/Developer/api_implementation_reference.md)，架构决策见
[固件方案](Docs/Developer/firmware_plan.md)。

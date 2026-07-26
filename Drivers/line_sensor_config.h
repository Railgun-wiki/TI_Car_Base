#ifndef LINE_SENSOR_CONFIG_H
#define LINE_SENSOR_CONFIG_H

// 灰度传感器极性配置。
//
// 八路灰度模块的指示 LED 默认“贴白亮、贴黑灭”，但 OUT 引脚电平是否与
// 指示灯同相取决于传感器板设计，必须实机确认。本宏描述固件侧的约定：
//   LINE_SENSOR_LINE_IS_HIGH = 1：OUT 高电平 = 压在线上（默认）。
//   LINE_SENSOR_LINE_IS_HIGH = 0：OUT 低电平 = 压在线上。
//
// 切换跑道（黑线白底 / 白线黑底）或更换传感器板时，按实测电平在此翻转，
// 不要改动 line_sensor_array.cpp 的权重逻辑。默认值匹配当前固件历史行为。
#ifndef LINE_SENSOR_LINE_IS_HIGH
#define LINE_SENSOR_LINE_IS_HIGH 1
#endif

#if LINE_SENSOR_LINE_IS_HIGH != 0 && LINE_SENSOR_LINE_IS_HIGH != 1
#error "LINE_SENSOR_LINE_IS_HIGH must be 0 or 1"
#endif

#endif

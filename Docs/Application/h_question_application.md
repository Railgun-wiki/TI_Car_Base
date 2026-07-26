# HQuestionApplication

`HQuestionApplication` 是 H 题四程序的独立 App；它与 `CarApplication` 共用
Drivers、BSP 和 Middleware，但不复用其业务状态或按键调速逻辑。编译期选择由
`Application/app_selector.hpp` 的 `APP_ACTIVE` 决定，默认 `APP_H_QUESTION`。

## 程序与交互

| 程序 | 路径 |
| --- | --- |
| P1 | A→B |
| P2 | A→B→C→D→A |
| P3 | A→C→B→D→A |
| P4 | P3 连续 4 圈 |

- 菜单：LEFT/RIGHT 循环选择程序，CENTER 开始 3 秒倒计时；IMU 未就绪时禁止启动。
- 倒计时、运行、顶点暂停、完成或故障：CENTER 立即停车并回到菜单。
- 运行中不响应方向键，故程序切换必须先停机。

## 控制和标定

`HQuestionRace` 是不依赖硬件的状态机。直线/对角段按编码器平均绝对 tick 计算距离，
并以 DMP 相对 yaw 做 heading PID；弧线段使用 `LineFollower`。每段超过目标距离
60% 后启用灰度辅助顶点判断，目标距离为最终兜底。

`RaceConfig` 的轮径、减速比、编码器分辨率、场地长度、航向角、速度和 heading PID
是起始参数，均待实车标定。特别是编码器正方向、tick→厘米比例、yaw 正负方向和
灰度极性未验证前，禁止将路径完成视为赛道验收。

## 验证状态

- 已构建：2026-07-26，CCS Debug 下 `APP_H_QUESTION` 与 `APP_LINE_FOLLOW`
  分支均构建成功。
- 待实机验证：倒计时取消、运行急停、四路径顶点判断、P4 圈数、DMA OLED 刷新及所有
  标定参数。

# 电赛天猛星拓展板v2_copy：U8 引脚连接关系

原理图页面：`Schematic1 / P1`  
模块：`U8`（天猛星 MSPM0G3507 开发板）

下表记录当前原理图中 `U8` 引脚、网络名及同网对端。`J5`–`J8` 为四个 1x20P、2.54 mm 扩展排针；`—` 表示未检出导线或同网对端。

## 固件接口映射

本节是固件开发入口：它将原理图网络转换为 `TI_Car_Base.syscfg` 的外设与 GPIO 实例。引脚、电气属性和时钟配置的唯一可执行来源仍然是 SysConfig；本表用于硬件—软件追溯。

### 已配置的控制接口

| 功能 | 原理图网络 | U8 引脚 | SysConfig 实例 | 配置 | 固件约束 |
| --- | --- | --- | --- | --- | --- |
| 左电机 PWM（PWMA） | `PWMA` | `PA28` | `PWM_MOTOR` / `TIMA1 CCP0` | 10 kHz，初始 0% | 初始化后显式启动 `DL_TimerA_startCounter(PWM_MOTOR_INST)`。 |
| 右电机 PWM（PWMB） | `PWMB` | `PB20` | `PWM_MOTOR_B` / `TIMG12 CCP0` | 10 kHz，初始 0% | 与 PWMA 不能共享 timer；初始化后显式启动 `DL_TimerG_startCounter(PWM_MOTOR_B_INST)`。 |
| A 通道方向 | `AIN1` / `AIN2` | `PA13` / `PB26` | `GPIO_MOTOR_DIR` | output，初始 low | 换向时先停止 PWM，再经死区更新方向。 |
| B 通道方向 | `BIN1` / `BIN2` | `PB9` / `PB7` | `GPIO_MOTOR_DIR` | output，初始 low | 不允许在 PWM 有效时直接翻转。 |
| IMU I2C data | `ICM_SDA` | `PA0` | `I2C_MPU6050` / `I2C0 SDA` | controller，400 kHz | 需要外部 3.3 V 上拉。 |
| IMU I2C clock | `ICM_SCLK` | `PA1` | `I2C_MPU6050` / `I2C0 SCL` | controller，400 kHz | I2C FIFO 为 8 bytes。 |
| IMU data-ready | `MPU6050_INT` | `PB11` | `GPIO_MPU6050_DATA_READY.MPU6050_INT` | input，上升沿 GPIO interrupt | ISR 仅清标志并置 `imu_due`；I2C 读取在主循环。 |
| OLED I2C clock | `OLED_SCL` | `PB2` | `I2C_OLED` / `I2C1 SCL` | controller，400 kHz | 使用独立硬件 I2C1，不占用 MPU 的 I2C0；需要外部 3.3 V 上拉。 |
| OLED I2C data | `OLED_SDA` | `PB3` | `I2C_OLED` / `I2C1 SDA` | controller，400 kHz | 与 SCL 构成 I2C1 总线；刷新仅在低优先级主循环执行。 |
| telemetry TX/RX | `UART0_TX` / `UART0_RX` | `PA10` / `PA11` | `UART_CONSOLE` / `UART0` | 115200 8N1 | 在 LaunchPad 上可使用 XDS110 backchannel；车板端按 CN5 接线。 |
| 扩展 UART1 | `UART1_TX` / `UART1_RX` | `PA8` / `PA9` | `UART1_MODULE` / `UART1` | 115200 8N1，FIFO | CN6。 |
| 扩展 UART2 | `UART2_TX` / `UART2_RX` | `PB15` / `PB16` | `UART2_MODULE` / `UART2` | 115200 8N1，FIFO | CN7。 |
| 扩展 UART3 | `UART3_TX` / `UART3_RX` | `PA26` / `PB13` | `UART3_MODULE` / `UART3` | 115200 8N1，FIFO | CN8。 |
| 左编码器 | `E1A` / `E1B` | `PB23` / `PB12` | `GPIO_ENCODER` | input，双边沿 GPIO interrupt | 软件正交解码；非硬件 QEI。 |
| 右编码器 | `E2A` / `E2B` | `PB4` / `PB5` | `GPIO_ENCODER` | input，双边沿 GPIO interrupt | 软件正交解码；非硬件 QEI。 |
| 五向按键 | UP/LEFT/DOWN/RIGHT/CENTER | `PA14` / `PA15` / `PA17` / `PB25` / `PB24` | `GPIO_KEY` | input，internal pull-up | TM2027 的 Common 接地；按下为 low；由 Driver polling + debounce。 |
| LED1/2/3 | `LED1` / `LED2` / `LED3` | `PB14` / `PB18` / `PA22` | `GPIO_LED` | push-pull output，初始 high | LED 阳极上拉至 3.3 V，GPIO low 点亮。 |
| 用户 LED | `USER_LED` | `PB22` | `GPIO_LED.USER_LED` | push-pull output，初始 low | 用户确认高电平点亮。 |
| 有源蜂鸣器 | `BUZZER` | `PA21` | `GPIO_BUZZER.BUZZER` | push-pull output，初始 high | S8550 PNP 高边驱动；GPIO low 开启、high 关闭；不使用 PWM。 |

### 八路灰度循线

`H6` 的 `C1`–`C8` 是左右各四路灰度模块输出。固件将它们统一配置为普通 GPIO input，不启用 GPIO interrupt；信号的黑/白有效电平尚待实机确认，必须在 `LineSensorArray` 驱动层统一反相，不能散落在巡线算法中。

| 逻辑通道 | 原理图网络 | U8 引脚 | SysConfig GPIO pin | 备注 |
| --- | --- | --- | --- | --- |
| `C1` | `C1` | `PA31` | `GPIO_LINE_SENSOR.C1` | H6.8 |
| `C2` | `C2` | `PA12` | `GPIO_LINE_SENSOR.C2` | H6.7 |
| `C3` | `C3` | `PB8` | `GPIO_LINE_SENSOR.C3` | H6.6 |
| `C4` | `C4` | `PA27` | `GPIO_LINE_SENSOR.C4` | 用户确认的灰度输入；自动提取表仍标为 `ADC_read`，须以实机复核。 |
| `C5` | `C5` | `PB0` | `GPIO_LINE_SENSOR.C5` | 用户确认的灰度输入；自动提取表仍标为 `AD2`，须以实机复核。 |
| `C6` | `C6` | `PA30` | `GPIO_LINE_SENSOR.C6` | H6.3 |
| `C7` | `C7` | `PB21` | `GPIO_LINE_SENSOR.C7` | 用户确认的灰度输入；自动提取表仍标为 `AD0`，须以实机复核。 |
| `C8` | `C8` | `PB10` | `GPIO_LINE_SENSOR.C8` | H6.1 |

### 未启用的板载外设

以下器件存在于开发板，但当前固件不为它们生成 SysConfig 外设实例。它们不是遗漏：禁用是为了保留小车电机、灰度与启动行为的确定性。

| 板载器件 | 网络与引脚 | 当前状态 | 原因与重新启用条件 |
| --- | --- | --- | --- |
| W25Q128JVSIQ SPI Flash | CS `PB6`、MISO `PB7`、MOSI `PB8`、CLK `PB9` | 未配置 SPI、GPIO CS 或 Flash 驱动 | `PB7`/`PB8`/`PB9` 已分别用于 `BIN2`、`C3`、`BIN1`。保留完整 TB6612 与八路灰度功能优先；仅在硬件改线或放弃对应小车信号后才能启用。 |
| BSL 按键 | `PA18`，端口下拉，按下为 3.3 V | 未配置 GPIO | 该键用于 BSL/启动维护，用户当前不需要其作为应用输入。若要启用，需先确认复位/启动流程不会将其误判为 BSL 请求，再配置为 input + pull-down。 |
| 板载用户 KEY | `PB21`，按下接地 | 未配置 GPIO | 与灰度通道 `C7` 共用引脚；当前选择保留灰度输入。若需此键，必须改线或放弃 `C7`。 |

### 未配置或待确认项

| 项目 | 原理图事实 | 处理决定 |
| --- | --- | --- |
| MPU/ICM data-ready | 用户确认 MPU 已接中断线；使用 `PB11`。 | 已配置为上升沿 GPIO interrupt。需在实机确认 MPU 寄存器中 data-ready 的有效极性；若为 low active，改为 falling edge。 |
| OLED I2C | MSPM0G3507 具有 `I2C0` 和 `I2C1`；`PB2/PB3` 可由 `I2C1` 使用。 | 已配置 `I2C_OLED` / `I2C1` 为 `PB2/PB3` 400 kHz；MPU 保持 `I2C_MPU6050` / `I2C0` 的 `PA0/PA1`。两条硬件总线独立，无资源冲突；OLED 上拉必须至 3.3 V。 |
| 板载 Flash / BSL Key / 用户 KEY | 未启用器件和引脚冲突详见上表。 | W25Q128 与 PB21 KEY 分别受小车引脚复用限制；PA18 BSL Key 因用户不需要且应保持启动维护用途而禁用。TM2027 五向按键不受影响，保持已配置。 |
| 灰度电平 | 原理图未说明黑线/白底对应高低电平。 | 实机采样确认后在 Driver 层配置 `activeLow`，不直接改巡线策略。 |
| C4/C5/C7 网络名 | 自动提取结果分别为 `ADC_read`、`AD2`、`AD0`，但用户已确认其属于八路灰度输入。 | SysConfig 按用户确认配置为 GPIO input；上电前须用万用表/逻辑分析确认这些网络未被 U1 的模拟功能驱动。 |

## 已连接到其他模块或接口

| U8 针号 | 信号 | 网络 | 对端 |
| ---: | --- | --- | --- |
| 1 | GND | GND | SW6.4 |
| 2 | GND | GND | SW6.4 |
| 3 | A00 | ICM_SDA | U10.4 |
| 4 | A28 | PWMA | U14.16 |
| 5 | A01 | ICM_SCLK | U10.3 |
| 6 | A31 | C1 | H6.8 |
| 7 | A08 | UART1_TX | CN6.3 |
| 8 | B04 | E2A | U9.4 |
| 9 | A09 | UART1_RX | CN6.4 |
| 10 | B05 | E2B | U9.3 |
| 11 | B15 | UART2_TX | CN7.3 |
| 12 | A10 | UART0_TX | CN5.3 |
| 13 | B16 | UART2_RX | CN7.4 |
| 14 | A11 | UART0_RX | CN5.4 |
| 15 | B02 | OLED_SCL | OLED2.3 |
| 16 | B12 | E1B | U12.3 |
| 17 | B03 | OLED_SDA | OLED2.4 |
| 18 | B13 | UART3_RX | CN8.4 |
| 21 | GND | GND | SW6.4 |
| 23 | GND | GND | SW6.4 |
| 26 | B07 | BIN2 | U14.11 |
| 27 | B08 | C3 | H6.6 |
| 28 | B09 | BIN1 | U14.12 |
| 29 | A12 | C2 | H6.7 |
| 30 | A13 | AIN1 | U14.14 |
| 31 | B23 | E1A | U12.4 |
| 32 | B26 | AIN2 | U14.15 |
| 37 | GND | GND | SW6.4 |
| 39 | GND | GND | SW6.4 |
| 41 | GND | GND | SW6.4 |
| 43 | GND | GND | SW6.4 |
| 48 | B14 | LED1 | LED1.1 |
| 49 | B10 | C8 | H6.1 |
| 51 | B00 | AD2 | U1.4 |
| 53 | A30 | C6 | H6.3 |
| 54 | GND | GND | SW6.4 |
| 55 | B21 | AD0 | U1.2 |
| 57 | GND | GND | SW6.4 |
| 59 | GND | GND | SW6.4 |
| 64 | A21 | BUZZER | R2.2 |
| 66 | B18 | LED2 | LED2.1 |
| 68 | A17 | KEY_DOWN | SW6.3 |
| 69 | A14 | KEY_UP | SW6.1 |
| 70 | A15 | KEY_LEFT | SW6.2 |
| 71 | B20 | PWMB | U14.10 |
| 72 | A22 | LED3 | LED3.1 |
| 73 | B25 | KEY_RIGHT | SW6.5 |
| 74 | B24 | KEY_CENTER | SW6.6 |
| 77 | A27 | ADC_read | U1.6 |
| 78 | A26 | UART3_TX | CN8.3 |
| 79 | GND | GND | SW6.4 |
| 80 | GND | GND | SW6.4 |

## 仅引出至 20P 扩展排针

| U8 针号 | 信号 | 网络 | 扩展排针 |
| ---: | --- | --- | --- |
| 25 | B06 | B06_spare | J6.8 |
| 34 | A29 | A29_spare | J5.17 |
| 46 | A18 | A18_spare | J8.18 |
| 47 | B11 | B11_spare | J7.4 |
| 50 | A07 | A07_spare | J8.16 |
| 52 | B01 | B01_spare | J8.15 |
| 56 | B22 | B22_spare | J8.13 |
| 58 | 5V | +5V | J8.12 |
| 61 | A02 | A02_spare | J7.11 |
| 62 | A23 | A23_spare | J8.10 |
| 63 | B19 | B19_spare | J7.12 |
| 65 | B17 | B17_spare | J7.13 |
| 67 | A16 | A16_spare | J7.14 |
| 75 | A25 | A25_spare | J7.18 |
| 76 | A24 | A24_spare | J8.3 |

## 当前未检出对端

| U8 针号 | 信号 | 网络 | 对端 |
| ---: | --- | --- | --- |
| 19 | DIO | — | — |
| 20 | CLK | — | — |
| 22 | 3V3 | — | — |
| 24 | 5V | — | — |
| 33 | B27 | — | — |
| 35 | AGND | — | — |
| 36 | CHIP | — | — |
| 38 | 3V3 | — | — |
| 40 | 5V | — | — |
| 42 | 5V | — | — |
| 44 | 3V3 | — | — |
| 45 | RST | — | — |
| 60 | 3V3 | — | — |

## 连接件/模块端口反查 U8

下表按连接件或模块端口反查 U8。仅列出与 U8 有网络关系的端口；同一 GND 端口连接多个 U8 地脚时合并显示。

| 连接件或模块 | 端口 | 网络 | U8 针号 | U8 信号 |
| --- | --- | --- | ---: | --- |
| CN5 | 3 | UART0_TX | 12 | A10 |
| CN5 | 4 | UART0_RX | 14 | A11 |
| CN6 | 3 | UART1_TX | 7 | A08 |
| CN6 | 4 | UART1_RX | 9 | A09 |
| CN7 | 3 | UART2_TX | 11 | B15 |
| CN7 | 4 | UART2_RX | 13 | B16 |
| CN8 | 3 | UART3_TX | 78 | A26 |
| CN8 | 4 | UART3_RX | 18 | B13 |
| H6 | 1 | C8 | 49 | B10 |
| H6 | 3 | C6 | 53 | A30 |
| H6 | 6 | C3 | 27 | B08 |
| H6 | 7 | C2 | 29 | A12 |
| H6 | 8 | C1 | 6 | A31 |
| J5 | 1 | GND | 1, 2, 21, 23, 37, 39, 41, 43, 54, 57, 59, 79, 80 | GND |
| J5 | 2 | PWMA | 4 | A28 |
| J5 | 3 | C1 | 6 | A31 |
| J5 | 4 | E2A | 8 | B04 |
| J5 | 5 | E2B | 10 | B05 |
| J5 | 6 | UART0_TX | 12 | A10 |
| J5 | 7 | UART0_RX | 14 | A11 |
| J5 | 8 | E1B | 16 | B12 |
| J5 | 9 | UART3_RX | 18 | B13 |
| J5 | 13 | BIN2 | 26 | B07 |
| J5 | 14 | BIN1 | 28 | B09 |
| J5 | 15 | AIN1 | 30 | A13 |
| J5 | 16 | AIN2 | 32 | B26 |
| J5 | 17 | A29_spare | 34 | A29 |
| J6 | 1, 2, 9, 10, 20 | GND | 1, 2, 21, 23, 37, 39, 41, 43, 54, 57, 59, 79, 80 | GND |
| J6 | 5 | E1A | 31 | B23 |
| J6 | 6 | C2 | 29 | A12 |
| J6 | 7 | C3 | 27 | B08 |
| J6 | 8 | B06_spare | 25 | B06 |
| J6 | 12 | OLED_SDA | 17 | B03 |
| J6 | 13 | OLED_SCL | 15 | B02 |
| J6 | 14 | UART2_RX | 13 | B16 |
| J6 | 15 | UART2_TX | 11 | B15 |
| J6 | 16 | UART1_RX | 9 | A09 |
| J6 | 17 | UART1_TX | 7 | A08 |
| J6 | 18 | ICM_SCLK | 5 | A01 |
| J6 | 19 | ICM_SDA | 3 | A00 |
| J7 | 1, 2, 9, 10, 20 | GND | 1, 2, 21, 23, 37, 39, 41, 43, 54, 57, 59, 79, 80 | GND |
| J7 | 4 | B11_spare | 47 | B11 |
| J7 | 5 | C8 | 49 | B10 |
| J7 | 6 | AD2 | 51 | B00 |
| J7 | 7 | C6 | 53 | A30 |
| J7 | 8 | AD0 | 55 | B21 |
| J7 | 11 | A02_spare | 61 | A02 |
| J7 | 12 | B19_spare | 63 | B19 |
| J7 | 13 | B17_spare | 65 | B17 |
| J7 | 14 | A16_spare | 67 | A16 |
| J7 | 15 | KEY_UP | 69 | A14 |
| J7 | 16 | PWMB | 71 | B20 |
| J7 | 17 | KEY_RIGHT | 73 | B25 |
| J7 | 18 | A25_spare | 75 | A25 |
| J7 | 19 | ADC_read | 77 | A27 |
| J8 | 1, 14 | GND | 1, 2, 21, 23, 37, 39, 41, 43, 54, 57, 59, 79, 80 | GND |
| J8 | 2 | UART3_TX | 78 | A26 |
| J8 | 3 | A24_spare | 76 | A24 |
| J8 | 4 | KEY_CENTER | 74 | B24 |
| J8 | 5 | LED3 | 72 | A22 |
| J8 | 6 | KEY_LEFT | 70 | A15 |
| J8 | 7 | KEY_DOWN | 68 | A17 |
| J8 | 8 | LED2 | 66 | B18 |
| J8 | 9 | BUZZER | 64 | A21 |
| J8 | 10 | A23_spare | 62 | A23 |
| J8 | 12 | +5V | 58 | 5V |
| J8 | 13 | B22_spare | 56 | B22 |
| J8 | 15 | B01_spare | 52 | B01 |
| J8 | 16 | A07_spare | 50 | A07 |
| J8 | 17 | LED1 | 48 | B14 |
| J8 | 18 | A18_spare | 46 | A18 |
| OLED2 | 3 | OLED_SCL | 15 | B02 |
| OLED2 | 4 | OLED_SDA | 17 | B03 |
| U1 | 2 | AD0 | 55 | B21 |
| U1 | 4 | AD2 | 51 | B00 |
| U1 | 6 | ADC_read | 77 | A27 |
| U9 | 3 | E2B | 10 | B05 |
| U9 | 4 | E2A | 8 | B04 |
| U10 | 3 | ICM_SCLK | 5 | A01 |
| U10 | 4 | ICM_SDA | 3 | A00 |
| U12 | 3 | E1B | 16 | B12 |
| U12 | 4 | E1A | 31 | B23 |
| U14 | 10 | PWMB | 71 | B20 |
| U14 | 11 | BIN2 | 26 | B07 |
| U14 | 12 | BIN1 | 28 | B09 |
| U14 | 14 | AIN1 | 30 | A13 |
| U14 | 15 | AIN2 | 32 | B26 |
| U14 | 16 | PWMA | 4 | A28 |

## 更新方式

连接 EasyEDA bridge 后，可运行下列只读脚本重新采集关系：

```bash
.venv/bin/python scripts/analyze_easyeda_schematic.py --json
```

# 温度测量与界面操作说明

## 功能概览
- **温度监测模块**：使用 ADC1 采样热敏电阻电压，计算当前温度 `T_now`，显示当前值与温度上限 `T_high`，超限时点亮 LED。
- **定时/蜂鸣器模块**：沿用原有等待 + 报警时序，加入到 UI 主菜单作为一个独立模块，可通过按钮启动/停止。
- **风扇控制（PWM，手动模式）**：在主菜单新增“Fan PWM”子项，支持旋转编码器设定占空比，编码器按键/主按键可控启停。
- **单键 UI**：短按切换/执行，长按在“菜单 ↔ 模块”之间切换。

## 模块封装与调用
- **主菜单 (UI)**
  - 入口：`main.c` 中的 `uiState`/`currentModule` 状态机。
  - 显示：`RenderMenu()`：列出 `Timer`、`Temp` 与 `Fan PWM` 三个子模块，标记当前选择项。
  - 交互：短按切换选择，长按进入所选模块；在模块内长按返回菜单。
- **温度模块**
  - 周期处理：`TemperatureModule_Process()`（`Core/Src/main.c`）。
  - 采样：`ReadTemperatureC()` 调用 HAL ADC 单次转换，使用 Beta 曲线将 ADC 码值换算为摄氏度。
  - 显示：`RenderTemperature()` 输出 `Temp` 与 `High` 数值、告警状态，短按提高上限 `+0.5°C`（循环 20–80°C）。
- **定时模块**
  - 逻辑：`TimerModule_Process()`/`TimerModule_Toggle()`/`TimerModule_Reset()` 保持原有等待 + 报警 + 结束阶段。
  - 显示：`RenderTimer()` 展示 LED 状态、剩余时间、短按动作提示。
  - 硬件：使用 LED/Buzzer，退出模块或停止时自动复位。
- **风扇控制模块（手动 PWM）**
  - 逻辑：`FanModule_Process()` 读取 TIM3 旋转编码器增量，实时调整 `fanDuty` 并调用 `FanModule_ApplyOutput()` 输出到 DRV8833。
  - 显示：`RenderFanManual()` 呈现模式=手动、风扇启停状态、当前占空比百分比。
  - 启停：
    - 主按键（PB14）：短按在风扇开/关之间切换；长按返回主菜单。
    - 旋转编码器按键（PA10）：任意时刻短按切换开/关，用于现场停转。
  - 速度：旋转编码器一圈对应 40 脉冲，顺时针加速、逆时针减速，占空比 0–100% 直接映射 PWM 占空比，调节即刻生效。

## 代码结构与命名
- 主要文件：
  - `Core/Src/main.c`：UI 状态机、ADC 初始化、三个业务模块及显示逻辑。
  - `Core/Inc/main.h`：温度采样引脚、按键、编码器、风扇驱动引脚宏。
  - `Core/Src/stm32f1xx_hal_msp.c`：ADC1 MSP 时钟 + 引脚初始化/反初始化。
  - `cmake/stm32cubemx/CMakeLists.txt`：加入 `stm32f1xx_hal_adc*.c` 驱动。
  - `CMakeLists.txt`：链接 `m` 数学库以支持 `logf`。
- 关键变量命名：
  - `tempHighC`：温度上限；`temperatureC`：最新温度值；
  - `uiState`/`currentModule`：UI 状态机；`buttonDown`/`buttonDownTick`：按键消抖与长按判定；
  - `fanEnabled`/`fanDuty`：风扇启停与占空比；`encoderLastCount`：旋转编码器计数缓存；
  - `phase`/`buzzerEventTick`：定时模块状态机；
  - 常量使用全大写下划线：`TEMP_HIGH_STEP`、`BUTTON_LONG_PRESS_MS`、`ENCODER_PULSES_PER_TURN` 等，便于辨识配置项。

## 执行流程框图
```
+------------------+
|    上电/初始化   |
+--------+---------+
         |
         v
+------------------+
|  显示主菜单(UI)  |
+----+-------------+
     |短按切换
     |长按进入模块
     v
+------------------+
|  模块运行状态    |
|  Timer / Temp /  |
|  Fan PWM         |
+----+-------------+
     |短按: 模块动作
     |长按: 返回菜单
     v
+------------------+
|    UI 更新/显示  |
+------------------+
```

## CubeMX 关键设置
1. **ADC1**
   - 模式：`Independent mode`，`Scan Conversion Disabled`，`Continuous Conversion Disabled`，触发 `Software Start`，数据对齐 `Right`，转换数 `1`。
   - 通道：`IN0 (PA0)`，`Rank 1`，采样时间 `239.5 Cycles`（确保热敏电阻 RC 充电稳定）。
   - 校准：初始化后执行 `HAL_ADCEx_Calibration_Start`（代码已调用）。
2. **GPIO**
   - `PA0` 配置为 `Analog`（热敏电阻 + 分压输入）。
   - `PA7`：LED，`Push-Pull Output`，无上拉；`PA9`：蜂鸣器输出。
   - `PB14`：主按键输入，`GPIO Input`，`Pull-Down`；`PC13`：测试 LED（开漏保留）。
   - `PB4/PB5`：旋转编码器 A/B 接入 TIM3 编码器模式；`PA10`：编码器按键输入。
   - `PA15`/`PB3`：DRV8833 IN1/IN2，TIM2 PWM 输出。
3. **I2C1**
   - 时钟 `100 kHz`，`PB6/PB7` 开漏复用；用于 SSD1306 OLED。
4. **时钟树**
   - HSI 8 MHz，SYSCLK 8 MHz，AHB/APB1/APB2 不分频即可满足 ADC 与 I2C 时序。

## 执行与操作步骤
1. 上电后进入主菜单；短按主按键循环选中 `Timer`、`Temp`、`Fan PWM`。
2. 长按进入模块：
   - **Timer**：短按启动/停止；LED 在报警阶段亮，蜂鸣器按配置鸣叫；长按返回菜单。
   - **Temp**：周期显示 `Temp` 与 `High`；超限 LED 亮；短按将 `High` 递增 0.5°C（20–80°C 循环）；长按返回菜单。编码器按键可切换阈值编辑开/关。
   - **Fan PWM（手动）**：短按主按键或按下编码器键控制风扇启停；旋转编码器调节占空比（0–100%），数值变化立即反映在风扇输出与屏幕。
3. 回到菜单后蜂鸣器与 LED 会自复位到安全状态，风扇维持进入前的上电默认值（上电默认占空比 30%、关闭）。

## 接线方法与要求

| 功能           | MCU 管脚（定义于 `Core/Inc/main.h`） | 说明 |
| -------------- | ------------------------------------- | ---- |
| 主按键         | `PB14` (`KEY_Pin`, 下拉输入)          | 短按：菜单切换/模块动作；长按：菜单与模块切换。 |
| 旋转编码器 A/B | `PB4` (`ENC_A_Pin`), `PB5` (`ENC_B_Pin`) | 连接到 TIM3 编码器接口，A/B 相信号需与 MCU GND 共地。 |
| 旋转编码器按键 | `PA10` (`ENC_KEY_Pin`)                | 建议外接上拉（按下接地），短按在温度模块中切换编辑、在风扇模块中启停。 |
| 蜂鸣器         | `PA9` (`BUZZER_Pin`)                  | 推挽输出，高电平关闭、低电平驱动。 |
| LED            | `PA7` (`LED_Pin`)                     | 指示灯输出。 |
| DRV8833 IN1/IN2| `PA15` (`FAN_A_Pin`/TIM2_CH1), `PB3` (`FAN_B_Pin`/TIM2_CH2) | 作为 H 桥 PWM 输入，需共地；占空比 0–100% 对应风扇速度。 |
| OLED (SSD1306) | I2C1 `PB6`/`PB7`                      | 开漏上拉，地址随屏配置。 |
| 热敏电阻分压   | `PA0`（ADC1 IN0）                     | 分压接入，匹配 10k NTC + 4.7k 串联。 |

注意：编码器与 DRV8833/风扇电源必须与 MCU 共地；DRV8833 VM 端按风扇规格供电，保证 TIM2 PWM 管脚与驱动输入匹配（默认使用 TIM2 CH1/CH2）。

### 如何用万用表验证 PA10（旋转编码器按键）是否正常

1. **断电前确认接线**：PA10 通过外部上拉到 VCC（3.3V），按下时短接到 GND，万用表黑表笔接到 GND，红表笔接到 PA10。 
2. **上电前静态测量**：不按键时表笔保持接触，万用表应读到上拉电压（≈3.3V）；若接近 0V，检查上拉或短路。 
3. **按下测量**：保持表笔位置，按下编码器按键，电压应迅速跌至 0V 左右，松开后恢复到 3.3V；无变化则检查按键端到 GND 的连接或引脚焊接。 
4. **通断档辅助检查（可选）**：将万用表拨到蜂鸣/电阻档，黑表笔接 GND，红表笔接 PA10；按下时应有通路蜂鸣，松开后断开。 
5. **结合软件观察**：在固件运行时进入风扇或温度模块，按键短按应能切换启停/编辑状态；若万用表测试正常但功能异常，需排查 GPIO 模式是否为 `GPIO_MODE_INPUT` 且未误配内部下拉。

## 热敏电阻换算说明
- 分压公式求得热敏电阻阻值：`R_therm = R_series * Vout / (Vref - Vout)`。
- Beta 公式求温度（开尔文）：`T = 1 / (1/T0 + (1/BETA) * ln(R/R0))`，最终换算为摄氏度。
- 相关常量在 `main.c` 顶部配置，如需要适配不同热敏电阻可修改 `THERMISTOR_BETA`、`THERMISTOR_R0`、`SERIES_RESISTOR` 等。

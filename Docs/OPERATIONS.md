# 温度测量与界面操作说明

## 功能概览
- **温度监测模块**：使用 ADC1 采样热敏电阻电压，计算当前温度 `T_now`，显示当前值与温度上限 `T_high`，超限时点亮 LED。
- **定时/蜂鸣器模块**：沿用原有等待 + 报警时序，加入到 UI 主菜单作为一个独立模块，可通过按钮启动/停止。
- **单键 UI**：短按切换/执行，长按在“菜单 ↔ 模块”之间切换，保证现场可见当前温度与上限数值。

## 模块封装与调用
- **主菜单 (UI)**
  - 入口：`main.c` 中的 `uiState`/`currentModule` 状态机。
  - 显示：`RenderMenu()`：列出 `Timer` 与 `Temp` 两个子模块，标记当前选择项。
  - 交互：短按切换选择，长按进入所选模块；在模块内长按返回菜单。
- **温度模块**
  - 周期处理：`TemperatureModule_Process()`（`Core/Src/main.c`）。
  - 采样：`ReadTemperatureC()` 调用 HAL ADC 单次转换，使用 Beta 曲线将 ADC 码值换算为摄氏度。
  - 显示：`RenderTemperature()` 输出 `Temp` 与 `High` 数值、告警状态，短按提高上限 `+0.5°C`（循环 20–80°C）。
- **定时模块**
  - 逻辑：`TimerModule_Process()`/`TimerModule_Toggle()`/`TimerModule_Reset()` 保持原有等待 + 报警 + 结束阶段。
  - 显示：`RenderTimer()` 展示 LED 状态、剩余时间、短按动作提示。
  - 硬件：使用 LED/Buzzer，退出模块或停止时自动复位。

## 代码结构与命名
- 主要文件：
  - `Core/Src/main.c`：UI 状态机、ADC 初始化、两个业务模块及显示逻辑。
  - `Core/Inc/main.h`：温度采样引脚宏 `TEMP_SENSOR_Pin`/`TEMP_SENSOR_GPIO_Port`。
  - `Core/Src/stm32f1xx_hal_msp.c`：ADC1 MSP 时钟 + 引脚初始化/反初始化。
  - `cmake/stm32cubemx/CMakeLists.txt`：加入 `stm32f1xx_hal_adc*.c` 驱动。
  - `CMakeLists.txt`：链接 `m` 数学库以支持 `logf`。
- 关键变量命名：
  - `tempHighC`：温度上限；`temperatureC`：最新温度值；
  - `uiState`/`currentModule`：UI 状态机；`buttonDown`/`buttonDownTick`：按键消抖与长按判定；
  - `phase`/`buzzerEventTick`：定时模块状态机；
  - 常量使用全大写下划线：`TEMP_HIGH_STEP`、`BUTTON_LONG_PRESS_MS` 等，便于辨识配置项。

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
|  Timer 或 Temp   |
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
   - `PB14`：按键输入，`GPIO Input`，`Pull-Down`；`PC13`：测试 LED（开漏保留）。
3. **I2C1**
   - 时钟 `100 kHz`，`PB6/PB7` 开漏复用；用于 SSD1306 OLED。
4. **时钟树**
   - HSI 8 MHz，SYSCLK 8 MHz，AHB/APB1/APB2 不分频即可满足 ADC 与 I2C 时序。

## 执行与操作步骤
1. 上电后进入主菜单；短按循环选中 `Timer` 或 `Temp`。
2. 长按进入模块：
   - **Timer**：短按启动/停止；LED 在报警阶段亮，蜂鸣器按配置鸣叫；长按返回菜单。
   - **Temp**：周期显示 `Temp` 与 `High`；超限 LED 亮；短按将 `High` 递增 0.5°C（20–80°C 循环）；长按返回菜单。
3. 回到菜单后蜂鸣器与 LED 会自动复位到安全状态。

## 热敏电阻换算说明
- 分压公式求得热敏电阻阻值：`R_therm = R_series * Vout / (Vref - Vout)`。
- Beta 公式求温度（开尔文）：`T = 1 / (1/T0 + (1/BETA) * ln(R/R0))`，最终换算为摄氏度。
- 相关常量在 `main.c` 顶部配置，如需要适配不同热敏电阻可修改 `THERMISTOR_BETA`、`THERMISTOR_R0`、`SERIES_RESISTOR` 等。

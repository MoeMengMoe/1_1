# 主菜单与温度防护/循环提醒说明

## 功能概述
- 主菜单保留 2 项：`1/2 Reminder`（定时循环提醒）与 `2/2 TempGuard`（温度测量 + 旋钮人机交互）。
- PB14（按压）**短按**在两项间切换；**长按**执行当前项的操作：
  - Reminder：启动/停止循环提醒。
  - TempGuard：进入/退出阈值编辑，高亮 `High` 行。
- 旋钮旋转：
  - Reminder：调整提醒周期（5 s 步进，10~900 s 自动夹取）。
  - TempGuard：在编辑模式下调整温度阈值 `T_high`（20~40 ℃，步进 1 ℃）。
- TempGuard 同屏显示 `Temp` 与 `High`，并在 `T_now ≥ T_high` 时驱动 LED+蜂鸣器报警。
- 保留温度校准接口，便于实测修正偏差；循环提醒与温度报警共用原有接线（LED、蜂鸣器、编码器 A/B + PB14）。

## 操作逻辑
1. 上电进入 `Reminder` 页面，可通过旋转调整周期；短按 PB14 可切换到 `TempGuard`。
2. Reminder 长按 PB14 可启动/暂停循环；到点时 LED 点亮+蜂鸣器鸣叫短脉冲，界面显示剩余倒计时。
3. TempGuard 界面长按 PB14 进入/退出编辑，进入后 `High` 行反白；旋转修改阈值（20~40 ℃）。
4. TempGuard 界面显示：`Temp` 当前温度、`High` 阈值、`Cal` 校准偏移与报警状态 `ALERT/OK`，底行提示当前可用按键操作。
5. 当 `T_now ≥ T_high` 时立即报警，并驱动 LED/蜂鸣器闪烁提醒。

## 代码架构
- `Core/Src/main.c`
  - `MenuState` 记录当前菜单与 TempGuard 编辑状态；短按/长按 PB14 区分菜单切换与功能操作。
  - `HandleButton`/`HandleEncoderStep` 处理 PB14 与旋转；Reminder 使用旋转调周期，TempGuard 在编辑模式下调阈值。
  - `UpdateDisplay` 分别绘制 Reminder 倒计时/周期/状态提示与 TempGuard 温度、阈值、校准与报警状态。
  - 主循环根据所选菜单驱动 `Reminder_Service` 或 `Alarm_Service`，并定期采样温度。
- `Core/Src/reminder.c`
  - 提供循环提醒定时、启动/停止、剩余秒数计算与 LED/蜂鸣器脉冲输出。
- `Core/Src/temperature.c`
  - `Temperature_ReadWithCalibration` 读取并应用校准；`Temperature_SetCalibrationOffset` 提供实机偏移设定。
- 其余模块：`encoder.c` 读取旋钮，`alarm.c` 负责超温报警脉冲，`ssd1306.c` 绘制 OLED。

## 校准提示
- 现场若发现温度偏差，可在完成实测后调用 `Temperature_SetCalibrationOffset(<偏差值>)` 设置偏移，屏幕 `Cal` 行会实时显示偏移量，便于校准验证。
- 当前温度为模拟值，替换真实传感器时，在 `Temperature_ReadWithCalibration` 中读取硬件并叠加 `calibrationOffsetC` 即可。

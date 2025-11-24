# 旋转编码器温度防护模块说明

## 功能概述
- 主菜单新增第 3 项 **TempGuard**，用于旋钮+按压调节温度上限 `T_high`，并与当前温度 `T_now` 同屏显示。
- 通过旋钮按压进入/退出编辑模式，进入后高亮显示 `High`；旋转在 20~40 ℃ 范围内以 1 ℃ 步进调整阈值，自动夹取越界值。
- 当 `T_now ≥ T_high` 时，蜂鸣器脉冲+LED 闪烁报警，屏幕状态切换为 `ALERT`。
- 默认选中主菜单第 3 项，保持原有接线：编码器 A/B 相 + 按压输入，LED 与蜂鸣器输出不变。
- 温度数值保留校准接口，便于未来实测修正显示误差。

## 操作逻辑
1. 上电后直接处于菜单第 3 项 **TempGuard**。
2. **旋钮按压**：切换是否编辑 `T_high`（仅在第 3 项有效）；进入编辑后 `High` 行反白。
3. **旋钮旋转**：
   - 编辑模式：调节 `T_high`，步进 1 ℃，范围 20~40 ℃。
   - 非编辑模式：可切换主菜单其它项（1/3 概览、2/3 接线提示）。
4. 屏幕行内容（TempGuard）：
   - `Temp`：当前温度 `T_now`。
   - `High`：参考上限，高亮表示可编辑。
   - `Cal`：当前校准偏移；`ALERT/OK` 表示报警状态。
   - `Press/Rotate` 提示当前可操作动作，显示报警计时切换剩余秒数。
5. 当 `T_now ≥ T_high` 时，报警立即触发并更新显示。

## 代码架构
- `Core/Src/main.c`
  - `MenuState` 记录当前菜单与编辑状态。
  - `HandleButton`/`HandleEncoderStep` 统一处理按压与旋转，保证编辑与菜单切换的分离。
  - `UpdateDisplay` 根据菜单项绘制 UI，第 3 项展示温度与阈值并高亮编辑行。
  - 主循环负责采样温度、调用 `Alarm_Service`、刷新屏幕。
- `Core/Src/temperature.c`
  - `Temperature_ReadWithCalibration` 返回应用校准后的温度（当前使用仿真波形作为占位）。
  - `Temperature_SetCalibrationOffset`/`GetCalibrationOffset` 提供校准接口，可在实机测试时写入偏移量修正显示。
- 其余模块保持原职责：`encoder.c` 读取旋转编码器，`alarm.c` 生成 LED+蜂鸣器脉冲，`ssd1306.c` 驱动屏幕。

## 校准提示
- 现场若发现温度显示偏差，可在完成实测后调用 `Temperature_SetCalibrationOffset(<偏差值>)` 设置偏移，偏移量同时显示在屏幕 `Cal` 行，便于验证。
- 当前温度来源为模拟函数，替换真实传感器时，只需在 `Temperature_ReadWithCalibration` 中改为读取硬件值并叠加 `calibrationOffsetC`。

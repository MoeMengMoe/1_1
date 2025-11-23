# CubeMX 配置要点（匹配当前代码）

以下步骤针对 `STM32F103C8Tx`，保证生成的代码与仓库里的主循环、模块接口一致。

## 时钟与全局
1. 在 **Project Manager → Advanced Settings** 里保持 **HAL** 库默认配置即可。
2. 在 **Clock Configuration** 使用默认 72MHz HSI→PLL 设置（与仓库生成的 `SystemClock_Config` 一致）。

## I2C1（OLED 屏）
1. 在 **Pinout & Configuration → Peripherals** 里启用 **I2C1**，模式选 **I2C**。
2. 分配引脚：
   - **PB6** → `I2C1_SCL`
   - **PB7** → `I2C1_SDA`
3. 在 **I2C1 → Configuration → Parameter Settings** 中，保持默认 100kHz/标准模式即可；勾选 **Analog Filter**，必要时打开 **No Stretch** 选项保持默认关闭。
4. 生成代码后确认 `MX_I2C1_Init` 与仓库一致（`hi2c1.Instance = I2C1;`，时钟和滤波配置默认）。

## GPIO（旋转编码器、按键、LED、蜂鸣器）
在 **Pinout** 给下列引脚设置 **GPIO**，并在 **GPIO** 配置页设置模式：

| 功能            | 引脚 | 模式                      | 上/下拉        | 输出默认电平 |
|-----------------|------|---------------------------|---------------|-------------|
| 状态/调试 LED   | PC13 | `GPIO_Output`（Open-Drain)| `No pull`     | High（在生成后 `HAL_GPIO_WritePin` 设为 SET）|
| 报警 LED        | PA7  | `GPIO_Output`（Push-Pull) | `No pull`     | Low         |
| 蜂鸣器          | PA9  | `GPIO_Output`（Push-Pull) | `No pull`     | High（初始化写 1 关闭有源蜂鸣器）|
| 旋钮 A 相       | PB0  | `GPIO_Input`              | `Pull-up`     | —           |
| 旋钮 B 相       | PB1  | `GPIO_Input`              | `Pull-up`     | —           |
| 旋钮按压键      | PB14 | `GPIO_Input`              | `Pull-down`   | —           |
| 备用/单独按键   | PB14 | *与旋钮按键同脚位*        | `Pull-down`   | —           |

> 说明：代码里 `ENC_BTN_Pin` 与 `KEY_Pin` 都指向 **PB14**，如果硬件只接一个按键，保持当前定义即可；若需要区分，请在 `main.h` 和硬件连线中拆分到不同引脚，并同步更新 CubeMX。

生成代码后请确保 `MX_GPIO_Init` 中：
- 为 **GPIOC、GPIOA、GPIOB** 使能时钟。
- 对 PC13 使用 **`GPIO_MODE_OUTPUT_OD`**，对 PA7/PA9 使用 **`GPIO_MODE_OUTPUT_PP`**。
- PB0/PB1 设为输入上拉，PB14 设为输入下拉。

## 中断（可选）
目前代码使用 **轮询** 读取旋钮和按键，不需要开启 EXTI；若想用中断，可在 **GPIO** 中为 PB0/PB1/PB14 配置外部中断，并在用户代码里添加处理逻辑。

## 生成代码后检查
1. 在 **Project Manager → Code Generator** 勾选 *"Keep User Code"* 相关选项，避免覆盖 `/* USER CODE */` 区域。
2. 重新生成后核对 `Core/Inc/main.h` 中的引脚定义与上表一致。
3. 若更改了引脚，需要同步更新原理图与接线，确保 OLED、蜂鸣器、LED 与旋钮连接到指定脚位。

完成以上配置后，生成的 CubeMX 工程即可与当前仓库代码匹配，无需额外手工改动即可编译运行（在具有交叉编译链的环境中）。

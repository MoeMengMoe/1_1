/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  PHASE_WAIT = 0U,
  PHASE_ALERT
} Phase_t;

typedef enum
{
  UI_STATE_MENU = 0U,
  UI_STATE_MODULE
} UiState_t;

typedef enum
{
  MODULE_TIMER = 0U,
  MODULE_TEMPERATURE,
  MODULE_COUNT
} ModuleId_t;

typedef enum
{
  BUTTON_EVENT_NONE = 0U,
  BUTTON_EVENT_SHORT,
  BUTTON_EVENT_LONG
} ButtonEvent_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY_PRESSED_STATE GPIO_PIN_SET
#define BUTTON_LONG_PRESS_MS 800U
#define BUTTON_DEBOUNCE_MS 20U
#define TEMP_SAMPLE_PERIOD_MS 1000U
#define TEMP_HIGH_STEP 0.5f
#define TEMP_HIGH_MIN 20.0f
#define TEMP_HIGH_MAX 80.0f
#define TEMP_ALERT_BLINK_MS 250U

#define THERMISTOR_BETA 3950.0f
#define THERMISTOR_R0 10000.0f
#define THERMISTOR_T0 298.15f
#define SERIES_RESISTOR 4700.0f
#define VREF 3.3f
#define ADC_MAX 4095.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
static uint8_t isRunning = 0U;
static Phase_t phase = PHASE_WAIT;
static uint32_t phaseStartTick = 0U;
static uint8_t buzzerPulseActive = 0U;
static uint32_t buzzerEventTick = 0U;
static uint32_t lastTimerDisplaySecond = UINT32_MAX;

static UiState_t uiState = UI_STATE_MENU;
static ModuleId_t currentModule = MODULE_TIMER;
static uint8_t buttonDown = 0U;
static uint32_t buttonDownTick = 0U;
static uint8_t displayDirty = 1U;

static float temperatureC = 0.0f;
static float tempHighC = 30.0f;
static uint32_t lastTempSampleTick = 0U;
static uint32_t lastUiRefreshTick = 0U;
static uint8_t tempEditMode = 0U;
static uint8_t encoderLastState = 0U;
static uint32_t tempAlertBlinkTick = 0U;
static uint8_t tempAlertPhase = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
static void TimerModule_Reset(uint32_t now);
static void TimerModule_Toggle(uint32_t now);
static void TimerModule_Process(uint32_t now);
static void TemperatureModule_Process(uint32_t now);
static void RenderMenu(void);
static void RenderTimer(uint8_t ledState, uint32_t msToToggle, uint8_t running);
static void RenderTemperature(float tempC, float highC, uint8_t isAlert, uint8_t editMode);
static ButtonEvent_t Button_Poll(uint32_t now, GPIO_PinState keyState);
static void HandleButtonEvent(ButtonEvent_t event, uint32_t now);
static float ReadTemperatureC(void);
static uint8_t Encoder_ReadState(void);
static int8_t Encoder_GetDelta(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const uint32_t Wait_time = 2000U;
const uint32_t Last_time = 10000U;
const uint32_t BuzzerOnTime = 200U;
const uint32_t BuzzerOffTime = 300U;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_Init();
  SSD1306_Fill(0U);
  SSD1306_UpdateScreen();
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
  TimerModule_Reset(HAL_GetTick());
  encoderLastState = Encoder_ReadState();
  displayDirty = 1U;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();
    HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin, GPIO_PIN_RESET);
    GPIO_PinState keyState = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
    ButtonEvent_t event = Button_Poll(now, keyState);
    HandleButtonEvent(event, now);

    if (uiState == UI_STATE_MENU)
    {
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      if ((displayDirty != 0U) || ((now - lastUiRefreshTick) > 500U))
      {
        RenderMenu();
        displayDirty = 0U;
        lastUiRefreshTick = now;
      }
    }
    else
    {
      if (currentModule == MODULE_TIMER)
      {
        TimerModule_Process(now);
      }
      else
      {
        TemperatureModule_Process(now);
      }
    }
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_TEST_GPIO_Port, LED_TEST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : ENC_A_Pin ENC_B_Pin */
  GPIO_InitStruct.Pin = ENC_A_Pin|ENC_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_TEST_Pin */
  GPIO_InitStruct.Pin = LED_TEST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_TEST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = LED_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC_A_Pin ENC_B_Pin */
  GPIO_InitStruct.Pin = ENC_A_Pin|ENC_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void RenderMenu(void)
{
  char line1[18];
  char line2[18];
  snprintf(line1, sizeof(line1), "%c Timer", (currentModule == MODULE_TIMER) ? '>' : ' ');
  snprintf(line2, sizeof(line2), "%c Temp", (currentModule == MODULE_TEMPERATURE) ? '>' : ' ');
  SSD1306_Fill(0U);
  SSD1306_SetCursor(0U, 0U);
  SSD1306_WriteString("Main Menu");
  SSD1306_SetCursor(0U, 16U);
  SSD1306_WriteString(line1);
  SSD1306_SetCursor(0U, 32U);
  SSD1306_WriteString(line2);
  SSD1306_SetCursor(8U, 48U);
  SSD1306_WriteString("Short: Next");
  SSD1306_SetCursor(8U, 56U);
  SSD1306_WriteString("Long: Enter ");
  SSD1306_UpdateScreen();
}

static void RenderTimer(uint8_t ledState, uint32_t msToToggle, uint8_t running)
{
  uint32_t seconds = (msToToggle + 999U) / 1000U;
  const char *stateText = ledState ? "ON" : "OFF";
  const char *direction = ledState ? "to OFF" : "to ON";

  if (running == 0U)
  {
    direction = "to ON";
  }
  char line1[21];
  char line2[25];
  char line3[25];

  snprintf(line1, sizeof(line1), "LED: %s", stateText);
  if (running != 0U)
  {
    snprintf(line2, sizeof(line2), "REMAIN: %03lus (%s)", (unsigned long)seconds, direction);
  }
  else
  {
    snprintf(line2, sizeof(line2), "REMAIN: ---s (%s)", direction);
  }
  snprintf(line3, sizeof(line3), "Short: %s", (running != 0U) ? "Stop" : "Start");

  SSD1306_Fill(0U);
  SSD1306_SetCursor(0U, 0U);
  SSD1306_WriteString("Timer Module");
  SSD1306_SetCursor(0U, 16U);
  SSD1306_WriteString(line1);
  SSD1306_SetCursor(0U, 32U);
  SSD1306_WriteString(line2);
  SSD1306_SetCursor(0U, 48U);
  SSD1306_WriteString(line3);
  SSD1306_SetCursor(90U, 48U);
  SSD1306_WriteString("Long:Menu");
  SSD1306_UpdateScreen();
}

static void RenderTemperature(float tempC, float highC, uint8_t isAlert, uint8_t editMode)
{
  char line1[25];
  char line2[25];
  char line3[25];
  snprintf(line1, sizeof(line1), "Temp: %0.1fC", (double)tempC);
  snprintf(line2, sizeof(line2), "High%s %0.1fC", (editMode != 0U) ? "*" : ":", (double)highC);
  snprintf(line3, sizeof(line3), "State: %s", isAlert ? "ALERT" : "Normal");

  SSD1306_Fill(0U);
  SSD1306_SetCursor(0U, 0U);
  SSD1306_WriteString("Temp Module");
  SSD1306_SetCursor(0U, 16U);
  SSD1306_WriteString(line1);
  SSD1306_SetCursor(0U, 32U);
  SSD1306_WriteString(line2);
  SSD1306_SetCursor(0U, 48U);
  SSD1306_WriteString(line3);
  SSD1306_SetCursor(90U, 48U);
  SSD1306_WriteString("Press:Edit");
  SSD1306_SetCursor(90U, 56U);
  SSD1306_WriteString("Long:Menu");
  SSD1306_UpdateScreen();
}

static uint8_t Encoder_ReadState(void)
{
  uint8_t a = (HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  uint8_t b = (HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  return (uint8_t)((a << 1) | b);
}

static int8_t Encoder_GetDelta(void)
{
  uint8_t state = Encoder_ReadState();
  int8_t delta = 0;
  uint8_t transition = (uint8_t)((encoderLastState << 2U) | state);

  switch (transition)
  {
    case 0x01:
    case 0x07:
    case 0x08:
    case 0x0E:
      delta = 1;
      break;
    case 0x02:
    case 0x04:
    case 0x0B:
    case 0x0D:
      delta = -1;
      break;
    default:
      break;
  }

  encoderLastState = state;
  return delta;
}

static ButtonEvent_t Button_Poll(uint32_t now, GPIO_PinState keyState)
{
  if ((keyState == KEY_PRESSED_STATE) && (buttonDown == 0U))
  {
    buttonDown = 1U;
    buttonDownTick = now;
  }
  else if ((keyState != KEY_PRESSED_STATE) && (buttonDown != 0U))
  {
    uint32_t duration = now - buttonDownTick;
    buttonDown = 0U;
    if (duration >= BUTTON_LONG_PRESS_MS)
    {
      return BUTTON_EVENT_LONG;
    }
    else if (duration >= BUTTON_DEBOUNCE_MS)
    {
      return BUTTON_EVENT_SHORT;
    }
  }
  return BUTTON_EVENT_NONE;
}

static void HandleButtonEvent(ButtonEvent_t event, uint32_t now)
{
  if (event == BUTTON_EVENT_LONG)
  {
    if (uiState == UI_STATE_MENU)
    {
      uiState = UI_STATE_MODULE;
      if (currentModule == MODULE_TIMER)
      {
        TimerModule_Reset(now);
      }
      tempEditMode = 0U;
      displayDirty = 1U;
    }
    else
    {
      uiState = UI_STATE_MENU;
      TimerModule_Reset(now);
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      tempEditMode = 0U;
      displayDirty = 1U;
    }
  }
  else if (event == BUTTON_EVENT_SHORT)
  {
    if (uiState == UI_STATE_MENU)
    {
      currentModule = (ModuleId_t)((currentModule + 1U) % MODULE_COUNT);
      displayDirty = 1U;
    }
    else if (currentModule == MODULE_TIMER)
    {
      TimerModule_Toggle(now);
      displayDirty = 1U;
    }
    else
    {
      tempEditMode = (tempEditMode == 0U) ? 1U : 0U;
      displayDirty = 1U;
    }
  }
}

static void TimerModule_Reset(uint32_t now)
{
  isRunning = 0U;
  phase = PHASE_WAIT;
  phaseStartTick = now;
  buzzerPulseActive = 0U;
  buzzerEventTick = now;
  lastTimerDisplaySecond = UINT32_MAX;
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

static void TimerModule_Toggle(uint32_t now)
{
  isRunning = !isRunning;
  phase = PHASE_WAIT;
  phaseStartTick = now;
  buzzerPulseActive = 0U;
  buzzerEventTick = now;
  lastTimerDisplaySecond = UINT32_MAX;
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
}

static void TimerModule_Process(uint32_t now)
{
  uint8_t stateChanged = 0U;
  uint32_t msToNextEvent = Wait_time;

  if (isRunning == 1U)
  {
    switch (phase)
    {
      case PHASE_WAIT:
      {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
        uint32_t elapsed = now - phaseStartTick;
        if (elapsed >= Wait_time)
        {
          phase = PHASE_ALERT;
          phaseStartTick = now;
          buzzerPulseActive = 0U;
          buzzerEventTick = now;
          stateChanged = 1U;
        }
        msToNextEvent = (elapsed < Wait_time) ? (Wait_time - elapsed) : 0U;
        break;
      }
      case PHASE_ALERT:
      default:
      {
        if ((buzzerPulseActive == 0U) && (now >= buzzerEventTick))
        {
          HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
          buzzerPulseActive = 1U;
          buzzerEventTick = now + BuzzerOnTime;
        }
        else if ((buzzerPulseActive == 1U) && (now >= buzzerEventTick))
        {
          HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
          buzzerPulseActive = 0U;
          buzzerEventTick = now + BuzzerOffTime;
        }

        uint32_t elapsed = now - phaseStartTick;
        if (elapsed >= Last_time)
        {
          HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
          phase = PHASE_WAIT;
          phaseStartTick = now;
          buzzerPulseActive = 0U;
          buzzerEventTick = now;
          stateChanged = 1U;
        }
        msToNextEvent = (elapsed < Last_time) ? (Last_time - elapsed) : 0U;
        break;
      }
    }
  }
  else
  {
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    msToNextEvent = Wait_time;
  }

  uint8_t ledState = (isRunning == 1U) && (phase == PHASE_ALERT);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, ledState ? GPIO_PIN_SET : GPIO_PIN_RESET);
  uint32_t currentSecond = (msToNextEvent + 999U) / 1000U;
  if ((displayDirty != 0U) || (currentSecond != lastTimerDisplaySecond) || (stateChanged != 0U))
  {
    RenderTimer(ledState, msToNextEvent, isRunning);
    lastTimerDisplaySecond = currentSecond;
    displayDirty = 0U;
  }
}

static float ReadTemperatureC(void)
{
  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    return temperatureC;
  }
  if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK)
  {
    HAL_ADC_Stop(&hadc1);
    return temperatureC;
  }
  uint32_t raw = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  float voltage = ((float)raw / ADC_MAX) * VREF;
  if ((voltage <= 0.01f) || (voltage >= (VREF - 0.01f)))
  {
    return temperatureC;
  }
  float resistance = (SERIES_RESISTOR * (VREF - voltage)) / voltage;
  float tempK = 1.0f / ((1.0f / THERMISTOR_BETA) * logf(resistance / THERMISTOR_R0) + (1.0f / THERMISTOR_T0));
  return tempK - 273.15f;
}

static void TemperatureModule_Process(uint32_t now)
{
  int8_t delta = Encoder_GetDelta();
  if (tempEditMode != 0U)
  {
    if (delta > 0)
    {
      tempHighC += TEMP_HIGH_STEP;
    }
    else if (delta < 0)
    {
      tempHighC -= TEMP_HIGH_STEP;
    }

    if (tempHighC > TEMP_HIGH_MAX)
    {
      tempHighC = TEMP_HIGH_MAX;
    }
    if (tempHighC < TEMP_HIGH_MIN)
    {
      tempHighC = TEMP_HIGH_MIN;
    }

    if (delta != 0)
    {
      displayDirty = 1U;
    }
  }

  if ((lastTempSampleTick == 0U) || ((now - lastTempSampleTick) >= TEMP_SAMPLE_PERIOD_MS))
  {
    temperatureC = ReadTemperatureC();
    lastTempSampleTick = now;
    displayDirty = 1U;
  }

  uint8_t alert = (temperatureC >= tempHighC) ? 1U : 0U;
  if (alert != 0U)
  {
    if ((tempAlertBlinkTick == 0U) || ((now - tempAlertBlinkTick) >= TEMP_ALERT_BLINK_MS))
    {
      tempAlertBlinkTick = now;
      tempAlertPhase ^= 1U;
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (tempAlertPhase != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, (tempAlertPhase != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }
  }
  else
  {
    tempAlertPhase = 0U;
    tempAlertBlinkTick = now;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
  }

  if ((displayDirty != 0U) || ((now - lastUiRefreshTick) > 500U))
  {
    RenderTemperature(temperatureC, tempHighC, alert, tempEditMode);
    displayDirty = 0U;
    lastUiRefreshTick = now;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

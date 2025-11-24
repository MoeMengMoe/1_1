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
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "ssd1306.h"
#include "encoder.h"
#include "temperature.h"
#include "alarm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY_PRESSED_STATE GPIO_PIN_SET
#define TEMPERATURE_SAMPLE_PERIOD_MS 500U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
typedef enum
{
  MENU_ITEM_OVERVIEW = 0U,
  MENU_ITEM_WIRING = 1U,
  MENU_ITEM_TEMPERATURE_GUARD = 2U,
  MENU_ITEM_COUNT
} MainMenuItem;

typedef struct
{
  MainMenuItem currentItem;
  uint8_t editModeActive;
} MenuState;

static uint8_t keyReady = 1U;
static MenuState menuState = {MENU_ITEM_TEMPERATURE_GUARD, 0U};
static int32_t temperatureHighC = 30;
static float currentTemperatureC = 28.0f;
static uint32_t lastTemperatureSampleTick = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void UpdateDisplay(const MenuState *menu, float currentTempC, int32_t tempHigh, uint8_t overheatActive, uint32_t msToToggle);
static void HandleButton(GPIO_PinState keyState, MenuState *menu, uint8_t *stateChanged);
static void HandleEncoderStep(int8_t step, MenuState *menu, int32_t *tempHighC, uint8_t *stateChanged);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
  /* USER CODE BEGIN 2 */
  SSD1306_Init();
  SSD1306_Fill(0U);
  SSD1306_UpdateScreen();
  temperatureHighC = Temperature_ClampHigh(temperatureHighC);
  Temperature_SetCalibrationOffset(0.0f);
  Encoder_Init();
  uint32_t initialTick = HAL_GetTick();
  lastTemperatureSampleTick = initialTick;
  Alarm_Init(initialTick);
  uint32_t lastDisplaySecond = UINT32_MAX;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_WritePin(LED_TEST_GPIO_Port,LED_TEST_Pin, GPIO_PIN_RESET);
    uint32_t now = HAL_GetTick();
    GPIO_PinState keyState = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
    uint8_t stateChanged = 0U;

    HandleButton(keyState, &menuState, &stateChanged);

    int8_t step = Encoder_ReadStep();
    HandleEncoderStep(step, &menuState, &temperatureHighC, &stateChanged);

    if (menuState.editModeActive == 0U)
    {
      Encoder_ResetState();
    }

    if ((now - lastTemperatureSampleTick) >= TEMPERATURE_SAMPLE_PERIOD_MS)
    {
      currentTemperatureC = Temperature_ReadWithCalibration(now);
      lastTemperatureSampleTick = now;
      stateChanged = 1U;
    }

    uint8_t overheat = (currentTemperatureC >= (float)temperatureHighC) ? 1U : 0U;
    AlarmStatus alarmStatus = Alarm_Service(now, overheat, TEMPERATURE_SAMPLE_PERIOD_MS);
    uint32_t msToNextEvent = alarmStatus.ms_to_next_event;
    if (alarmStatus.alarm_changed != 0U)
    {
      stateChanged = 1U;
    }
    uint32_t currentSecond = now / 500U;
    if ((currentSecond != lastDisplaySecond) || (stateChanged != 0U))
    {
      UpdateDisplay(&menuState, currentTemperatureC, temperatureHighC, overheat, msToNextEvent);
      lastDisplaySecond = currentSecond;
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
static void HandleButton(GPIO_PinState keyState, MenuState *menu, uint8_t *stateChanged)
{
  if ((keyState == KEY_PRESSED_STATE) && (keyReady == 1U))
  {
    HAL_Delay(10);
    if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == KEY_PRESSED_STATE)
    {
      if (menu->currentItem == MENU_ITEM_TEMPERATURE_GUARD)
      {
        menu->editModeActive = (menu->editModeActive == 0U) ? 1U : 0U;
      }
      else
      {
        menu->editModeActive = 0U;
      }
      keyReady = 0U;
      *stateChanged = 1U;
    }
  }
  else if (keyState != KEY_PRESSED_STATE)
  {
    keyReady = 1U;
  }
}

static void HandleEncoderStep(int8_t step, MenuState *menu, int32_t *tempHighC, uint8_t *stateChanged)
{
  if (step == 0)
  {
    return;
  }

  if ((menu->editModeActive != 0U) && (menu->currentItem == MENU_ITEM_TEMPERATURE_GUARD))
  {
    int32_t newHigh = Temperature_ClampHigh(*tempHighC + (int32_t)step);
    if (newHigh != *tempHighC)
    {
      *tempHighC = newHigh;
      *stateChanged = 1U;
    }
    return;
  }

  if (menu->editModeActive == 0U)
  {
    int32_t nextItem = (int32_t)menu->currentItem + (int32_t)step;
    while (nextItem < 0)
    {
      nextItem += MENU_ITEM_COUNT;
    }
    while (nextItem >= MENU_ITEM_COUNT)
    {
      nextItem -= MENU_ITEM_COUNT;
    }

    if (nextItem != (int32_t)menu->currentItem)
    {
      menu->currentItem = (MainMenuItem)nextItem;
      *stateChanged = 1U;
    }
  }
}

static void UpdateDisplay(const MenuState *menu, float currentTempC, int32_t tempHigh, uint8_t overheatActive, uint32_t msToToggle)
{
  static const char *menuTitles[MENU_ITEM_COUNT] =
  {
    "1/3 Overview",
    "2/3 Wiring",
    "3/3 TempGuard"
  };

  SSD1306_Fill(0U);
  SSD1306_SetCursor(0U, 0U);
  SSD1306_WriteString(menuTitles[menu->currentItem]);

  if (menu->currentItem == MENU_ITEM_OVERVIEW)
  {
    SSD1306_SetCursor(0U, 16U);
    SSD1306_WriteString("Rotate: switch menu");
    SSD1306_SetCursor(0U, 32U);
    SSD1306_WriteString("Press: edit High");
    SSD1306_SetCursor(0U, 48U);
    SSD1306_WriteString("Use item 3 for alarm");
    SSD1306_UpdateScreen();
    return;
  }

  if (menu->currentItem == MENU_ITEM_WIRING)
  {
    SSD1306_SetCursor(0U, 16U);
    SSD1306_WriteString("Keep original wiring");
    SSD1306_SetCursor(0U, 32U);
    SSD1306_WriteString("ENC A/B + Key used");
    SSD1306_SetCursor(0U, 48U);
    SSD1306_WriteString("LED+Buzzer alarm");
    SSD1306_UpdateScreen();
    return;
  }

  uint32_t seconds = (msToToggle + 999U) / 1000U;
  int32_t scaledTemp = (int32_t)(currentTempC * 10.0f);
  int32_t tempInteger = scaledTemp / 10;
  int32_t tempDecimal = scaledTemp - (tempInteger * 10);

  if (tempDecimal < 0)
  {
    tempDecimal = -tempDecimal;
  }

  char tempLine[21];
  char highLine[21];
  char infoLine[21];
  char actionLine[24];

  snprintf(tempLine, sizeof(tempLine), "Temp:%4ld.%01ldC", (long)tempInteger, (long)tempDecimal);
  snprintf(highLine, sizeof(highLine), "High:%2ldC", (long)tempHigh);
  snprintf(infoLine, sizeof(infoLine), "Cal:%+2.1fC %s", (double)Temperature_GetCalibrationOffset(), overheatActive ? "ALERT" : "OK   ");

  if (menu->editModeActive != 0U)
  {
    snprintf(actionLine, sizeof(actionLine), "Rotate:+/-1C    ");
  }
  else
  {
    snprintf(actionLine, sizeof(actionLine), "Press:Edit Tgl:%2lus", (unsigned long)seconds);
  }

  SSD1306_SetCursor(0U, 16U);
  SSD1306_WriteString(tempLine);
  SSD1306_SetCursor(0U, 32U);
  if (menu->editModeActive != 0U)
  {
    SSD1306_WriteStringStyled(highLine, 1U);
  }
  else
  {
    SSD1306_WriteString(highLine);
  }
  SSD1306_SetCursor(0U, 48U);
  SSD1306_WriteString(infoLine);
  SSD1306_SetCursor(0U, 56U);
  SSD1306_WriteString(actionLine);
  SSD1306_UpdateScreen();
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

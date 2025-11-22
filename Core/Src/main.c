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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void UpdateDisplay(uint8_t ledState, uint32_t msToToggle, uint8_t isRunning);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
const uint32_t Wait_time = 1000U;
const uint32_t Last_time = 5000U;
const uint32_t Interval = 500U;
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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
  uint8_t isRunning = 0;
  static uint8_t keyReady = 1;
  uint8_t ledState = 0;
  static uint32_t lastBlinkTime = 0;
  static uint32_t runStartTime = 0;
  uint32_t lastDisplaySecond = UINT32_MAX;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();
    GPIO_PinState keyState = HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin);
    uint8_t stateChanged = 0U;
    uint32_t msToNextToggle = Wait_time;

    if ((keyState == GPIO_PIN_SET) && keyReady == 1)
    {
      HAL_Delay(10);
      if (HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin) == GPIO_PIN_SET)
      {
        isRunning = !isRunning;
        keyReady = 0;
        now = HAL_GetTick();
        if (isRunning == 1)
        {
          runStartTime = now;
          lastBlinkTime = now;
          ledState = 0U;
        }
        else
        {
          runStartTime = 0;
          lastBlinkTime = 0;
          ledState = 0U;
          HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
          stateChanged = 1U;
        }
      }
    }
    else if (keyState == GPIO_PIN_RESET)
    {
      keyReady = 1;
    }

    if (isRunning == 1)
    {
      if (runStartTime == 0U)
      {
        runStartTime = now;
        lastBlinkTime = now;
        ledState = 0U;
      }

      uint32_t elapsedSinceLastToggle = now - lastBlinkTime;
      msToNextToggle = (elapsedSinceLastToggle < Wait_time) ? (Wait_time - elapsedSinceLastToggle) : 0U;

      if (elapsedSinceLastToggle >= Wait_time)
      {
        lastBlinkTime = now;
        ledState = !ledState;
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, ledState ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
        HAL_Delay(Interval);
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
        msToNextToggle = Wait_time;
        stateChanged = 1U;
        now = HAL_GetTick();
      }

      if ((now - runStartTime) >= Last_time)
      {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
        isRunning = 0;
        ledState = 0U;
        runStartTime = 0;
        lastBlinkTime = 0;
        msToNextToggle = 0U;
        keyReady = 1;
        stateChanged = 1U;
      }
    }
    else
    {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
      ledState = 0U;
      msToNextToggle = Wait_time;
    }

    uint32_t currentSecond = (msToNextToggle + 999U) / 1000U;
    if ((currentSecond != lastDisplaySecond) || (stateChanged != 0U))
    {
      UpdateDisplay(ledState, msToNextToggle, isRunning);
      lastDisplaySecond = currentSecond;
    }
  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = LED_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Display the LED state and remaining time on the selected interface.
  * @param  ledState Indicates whether the LED is currently ON (1) or OFF (0)
  * @param  msToToggle Remaining time in milliseconds before the next state flip
  * @param  isRunning Scheduler state (1 when active, 0 when stopped)
  * @retval None
  */
static void UpdateDisplay(uint8_t ledState, uint32_t msToToggle, uint8_t isRunning)
{
  uint32_t seconds = (msToToggle + 999U) / 1000U;
  const char *stateText = ledState ? "ON" : "OFF";
  const char *direction = ledState ? "to OFF" : "to ON";

  if (isRunning == 0U)
  {
    direction = "to ON";
  }
  char line1[21];
  char line2[25];

  snprintf(line1, sizeof(line1), "LED: %s", stateText);
  if (isRunning != 0U)
  {
    snprintf(line2, sizeof(line2), "REMAIN: %03lus (%s)", (unsigned long)seconds, direction);
  }
  else
  {
    snprintf(line2, sizeof(line2), "REMAIN: ---s (%s)", direction);
  }

  SSD1306_Fill(0U);
  SSD1306_SetCursor(0U, 0U);
  SSD1306_WriteString(line1);
  SSD1306_SetCursor(0U, 16U);
  SSD1306_WriteString(line2);
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

#include "alarm.h"

static const uint32_t BuzzerOnTimeMs = 200U;
static const uint32_t BuzzerOffTimeMs = 300U;
static uint8_t buzzerPulseActive = 0U;
static uint32_t buzzerEventTick = 0U;

void Alarm_Init(uint32_t startTick)
{
    buzzerPulseActive = 0U;
    buzzerEventTick = startTick;
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

AlarmStatus Alarm_Service(uint32_t nowTick, uint8_t overheatActive, uint32_t idlePeriodMs)
{
    AlarmStatus status =
    {
        .led_state = 0U,
        .ms_to_next_event = idlePeriodMs,
        .alarm_changed = 0U
    };

    if (overheatActive != 0U)
    {
        if (nowTick >= buzzerEventTick)
        {
            if (buzzerPulseActive == 0U)
            {
                HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
                buzzerPulseActive = 1U;
                buzzerEventTick = nowTick + BuzzerOnTimeMs;
            }
            else
            {
                HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
                buzzerPulseActive = 0U;
                buzzerEventTick = nowTick + BuzzerOffTimeMs;
            }
            status.alarm_changed = 1U;
        }

        status.led_state = buzzerPulseActive;
        status.ms_to_next_event = (buzzerEventTick > nowTick) ? (buzzerEventTick - nowTick) : 0U;
    }
    else
    {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
        buzzerPulseActive = 0U;
        buzzerEventTick = nowTick + BuzzerOffTimeMs;
        status.led_state = 0U;
    }

    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, status.led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return status;
}

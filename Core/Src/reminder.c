#include "reminder.h"
#include "main.h"

#define REMINDER_PULSE_MS 800U
#define REMINDER_MIN_PERIOD_SEC 10U
#define REMINDER_MAX_PERIOD_SEC 900U
#define REMINDER_STEP_SEC 5U

static uint32_t reminderPeriodSec = 30U;
static uint32_t nextReminderTick = 0U;
static uint32_t pulseEndTick = 0U;
static uint8_t reminderActive = 0U;

static uint32_t clamp_period(uint32_t seconds)
{
    if (seconds < REMINDER_MIN_PERIOD_SEC)
    {
        return REMINDER_MIN_PERIOD_SEC;
    }

    if (seconds > REMINDER_MAX_PERIOD_SEC)
    {
        return REMINDER_MAX_PERIOD_SEC;
    }

    return seconds;
}

void Reminder_Init(uint32_t defaultPeriodSeconds, uint32_t nowMs)
{
    reminderPeriodSec = clamp_period(defaultPeriodSeconds);
    reminderActive = 0U;
    nextReminderTick = nowMs + (reminderPeriodSec * 1000U);
    pulseEndTick = nowMs;
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void Reminder_Stop(uint32_t nowMs)
{
    reminderActive = 0U;
    pulseEndTick = nowMs;
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void Reminder_Toggle(uint32_t nowMs)
{
    if (reminderActive == 0U)
    {
        reminderActive = 1U;
        nextReminderTick = nowMs + (reminderPeriodSec * 1000U);
        pulseEndTick = nowMs;
    }
    else
    {
        Reminder_Stop(nowMs);
    }
}

void Reminder_AdjustPeriod(int32_t deltaSeconds, uint32_t nowMs)
{
    int32_t newPeriod = (int32_t)reminderPeriodSec + (deltaSeconds * (int32_t)REMINDER_STEP_SEC);

    if (newPeriod < 0)
    {
        newPeriod = 0;
    }

    reminderPeriodSec = clamp_period((uint32_t)newPeriod);
    nextReminderTick = nowMs + (reminderPeriodSec * 1000U);
}

ReminderStatus Reminder_Service(uint32_t nowMs, uint8_t allowOutputs)
{
    ReminderStatus status =
    {
        .active = reminderActive,
        .period_seconds = reminderPeriodSec,
        .remaining_seconds = reminderPeriodSec,
        .pulse_on = 0U
    };

    if (reminderActive != 0U)
    {
        if (nowMs >= nextReminderTick)
        {
            nextReminderTick = nowMs + (reminderPeriodSec * 1000U);
            pulseEndTick = nowMs + REMINDER_PULSE_MS;
        }

        if (nextReminderTick > nowMs)
        {
            uint32_t remainingMs = nextReminderTick - nowMs;
            status.remaining_seconds = (remainingMs + 999U) / 1000U;
        }
        else
        {
            status.remaining_seconds = 0U;
        }
    }

    if (allowOutputs != 0U)
    {
        if ((reminderActive != 0U) && (nowMs < pulseEndTick))
        {
            HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
            status.pulse_on = 1U;
        }
        else
        {
            HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
            status.pulse_on = 0U;
        }
    }
    else
    {
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        status.pulse_on = 0U;
    }

    return status;
}

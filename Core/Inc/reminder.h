#ifndef REMINDER_H
#define REMINDER_H

#include <stdint.h>

typedef struct
{
    uint8_t active;
    uint32_t remaining_seconds;
    uint32_t period_seconds;
    uint8_t pulse_on;
} ReminderStatus;

void Reminder_Init(uint32_t defaultPeriodSeconds, uint32_t nowMs);
void Reminder_Toggle(uint32_t nowMs);
void Reminder_AdjustPeriod(int32_t deltaSeconds, uint32_t nowMs);
ReminderStatus Reminder_Service(uint32_t nowMs, uint8_t allowOutputs);
void Reminder_Stop(uint32_t nowMs);

#endif // REMINDER_H

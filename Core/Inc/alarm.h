#ifndef ALARM_H
#define ALARM_H

#include <stdint.h>
#include "main.h"

typedef struct
{
    uint8_t led_state;
    uint32_t ms_to_next_event;
    uint8_t alarm_changed;
} AlarmStatus;

void Alarm_Init(uint32_t startTick);
AlarmStatus Alarm_Service(uint32_t nowTick, uint8_t overheatActive, uint32_t idlePeriodMs);

#endif /* ALARM_H */

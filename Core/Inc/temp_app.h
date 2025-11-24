#ifndef TEMP_APP_H
#define TEMP_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "temp_sensor.h"
#include "temp_ui.h"

typedef struct
{
  TempSensorHandle_t sensor;
  TempUiContext_t ui;
  ADC_HandleTypeDef *adc;
  float highLimitCelsius;
  uint32_t sampleIntervalMs;
  uint32_t lastSampleTick;
} TempAppContext_t;

void TempApp_Init(TempAppContext_t *ctx, ADC_HandleTypeDef *adc);
void TempApp_Task(TempAppContext_t *ctx, uint32_t nowMs);
void TempApp_SetHighLimit(TempAppContext_t *ctx, float highLimitCelsius);
float TempApp_GetCurrentTemp(const TempAppContext_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* TEMP_APP_H */

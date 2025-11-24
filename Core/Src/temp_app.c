#include "temp_app.h"

#include <string.h>

#ifndef TEMP_APP_DEFAULT_SAMPLE_MS
#define TEMP_APP_DEFAULT_SAMPLE_MS 500U
#endif

#ifndef TEMP_APP_DEFAULT_HIGH_LIMIT
#define TEMP_APP_DEFAULT_HIGH_LIMIT 30.0f
#endif

void TempApp_Init(TempAppContext_t *ctx, ADC_HandleTypeDef *adc)
{
  if ((ctx == NULL) || (adc == NULL))
  {
    return;
  }

  memset(ctx, 0, sizeof(TempAppContext_t));
  ctx->adc = adc;
  ctx->highLimitCelsius = TEMP_APP_DEFAULT_HIGH_LIMIT;
  ctx->sampleIntervalMs = TEMP_APP_DEFAULT_SAMPLE_MS;

  TempSensorConfig_t cfg = {
      .seriesResistor = 10000.0f,
      .nominalResistance = 10000.0f,
      .nominalTemperatureC = 25.0f,
      .betaCoefficient = 3950.0f,
      .adcMaxCounts = 4095U};

  TempSensor_Init(&ctx->sensor, &cfg, ctx->adc);
  TempUi_Init(&ctx->ui, ctx->highLimitCelsius);
  ctx->lastSampleTick = HAL_GetTick();
}

void TempApp_Task(TempAppContext_t *ctx, uint32_t nowMs)
{
  if (ctx == NULL)
  {
    return;
  }

  if ((uint32_t)(nowMs - ctx->lastSampleTick) >= ctx->sampleIntervalMs)
  {
    float celsius = 0.0f;
    if (TempSensor_Read(&ctx->sensor, &celsius) == HAL_OK)
    {
      uint8_t over = (celsius >= ctx->highLimitCelsius) ? 1U : 0U;
      TempUi_UpdateReadings(&ctx->ui, celsius, over);
    }
    ctx->lastSampleTick = nowMs;
  }

  TempUi_Process(&ctx->ui, nowMs);
}

void TempApp_SetHighLimit(TempAppContext_t *ctx, float highLimitCelsius)
{
  if (ctx == NULL)
  {
    return;
  }

  ctx->highLimitCelsius = highLimitCelsius;
  TempUi_SetHighLimit(&ctx->ui, highLimitCelsius);
}

float TempApp_GetCurrentTemp(const TempAppContext_t *ctx)
{
  return (ctx != NULL) ? TempSensor_GetLastCelsius(&ctx->sensor) : 0.0f;
}

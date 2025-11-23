#include "temp_sensor.h"

#include <math.h>
#include <string.h>

#ifndef TEMP_SENSOR_ADC_TIMEOUT_MS
#define TEMP_SENSOR_ADC_TIMEOUT_MS 50U
#endif

static float TempSensor_ConvertRawToCelsius(const TempSensorHandle_t *handle, uint32_t raw)
{
  if ((handle == NULL) || (handle->config.adcMaxCounts == 0U))
  {
    return 0.0f;
  }

  float ratio = (float)raw / (float)handle->config.adcMaxCounts;
  if (ratio <= 0.0f)
  {
    ratio = 0.0001f;
  }
  else if (ratio >= 0.9999f)
  {
    ratio = 0.9999f;
  }

  float resistance = handle->config.seriesResistor * ratio / (1.0f - ratio);
  float steinhart = resistance / handle->config.nominalResistance;
  steinhart = logf(steinhart);
  steinhart /= handle->config.betaCoefficient;
  steinhart += 1.0f / (handle->config.nominalTemperatureC + 273.15f);
  steinhart = 1.0f / steinhart;
  steinhart -= 273.15f;
  return steinhart;
}

void TempSensor_Init(TempSensorHandle_t *handle,
                     const TempSensorConfig_t *config,
                     ADC_HandleTypeDef *adc)
{
  if ((handle == NULL) || (config == NULL) || (adc == NULL))
  {
    return;
  }

  memset(handle, 0, sizeof(TempSensorHandle_t));
  handle->config = *config;
  handle->adc = adc;
  handle->initialized = 1U;
}

HAL_StatusTypeDef TempSensor_Read(TempSensorHandle_t *handle, float *outCelsius)
{
  if ((handle == NULL) || (handle->initialized == 0U))
  {
    return HAL_ERROR;
  }

  if (HAL_ADC_Start(handle->adc) != HAL_OK)
  {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef status = HAL_ADC_PollForConversion(handle->adc, TEMP_SENSOR_ADC_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    uint32_t raw = HAL_ADC_GetValue(handle->adc);
    handle->lastRaw = raw;
    handle->lastCelsius = TempSensor_ConvertRawToCelsius(handle, raw);
    if (outCelsius != NULL)
    {
      *outCelsius = handle->lastCelsius;
    }
  }

  HAL_ADC_Stop(handle->adc);
  return status;
}

float TempSensor_GetLastCelsius(const TempSensorHandle_t *handle)
{
  return (handle != NULL) ? handle->lastCelsius : 0.0f;
}

uint32_t TempSensor_GetLastRaw(const TempSensorHandle_t *handle)
{
  return (handle != NULL) ? handle->lastRaw : 0U;
}

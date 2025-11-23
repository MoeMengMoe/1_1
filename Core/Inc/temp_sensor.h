#ifndef TEMP_SENSOR_H
#define TEMP_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct
{
  float seriesResistor;
  float nominalResistance;
  float nominalTemperatureC;
  float betaCoefficient;
  uint32_t adcMaxCounts;
} TempSensorConfig_t;

typedef struct
{
  TempSensorConfig_t config;
  ADC_HandleTypeDef *adc;
  uint32_t lastRaw;
  float lastCelsius;
  uint8_t initialized;
} TempSensorHandle_t;

void TempSensor_Init(TempSensorHandle_t *handle,
                     const TempSensorConfig_t *config,
                     ADC_HandleTypeDef *adc);

HAL_StatusTypeDef TempSensor_Read(TempSensorHandle_t *handle, float *outCelsius);

float TempSensor_GetLastCelsius(const TempSensorHandle_t *handle);
uint32_t TempSensor_GetLastRaw(const TempSensorHandle_t *handle);

#ifdef __cplusplus
}
#endif

#endif /* TEMP_SENSOR_H */

#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <stdint.h>

#define TEMP_HIGH_MIN 20
#define TEMP_HIGH_MAX 40

int32_t Temperature_ClampHigh(int32_t candidate);
float Temperature_Simulate(uint32_t nowTick);
void Temperature_SetCalibrationOffset(float offsetC);
float Temperature_GetCalibrationOffset(void);
float Temperature_ReadWithCalibration(uint32_t nowTick);

#endif /* TEMPERATURE_H */

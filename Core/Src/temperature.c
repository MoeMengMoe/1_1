#include "temperature.h"

static float calibrationOffsetC = 0.0f;

int32_t Temperature_ClampHigh(int32_t candidate)
{
    if (candidate < TEMP_HIGH_MIN)
    {
        return TEMP_HIGH_MIN;
    }
    if (candidate > TEMP_HIGH_MAX)
    {
        return TEMP_HIGH_MAX;
    }
    return candidate;
}

float Temperature_Simulate(uint32_t nowTick)
{
    const float baseTemp = 28.0f;
    const float swing = 8.0f;
    const float periodMs = 15000.0f;
    float phase = (float)(nowTick % (uint32_t)periodMs) / periodMs;
    float slope = (phase < 0.5f) ? (phase * 2.0f) : ((1.0f - phase) * 2.0f);

    return baseTemp + (swing * slope);
}

void Temperature_SetCalibrationOffset(float offsetC)
{
    calibrationOffsetC = offsetC;
}

float Temperature_GetCalibrationOffset(void)
{
    return calibrationOffsetC;
}

float Temperature_ReadWithCalibration(uint32_t nowTick)
{
    float raw = Temperature_Simulate(nowTick);
    return raw + calibrationOffsetC;
}

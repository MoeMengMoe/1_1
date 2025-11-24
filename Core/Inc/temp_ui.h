#ifndef TEMP_UI_H
#define TEMP_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
  float currentCelsius;
  float highLimitCelsius;
  uint8_t overLimit;
  uint8_t needRefresh;
  uint32_t lastRefreshTick;
  uint32_t minRefreshIntervalMs;
} TempUiContext_t;

void TempUi_Init(TempUiContext_t *ctx, float highLimitCelsius);
void TempUi_SetHighLimit(TempUiContext_t *ctx, float highLimitCelsius);
void TempUi_UpdateReadings(TempUiContext_t *ctx, float currentCelsius, uint8_t overLimit);
void TempUi_RequestRefresh(TempUiContext_t *ctx);
void TempUi_Process(TempUiContext_t *ctx, uint32_t nowMs);

#ifdef __cplusplus
}
#endif

#endif /* TEMP_UI_H */

#include "temp_ui.h"

#include <stdio.h>

#include "ssd1306.h"

#define TEMP_UI_DEFAULT_REFRESH_MS 200U

static void TempUi_Render(const TempUiContext_t *ctx)
{
  char line1[21];
  char line2[21];
  char line3[21];

  snprintf(line1, sizeof(line1), "Temp: %05.1fC", ctx->currentCelsius);
  snprintf(line2, sizeof(line2), "High: %05.1fC", ctx->highLimitCelsius);
  snprintf(line3, sizeof(line3), "Status: %s", ctx->overLimit ? "WARN" : "OK");

  SSD1306_Fill(0U);
  SSD1306_SetCursor(0U, 0U);
  SSD1306_WriteString(line1);
  SSD1306_SetCursor(0U, 16U);
  SSD1306_WriteString(line2);
  SSD1306_SetCursor(0U, 32U);
  SSD1306_WriteString(line3);
  SSD1306_UpdateScreen();
}

void TempUi_Init(TempUiContext_t *ctx, float highLimitCelsius)
{
  if (ctx == NULL)
  {
    return;
  }

  ctx->currentCelsius = 0.0f;
  ctx->highLimitCelsius = highLimitCelsius;
  ctx->overLimit = 0U;
  ctx->needRefresh = 1U;
  ctx->lastRefreshTick = 0U;
  ctx->minRefreshIntervalMs = TEMP_UI_DEFAULT_REFRESH_MS;
}

void TempUi_SetHighLimit(TempUiContext_t *ctx, float highLimitCelsius)
{
  if (ctx == NULL)
  {
    return;
  }

  ctx->highLimitCelsius = highLimitCelsius;
  ctx->needRefresh = 1U;
}

void TempUi_UpdateReadings(TempUiContext_t *ctx, float currentCelsius, uint8_t overLimit)
{
  if (ctx == NULL)
  {
    return;
  }

  ctx->currentCelsius = currentCelsius;
  ctx->overLimit = overLimit ? 1U : 0U;
  ctx->needRefresh = 1U;
}

void TempUi_RequestRefresh(TempUiContext_t *ctx)
{
  if (ctx != NULL)
  {
    ctx->needRefresh = 1U;
  }
}

void TempUi_Process(TempUiContext_t *ctx, uint32_t nowMs)
{
  if ((ctx == NULL) || (ctx->needRefresh == 0U))
  {
    return;
  }

  if ((ctx->lastRefreshTick == 0U) ||
      ((uint32_t)(nowMs - ctx->lastRefreshTick) >= ctx->minRefreshIntervalMs))
  {
    TempUi_Render(ctx);
    ctx->lastRefreshTick = nowMs;
    ctx->needRefresh = 0U;
  }
}

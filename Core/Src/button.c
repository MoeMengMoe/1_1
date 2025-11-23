#include "button.h"

#include <string.h>

static uint8_t Button_IsActive(const ButtonHandle_t *button, GPIO_PinState state)
{
  return (state == button->activeState) ? 1U : 0U;
}

static uint8_t Button_HasElapsed(uint32_t now, uint32_t since, uint32_t interval)
{
  return (uint32_t)(now - since) >= interval ? 1U : 0U;
}

void Button_Init(ButtonHandle_t *button,
                 GPIO_TypeDef *port,
                 uint16_t pin,
                 GPIO_PinState activeState)
{
  if (button == NULL)
  {
    return;
  }

  memset(button, 0, sizeof(ButtonHandle_t));
  button->port = port;
  button->pin = pin;
  button->activeState = activeState;
  button->debounceMs = BUTTON_DEFAULT_DEBOUNCE_MS;
  button->longPressMs = BUTTON_DEFAULT_LONGPRESS_MS;
  button->multiClickMs = BUTTON_DEFAULT_MULTI_CLICK_MS;
  button->stableState = HAL_GPIO_ReadPin(port, pin);
  button->lastRawState = button->stableState;
  button->lastRawChangeTick = HAL_GetTick();
  button->initialized = 1U;
}

void Button_ConfigureTiming(ButtonHandle_t *button,
                            uint32_t debounceMs,
                            uint32_t longPressMs,
                            uint32_t multiClickMs)
{
  if (button == NULL)
  {
    return;
  }

  button->debounceMs = (debounceMs > 0U) ? debounceMs : BUTTON_DEFAULT_DEBOUNCE_MS;
  button->longPressMs = (longPressMs > button->debounceMs) ? longPressMs : BUTTON_DEFAULT_LONGPRESS_MS;
  button->multiClickMs = (multiClickMs > 0U) ? multiClickMs : BUTTON_DEFAULT_MULTI_CLICK_MS;
}

static ButtonEvent_t Button_FlushClickQueue(ButtonHandle_t *button)
{
  ButtonEvent_t event = BUTTON_EVENT_NONE;
  if (button->clickCount == 0U)
  {
    return event;
  }

  if (button->clickCount == 1U)
  {
    event = BUTTON_EVENT_SHORT_PRESS;
  }
  else if (button->clickCount == 2U)
  {
    event = BUTTON_EVENT_DOUBLE_CLICK;
  }
  else
  {
    event = BUTTON_EVENT_TRIPLE_CLICK;
  }

  button->clickCount = 0U;
  return event;
}

ButtonEvent_t Button_Update(ButtonHandle_t *button, uint32_t nowMs)
{
  if ((button == NULL) || (button->initialized == 0U))
  {
    return BUTTON_EVENT_NONE;
  }

  ButtonEvent_t event = BUTTON_EVENT_NONE;
  GPIO_PinState rawState = HAL_GPIO_ReadPin(button->port, button->pin);

  if (rawState != button->lastRawState)
  {
    button->lastRawState = rawState;
    button->lastRawChangeTick = nowMs;
  }
  else if ((rawState != button->stableState) &&
           Button_HasElapsed(nowMs, button->lastRawChangeTick, button->debounceMs))
  {
    button->stableState = rawState;
    if (Button_IsActive(button, rawState) == 1U)
    {
      button->pressTick = nowMs;
      button->longReported = 0U;
    }
    else
    {
      if (button->longReported == 0U)
      {
        if (button->clickCount < 3U)
        {
          button->clickCount++;
        }
        else
        {
          button->clickCount = 3U;
        }
        button->lastReleaseTick = nowMs;
      }
      else
      {
        button->longReported = 0U;
      }
    }
  }

  if ((Button_IsActive(button, button->stableState) == 1U) &&
      (button->longReported == 0U) &&
      Button_HasElapsed(nowMs, button->pressTick, button->longPressMs))
  {
    event = BUTTON_EVENT_LONG_PRESS;
    button->longReported = 1U;
    button->clickCount = 0U;
    return event;
  }

  if ((button->clickCount > 0U) &&
      Button_HasElapsed(nowMs, button->lastReleaseTick, button->multiClickMs))
  {
    event = Button_FlushClickQueue(button);
  }

  return event;
}

void Button_Reset(ButtonHandle_t *button)
{
  if ((button == NULL) || (button->initialized == 0U))
  {
    return;
  }

  button->stableState = HAL_GPIO_ReadPin(button->port, button->pin);
  button->lastRawState = button->stableState;
  button->lastRawChangeTick = HAL_GetTick();
  button->pressTick = button->lastRawChangeTick;
  button->lastReleaseTick = button->lastRawChangeTick;
  button->clickCount = 0U;
  button->longReported = 0U;
}

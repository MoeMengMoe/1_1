#ifndef BUTTON_H
#define BUTTON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define BUTTON_DEFAULT_DEBOUNCE_MS      30U
#define BUTTON_DEFAULT_LONGPRESS_MS     800U
#define BUTTON_DEFAULT_MULTI_CLICK_MS   300U

typedef enum
{
  BUTTON_EVENT_NONE = 0,
  BUTTON_EVENT_SHORT_PRESS,
  BUTTON_EVENT_LONG_PRESS,
  BUTTON_EVENT_DOUBLE_CLICK,
  BUTTON_EVENT_TRIPLE_CLICK
} ButtonEvent_t;

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
  GPIO_PinState activeState;
  uint32_t debounceMs;
  uint32_t longPressMs;
  uint32_t multiClickMs;
  GPIO_PinState stableState;
  GPIO_PinState lastRawState;
  uint32_t lastRawChangeTick;
  uint32_t pressTick;
  uint32_t lastReleaseTick;
  uint8_t clickCount;
  uint8_t longReported;
  uint8_t initialized;
} ButtonHandle_t;

void Button_Init(ButtonHandle_t *button,
                 GPIO_TypeDef *port,
                 uint16_t pin,
                 GPIO_PinState activeState);

void Button_ConfigureTiming(ButtonHandle_t *button,
                            uint32_t debounceMs,
                            uint32_t longPressMs,
                            uint32_t multiClickMs);

ButtonEvent_t Button_Update(ButtonHandle_t *button, uint32_t nowMs);

void Button_Reset(ButtonHandle_t *button);

#ifdef __cplusplus
}
#endif

#endif /* BUTTON_H */

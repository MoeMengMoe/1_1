#include "encoder.h"

static uint8_t Encoder_ReadState(void);
static uint8_t lastEncoderState = 0U;

void Encoder_Init(void)
{
    lastEncoderState = Encoder_ReadState();
}

int8_t Encoder_ReadStep(void)
{
    static const int8_t encoderTable[4][4] =
    {
        { 0, -1,  1,  0},
        { 1,  0,  0, -1},
        {-1,  0,  0,  1},
        { 0,  1, -1,  0}
    };

    uint8_t currentState = Encoder_ReadState();
    int8_t delta = encoderTable[lastEncoderState][currentState];
    lastEncoderState = currentState;
    return delta;
}

void Encoder_ResetState(void)
{
    lastEncoderState = Encoder_ReadState();
}

static uint8_t Encoder_ReadState(void)
{
    uint8_t state = 0U;
    if (HAL_GPIO_ReadPin(ENC_A_GPIO_Port, ENC_A_Pin) == GPIO_PIN_SET)
    {
        state |= 0x01U;
    }
    if (HAL_GPIO_ReadPin(ENC_B_GPIO_Port, ENC_B_Pin) == GPIO_PIN_SET)
    {
        state |= 0x02U;
    }

    return state;
}

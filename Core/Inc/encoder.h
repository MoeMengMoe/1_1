#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include "main.h"

void Encoder_Init(void);
int8_t Encoder_ReadStep(void);
void Encoder_ResetState(void);

#endif /* ENCODER_H */

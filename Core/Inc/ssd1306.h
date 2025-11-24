#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>

#define SSD1306_WIDTH 128U
#define SSD1306_HEIGHT 64U

void SSD1306_Init(void);
void SSD1306_Fill(uint8_t color);
void SSD1306_UpdateScreen(void);
void SSD1306_SetCursor(uint8_t x, uint8_t y);
void SSD1306_WriteString(const char *text);
void SSD1306_WriteStringStyled(const char *text, uint8_t invert);

#endif // SSD1306_H

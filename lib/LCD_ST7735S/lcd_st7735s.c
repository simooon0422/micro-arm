#include "lcd_st7735s.h"

#define PIN_NUM_MOSI 23 // MOSI
#define PIN_NUM_CLK 18  // SCK
#define PIN_NUM_CS 5    // CS (chip select)
#define PIN_NUM_DC 19   // DC (data/command)
#define PIN_NUM_RST 27  // RST (reset)

#define ST7735_SWRESET 0x01 // Software Reset
#define ST7735_SLPOUT 0x11  // Sleep Out
#define ST7735_COLMOD 0x3A  // Interface Pixel Format
#define ST7735_MADCTL 0x36  // Memory Data Access Control
#define ST7735_DISPON 0x29  // Display ON

#define ST7735S_PIXEL_FORMAT_16BIT 0x05    // 16-bit format (RGB565)
#define ST7735S_MEMORY_ACCESS_DEFAULT 0xC8 // Default display orientation

#define ST7735_CASET 0x2A // Column Address Set
#define ST7735_RASET 0x2B // Row Address Set
#define ST7735_RAMWR 0x2C // Memory Write

#define TAG "ST7735"
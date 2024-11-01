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

#define ST7735_TAG "ST7735"

spi_device_handle_t spi; // Handle to SPI bus

esp_err_t init_spi()
{
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000, // SPI speed 10 MHz
        .mode = 0,                          // SPI mode 0
        .spics_io_num = PIN_NUM_CS,         // CS pin
        .queue_size = 7,                    // Transaction queue length
    };

    // Initialize SPI bus
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        ESP_LOGE(ST7735_TAG, "Initialization failed");
        return ret;
    }
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    if (ret != ESP_OK)
    {
        ESP_LOGE(ST7735_TAG, "Initialization failed");
        return ret;
    }

    return ret;
}

esp_err_t lcd_send_command(uint8_t cmd)
{
    // Transaction struct
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    gpio_set_level(PIN_NUM_DC, 0); // DC=0 for command

    esp_err_t ret = spi_device_transmit(spi, &t); // Transmit to the device
    if (ret != ESP_OK)
    {
        ESP_LOGE(ST7735_TAG, "Send command failed");
    }
    return ret;
}

esp_err_t lcd_send_data(uint8_t data)
{
    // Transaction struct
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
    };
    gpio_set_level(PIN_NUM_DC, 1); // DC=1 for data

    esp_err_t ret = spi_device_transmit(spi, &t); // Transmit to the device
    if (ret != ESP_OK)
    {
        ESP_LOGE(ST7735_TAG, "Send data failed");
    }
    return ret;
}

void st7735_init()
{
}

void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color)
{
}
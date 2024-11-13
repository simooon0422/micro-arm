#include "lcd_st7735s.h"

#define PIN_NUM_MOSI 23            // MOSI
#define PIN_NUM_CLK 18             // SCK
#define PIN_NUM_CS 5               // CS (chip select)
#define PIN_NUM_DC 19              // DC (data/command)
#define PIN_NUM_RST 27             // RST (reset)
#define MAX_SPI_TRANSFER_SIZE 4096 // Maximum transaction size

#define ST7735_SWRESET 0x01 // Software Reset
#define ST7735_SLPOUT 0x11  // Sleep Out
#define ST7735_COLMOD 0x3A  // Interface Pixel Format
#define ST7735_MADCTL 0x36  // Memory Data Access Control
#define ST7735_DISPON 0x29  // Display ON

#define ST7735S_PIXEL_FORMAT_16BIT 0x05    // 16-bit format (RGB565)
#define ST7735S_MEMORY_ACCESS_DEFAULT 0xA0 // Horizontal display orientation

#define ST7735_CASET 0x2A // Column Address Set
#define ST7735_RASET 0x2B // Row Address Set
#define ST7735_RAMWR 0x2C // Memory Write

#define LCD_OFFSET_X 1 // X offset of LCD display
#define LCD_OFFSET_Y 2 // Y offset of LCD display

#define ST7735_TAG "ST7735"

uint16_t lcd_display_buffer[LCD_WIDTH * LCD_HEIGHT] = {0x0000};

spi_device_handle_t spi; // Handle to SPI bus

esp_err_t init_spi()
{
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = MAX_SPI_TRANSFER_SIZE,
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

esp_err_t lcd_send_data16(uint16_t data)
{
    esp_err_t ret = lcd_send_data(data >> 8);
    if (ret != ESP_OK)
    {
        ESP_LOGE(ST7735_TAG, "Send data16 part 1 failed");
        return ret;
    }

    ret = lcd_send_data(data);
    if (ret != ESP_OK)
    {
        ESP_LOGE(ST7735_TAG, "Send data16 part 2 failed");
    }
    return ret;
}

void st7735_init()
{
    init_spi();

    // Set data and reset pins
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);

    // Reset dispaly
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Initialization sequence for ST7735
    lcd_send_command(ST7735_SWRESET); // Software Reset
    vTaskDelay(pdMS_TO_TICKS(150));

    lcd_send_command(ST7735_SLPOUT); // Sleep Out
    vTaskDelay(pdMS_TO_TICKS(500));

    lcd_send_command(ST7735_COLMOD);           // Interface Pixel Format
    lcd_send_data(ST7735S_PIXEL_FORMAT_16BIT); // 16-bit/pixel

    lcd_send_command(ST7735_MADCTL);              // Memory Data Access Control
    lcd_send_data(ST7735S_MEMORY_ACCESS_DEFAULT); // Set orientation

    lcd_send_command(ST7735_DISPON); // Display ON
    vTaskDelay(pdMS_TO_TICKS(100));

    lcd_send_buffer(); // Display default black screen
}

void lcd_set_box_borders(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{
    lcd_send_command(ST7735_CASET); // Column Address Set start X and end X
    lcd_send_data16(LCD_OFFSET_X + x);
    lcd_send_data16(LCD_OFFSET_X + x + width - 1);

    lcd_send_command(ST7735_RASET); // Row Address Set start Y and end Y
    lcd_send_data16(LCD_OFFSET_Y + y);
    lcd_send_data16(LCD_OFFSET_Y + y + height - 1);
}

void lcd_draw_box(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
    lcd_set_box_borders(x, y, width, height); // Set borders of the box

    lcd_send_command(ST7735_RAMWR); // Memory Write color
    for (int i = 0; i < width * height; i++)
    {
        lcd_send_data16(color);
    }
}

void lcd_put_pixel(void *_surface, int16_t x, int16_t y, uint16_t color)
{
    lcd_display_buffer[x + y * LCD_WIDTH] = color;
}

esp_err_t lcd_send_buffer()
{
    lcd_set_box_borders(0, 0, LCD_WIDTH, LCD_HEIGHT);
    lcd_send_command(ST7735_RAMWR); // Memory Write color

    int offset = 0;
    esp_err_t ret = 0;
    while (offset < sizeof(lcd_display_buffer))
    {
        int chunk_size = sizeof(lcd_display_buffer) - offset; // Calculate chunk size
        if (chunk_size > MAX_SPI_TRANSFER_SIZE)
        {
            chunk_size = MAX_SPI_TRANSFER_SIZE;
        }
        spi_transaction_t t = {
            .length = chunk_size * 8,
            .tx_buffer = lcd_display_buffer + offset / 2, // Divide offset by 2 because lcd_display_buffer is uint16_t
        };
        gpio_set_level(PIN_NUM_DC, 1); // DC=1 for data

        ret = spi_device_transmit(spi, &t); // Transmit to the device
        if (ret != ESP_OK)
        {
            ESP_LOGE(ST7735_TAG, "Send buffer failed");
            return ret;
        }
        offset += chunk_size; // Update offset
    }

    return ret;
}
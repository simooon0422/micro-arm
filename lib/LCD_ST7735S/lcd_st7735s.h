#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LCD_WIDTH 160  // Width of LCD display
#define LCD_HEIGHT 128 // Height of LCD display

/**
 * @brief Initialize SPI bus.
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t init_spi();

/**
 * @brief Send SPI command.
 * @param cmd Command to send.
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t lcd_send_command(uint8_t cmd);

/**
 * @brief Send data through SPI.
 * @param data Data to send.
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t lcd_send_data(uint8_t data);

/**
 * @brief Send 16 bit data through SPI.
 * @param data Data to send.
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t lcd_send_data16(uint16_t data);

/**
 * @brief Initialize display with st7735 driver.
 */
void st7735_init();

/**
 * @brief Set borders for box.
 * @param x X coordinate of box starting corner.
 * @param y Y coordinate of box starting corner.
 * @param width Width of box
 * @param height Height of box
 */
void lcd_set_box_borders(uint16_t x, uint16_t y, uint16_t width, uint16_t height);

/**
 * @brief Draw single pixel on display.
 * @param x X coordinate of box starting corner.
 * @param y Y coordinate of box starting corner.
 * @param width Width of box
 * @param height Height of box
 * @param color Color of pixel
 */
void lcd_draw_box(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);

/**
 * @brief Draw single pixel on display.
 * @param x X coordinate of pixel.
 * @param y Y coordinate of pixel.
 * @param color Color of pixel.
 */
void lcd_put_pixel(uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief Copy buffer to display.
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t lcd_send_buffer();

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

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
 * @brief Initialize display with st7735 driver.
 */
void st7735_init();

/**
 * @brief Draw single pixel on display.
 * @param x X coordinate of pixel.
 * @param y Y coordinate of pixel.
 * @param color Color of pixel
 */
void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t color);

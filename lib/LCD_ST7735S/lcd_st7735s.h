#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LCD_WIDTH   160  // Width of LCD display
#define LCD_HEIGHT  128 // Height of LCD display

// Colors
#define BLACK   0x0000
#define WHITE   0xffff
#define RED		0x00f8
#define GREEN	0xe007
#define YELLOW	0xe0ff

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
 * @param _surface Hagl backend.
 * @param x X coordinate of pixel.
 * @param y Y coordinate of pixel.
 * @param color Color of pixel.
 */
void lcd_put_pixel(void *_surface, int16_t x, int16_t y, uint16_t color);

/**
 * @brief Draw image on display.
 * @param _surface Hagl backend.
 * @param x0 X coordinate of image starting corner.
 * @param y0 Y coordinate of image starting corner.
 * @param width Width of image
 * @param height Height of image.
 * @param image Array with image pixels color.
 */
void lcd_draw_image(void *_surface, uint16_t x0, uint16_t y0, uint16_t width, uint16_t height, const uint16_t image[]);

/**
 * @brief Copy buffer to display.
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t lcd_send_buffer();

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include <string.h>

#include "pca9685.h"
#include "cd4051_analog.h"
#include "eeprom.h"
#include "lcd_st7735s.h"
#include "icons.h"

#include <hagl_hal.h>
#include <hagl.h>
#include "font6x9.h"
#include "rgb565.h"

#define LINKS_NUMBER 4         // Number of arm links
#define PATH_STEPS_ADDRESS 100 // Address of steps number in EEPROM

/**
 * @brief Map value to given scope.
 * @param x Value to map.
 * @param in_min Minimum input number
 * @param in_max Maximum input number.
 * @param out_min Minimum output number.
 * @param out_max Maximum output number.
 * @return Mapped value.
 */
int map(int, int in_min, int in_max, int out_min, int out_max);

/**
 * @brief Initialize pins for buttons.
 */
void buttons_init();

/**
 * @brief Initialize pins for diodes.
 */
void diodes_init();

/**
 * @brief Open gripper.
 */
void gripper_open();

/**
 * @brief Close gripper.
 */
void gripper_close();

/**
 * @brief Modify target positions with new array of positions.
 * @param new_target Array with new positions.
 * @param new_target_size Size of the new array
 */
void set_target_position(uint8_t new_target[LINKS_NUMBER], uint8_t new_target_size);

/**
 * @brief Move all servos to home position.
 */
void move_home();

/**
 * @brief Check if current servo positions matches target positions.
 * @param current Array with current position.
 * @param target Array with target position.
 * @param n Size of array.
 * @return Position status (True - current position matches target position, False - current position doesn't match target position).
 */
bool check_position(uint8_t current[], uint8_t target[], uint8_t n);

/**
 * @brief Get value of step for servo movement.
 * @return Step value.
 */
int get_step(uint8_t current, uint8_t target);

/**
 * @brief Move servo by step value.
 */
void move_step(uint8_t link);

/**
 * @brief Write path for automatic mode to EEPROM.
 * @param arr Array to write in EEPROM.
 * @param steps_n Number of steps to write.
 */
void write_auto_path(uint8_t arr[][LINKS_NUMBER + 1], uint8_t steps_n);

/**
 * @brief Read path for automatic mode from EEPROM.
 * @param arr Array to store readings from EEPROM.
 * @param steps_n Pointer to variable to store number of path steps
 */
void read_auto_path(uint8_t arr[][LINKS_NUMBER + 1], uint8_t *steps_n);

/**
 * @brief Gripper button interrupt handler.
 */
static void isr_gripper_handler();

/**
 * @brief Mode button interrupt handler.
 */
static void isr_mode_handler();

/**
 * @brief Calculate number of digits in number.
 * @param n Number
 * @return Number of digits.
 */
uint8_t get_num_digits(uint8_t n);

/**
 * @brief Convert number to text displayable on LCD screen.
 * @param text_arr Array to store converted number.
 * @param x Number for conversion.
 */
void get_text(wchar_t text_arr[], uint8_t x);

/**
 * @brief Display icons on LCD screen.
 * @param display Hagl backend pointer
 */
void show_icons(hagl_backend_t *display);

/**
 * @brief Display headers for values on LCD screen.
 * @param display Hagl backend pointer
 */
void show_headers(hagl_backend_t *display);

/**
 * @brief Display single position array values.
 * @param display Hagl backend pointer.
 * @param y0 Screen height of displayed values.
 * @param arr Array of positions to display.
 */
void show_position_array(hagl_backend_t *display, uint8_t arr[], uint8_t y0);

/**
 * @brief Display all target, current and potentiometers position values on LCD screen.
 * @param display Hagl backend pointer.
 */
void show_positions(hagl_backend_t *display);

/**
 * @brief Task for reading values from control potentiometers.
 */
void read_potentiometers_task(void *pvParameter);

/**
 * @brief Task for controlling servo movement.
 */
void servo_control_task(void *pvParameter);

/**
 * @brief Task for controlling position flag.
 */
void position_control_task(void *pvParameter);

/**
 * @brief Task for controlling LCD display.
 */
void lcd_display_task(void *pvParameter);

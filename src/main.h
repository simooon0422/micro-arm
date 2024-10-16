#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include <string.h>

#include "pca9685.h"
#include "cd4051_analog.h"
#include "eeprom.h"

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
 * @brief Open gripper.
 */
void gripper_open();

/**
 * @brief Close gripper.
 */
void gripper_close();

/**
 * @brief Move all servos to home position.
 */
void move_home();

/**
 * @brief Check if current servo positions matches target positions.
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

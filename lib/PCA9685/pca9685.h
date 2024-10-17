#include "driver/i2c.h"
#include "esp_log.h"

#define PCA9685_ADDR 0x40 // Default PCA9685 address

/**
 * @brief Initialize I2C.
 */
esp_err_t i2c_master_init();

/**
 * @brief Write one byte through I2C.
 */
esp_err_t i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

/**
 * @brief Set PWM frequency for PCA9685.
 * @param freq Desired PWM frequency.
 */
void pca9685_set_pwm_freq(uint16_t freq);

/**
 * @brief Initialize PCA9685.
 */
void pca9685_init(void);

/**
 * @brief Set pulse width on the channel.
 * @param channel Channel (0-15).
 * @param on Pulse start (0-4095).
 * @param off Pulse end (0-4095).
 */
void pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off);

/**
 * @brief Calculate angle to PWM pulse.
 * @param angle Angle in degrees (0-180).
 * @return PWM pulse for PCA9685.
 */
uint16_t servo_angle_to_pwm(uint16_t angle);

/**
 * @brief Set desired angle on the channel.
 * @param channel PCA9685 channel (0-15).
 * @param angle Desired angle in degrees (0-180).
 */
void pca9685_set_servo_angle(uint8_t channel, uint16_t angle);
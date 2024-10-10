#include "driver/i2c.h"
#include "esp_log.h"

/**
 * @brief Initialize I2C communication with EEPROM.
 * @return ESP_OK on success, Error code on fail.
 */
esp_err_t eeprom_init();

/**
 * @brief Write byte to EEPROM memory.
 * @param address Address in EEPROM memory.
 * @param data Data to write in memory.
 * @return ESP_OK on success, Error code on fail.
 */
esp_err_t eeprom_write_byte(uint8_t address, uint8_t data);

/**
 * @brief Read byte from EEPROM memory.
 * @param address Address in EEPROM memory.
 * @param data Pointer to variable storing reading from EEPROM.
 * @return ESP_OK on success, Error code on fail.
 */
esp_err_t eeprom_read_byte(uint8_t address, uint8_t *data);
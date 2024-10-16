#include "eeprom.h"

#define I2C_MASTER_SCL_IO 17      // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO 16      // GPIO number for I2C master data
#define I2C_MASTER_FREQ_HZ 100000 // I2C master clock frequency
#define I2C_MASTER_NUM I2C_NUM_1   // I2C port number for master dev
#define I2C_MASTER_TIMEOUT_MS 1000 // I2C timeout in ms

#define EEPROM_24AA01_ADDRESS 0xA0  // Default EEPROM 24AA01 address

static const char *EEPROM_TAG = "EEPROM";

esp_err_t eeprom_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(EEPROM_TAG, "I2C parameter configuration failed");
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(EEPROM_TAG, "I2C driver installation failed");
    }

    return ret;
}

esp_err_t eeprom_write_byte(uint8_t address, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    
    // Set write mode and send address, then data
    i2c_master_write_byte(cmd, EEPROM_24AA01_ADDRESS | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, address, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(EEPROM_TAG, "Failed to write byte to EEPROM: %s", esp_err_to_name(err));
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    return err;
}

esp_err_t eeprom_read_byte(uint8_t address, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    
    // Set write mode and send address
    i2c_master_write_byte(cmd, EEPROM_24AA01_ADDRESS | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, address, true);

    // Set read mode
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, EEPROM_24AA01_ADDRESS | I2C_MASTER_READ, true);

    // Read from sent address
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(EEPROM_TAG, "Failed to read byte from EEPROM: %s", esp_err_to_name(err));
    }

    return err;
}

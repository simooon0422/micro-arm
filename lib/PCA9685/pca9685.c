#include "pca9685.h"

#define I2C_MASTER_SCL_IO 22      //GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO 21      //GPIO number for I2C master data
#define I2C_MASTER_FREQ_HZ 100000 //I2C master clock frequency
#define I2C_MASTER_NUM I2C_NUM_0  //I2C port number for master dev
#define I2C_MASTER_TIMEOUT_MS 1000 //I2C timeout in ms

#define PCA9685_ADDR 0x40 // Default PCA9685 address
#define SERVO_MIN_US 500  // Minimum pulse width in µs
#define SERVO_MAX_US 2600 // Maximum pulse width in µs
#define PWM_FREQ 50       // PWM frequency for SG90 servo

void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

esp_err_t i2c_write_byte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

void pca9685_set_pwm_freq(uint16_t freq)
{
    uint8_t prescale_val = (uint8_t)(25000000 / (4096 * freq)) - 1; // Calculate prescaler value
    i2c_write_byte(PCA9685_ADDR, 0x00, 0x10);                       // Enter sleep mode
    i2c_write_byte(PCA9685_ADDR, 0xFE, prescale_val);               // Set prescaler
    i2c_write_byte(PCA9685_ADDR, 0x00, 0x80);                       // Exit sleep mode
    i2c_write_byte(PCA9685_ADDR, 0x01, 0x04);                       // Enable auto-increment
}

void pca9685_init(void)
{
    i2c_write_byte(PCA9685_ADDR, 0x00, 0x00); // Reset PCA9685
    pca9685_set_pwm_freq(PWM_FREQ);           // Set PWM frequency to 50Hz
}

void pca9685_set_pwm(uint8_t channel, uint16_t on, uint16_t off)
{
    uint8_t reg_base = 0x06 + 4 * channel;                  // Base register for each channel
    i2c_write_byte(PCA9685_ADDR, reg_base, on & 0xFF);      // Set lower byte ON
    i2c_write_byte(PCA9685_ADDR, reg_base + 1, on >> 8);    // Set higher byte ON
    i2c_write_byte(PCA9685_ADDR, reg_base + 2, off & 0xFF); // Set lower byte OFF
    i2c_write_byte(PCA9685_ADDR, reg_base + 3, off >> 8);   // Set higher byte OFF
}

uint16_t servo_angle_to_pwm(uint16_t angle)
{
    uint16_t pulse_width = SERVO_MIN_US + ((SERVO_MAX_US - SERVO_MIN_US) * angle) / 180;
    return (uint16_t)(pulse_width / 4.88); // Calculate number of PWM steps
}

void pca9685_set_servo_angle(uint8_t channel, uint16_t angle)
{
    uint16_t pwm_value = servo_angle_to_pwm(angle);
    pca9685_set_pwm(channel, 0, pwm_value);
}
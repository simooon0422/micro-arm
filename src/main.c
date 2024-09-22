#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "pca9685.h"

static const char *TAG = "ServoControl";
uint8_t links_number = 4;
uint8_t gripper_channel = 4;

uint16_t home_position[] = {90, 135, 30, 90};
uint16_t current_position[4];
uint16_t target_position[] = {30, 90, 0, 135};

void gripper_open()
{
    pca9685_set_servo_angle(gripper_channel, 0);
    ESP_LOGI(TAG, "Gripper is open");
}

void gripper_close()
{
    pca9685_set_servo_angle(gripper_channel, 180);
    ESP_LOGI(TAG, "Gripper is closed");
}

void move_home()
{
    for (int i = 0; i < links_number; i++)
    {
        pca9685_set_servo_angle(i, home_position[i]);
    }
    for (int i = 0; i < links_number; i++)
    {
        current_position[i] = home_position[i];
    }

    ESP_LOGI(TAG, "Homing completed, current position:");
    for (int i = 0; i < links_number; i++)
    {
        ESP_LOGI(TAG, "Servo %d: %d", i, current_position[i]);
    }
}

void servo_control_task(void *pvParameter)
{
    move_home();
    while (1)
    {
        ESP_LOGI(TAG, "Target position:");
        for (int i = 0; i < links_number; i++)
        {
            ESP_LOGI(TAG, "Servo %d: %d", i, target_position[i]);
        }

        for (int i = 0; i < links_number; i++)
        {
            while (current_position[i] != target_position[i])
            {
                if (target_position[i] - current_position[i] > 0)
                {
                    pca9685_set_servo_angle(i, current_position[i] + 1);
                    current_position[i] = current_position[i] + 1;
                }
                else if (target_position[i] - current_position[i] < 0)
                {
                    pca9685_set_servo_angle(i, current_position[i] - 1);
                    current_position[i] = current_position[i] - 1;
                }
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }

        ESP_LOGI(TAG, "Target reached");

        gripper_close();

        vTaskDelay(pdMS_TO_TICKS(2000));

        gripper_open();

        vTaskDelay(pdMS_TO_TICKS(2000));

        for (int i = 0; i < links_number; i++)
        {
            target_position[i] = home_position[i];
        }
    }
}

void app_main(void)
{
    i2c_master_init(); // Initialize I2C
    pca9685_init();    // Initialize PCA9685

    xTaskCreate(&servo_control_task, "blink_led_task", 2048, NULL, 5, NULL);
}

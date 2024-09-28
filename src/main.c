#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "pca9685.h"
#include "cd4051_analog.h"

static const char *SERVO_TAG = "ServoControl";
uint8_t links_number = 4;
uint8_t gripper_channel = 4;
bool position_flag = 0;

uint16_t home_position[] = {90, 135, 30, 90};
uint16_t current_position[4];
uint16_t target_position[] = {30, 90, 0, 135};

void gripper_open()
{
    pca9685_set_servo_angle(gripper_channel, 0);
    ESP_LOGI(SERVO_TAG, "Gripper is open");
}

void gripper_close()
{
    pca9685_set_servo_angle(gripper_channel, 180);
    ESP_LOGI(SERVO_TAG, "Gripper is closed");
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

    ESP_LOGI(SERVO_TAG, "Homing completed, current position:");
    for (int i = 0; i < links_number; i++)
    {
        ESP_LOGI(SERVO_TAG, "Servo %d: %d", i, current_position[i]);
    }
}

void read_potentiometers_task(void *pvParameter)
{
    cd4051_init();
    uint16_t readings[5] = {0, 0, 0, 0, 0};

    while(1)
    {
        for (int i = 0; i < 5; i++)
        {
            readings[i] = cd4051_read_channel(i);
        }
        
        // readings[0] = cd4051_read_channel(0);
        // readings[1] = cd4051_read_channel(1);
        // readings[2] = cd4051_read_channel(2);
        // readings[3] = cd4051_read_channel(3);
        // readings[4] = cd4051_read_channel(4);
        ESP_LOGI(SERVO_TAG, "Readings: %d, %d, %d, %d, %d ", readings[0], readings[1], readings[2], readings[3], readings[4]);

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void servo_control_task(void *pvParameter)
{
    move_home();
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1)
    {
        if (position_flag == 0)
        {
            ESP_LOGI(SERVO_TAG, "Target position:");
            for (int i = 0; i < links_number; i++)
            {
                ESP_LOGI(SERVO_TAG, "Servo %d: %d", i, target_position[i]);
            }

            int move_step = 0;

            for (int i = 0; i < links_number; i++)
            {
                if (current_position[i] != target_position[i])
                {
                    move_step = (target_position[i] - current_position[i]) / abs(target_position[i] - current_position[i]);
                    pca9685_set_servo_angle(i, current_position[i] + move_step);
                    current_position[i] = current_position[i] + move_step;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(20));

            for (int i = 0; i < links_number; i++)
            {
                if (current_position[i] != target_position[i])
                {
                    position_flag = 0;
                    break;
                }     
                position_flag = 1;         
                ESP_LOGI(SERVO_TAG, "Target reached");
            }
        }

        if (position_flag == 1)
        {
            vTaskDelay(pdMS_TO_TICKS(2000));
            for (int i = 0; i < links_number; i++)
            {
                target_position[i] = home_position[i];
            }
            position_flag = 0;
        }
    }
}

void app_main(void)
{
    // i2c_master_init(); // Initialize I2C
    // pca9685_init();    // Initialize PCA9685

    // xTaskCreate(&servo_control_task, "servo_control_task", 2048, NULL, 5, NULL);
    xTaskCreate(&read_potentiometers_task, "read_potentiometers_task", 4096, NULL, 5, NULL);
}

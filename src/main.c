#include "main.h"

static const char *SERVO_TAG = "ServoControl";
uint8_t links_number = 4;
uint8_t gripper_channel = 4;
bool position_flag = 1;

uint8_t home_position[] = {90, 135, 15, 30};
uint8_t current_position[] = {90, 135, 30, 90};
uint8_t target_position[] = {90, 135, 30, 90};

SemaphoreHandle_t xMutexTargetPosition = NULL;

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
    if (x < in_min)
    {
        x = in_min;
    }
    else if (x > in_max)
    {
        x = in_max;
    }
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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

    ESP_LOGI(SERVO_TAG, "Homing completed, current position:");
    for (int i = 0; i < links_number; i++)
    {
        ESP_LOGI(SERVO_TAG, "Servo %d: %d", i, current_position[i]);
    }
}

bool check_position(uint8_t current[], uint8_t target[], uint8_t n)
{
    for (int i = 0; i < n; i++)
    {
        if (current[i] != target[i])
        {
            return false;
        }
    }
    return true;
}

int get_step(uint8_t current, uint8_t target)
{
    if (current != target)
    {
        return ((target - current) / abs(target - current));
    }
    else
        return 0;
}

void move_step(uint8_t link)
{
    int step = get_step(current_position[link], target_position[link]);
    pca9685_set_servo_angle(link, current_position[link] + step);
    current_position[link] = current_position[link] + step;
}

void servo_control_task(void *pvParameter)
{
    move_home(); // Move all servos to home position
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1)
    {
        if (position_flag == 0)
        {
            if (xSemaphoreTake(xMutexTargetPosition, (TickType_t)10) == pdTRUE)
            {
                ESP_LOGI(SERVO_TAG, "Target position:");
                for (int i = 0; i < links_number; i++)
                {
                    ESP_LOGI(SERVO_TAG, "Servo %d: %d", i, target_position[i]);
                }

                for (int i = 0; i < links_number; i++)
                {
                    move_step(i);
                }

                // if (check_position(current_position, target_position, links_number) == true)
                // {
                //     position_flag = 1;
                //     ESP_LOGI(SERVO_TAG, "Target reached");
                // }
                xSemaphoreGive(xMutexTargetPosition);
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
    }
}

void read_potentiometers_task(void *pvParameter)
{
    while (1)
    {
        if (xSemaphoreTake(xMutexTargetPosition, (TickType_t)10) == pdTRUE)
        {
            for (int i = 0; i < links_number; i++)
            {
                target_position[i] = 10 * map(cd4051_read_channel(i), 0, 4095, 0, 18);
            }
            // if ((position_flag == 1) & (check_position(current_position, target_position, links_number) == false))
            // {
            //     position_flag = 0;
            // }
            ESP_LOGI(SERVO_TAG, "Readings: %d, %d, %d, %d", target_position[0], target_position[1], target_position[2], target_position[3]);
            xSemaphoreGive(xMutexTargetPosition);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void position_control_task(void *pvParameter)
{
    while (1)
    {
        if (xSemaphoreTake(xMutexTargetPosition, (TickType_t)10) == pdTRUE)
        {
            if ((position_flag == 1) & (check_position(current_position, target_position, links_number) == false))
            {
                position_flag = 0;
            }
            else if ((position_flag == 0) & (check_position(current_position, target_position, links_number) == true))
            {
                position_flag = 1;
            }
            xSemaphoreGive(xMutexTargetPosition);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

#ifndef TESTING_ENVIRONMENT
void app_main(void)
{
    xMutexTargetPosition = xSemaphoreCreateMutex();

    i2c_master_init(); // Initialize I2C
    pca9685_init();    // Initialize PCA9685
    cd4051_init();     // Initialize CD4051 multiplexer

    xTaskCreate(&servo_control_task, "servo_control_task", 2048, NULL, 5, NULL);
    xTaskCreate(&read_potentiometers_task, "read_potentiometers_task", 4096, NULL, 5, NULL);
    xTaskCreate(&position_control_task, "position_control_task", 2048, NULL, 5, NULL);
}
#endif

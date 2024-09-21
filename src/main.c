#include "pca9685.h"

uint16_t home_position[] = {90, 135, 30, 90, 0};
uint16_t current_position[5];
uint16_t target_position[] = {30, 90, 0, 135, 180};

void move_home()
{
    for (int i = 0; i < 5; i++)
    {
        pca9685_set_servo_angle(i, home_position[i]);
    }
    for (int i = 0; i < 5; i++)
    {
        current_position[i] = home_position[i];
    }
}
void sample_move()
{
    for (int i = 0; i < 5; i++)
    {
        while (current_position[i] != target_position[i])
        {
            if (target_position[i] - current_position[i] > 0)
            {
                pca9685_set_servo_angle(i, current_position[i] + 1);
                current_position[i] = current_position[i] + 1;
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            else if (target_position[i] - current_position[i] < 0)
            {
                pca9685_set_servo_angle(i, current_position[i] - 1);
                current_position[i] = current_position[i] - 1;
                vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
    }

    vTaskDelay(pdMS_TO_TICKS(2000));

    for (int i = 0; i < 5; i++)
    {
        target_position[i] = home_position[i];
    }
}

void app_main(void)
{
    i2c_master_init(); // Initialize I2C
    pca9685_init();    // Initialize PCA9685

    move_home();

    while (true)
    {
        sample_move();
    }
}

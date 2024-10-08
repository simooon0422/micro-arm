#include "main.h"

#define GRIPPER_BUTTON_PIN GPIO_NUM_13 // Gripper control button pin
#define MODE_BUTTON_PIN GPIO_NUM_12    // Mode control button pin
#define LINKS_NUMBER 4                 // Number of arm links
#define GRIPPER_CHANNEL 4              // PCA9685 channel for gripper
#define MODES_NUMBER 2                 // Number of arm modes of working

static const char *SERVO_TAG = "ServoControl";
static const char *POT_TAG = "Potentiometers";
static const char *GRIPPER_TAG = "Gripper";

bool movement_flag = 0; // 0 - no movement required, 1 - movement required
bool gripper_flag = 0;  // 0 - gripper open, 1 - gripper closed
uint8_t mode_flag = 0;  // 0 - Home mode, 1 - Manual mode

uint8_t home_position[LINKS_NUMBER] = {90, 135, 15, 30};    // Angle values of servo home positions
uint8_t current_position[LINKS_NUMBER] = {90, 135, 15, 30}; // Angle values of servo current positions
uint8_t target_position[LINKS_NUMBER] = {90, 135, 15, 30};  // Angle values of servo target positions
uint8_t pot_readings[LINKS_NUMBER];                         // Readings from potentiometers mapped to angle values

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

void buttons_init(void)
{
    // Set up gripper button pin
    gpio_reset_pin(GRIPPER_BUTTON_PIN);
    gpio_set_direction(GRIPPER_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GRIPPER_BUTTON_PIN, GPIO_PULLUP_ONLY);

    // Set up mode button pin
    gpio_reset_pin(MODE_BUTTON_PIN);
    gpio_set_direction(MODE_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(MODE_BUTTON_PIN, GPIO_PULLUP_ONLY);
}

void gripper_open()
{
    pca9685_set_servo_angle(GRIPPER_CHANNEL, 0);
    ESP_LOGI(GRIPPER_TAG, "Gripper is open");
    gripper_flag = 0;
}

void gripper_close()
{
    pca9685_set_servo_angle(GRIPPER_CHANNEL, 180);
    ESP_LOGI(GRIPPER_TAG, "Gripper is closed");
    gripper_flag = 1;
}

void move_home()
{
    for (int i = 0; i < LINKS_NUMBER; i++)
    {
        pca9685_set_servo_angle(i, home_position[i]);
    }

    ESP_LOGI(SERVO_TAG, "Homing completed, current position:");
    for (int i = 0; i < LINKS_NUMBER; i++)
    {
        ESP_LOGI(SERVO_TAG, "Servo %d: %d", i, current_position[i]);
    }

    gripper_open();
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
        if (movement_flag == 1)
        {
            if (xSemaphoreTake(xMutexTargetPosition, (TickType_t)10) == pdTRUE)
            {
                ESP_LOGI(SERVO_TAG, "Target position:");
                for (int i = 0; i < LINKS_NUMBER; i++)
                {
                    ESP_LOGI(SERVO_TAG, "Servo %d: %d", i, target_position[i]);
                }

                for (int i = 0; i < LINKS_NUMBER; i++)
                {
                    move_step(i);
                }

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
        for (int i = 0; i < LINKS_NUMBER; i++)
        {
            pot_readings[i] = 10 * map(cd4051_read_channel(i), 0, 4095, 0, 18);
        }
        if (mode_flag == 1)
        {
            if (xSemaphoreTake(xMutexTargetPosition, (TickType_t)10) == pdTRUE)
            {
                memcpy(target_position, pot_readings, sizeof(pot_readings));
                xSemaphoreGive(xMutexTargetPosition);
            }
        }
        ESP_LOGI(POT_TAG, "Readings: %d, %d, %d, %d", pot_readings[0], pot_readings[1], pot_readings[2], pot_readings[3]);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void position_control_task(void *pvParameter)
{
    while (1)
    {
        if (xSemaphoreTake(xMutexTargetPosition, (TickType_t)10) == pdTRUE)
        {
            if ((movement_flag == 0) & (check_position(current_position, target_position, LINKS_NUMBER) == false))
            {
                movement_flag = 1;
            }
            else if ((movement_flag == 1) & (check_position(current_position, target_position, LINKS_NUMBER) == true))
            {
                movement_flag = 0;
            }
            xSemaphoreGive(xMutexTargetPosition);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

void gripper_control_task(void *pvParameter)
{
    while (1)
    {
        if (gpio_get_level(GRIPPER_BUTTON_PIN) == 0)
        {
            if (gripper_flag == 0)
            {
                gripper_close();
            }
            else
                gripper_open();

            vTaskDelay(pdMS_TO_TICKS(500));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void mode_control_task(void *pvParameter)
{
    while (1)
    {
        if (gpio_get_level(MODE_BUTTON_PIN) == 0)
        {
            mode_flag++;
            if (mode_flag >= MODES_NUMBER)
            {
                mode_flag = 0;
                if (xSemaphoreTake(xMutexTargetPosition, portMAX_DELAY) == pdTRUE)
                {
                    memcpy(target_position, home_position, sizeof(home_position));
                    xSemaphoreGive(xMutexTargetPosition);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

#ifndef TESTING_ENVIRONMENT
void app_main(void)
{
    xMutexTargetPosition = xSemaphoreCreateMutex();

    i2c_master_init(); // Initialize I2C
    pca9685_init();    // Initialize PCA9685
    cd4051_init();     // Initialize CD4051 multiplexer
    buttons_init();    // Initialize buttons

    xTaskCreate(&servo_control_task, "servo_control_task", 2048, NULL, 2, NULL);
    xTaskCreate(&read_potentiometers_task, "read_potentiometers_task", 4096, NULL, 3, NULL);
    xTaskCreate(&position_control_task, "position_control_task", 2048, NULL, 4, NULL);
    xTaskCreate(&gripper_control_task, "gripper_control_task", 2048, NULL, 1, NULL);
    xTaskCreate(&mode_control_task, "mode_control_task", 2048, NULL, 4, NULL);
}
#endif

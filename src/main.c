#include "main.h"

// #define WRITE_EEPROM_ON_START // Uncomment if you want to modify automatic path in EEPROM on start

#define GRIPPER_BUTTON_PIN GPIO_NUM_13 // Gripper control button pin
#define MODE_BUTTON_PIN GPIO_NUM_12    // Mode control button pin
#define GREEN_LED_PIN GPIO_NUM_4       // Greed diode pin
#define YELLOW_LED_PIN GPIO_NUM_2      // Yellow diode pin
#define RED_LED_PIN GPIO_NUM_15        // Red diode pin

#define GRIPPER_CHANNEL 4                // PCA9685 channel for gripper
#define MODES_NUMBER 3                   // Number of arm's modes of working
#define MAX_PATH_STEPS 10                // Maximum number of steps in automatic mode path
#define DEBOUNCE_TIME pdMS_TO_TICKS(500) // Time for debouncing buttons in ms
#define ESP_INTR_FLAG_DEFAULT 0          // Flags used to allocate the interrupt

static const char *SERVO_TAG = "ServoControl";
static const char *POT_TAG = "Potentiometers";
static const char *GRIPPER_TAG = "Gripper";

bool movement_flag = 0;               // 0 - no movement required, 1 - movement required
bool gripper_flag = 0;                // 0 - gripper open, 1 - gripper closed
volatile bool isr_gripper_update = 0; // 0 - gripper position update not required, 1 - gripper position update required
TickType_t last_gripper_time = 0;     // Last time of gripper interrupt

enum Modes // Modes of working
{
    HOME = 0,
    MANUAL = 1,
    AUTO = 2
};
typedef enum Modes Modes_t;

volatile Modes_t mode_flag = 0;    // 0 - Home mode, 1 - Manual mode, 2 - Automatic mode
volatile bool isr_mode_update = 1; // 0 - no mode update required, 1 - mode update required
TickType_t last_mode_time = 0;     // Last time of mode interrupt

uint8_t path_steps = 0; // Number of path steps for automatic mode, default 0

uint8_t home_position[LINKS_NUMBER] = {90, 135, 30, 30};    // Angle values of servo home positions
uint8_t current_position[LINKS_NUMBER] = {90, 135, 30, 30}; // Angle values of servo current positions
uint8_t target_position[LINKS_NUMBER] = {90, 135, 30, 30};  // Angle values of servo target positions
uint8_t pot_readings[LINKS_NUMBER];                         // Readings from potentiometers mapped to angle values

uint8_t work_path[MAX_PATH_STEPS][LINKS_NUMBER + 1] = {0};

#ifdef WRITE_EEPROM_ON_START
#define WRITE_PATH_STEPS 3 // Number of steps for sequence in automatic mode to write in EEPROM
uint8_t write_path[WRITE_PATH_STEPS][LINKS_NUMBER + 1] = {
    {90, 135, 30, 30},
    {0, 135, 30, 30},
    {90, 135, 30, 30}};
#endif

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

void buttons_init()
{
    // Set up gripper button pin
    gpio_reset_pin(GRIPPER_BUTTON_PIN);
    gpio_set_direction(GRIPPER_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GRIPPER_BUTTON_PIN, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(GRIPPER_BUTTON_PIN, GPIO_INTR_NEGEDGE);

    // Set up mode button pin
    gpio_reset_pin(MODE_BUTTON_PIN);
    gpio_set_direction(MODE_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(MODE_BUTTON_PIN, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(MODE_BUTTON_PIN, GPIO_INTR_NEGEDGE);
}

void diodes_init()
{
    gpio_reset_pin(GREEN_LED_PIN);
    gpio_reset_pin(YELLOW_LED_PIN);
    gpio_reset_pin(RED_LED_PIN);

    gpio_set_direction(GREEN_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(YELLOW_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(RED_LED_PIN, GPIO_MODE_OUTPUT);
}

void gripper_open()
{
    pca9685_set_servo_angle(GRIPPER_CHANNEL, 0);
    gripper_flag = 0;
}

void gripper_close()
{
    pca9685_set_servo_angle(GRIPPER_CHANNEL, 180);
    gripper_flag = 1;
}

void set_target_position(uint8_t new_target[LINKS_NUMBER], uint8_t new_target_size)
{
    if (xSemaphoreTake(xMutexTargetPosition, portMAX_DELAY) == pdTRUE)
    {
        memcpy(target_position, new_target, new_target_size);
        xSemaphoreGive(xMutexTargetPosition);
    }
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

void write_auto_path(uint8_t arr[][LINKS_NUMBER + 1], uint8_t steps_n)
{
    eeprom_write_byte(PATH_STEPS_ADDRESS, steps_n);
    for (int i = 0; i < steps_n; i++)
    {
        for (int j = 0; j < LINKS_NUMBER + 1; j++)
        {
            eeprom_write_byte(j + (i * (LINKS_NUMBER + 1)), arr[i][j]);
        }
    }
}

void read_auto_path(uint8_t arr[][LINKS_NUMBER + 1], uint8_t *steps_n)
{
    eeprom_read_byte(PATH_STEPS_ADDRESS, steps_n);
    for (int i = 0; i < *steps_n; i++)
    {
        for (int j = 0; j < LINKS_NUMBER + 1; j++)
        {
            eeprom_read_byte(j + (i * (LINKS_NUMBER + 1)), &arr[i][j]);
        }
    }
}

static void isr_gripper_handler()
{
    if (xTaskGetTickCountFromISR() - last_gripper_time >= DEBOUNCE_TIME)
    {
        isr_gripper_update = 1;
        last_gripper_time = xTaskGetTickCountFromISR();
    }
}

static void isr_mode_handler()
{
    if (xTaskGetTickCountFromISR() - last_mode_time >= DEBOUNCE_TIME)
    {
        isr_mode_update = 1;
        mode_flag++;
        if (mode_flag >= MODES_NUMBER)
        {
            mode_flag = 0;
        }
        last_mode_time = xTaskGetTickCountFromISR();
    }
}

void servo_control_task(void *pvParameter)
{
    move_home(); // Move all servos to home position on start
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1)
    {
        // Execute only if movement is allowed
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
        else
            vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void read_potentiometers_task(void *pvParameter)
{
    while (1)
    {
        // Read and map analog values
        for (int i = 0; i < LINKS_NUMBER; i++)
        {
            pot_readings[i] = 10 * map(cd4051_read_channel(i), 0, 4095, 0, 18);
        }

        // Update target position only in mode 1
        if (mode_flag == 1)
        {
            set_target_position(pot_readings, sizeof(pot_readings));
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
            // Set movement flag to 1 if current position and target position are different
            if ((movement_flag == 0) & (check_position(current_position, target_position, LINKS_NUMBER) == false))
            {
                movement_flag = 1;
            }
            // Set movement flag to 0 if current position and target position are the same
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
        if (isr_gripper_update == 1)
        {
            if (gripper_flag == 0)
            {
                gripper_close();
                ESP_LOGI(GRIPPER_TAG, "Gripper is closed");
            }
            else
            {
                gripper_open();
                ESP_LOGI(GRIPPER_TAG, "Gripper is open");
            }

            isr_gripper_update = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void mode_control_task(void *pvParameter)
{
    while (1)
    {
        if (isr_mode_update == 1) // Update mode only if user pressed mode button
        {
            switch (mode_flag)
            {
            case HOME: // Home mode - set home position and light green diode
                set_target_position(home_position, sizeof(home_position));
                gpio_set_level(GREEN_LED_PIN, 1);
                gpio_set_level(YELLOW_LED_PIN, 0);
                gpio_set_level(RED_LED_PIN, 0);
                break;

            case MANUAL: // Manual mode - light yellow diode
                gpio_set_level(GREEN_LED_PIN, 0);
                gpio_set_level(YELLOW_LED_PIN, 1);
                gpio_set_level(RED_LED_PIN, 0);
                break;

            case AUTO: // Automatic mode - light red diode
                gpio_set_level(GREEN_LED_PIN, 0);
                gpio_set_level(YELLOW_LED_PIN, 0);
                gpio_set_level(RED_LED_PIN, 1);
                break;

            default:
                break;
            }

            isr_mode_update = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void automatic_sequence_task(void *pvParameter)
{
    while (1)
    {
        if ((mode_flag == 2) && (path_steps <= MAX_PATH_STEPS))
        {
            // Update position of links
            for (int i = 0; i < path_steps; i++)
            {
                if (xSemaphoreTake(xMutexTargetPosition, portMAX_DELAY) == pdTRUE)
                {
                    for (int j = 0; j < LINKS_NUMBER; j++)
                    {
                        target_position[j] = work_path[i][j];
                    }
                    xSemaphoreGive(xMutexTargetPosition);
                }

                // If mode has changed in the meantime -> break the loop
                if (mode_flag != 2)
                {
                    set_target_position(home_position, sizeof(home_position));
                    break;
                }

                // Update position of gripper
                if (work_path[i][LINKS_NUMBER] == 1)
                {
                    gripper_close();
                }
                else
                    gripper_open();

                // Wait for servos to finish moving
                vTaskDelay(100);
                while (movement_flag == 1)
                {
                }
            }
        }
        else
            vTaskDelay(200);
    }
}

#ifndef TESTING_ENVIRONMENT
void app_main(void)
{
    eeprom_init();  // Initialize EEPROM on I2C1
    pca9685_init(); // Initialize PCA9685 on I2C0
    cd4051_init();  // Initialize CD4051 multiplexer
    buttons_init(); // Initialize buttons
    diodes_init();  // Initialize diodes

#ifdef WRITE_EEPROM_ON_START
    write_auto_path(write_path, WRITE_PATH_STEPS); // Write automatic path to EEPROM
#endif
    read_auto_path(work_path, &path_steps); // Read automatic path from EEPROM

    xMutexTargetPosition = xSemaphoreCreateMutex(); // Create mutex for target_position

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);                     // Allow per-pin GPIO interrupt handlers
    gpio_isr_handler_add(GRIPPER_BUTTON_PIN, isr_gripper_handler, NULL); // Gripper interrupt
    gpio_isr_handler_add(MODE_BUTTON_PIN, isr_mode_handler, NULL);       // Mode change interrupt

    xTaskCreate(&servo_control_task, "servo_control_task", 2048, NULL, 2, NULL);
    xTaskCreate(&read_potentiometers_task, "read_potentiometers_task", 4096, NULL, 3, NULL);
    xTaskCreate(&position_control_task, "position_control_task", 2048, NULL, 4, NULL);
    xTaskCreate(&gripper_control_task, "gripper_control_task", 2048, NULL, 1, NULL);
    xTaskCreate(&mode_control_task, "mode_control_task", 2048, NULL, 4, NULL);
    xTaskCreate(&automatic_sequence_task, "automatic_sequence_task", 2048, NULL, 3, NULL);
}
#endif

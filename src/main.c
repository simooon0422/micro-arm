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

static const char *SERVO_TAG = "ServoControl"; // Tag for servo logging
static const char *POT_TAG = "Potentiometers"; // Tag for potentiometers logging
static const char *GRIPPER_TAG = "Gripper";    // Tag for gripper logging

bool movement_flag = 0;                    // 0 - no movement required, 1 - movement required
bool gripper_flag = 0;                     // 0 - gripper open, 1 - gripper closed
volatile bool isr_gripper_update_flag = 0; // 0 - gripper position update not required, 1 - gripper position update required
TickType_t last_gripper_time = 0;          // Last time of gripper interrupt

typedef enum // Modes of working
{
    HOME = 0,
    MANUAL = 1,
    AUTO = 2
} Modes_t;

volatile Modes_t current_mode = 0;      // 0 - Home mode, 1 - Manual mode, 2 - Automatic mode
volatile bool isr_mode_update_flag = 1; // 0 - no mode update required, 1 - mode update required
TickType_t last_mode_time = 0;          // Last time of mode interrupt

uint8_t path_steps = 0; // Number of path steps for automatic mode, default 0

uint8_t home_position[LINKS_NUMBER] = {90, 135, 30, 30};    // Angle values of servo home positions
uint8_t current_position[LINKS_NUMBER] = {90, 135, 30, 30}; // Angle values of servo current positions
uint8_t target_position[LINKS_NUMBER] = {90, 135, 30, 30};  // Angle values of servo target positions
uint8_t pot_readings[LINKS_NUMBER];                         // Readings from potentiometers mapped to angle values

uint8_t work_path[MAX_PATH_STEPS][LINKS_NUMBER + 1] = {0}; // Array for storing automatic path from EEPROM

#ifdef WRITE_EEPROM_ON_START
#define WRITE_PATH_STEPS 3                                 // Number of steps for sequence in automatic mode to write in EEPROM
uint8_t write_path[WRITE_PATH_STEPS][LINKS_NUMBER + 1] = { // Array with automatic path to write to EEPROM
    {90, 135, 30, 30},
    {0, 135, 30, 30},
    {90, 135, 30, 30}};
#endif

SemaphoreHandle_t xMutexTargetPosition = NULL;     // Mutex for target position
QueueHandle_t xQueueTargetPosition = NULL;         // Queue for target position
QueueHandle_t xQueueCurrentPosition = NULL;        // Queue for current position
QueueHandle_t xQueuePotentiometersPosition = NULL; // Queue for potentiometers position

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
    if (x < in_min)
    {
        x = in_min; // set given value to minimum if it was below minimum
    }
    else if (x > in_max)
    {
        x = in_max; // set given value to maximum if it was above maximum
    }
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; // return mapped value
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
    // Reset diodes pins
    gpio_reset_pin(GREEN_LED_PIN);
    gpio_reset_pin(YELLOW_LED_PIN);
    gpio_reset_pin(RED_LED_PIN);

    // Set diodes pins directions to output
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
        memcpy(target_position, new_target, new_target_size);                      // Copy new positions to target_position
        xQueueSend(xQueueTargetPosition, (void *)target_position, (TickType_t)10); // Send target position to LCD task
        xSemaphoreGive(xMutexTargetPosition);
    }
}

void move_home()
{
    for (int i = 0; i < LINKS_NUMBER; i++) // Set all servos to home positions
    {
        pca9685_set_servo_angle(i, home_position[i]);
    }

    ESP_LOGI(SERVO_TAG, "Homing completed, current position:"); // Log all home positions
    for (int i = 0; i < LINKS_NUMBER; i++)
    {
        ESP_LOGI(SERVO_TAG, "Servo %d: %d", i, current_position[i]);
    }

    gripper_open(); // Open robot gripper

    xQueueSend(xQueueCurrentPosition, (void *)current_position, (TickType_t)10); // Send current home position to LCD task
}

bool check_position(uint8_t current[], uint8_t target[], uint8_t n)
{
    for (int i = 0; i < n; i++)
    {
        if (current[i] != target[i])
        {
            return false; // false if arrays are different
        }
    }
    return true; // true if arrays are the same
}

int get_step(uint8_t current, uint8_t target)
{
    if (current != target)
    {
        return ((target - current) / abs(target - current)); // returns 1 or -1 depending on the required movement direction
    }
    else
        return 0; // returns 0 if positions are the same
}

void move_step(uint8_t link)
{
    int step = get_step(current_position[link], target_position[link]);          // Acquire step for movement
    pca9685_set_servo_angle(link, current_position[link] + step);                // Move servo by acquired step
    current_position[link] = current_position[link] + step;                      // Update current position
    xQueueSend(xQueueCurrentPosition, (void *)current_position, (TickType_t)10); // Send current position to LCD task
}

void write_auto_path(uint8_t arr[][LINKS_NUMBER + 1], uint8_t steps_n)
{
    eeprom_write_byte(PATH_STEPS_ADDRESS, steps_n); // Write to EEPROM number of path steps
    for (int i = 0; i < steps_n; i++)               // Write to EEPROM path for automatic mode
    {
        for (int j = 0; j < LINKS_NUMBER + 1; j++)
        {
            eeprom_write_byte(j + (i * (LINKS_NUMBER + 1)), arr[i][j]);
        }
    }
}

void read_auto_path(uint8_t arr[][LINKS_NUMBER + 1], uint8_t *steps_n)
{
    eeprom_read_byte(PATH_STEPS_ADDRESS, steps_n); // Read from EEPROM number of path steps
    for (int i = 0; i < *steps_n; i++)             // Read from EEPROM path for automatic mode
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
        isr_gripper_update_flag = 1;                    // Set flag meaning that gripper status change is required
        last_gripper_time = xTaskGetTickCountFromISR(); // Update time of last interrupt
    }
}

static void isr_mode_handler()
{
    if (xTaskGetTickCountFromISR() - last_mode_time >= DEBOUNCE_TIME)
    {
        isr_mode_update_flag = 1;         // Set flag meaning that mode has changed
        current_mode++;                   // Set new mode
        if (current_mode >= MODES_NUMBER) // Set new mode to 0 if it exceeds number of modes
        {
            current_mode = 0;
        }
        last_mode_time = xTaskGetTickCountFromISR(); // Update time of last interrupt
    }
}

uint8_t get_num_digits(uint8_t n)
{
    if (n < 10)
        return 1;
    return 1 + get_num_digits(n / 10);
}

void get_text(wchar_t text_arr[], uint8_t x)
{
    uint8_t x_digits_num = get_num_digits(x);
    char x_char[x_digits_num + 1];

    sprintf(x_char, "%d", x);

    for (int i = 0; i < x_digits_num; i++)
    {
        text_arr[i] = x_char[i];
    }
}

void show_headers(hagl_backend_t *display)
{
    hagl_fill_rectangle(display, 0, 0, 60, 40, GREEN);       // Placeholder for gripper icon
    hagl_fill_rectangle(display, 60, 0, LCD_WIDTH, 40, RED); // Placeholder for mode icon

    // Display links header
    for (int i = 0; i < LINKS_NUMBER; i++)
    {
        wchar_t header_text[2] = L"L ";
        wchar_t link_num[2];
        get_text(link_num, i);
        header_text[1] = link_num[0];
        hagl_put_text(display, header_text, 50 + (30 * i), 42, YELLOW, font6x9);
    }

    // Display positions headers
    hagl_put_text(display, L"TAR:", 10, 60, YELLOW, font6x9);  // Display target position header
    hagl_put_text(display, L"CUR:", 10, 85, YELLOW, font6x9);  // Display current position header
    hagl_put_text(display, L"POT:", 10, 110, YELLOW, font6x9); // Display potentiometers position header
}

void show_target(hagl_backend_t *display, uint8_t arr_target[])
{
    if (xQueueReceive(xQueueTargetPosition, arr_target, (TickType_t)5) == pdPASS)
    {
        for (int i = 0; i < LINKS_NUMBER; i++)
        {
            wchar_t text[4] = L"   ";
            get_text(text, arr_target[i]);
            hagl_put_text(display, text, 50 + (30 * i), 60, YELLOW, font6x9);
        }
    }
    else
    {
        for (int i = 0; i < LINKS_NUMBER; i++)
        {
            wchar_t text[4] = L"   ";
            get_text(text, arr_target[i]);
            hagl_put_text(display, text, 50 + (30 * i), 60, YELLOW, font6x9);
        }
    }
}

void show_current(hagl_backend_t *display, uint8_t arr_current[])
{
    if (xQueueReceive(xQueueCurrentPosition, arr_current, (TickType_t)5) == pdPASS)
    {
        for (int i = 0; i < LINKS_NUMBER; i++)
        {
            wchar_t text[4] = L"   ";
            get_text(text, arr_current[i]);
            hagl_put_text(display, text, 50 + (30 * i), 85, YELLOW, font6x9);
        }
    }
    else
    {
        for (int i = 0; i < LINKS_NUMBER; i++)
        {
            wchar_t text[4] = L"   ";
            get_text(text, arr_current[i]);
            hagl_put_text(display, text, 50 + (30 * i), 85, YELLOW, font6x9);
        }
    }
}

void show_potentiometers(hagl_backend_t *display, uint8_t arr_potentiometers[])
{
    if (xQueueReceive(xQueuePotentiometersPosition, arr_potentiometers, (TickType_t)5) == pdPASS)
    {
        for (int i = 0; i < LINKS_NUMBER; i++)
        {
            wchar_t text[4] = L"   ";
            get_text(text, arr_potentiometers[i]);
            hagl_put_text(display, text, 50 + (30 * i), 110, YELLOW, font6x9);
        }
    }
    else
    {
        for (int i = 0; i < LINKS_NUMBER; i++)
        {
            wchar_t text[4] = L"   ";
            get_text(text, arr_potentiometers[i]);
            hagl_put_text(display, text, 50 + (30 * i), 110, YELLOW, font6x9);
        }
    }
}

void servo_control_task(void *pvParameter)
{
    move_home();                     // Move all servos to home position on start
    vTaskDelay(pdMS_TO_TICKS(2000)); // Wait to finish homing

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
            xQueueSend(xQueuePotentiometersPosition, (void *)pot_readings, (TickType_t)10); // Send potentiometers position to LCD task
        }

        // Update target position only in mode 1
        if (current_mode == 1)
        {
            set_target_position(pot_readings, sizeof(pot_readings));
        }
        ESP_LOGI(POT_TAG, "Readings: %d, %d, %d, %d", pot_readings[0], pot_readings[1], pot_readings[2], pot_readings[3]); // Log potentiometers readings
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
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void gripper_control_task(void *pvParameter)
{
    while (1)
    {
        if (isr_gripper_update_flag == 1)
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

            isr_gripper_update_flag = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void mode_control_task(void *pvParameter)
{
    while (1)
    {
        if (isr_mode_update_flag == 1) // Update mode only if user pressed mode button
        {
            switch (current_mode)
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

            isr_mode_update_flag = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void automatic_sequence_task(void *pvParameter)
{
    while (1)
    {
        if ((current_mode == 2) && (path_steps <= MAX_PATH_STEPS))
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
                    xQueueSend(xQueueTargetPosition, (void *)target_position, (TickType_t)10);
                    xSemaphoreGive(xMutexTargetPosition);
                }

                // If mode has changed in the meantime -> break the loop
                if (current_mode != 2)
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

void lcd_display_task(void *pvParameter)
{
    hagl_backend_t *display = (hagl_backend_t *)pvParameter;
    uint8_t received_target[LINKS_NUMBER] = {0};
    uint8_t received_current[LINKS_NUMBER] = {0};
    uint8_t received_pot[LINKS_NUMBER] = {0};

    while (1)
    {
        hagl_fill_rectangle(display, 0, 0, LCD_WIDTH, LCD_HEIGHT, BLACK); // Reset screen
        show_headers(display);                                            // Display headers on LCD
        show_target(display, received_target);                            // Display target position values on LCD
        show_current(display, received_current);                          // Display current position values on LCD
        show_potentiometers(display, received_pot);                       // Display potentiometers position values on LCD
        lcd_send_buffer();
        vTaskDelay(10);
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
    st7735_init();  // Initialize LCD display

    hagl_backend_t *display = hagl_init(); // Initialize hagl backend

#ifdef WRITE_EEPROM_ON_START
    write_auto_path(write_path, WRITE_PATH_STEPS); // Write automatic path to EEPROM
#endif
    read_auto_path(work_path, &path_steps); // Read automatic path from EEPROM

    xMutexTargetPosition = xSemaphoreCreateMutex();                                 // Create mutex for target position
    xQueueTargetPosition = xQueueCreate(2, sizeof(uint8_t) * LINKS_NUMBER);         // Create queue for target position
    xQueueCurrentPosition = xQueueCreate(2, sizeof(uint8_t) * LINKS_NUMBER);        // Create queue for current position
    xQueuePotentiometersPosition = xQueueCreate(2, sizeof(uint8_t) * LINKS_NUMBER); // Create queue for potentiometers position

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);                     // Allow per-pin GPIO interrupt handlers
    gpio_isr_handler_add(GRIPPER_BUTTON_PIN, isr_gripper_handler, NULL); // Gripper interrupt
    gpio_isr_handler_add(MODE_BUTTON_PIN, isr_mode_handler, NULL);       // Mode change interrupt

    xTaskCreate(&servo_control_task, "servo_control_task", 2048, NULL, 2, NULL);             // Create task for servo control
    xTaskCreate(&read_potentiometers_task, "read_potentiometers_task", 4096, NULL, 3, NULL); // Create task for reading potentiometers values
    xTaskCreate(&position_control_task, "position_control_task", 2048, NULL, 4, NULL);       // Create task for position control
    xTaskCreate(&gripper_control_task, "gripper_control_task", 2048, NULL, 1, NULL);         // Create task for gripper control
    xTaskCreate(&mode_control_task, "mode_control_task", 2048, NULL, 4, NULL);               // Create task for mode control
    xTaskCreate(&automatic_sequence_task, "automatic_sequence_task", 2048, NULL, 3, NULL);   // Create task for automatic mode
    xTaskCreate(&lcd_display_task, "lcd_display_task", 2048, display, 3, NULL);              // Create task for display control
}
#endif

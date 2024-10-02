#include "driver/gpio.h"
#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

/**
 * @brief Initialize ADC for multiplexer.
 */
void adc_init(void);

/**
 * @brief Initialize CD4051 multiplexer.
 * @param com_direction Mode of multiplexer (writing - GPIO_MODE_OUTPUT, reading - GPIO_MODE_INPUT).
 */
void cd4051_init(void);

/**
 * @brief Set logic for one of the SELECT pins.
 * @param channel Channel (0-7).
 * @param current_select SELECT pin to set logic for (0 - SELECT A, 1 - SELECT B, 2 - SELECT C).
 * @return Logic level for pin (0 or 1).
 */
bool set_select_logic(uint8_t channel, uint8_t current_select);

/**
 * @brief Select current multiplexer channel.
 * @param channel Channel (0-7).
 * @return Channel that has been set.
 */
uint8_t select_channel(uint8_t channel);

/**
 * @brief Turn on multiplexer.
 */
void cd4051_activate(void);

/**
 * @brief Turn off multiplexer.
 */
void cd4051_deactivate(void);

/**
 * @brief Read analog value from selected channel.
 * @param channel Channel to read from.
 * @return Measured analog value.
 */
int cd4051_read_channel(uint8_t channel);

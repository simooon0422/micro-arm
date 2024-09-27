#include "driver/gpio.h"
#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

/**
 * @brief Initialize ADC for multiplexer.
 */
void adc_init();

/**
 * @brief Initialize CD4051 multiplexer.
 * @param com_direction Mode of multiplexer (writing - GPIO_MODE_OUTPUT, reading - GPIO_MODE_INPUT)
 */
void cd4051_init();

/**
 * @brief Select current multiplexer channel.
 * @param channel Channel (0-7).
 */
void select_channel(uint8_t channel);

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
uint16_t cd4051_read_channel(uint8_t channel);

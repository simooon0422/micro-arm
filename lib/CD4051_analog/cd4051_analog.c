#include "cd4051_analog.h"

#define SELECT_A GPIO_NUM_14 // Select A pin number
#define SELECT_B GPIO_NUM_12 // Select B pin number
#define SELECT_C GPIO_NUM_13 // Select C pin number
#define INHIBIT GPIO_NUM_26  // Inhibit pin number

#define COM ADC_CHANNEL_7         // GPIO35 -> ADC1_CHANNEL_7
#define ADC_WIDTH ADC_BITWIDTH_12 // 12-bit resolution (0-4095)
#define ADC_ATTEN ADC_ATTEN_DB_12 // Attenuation - 0-3.3V

static const char *ADC_TAG = "ADC1";
adc_oneshot_unit_handle_t adc1_handle;
gpio_num_t select_pins[] = {SELECT_A, SELECT_B, SELECT_C};

void adc_init()
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_WIDTH,
        .atten = ADC_ATTEN};
    adc_oneshot_config_channel(adc1_handle, COM, &config);
}
void cd4051_init()
{
    for (int i = 0; i < 3; i++)
    {
        gpio_reset_pin(select_pins[i]);
        gpio_set_direction(select_pins[i], GPIO_MODE_OUTPUT);
    }

    gpio_reset_pin(INHIBIT);
    gpio_set_direction(INHIBIT, GPIO_MODE_OUTPUT);

    adc_init();
    cd4051_activate();
}

void cd4051_activate()
{
    gpio_set_level(INHIBIT, 0);
}

void cd4051_deactivate()
{
    gpio_set_level(INHIBIT, 1);
}

void select_channel(uint8_t channel)
{
    for (int i = 0; i < 3; i++)
    {
        if ((channel & (1 << i)) == 0)
        {
            gpio_set_level(select_pins[i], 0);
        }
        else
            gpio_set_level(select_pins[i], 1);
    }
}

uint16_t cd4051_read_channel(uint8_t channel)
{
    int adc_result;

    select_channel(channel);

    if (adc_oneshot_read(adc1_handle, COM, &adc_result) == ESP_ERR_TIMEOUT)
    {
        ESP_LOGE(ADC_TAG, "ADC TIMEOUT");
    }
    else if (adc_oneshot_read(adc1_handle, COM, &adc_result) == ESP_ERR_INVALID_ARG)
    {
        ESP_LOGE(ADC_TAG, "Invalid arguments");
    }

    return adc_result;
}
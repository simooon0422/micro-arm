#include "cd4051_analog.h"

#define SELECT_A GPIO_NUM_25 // Select A pin number
#define SELECT_B GPIO_NUM_33 // Select B pin number
#define SELECT_C GPIO_NUM_32 // Select C pin number
#define INHIBIT GPIO_NUM_26  // Inhibit pin number

#define COM ADC_CHANNEL_7         // GPIO35 -> ADC1_CHANNEL_7
#define ADC_WIDTH ADC_BITWIDTH_12 // 12-bit resolution (0-4095)
#define ADC_ATTEN ADC_ATTEN_DB_12 // Attenuation - 0-3.3V

static const char *ADC_TAG = "ADC1";
adc_oneshot_unit_handle_t adc1_handle;
gpio_num_t select_pins[] = {SELECT_A, SELECT_B, SELECT_C};

void adc_init(void)
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
void cd4051_init(void)
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

void cd4051_activate(void)
{
    gpio_set_level(INHIBIT, 0);
}

void cd4051_deactivate(void)
{
    gpio_set_level(INHIBIT, 1);
}

bool set_select_logic(uint8_t channel, uint8_t current_select)
{
    if ((channel & (1 << current_select)) == 0)
    {
        return 0;
    }
    else
        return 1;
}

uint8_t select_channel(uint8_t channel)
{
    uint8_t channel_set = 0;
    for (int i = 0; i < 3; i++)
    {   
        bool select_logic = set_select_logic(channel, i);
        gpio_set_level(select_pins[i], select_logic);
        channel_set = channel_set | (select_logic << i);
    } 
    return channel_set;
}

int cd4051_read_channel(uint8_t channel)
{
    int adc_result;
    if (channel < 8)
    {
        select_channel(channel);

        if (adc_oneshot_read(adc1_handle, COM, &adc_result) == ESP_ERR_TIMEOUT)
        {
            ESP_LOGE(ADC_TAG, "ADC TIMEOUT");
            return -1;
        }
        else if (adc_oneshot_read(adc1_handle, COM, &adc_result) == ESP_ERR_INVALID_ARG)
        {
            ESP_LOGE(ADC_TAG, "Invalid arguments");
            return -2;
        }

        return adc_result;
    }
    else
        return -3;
}
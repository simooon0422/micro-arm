#include <unity.h>
#include "eeprom.h"

uint8_t test_address = 127;
uint8_t test_write_data = 100;
uint8_t test_read_data;

void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}

void test_eeprom_init_ok(void)
{
    esp_err_t ret = eeprom_init();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

void test_eeprom_write_ok(void)
{
    esp_err_t ret = eeprom_write_byte(test_address, 0);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

void test_eeprom_read_ok(void)
{
    esp_err_t ret = eeprom_read_byte(test_address, &test_read_data);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

void test_eeprom_read_data_correct(void)
{
    eeprom_write_byte(test_address, test_write_data);
    eeprom_read_byte(test_address, &test_read_data);
    TEST_ASSERT_EQUAL(test_write_data, test_read_data);
}

/**
 * @brief Run all tests for eeprom library
 */
void run_test_group_eeprom(void)
{
    RUN_TEST(test_eeprom_init_ok);
    RUN_TEST(test_eeprom_write_ok);
    RUN_TEST(test_eeprom_read_ok);
    RUN_TEST(test_eeprom_read_data_correct);
}

int runUnityTests(void)
{
    vTaskDelay(pdMS_TO_TICKS(2500));
    UNITY_BEGIN();
    run_test_group_eeprom();
    return UNITY_END();
}

void app_main()
{
    runUnityTests();
}
#include <unity.h>
#include "lcd_st7735s.h"

void test_init_OK(void)
{
    esp_err_t ret = init_spi();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

void test_send_command_NOK(void)
{
    esp_err_t ret = lcd_send_command(0x3A);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);
}

void test_send_data_NOK(void)
{
    esp_err_t ret = lcd_send_data(0x05);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);
}

void test_send_command_OK(void)
{
    esp_err_t ret = lcd_send_command(0x3A);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

void test_send_data_OK(void)
{
    esp_err_t ret = lcd_send_command(0x05);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

int runUnityTests(void)
{
    vTaskDelay(pdMS_TO_TICKS(2500));
    UNITY_BEGIN();
    RUN_TEST(test_send_command_NOK);
    RUN_TEST(test_send_data_NOK);
    RUN_TEST(test_init_OK);
    RUN_TEST(test_send_command_OK);
    RUN_TEST(test_send_data_OK);
    return UNITY_END();
}

void app_main()
{
    runUnityTests();
}
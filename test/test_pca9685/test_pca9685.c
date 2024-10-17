#include <unity.h>
#include "pca9685.h"

void test_init(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, i2c_master_init());
}

void test_write_byte(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, i2c_write_byte(PCA9685_ADDR, 0x00, 0x00));
}

void test_servo_angle_to_pwm_0(void)
{
    TEST_ASSERT_EQUAL(102, servo_angle_to_pwm(0));
}

void test_servo_angle_to_pwm_90(void)
{
    TEST_ASSERT_EQUAL(317, servo_angle_to_pwm(90));
}

void test_servo_angle_to_pwm_180(void)
{
    TEST_ASSERT_EQUAL(532, servo_angle_to_pwm(180));
}

void test_servo_angle_to_pwm_above_180(void)
{
    TEST_ASSERT_EQUAL(532, servo_angle_to_pwm(200));
}

/**
 * @brief Run all tests for servo_angle_to_pwm() function
 */
void run_test_servo_angle_group(void)
{
    RUN_TEST(test_servo_angle_to_pwm_0);
    RUN_TEST(test_servo_angle_to_pwm_90);
    RUN_TEST(test_servo_angle_to_pwm_180);
    RUN_TEST(test_servo_angle_to_pwm_above_180);
}

int runUnityTests(void)
{
    vTaskDelay(pdMS_TO_TICKS(2500));
    UNITY_BEGIN();
    RUN_TEST(test_init);
    RUN_TEST(test_write_byte);
    run_test_servo_angle_group();
    return UNITY_END();
}

void app_main()
{
    runUnityTests();
}
#include <unity.h>
#include "main.h"

// TEST MAP GROUP BEGIN
void test_map_min_in_returns_min_out(void)
{
    TEST_ASSERT_EQUAL(0, map(0, 0, 4095, 0, 180));
}

void test_map_max_in_returns_max_out(void)
{
    TEST_ASSERT_EQUAL(180, map(4095, 0, 4095, 0, 180));
}

void test_map_mid_in_returns_mid_out(void)
{
    TEST_ASSERT_EQUAL(90, map(2048, 0, 4095, 0, 180));
}

void test_map_above_in_returns_max_out(void)
{
    TEST_ASSERT_EQUAL(180, map(5000, 0, 4095, 0, 180));
}

void test_map_below_in_returns_min_out(void)
{
    TEST_ASSERT_EQUAL(0, map(-10, 0, 4095, 0, 180));
}

void test_map_same_in_returns_same_out(void)
{
    TEST_ASSERT_EQUAL(1000, map(1000, 0, 4095, 0, 4095));
}

/**
 * @brief Run all tests for map() function
 */
void run_test_group_map(void)
{
    RUN_TEST(test_map_min_in_returns_min_out);
    RUN_TEST(test_map_max_in_returns_max_out);
    RUN_TEST(test_map_mid_in_returns_mid_out);
    RUN_TEST(test_map_above_in_returns_max_out);
    RUN_TEST(test_map_below_in_returns_min_out);
    RUN_TEST(test_map_same_in_returns_same_out);
}
// TEST MAP GROUP END

// TEST CHECK POSITION GROUP BEGIN
void test_check_position_same_position_returns_true(void)
{
    uint8_t links_n = 4;
    uint8_t curr_pos[] = {90, 135, 30, 90};
    uint8_t targ_pos[] = {90, 135, 30, 90};
    TEST_ASSERT_TRUE(check_position(curr_pos, targ_pos, links_n));
}

void test_check_position_different_position_returns_false(void)
{
    uint8_t links_n = 4;
    uint8_t curr_pos[] = {90, 135, 30, 90};
    uint8_t targ_pos[] = {0, 100, 60, 180};
    TEST_ASSERT_FALSE(check_position(curr_pos, targ_pos, links_n));
}

/**
 * @brief Run all tests for check_position() function
 */
void run_test_group_check_position(void)
{
    RUN_TEST(test_check_position_same_position_returns_true);
    RUN_TEST(test_check_position_different_position_returns_false);
}
// TEST CHECK POSITION GROUP END

// TEST GET STEP GROUP BEGIN
void test_get_step_return_1(void)
{
    TEST_ASSERT_EQUAL(1, get_step(90, 180));
}

void test_get_step_return_0(void)
{
    TEST_ASSERT_EQUAL(0, get_step(90, 90));
}

void test_get_step_return_minus_1(void)
{
    TEST_ASSERT_EQUAL(-1, get_step(90, 0));
}

/**
 * @brief Run all tests for get_step() function
 */
void run_test_group_get_step(void)
{
    RUN_TEST(test_get_step_return_1);
    RUN_TEST(test_get_step_return_0);
    RUN_TEST(test_get_step_return_minus_1);
}
// TEST GET STEP GROUP END

// TEST EEPROM SEQUENCE GROUP BEGIN
void test_eeprom_init_ok(void)
{
    esp_err_t ret = eeprom_init();
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

void test_eeprom_sequence_read_equals_write(void)
{
    vTaskDelay(pdMS_TO_TICKS(500));
    uint8_t test_steps = 3;
    uint8_t test_rcv_steps;
    uint8_t test_write_sequence[3][LINKS_NUMBER + 1] = {
        {90, 135, 30, 30, 0},
        {60, 100, 60, 0, 1},
        {60, 180, 0, 45, 0}};

    uint8_t test_read_sequence[3][LINKS_NUMBER + 1] = {0};

    write_auto_path(test_write_sequence, test_steps);
    read_auto_path(test_read_sequence, &test_rcv_steps);

    TEST_ASSERT_EQUAL(test_steps, test_rcv_steps);
    for (int i = 0; i < 3; i++)
    {
        TEST_ASSERT_EQUAL_UINT8_ARRAY(test_write_sequence[i], test_read_sequence[i], LINKS_NUMBER + 1);
    }
}

/**
 * @brief Run all tests for writing/reading EEPROM sequence
 */
void run_test_group_eeprom_sequence(void)
{
    RUN_TEST(test_eeprom_init_ok);
    RUN_TEST(test_eeprom_sequence_read_equals_write);
}
// TEST EEPROM SEQUENCE GROUP END

int runUnityTests(void)
{
    vTaskDelay(pdMS_TO_TICKS(2500));
    UNITY_BEGIN();
    run_test_group_map();
    run_test_group_check_position();
    run_test_group_get_step();
    run_test_group_eeprom_sequence();
    return UNITY_END();
}

void app_main()
{
    runUnityTests();
}
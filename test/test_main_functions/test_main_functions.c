#include <unity.h>
#include "freertos/FreeRTOS.h"
#include "main.h"

void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}

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
    uint8_t curr_pos[] ={90, 135, 30, 90};
    uint8_t targ_pos[] = {90, 135, 30, 90};
    TEST_ASSERT_TRUE(check_position(curr_pos, targ_pos, links_n));
}

void test_check_position_different_position_returns_false(void)
{
    uint8_t links_n = 4;
    uint8_t curr_pos[] ={90, 135, 30, 90};
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

int runUnityTests(void)
{
    vTaskDelay(pdMS_TO_TICKS(2500));
    UNITY_BEGIN();
    run_test_group_map();
    run_test_group_check_position();
    run_test_group_get_step();
    return UNITY_END();
}

void app_main()
{
    runUnityTests();
}
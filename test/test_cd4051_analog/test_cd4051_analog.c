#include <unity.h>
#include "freertos/FreeRTOS.h"
#include "cd4051_analog.h"

void setUp(void)
{
    // set stuff up here
}

void tearDown(void)
{
    // clean stuff up here
}

// TEST GROUP CD4051 READ CHANNEL BEGIN
void test_cd4051_read_channel_in_range(void)
{
    cd4051_init();
    for (int i = 0; i < 8; i++)
    {
        TEST_ASSERT_INT_WITHIN(2048, 2048, cd4051_read_channel(i));
    }  
}

void test_cd4051_read_channel_wrong_channel(void)
{
    cd4051_init();
    TEST_ASSERT_EQUAL(-3, cd4051_read_channel(10));
}

/**
 * @brief Run all tests for cd4051_read_channel() function
 */
void run_test_group_cd4051_read_channel(void)
{
    RUN_TEST(test_cd4051_read_channel_in_range);
    RUN_TEST(test_cd4051_read_channel_wrong_channel);
}
// TEST GROUP CD4051 READ CHANNEL END


// TEST GROUP SET SELECT LOGIC BEGIN
void test_set_select_logic_returns_0(void)
{
    TEST_ASSERT_EQUAL(0, set_select_logic(1, 2));  
}

void test_set_select_logic_returns_1(void)
{
    TEST_ASSERT_EQUAL(1, set_select_logic(5, 0));  
}

/**
 * @brief Run all tests for set_select_logic() function
 */
void run_test_group_set_select_logic(void)
{
    RUN_TEST(test_set_select_logic_returns_0);
    RUN_TEST(test_set_select_logic_returns_1);
}
// TEST GROUP SET SELECT LOGIC END


// TEST GROUP SELECT CHANNEL LOGIC BEGIN
void test_select_channel_correct_channel(void)
{
    for (int i = 0; i < 8; i++)
    {
        TEST_ASSERT_EQUAL(i, select_channel(i));
    }
}
/**
 * @brief Run all tests for select_channel() function
 */
void run_test_group_select_channel(void)
{
    RUN_TEST(test_select_channel_correct_channel);
}
// TEST GROUP SELECT CHANNEL LOGIC END

int runUnityTests(void)
{
    vTaskDelay(pdMS_TO_TICKS(2500));
    UNITY_BEGIN();
    run_test_group_cd4051_read_channel();
    run_test_group_set_select_logic();
    run_test_group_select_channel();
    return UNITY_END();
}

void app_main()
{
    runUnityTests();
}
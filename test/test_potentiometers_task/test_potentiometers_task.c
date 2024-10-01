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

int runUnityTests(void)
{
    vTaskDelay(pdMS_TO_TICKS(2500));
    UNITY_BEGIN();
    return UNITY_END();
}

void app_main()
{
    runUnityTests();
}
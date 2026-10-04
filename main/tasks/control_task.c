#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <stdio.h>
void Control_Task(void *pvParameters)
{
    while (1)
    {
        printf("Control_Task is running\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "tasks/control_task.h"
void app_main(void)
{
    xTaskCreatePinnedToCore(Control_Task, "Control_Task", 4096, NULL, 1, NULL, 1);
}
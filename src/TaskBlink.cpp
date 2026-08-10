#include <Arduino.h>
#include "Tasks.h"
#include <Shared.h>

void TaskBlink(void *pvParameters) {
    (void) pvParameters;

    
    bool ledState = false;



    for (;;) {
        // Eğer LED'i komutla HIGH yaptıysan burası yine de toggle eder.
        // Dilersen burayı "sadece LED kapalıysa blink et" diye değiştirebilirsin.
        digitalWrite(LED_RUN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(LED_RUN, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(LED_RUN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        digitalWrite(LED_RUN, 0);
        vTaskDelay(pdMS_TO_TICKS(700));
    }
}

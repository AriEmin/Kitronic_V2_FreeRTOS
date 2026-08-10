#include <Arduino.h>
#include "Tasks.h"
#include <Shared.h>

void TaskWorker(void *pvParameters) {
    (void) pvParameters;

    for (;;) {
        // Burada ileride:
        // - Solenoid PWM test
        // - TWAI frame gönder
        // - INA219 okuma
        // - DRV8243 config
        // hepsi yapılabilir

        {
            const char *msg = "[WORKER] is doing background job...";
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        }

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

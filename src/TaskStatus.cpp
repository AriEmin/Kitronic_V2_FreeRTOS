#include <Arduino.h>
#include "Tasks.h"
#include <Shared.h>

void TaskStatus(void *pvParameters) {
    (void) pvParameters;
    uint32_t counter = 0;

    for (;;) {
        {
            char msg[64];
            snprintf(msg, sizeof(msg), "[STATUS] alive=%lu", (unsigned long)counter++);
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

#include <Arduino.h>
#include "Tasks.h"
#include <Shared.h>

void TaskAnalog(void *pvParameters) {
    (void) pvParameters;

    // ESP32 ADC kalibrasyonu ileride eklenir
    for (;;) {
        int adcVal = analogRead(PRESSURE_PIN);
        float voltage = (adcVal / 4095.0f) * 3.3f;

        /*{
            char msg[64];
            snprintf(msg, sizeof(msg), "[ADC] pin=%d val=%d volt=%.2f", PRESSURE_PIN, adcVal, voltage);
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        }*/

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

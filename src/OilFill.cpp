#include <Arduino.h>
#include "Tasks.h"

// ---------------------------------------------------------------------------
// OilFill — Isıtıcı SSR görev
//
// Duty cycle modu (g_heater_on_ms > 0 && g_heater_off_ms > 0):
//   Fişekler g_heater_on_ms süre açık kalır, ardından g_heater_off_ms süre
//   kapanır. Bu döngü fişeklerin mekatroniği haddinden fazla ısıtmasını önler.
//
// Setpoint modu (g_heater_setpoint > 0):
//   g_temp2_C (yağ sıcaklığı) hedefe ulaştığında ısıtıcı otomatik kapanır.
//
// Güvenlik: TaskADSMonitor 90°C'de g_ssrDesired=0 yapar (üst limit).
// ---------------------------------------------------------------------------

void OilFill(void *pvParameters) {
    (void) pvParameters;

    pinMode(BLDC_SELECT, OUTPUT);
    digitalWrite(BLDC_SELECT, LOW);
    pinMode(SSR_CONTROL, OUTPUT);
    digitalWrite(SSR_CONTROL, LOW);

    bool     phaseOn    = true;    // true = ON fazı, false = OFF fazı
    uint32_t phaseStart = millis();

    for (;;) {
        uint8_t requested = g_ssrDesired;

        if (requested == 0) {
            digitalWrite(SSR_CONTROL, LOW);
            phaseOn    = true;
            phaseStart = millis();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // --- Setpoint kontrolü ---
        float setpt = g_heater_setpoint;
        if (setpt > 0.0f && g_temp2_C >= setpt) {
            digitalWrite(SSR_CONTROL, LOW);
            g_ssrDesired = 0;
            phaseOn    = true;
            phaseStart = millis();
            {
                char buf[56];
                snprintf(buf, sizeof(buf), "[HEATER] Setpoint %.0fC reached, OFF", setpt);
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // --- Duty cycle modu ---
        uint16_t onMs  = 5000;
        uint16_t offMs = 10000;

        if (onMs == 0 || offMs == 0) {
            // Duty cycle devre dışı: kesintisiz çalış
            digitalWrite(SSR_CONTROL, HIGH);
        } else {
            uint32_t elapsed  = millis() - phaseStart;
            uint16_t phaseDur = phaseOn ? onMs : offMs;

            if (elapsed >= phaseDur) {
                phaseOn    = !phaseOn;
                phaseStart = millis();
                {
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, phaseOn ? (char*)"[HEATER] Duty-ON" : (char*)"[HEATER] Duty-OFF");
                }
            }
            digitalWrite(SSR_CONTROL, phaseOn ? HIGH : LOW);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

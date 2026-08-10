#include <Arduino.h>
#include "Tasks.h"

void TaskValveClean(void *pvParameters) {
    (void)pvParameters;
    
    // Pin setup
    pinMode(VALVE_CLEAN_1, OUTPUT);
    pinMode(VALVE_CLEAN_2, OUTPUT);
    digitalWrite(VALVE_CLEAN_1, LOW);
    digitalWrite(VALVE_CLEAN_2, LOW);
    
    // Her kanal için bağımsız state
    bool outputState[2] = {false, false};
    uint32_t lastToggle[2] = {0, 0};
    uint16_t lastPeriod[2] = {0, 0};
    const uint8_t pins[2] = {VALVE_CLEAN_1, VALVE_CLEAN_2};
    
    for (;;) {
        uint32_t now = millis();
        
        for (int i = 0; i < 2; i++) {
            if (g_valveClean.ch[i].active) {
                uint16_t halfPeriod = g_valveClean.ch[i].period_ms / 2;
                
                // Periyot değişti mi?
                if (lastPeriod[i] != g_valveClean.ch[i].period_ms) {
                    lastPeriod[i] = g_valveClean.ch[i].period_ms;
                    lastToggle[i] = now;
                    outputState[i] = true;
                }
                
                // %50 duty cycle toggle
                if (now - lastToggle[i] >= halfPeriod) {
                    lastToggle[i] = now;
                    outputState[i] = !outputState[i];
                }
                
                digitalWrite(pins[i], outputState[i] ? HIGH : LOW);
            } else {
                // Kapalı - çıkışı sıfırla
                digitalWrite(pins[i], LOW);
                outputState[i] = false;
                lastPeriod[i] = 0;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms resolution
    }
}

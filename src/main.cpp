#include <Arduino.h>

#include <esp_system.h>

#include <Tasks.h>

#include <Shared.h>

#include <PistonControl.h>

#include <AutoShiftV2.h>

#include <PistonMonitor.h>

#include <AutoCalibration.h>

#include <Protocol.h>

// Stack overflow hook - hangi task'ın overflow ettiğini loglar
extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    ets_printf("[STACK OVERFLOW] Task: %s\n", pcTaskName ? pcTaskName : "?");
    abort();
}

// Reset sebebini insan-okunabilir stringe çevir (brownout/watchdog/panic
// teşhisi için — beklenmedik seri kopmalarının kaynağını bulmak amaçlı)
static const char* resetReasonStr(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXT_PIN";
        case ESP_RST_SW:        return "SW_RESET";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "OTHER_WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN";
    }
}



void setup() {

    Serial.setRxBufferSize(2048);
    Serial.begin(115200);

    // Serialin hazır olmasını bekle (kısa)

    delay(500);

    // Reset sebebini logla — beklenmedik seri kopmalarının kaynağını
    // (brownout/watchdog/panic vs.) tespit etmek için
    {
        esp_reset_reason_t rr = esp_reset_reason();
        char msg[64];
        snprintf(msg, sizeof(msg), "[BOOT] reset_reason=%s", resetReasonStr(rr));
        Serial.println(msg);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
    }

    Shared_Init();

    PistonControl_Init();



    pinMode(LED_RUN, OUTPUT);

    digitalWrite(LED_RUN, LOW);



    // --- Taskleri oluştur ---

    // main.cpp (setup içinde)

    xTaskCreatePinnedToCore(TaskSerial, "TaskSerial",

                            8192 /* stack bytes (ESP32'dede words değil bytes alıyor Arduino çekirdeği) */,

                            nullptr, 4, nullptr, APP_CPU_NUM);



    xTaskCreatePinnedToCore(TaskBlink,  "TaskBlink",  2048, NULL, 1, NULL, 1);

    xTaskCreatePinnedToCore(TaskAnalog, "TaskAnalog", 3072, NULL, 2, NULL, 1);

    //xTaskCreatePinnedToCore(TaskStatus, "TaskStatus", 3072, NULL, 1, NULL, 1);

    //xTaskCreatePinnedToCore(TaskWorker, "TaskWorker", 3072, NULL, 1, NULL, 1);

    xTaskCreatePinnedToCore(OilFill, "OilFill", 3072, NULL, 1, NULL, 1);       // Stack: pinMode(init)+digitalWrite(loop)

    xTaskCreatePinnedToCore(TaskI2CMonitor, "TaskI2CMon", 6144, NULL, 2, NULL, 1);  // 8x INA219.begin() + INA226 + Adafruit I2CDevice chain

    xTaskCreatePinnedToCore(TaskADSMonitor, "TaskADS", 4096, NULL, 2, NULL, 1);

    xTaskCreatePinnedToCore(TaskValveControl, "TaskValve", 6144, NULL, 3, NULL, 1);  // TCA+DRV+PWM+NVS init chain

    xTaskCreatePinnedToCore(TaskBLDCPump, "TaskBLDCPump", 4096, NULL, 3, NULL, 1);

    xTaskCreatePinnedToCore(TaskOilCheck, "TaskOil", 4096, NULL, 3, NULL, 0);

    xTaskCreatePinnedToCore(TaskDiag, "TaskDiag", 6144, NULL, 3, NULL, 0);

    xTaskCreatePinnedToCore(TaskAutoShiftV2, "TaskAutoV2", 6144, NULL, 3, NULL, 1);

    xTaskCreatePinnedToCore(TaskTMAG5173, "TaskTMAG", 4096, NULL, 2, NULL, 1);

    xTaskCreatePinnedToCore(TaskValveClean, "TaskClean", 3072, NULL, 1, NULL, 1);  // pinMode init margin

    xTaskCreatePinnedToCore(TaskValveDiag, "TaskVDiag", 4096, NULL, 3, NULL, 1);  // JsonDocument + serializeJson at test runtime

    xTaskCreatePinnedToCore(TaskHallTest, "TaskHall", 4096, NULL, 4, NULL, 1);  // Hall stabilite testi (yüksek öncelik)

    xTaskCreatePinnedToCore(TaskCurrentCalib, "TaskCCal", 4096, NULL, 3, NULL, 1);  // holdcontrol_V2 akim kalibrasyonu

    xTaskCreatePinnedToCore(TaskAutoTest, "TaskAutoTest", 8192, NULL, 3, NULL, 1); // 9-Fazlı Otomatik Test

    xTaskCreatePinnedToCore(TaskOLED, "TaskOLED", 4096, NULL, 1, NULL, 1); // I2C OLED durum ekranı

    xTaskCreatePinnedToCore(TaskButtonPad, "TaskButtonPad", 4096, NULL, 1, NULL, 1); // TCA9555 @ 0x22 buton paneli














    Serial.println("[OK] FreeRTOS taskleri baslatildi.");

    Serial.println("Komutlar: LED ON | LED OFF | STATUS | HELP");

}



void loop() {

    // Her şey tasklerde

    vTaskDelay(1000 / portTICK_PERIOD_MS);

}


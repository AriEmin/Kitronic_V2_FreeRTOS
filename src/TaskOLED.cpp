#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <U8g2lib.h>
#include <string.h>

#include "Tasks.h"
#include "Shared.h"

// HS96L03W2C03: 128x64 I2C OLED, adres 0x3C
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C g_u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Pressure conversion: filtered pressure value produced by TaskBLDCPump
static inline float pressureBar() {
    return g_pumpPub.bar;
}

// I2C mutex wrapper
static bool i2cLock(TickType_t timeout) {
    if (!g_i2cMutex) return false;
    return xSemaphoreTake(g_i2cMutex, timeout) == pdTRUE;
}

static void i2cUnlock() {
    if (g_i2cMutex) xSemaphoreGive(g_i2cMutex);
}

static const char *translateButtonText(const char *text, uint8_t language) {
    if (language != 1) return text;
    if (!strcmp(text, "ACIK")) return "OPEN";
    if (!strcmp(text, "KAPALI")) return "CLOSED";
    if (!strcmp(text, "BASLADI")) return "STARTED";
    if (!strcmp(text, "DURDU")) return "STOPPED";
    if (!strcmp(text, "OTOMATIK")) return "AUTO";
    if (!strcmp(text, "MESGUL")) return "BUSY";
    if (!strcmp(text, "ISLEM AKTIF")) return "BUSY";
    if (!strcmp(text, "10 BAR ALTI")) return "BELOW 10 BAR";
    if (!strcmp(text, "BOSALTILIYOR")) return "DRAINING";
    if (!strcmp(text, "GIRIS GEREKLI")) return "LOGIN REQUIRED";
    if (!strcmp(text, "YAG DOLUM")) return "OIL FILL";
    if (!strcmp(text, "BASINC")) return "PRESSURE";
    if (!strcmp(text, "TEMIZLEME")) return "CLEANING";
    if (!strcmp(text, "SISTEM")) return "SYSTEM";
    return text;
}

void TaskOLED(void *pvParameters) {
    (void) pvParameters;

    // Wait for I2C bus to be initialized by TaskI2CMonitor
    vTaskDelay(pdMS_TO_TICKS(1500));

    bool display_ok = false;
    if (i2cLock(pdMS_TO_TICKS(200))) {
        display_ok = g_u8g2.begin();
        if (display_ok) {
            g_u8g2.setFont(u8g2_font_6x13_tf);
            g_u8g2.setContrast(255);
            g_u8g2.clearBuffer();
            g_u8g2.setFont(u8g2_font_9x18B_tf);
            g_u8g2.drawStr(10, 30, "Kitronic");
            g_u8g2.setFont(u8g2_font_6x13_tf);
            g_u8g2.drawStr(10, 50, "OLED init OK");
            g_u8g2.sendBuffer();
        }
        i2cUnlock();
    }

    char pressure[16];
    ButtonDisplayEvent event{};

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));

        float bar = 0.0f;
        bool sessionActive = false;
        uint8_t language = 0;
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            bar = pressureBar();
            event = g_buttonDisplayEvent;
            sessionActive = g_controlSession.active;
            language = g_uiLanguage;
            xSemaphoreGive(g_sharedMutex);
        }

        bool showEvent = sessionActive && event.seq != 0 && millis() - event.timestampMs < 3000;
        if (display_ok && i2cLock(pdMS_TO_TICKS(200))) {
            g_u8g2.clearBuffer();
            if (!sessionActive) {
                snprintf(pressure, sizeof(pressure), "%.1f", (double)bar);
                g_u8g2.setFont(u8g2_font_logisoso24_tn);
                int xPressure = (128 - g_u8g2.getStrWidth(pressure)) / 2;
                g_u8g2.drawStr(xPressure > 0 ? xPressure : 0, 27, pressure);
                g_u8g2.setFont(u8g2_font_6x13B_tf);
                const char *message = language == 1 ? "UNLOCK DEVICE" : "KILIDI ACIN";
                int xMessage = (128 - g_u8g2.getStrWidth(message)) / 2;
                g_u8g2.drawStr(xMessage > 0 ? xMessage : 0, 55, message);
            } else if (showEvent) {
                const char *button = translateButtonText(event.button, language);
                const char *state = translateButtonText(event.state, language);
                g_u8g2.setFont(u8g2_font_9x18B_tf);
                int xButton = (128 - g_u8g2.getStrWidth(button)) / 2;
                g_u8g2.drawStr(xButton > 0 ? xButton : 0, 24, button);
                g_u8g2.setFont(u8g2_font_10x20_tf);
                int xState = (128 - g_u8g2.getStrWidth(state)) / 2;
                g_u8g2.drawStr(xState > 0 ? xState : 0, 53, state);
            } else {
                snprintf(pressure, sizeof(pressure), "%.1f", (double)bar);
                g_u8g2.setFont(u8g2_font_logisoso32_tn);
                int xPressure = (128 - g_u8g2.getStrWidth(pressure)) / 2;
                g_u8g2.drawStr(xPressure > 0 ? xPressure : 0, 40, pressure);
                g_u8g2.setFont(u8g2_font_7x14B_tf);
                int xBar = (128 - g_u8g2.getStrWidth("bar")) / 2;
                g_u8g2.drawStr(xBar, 62, "bar");
            }
            g_u8g2.sendBuffer();
            i2cUnlock();
        }
    }
}

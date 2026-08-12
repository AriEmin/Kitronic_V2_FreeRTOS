#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <U8g2lib.h>

#include "Tasks.h"
#include "Shared.h"

// HS96L03W2C03: 128x64 I2C OLED, adres 0x3C
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C g_u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// Pressure conversion: g_pressure0_V * 100 -> bar (matches TelemetrySensor in TaskSerial.cpp)
static inline float pressureBar() {
    return g_pressure0_V * 100.0f;
}

// I2C mutex wrapper
static bool i2cLock(TickType_t timeout) {
    if (!g_i2cMutex) return false;
    return xSemaphoreTake(g_i2cMutex, timeout) == pdTRUE;
}

static void i2cUnlock() {
    if (g_i2cMutex) xSemaphoreGive(g_i2cMutex);
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

    char line0[32];
    char line1[32];
    char line2[32];

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(250));

        float bar = 0.0f;
        float mainV = 0.0f;
        bool ocp = false;

        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            bar = pressureBar();
            mainV = g_mainPwrVoltage_V;
            xSemaphoreGive(g_sharedMutex);
        }

        // OCP latch is volatile; read with critical section for size-safe access
        portENTER_CRITICAL(&g_portMux);
        ocp = g_drvOcpLatch;
        portEXIT_CRITICAL(&g_portMux);

        snprintf(line0, sizeof(line0), "Basinc: %.1f bar", (double)bar);
        snprintf(line1, sizeof(line1), "Main V: %.1f V", (double)mainV);
        if (ocp) {
            snprintf(line2, sizeof(line2), "UYARI: OCP AKTIF");
        } else {
            snprintf(line2, sizeof(line2), "Sistem Normal");
        }

        if (display_ok && i2cLock(pdMS_TO_TICKS(200))) {
            g_u8g2.clearBuffer();
            g_u8g2.setFont(u8g2_font_9x18B_tf);
            g_u8g2.drawStr(0, 18, line0);
            g_u8g2.setFont(u8g2_font_7x14_tf);
            g_u8g2.drawStr(0, 40, line1);
            if (ocp) {
                g_u8g2.drawStr(0, 58, line2);
            } else {
                g_u8g2.drawStr(0, 58, line2);
            }
            g_u8g2.sendBuffer();
            i2cUnlock();
        }
    }
}

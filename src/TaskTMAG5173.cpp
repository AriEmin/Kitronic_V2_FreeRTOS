// TaskTMAG5173.cpp
// TMAG5173-Q1 manyetik sensörleri TCA9548A I2C mux üzerinden okur
// 8 kanal: 4 vites pistonu (sadece Z), 4 kavrama sensörü (X,Y,Z)

#include <Arduino.h>
#include <Wire.h>
#include "Tasks.h"
#include "Shared.h"
#include "modules/tca9548a.h"
#include "modules/tmag5173.h"

// Global TMAG5173 objeleri Shared.h/cpp'de tanımlanmış (extern)

// Kanal isimleri (debug için)
static const char* kChannelNames[TMAG_CH_COUNT] = {
    "1_3", "5_7", "2_4", "6_R",
    "K1_1", "K1_2", "K2_1", "K2_2"
};

// I2C erişimi için güvenli wrapper (global mutex kullanır)
static bool i2cLock(uint32_t timeout_ms = 50) {
    if (!g_i2cMutex) return true;  // Mutex yoksa devam et
    return xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

static void i2cUnlock() {
    if (g_i2cMutex) xSemaphoreGive(g_i2cMutex);
}

void TaskTMAG5173(void *pvParameters) {
    (void)pvParameters;

    // I2C zaten TaskI2CMonitor'da başlatılmış olmalı
    // Biraz bekle ki diğer I2C cihazları init olsun
    vTaskDelay(pdMS_TO_TICKS(1000));

    // TCA9548A mux'u kontrol et
    bool muxOk = false;
    if (i2cLock(100)) {
        muxOk = g_mux.begin();
        i2cUnlock();
    }
    
    {
        char msg[64];
        snprintf(msg, sizeof(msg), "[TMAG] MUX %s (0x%02X)", 
                 muxOk ? "OK" : "FAIL", TCA9548A_ADDR);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
    }

    if (!muxOk) {
        // Mux bulunamadı, task'ı durdurma ama periyodik deneme yap
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(5000));
            if (i2cLock(100)) {
                muxOk = g_mux.begin();
                i2cUnlock();
            }
            if (muxOk) {
                {
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[TMAG] MUX retry OK");
                }
                break;
            }
        }
    }

    // Her kanalı başlat (I2C mutex ile)
    uint8_t okCount = 0;
    if (i2cLock(200)) {
        for (uint8_t ch = 0; ch < TMAG_CH_COUNT; ch++) {
            g_mux.selectChannel(ch);
            vTaskDelay(pdMS_TO_TICKS(10));  // Kanal değişimi için kısa bekleme
            
            g_tmagOk[ch] = g_tmag[ch].begin();
            if (g_tmagOk[ch]) okCount++;
            
            {
                char msg[48];
                snprintf(msg, sizeof(msg), "[TMAG] ch%d (%s): %s", 
                         ch, kChannelNames[ch], g_tmagOk[ch] ? "OK" : "FAIL");
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
            }
        }
        i2cUnlock();
    }

    {
        char msg[48];
        snprintf(msg, sizeof(msg), "[TMAG] %d/%d sensors ready", okCount, TMAG_CH_COUNT);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
    }

    // Ana okuma döngüsü
    for (;;) {
        TMAG5173_Reading readings[TMAG_CH_COUNT];
        
        // I2C mutex al
        if (i2cLock(50)) {
            // Tüm kanalları oku
            for (uint8_t ch = 0; ch < TMAG_CH_COUNT; ch++) {
                if (!g_tmagOk[ch]) {
                    readings[ch].valid = false;
                    readings[ch].x = 0;
                    readings[ch].y = 0;
                    readings[ch].z = 0;
                    continue;
                }

                // Mux kanalını seç
                if (!g_mux.selectChannel(ch)) {
                    readings[ch].valid = false;
                    continue;
                }

                // X, Y, Z oku
                int16_t x, y, z;
                if (g_tmag[ch].readXYZ_raw(x, y, z)) {
                    readings[ch].x = x;
                    readings[ch].y = y;
                    readings[ch].z = z;
                    readings[ch].valid = true;
                } else {
                    readings[ch].valid = false;
                    // Sensör ile iletişim kesilmiş olabilir, yeniden init dene
                    g_tmagOk[ch] = g_tmag[ch].begin();
                }
            }
            i2cUnlock();
        } else {
            // Mutex alınamadı, tüm okumaları invalid yap
            for (uint8_t ch = 0; ch < TMAG_CH_COUNT; ch++) {
                readings[ch].valid = false;
            }
        }

        // Shared'e güvenli yaz
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            for (uint8_t ch = 0; ch < TMAG_CH_COUNT; ch++) {
                g_tmagData[ch] = readings[ch];
            }
            xSemaphoreGive(g_sharedMutex);
        }

        // 100ms periyot (~10Hz) - diğer I2C task'lara yer bırak
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

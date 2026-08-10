// TaskHallTest.cpp
// Hall sensör stabilite testi - yüksek hızlı sampling ve analiz
// TMAG5173 sensörlerden direkt Z ekseni okuma, 1ms periyot desteği

#include <Arduino.h>
#include "Tasks.h"
#include "Shared.h"
#include "modules/tca9548a.h"
#include "modules/tmag5173.h"

// Global TMAG5173 objeleri Shared.h'de extern olarak tanımlanmış

// Piston -> TMAG kanal eşleşmesi (TaskValveControl.cpp ile aynı)
static const uint8_t PISTON_TO_TMAG[6] = {
    TMAG_CH_1_3,   // PISTON_1_3 -> 1
    TMAG_CH_5_7,   // PISTON_5_7 -> 0
    TMAG_CH_2_4,   // PISTON_2_4 -> 2
    TMAG_CH_6_R,   // PISTON_6_R -> 3
    TMAG_CH_K1_2,  // PISTON_K1  -> 5 (birincil)
    TMAG_CH_K2_2   // PISTON_K2  -> 7 (birincil)
};

// Kalibrasyon'dan mm hesapla (sadece test için basit interpolasyon)
static float rawToMm(int16_t raw, uint8_t pistonIdx) {
    if (pistonIdx < 4) {
        // Vites pistonları (0-26mm)
        const TMAGPistonCalib& cal = g_tmagPistonCalib[pistonIdx];
        if (!cal.valid) {
            // Kalibrasyon yok - varsayılan
            return ((float)(raw + 2000) / 4000.0f) * 26.0f;
        }
        // Parçalı lineer interpolasyon
        float zMinF = (float)cal.zMin;
        float zMaxF = (float)cal.zMax;
        float zMidF = (float)cal.zMid;
        float zF = (float)raw;
        float halfStroke = cal.strokeMm * 0.5f;
        
        if (cal.zMid == 0 || fabsf(zMidF - zMinF) < 10.0f) {
            // Basit lineer
            float span = zMaxF - zMinF;
            if (fabsf(span) < 10.0f) return 0.0f;
            return ((zF - zMinF) / span) * cal.strokeMm;
        }
        
        if (zMinF < zMaxF) {
            if (zF <= zMidF) {
                return ((zF - zMinF) / (zMidF - zMinF)) * halfStroke;
            } else {
                return halfStroke + ((zF - zMidF) / (zMaxF - zMidF)) * halfStroke;
            }
        } else {
            if (zF >= zMidF) {
                return ((zMinF - zF) / (zMinF - zMidF)) * halfStroke;
            } else {
                return halfStroke + ((zMidF - zF) / (zMidF - zMaxF)) * halfStroke;
            }
        }
    } else {
        // K1/K2 kavrama (0-10mm)
        // Basit lineer
        return ((float)(raw + 2000) / 4000.0f) * 10.0f;
    }
}

// Tek bir TMAG sensöründen Z değeri oku (direkt, task döngüsünden)
static bool readTmagZ(uint8_t tmagCh, int16_t& outZ) {
    if (tmagCh >= TMAG_CH_COUNT || !g_tmagOk[tmagCh]) {
        return false;
    }
    
    // Mux kanal seç
    if (!g_mux.selectChannel(tmagCh)) {
        return false;
    }
    
    // Z ekseni oku (TMAG5173'ün hazır readZ_raw metodunu kullan)
    outZ = g_tmag[tmagCh].readZ_raw();
    return true;
}

// Hall test verisi gönder (JSON) - her 5. örneği gönder (seri port yükünü azalt)
static void sendHallSample(uint32_t timestampMs, int16_t rawValue, float mmValue, uint8_t pistonIdx, uint32_t sampleCount) {
        
    // Her 5. örneği gönder (200Hz'e kadar veri)
    if (sampleCount % 5 != 0) return;
    
    char msg[128];
    int len = snprintf(msg, sizeof(msg), 
             "{\"_t\":\"HS\",\"p\":%d,\"t\":%u,\"r\":%d,\"m\":%.2f}",
             pistonIdx, timestampMs, rawValue, mmValue);
    
    // Timeout 5ms - queue doluysa bekle, yoksa atla
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
}

// Test tamamlandı bildirimi
static void sendHallDone(uint8_t pistonIdx, uint32_t sampleCount) {
        
    char msg[64];
    snprintf(msg, sizeof(msg), 
             "{\"_t\":\"HS\",\"done\":true,\"p\":%d,\"n\":%u}",
             pistonIdx, sampleCount);
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
}

void TaskHallTest(void *pvParameters) {
    (void)pvParameters;
    
    uint32_t lastReqSeq = 0;
    
    // Test durumu
    bool isRunning = false;
    uint32_t testStartTime = 0;
    uint32_t nextSampleTime = 0;
    uint32_t sampleCount = 0;
    uint8_t targetPiston = 0;
    uint8_t sampleIntervalMs = 1;
    uint16_t durationMs = 1000;
    uint8_t testType = 0;  // 0=static, 1=emi, 2=dynamic
    
    // I2C mutex için kısa timeout (test esnasında normal TMAG task'ı durur)
    const uint32_t I2C_TIMEOUT_MS = 10;
    
    for (;;) {
        // İstek kontrolü
        bool shouldStart = false;
        bool shouldStop = false;
        
        if (g_hallStabilityReqSeq != lastReqSeq) {
            lastReqSeq = g_hallStabilityReqSeq;
            
            if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                HallStabilityRequest req = g_hallStabilityReq;
                xSemaphoreGive(g_sharedMutex);
                
                if (req.start && !isRunning) {
                    shouldStart = true;
                    targetPiston = req.pistonIdx;
                    durationMs = req.durationMs;
                    sampleIntervalMs = req.sampleIntervalMs;
                    testType = req.testType;
                } else if (!req.start && isRunning) {
                    shouldStop = true;
                }
            }
        }
        
        // Test başlat
        if (shouldStart) {
            isRunning = true;
            g_hallStabilityRunning = true;
            testStartTime = millis();
            nextSampleTime = testStartTime;
            sampleCount = 0;
            
            // Test tipine göre hazırlık
            if (testType == 1) {
                // EMI Test: Pompa çalıştır
                {
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, "[HS] EMI mode: Pump ON");
                }
                // Pompa çalıştırma (g_pumpCmd kullanarak)
                if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    g_pumpCmd.cmd = PUMP_CMD_START;
                    g_pumpCmd.seq++;
                    g_pumpCmd.setRpm = 3000.0f;  // 3000 RPM
                    xSemaphoreGive(g_sharedMutex);
                }
            } else if (testType == 2) {
                // Dinamik Test: Valf aç, piston hareket etsin
                {
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, "[HS] Dynamic mode: Valve+PCV open");
                }
                // İlgili valfi aç (OPEN mod - 1500 duty)
                // Piston mapping: 0=P1-3(N433), 1=P5-7(N434), 2=P2-4(N437), 3=P6-R(N438)
                // Valf index:     0      ,     2      ,     7      ,     4
                const uint8_t pistonToValve[6] = {0, 2, 7, 4, 3, 6};  // K1=N435(3), K2=N439(6)
                if (targetPiston < 6) {
                    uint8_t valveIdx = pistonToValve[targetPiston];
                    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        g_valveTargetDuty[valveIdx] = 1500;  // Piston valfi OPEN
                        g_valveTargetDuty[1] = 1500;         // PCV N436 OPEN
                        g_valveTargetDuty[5] = 1500;         // PCV N440 OPEN
                        xSemaphoreGive(g_sharedMutex);
                    }
                }
            }
            
            {
                char msg[64];
                snprintf(msg, sizeof(msg), 
                         "[HS] START piston=%d type=%d duration=%dms",
                         targetPiston, testType, durationMs);
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
            }
        }
        
        // Test çalışıyorsa örnekleme yap
        if (isRunning) {
            uint32_t now = millis();
            
            // Zaman aşımı kontrolü
            if (now - testStartTime >= durationMs) {
                shouldStop = true;
            }
            
            // Örnekleme zamanı geldi mi?
            if (now >= nextSampleTime && !shouldStop) {
                nextSampleTime = now + sampleIntervalMs;
                
                // TMAG Z oku (I2C mutex ile)
                int16_t rawZ = 0;
                bool ok = false;
                
                if (g_i2cMutex && xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == pdTRUE) {
                    uint8_t tmagCh = PISTON_TO_TMAG[targetPiston];
                    ok = readTmagZ(tmagCh, rawZ);
                    xSemaphoreGive(g_i2cMutex);
                }
                
                if (ok) {
                    float mm = rawToMm(rawZ, targetPiston);
                    uint32_t timestamp = now - testStartTime;
                    
                    // Veri gönder (içeride her 5. örnek filtresi var)
                    sendHallSample(timestamp, rawZ, mm, targetPiston, sampleCount);
                    sampleCount++;
                }
            }
        }
        
        // Test durdur
        if (shouldStop && isRunning) {
            isRunning = false;
            g_hallStabilityRunning = false;
            
            // Test tipine göre temizlik
            if (testType == 1) {
                // EMI Test: Pompa durdur
                if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    g_pumpCmd.cmd = PUMP_CMD_STOP;
                    g_pumpCmd.seq++;
                    xSemaphoreGive(g_sharedMutex);
                }
                {
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, "[HS] EMI mode: Pump OFF");
                }
            } else if (testType == 2) {
                // Dinamik Test: Valf kapat
                {
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, "[HS] Dynamic mode: Valve+PCV closed");
                }
                const uint8_t pistonToValve[6] = {0, 2, 7, 4, 3, 6};  // K1=N435(3), K2=N439(6)
                if (targetPiston < 6) {
                    uint8_t valveIdx = pistonToValve[targetPiston];
                    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        g_valveTargetDuty[valveIdx] = 0;   // Piston valfi CLOSE
                        g_valveTargetDuty[1] = 0;          // PCV N436 CLOSE
                        g_valveTargetDuty[5] = 0;          // PCV N440 CLOSE
                        xSemaphoreGive(g_sharedMutex);
                    }
                }
            }
            
            sendHallDone(targetPiston, sampleCount);
            
            {
                char msg[48];
                snprintf(msg, sizeof(msg), "[HS] DONE samples=%u", sampleCount);
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
            }
        }
        
        // Kısa bekleme (1ms - yüksek hızlı polling için)
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

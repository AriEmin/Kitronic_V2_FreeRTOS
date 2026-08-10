// src/TaskValveDiag.cpp
// Valf Diagnostik Task - PWM Rampa Testi
// 0 → maxDuty, stepCount adımla, stepMs periyotla
// Her adımda INA219 (mA/V) + hall (TMAG) + basınç okur
// JSON tipi "VD" ile gönderir: {v, t, d, I, V, h, p}
// Test bitince {_t:"VD", v:"N433", done:true} gönderir
// g_valveDiagRunning = true iken S/P/I/W/T telemetri duraklar

#include "Shared.h"
#include "Tasks.h"
#include "Protocol.h"
#include <Arduino.h>
#include <ArduinoJson.h>

// Valf ismi (indeks → string)
static const char* valveNames[] = {
    "N433", "N436", "N434", "N435", "N438", "N440", "N439", "N437"
};

// Valf → TMAG piston kanalı (hall okuma için)
// 0=N433(P1-3), 2=N434(P5-7), 7=N437(P2-4), 4=N438(P6-R)
// Diğerleri (kavrama/ana) için -1 (hall yok)
static const int8_t valveToTmag[] = {
    TMAG_CH_1_3,   // 0: N433 → P1-3
    -1,            // 1: N436 → PCV (hall yok)
    TMAG_CH_5_7,   // 2: N434 → P5-7
    4,             // 3: N435 → K1 (TMAG_CH_K1_1)
    TMAG_CH_6_R,   // 4: N438 → P6-R
    -1,            // 5: N440 → PCV (hall yok)
    6,             // 6: N439 → K2 (TMAG_CH_K2_1)
    TMAG_CH_2_4,   // 7: N437 → P2-4
};

void TaskValveDiag(void *pvParameters) {
    (void)pvParameters;

    static uint32_t lastReqSeq = 0;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(20));

        // Yeni istek var mı?
        if (g_valveDiagReqSeq == lastReqSeq) continue;
        lastReqSeq = g_valveDiagReqSeq;

        // İstek kopyası
        ValveDiagRequest req;
        if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            req = g_valveDiagReq;
            xSemaphoreGive(g_sharedMutex);
        } else {
            continue;
        }

        if (req.valveIdx >= 8) continue;

        // Varsayılan değerler
        if (req.dutyMax == 0)        req.dutyMax = 2000;
        if (req.dutyStep == 0)       req.dutyStep = 10;
        if (req.stepMs == 0)         req.stepMs = 100;
        if (req.pressureTarget == 0) req.pressureTarget = 50.0f;

        // Adım sayısını türet
        uint16_t stepCount = req.dutyMax / req.dutyStep;
        if (stepCount == 0) stepCount = 1;

        const char* valveName = valveNames[req.valveIdx];
        int8_t tmagCh = valveToTmag[req.valveIdx];

        // Testi başlat
        g_valveDiagRunning = true;

        {
            char msg[80];
            snprintf(msg, sizeof(msg), "[VD] Test basliyor: %s duty=0..%u step=%u ms=%u",
                     valveName, req.dutyMax, stepCount, req.stepMs);
            kitronic::SerialTx_SendLog(kitronic::MsgCode::VALVE_DIAG_STARTED, msg);
        }

        // Pompayı AUTO modda başlat - basınç bekle
        g_pumpCmd.cmd = PUMP_CMD_AUTO;
        g_pumpCmd.seq++;

        // Ana valfleri aç
        g_valveTargetDuty[1] = 2000;  // N436
        g_valveTargetDuty[5] = 2000;  // N440

        // Basınç bekleme (max 15 saniye)
        uint32_t pressureWaitStart = millis();
        bool pressureOk = false;
        while (millis() - pressureWaitStart < 15000) {
            if (g_pumpPub.bar >= req.pressureTarget) {
                pressureOk = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (!pressureOk) {
            char msg[80];
            snprintf(msg, sizeof(msg), "[VD] IPTAL: %.0f bar basinca ulasilamadi (%.1f bar)",
                     req.pressureTarget, g_pumpPub.bar);
            kitronic::SerialTx_SendLog(kitronic::MsgCode::VALVE_DIAG_DONE, msg);
            g_valveDiagRunning = false;
            continue;
        }

        // Diğer tüm valfleri kapat (test valfi hariç)
        for (int i = 0; i < 8; i++) {
            if (i != req.valveIdx && i != 1 && i != 5) {  // Ana valfler açık kalır
                g_valveTargetDuty[i] = 0;
            }
        }

        // PWM Rampa Testi
        for (uint16_t step = 0; step <= stepCount; step++) {
            uint16_t duty = (uint16_t)((uint32_t)req.dutyMax * step / stepCount);
            g_valveTargetDuty[req.valveIdx] = duty;

            vTaskDelay(pdMS_TO_TICKS(req.stepMs));

            // Ölçüm al
            float current_mA = g_valveCurrent_mA[req.valveIdx];
            float voltage_V  = g_valveBusVoltage_V[req.valveIdx];
            int16_t hallRaw  = (tmagCh >= 0) ? g_tmagData[tmagCh].z : 0;
            float pressure   = g_pumpPub.bar;

            // FT_EVENT frame: {_t:"VD", v:"N433", t:ms, d:duty, I:mA, V:mV, h:hall, p:bar*10}
            static JsonDocument doc;
            doc.clear();
            doc["_t"] = "VD";
            doc["v"]  = valveName;
            doc["t"]  = (uint32_t)millis();
            doc["d"]  = duty;
            doc["I"]  = (int)round(current_mA);
            doc["V"]  = (int)round(voltage_V * 1000.0f);  // mV
            doc["h"]  = hallRaw;
            doc["p"]  = (int)round(pressure * 10.0f);     // bar*10
            kitronic::SerialTx_SendEvent(doc);
        }

        // Valfi kapat
        g_valveTargetDuty[req.valveIdx] = 0;

        // Done mesajı gönder
        {
            static JsonDocument doneDoc;
            doneDoc.clear();
            auto& doc = doneDoc;
            doc["_t"] = "VD";
            doc["v"]  = valveName;
            doc["done"] = true;
            kitronic::SerialTx_SendEvent(doc);
        }

        {
            char msg[64];
            snprintf(msg, sizeof(msg), "[VD] Test tamamlandi: %s", valveName);
            kitronic::SerialTx_SendLog(kitronic::MsgCode::VALVE_DIAG_DONE, msg);
        }

        g_valveDiagRunning = false;
    }
}

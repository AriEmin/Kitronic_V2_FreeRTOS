/**
 * TaskAutoTest.cpp
 * 4-Fazli Ariza Tespiti
 *
 * Faz 0 : Elektriksel Valf Kontrolu      (8 valf bobini akim olcumu)
 * Faz 1 : Pompa Doldurma Testi           (doldurma suresi + pompa akimi)
 * Faz 2 : Hizli Hava Alma                (6 piston tek tek 10 kere ac/kapa)
 * Faz 3 : Piston Kalibrasyonu            (6 piston icin kapali/acik hall degerleri)
 *
 * Tetikleme: g_autoTestReqSeq arttirilinca test baslar
 * Durdurma : g_autoTestStop = true
 * Sonuclar : g_autoTestResult
 * Telemetri: JSON tipi "AT" ile GUI'ye iletilir
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include "Tasks.h"
#include "Shared.h"

// -----------------------------------------------------------------------------
// Sabitler / Mapping
// -----------------------------------------------------------------------------
static const int PISTON_VALVE_IDX[PISTON_CHANNEL_COUNT] = {2, 0, 7, 4, 3, 6};
// PISTON_5_7->N434(2), PISTON_1_3->N433(0), PISTON_2_4->N437(7),
// PISTON_6_R->N438(4), PISTON_K1->N435(3), PISTON_K2->N439(6)

static const uint8_t PISTON_TMAG_CH[PISTON_CHANNEL_COUNT] = {
    TMAG_CH_5_7,   // PISTON_5_7 = 0
    TMAG_CH_1_3,   // PISTON_1_3 = 1
    TMAG_CH_2_4,   // PISTON_2_4 = 2
    TMAG_CH_6_R,   // PISTON_6_R = 3
    TMAG_CH_K1_2,  // PISTON_K1  = 4 (kapali/birincil sensor)
    TMAG_CH_K2_2,  // PISTON_K2  = 5 (kapali/birincil sensor)
};

static const uint8_t PCV_INDEX[2] = {1, 5}; // N436, N440

static const char* PISTON_NAME[PISTON_CHANNEL_COUNT] = {
    "P5-7", "P1-3", "P2-4", "P6-R", "K1", "K2"
};
static const char* VALVE_NAME[8] = {
    "N433", "N436", "N434", "N435", "N438", "N440", "N439", "N437"
};

static const uint8_t VALVE_TO_INA[8] = {0, 3, 1, 2, 5, 7, 6, 4};

// -----------------------------------------------------------------------------
// Makrolar
// -----------------------------------------------------------------------------
#define AT_CHECK_STOP() do { if (g_autoTestStop) { return false; } } while(0)

// -----------------------------------------------------------------------------
// Yardimci fonksiyonlar
// -----------------------------------------------------------------------------
static void atLog(const char* msg) {
    char buf[128];
    snprintf(buf, sizeof(buf), "[AT] %s", msg);
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
}

static void atAllValvesOff() {
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int i = 0; i < 8; i++) {
            g_valveTargetDuty[i] = 0;
            g_valveCustomCurrent_mA[i] = 0.0f;
        }
        xSemaphoreGive(g_sharedMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void atSetValveCurrent(int valveIdx, float mA) {
    if (valveIdx < 0 || valveIdx >= 8) return;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_valveTargetDuty[valveIdx] = 0;
        g_valveCustomCurrent_mA[valveIdx] = mA;
        xSemaphoreGive(g_sharedMutex);
    }
}

static void atAllValvesCloseCurrent() {
    const auto& p = g_autoTestParams;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (int i = 0; i < 8; i++) {
            g_valveTargetDuty[i] = 0;
            g_valveCustomCurrent_mA[i] = p.valveCloseCurrent_mA[i];
        }
        xSemaphoreGive(g_sharedMutex);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
}

static float atGetValveCurrent(int valveIdx) {
    if (valveIdx < 0 || valveIdx >= 8) return 0.0f;
    int inaIdx = VALVE_TO_INA[valveIdx];
    float mA = 0.0f;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        mA = g_tele.inaI_mA[inaIdx];
        xSemaphoreGive(g_sharedMutex);
    }
    return mA;
}

static float atGetPressure() {
    float p = 0.0f;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        p = g_pumpPub.bar;
        xSemaphoreGive(g_sharedMutex);
    }
    return p;
}

static float atGetPumpCurrent() {
    float a = 0.0f;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        a = g_tele.vescI;
        xSemaphoreGive(g_sharedMutex);
    }
    return a;
}

static float atGetRpm() {
    int32_t rpm = 0;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        rpm = g_pumpPub.rpm;
        xSemaphoreGive(g_sharedMutex);
    }
    return (float)fabsf((float)rpm);
}

static int16_t atGetHall(uint8_t pistonIdx) {
    if (pistonIdx >= PISTON_CHANNEL_COUNT) return 0;
    int16_t val = 0;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        val = g_tmagData[PISTON_TMAG_CH[pistonIdx]].z;
        xSemaphoreGive(g_sharedMutex);
    }
    return val;
}

static void atPumpStart() {
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_pumpCmd.cmd = PUMP_CMD_START;
        g_pumpCmd.seq++;
        xSemaphoreGive(g_sharedMutex);
    }
}

static void atPumpAuto() {
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_pumpCmd.cmd = PUMP_CMD_AUTO;
        g_pumpCmd.seq++;
        xSemaphoreGive(g_sharedMutex);
    }
}

static void atPumpStop() {
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_pumpCmd.cmd = PUMP_CMD_STOP;
        g_pumpCmd.seq++;
        xSemaphoreGive(g_sharedMutex);
    }
}

/** Basinci min..max araliginda tut (histeresis) */
static void atPressureMaintain(float minBar, float maxBar) {
    static float lastCmdBar = 0.0f;
    float p = atGetPressure();
    if (p < minBar && lastCmdBar < minBar) {
        atPumpStart();
        lastCmdBar = minBar;
    } else if (p > maxBar && lastCmdBar > 0.0f) {
        atPumpStop();
        lastCmdBar = 0.0f;
    }
}

static void atResetPressureMaintain() {
    atPressureMaintain(0.0f, -1.0f); // durdur
}

static void atSendPhaseUpdate(int phaseIdx) {
    static JsonDocument doc;
    doc.clear();
    doc["_t"]    = "AT";
    doc["phase"] = phaseIdx;
    doc["run"]   = g_autoTestResult.running;
    doc["done"]  = g_autoTestResult.done;
    doc["pass"]  = g_autoTestResult.pass;
    if (phaseIdx >= 0 && phaseIdx < 4) {
        auto& ph = g_autoTestResult.phases[phaseIdx];
        doc["ph_done"] = ph.done;
        doc["ph_pass"] = ph.pass;
        doc["ph_det"]  = ph.detail;
        doc["ph_meas"] = ph.measured;
        doc["ph_fm"]   = ph.faultMask;
    }
    if (phaseIdx == 3) {
        // Faz 3 kalibrasyon sonuclarini gonder
        auto arr = doc["pistons"].to<JsonArray>();
        for (int i = 0; i < PISTON_CHANNEL_COUNT; i++) {
            auto& c = g_autoTestResult.pistonCalib[i];
            auto o = arr.add<JsonObject>();
            o["name"]    = c.name;
            o["closed"]  = c.closedRaw;
            o["open"]    = c.openRaw;
            o["stroke"]  = c.strokeMm;
            o["valid"]   = c.valid;
        }
    }
    kitronic::SerialTx_SendEvent(doc, true);
}

static void atFinishPhase(int phaseIdx, bool pass, const char* detail, float measured = 0.0f, uint8_t faultMask = 0) {
    if (phaseIdx < 0 || phaseIdx >= 4) return;
    auto& ph    = g_autoTestResult.phases[phaseIdx];
    ph.done     = true;
    ph.pass     = pass;
    ph.measured = measured;
    ph.faultMask= faultMask;
    snprintf(ph.detail, sizeof(ph.detail), "%s", detail);
    if (!pass) g_autoTestResult.pass = false;
    atSendPhaseUpdate(phaseIdx);

    char logMsg[96];
    snprintf(logMsg, sizeof(logMsg), "Faz %d %s: %s", phaseIdx, pass ? "GECTI" : "KALDI", detail);
    atLog(logMsg);
}

// -----------------------------------------------------------------------------
// FAZ 0: Elektriksel Valf Kontrolu
// -----------------------------------------------------------------------------
static bool atPhase0_ValveElecCheck() {
    const auto& p = g_autoTestParams;
    const float COIL_MIN = p.valveCoilMinCurrent_mA;
    const uint32_t SETTLE_MS = 400;

    atLog("Faz 0: Elektriksel valf kontrolu");
    atAllValvesCloseCurrent();
    vTaskDelay(pdMS_TO_TICKS(200));

    char failList[72] = "";
    uint8_t failMask = 0;
    char logBuf[80];

    for (int v = 0; v < 8; v++) {
        AT_CHECK_STOP();
        atSetValveCurrent(v, p.valveOpenCurrent_mA[v]);
        vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));

        float mA = atGetValveCurrent(v);

        atSetValveCurrent(v, p.valveCloseCurrent_mA[v]);
        vTaskDelay(pdMS_TO_TICKS(100));

        snprintf(logBuf, sizeof(logBuf), "Faz 0: %s %.0f mA", VALVE_NAME[v], mA);
        atLog(logBuf);

        if (mA < COIL_MIN) {
            failMask |= (uint8_t)(1u << v);
            if (strlen(failList) > 0)
                strncat(failList, ", ", sizeof(failList) - strlen(failList) - 1);
            strncat(failList, VALVE_NAME[v], sizeof(failList) - strlen(failList) - 1);
        }
    }

    if (failMask == 0) {
        atFinishPhase(0, true, "OK - Tum valfler elektriksel baglanti OK", 0, 0);
        return true;
    }

    char detail[96];
    snprintf(detail, sizeof(detail), "ARIZA: %s acik devre / baglanti sorunu", failList);
    atFinishPhase(0, false, detail, 0, failMask);
    return false;
}

// -----------------------------------------------------------------------------
// FAZ 1: Pompa Doldurma Testi
// -----------------------------------------------------------------------------
static bool atPhase1_PumpFill() {
    const auto& p = g_autoTestParams;

    atLog("Faz 1: Pompa doldurma testi");
    atAllValvesCloseCurrent();
    atSetValveCurrent(PCV_INDEX[0], p.valveOpenCurrent_mA[PCV_INDEX[0]]);
    atSetValveCurrent(PCV_INDEX[1], p.valveOpenCurrent_mA[PCV_INDEX[1]]);

    atPumpStart();
    uint32_t t0 = millis();
    float maxI = 0.0f;
    float fillTime = -1.0f;
    bool filled = false;
    bool currentFault = false;
    bool timeoutFault = false;

    while (true) {
        AT_CHECK_STOP();
        vTaskDelay(pdMS_TO_TICKS(200));

        float bar = atGetPressure();
        float cur = atGetPumpCurrent();
        if (cur > maxI) maxI = cur;

        if (!filled && bar >= p.pumpTargetPressure_bar) {
            fillTime = (millis() - t0) / 1000.0f;
            filled = true;
            atPumpStop();
        }

        if (!filled && (millis() - t0) > (uint32_t)(p.pumpFillTimeout_s * 1000.0f)) {
            timeoutFault = true;
            atPumpStop();
            break;
        }

        if (maxI > p.pumpMaxCurrent_A) {
            currentFault = true;
            atPumpStop();
            break;
        }

        if (filled) break;
    }

    if (currentFault) {
        char detail[96];
        snprintf(detail, sizeof(detail), "ARIZA: Pompa akimi %.2f A (esik %.2f A)", maxI, p.pumpMaxCurrent_A);
        atFinishPhase(1, false, detail, maxI, 0);
        return false;
    }

    if (timeoutFault) {
        char detail[96];
        snprintf(detail, sizeof(detail), "ARIZA: %u sn icinde %.1f bar doldurulamadi",
                 (unsigned)(p.pumpFillTimeout_s), atGetPressure());
        atFinishPhase(1, false, detail, atGetPressure(), 0);
        return false;
    }

    if (fillTime > p.pumpFillMaxTime_s) {
        char detail[96];
        snprintf(detail, sizeof(detail), "ARIZA: Doldurma suresi %.1f sn (esik %.1f sn)",
                 fillTime, p.pumpFillMaxTime_s);
        atFinishPhase(1, false, detail, fillTime, 0);
        return false;
    }

    char detail[80];
    snprintf(detail, sizeof(detail), "OK - %.1f sn, max %.2f A, %.1f bar",
             fillTime, maxI, atGetPressure());
    atFinishPhase(1, true, detail, fillTime, 0);
    return true;
}

// -----------------------------------------------------------------------------
// FAZ 2: Hizli Hava Alma
// -----------------------------------------------------------------------------
static bool atPhase2_AirBleed() {
    const auto& p = g_autoTestParams;

    atLog("Faz 2: Hizli hava alma");
    atAllValvesCloseCurrent();
    // PCV valflerini acma akiminda ac
    atSetValveCurrent(PCV_INDEX[0], p.valveOpenCurrent_mA[PCV_INDEX[0]]);
    atSetValveCurrent(PCV_INDEX[1], p.valveOpenCurrent_mA[PCV_INDEX[1]]);

    // Test boyunca basinci 42-60 bar arasinda tut
    atPumpAuto();

    bool ok = true;
    for (int piston = 0; piston < PISTON_CHANNEL_COUNT; piston++) {
        AT_CHECK_STOP();
        int vIdx = PISTON_VALVE_IDX[piston];
        char logBuf[80];
        snprintf(logBuf, sizeof(logBuf), "Faz 2: %s hava alma %d cevrim", PISTON_NAME[piston], (int)p.airBleedCycles);
        atLog(logBuf);

        for (int c = 0; c < p.airBleedCycles; c++) {
            AT_CHECK_STOP();
            atPressureMaintain(p.pumpMinPressure_bar, p.pumpMaxPressure_bar);
            atSetValveCurrent(vIdx, p.pistonOpenCurrent_mA[piston]);
            vTaskDelay(pdMS_TO_TICKS(p.airBleedOpenMs));
            atSetValveCurrent(vIdx, p.pistonCloseCurrent_mA[piston]);
            vTaskDelay(pdMS_TO_TICKS(p.airBleedCloseMs));
        }
        // Pistonu kapatma akiminda tut
        atSetValveCurrent(vIdx, p.pistonCloseCurrent_mA[piston]);
    }

    atPumpStop();
    if (ok) {
        atFinishPhase(2, true, "OK - Tum pistonlar hava alindi", 0, 0);
        return true;
    }
    return false;
}

// -----------------------------------------------------------------------------
// FAZ 3: Piston Kalibrasyonu
// -----------------------------------------------------------------------------
static bool atPhase3_PistonCalib() {
    const auto& p = g_autoTestParams;

    atLog("Faz 3: Piston kalibrasyonu");
    atAllValvesCloseCurrent();
    atSetValveCurrent(PCV_INDEX[0], p.valveOpenCurrent_mA[PCV_INDEX[0]]);
    atSetValveCurrent(PCV_INDEX[1], p.valveOpenCurrent_mA[PCV_INDEX[1]]);
    atPumpAuto();

    bool overall = true;
    for (int piston = 0; piston < PISTON_CHANNEL_COUNT; piston++) {
        AT_CHECK_STOP();
        int vIdx = PISTON_VALVE_IDX[piston];

        char logBuf[80];
        snprintf(logBuf, sizeof(logBuf), "Faz 3: %s kalibrasyon", PISTON_NAME[piston]);
        atLog(logBuf);

        // Kapali konum (valf zaten close current'ta)
        vTaskDelay(pdMS_TO_TICKS(p.pistonCalibSettleMs));
        int16_t closedRaw = atGetHall(piston);

        // Ac
        atSetValveCurrent(vIdx, p.pistonOpenCurrent_mA[piston]);
        vTaskDelay(pdMS_TO_TICKS(p.pistonCalibOpenMs));
        atPressureMaintain(p.pumpMinPressure_bar, p.pumpMaxPressure_bar);
        vTaskDelay(pdMS_TO_TICKS(p.pistonCalibSettleMs));
        int16_t openRaw = atGetHall(piston);

        // Kapat
        atSetValveCurrent(vIdx, p.pistonCloseCurrent_mA[piston]);
        vTaskDelay(pdMS_TO_TICKS(p.pistonCalibCloseMs));
        vTaskDelay(pdMS_TO_TICKS(p.pistonCalibSettleMs));

        int16_t span = (int16_t)abs((int)openRaw - (int)closedRaw);
        bool valid = (span > 100); // minimum hareket algilama

        // Sonucu global kalibrasyon yapilarina ve sonuc struct'ina kaydet
        if (valid) {
            if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_tmagPistonCalib[piston].zMin     = closedRaw;
                g_tmagPistonCalib[piston].zMax     = openRaw;
                g_tmagPistonCalib[piston].strokeMm = 26.0f;
                g_tmagPistonCalib[piston].valid    = true;
                xSemaphoreGive(g_sharedMutex);
            }
            TMAGCalib_SavePiston(piston);
            g_tmagCalibSeq++;
        }

        auto& res = g_autoTestResult.pistonCalib[piston];
        snprintf(res.name, sizeof(res.name), "%s", PISTON_NAME[piston]);
        res.closedRaw = closedRaw;
        res.openRaw   = openRaw;
        res.strokeMm  = 26.0f;
        res.valid     = valid;

        snprintf(logBuf, sizeof(logBuf), "Faz 3: %s closed=%d open=%d span=%d %s",
                 PISTON_NAME[piston], closedRaw, openRaw, span, valid ? "OK" : "HATA");
        atLog(logBuf);

        if (!valid) overall = false;
    }

    atPumpStop();
    if (overall) {
        atFinishPhase(3, true, "OK - Tum piston kalibrasyonlari kaydedildi", 0, 0);
        return true;
    }
    atFinishPhase(3, false, "ARIZA: Bazi pistonlarda kalibrasyon yapilamadi", 0, 0);
    return false;
}

// -----------------------------------------------------------------------------
// Ana test akisi
// -----------------------------------------------------------------------------
static void atRunTest() {
    g_autoTestResult = {};
    g_autoTestResult.running    = true;
    g_autoTestResult.done       = false;
    g_autoTestResult.pass       = true;
    g_autoTestResult.startMs    = millis();
    g_autoTestResult.currentPhase = 0;

    atLog("=== ARIZA TESPITI BASLADI ===");

    // Faz 0
    g_autoTestResult.currentPhase = 0;
    if (!atPhase0_ValveElecCheck()) {
        goto done;
    }

    // Faz 1
    g_autoTestResult.currentPhase = 1;
    if (!atPhase1_PumpFill()) {
        goto done;
    }

    // Faz 2
    g_autoTestResult.currentPhase = 2;
    if (!atPhase2_AirBleed()) {
        goto done;
    }

    // Faz 3
    g_autoTestResult.currentPhase = 3;
    if (!atPhase3_PistonCalib()) {
        goto done;
    }

done:
    atAllValvesOff();
    atPumpStop();
    atResetPressureMaintain();

    g_autoTestResult.running  = false;
    g_autoTestResult.done     = true;
    g_autoTestResult.endMs    = millis();

    // T sonucu gonder
    atSendPhaseUpdate(g_autoTestResult.pass ? 3 : g_autoTestResult.currentPhase);

    if (g_autoTestResult.pass) {
        atLog("=== ARIZA TESPITI TAMAMLANDI - GECTI ===");
    } else {
        atLog("=== ARIZA TESPITI DURDURULDU - KALDI ===");
    }
}

// -----------------------------------------------------------------------------
// FreeRTOS Task
// -----------------------------------------------------------------------------
void TaskAutoTest(void* pvParameters) {
    (void)pvParameters;
    uint32_t lastSeq = g_autoTestReqSeq;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));

        if (g_autoTestReqSeq != lastSeq) {
            lastSeq = g_autoTestReqSeq;
            g_autoTestStop = false;
            atRunTest();
            lastSeq = g_autoTestReqSeq; // durdurma sirasinda yeni istek gelirse tekrar calis
        }
    }
}

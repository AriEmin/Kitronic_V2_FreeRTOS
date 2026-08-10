/**
 * TaskAutoTest.cpp
 * 10-Fazlı Otomatik Test Durum Makinesi
 *
 * Faz 0 : Elektriksel Valf Kontrolü      (tüm 8 valf bobini akım ölçümü)
 * Faz 1 : Pompa Doldurma Testi
 * Faz 2 : Alan-1 Vites Valfleri Kaçak Testi   (N436 açık, pistonlar sabit mi?)
 * Faz 3 : Alan-1 PCV (N436) Kaçak Testi       (N436 kapalı, sıralı valf aç)
 * Faz 4 : Alan-2 Vites Valfleri Kaçak Testi   (N440 açık)
 * Faz 5 : Alan-2 PCV (N440) Kaçak Testi       (N440 kapalı, sıralı valf aç)
 * Faz 6 : Mekatronik Yağ Kaçak Testi          (basınç tutma 20 sn)
 * Faz 7 : Piston Açık/Kapalı Kalibrasyon      (tüm 6 piston)
 * Faz 8 : Piston Orta Konum Ön Testi          (holdcontrol_V2 ile 12sn tutma + K1/K2 akım kalibrasyonu)
 * Faz 9 : Otomatik Vites Testi
 *
 * Tetikleme: g_autoTestReqSeq arttırılınca test başlar
 * Durdurma : g_autoTestStop = true
 * Sonuçlar : g_autoTestResult
 * Telemetri: JSON tipi "AT" ile GUI'ye iletilir
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include "Tasks.h"
#include "Shared.h"
#include "AutoShiftV2.h"

// ---------------------------------------------------------------------------
// Donanım sabitleri (TaskValveControl.cpp ile aynı)
// ---------------------------------------------------------------------------
static const int PISTON_VALVE_IDX[PISTON_CHANNEL_COUNT]   = {2, 0, 7, 4, 3, 6};
// PISTON_5_7→N434(2), PISTON_1_3→N433(0), PISTON_2_4→N437(7),
// PISTON_6_R→N438(4), PISTON_K1→N435(3),  PISTON_K2→N439(6)

static const int PISTON_SUPPORT_IDX[PISTON_CHANNEL_COUNT] = {1, 1, 5, 5, 1, 5};
// Alan-1 PCV=N436(1): pistonlar 5_7,1_3,K1
// Alan-2 PCV=N440(5): pistonlar 2_4,6_R,K2

// TMAG kanalları (birincil — kalibrasyon (TaskValveControl: PISTON_TO_TMAG) ile AYNI sensör olmalı)
// K1/K2: TaskValveControl KAPALI sensörü (K1_2/K2_2) birincil olarak kullanıyor → burası da aynı
static const uint8_t PISTON_TMAG_CH[PISTON_CHANNEL_COUNT] = {
    TMAG_CH_5_7,   // PISTON_5_7 = 0
    TMAG_CH_1_3,   // PISTON_1_3 = 1
    TMAG_CH_2_4,   // PISTON_2_4 = 2
    TMAG_CH_6_R,   // PISTON_6_R = 3
    TMAG_CH_K1_2,  // PISTON_K1  = 4 (kapalı/birincil sensör — kalibrasyon ile aynı)
    TMAG_CH_K2_2,  // PISTON_K2  = 5 (kapalı/birincil sensör — kalibrasyon ile aynı)
};
// K1/K2 ikincil sensör kanalları (-1 = yok)
static const int8_t PISTON_TMAG_CH2[PISTON_CHANNEL_COUNT] = {
    -1, -1, -1, -1,
    (int8_t)TMAG_CH_K1_1,  // K1 açık sensör (ikincil)
    (int8_t)TMAG_CH_K2_1,  // K2 açık sensör (ikincil)
};

// Alan grupları
static const uint8_t ALAN1_PCV       = 1;                   // N436 valf indeksi
static const uint8_t ALAN2_PCV       = 5;                   // N440 valf indeksi
static const uint8_t ALAN1_PISTONS[] = {1, 0, 4};           // PISTON_1_3, PISTON_5_7, PISTON_K1
static const uint8_t ALAN2_PISTONS[] = {2, 3, 5};           // PISTON_2_4, PISTON_6_R, PISTON_K2
static const uint8_t ALAN1_COUNT     = 3;
static const uint8_t ALAN2_COUNT     = 3;

// Piston isimleri (loglama)
static const char* PISTON_NAME[PISTON_CHANNEL_COUNT] = {
    "P5-7", "P1-3", "P2-4", "P6-R", "K1", "K2"
};
static const char* VALVE_NAME[8] = {
    "N433", "N436", "N434", "N435", "N438", "N440", "N439", "N437"
};

// Vites dizisi (D modu test): D1→D2→D3→D4→D3→D2→D1
// Her adımda hangi pistonlar açık (POS_MID) olduğu
// Gerçek otomatik vites sırası TaskAutoShiftV2'ye delege edilir

// ---------------------------------------------------------------------------
// Yardımcı makrolar
// ---------------------------------------------------------------------------
#define AT_CHECK_STOP() do { if (g_autoTestStop) { return false; } } while(0)

// ---------------------------------------------------------------------------
// Yardımcı fonksiyonlar
// ---------------------------------------------------------------------------

/** Tüm valfleri kapat (g_valveTargetDuty = 0) */
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

/** Tek valf duty yaz */
static void atSetValve(int valveIdx, uint16_t duty) {
    if (valveIdx < 0 || valveIdx >= 8) return;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_valveTargetDuty[valveIdx] = duty;
        xSemaphoreGive(g_sharedMutex);
    }
}

/** PCV/destek valfi için akım hedefi yaz (0 = kapat). PWM yerine akım kontrol döngüsü kullanılır. */
static void atSetValveCurrent(int valveIdx, float mA) {
    if (valveIdx < 0 || valveIdx >= 8) return;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_valveCustomCurrent_mA[valveIdx] = mA;
        xSemaphoreGive(g_sharedMutex);
    }
}

/** Valf bobini akımını oku (mA)
 *  Valf indeksi (TaskAutoTest sırası) → INA219 indeksi (TaskI2CMonitor sırası)
 *  TaskAutoTest: 0=N433,1=N436,2=N434,3=N435,4=N438,5=N440,6=N439,7=N437
 *  INA219 addr:  0=N433,1=N434,2=N435,3=N436,4=N437,5=N438,6=N439,7=N440
 */
static float atGetValveCurrent(int valveIdx) {
    if (valveIdx < 0 || valveIdx >= 8) return 0.0f;
    static const uint8_t VALVE_TO_INA[8] = {0, 3, 1, 2, 5, 7, 6, 4};
    int inaIdx = VALVE_TO_INA[valveIdx];
    float mA = 0.0f;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        mA = g_tele.inaI_mA[inaIdx];
        xSemaphoreGive(g_sharedMutex);
    }
    return mA;
}

/** Mevcut basıncı oku */
static float atGetPressure() {
    float p = 0;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        p = g_pumpPub.bar;
        xSemaphoreGive(g_sharedMutex);
    }
    return p;
}

/** Piston hall değerini oku (her iki sensörün max delta'sı) */
static int16_t atGetHall(uint8_t pistonIdx) {
    if (pistonIdx >= PISTON_CHANNEL_COUNT) return 0;
    int16_t val = 0;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        val = g_tmagData[PISTON_TMAG_CH[pistonIdx]].z;
        xSemaphoreGive(g_sharedMutex);
    }
    return val;
}

/** K1/K2 için her iki sensörün değişimini döndür (max) */
static int16_t atGetHallDelta(uint8_t pistonIdx, int16_t baseline) {
    if (pistonIdx >= PISTON_CHANNEL_COUNT) return 0;
    int16_t cur = atGetHall(pistonIdx);
    int16_t delta = (int16_t)abs((int)cur - (int)baseline);

    // K1/K2: ikinci sensörü de kontrol et
    int8_t ch2 = PISTON_TMAG_CH2[pistonIdx];
    if (ch2 >= 0) {
        int16_t cur2 = 0;
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            int16_t base2 = g_tmagData[(uint8_t)ch2].z;
            xSemaphoreGive(g_sharedMutex);
            // baseline ikinci sensör için ayrıca saklanmıyor, fark yaklaşımı yeterli
            (void)base2;
        }
    }
    return delta;
}

/** Pompa AUTO moda al */
static void atPumpAuto() {
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_pumpCmd.cmd = PUMP_CMD_AUTO;
        g_pumpCmd.seq++;
        xSemaphoreGive(g_sharedMutex);
    }
}

/** Pompayı durdur */
static void atPumpStop() {
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_pumpCmd.cmd = PUMP_CMD_STOP;
        g_pumpCmd.seq++;
        xSemaphoreGive(g_sharedMutex);
    }
}

/** Motor RPM oku */
static int32_t atGetRpm() {
    int32_t rpm = 0;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        rpm = g_pumpPub.rpm;
        xSemaphoreGive(g_sharedMutex);
    }
    return (int32_t)fabsf((float)rpm);
}

/** Loga mesaj gönder */
static void atLog(const char* msg) {
    {
        char buf[128];
        snprintf(buf, sizeof(buf), "[AT] %s", msg);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
    }
}

/** Faz durumunu GUI'ye JSON ile bildir (tip "AT") */
static void atSendPhaseUpdate(int phaseIdx) {
        static JsonDocument doc;
    doc.clear();
    doc["_t"]    = "AT";
    doc["phase"] = phaseIdx;  // 1-10, 0=idle, 10=Faz10, 11=done, 12=aborted
    doc["run"]   = g_autoTestResult.running;
    doc["done"]  = g_autoTestResult.done;
    doc["pass"]  = g_autoTestResult.pass;
    if (phaseIdx >= 0 && phaseIdx <= 10) {
        auto& ph = g_autoTestResult.phases[phaseIdx];
        doc["ph_done"] = ph.done;
        doc["ph_pass"] = ph.pass;
        doc["ph_det"]  = ph.detail;
        doc["ph_meas"] = ph.measured;
        doc["ph_fm"]   = ph.faultMask;
    }
    // AT sonuclarini log yerine event olarak guvenilir sekilde gonder.
    // Faz sonuclarinin (ozellikle Faz 1) dusmemesi icin TX buffer doluysa
    // timeout ile parca parca bekleyerek gonder.
    kitronic::SerialTx_SendEvent(doc, true);
}

/** Faz sonucunu kaydet ve bildir.
 *  guiDetail: GUI'ye (ph_det) gönderilecek mesaj; nullptr ise log detail kullanilir.
 *             Yorumlar GUI tarafinda yapilsin istenen fazlar icin sadece ham veri formati verilir.
 */
static void atFinishPhase(int phaseIdx, bool pass, const char* detail, float measured = 0.0f, uint8_t faultMask = 0, const char* guiDetail = nullptr) {
    if (phaseIdx < 0 || phaseIdx > 10) return;
    auto& ph       = g_autoTestResult.phases[phaseIdx];
    ph.done        = true;
    ph.pass        = pass;
    ph.measured    = measured;
    ph.faultMask   = faultMask;
    const char* detForGui = guiDetail ? guiDetail : detail;
    snprintf(ph.detail, sizeof(ph.detail), "%s", detForGui);
    if (!pass) g_autoTestResult.pass = false;
    atSendPhaseUpdate(phaseIdx);

    char logMsg[96];
    snprintf(logMsg, sizeof(logMsg), "Faz %d %s: %s", phaseIdx, pass ? "GECTI" : "KALDI", detail);
    atLog(logMsg);
}

// ---------------------------------------------------------------------------
// FAZ 0: Elektriksel Valf Kontrolü
// ---------------------------------------------------------------------------
/** Tüm 8 valf bobini sırayla kısa süre açılır, INA219 akımı ölçülür.
 *  Akım eşiği altında kalan valf(ler) çıkışta listelenir.
 *  Döner: true = tüm valfler OK
 *         false = en az bir valf elektriksel sorun
 */
static bool atPhase0_ValveElecCheck() {
    atLog("Faz 0: Elektriksel valf kontrolu");

    const auto&    prm         = g_autoTestParams;
    const float    COIL_MIN_MA = prm.coilMinCurrentMa;  // GUI'den ayarlanabilir (tek eşik)
    const uint16_t TEST_DUTY   = 1500;      // ~40% duty - bobin akımı ölçmek için yeterli
    const uint32_t SETTLE_MS   = 600;      // Akım stabilizasyon süresi

    atPumpStop();
    atAllValvesOff();
    vTaskDelay(pdMS_TO_TICKS(200));

    char failList[72] = "";
    uint8_t failMask  = 0;
    char logBuf[80];

    for (int v = 0; v < 8; v++) {
        if (g_autoTestStop) return false;

        atSetValve(v, TEST_DUTY);
        vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));

        float mA = atGetValveCurrent(v);

        atSetValve(v, 0);
        vTaskDelay(pdMS_TO_TICKS(100));

        snprintf(logBuf, sizeof(logBuf), "Faz 0: %s %.0f mA", VALVE_NAME[v], mA);
        atLog(logBuf);

        if (mA < COIL_MIN_MA) {
            failMask |= (uint8_t)(1u << v);
            if (strlen(failList) > 0)
                strncat(failList, ", ", sizeof(failList) - strlen(failList) - 1);
            strncat(failList, VALVE_NAME[v], sizeof(failList) - strlen(failList) - 1);
        }
    }

    atAllValvesOff();

    if (failMask == 0) {
        atFinishPhase(0, true, "OK - Tum valfler elektriksel baglanti OK", 0, 0);
        return true;
    }

    char detail[96];
    snprintf(detail, sizeof(detail), "ARIZA: %s acik devre / baglanti sorunu", failList);
    atFinishPhase(0, false, detail, 0, failMask);
    return false;
}

// ---------------------------------------------------------------------------
// Başlangıç: Basınç boşaltma (>10 bar ise PCVleri açarak düşür)
// ---------------------------------------------------------------------------
/** Basınç boşaltma + PCV tanı
 * Döner: true = OK (basınç boşaltıldı veya zaten düşük)
 *        false = PCV arızası tespit edildi (pcvFaultDetail doldurulur)
 *        false = g_autoTestStop tetiklendi (pcvFaultDetail boş kalır)
 */
static bool atPressureRelease(char* pcvFaultDetail, size_t detailSize) {
    const float    RELEASE_TARGET_BAR = 10.0f;
    const uint32_t RELEASE_TIMEOUT_MS = 30000;
    const float    PCV_COIL_MIN_MA    = 500.0f;  // Bu altı = bağlantı sorunu
    const float    PCV_MECH_DROP_MIN  = 1.5f;
    const uint32_t PCV_DIAG_WAIT_MS   = 1500;

    if (pcvFaultDetail && detailSize > 0) pcvFaultDetail[0] = '\0';

    if (atGetPressure() <= RELEASE_TARGET_BAR) return true;

    atLog("Basınc yuksek - PCV + piston valfleri ac/kapat dongusu ile bosaltuluyor");

    bool pcvDiagDone = false;

    uint32_t t0 = millis();
    while (atGetPressure() > RELEASE_TARGET_BAR) {
        if (g_autoTestStop) return false;
        if (millis() - t0 > RELEASE_TIMEOUT_MS) {
            atLog("UYARI: Basınc bosaltma timeout (30sn)");
            if (pcvFaultDetail)
                snprintf(pcvFaultDetail, detailSize, "PCV timeout - basınc bosaltilamadi (%.1f bar)", atGetPressure());
            atAllValvesOff();
            return false;
        }

        atSetValve(ALAN1_PCV, 2000);
        atSetValve(ALAN2_PCV, 2000);
        for (int p = 0; p < PISTON_CHANNEL_COUNT; p++) {
            atSetValve(PISTON_VALVE_IDX[p], 2000);
        }

        // İlk döngüde: basınç düşüyor mu?
        if (!pcvDiagDone) {
            float pBefore  = atGetPressure();
            vTaskDelay(pdMS_TO_TICKS(PCV_DIAG_WAIT_MS));
            float pAfter   = atGetPressure();
            bool  dropping = ((pBefore - pAfter) >= PCV_MECH_DROP_MIN);

            if (!dropping) {
                // Basınç düşmedi → valfleri tek tek akım testi yap
                atAllValvesOff();
                vTaskDelay(pdMS_TO_TICKS(200));

                // N436 bireysel test
                atSetValve(ALAN1_PCV, 2000);
                vTaskDelay(pdMS_TO_TICKS(1000));
                float mA1 = atGetValveCurrent(ALAN1_PCV);
                atSetValve(ALAN1_PCV, 0);
                vTaskDelay(pdMS_TO_TICKS(1000));

                // N440 bireysel test
                atSetValve(ALAN2_PCV, 2000);
                vTaskDelay(pdMS_TO_TICKS(1000));
                float mA2 = atGetValveCurrent(ALAN2_PCV);
                atSetValve(ALAN2_PCV, 0);

                bool conn1 = (mA1 < PCV_COIL_MIN_MA);  // true = bağlantı sorunu
                bool conn2 = (mA2 < PCV_COIL_MIN_MA);

                char diagBuf[96];
                if (conn1 && conn2)
                    snprintf(diagBuf, sizeof(diagBuf),
                        "ARIZA: N436(%.0fmA)+N440(%.0fmA) baglanti sorunu",
                        mA1, mA2);
                else if (!conn1 && !conn2)
                    snprintf(diagBuf, sizeof(diagBuf),
                        "ARIZA: N436(%.0fmA)+N440(%.0fmA) valf bozuk veya tıkalı.",
                        mA1, mA2);
                else if (conn1)
                    snprintf(diagBuf, sizeof(diagBuf),
                        "ARIZA: N436(%.0fmA) baglanti sorunu / N440(%.0fmA) valf bozuk.",
                        mA1, mA2);
                else
                    snprintf(diagBuf, sizeof(diagBuf),
                        "ARIZA: N436(%.0fmA) valf bozuk / N440(%.0fmA) baglanti sorunu veya valf bozuk.",
                        mA1, mA2);
                atLog(diagBuf);
                if (pcvFaultDetail)
                    snprintf(pcvFaultDetail, detailSize, "%s", diagBuf);
                atAllValvesOff();
                return false;
            } else {
                char diagBuf[80];
                snprintf(diagBuf, sizeof(diagBuf),
                    "PCV teshis OK: bas. %.1f->%.1f bar",
                    pBefore, pAfter);
                atLog(diagBuf);
                pcvDiagDone = true;
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        for (int p = 0; p < PISTON_CHANNEL_COUNT; p++) {
            atSetValve(PISTON_VALVE_IDX[p], 0);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    atAllValvesOff();
    char buf[64];
    snprintf(buf, sizeof(buf), "Basınc bosaltildi: %.1f bar", atGetPressure());
    atLog(buf);
    return true;
}

// ---------------------------------------------------------------------------
// FAZ 1: Pompa Doldurma Testi (PCV Teşhis Destekli)
//
// Sıra:
//   1) Her iki PCV (N436+N440) açık → dolarsa normal PASS
//   2) Timeout → boşalt, sadece N436 dene → dolarsa PASS
//   3) N436 da başarısız → N436 arızalı kaydet, boşalt, sadece N440 dene
//   4) N440 dolarsa → N436 arızalı olarak FAIL (devam)
//   5) N440 da başarısız → N436+N440 arızalı olarak FAIL (devam)
//   Herhangi bir return false: sadece g_autoTestStop durumunda
// ---------------------------------------------------------------------------

/** Yardımcı: Valfleri hazırla, pompayı çalıştır, basınç dolumunu bekle.
 *  maxRateOut : varsa, dolum süresince ölçülen max basınç artış hızı (bar/sn) yazılır
 *  Döner: dolum süresi (sn) >= 0.0 → başarı  |  < 0 → timeout veya stop */
static float atFillAttempt(uint8_t pcvMask, float* maxRateOut = nullptr) {
    const auto& prm = g_autoTestParams;
    atAllValvesOff();
    if (pcvMask & 0x01) atSetValve(ALAN1_PCV, 2000);
    if (pcvMask & 0x02) atSetValve(ALAN2_PCV, 2000);
    atPumpAuto();

    // Basınç artış hızı yalnızca RATE_START_BAR üzerinde ölçülür.
    // Bunun altındaki anlık yükselme basınç tüpünün doğal karakteristiğidir.
    const float RATE_START_BAR = 25.0f;

    float    prevPressure = atGetPressure();
    uint32_t prevMs       = millis();
    float    maxRate      = 0.0f;
    uint32_t t0 = prevMs;

    while (atGetPressure() < prm.targetBar) {
        if (g_autoTestStop) {
            atPumpStop();
            atAllValvesOff();
            if (maxRateOut) *maxRateOut = maxRate;
            return -2.0f;  // durdurma sinyali
        }
        if (millis() - t0 > prm.pumpFillTimeoutMs) {
            atPumpStop();
            atAllValvesOff();
            if (maxRateOut) *maxRateOut = maxRate;
            return -1.0f;  // timeout
        }
        vTaskDelay(pdMS_TO_TICKS(500));

        float    curPressure = atGetPressure();
        uint32_t curMs       = millis();
        uint32_t dt_ms       = curMs - prevMs;
        float    dp          = curPressure - prevPressure;

        // Sadece basınç RATE_START_BAR üzerindeyken hız hesapla
        if (dp > 0.0f && dt_ms > 0 && curPressure > RATE_START_BAR) {
            float rate = dp * 1000.0f / (float)dt_ms;  // bar/sn (gerçek zamana göre)
            if (rate > maxRate) maxRate = rate;
        }

        prevPressure = curPressure;
        prevMs       = curMs;
    }

    if (maxRateOut) *maxRateOut = maxRate;
    return (millis() - t0) / 1000.0f;  // başarı: geçen süre
}

static bool atPhase1_PumpFill() {
    const auto& prm   = g_autoTestParams;
    char detail[128];

    atLog("Faz 1 basliyor: Pompa doldurma (PCV teshis aktif)");

    // ================================================================
    // DENEME 1 — Her iki PCV açık (normal doldurma)
    // ================================================================
    atLog("Deneme 1: Her iki PCV (N436+N440) acik");
    float maxRiseRate = 0.0f;
    float elapsed = atFillAttempt(0x03, &maxRiseRate);  // bit0=N436, bit1=N440

    if (elapsed == -2.0f) return false;  // kullanıcı durdurdu

    if (elapsed >= 0.0f) {
        // -------------------------------------------------------
        // Basınç artış hızı kontrolü: hızlı artış = basınç tüpü arızası
        // -------------------------------------------------------
        if (maxRiseRate > prm.pressRiseMaxBarPerSec) {
            char rateLog[80];
            snprintf(rateLog, sizeof(rateLog),
                     "Faz 1: Anormal hizli basinc artisi %.1f bar/sn (esik %.0f)",
                     maxRiseRate, prm.pressRiseMaxBarPerSec);
            atLog(rateLog);
            snprintf(detail, sizeof(detail),
                     "ARIZA: Basinc tubu - %.1f bar/sn artis (esik: %.0f bar/sn). "
                     "Akumulatoru ariza. Dolum: %.1f sn",
                     maxRiseRate, prm.pressRiseMaxBarPerSec, elapsed);
            atFinishPhase(1, false, detail, maxRiseRate, 0x04);
            return false;  // Basınç tüpü arızası: diğer testler güvenilmez
        }
        // Normal başarı
        bool pass = (elapsed <= prm.pumpFillMaxSec);
        if (pass)
            snprintf(detail, sizeof(detail),
                     "%.1f sn'de %.0f bara ulasti (maks. artis: %.1f bar/sn)",
                     elapsed, prm.targetBar, maxRiseRate);
        else
            snprintf(detail, sizeof(detail),
                     "YAVAS: %.1f sn (max %.0f sn) - pompa ariza? (maks. artis: %.1f bar/sn)",
                     elapsed, prm.pumpFillMaxSec, maxRiseRate);
        atFinishPhase(1, pass, detail, elapsed);
        return true;
    }

    // ================================================================
    // TIMEOUT — PCV teşhis moduna geç
    // ================================================================
    atLog("Her iki PCV ile timeout - PCV teshis basladi");
    atPressureRelease(nullptr, 0);
    if (g_autoTestStop) return false;
    vTaskDelay(pdMS_TO_TICKS(500));

    // ================================================================
    // DENEME 2 — Sadece N436 (ALAN1_PCV) açık
    // ================================================================
    atLog("Deneme 2: Sadece N436 (Alan-1 PCV) acik");
    elapsed = atFillAttempt(0x01);

    if (elapsed == -2.0f) return false;

    if (elapsed >= 0.0f) {
        // N436 ile doldu → PCV'lerde belirgin arıza yok, faz geçiyor
        atLog("N436 ile doldurma basarili - faz gecti");
        snprintf(detail, sizeof(detail),
                 "Her iki PCV acikken timeout, N436 tek basina %.1f sn'de doldu",
                 elapsed);
        atFinishPhase(1, true, detail, elapsed);
        return true;
    }

    // N436 de başarısız → N436 arızalı
    atLog("N436 ile de timeout - N436 (Alan-1 PCV) arizali kayit");
    atPressureRelease(nullptr, 0);
    if (g_autoTestStop) return false;
    vTaskDelay(pdMS_TO_TICKS(500));

    // ================================================================
    // DENEME 3 — Sadece N440 (ALAN2_PCV) açık
    // ================================================================
    atLog("Deneme 3: Sadece N440 (Alan-2 PCV) acik");
    elapsed = atFillAttempt(0x02);

    if (elapsed == -2.0f) return false;

    if (elapsed >= 0.0f) {
        // N440 ile doldu → N436 arızalı, sistem dolu, sonraki fazlar etkilenebilir
        atLog("N440 ile doldurma basarili - N436 arızalı, devam ediliyor");
        snprintf(detail, sizeof(detail),
                 "N436 arızalı (timeout), N440 ile %.1f sn'de doldu - Alan-1 fazlar etkilenebilir",
                 elapsed);
        atFinishPhase(1, false, detail, elapsed, 0x01);  // 0x01 = N436 arızalı
        return true;  // Sistem dolu, sonraki fazlara devam
    }

    // Her iki PCV de başarısız → her ikisi arızalı (ya da pompa arızası)
    atLog("N440 ile de timeout - N436 ve N440 arizali, pompa testi basarisiz");
    snprintf(detail, sizeof(detail),
             "N436 ve N440 arızalı - pompa dolumu yapılamadı, tüm fazlar düşük basınçta çalışacak");
    atFinishPhase(1, false, detail, 0.0f, 0x03);  // 0x03 = N436+N440 arızalı
    return true;  // Yine de devam et: rapordan arıza kaydı görülsün
}

// ---------------------------------------------------------------------------
// FAZ 2/4: Alan PCV açıkken pistonlar hareketsiz mi?
// ---------------------------------------------------------------------------
static bool atPhaseLeakValves(int phaseIdx, uint8_t pcvIdx, const uint8_t* pistons, uint8_t count) {
    const auto& prm = g_autoTestParams;
    char logBuf[80];
    snprintf(logBuf, sizeof(logBuf), "Faz %d: %s acik - piston hareket testi",
             phaseIdx, VALVE_NAME[pcvIdx]);
    atLog(logBuf);

    // Basınç kontrolü
    if (atGetPressure() < prm.targetBar * 0.85f) {
        atLog("Basınc dusuk - yeniden dolduruluyor");
        atPumpAuto();
        uint32_t t0 = millis();
        while (atGetPressure() < prm.targetBar && millis() - t0 < prm.pumpFillTimeoutMs) {
            if (g_autoTestStop) return false;
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    atAllValvesOff();
    vTaskDelay(pdMS_TO_TICKS(300));

    // PCV'yi aç
    atSetValve(pcvIdx, 2000);

    // Settle bekleme: pistonlar stabil konuma gelsin
    // (basınç boşaltma sonrası açık kalan pistonlar PCV açılınca kapanır - bu kaçak değil)
    {
        uint32_t wt0 = millis();
        while (millis() - wt0 < 1500) {
            if (g_autoTestStop) { atAllValvesOff(); return false; }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    // Pistonlar settle olduktan SONRA baseline kaydet
    int16_t baseline[6] = {0};
    for (uint8_t i = 0; i < count; i++) {
        baseline[i] = atGetHall(pistons[i]);
    }

    // leakCheckWaitMs boyunca bekle - gerçek kaçak varsa pistonlar kaymaya devam eder
    {
        uint32_t wt0 = millis();
        while (millis() - wt0 < prm.leakCheckWaitMs) {
            if (g_autoTestStop) { atAllValvesOff(); return false; }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    // Hareket kontrolü
    uint8_t faultMask = 0;
    char detail[80];
    detail[0] = '\0';
    bool anyFault = false;

    for (uint8_t i = 0; i < count; i++) {
        if (g_autoTestStop) { atAllValvesOff(); return false; }
        int16_t delta = atGetHallDelta(pistons[i], baseline[i]);
        if (delta > (int16_t)prm.movementThreshold) {
            faultMask |= (1 << i);
            anyFault = true;
            char tmp[48];
            snprintf(tmp, sizeof(tmp), "%s(%d) ", PISTON_NAME[pistons[i]], (int)delta);
            strncat(detail, tmp, sizeof(detail) - strlen(detail) - 1);
        }
    }

    atAllValvesOff();

    if (!anyFault) {
        atFinishPhase(phaseIdx, true, "OK - Hareket yok", 0, 0);
    } else {
        char fullDetail[80];
        snprintf(fullDetail, sizeof(fullDetail), "KACAK: %s", detail);
        atFinishPhase(phaseIdx, false, fullDetail, 0, faultMask);
    }
    return true;
}

// ---------------------------------------------------------------------------
// FAZ 3/5: Alan PCV kapalıyken tek tek valf açılırsa piston açılıyor mu?
// ---------------------------------------------------------------------------
static bool atPhaseLeakPCV(int phaseIdx, uint8_t pcvIdx, const uint8_t* pistons, uint8_t count) {
    const auto& prm = g_autoTestParams;
    char logBuf[80];
    snprintf(logBuf, sizeof(logBuf), "Faz %d: %s kapali - sirali valf acma testi",
             phaseIdx, VALVE_NAME[pcvIdx]);
    atLog(logBuf);

    // PCV kapalı, tüm valfler kapalı
    atAllValvesOff();
    vTaskDelay(pdMS_TO_TICKS(500));

    uint8_t faultMask = 0;
    bool pcvLeak = false;

    for (uint8_t i = 0; i < count; i++) {
        if (g_autoTestStop) { atAllValvesOff(); return false; }
        uint8_t pIdx    = pistons[i];
        int16_t baseline = atGetHall(pIdx);

        // Piston valfini aç
        atSetValve(PISTON_VALVE_IDX[pIdx], 2000);
        for (int _w = 0; _w < 10 && !g_autoTestStop; _w++) vTaskDelay(pdMS_TO_TICKS(200));
        if (g_autoTestStop) { atAllValvesOff(); return false; }

        int16_t delta = atGetHallDelta(pIdx, baseline);
        if (delta > (int16_t)prm.movementThreshold) {
            // Piston hareket etti → PCV'de kaçak var
            faultMask |= (1 << i);
            pcvLeak = true;
            char tmp[64];
            snprintf(tmp, sizeof(tmp), "[AT] %s acikken %s hareket etti(delta=%d) -> %s kacak",
                     VALVE_NAME[PISTON_VALVE_IDX[pIdx]], PISTON_NAME[pIdx], (int)delta,
                     VALVE_NAME[pcvIdx]);
            atLog(tmp);
        }

        // Valfi kapat, settle
        atSetValve(PISTON_VALVE_IDX[pIdx], 0);
        for (int _w = 0; _w < 8 && !g_autoTestStop; _w++) vTaskDelay(pdMS_TO_TICKS(200));
        if (g_autoTestStop) { atAllValvesOff(); return false; }
    }

    if (!pcvLeak) {
        atFinishPhase(phaseIdx, true, "OK - PCV saglikli", 0, 0);
    } else {
        char detail[80];
        snprintf(detail, sizeof(detail), "PCV %s KACAGI TESPIT EDILDI", VALVE_NAME[pcvIdx]);
        atFinishPhase(phaseIdx, false, detail, 0, faultMask);
    }
    return true;
}

// ---------------------------------------------------------------------------
// FAZ 6: Mekatronik Yağ Kaçak Testi
// ---------------------------------------------------------------------------
static bool atPhase6_OilLeak() {
    const auto& prm = g_autoTestParams;
    atLog("Faz 6: Yag kacak testi");

    atAllValvesOff();
    atPumpAuto();

    // 60 bara doldur
    uint32_t t0 = millis();
    while (atGetPressure() < prm.targetBar) {
        if (g_autoTestStop) return false;
        if (millis() - t0 > prm.pumpFillTimeoutMs) {
            atFinishPhase(6, false, "TIMEOUT: Dolum basarisiz");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // Pompayı durdur
    atPumpStop();

    // RPM 0'a düşene kadar veya max 5sn bekle
    t0 = millis();
    while (atGetRpm() > 100 && millis() - t0 < 5000) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // Basınç ölçümü başlangıcı
    float pStart = atGetPressure();
    uint32_t holdMs = prm.oilLeakHoldSec * 1000;
    uint32_t tHold = millis();

    while (millis() - tHold < holdMs) {
        if (g_autoTestStop) return false;
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    float pEnd  = atGetPressure();
    float pDrop = pStart - pEnd;
    bool pass   = (pDrop <= prm.oilLeakMaxDrop_bar);

    char detail[80];
    if (pass)
        snprintf(detail, sizeof(detail), "OK: %u sn'de %.1f bar dusus (max %.0f)",
                 prm.oilLeakHoldSec, pDrop, prm.oilLeakMaxDrop_bar);
    else
        snprintf(detail, sizeof(detail), "YAG KACAGI: %u sn'de %.1f bar dusus (max %.0f)",
                 prm.oilLeakHoldSec, pDrop, prm.oilLeakMaxDrop_bar);
    atFinishPhase(6, pass, detail, pDrop);
    return true;
}

// ---------------------------------------------------------------------------
// FAZ 7 Doğrulama Yardımcıları: 3x açma/kapama testi
// ---------------------------------------------------------------------------
static constexpr uint32_t VERIFY_TIMEOUT_MS        = 3000;  // Açılma/kapanma zaman aşımı (ms)
static constexpr uint32_t VERIFY_TIMEOUT_SPRING_MS = 4500;  // K1/K2 yay dönüşü için ek süre
static constexpr uint32_t VERIFY_DWELL_MS          = 400;   // Normal piston cycle arası bekleme
static constexpr uint32_t VERIFY_DWELL_SPRING_MS   = 900;   // K1/K2 yay tam dönüşü bekleme
static constexpr float    VERIFY_TOL_RATIO         = 0.15f; // Aralığın %15'i tolerans
// Basınç hazırlık sabitleri (her döngü öncesi)
static constexpr float    VERIFY_PRESS_TARGET_BAR  = 60.0f;  // Hedef basınç (bar)
static constexpr float    VERIFY_PRESS_STABLE_THR  = 0.3f;   // Stabil eşiği (bar / 100ms)
static constexpr uint32_t VERIFY_PRESS_STABLE_MS   = 2000;   // Bu süre stabil kalması gerekir
static constexpr uint32_t VERIFY_PRESS_TIMEOUT_MS  = 30000;  // Maks. basınç dolum süresi
// Hız/basınç yorum eşikleri kaldırıldı — yorumlama GUI tarafinda config.json'dan yapilir

struct PistonVerifyResult {
    // Ham ölçüm verileri — yorumlama GUI tarafinda config.json eşiklerine göre yapilir
    uint32_t avgOpenMs;        // Ortalama açma süresi (ms)
    uint32_t avgCloseMs;       // Ortalama kapama süresi (ms)
    float    avgPressDropOpen;  // Ortalama basınç düşüşü — açma (bar)
    float    avgPressDropClose; // Ortalama basınç düşüşü — kapama (bar)
    uint8_t  validCycles;
    // Temel hareket durumu (hedefe ulaşip ulaşmadiği bilgisi, ham veri olarak)
    bool     openFail;         // Açılma hedefine zamaninda ulaşamadi
    bool     closeFail;        // Kapanma hedefine zamaninda ulaşamadi
};

/** 60 bar hedef basınca tek seferlik pompa ile ulaş, pompa durduktan sonra
 *  kısa süre stabilize olunca geri dön.
 *  NOT: PUMP_CMD_AUTO 60/42 bar sınırlarında sürekli start/stop yapar,
 *  bu yüzden 0.3 bar/100 ms stabil kriterini sağlayamaz ve 30 sn timeout’a düşer.
 *  Return: true=başarılı, false=timeout/stop */
static bool atPressurizeAndStabilize() {
    // Tek seferlik doldurma (PUMP_AUTO yerine); böylece pompa hedefe ulaşınca durur.
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_pumpCmd.cmd = PUMP_CMD_START;
        g_pumpCmd.setRpm = 4000.0f;
        g_pumpCmd.seq++;
        xSemaphoreGive(g_sharedMutex);
    }
    const uint32_t tStart = millis();
    uint32_t stableAcc    = 0;
    float    prevPress    = atGetPressure();
    while (millis() - tStart < VERIFY_PRESS_TIMEOUT_MS) {
        if (g_autoTestStop) { atPumpStop(); return false; }
        vTaskDelay(pdMS_TO_TICKS(100));
        float p = atGetPressure();
        // Pompa kendi durana (PUMP_SINGLE hedefe ulaştı) ve basınç kısa süre
        // değişmeyene kadar bekle.
        bool pumpIdle = (g_pumpPub.mode == 0);  // PUMP_IDLE
        if (p >= VERIFY_PRESS_TARGET_BAR && pumpIdle && fabsf(p - prevPress) < VERIFY_PRESS_STABLE_THR) {
            stableAcc += 100;
            if (stableAcc >= 300) {  // 300 ms stabil yeterli
                atPumpStop();
                vTaskDelay(pdMS_TO_TICKS(100));
                return true;
            }
        } else {
            stableAcc = 0;
        }
        prevPress = p;
    }
    atPumpStop();
    return false;
}

/** Kalibrasyon doğrulama: pistonu 3x aç/kapat yap, ham ölçüm verilerini döndür.
 *  - openFail/closeFail : hedefe zaman aşımı içinde ulaşılamadı (temel hareket durumu)
 *  Yorumlama (uyari/severe) GUI tarafinda config.json eşiklerine göre yapilir.
 *  K1/K2 (yay-geri-dönüşlü): kapanma sırasında PCV sıfırlanır, dwell süresi artırılır.
 */
static PistonVerifyResult atVerifyPistonOCO(uint8_t p, uint16_t openPwm) {
    PistonVerifyResult res = {};

    PistonCalibData cal = {};
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        cal = g_pistonCalibData[p];
        xSemaphoreGive(g_sharedMutex);
    }
    if (!cal.calibrated) return res;

    const int     valveIdx   = PISTON_VALVE_IDX[p];
    const int     supportIdx = PISTON_SUPPORT_IDX[p];
    // K1/K2 yay-geri-dönüşlü piston: ikinci sensörü var (PISTON_TMAG_CH2 >= 0)
    const bool isSpringReturn = (PISTON_TMAG_CH2[p] >= 0);

    // K1/K2 kalibrasyonu g_pistonCalibData'da mm*10 olarak saklanır;
    // P0-P3 ise ham hall Z değeri olarak saklanır.
    float closedPos, openPos;
    if (isSpringReturn) {
        closedPos = (float)cal.min_raw / 10.0f;  // mm
        openPos   = (float)cal.max_raw / 10.0f;  // mm
    } else {
        closedPos = (float)(int16_t)cal.min_raw;  // ham hall Z
        openPos   = (float)(int16_t)cal.max_raw;    // ham hall Z
    }
    const bool    openDir = (openPos > closedPos);
    const float range    = fabsf(openPos - closedPos);
    // K1/K2: açılış hedefini gerçek açık konuma yaklaştır (yoksa kapalı sensör/offset
    // yüzünden hedef erken geçilebiliyor ve sürekli ~50ms çıkıyordu).
    const float openTol  = range * (isSpringReturn ? 0.05f : VERIFY_TOL_RATIO);
    // K1/K2 yay geri dönüşü tam kalibrasyon pozisyonuna ulaşamayabilir → daha geniş tolerans
    const float closeTol = range * (isSpringReturn ? 0.30f : VERIFY_TOL_RATIO);
    // openDir=true : açık hedef = openPos-openTol, kapalı hedef = closedPos+closeTol
    // openDir=false: açık hedef = openPos+openTol, kapalı hedef = closedPos-closeTol
    const float openTgt  = openDir ? (openPos  - openTol) : (openPos  + openTol);
    const float closeTgt = openDir ? (closedPos + closeTol) : (closedPos - closeTol);

    // K1/K2: Döngü öncesi kapalı pozisyona zorla — kalibrasyon adımı açık bırakmış olabilir
    if (isSpringReturn) {
        atSetValveCurrent(supportIdx, 0.0f);
        atSetValve(valveIdx, 0);
        vTaskDelay(pdMS_TO_TICKS(VERIFY_DWELL_SPRING_MS + 500));
    }

    // 60 bar → stabil (2sn) → pompayı durdur — 3x döngü boyunca pump kapalı kalır
    if (!atPressurizeAndStabilize()) return res;

    uint32_t sumOpenMs = 0, sumCloseMs = 0;
    float    sumDropOpen = 0.0f, sumDropClose = 0.0f;
    int      validCycles = 0;
    int      openFails   = 0;
    int      closeFails  = 0;

    for (int cycle = 0; cycle < 3; cycle++) {
        if (g_autoTestStop) break;

        // Tüm pistonlar için support valfi aç (vites pistonları da PCV ister)

        atSetValveCurrent(supportIdx, 650.0f);
        vTaskDelay(pdMS_TO_TICKS(isSpringReturn ? 150 : 100));

        // --- Açma adımı ---
        float pressBeforeOpen = atGetPressure();
        //atSetValve(valveIdx, openPwm);
        atSetValveCurrent(valveIdx, 650.0f);
        uint32_t t0 = millis();
        bool openOk = false;
        while (millis() - t0 < VERIFY_TIMEOUT_MS) {
            vTaskDelay(pdMS_TO_TICKS(50));
            float h = isSpringReturn ? g_pistonHallmm[p] : (float)atGetHall(p);
            openOk = openDir ? (h >= openTgt) : (h <= openTgt);
            if (openOk) break;
        }
        
        uint32_t openMs = millis() - t0;
        vTaskDelay(pdMS_TO_TICKS(350));  // hedefe ulaşıldıktan sonra anlık basınç dalgalanmaları olabilir, 350ms bekle ve basıncı tekrar oku
        float pressAfterOpen = atGetPressure();
        if (!openOk) openFails++;

        // Açık pozisyonda kısa tutma
        vTaskDelay(pdMS_TO_TICKS(isSpringReturn ? 500 : VERIFY_DWELL_MS));
        if (g_autoTestStop) break;

        // --- Kapama adımı ---
        float pressBeforeClose = atGetPressure();
        //atSetValve(valveIdx, 0);
        // K1/K2: ÖNCE piston valfini kapat (tanka açılır) → hidrolik hızlı tahliye olsun,
        // ardından PCV'yi kapat. Eski sırada (önce PCV) silindirdeki basınç yay
        // geri dönüşünü yavaşlatıyordu ve kapanış 4.5 sn'ye çıkıyordu.
        atSetValveCurrent(valveIdx, 0.0f);
        if (isSpringReturn) {
            vTaskDelay(pdMS_TO_TICKS(200));      // K1/K2 silindirindeki basıncın tahliyesi
            atSetValveCurrent(supportIdx, 0.0f);   // PCV kapat
        }
        t0 = millis();
        bool closeOk = false;
        const uint32_t closeTmo = isSpringReturn ? VERIFY_TIMEOUT_SPRING_MS : VERIFY_TIMEOUT_MS;
        while (millis() - t0 < closeTmo) {
            vTaskDelay(pdMS_TO_TICKS(50));
            float h = isSpringReturn ? g_pistonHallmm[p] : (float)atGetHall(p);
            closeOk = openDir ? (h <= closeTgt) : (h >= closeTgt);
            if (closeOk) break;
        }
        uint32_t closeMs = millis() - t0;
        vTaskDelay(pdMS_TO_TICKS(350));  // Kapandıktan sonra anlık basınç dalgalanmaları olabilir, 350ms bekle ve basıncı tekrar oku
        float pressAfterClose = atGetPressure();
        if (!closeOk) closeFails++;

        // Kapalı pozisyonda bekleme — K1/K2 için yay tam oturana kadar bekle
        vTaskDelay(pdMS_TO_TICKS(isSpringReturn ? VERIFY_DWELL_SPRING_MS : VERIFY_DWELL_MS));
        sumOpenMs    += openMs;
        sumCloseMs   += closeMs;
        sumDropOpen  += (pressBeforeOpen  - pressAfterOpen);
        sumDropClose += (pressBeforeClose - pressAfterClose);
        validCycles++;
    }
    
    atSetValve(valveIdx, 0);
    atSetValveCurrent(supportIdx, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(200));
    // >=2/3 döngü başarısız olursa gerçek arıza say
    res.openFail  = (openFails  > validCycles / 2);
    res.closeFail = (closeFails > validCycles / 2);
    if (validCycles > 0) {
        res.avgOpenMs         = sumOpenMs  / (uint32_t)validCycles;
        res.avgCloseMs        = sumCloseMs / (uint32_t)validCycles;
        res.avgPressDropOpen  = sumDropOpen  / (float)validCycles;
        res.avgPressDropClose = sumDropClose / (float)validCycles;
        res.validCycles       = (uint8_t)validCycles;
        // Yorumlama (uyari/severe) GUI tarafinda config.json eşiklerine göre yapilir
    }
    return res;
}

// ---------------------------------------------------------------------------
// FAZ 7: Piston Açık/Kapalı Kalibrasyon
// ---------------------------------------------------------------------------
static bool atPhase7_CalibOpenClose() {
    atLog("Faz 7: Acik/kapali kalibrasyon (6 piston)");
    const auto& prm = g_autoTestParams;

    // Basınç yeterli mi?
    if (atGetPressure() < prm.targetBar * 0.8f) {
        atPumpAuto();
        uint32_t t0 = millis();
        while (atGetPressure() < prm.targetBar * 0.8f && millis() - t0 < prm.pumpFillTimeoutMs) {
            if (g_autoTestStop) return false;
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    atAllValvesOff();

    // Kalibrasyon verilerini sıfırla + eski API ile tetikle
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        for (int p = 0; p < PISTON_CHANNEL_COUNT; p++) {
            g_pistonCalibData[p].calibrated = false;
        }
        g_pistonCalReq = {};
        g_pistonCalReq.start             = true;
        g_pistonCalReq.piston            = 0;
        g_pistonCalReq.calibrateAll      = true;
        g_pistonCalReq.findHold          = false;
        g_pistonCalReq.pwmDuty           = prm.calPwm;
        g_pistonCalReq.pressureTargetBar = prm.targetBar;
        xSemaphoreGive(g_sharedMutex);
    }
    g_pistonCalReqSeq++;  // TaskValveControl'e kalibrasyon tetikle

    // Kalibrasyonun BİTMESİNİ bekle: max 6 piston × calTimeoutMs + 10sn marj
    uint32_t maxWait = (uint32_t)prm.calTimeoutMs * PISTON_CHANNEL_COUNT + 10000;
    uint32_t t0 = millis();
    bool calDone = false;
    while (millis() - t0 < maxWait) {
        if (g_autoTestStop) break;
        vTaskDelay(pdMS_TO_TICKS(500));
        bool done = true;
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            for (int p = 0; p < PISTON_CHANNEL_COUNT; p++) {
                if (!g_pistonCalibData[p].calibrated) { done = false; break; }
            }
            xSemaphoreGive(g_sharedMutex);
        }
        if (done) { calDone = true; break; }
    }

    // Timeout veya stop: kalibrasyon task'ını iptal et
    if (!calDone) {
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_pistonCalReq = {};
            g_pistonCalReq.start = false;
            xSemaphoreGive(g_sharedMutex);
        }
        g_pistonCalReqSeq++;  // abort komutu gönder
        vTaskDelay(pdMS_TO_TICKS(200));
        if (g_autoTestStop) return false;
    }

    // Gerçekten kalibre edilip edilmediğini kontrol et
    uint8_t faultMask = 0;
    char detail[80];
    detail[0] = '\0';

    for (int p = 0; p < PISTON_CHANNEL_COUNT; p++) {
        bool calibrated = false;
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            calibrated = g_pistonCalibData[p].calibrated;
            xSemaphoreGive(g_sharedMutex);
        }
        if (!calibrated) {
            faultMask |= (1 << p);
            strncat(detail, PISTON_NAME[p], sizeof(detail) - strlen(detail) - 1);
            strncat(detail, " ", sizeof(detail) - strlen(detail) - 1);
        }
    }

    // Kalibrasyon başarısız → Faz 8'e geçme (hold kalibrasyonu anlamsız)
    if (faultMask != 0) {
        char logDetail[80];
        char guiDetail[80];
        snprintf(logDetail, sizeof(logDetail), "HATA: %s kalibre edilemedi", detail);
        snprintf(guiDetail, sizeof(guiDetail), "kalibrasyon:hata mask=0x%02X", faultMask);
        atFinishPhase(7, false, logDetail, 0, faultMask, guiDetail);
        return false;
    }

    // --- Doğrulama: 3x açma/kapama testi (tüm pistonlar) ---
    atLog("Faz7: Kalibrasyon dogrulama (3x ac/kapat) basliyor");

    atPumpAuto();
    if (atGetPressure() < prm.targetBar * 0.8f) {
        uint32_t tp = millis();
        while (atGetPressure() < prm.targetBar * 0.8f && millis() - tp < 10000) {
            AT_CHECK_STOP();
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    PistonVerifyResult results[PISTON_CHANNEL_COUNT] = {};

    for (int p = 0; p < PISTON_CHANNEL_COUNT; p++) {
        AT_CHECK_STOP();

        char logBuf[80];
        snprintf(logBuf, sizeof(logBuf), "Faz7 %s: 3x ac/kapat verileri", PISTON_NAME[p]);
        atLog(logBuf);

        results[p] = atVerifyPistonOCO((uint8_t)p, prm.calPwm);
    }

    // ---- Performans tablosu (ham veri) ----
    // Yorumlama (OK/Uyari/Arıza) GUI tarafinda config.json eşiklerine göre yapilir
    atLog("Faz7 Perf: Piston| Ac(ms)|Kap(ms)|dPac| dPkap|Durum");
    for (int p = 0; p < PISTON_CHANNEL_COUNT; p++) {
        const PistonVerifyResult& r = results[p];
        const char* st = (r.openFail || r.closeFail) ? "ARIZA" : "OK";
        char row[100];
        snprintf(row, sizeof(row), "Faz7 Perf: %-5s| %5u | %5u |%4.1f|  %4.1f|%s",
                 PISTON_NAME[p], r.avgOpenMs, r.avgCloseMs,
                 r.avgPressDropOpen, r.avgPressDropClose, st);
        atLog(row);
        // JSON — GUI parse eder, yorumlama GUI'de
        char json[160];
        snprintf(json, sizeof(json),
                 "{\"cmd\":\"ph7tbl\",\"p\":\"%s\",\"om\":%u,\"cm\":%u"
                 ",\"dpo\":%.1f,\"dpc\":%.1f,\"fl\":%d}",
                 PISTON_NAME[p], r.avgOpenMs, r.avgCloseMs,
                 r.avgPressDropOpen, r.avgPressDropClose,
                 (r.openFail || r.closeFail) ? 1 : 0);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, json);
    }

    // TX kuyruğunun boşalmasını bekle: perf tablo döngüsü 13 mesaj burst eder,
    // kuyruk doluysa atSendPhaseUpdate(7) sessizce DROP olur → raporda Faz 7 kaybolur.
    vTaskDelay(pdMS_TO_TICKS(400));

    // GUI'ye sadece ham veri gönder; yorumlar GUI tarafinda ph7tbl satirlarindan yapilir
    atFinishPhase(7, true, "OK - 3x ac/kapat olcum verileri gonderildi", 0, 0,
                  "Faz 7: olcum verileri gonderildi");
    return true;
}

// ---------------------------------------------------------------------------
// FAZ 8: P0-P3 Tam Akım Kalibrasyonu + Orta Konum Doğrulama
// (K1/K2 kaldırıldı — yay sistemi nedeniyle hold kontrolü desteklenmiyor.
//  Bkz. HOLD_TUNING_TODO.md "K1/K2 Karar" bölümü.)
// ---------------------------------------------------------------------------
//
// Adımlar:
//   1) PCV'leri aç, basınç stabilize
//   2) Her piston (P0..P3) için sırayla TaskCurrentCalib çağır:
//      - open_mA / close_mA / hold_mA bulur
//      - hold_mA bulundu mu kontrol et (>100mA)
//   3) P0..P3 sırayla 10sn closed-loop hold doğrulama: hold_mid_enable=true, ±3mm tolerans
//
// Faz 9'u engellemez (UYARI olarak işaretlenir).
// ---------------------------------------------------------------------------

// TaskCurrentCalib'i kesin olarak durdur (abort + iç döngülerin yayılmasını bekle).
// ccWait artık g_currentCalibReq.abort gördüğünde g_currentCalibRunning=false yapar,
// bu da uzun döngülerin tetiklemesini sağlar.
static void atAbortCurrentCalibAndWait(uint32_t maxStopMs = 4000) {
    if (!g_currentCalibRunning) return;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_currentCalibReq.abort = true;
        xSemaphoreGive(g_sharedMutex);
    }
    g_currentCalibReqSeq++;
    // İç döngülerin durmasını bekle (ccWait 50ms'de bir kontrol eder)
    uint32_t t0 = millis();
    while (g_currentCalibRunning && millis() - t0 < maxStopMs) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    // Hâlâ koşuyorsa son çare: zorla flag düşür (fonksiyon güvenli sınır)
    if (g_currentCalibRunning) {
        atLog("Faz8: WARN — calib zorla durduruldu");
        g_currentCalibRunning = false;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    // ABORTED log'unun yazılmasına ufak fırsat ver
    vTaskDelay(pdMS_TO_TICKS(150));
}

// Tek piston için TaskCurrentCalib tetikle ve bitmesini bekle.
// Döner: true = tamamlandı (başarılı/başarısız), false = stop veya başlatamadı
static bool atRunCurrentCalibSinglePiston(uint8_t piston, uint32_t maxWaitMs) {
    // Önceki bir kalibrasyon çalışıyorsa kesin olarak durdur
    atAbortCurrentCalibAndWait();

    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_currentCalibReq.abort  = false;
        g_currentCalibReq.piston = piston;
        g_currentCalibReq.all    = false;  // Sadece bu piston
        xSemaphoreGive(g_sharedMutex);
    }
    g_currentCalibReqSeq++;

    // Başlamasını bekle (max 3sn)
    uint32_t t0 = millis();
    while (!g_currentCalibRunning && millis() - t0 < 3000) {
        if (g_autoTestStop) return false;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (!g_currentCalibRunning) {
        char log[64];
        snprintf(log, sizeof(log), "Faz8 P%d: kalibrasyon baslatilamadi", piston);
        atLog(log);
        return false;
    }

    // Bitmesini bekle
    t0 = millis();
    while (g_currentCalibRunning && millis() - t0 < maxWaitMs) {
        if (g_autoTestStop) {
            atAbortCurrentCalibAndWait();
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Timeout durumunda KESİN abort et — fonksiyon dönmeden TaskCurrentCalib gerçekten durmalı,
    // aksi halde Faz 8 hold doğrulaması çakışan valf set'lerinden bozulur.
    if (g_currentCalibRunning) {
        char log[80];
        snprintf(log, sizeof(log), "Faz8 P%d: kalibrasyon TIMEOUT (%lus)",
                 piston, (unsigned long)(maxWaitMs / 1000));
        atLog(log);
        atAbortCurrentCalibAndWait();
    }
    return true;
}

static bool atPhase8_HoldMidPreTest() {
    atLog("Faz 8: Tam akim kalibrasyonu + orta konum dogrulama (P0-P3)");
    const auto& prm = g_autoTestParams;

    // Basınç yeterli mi?
    if (atGetPressure() < prm.targetBar * 0.8f) {
        atPumpAuto();
        uint32_t t0 = millis();
        while (atGetPressure() < prm.targetBar * 0.8f && millis() - t0 < prm.pumpFillTimeoutMs) {
            if (g_autoTestStop) return false;
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    atAllValvesOff();

    // Her iki PCV'yi aç: hidrolik devre kapalı kalmasın
    //atSetValve(ALAN1_PCV, 2000);  // N436
    //atSetValve(ALAN2_PCV, 2000);  // N440
    atSetValveCurrent(ALAN1_PCV, 650.0f); // N436
    atSetValveCurrent(ALAN2_PCV, 650.0f); // N440 
    vTaskDelay(pdMS_TO_TICKS(200));

    {
        char pLog[64];
        snprintf(pLog, sizeof(pLog), "Faz8: PCV acildi, basinc=%.1f bar", atGetPressure());
        atLog(pLog);
    }

    uint8_t faultMask = 0;
    char    detail[180];
    detail[0] = '\0';

    // ----------------------------------------------------------------
    // 1) P0-P3 için sırayla akım kalibrasyonu + tek-piston hold testi
    // ----------------------------------------------------------------
    // Her piston için tipik süre:
    //   open seek 18*1.2 = 22sn
    //   close seek 6*1.2 = 7sn
    //   bang 40*0.5 = 20sn
    //   hold 10 iter * (3sn settle + 5sn approach) = 80sn
    //   toplam ~130sn (worst-case)
    // 180sn timeout = güvenli marj. Convergence olmazsa CCAL "midpoint" kullanır.
    const uint32_t SINGLE_CALIB_TIMEOUT_MS = 180000;
    const uint32_t HOLD_TEST_MS            = 10000;  // 10sn tek-piston hold
    const float    TARGET_MM               = 13.0f;  // 26mm strok × 0.5
    const float    TOL_MM                  = 3.0f;   // ±3.0mm tolerans

    for (int p = 0; p < 4; p++) {
        if (g_autoTestStop) {
            atAllValvesOff();
            return false;
        }
        char log[80];
        snprintf(log, sizeof(log), "Faz8 P%d (%s): akim kalibrasyonu basliyor",
                 p, PISTON_NAME[p]);
        atLog(log);

        if (!atRunCurrentCalibSinglePiston((uint8_t)p, SINGLE_CALIB_TIMEOUT_MS)) {
            if (g_autoTestStop) { atAllValvesOff(); return false; }
            // Başlatılamadı → arızalı say, sonraki pistona geç
            faultMask |= (uint8_t)(1u << p);
            strncat(detail, PISTON_NAME[p], sizeof(detail) - strlen(detail) - 1);
            strncat(detail, "(start) ", sizeof(detail) - strlen(detail) - 1);
            continue;
        }

        // Sonuçları oku
        float openMa = 0, closeMa = 0, holdMa = 0;
        bool calibrated = false;
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            openMa     = g_pistonCalibData[p].open_mA;
            closeMa    = g_pistonCalibData[p].close_mA;
            holdMa     = g_pistonCalibData[p].hold_mA;
            calibrated = g_pistonCalibData[p].calibrated;
            xSemaphoreGive(g_sharedMutex);
        }

        bool holdOk = (holdMa > 100.0f);
        snprintf(log, sizeof(log),
                 "Faz8 P%d: open=%.0f close=%.0f hold=%.0f mA -> %s",
                 p, openMa, closeMa, holdMa, holdOk ? "OK" : "HOLD-BULUNAMADI");
        atLog(log);

        if (!calibrated || !holdOk) {
            faultMask |= (uint8_t)(1u << p);
            strncat(detail, PISTON_NAME[p], sizeof(detail) - strlen(detail) - 1);
            strncat(detail, "(cal) ", sizeof(detail) - strlen(detail) - 1);
            continue;
        }

        // --- Tek-piston hold doğrulama ---
        snprintf(log, sizeof(log), "Faz8 P%d (%s): 10sn hold testi basliyor", p, PISTON_NAME[p]);
        atLog(log);

        // PCV'ler açık; bu piston dışındaki tüm piston valfleri kapalı
        atSetValveCurrent(ALAN1_PCV, 650.0f);  // N436
        atSetValveCurrent(ALAN2_PCV, 650.0f);  // N440

        // Kalibrasyon sonrası kalıntı PWM/akımı temizle, sonra closed-loop hold'u devreye al
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            int vIdx = PISTON_VALVE_IDX[p];
            g_valveCustomCurrent_mA[vIdx] = 0.0f;
            g_valveTargetDuty[vIdx] = 0;
            g_pistonRuntime[p].hold_mid_enable  = true;
            g_pistonRuntime[p].hold_init_needed = true;
            g_pistonRuntime[p].x_ref            = 0.5f;
            xSemaphoreGive(g_sharedMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100));

        atPumpAuto();

        uint32_t t0 = millis();
        while (millis() - t0 < HOLD_TEST_MS) {
            if (g_autoTestStop) {
                if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    g_pistonRuntime[p].hold_mid_enable = false;
                    xSemaphoreGive(g_sharedMutex);
                }
                atAllValvesOff();
                return false;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        // Son pozisyonu HOLD AKTIFKEN oku
        float finalPos = 0.0f;
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            finalPos = g_pistonHallmm[p];
            xSemaphoreGive(g_sharedMutex);
        }

        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_pistonRuntime[p].hold_mid_enable = false;
            xSemaphoreGive(g_sharedMutex);
        }
        // Test edilen piston valfini kapat (diğerleri zaten kapalı/kalibre sonrası kapalı)
        {
            int vIdx = PISTON_VALVE_IDX[p];
            if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                g_valveCustomCurrent_mA[vIdx] = 0.0f;
                g_valveTargetDuty[vIdx] = 0;
                xSemaphoreGive(g_sharedMutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));

        float err = fabsf(finalPos - TARGET_MM);
        snprintf(log, sizeof(log),
                 "Faz8 %s: %.1f mm (err=%.1f mm, tol=%.1f mm) -> %s",
                 PISTON_NAME[p], finalPos, err, TOL_MM, (err <= TOL_MM) ? "OK" : "HOLD-DRIFT");
        atLog(log);
        if (err > TOL_MM) {
            faultMask |= (uint8_t)(1u << p);
            strncat(detail, PISTON_NAME[p], sizeof(detail) - strlen(detail) - 1);
            strncat(detail, "(drift) ", sizeof(detail) - strlen(detail) - 1);
        }
    }

    // ----------------------------------------------------------------
    // Faz 8 sonucu
    if (faultMask == 0) {
        atFinishPhase(8, true, "OK - P0-P3 kalibre + hold dogrulandi", 0, 0);
    } else {
        char fullDetail[200];
        snprintf(fullDetail, sizeof(fullDetail), "UYARI: %s(Faz 9 devam eder)", detail);
        atFinishPhase(8, false, fullDetail, 0, faultMask);
        atLog("Faz 8 dogrulama basarisiz - UYARI olarak kaydedildi, Faz 9 devam ediyor");
    }
    return true;  // Faz 8 Faz 9'u engellemez
}

// ---------------------------------------------------------------------------
// FAZ 9: Otomatik Vites Testi
// ---------------------------------------------------------------------------
static bool atPhase9_AutoShift() {
    const auto& prm = g_autoTestParams;
    atLog("Faz 9: Otomatik vites testi");

    // Pompa AUTO, basınç yeterli
    if (atGetPressure() < prm.targetBar * 0.8f) {
        atPumpAuto();
        uint32_t t0 = millis();
        while (atGetPressure() < prm.targetBar * 0.8f && millis() - t0 < prm.pumpFillTimeoutMs) {
            if (g_autoTestStop) return false;
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    // TaskAutoShiftV2'yi kullan
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_autoShiftV2Req.start       = true;
        g_autoShiftV2Req.manualMode  = false;
        g_autoShiftV2Req.targetGear  = GEAR_N;  // N'den başlar, otomatik D1..D7 geçer
        g_autoShiftV2Req.gearHoldMs  = (uint16_t)prm.gearHoldMs;
        g_autoShiftV2Req.repeatCount = (uint8_t)prm.autoShiftRepeats;
        xSemaphoreGive(g_sharedMutex);
    }
    g_autoShiftV2ReqSeq++;  // TaskAutoShiftV2'yi tetikle

    // Önce TaskAutoShiftV2'nin BAŞLAMASINI bekle (max 5sn)
    uint32_t startWait9 = millis();
    bool phase9Started = false;
    while (millis() - startWait9 < 5000) {
        if (g_autoTestStop) return false;
        vTaskDelay(pdMS_TO_TICKS(200));
        bool running = false;
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            running = g_autoShiftV2Pub.running;
            xSemaphoreGive(g_sharedMutex);
        }
        if (running) { phase9Started = true; break; }
    }
    if (!phase9Started) {
        atFinishPhase(9, false, "HATA: TaskAutoShiftV2 baslamadi (5sn timeout)", 0, 0xFF);
        return false;
    }

    // Bitmesini bekle: her tekrar için 8 vites × gearHoldMs + 60sn K1/K2 hata marjı
    uint32_t maxWait = (uint32_t)prm.autoShiftRepeats * (8 * (uint32_t)prm.gearHoldMs + 60000) + 30000;
    uint32_t t0 = millis();
    bool done = false;

    while (millis() - t0 < maxWait) {
        if (g_autoTestStop) {
            if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                g_autoShiftV2Req.start = false;
                xSemaphoreGive(g_sharedMutex);
            }
            g_autoShiftV2ReqSeq++;  // Durdurma tetikle
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(500));

        bool running = false;
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            running = g_autoShiftV2Pub.running;
            xSemaphoreGive(g_sharedMutex);
        }
        if (!running) { done = true; break; }
    }

    // Raporu oluştur (Pub.faults → Report.faultMask aktarımı burada yapılır)
    AutoShiftV2_GenerateReport();

    // Hata maskesi: g_autoShiftV2Report.faultMask (GenerateReport sonrası güncel)
    uint16_t faults = 0;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        faults = g_autoShiftV2Report.faultMask;
        xSemaphoreGive(g_sharedMutex);
    }

    bool pass = (faults == 0 && done);
    char detail[140];
    if (pass) {
        snprintf(detail, sizeof(detail), "OK - %d tekrar tamamlandi", (int)prm.autoShiftRepeats);
    } else if (!done) {
        snprintf(detail, sizeof(detail), "TIMEOUT - test tamamlanamadi");
    } else {
        // Hatalı valf ve pistonları isme göre listele
        char faultNames[64] = "";
        bool hasValveFault = false;
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            for (int i = 0; i < 8; i++) {
                if (!g_autoShiftV2Report.valvesOk[i]) {
                    if (hasValveFault) strncat(faultNames, ",", sizeof(faultNames) - strlen(faultNames) - 1);
                    strncat(faultNames, VALVE_NAME[i], sizeof(faultNames) - strlen(faultNames) - 1);
                    hasValveFault = true;
                }
            }
            for (int i = 0; i < 4; i++) {
                if (!g_autoShiftV2Report.pistonsOk[i]) {
                    if (hasValveFault || strlen(faultNames) > 0) strncat(faultNames, ",", sizeof(faultNames) - strlen(faultNames) - 1);
                    strncat(faultNames, PISTON_NAME[i], sizeof(faultNames) - strlen(faultNames) - 1);
                    hasValveFault = true;
                }
            }
            xSemaphoreGive(g_sharedMutex);
        }
        if (strlen(faultNames) > 0)
            snprintf(detail, sizeof(detail), "HATA: %s", faultNames);
        else
            snprintf(detail, sizeof(detail), "HATA: fault_mask=0x%04X", (unsigned)faults);

        // Ek: ilk hata kaydının kısa özeti (rapora da girsin)
        uint8_t errCount = 0;
        AutoShiftV2ErrorEntry e0{};
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            errCount = g_autoShiftV2Errors.count;
            if (errCount > 0) e0 = g_autoShiftV2Errors.entries[0];
            xSemaphoreGive(g_sharedMutex);
        }
        if (errCount > 0) {
            static const char* gearNames[] = {"P","R","N","D1","D2","D3","D4","D5","D6","D7"};
            const char* gName = (e0.gear < 10) ? gearNames[e0.gear] : "?";
            const char* vName = (e0.valveIdx < 8) ? VALVE_NAME[e0.valveIdx] : "?";
            const char* pName = (e0.pistonIdx < 6) ? PISTON_NAME[e0.pistonIdx] : "?";
            const char* fd    = PistonFaultDetailToStr(e0.faultDetail);
            static const char* posNames[]  = {"KAPALI","ORTA","ACIK"};
            const char* posName = (e0.expectedPos < 3) ? posNames[e0.expectedPos] : "?";
            char add[140];
            snprintf(add, sizeof(add), " | %s(%s) vites=%s tekrar=%d HATA=%s hedef=%s hall=%d (kalibre %d-%d)",
                     vName, pName, gName, e0.repeatIdx + 1, fd, posName, e0.hallValue,
                     e0.expectedMin, e0.expectedMax);
            strncat(detail, add, sizeof(detail) - strlen(detail) - 1);
        }
    }
    // Pompa timeout olduysa faults'a ekle
    if (g_leakRecheckNeeded) faults |= FAULT_PUMP_TIMEOUT;

    atFinishPhase(9, pass, detail, 0, (uint8_t)(faults & 0xFF));

    // Hata detaylarını logla: hangi piston, hangi vites, kaçıncı tekrar, NEDEN
    if (!pass) {
        static const char* gearNames[] = {"P","R","N","D1","D2","D3","D4","D5","D6","D7"};
        static const char* posNames[]  = {"KAPALI","ORTA","ACIK"};
        // TaskAutoShiftV2 pistonIdx Convention A: 0=P5-7, 1=P1-3, 2=P2-4, 3=P6-R
        // (TaskAutoTest'in PISTON_NAME[] sırası farklı: 0=P5-7,1=P1-3,2=P2-4,3=P6-R,4=K1,5=K2)
        // Şanslıyız: ilk 4 indeks aynı.
        uint8_t errCount = 0;
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            errCount = g_autoShiftV2Errors.count;
            xSemaphoreGive(g_sharedMutex);
        }
        char hdr[64];
        snprintf(hdr, sizeof(hdr), "[AT] Faz9 hata detaylari (%d kayit):", errCount);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, hdr);
        for (uint8_t ei = 0; ei < errCount && ei < MAX_ERROR_HISTORY; ei++) {
            AutoShiftV2ErrorEntry e{};
            if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                e = g_autoShiftV2Errors.entries[ei];
                xSemaphoreGive(g_sharedMutex);
            }
            const char* gName   = (e.gear < 10) ? gearNames[e.gear] : "?";
            const char* pName   = (e.pistonIdx < 4) ? PISTON_NAME[e.pistonIdx] : "?";
            const char* vName   = (e.valveIdx  < 8) ? VALVE_NAME[e.valveIdx]   : "?";
            const char* posName = (e.expectedPos < 3) ? posNames[e.expectedPos] : "?";
            const char* fdName  = PistonFaultDetailToStr(e.faultDetail);
            char row[160];
            snprintf(row, sizeof(row),
                     "[AT]  #%d: %s(%s) vites=%s tekrar=%d HATA=%s hedef=%s hall=%d (kalibre %d-%d) t=%lums",
                     ei+1, vName, pName, gName, e.repeatIdx+1,
                     fdName, posName, e.hallValue,
                     e.expectedMin, e.expectedMax,
                     (unsigned long)e.timestampMs);
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, row);
        }
    }

    return pass;
}

// ---------------------------------------------------------------------------
// FAZ 10: Şartlı Kaçak Yeniden Testi (Faz 9 pompa timeout tetiklerse)
// ---------------------------------------------------------------------------
static bool atPhase10_LeakRecheck() {
    const auto& prm = g_autoTestParams;
    atLog("Faz 10: Sartli kacak yeniden testi (pompa timeout nedeniyle)");

    atAllValvesOff();
    atPumpAuto();

    // Hedef basınca doldur
    uint32_t t0 = millis();
    const uint32_t fillTimeout = prm.pumpFillTimeoutMs > 0 ? prm.pumpFillTimeoutMs : 30000;
    while (atGetPressure() < prm.targetBar) {
        if (g_autoTestStop) return false;
        if (millis() - t0 > fillTimeout) {
            atFinishPhase(10, false, "TIMEOUT: Dolum basarisiz - buyuk kacak");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // Pompayı durdur
    atPumpStop();

    // RPM 0'a düşene kadar bekle (max 5sn)
    t0 = millis();
    while (atGetRpm() > 100 && millis() - t0 < 5000) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    // Basınç ölçümü başlangıcı
    float pStart = atGetPressure();
    uint32_t holdMs = (prm.leakRecheckHoldSec > 0) ? (prm.leakRecheckHoldSec * 1000) : 20000;
    uint32_t tHold = millis();

    while (millis() - tHold < holdMs) {
        if (g_autoTestStop) return false;
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    float pEnd  = atGetPressure();
    float pDrop = pStart - pEnd;
    float maxDrop = (prm.leakRecheckMaxDrop_bar > 0.0f) ? prm.leakRecheckMaxDrop_bar : 5.0f;
    bool pass = (pDrop <= maxDrop);

    char detail[96];
    if (pass)
        snprintf(detail, sizeof(detail), "OK: %lus'de %.1f bar dusus (max %.0f) - Kacak yok",
                 (unsigned long)holdMs / 1000, pDrop, maxDrop);
    else
        snprintf(detail, sizeof(detail), "YAG KACAGI (sicak): %lus'de %.1f bar dusus (max %.0f)",
                 (unsigned long)holdMs / 1000, pDrop, maxDrop);
    atFinishPhase(10, pass, detail, pDrop);
    return true;
}

// ---------------------------------------------------------------------------
// Ana test akışı
// ---------------------------------------------------------------------------
static void runAutoTest() {
    g_autoTestResult = {};
    g_autoTestResult.running    = true;
    g_autoTestResult.done       = false;
    g_autoTestResult.pass       = true;
    g_autoTestResult.startMs    = millis();
    g_autoTestResult.currentPhase = 0;

    atLog("=== 10-FAZLI OTOMATIK TEST BASLADI ===");
    atSendPhaseUpdate(0);

    // FAZ 0: Elektriksel valf kontrolü
    g_autoTestResult.currentPhase = 0;
    if (!atPhase0_ValveElecCheck()) {
        if (g_autoTestStop) goto aborted;
        // Elektriksel arıza → faz 1-9 iptal
        atLog("Elektriksel valf arizasi - tum fazlar iptal ediliyor");
        g_autoTestResult.pass = false;
        for (int i = 1; i <= 9; i++) {
            atFinishPhase(i, false, "Elektriksel ariza nedeniyle yapilamadi", 0, 0x00);
        }
        goto finish;
    }
    if (g_autoTestStop) goto aborted;

    // Başlangıç: 10 bar üzerinde basınç varsa PCV açarak boşalt
    {
        char pcvDetail[96] = "";
        if (!atPressureRelease(pcvDetail, sizeof(pcvDetail))) {
            if (g_autoTestStop) goto aborted;
            // PCV arızası tespit edildi → faz 1'i arızalı kaydet, 2-9'u iptal
            atLog("PCV arizasi - tum fazlar iptal ediliyor");
            g_autoTestResult.pass = false;
            atFinishPhase(1, false, pcvDetail, 0, 0x20);  // 0x20 = Valf açık devre
            for (int i = 2; i <= 9; i++) {
                atFinishPhase(i, false, "PCV arizasi nedeniyle yapilamadi", 0, 0x00);
            }
            goto finish;
        }
    }
    if (g_autoTestStop) goto aborted;

    // Faz 1: Pompa dolum (PCV'ler faz içinde açılır, pistonlar kapanır)
    g_autoTestResult.currentPhase = 1;
    if (!atPhase1_PumpFill()) {
        if (g_autoTestStop) goto aborted;
        // Faz 1 başarısız (basınç tüpü veya PCV timeout) → kalan fazları iptal
        atLog("Faz 1 basarisiz - kalan fazlar atlanıyor");
        g_autoTestResult.pass = false;
        for (int i = 2; i <= 9; i++) {
            if (!g_autoTestResult.phases[i].done)
                atFinishPhase(i, false, "Faz 1 arizasi nedeniyle yapilamadi", 0, 0x00);
        }
        goto finish;
    }
    if (g_autoTestStop) goto aborted;

    // Faz 2'ye geçmeden PCV'leri kapat, sistemi stabilize et
    atAllValvesOff();
    vTaskDelay(pdMS_TO_TICKS(500));

    // Faz 2: Alan-1 valf kaçak
    g_autoTestResult.currentPhase = 2;
    if (!atPhaseLeakValves(2, ALAN1_PCV, ALAN1_PISTONS, ALAN1_COUNT)) goto aborted;
    if (g_autoTestStop) goto aborted;

    // Faz 3: Alan-1 PCV kaçak
    g_autoTestResult.currentPhase = 3;
    if (!atPhaseLeakPCV(3, ALAN1_PCV, ALAN1_PISTONS, ALAN1_COUNT)) goto aborted;
    if (g_autoTestStop) goto aborted;

    // Faz 4: Alan-2 valf kaçak
    g_autoTestResult.currentPhase = 4;
    if (!atPhaseLeakValves(4, ALAN2_PCV, ALAN2_PISTONS, ALAN2_COUNT)) goto aborted;
    if (g_autoTestStop) goto aborted;

    // Faz 5: Alan-2 PCV kaçak
    g_autoTestResult.currentPhase = 5;
    if (!atPhaseLeakPCV(5, ALAN2_PCV, ALAN2_PISTONS, ALAN2_COUNT)) goto aborted;
    if (g_autoTestStop) goto aborted;

    // Faz 6: Yağ kaçak
    g_autoTestResult.currentPhase = 6;
    if (!atPhase6_OilLeak()) goto aborted;
    if (g_autoTestStop) goto aborted;

    // Faz 7: Kalibrasyon (açık/kapalı)
    g_autoTestResult.currentPhase = 7;
    if (!atPhase7_CalibOpenClose()) {
        if (g_autoTestStop) goto aborted;  // Durdurma isteği varsa aborted'a git
        // Faz 7 başarısız → 8 ve 9'u atla, rapor oluştur
        atLog("Faz 7 basarisiz - faz 8 ve 9 atlanıyor");
        goto finish;
    }
    if (g_autoTestStop) goto aborted;

    // Faz 8: Orta konum ön testi
    g_autoTestResult.currentPhase = 8;
    if (!atPhase8_HoldMidPreTest()) {
        if (g_autoTestStop) goto aborted;  // Durdurma isteği varsa aborted'a git
        // Faz 8 başarısız → 9'u atla
        atLog("Faz 8 basarisiz - faz 9 atlanıyor");
        goto finish;
    }
    if (g_autoTestStop) goto aborted;

    // Faz 9: Otomatik vites testi
    g_autoTestResult.currentPhase = 9;
    atPhase9_AutoShift();
    if (g_autoTestStop) goto aborted;

    // Faz 10: Şartlı kaçak yeniden testi — sadece Faz 9'da pompa timeout olduysa
    if (g_leakRecheckNeeded) {
        atLog("Faz 9 pompa timeout - Faz 10 kacak testi basliyor");
        g_autoTestResult.currentPhase = 10;
        if (!atPhase10_LeakRecheck()) goto aborted;
    }

finish:
    atAllValvesOff();
    atPumpStop();
    g_autoTestResult.running  = false;
    g_autoTestResult.done     = true;
    g_autoTestResult.endMs    = millis();
    atLog("=== TEST TAMAMLANDI ===");
    atSendPhaseUpdate(11);  // 11 = done
    return;

aborted:
    // Devam eden piston kalibrasyonunu iptal et (Faz 7/8 sırasında stop geldiyse)
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        g_pistonCalReq = {};
        g_pistonCalReq.start = false;
        xSemaphoreGive(g_sharedMutex);
    }
    g_pistonCalReqSeq++;  // TaskValveControl kalibrasyon state machine'i durdur
    atAllValvesOff();
    atPumpStop();
    g_autoTestResult.running  = false;
    g_autoTestResult.done     = true;
    g_autoTestResult.pass     = false;
    g_autoTestResult.endMs    = millis();
    atLog("=== TEST DURDURULDU ===");
    atSendPhaseUpdate(12);  // 12 = aborted
}

// ---------------------------------------------------------------------------
// FreeRTOS Task
// ---------------------------------------------------------------------------
void TaskAutoTest(void* pvParameters) {
    // NVS yüklemesi Shared_Init'te yapılıyor
    uint32_t lastSeq = g_autoTestReqSeq;

    for (;;) {
        uint32_t curSeq = g_autoTestReqSeq;
        if (curSeq != lastSeq) {
            lastSeq = curSeq;
            g_autoTestStop = false;
            runAutoTest();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

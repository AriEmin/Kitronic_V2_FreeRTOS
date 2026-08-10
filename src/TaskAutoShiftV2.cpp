// src/TaskAutoShiftV2.cpp
#include "AutoShiftV2.h"
#include "Shared.h"
#include "Protocol.h"

// ============================================================================
// PWM Sabitleri - Max 2000 PWM!
// ============================================================================
static const uint16_t PWM_MAX           = 2000;   // Kesinlikle aşılmamalı!
static const uint16_t PWM_OPEN          = 1500;   // Pistonu açmak için
static const uint16_t PWM_CLOSE         = 850;    // Pistonu kapatmak için
static const uint16_t PWM_HOLD          = 1100;   // Ortada tutmak için
static const uint16_t PWM_MID_OPEN      = 1250;   // Orta konum için yumuşak açma (salınımı önler)
static const uint16_t PWM_MID_CLOSE     = 950;    // Orta konum için yumuşak kapatma (salınımı önler)
static const uint16_t PWM_CLUTCH        = 1500;   // Kavrama valfleri
static const uint16_t PWM_MIN           = 0;      // Kapalı
static const uint16_t PWM_KEEPALIVE     = 400;    // Test sırasında minimum valf PWM (~250mA bobin sıcaklık muhafaza)
static const uint16_t PWM_MAIN_VALVE    = 1400;   // Ana basınç valfleri (N436, N440) - PI akım kontrolü 1000mA'e regüle eder
static const uint16_t PWM_PISTON_OPEN   = 1500;   // Piston açma (eski kod uyumu)
static const uint16_t PWM_PISTON_MID    = 1100;   // Piston orta (eski kod uyumu)

// ============================================================================
// Global Değişkenler
// ============================================================================
volatile uint32_t     g_autoShiftV2ReqSeq = 0;
AutoShiftV2Request    g_autoShiftV2Req = {};
AutoShiftV2Publish    g_autoShiftV2Pub = {};
AutoShiftV2Config     g_autoShiftV2Cfg = {
    .gearHoldMs       = 2000,
    .clutchTransMs    = 300,
    .preselectMs      = 200,
    .minPressureBar   = 42.0f,
    .maxPressureBar   = 60.0f,
    .pressureDropWarn = 6.0f,
    .pressureDropFault= 10.0f,
    .valveCurrentMin  = 600,
    .valveCurrentMax  = 900,
    .mainValveCurrent = 950,
    .autoRepeat       = false,
    .repeatCount      = 1,
    .targetGear       = GEAR_D7,
    .manualMode       = false
};
AutoShiftV2Report     g_autoShiftV2Report = {};
AutoShiftV2StepDiag   g_autoShiftV2StepDiag = {};
AutoShiftV2ErrorHistory g_autoShiftV2Errors = {};
PerPistonStats        g_pistonStats[4] = {};

// 4 piston valfi için kalibrasyon (varsayılan değerler)
// İndeks: 0=1-3, 1=5-7, 2=2-4, 3=6-R (setPistonValve Convention B sırası)
PistonValveCalib      g_pistonCalib[4] = {
    // Piston 1-3 (N433/V0)
    { .dutyHold = 950, .dutyExtendBase = 1100, .dutyRetractBase = 800, .dutyMax = 2000, .kDutyPerMm = 30.0f, .deadbandMm = 0.5f, .calibrated = false },
    // Piston 5-7 (N434/V2)
    { .dutyHold = 950, .dutyExtendBase = 1100, .dutyRetractBase = 800, .dutyMax = 2000, .kDutyPerMm = 30.0f, .deadbandMm = 0.5f, .calibrated = false },
    // Piston 2-4 (V3)
    { .dutyHold = 950, .dutyExtendBase = 1100, .dutyRetractBase = 800, .dutyMax = 2000, .kDutyPerMm = 30.0f, .deadbandMm = 0.5f, .calibrated = false },
    // Piston 6-R (V4)
    { .dutyHold = 950, .dutyExtendBase = 1100, .dutyRetractBase = 800, .dutyMax = 2000, .kDutyPerMm = 30.0f, .deadbandMm = 0.5f, .calibrated = false }
};

// ============================================================================
// Vites Pozisyon Tablosu
// ============================================================================
// Sıralama: p1_3, p5_7, p2_4, p6_R, clutch (GearPosition struct sırası!)
const GearPosition GEAR_POSITIONS[GEAR_COUNT] = {
    // GEAR_P:  Tüm pistonlar orta, 6-R açık, kavrama yok
    { POS_CLOSED,    POS_CLOSED,    POS_CLOSED,    POS_OPEN,   CLUTCH_NONE },
    // GEAR_R:  6-R açık, K2 aktif
    { POS_CLOSED,    POS_CLOSED,    POS_CLOSED,    POS_OPEN,   CLUTCH_K2 },
    // GEAR_N:  1-3 açık, 5-7 orta, 6-R açık, kavrama yok
    { POS_OPEN,   POS_CLOSED,    POS_CLOSED,    POS_OPEN,   CLUTCH_NONE },
    // GEAR_D1: 1-3 açık, 5-7 orta, 2-4 açık, 6-R orta, K1
    { POS_OPEN,   POS_CLOSED,    POS_OPEN,   POS_CLOSED,    CLUTCH_K1 },
    // GEAR_D2: 1-3 kapalı, 5-7 orta, 2-4 açık, 6-R orta, K2
    { POS_CLOSED, POS_CLOSED,    POS_OPEN,   POS_CLOSED,    CLUTCH_K2 },
    // GEAR_D3: 1-3 kapalı, 5-7 orta, 2-4 kapalı, 6-R orta, K1
    { POS_CLOSED, POS_CLOSED,    POS_CLOSED, POS_CLOSED,    CLUTCH_K1 },
    // GEAR_D4: 1-3 orta, 5-7 açık, 2-4 kapalı, 6-R orta, K2
    { POS_CLOSED,    POS_OPEN,   POS_CLOSED, POS_CLOSED,    CLUTCH_K2 },
    // GEAR_D5: 1-3 orta, 5-7 açık, 2-4 orta, 6-R kapalı, K1
    { POS_CLOSED,    POS_OPEN,   POS_CLOSED,    POS_CLOSED, CLUTCH_K1 },
    // GEAR_D6: 1-3 orta, 5-7 kapalı, 2-4 orta, 6-R kapalı, K2
    { POS_CLOSED,    POS_CLOSED, POS_CLOSED,    POS_CLOSED, CLUTCH_K2 },
    // GEAR_D7: 1-3 orta, 5-7 kapalı, 2-4 orta, 6-R kapalı, K1
    { POS_CLOSED,    POS_CLOSED, POS_CLOSED,    POS_CLOSED, CLUTCH_K1 }
};


/*const GearPosition GEAR_POSITIONS[GEAR_COUNT] = {
    // GEAR_P:  Tüm pistonlar orta, 6-R açık, kavrama yok
    { POS_MID,    POS_MID,    POS_MID,    POS_OPEN,   CLUTCH_NONE },
    // GEAR_R:  6-R açık, K2 aktif
    { POS_MID,    POS_MID,    POS_MID,    POS_OPEN,   CLUTCH_K2 },
    // GEAR_N:  1-3 açık, 5-7 orta, 6-R açık, kavrama yok
    { POS_OPEN,   POS_MID,    POS_MID,    POS_OPEN,   CLUTCH_NONE },
    // GEAR_D1: 1-3 açık, 5-7 orta, 2-4 açık, 6-R orta, K1
    { POS_OPEN,   POS_MID,    POS_OPEN,   POS_MID,    CLUTCH_K1 },
    // GEAR_D2: 1-3 kapalı, 5-7 orta, 2-4 açık, 6-R orta, K2
    { POS_CLOSED, POS_MID,    POS_OPEN,   POS_MID,    CLUTCH_K2 },
    // GEAR_D3: 1-3 kapalı, 5-7 orta, 2-4 kapalı, 6-R orta, K1
    { POS_CLOSED, POS_MID,    POS_CLOSED, POS_MID,    CLUTCH_K1 },
    // GEAR_D4: 1-3 orta, 5-7 açık, 2-4 kapalı, 6-R orta, K2
    { POS_MID,    POS_OPEN,   POS_CLOSED, POS_MID,    CLUTCH_K2 },
    // GEAR_D5: 1-3 orta, 5-7 açık, 2-4 orta, 6-R kapalı, K1
    { POS_MID,    POS_OPEN,   POS_MID,    POS_CLOSED, CLUTCH_K1 },
    // GEAR_D6: 1-3 orta, 5-7 kapalı, 2-4 orta, 6-R kapalı, K2
    { POS_MID,    POS_CLOSED, POS_MID,    POS_CLOSED, CLUTCH_K2 },
    // GEAR_D7: 1-3 orta, 5-7 kapalı, 2-4 orta, 6-R kapalı, K1
    { POS_MID,    POS_CLOSED, POS_MID,    POS_CLOSED, CLUTCH_K1 }
};*/

// ============================================================================
// Yardımcı Fonksiyonlar
// ============================================================================

const char* GearStateToString(GearState gear) {
    static const char* names[] = {"P", "R", "N", "D1", "D2", "D3", "D4", "D5", "D6", "D7"};
    if (gear < GEAR_COUNT) return names[gear];
    return "??";
}

const char* PhaseToString(AutoShiftV2Phase phase) {
    static const char* names[] = {
        "IDLE", "INIT", "PRESSURE", "PRESELECT", "CLUTCH", 
        "ACTIVE", "SHIFT", "DONE", "ERROR", "ABORT"
    };
    if (phase <= PHASE_ABORTING) return names[phase];
    return "??";
}

// Basınç okuma (bar)
static float readPressure() {
    // Shared.h'den g_pumpPub.bar kullanılıyor
    return g_pumpPub.bar;
}

// Piston pozisyonunu oku (mm)
static float readPistonPosition(uint8_t pistonIdx) {
    if (pistonIdx >= PISTON_CHANNEL_COUNT) return 0.0f;
    return g_pistonHallmm[pistonIdx];
}

// Piston durumunu belirle (CLOSED/MID/OPEN)
static PistonPos determinePistonState(uint8_t pistonIdx) {
    if (pistonIdx >= PISTON_CHANNEL_COUNT) return POS_CLOSED;
    return (PistonPos)g_pistonState[pistonIdx];
}

// Valf akımını oku (mA)
static float readValveCurrent(uint8_t valveIdx) {
    if (valveIdx >= 8) return 0.0f;
    return g_valveCurrent_mA[valveIdx];
}

// Keepalive flag: test sırasında valfler 0'a düşmesin (bobin ısınmasını muhafaza eder, tepki süresini kısaltır)
static bool s_keepAliveActive = false;

// Valf PWM ayarla
static void setValveDuty(uint8_t valveIdx, uint16_t duty) {
    if (valveIdx >= 8) return;
    // Test sırasında valfler 0'a düşmesin: bobin ısınması ve hızlı tepki için minimum PWM uygula
    if (s_keepAliveActive && duty == 0) duty = PWM_KEEPALIVE;
    g_valveTargetDuty[valveIdx] = duty;
}

// Tüm valfleri kapat (keepalive'i bypass eder - test sonu için)
static void closeAllValves() {
    s_keepAliveActive = false;
    for (uint8_t i = 0; i < 8; i++) {
        g_valveTargetDuty[i] = 0;  // Keepalive bypass - gerçekten 0
    }
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        for (uint8_t i = 0; i < 8; i++) {
            g_valveCustomCurrent_mA[i] = 0.0f;  // Akım hedeflerini de sıfırla
        }
        xSemaphoreGive(g_sharedMutex);
    }
}

// Ana valfleri aç (V1, V5) - Max 2000 PWM!
static void openMainValves() {
    s_keepAliveActive = true;  // Test başlıyor - keepalive aktif
    g_valveTargetDuty[VALVE_MAIN1] = PWM_MAIN_VALVE;
    g_valveTargetDuty[VALVE_MAIN2] = PWM_MAIN_VALVE;
}

// Ana valfleri kapat
static void closeMainValves() {
    g_valveTargetDuty[VALVE_MAIN1] = 0;
    g_valveTargetDuty[VALVE_MAIN2] = 0;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
        g_valveCustomCurrent_mA[VALVE_MAIN1] = 0.0f;
        g_valveCustomCurrent_mA[VALVE_MAIN2] = 0.0f;
        xSemaphoreGive(g_sharedMutex);
    }
}

// Kavrama valfi kontrolü - Max 2000 PWM!
static void setClutch(ClutchState clutch) {
    switch (clutch) {
        case CLUTCH_K1:
            g_valveTargetDuty[VALVE_K1] = PWM_CLUTCH;
            setValveDuty(VALVE_K2, 0);  // keepalive uygulanır
            break;
        case CLUTCH_K2:
            setValveDuty(VALVE_K1, 0);  // keepalive uygulanır
            g_valveTargetDuty[VALVE_K2] = PWM_CLUTCH;
            break;
        default:
            setValveDuty(VALVE_K1, 0);  // keepalive uygulanır
            setValveDuty(VALVE_K2, 0);  // keepalive uygulanır
            break;
    }
}

// Son gönderilen piston pozisyonları (tekrar gönderimi önlemek için)
static PistonPos s_lastPistonPos[4] = {POS_CLOSED, POS_CLOSED, POS_CLOSED, POS_CLOSED};

// Pre-position state machine (manuel davranisini taklit):
//   POS_MID hedefine fresh giriste piston end-stop'tan basliyorsa, PD hold ENABLE
//   etmeden once yumusak duty (dutyHold +/- delta) ile ~13 mm civarina yaklas.
//   |err| < 2 mm olunca PD hold devralir. Boylelikle PD initial saturate olmaz,
//   end-stop overshoot ve bang-bang osilasyonu olmaz.
static PistonPos s_lastSetTarget[4]   = {POS_CLOSED, POS_CLOSED, POS_CLOSED, POS_CLOSED};
static bool      s_pistonInPrePos[4]  = {false, false, false, false};
static constexpr float PREPOS_EXIT_MM   = 2.0f;   // bu kadar yakinsa PD'ye devret
static constexpr float PREPOS_DEAD_MM   = 1.0f;   // bu deadband icinde sadece holdDuty
static constexpr uint16_t PREPOS_DELTA  = 150;    // PWM, hold etrafinda yumusak yon

// Piston hold isteği gönder (TaskValveControl tarafından işlenir)
// Sadece pozisyon değiştiğinde gönderir
static void queuePistonHold(uint8_t pistonIdx, PistonPos targetPos) {
    if (pistonIdx >= 4) return;
    
    // Pozisyon değişmediyse tekrar gönderme
    if (s_lastPistonPos[pistonIdx] == targetPos) return;
    s_lastPistonPos[pistonIdx] = targetPos;
    
    PistonHoldRequest req{};
    req.piston = pistonIdx;
    req.tolerance = 0.0f;
    
    if (targetPos == POS_MID) {
        req.enable = true;
        req.state = PISTON_REF_MID;  // x_ref = 0.5
    } else if (targetPos == POS_OPEN) {
        req.enable = true;
        req.state = PISTON_REF_OPEN; // x_ref = 1.0
    } else {
        req.enable = false;
        req.state = PISTON_REF_CLOSED;
    }
    
    if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_pistonHoldReq[pistonIdx] = req;
        xSemaphoreGive(g_sharedMutex);
        portENTER_CRITICAL(&g_portMux);
        g_pistonHoldReqSeq[pistonIdx]++;
        portEXIT_CRITICAL(&g_portMux);
    }
}

// Tüm piston pozisyonlarını sıfırla (test başlangıcında)
static void resetPistonHoldState() {
    for (int i = 0; i < 4; i++) {
        s_lastPistonPos[i]   = POS_CLOSED;
        s_lastSetTarget[i]   = POS_CLOSED;
        s_pistonInPrePos[i]  = false;
    }
}

// Tüm pistonların hold kontrolünü kapat (test bitiminde)
static void disableAllPistonHold() {
    for (int i = 0; i < 4; i++) {
        s_lastPistonPos[i] = POS_CLOSED;  // Tekrar gönderim için sıfırla
        
        PistonHoldRequest req{};
        req.piston = i;
        req.enable = false;
        req.state = PISTON_REF_CLOSED;
        req.tolerance = 0.0f;
        
        if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            g_pistonHoldReq[i] = req;
            xSemaphoreGive(g_sharedMutex);
            portENTER_CRITICAL(&g_portMux);
            g_pistonHoldReqSeq[i]++;
            portEXIT_CRITICAL(&g_portMux);
        }
    }
}

// Convention B (setPistonValve native) → valve / Convention A donusumu
// 0=P1-3, 1=P5-7, 2=P2-4, 3=P6-R
static const uint8_t kPistonToValve_B[4] = {VALVE_1_3, VALVE_5_7, VALVE_2_4, VALVE_6_R};
static const uint8_t kBToHoldIdx[4]      = {1, 0, 2, 3};

// Piston valfi kontrolü (pozisyona göre) - PD Hold destekli
static void setPistonValve(uint8_t pistonIdx, PistonPos targetPos) {
    if (pistonIdx >= 4) return;
    
    uint8_t valveIdx = kPistonToValve_B[pistonIdx];
    uint8_t holdPiston = kBToHoldIdx[pistonIdx];
    const PistonValveCalib& cal = g_pistonCalib[pistonIdx];

    // Pre-position state takibi: POS_MID'e fresh giriste s_pistonInPrePos=true
    PistonPos prevTarget = s_lastSetTarget[pistonIdx];
    s_lastSetTarget[pistonIdx] = targetPos;
    if (targetPos == POS_MID && prevTarget != POS_MID) {
        s_pistonInPrePos[pistonIdx] = true;  // fresh entry → pre-position fazi
    }
    if (targetPos != POS_MID) {
        s_pistonInPrePos[pistonIdx] = false; // baska hedef → reset
    }
    
    // POS_OPEN için sürekli açma PWM uygula (PD hold KULLANMA)
    if (targetPos == POS_OPEN) {
        queuePistonHold(holdPiston, POS_CLOSED);  // Varsa hold'u kapat
        uint16_t duty = cal.calibrated ? cal.dutyExtendBase : PWM_PISTON_OPEN;
        if (duty > PWM_MAX) duty = PWM_MAX;
        g_valveTargetDuty[valveIdx] = duty;
        return;
    }
    
    // POS_MID: pre-position → PD hold devir
    if (targetPos == POS_MID) {
        // KRITIK: setPistonValve Convention B (0=P1-3,1=P5-7) kullanir, ama
        // g_pistonHallmm[], g_tmagPistonCalib[], g_pistonCalibData[] hepsi
        // Convention A (PistonChannel) sirasinda. Donusum sart!
        uint8_t pA = holdPiston;  // toHoldIdx[pistonIdx] zaten Convention A indeksi

        // Anlik piston pozisyonu (mm) - dogru pistondan oku
        float posMm = g_pistonHallmm[pA];
        float strokeMm = (g_tmagPistonCalib[pA].valid &&
                          g_tmagPistonCalib[pA].strokeMm > 5.0f)
                         ? g_tmagPistonCalib[pA].strokeMm : 26.0f;
        float midMm = strokeMm * 0.5f;
        float errMm = posMm - midMm;

        // aynı hedef (POS_MID) sürerken drift büyükse pre-position tekrar başlat
        if (targetPos == POS_MID && prevTarget == POS_MID && !s_pistonInPrePos[pistonIdx] && fabsf(errMm) > PREPOS_EXIT_MM) {
            s_pistonInPrePos[pistonIdx] = true;
        }

        // Akim kalibrasyonu mevcut mu? (Faz 8 sonrasi open_mA/close_mA/hold_mA dolu)
        bool hasCurCalib = (pA < 4) && g_pistonCalibData[pA].calibrated &&
                           g_pistonCalibData[pA].open_mA  > 100.0f &&
                           g_pistonCalibData[pA].close_mA > 100.0f &&
                           g_pistonCalibData[pA].hold_mA  > 100.0f;

        // Pre-position fazindaysa ve hala uzaktaysa: PD hold OFF, yumusak surus
        if (s_pistonInPrePos[pistonIdx] && fabsf(errMm) > PREPOS_EXIT_MM) {
            queuePistonHold(holdPiston, POS_CLOSED);  // PD hold disable

            if (hasCurCalib) {
                // Kalibre akim degerleri ile PI akim donguusu uzerinden surus
                float openMa  = g_pistonCalibData[pA].open_mA;
                float closeMa = g_pistonCalibData[pA].close_mA;
                float holdMa  = g_pistonCalibData[pA].hold_mA;
                float tgtMa;
                if (errMm < -PREPOS_DEAD_MM) {
                    tgtMa = openMa + 30.0f;       // yumusak ac (esigin uzerinde)
                } else if (errMm > PREPOS_DEAD_MM) {
                    tgtMa = closeMa - 20.0f;      // yumusak kapat (esigin altinda)
                    if (tgtMa < 350.0f) tgtMa = 350.0f;
                } else {
                    tgtMa = holdMa;               // deadband: dengele
                }
                if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                    g_valveCustomCurrent_mA[valveIdx] = tgtMa;
                    xSemaphoreGive(g_sharedMutex);
                }
                g_valveTargetDuty[valveIdx] = 0;  // PI loop yonetir
            } else {
                // Fallback: kalibrasyon yok → PWM open-loop, daha buyuk delta
                uint16_t holdDuty = (uint16_t)PWM_PISTON_MID;
                constexpr uint16_t FALLBACK_DELTA = 250;
                uint16_t duty;
                if (errMm < -PREPOS_DEAD_MM) {
                    uint32_t d = (uint32_t)holdDuty + FALLBACK_DELTA;
                    duty = (d > PWM_MAX) ? (uint16_t)PWM_MAX : (uint16_t)d;
                } else if (errMm > PREPOS_DEAD_MM) {
                    duty = (holdDuty > FALLBACK_DELTA)
                           ? (uint16_t)(holdDuty - FALLBACK_DELTA)
                           : (uint16_t)PWM_MIN;
                } else {
                    duty = holdDuty;
                }
                g_valveTargetDuty[valveIdx] = duty;
            }
            return;
        }

        // Hedefe yakin (|err| <= EXIT_MM) → PD hold devraliyor
        s_pistonInPrePos[pistonIdx] = false;
        queuePistonHold(holdPiston, targetPos);
        uint16_t holdDuty = cal.calibrated ? cal.dutyHold : (uint16_t)PWM_PISTON_MID;
        if (holdDuty > PWM_MAX) holdDuty = PWM_MAX;
        g_valveTargetDuty[valveIdx] = holdDuty;
        return;
    }
    
    // POS_CLOSED için hold'u kapat ve valfi kapat
    queuePistonHold(holdPiston, POS_CLOSED);
    g_valveTargetDuty[valveIdx] = PWM_MIN;
}

// Pre-position fazindaki pistonlarin sürüsünü her dongude güncelle.
// setPistonValve sadece gear giriste bir kez calisir; bu fonksiyon
// pre-pos sirasinda piston pozisyonuna gore tgt akimi yeniden hesaplar.
static void updatePistonPrePos() {
    for (uint8_t pistonIdxB = 0; pistonIdxB < 4; pistonIdxB++) {
        if (!s_pistonInPrePos[pistonIdxB]) continue;

        uint8_t valveIdx = kPistonToValve_B[pistonIdxB];
        uint8_t pA       = kBToHoldIdx[pistonIdxB];

        // Anlik pozisyon (Convention A indeksli globaller)
        float posMm = g_pistonHallmm[pA];
        float strokeMm = (g_tmagPistonCalib[pA].valid &&
                          g_tmagPistonCalib[pA].strokeMm > 5.0f)
                         ? g_tmagPistonCalib[pA].strokeMm : 26.0f;
        float midMm = strokeMm * 0.5f;
        float errMm = posMm - midMm;

        // Hedefe yeterince yakin → PD hold devraliyor (handoff)
        if (fabsf(errMm) <= PREPOS_EXIT_MM) {
            s_pistonInPrePos[pistonIdxB] = false;
            queuePistonHold(pA, POS_MID);
            const PistonValveCalib& cal = g_pistonCalib[pistonIdxB];
            uint16_t holdDuty = cal.calibrated ? cal.dutyHold : (uint16_t)PWM_PISTON_MID;
            if (holdDuty > PWM_MAX) holdDuty = PWM_MAX;
            g_valveTargetDuty[valveIdx] = holdDuty;
            continue;
        }

        // Hala uzakta → pre-pos surusunu guncelle
        bool hasCurCalib = (pA < 4) && g_pistonCalibData[pA].calibrated &&
                           g_pistonCalibData[pA].open_mA  > 100.0f &&
                           g_pistonCalibData[pA].close_mA > 100.0f &&
                           g_pistonCalibData[pA].hold_mA  > 100.0f;

        if (hasCurCalib) {
            float openMa  = g_pistonCalibData[pA].open_mA;
            float closeMa = g_pistonCalibData[pA].close_mA;
            float holdMa  = g_pistonCalibData[pA].hold_mA;
            float tgtMa;
            if (errMm < -PREPOS_DEAD_MM) {
                tgtMa = openMa + 30.0f;
            } else if (errMm > PREPOS_DEAD_MM) {
                tgtMa = closeMa - 20.0f;
                if (tgtMa < 350.0f) tgtMa = 350.0f;
            } else {
                tgtMa = holdMa;
            }
            if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
                g_valveCustomCurrent_mA[valveIdx] = tgtMa;
                xSemaphoreGive(g_sharedMutex);
            }
            g_valveTargetDuty[valveIdx] = 0;  // PI loop yonetir
        } else {
            uint16_t holdDuty = (uint16_t)PWM_PISTON_MID;
            constexpr uint16_t FALLBACK_DELTA = 250;
            uint16_t duty;
            if (errMm < -PREPOS_DEAD_MM) {
                uint32_t d = (uint32_t)holdDuty + FALLBACK_DELTA;
                duty = (d > PWM_MAX) ? (uint16_t)PWM_MAX : (uint16_t)d;
            } else if (errMm > PREPOS_DEAD_MM) {
                duty = (holdDuty > FALLBACK_DELTA)
                       ? (uint16_t)(holdDuty - FALLBACK_DELTA)
                       : (uint16_t)PWM_MIN;
            } else {
                duty = holdDuty;
            }
            g_valveTargetDuty[valveIdx] = duty;
        }
    }
}

// Vites pozisyonlarını uygula
static void applyGearPosition(GearState gear) {
    if (gear >= GEAR_COUNT) return;
    
    const GearPosition& pos = GEAR_POSITIONS[gear];
    
    // Piston valflerini ayarla
    setPistonValve(0, pos.p1_3);
    setPistonValve(1, pos.p5_7);
    setPistonValve(2, pos.p2_4);
    setPistonValve(3, pos.p6_R);
    
    // Kavramayı ayarla
    setClutch(pos.clutch);
}

// Pompa otomatik kontrol başlat
static void startPumpAuto() {
    g_pumpCmd.cmd = PUMP_CMD_AUTO;
    g_pumpCmd.seq++;
}

// Pompa durdur
static void stopPump() {
    g_pumpCmd.cmd = PUMP_CMD_STOP;
    g_pumpCmd.seq++;
}

// Valf akımını kontrol et
static uint16_t checkValveCurrent(uint8_t valveIdx, bool isMain) {
    float current = readValveCurrent(valveIdx);
    uint16_t minCurrent = isMain ? g_autoShiftV2Cfg.mainValveCurrent - 100 : g_autoShiftV2Cfg.valveCurrentMin;
    uint16_t maxCurrent = isMain ? g_autoShiftV2Cfg.mainValveCurrent + 200 : g_autoShiftV2Cfg.valveCurrentMax;
    
    if (current < minCurrent && g_valveTargetDuty[valveIdx] > 0) {
        return FAULT_VALVE_OPEN;
    }
    if (current > maxCurrent) {
        return FAULT_VALVE_SHORT;
    }
    return FAULT_NONE;
}

// Piston pozisyonunu kontrol et
static uint16_t checkPistonPosition(uint8_t pistonIdx, PistonPos expected) {
    PistonPos actual = determinePistonState(pistonIdx);
    if (actual != expected) {
        return FAULT_PISTON_UNEXPECTED;
    }
    return FAULT_NONE;
}

// ============================================================================
// Rapor Üretimi
// ============================================================================
void AutoShiftV2_GenerateReport() {
    AutoShiftV2Report& r = g_autoShiftV2Report;
    
    // Test süresi
    r.testEndMs = millis();
    
    // Pub.faults → Report.faultMask (asıl hata kaynağı Pub'da tutuluyor)
    r.faultMask = g_autoShiftV2Pub.faults;
    
    // Bileşen durumlarını değerlendir
    r.pumpOk = !(r.faultMask & FAULT_PUMP_ERROR);
    r.pressureSensorOk = !(r.faultMask & FAULT_SENSOR_ERROR);

    // Varsayılan: hepsi OK
    for (uint8_t i = 0; i < 8; i++) r.valvesOk[i] = true;
    for (uint8_t i = 0; i < 4; i++) r.pistonsOk[i] = true;

    // Hata geçmişinden etkilenmiş piston/valfleri işaretle
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        uint8_t cnt = g_autoShiftV2Errors.count;
        for (uint8_t ei = 0; ei < cnt && ei < MAX_ERROR_HISTORY; ei++) {
            const AutoShiftV2ErrorEntry &e = g_autoShiftV2Errors.entries[ei];
            if (e.pistonIdx < 4) r.pistonsOk[e.pistonIdx] = false;
            if (e.valveIdx  < 8) r.valvesOk[e.valveIdx]   = false;
        }
        xSemaphoreGive(g_sharedMutex);
    }
    
    r.clutchK1Ok = true;  // Detaylı kontrol eklenecek
    r.clutchK2Ok = true;
    
    // Tamir önerisi oluştur
    if (r.faultMask == FAULT_NONE) {
        snprintf(r.recommendation, sizeof(r.recommendation), "Mekatronik saglikli");
        r.testPassed = true;
    } else {
        r.testPassed = false;
        
        if (r.faultMask & FAULT_VALVE_OPEN) {
            snprintf(r.recommendation, sizeof(r.recommendation), "Valf bobini kontrol et");
        } else if (r.faultMask & FAULT_VALVE_SHORT) {
            snprintf(r.recommendation, sizeof(r.recommendation), "Valf kisa devre - degistir");
        } else if (r.faultMask & FAULT_PISTON_STUCK) {
            snprintf(r.recommendation, sizeof(r.recommendation), "Piston sikismis - temizle");
        } else if (r.faultMask & FAULT_PRESSURE_DROP) {
            snprintf(r.recommendation, sizeof(r.recommendation), "Basinci kacak - conta kontrol");
        } else if (r.faultMask & FAULT_PUMP_ERROR) {
            snprintf(r.recommendation, sizeof(r.recommendation), "Pompa hatasi");
        } else {
            snprintf(r.recommendation, sizeof(r.recommendation), "Detayli inceleme gerekli");
        }
    }
}

// ============================================================================
// TEST FONKSİYONU - Kullanıcı buraya kendi kodunu yazacak
// Bu fonksiyon her 20ms'de bir çağrılır (running=true iken)
// Mevcut değerler:
//   - g_autoShiftV2Pub.pressure: Basınç değeri (bar)
//   - g_tmagData[ch].z: Hall sensör değerleri (piston pozisyonu)
//   - g_valveTargetDuty[valve]: Valf PWM değerleri (0-2000)
// Kullanılabilir fonksiyonlar:
//   - startPumpAuto(): Pompayı otomatik modda başlat
//   - stopPump(): Pompayı durdur
//   - openMainValves(): Ana valfleri aç (N436, N440)
// ============================================================================

// ============================================================================
// Bang-Bang Pozisyon Kontrolü ile Vites Geçişi
// ============================================================================

// Piston-Valf-TMAG eşleştirme tablosu - FİZİKSEL bağlantıya göre!
// Valf indeksleri: 0:N433  1:N436  2:N434  3:N435  4:N438  5:N440  6:N439  7:N437
// N433 (idx 0) → P1-3 kontrolü
// N434 (idx 2) → P5-7 kontrolü
// pistonIdx 0 = P5-7 → VALVE_5_7 (2)
// pistonIdx 1 = P1-3 → VALVE_1_3 (0)
static const uint8_t PISTON_VALVE[4] = {2, 0, 7, 4};  // N434(P5-7), N433(P1-3), N437, N438
static const uint8_t PISTON_TMAG[4]  = {TMAG_CH_5_7, TMAG_CH_1_3, TMAG_CH_2_4, TMAG_CH_6_R};

// Hall hedef aralıkları - kalibrasyondan alınır
// Tolerans yüzdeleri (strok mesafesinin %'si olarak)
static const float HALL_CONTROL_TOL_PCT = 0.05f;    // %5 tolerans (hareket kontrolü için - dar)
static const float HALL_TARGET_TOL_PCT = 0.10f;     // %10 tolerans (hedef kontrolü için - geniş)

// Pistonun kalibrasyon değerlerini al
// NOT: Hall sensörü lineer değil! 
// 0-1600 hall = 0-13mm (123 hall/mm), 1600-13000 hall = 13-26mm (877 hall/mm)
// Orta konum toleransı strok yüzdesi olarak hesaplanır (HALL_TARGET_TOL_PCT = %10)
// Sabit MID_HALL kaldırıldı - kalibrasyondan hesaplanıyor

static void getCalibLimits(int pistonIdx, int16_t& zMin, int16_t& zMid, int16_t& zMax) {
    // Akım kalibrasyonu sonucu (TaskCurrentCalib tarafından doldurulur)
    if (pistonIdx < 4 && g_pistonCalibData[pistonIdx].calibrated) {
        zMin = (int16_t)g_pistonCalibData[pistonIdx].min_raw;
        zMid = (int16_t)g_pistonCalibData[pistonIdx].mid_raw;
        zMax = (int16_t)g_pistonCalibData[pistonIdx].max_raw;
        return;
    }
    
    // 3. TMAG kalibrasyon verisi (manuel)
    const TMAGPistonCalib& cal = g_tmagPistonCalib[pistonIdx];
    if (cal.valid) {
        zMin = cal.zMin;
        zMax = cal.zMax;
        zMid = (zMin + zMax) / 2;  // Gerçek geometrik orta
    } else {
        // 4. Son çare - tipik sensör değerleri
        zMin = 1300;
        zMax = 32000;
        zMid = 16500;
    }
}

// Kalibre edilmiş hold duty değerini al
static uint16_t getCalibHoldDuty(int pistonIdx) {
    return PWM_HOLD;  // Varsayılan: 1100
}

// Test modu
enum TestMode : uint8_t {
    MODE_HOLD_MID = 0,     // Tüm pistonları ortada tut
    MODE_MANUAL_GEAR,      // GUI'den seçilen vitese git
    MODE_SEQUENTIAL_TEST   // Sıralı vites testi (1→2→3→4→5→6→7)
};

// Test durumu
static bool s_systemInitialized = false;
static TestMode s_testMode = MODE_HOLD_MID;
static GearState s_currentGear = GEAR_N;
static GearState s_targetGear = GEAR_N;
static uint32_t s_gearChangeStartMs = 0;
static uint32_t s_gearHoldStartMs = 0;
static bool s_gearReached = false;
static bool s_gearDirectionUp = true;  // true=yukarı (N→D7), false=aşağı (D7→N)
static uint32_t s_repeatIdx = 0;       // Mevcut tekrar sayacı
static bool s_testCompleted = false;   // Test tamamlandı mı

// Vites geçişi basınç takibi (per-piston istatistik için)
static float    s_gearPressureAtStart = 0.0f;  // applyGearTargets anındaki basınç
static PistonPos s_prevTargetPos[4] = {POS_CLOSED, POS_CLOSED, POS_CLOSED, POS_CLOSED};  // Önceki hedef

// Pompa timeout takibi (Faz 9 içi kaçak tespiti)
static uint32_t s_pumpLowPressStartMs = 0;  // Pompanın düşük basınçta çalışmaya başladığı an
static bool     s_pumpTimeoutReported  = false;

// Hareket süresi ölçümü için eşikler
static constexpr uint32_t PISTON_SLOW_OPEN_MS  = 2500;  // Bu üstü "yavaş açma"
static constexpr uint32_t PISTON_SLOW_CLOSE_MS = 2500;  // Bu üstü "yavaş kapama"

// Piston hedef pozisyonları (mevcut vites için)
static PistonPos s_targetPos[4] = {POS_MID, POS_MID, POS_MID, POS_MID};

// Pistonun hedef pozisyonda olup olmadığını kontrol et (geniş tolerans)
static bool isPistonAtTarget(int pistonIdx, PistonPos targetPos) {
    int16_t hall = g_tmagData[PISTON_TMAG[pistonIdx]].z;
    int16_t zMin, zMid, zMax;
    getCalibLimits(pistonIdx, zMin, zMid, zMax);
    
    // Strok mesafesi ve toleransları hesapla (hedef kontrolü için geniş tolerans)
    int16_t stroke = zMax - zMin;
    int16_t tolerance = (int16_t)(stroke * HALL_TARGET_TOL_PCT);       // %10
    
    switch (targetPos) {
        case POS_CLOSED:
            return (hall <= zMin + tolerance);
        case POS_MID: {
            // mm bazlı karşılaştırma: g_pistonHallmm zaten tmagZToMm() ile nonlinear dönüştürülmüş
            // Linear zMid (%50 raw) 13mm'e denk gelmiyor (power law nedeniyle) → doğrudan mm kullan
            float posMm = g_pistonHallmm[pistonIdx];
            float strokeMm = (g_tmagPistonCalib[pistonIdx].valid && g_tmagPistonCalib[pistonIdx].strokeMm > 5.0f)
                             ? g_tmagPistonCalib[pistonIdx].strokeMm : 26.0f;
            float tolMm = strokeMm * HALL_TARGET_TOL_PCT;  // %10 = 2.6mm
            return fabsf(posMm - strokeMm * 0.5f) <= tolMm;
        }
        case POS_OPEN:
            return (hall >= zMax - tolerance);
        default:
            return true;
    }
}

// Pistonun geçerli bir pozisyonda olup olmadığını kontrol et
// Geçiş sırasında min-max arasındaki tüm değerler geçerli
static bool isPistonAtValidPosition(int pistonIdx) {
    int16_t hall = g_tmagData[PISTON_TMAG[pistonIdx]].z;
    int16_t zMin, zMid, zMax;
    getCalibLimits(pistonIdx, zMin, zMid, zMax);
    
    // Geçiş sırasında min-max arasındaki tüm değerler geçerli
    // Sadece aralık dışındaki değerler arıza
    int16_t margin = 2000;  // Sensör toleransı (QC ölçümü ±2000 aşımı kabul)
    if (hall < zMin - margin || hall > zMax + margin) {
        return false;  // Aralık dışı - gerçek arıza
    }
    
    return true;  // min-max arasında - geçerli (geçiş dahil)
}

// Pistonun hangi pozisyonda olduğunu tespit et
static PistonPos detectPistonPosition(int pistonIdx) {
    int16_t hall = g_tmagData[PISTON_TMAG[pistonIdx]].z;
    int16_t zMin, zMid, zMax;
    getCalibLimits(pistonIdx, zMin, zMid, zMax);
    
    int16_t stroke = zMax - zMin;
    int16_t tolerance = (int16_t)(stroke * HALL_TARGET_TOL_PCT);
    
    if (hall <= zMin + tolerance) return POS_CLOSED;
    if (hall >= zMax - tolerance) return POS_OPEN;
    // Orta konum: mm bazlı karşılaştırma (linear zMid yanlış - power law nedeniyle)
    {
        float posMm = g_pistonHallmm[pistonIdx];
        float strokeMm = (g_tmagPistonCalib[pistonIdx].valid && g_tmagPistonCalib[pistonIdx].strokeMm > 5.0f)
                         ? g_tmagPistonCalib[pistonIdx].strokeMm : 26.0f;
        float tolMm = strokeMm * HALL_TARGET_TOL_PCT;  // %10
        if (fabsf(posMm - strokeMm * 0.5f) <= tolMm) return POS_MID;
    }
    
    // Belirsiz pozisyon - en yakın pozisyonu döndür
    int16_t distClosed = abs(hall - zMin);
    int16_t distMid = abs(hall - zMid);
    int16_t distOpen = abs(hall - zMax);
    
    if (distClosed <= distMid && distClosed <= distOpen) return POS_CLOSED;
    if (distOpen <= distMid && distOpen <= distClosed) return POS_OPEN;
    return POS_MID;
}

// Tek piston için bang-bang kontrol (dar tolerans - hedefe daha yakın gitsin)
// Kontrol toleransları hedef toleranslarından daha dar (hassas kontrol için)
static const int16_t MID_CTRL_CLOSED = 100;  // Kontrol için kapalı taraf toleransı (~0.8mm)
static const int16_t MID_CTRL_OPEN = 600;    // Kontrol için açık taraf toleransı (~0.7mm)

static void controlPiston(int pistonIdx, PistonPos targetPos) {
    uint8_t valve = PISTON_VALVE[pistonIdx];
    
    // POS_MID ve POS_OPEN için PD hold kontrolü devrede (TaskValveControl'da)
    // Bang-bang müdahale etmemeli - PD daha düzgün kontrol sağlar
    if (targetPos == POS_MID || targetPos == POS_OPEN) {
        return;  // PD hold halleder
    }
    
    // POS_CLOSED: basit kontrol - valfi kapat
    int16_t hall = g_tmagData[PISTON_TMAG[pistonIdx]].z;
    int16_t zMin, zMid, zMax;
    getCalibLimits(pistonIdx, zMin, zMid, zMax);
    int16_t stroke = zMax - zMin;
    int16_t tolerance = (int16_t)(stroke * HALL_CONTROL_TOL_PCT);
    
    if (hall > zMin + tolerance) {
        g_valveTargetDuty[valve] = PWM_CLOSE;
    } else {
        g_valveTargetDuty[valve] = PWM_MIN;
    }
}

// Piston fail takibi için static değişkenler
static uint32_t s_pistonFailStartMs = 0;
static bool s_pistonFaultReported = false;
static uint8_t s_pistonRetryCount = 0;       // Retry sayacı
static int s_pistonRetryIdx = -1;            // Retry yapılan piston
static bool s_pistonRetryInProgress = false; // Retry devam ediyor mu
static uint32_t s_pistonRetryStartMs = 0;    // Retry başlangıç zamanı
static uint8_t s_pistonRetryPhase = 0;       // 0: kapat, 1: bekle, 2: aç, 3: bekle

// Piston kararlılık takibi (hedefe ulaştıktan sonra kalması gereken süre)
static uint32_t s_pistonStableStartMs[4] = {0, 0, 0, 0};  // Her piston için kararlılık başlangıç zamanı
static bool s_pistonStable[4] = {false, false, false, false};  // Piston kararlı mı

#define PISTON_MAX_RETRY 3      // Maksimum retry sayısı
#define PISTON_RETRY_DELAY 500  // Retry fazları arası bekleme (ms)
#define PISTON_RETRY_PWM_BOOST 50   // Her retry'da PWM artışı (küçük adımlar)
#define PISTON_STABLE_TIME_MS 300   // Pistonun hedepte kalması gereken süre (ms)

// Piston fail takibini sıfırla
static void resetPistonFailTracking() {
    s_pistonFailStartMs = 0;
    s_pistonFaultReported = false;
    s_pistonRetryCount = 0;
    s_pistonRetryIdx = -1;
    s_pistonRetryInProgress = false;
    s_pistonRetryStartMs = 0;
    s_pistonRetryPhase = 0;
    // Kararlılık takibini sıfırla
    for (int i = 0; i < 4; i++) {
        s_pistonStableStartMs[i] = 0;
        s_pistonStable[i] = false;
    }
}

// Pistonun anlık durumuna göre hata detayını belirle
// (invalid range / not-opened / not-closed / not-held-mid)
static uint8_t computePistonFaultDetail(int pistonIdx) {
    if (pistonIdx < 0 || pistonIdx >= 4) return PFD_NONE;
    if (!isPistonAtValidPosition(pistonIdx)) return PFD_OUT_OF_RANGE;
    switch (s_targetPos[pistonIdx]) {
        case POS_OPEN:   return PFD_NOT_OPENED;
        case POS_CLOSED: return PFD_NOT_CLOSED;
        case POS_MID:    return PFD_NOT_HELD_MID;
        default:         return PFD_NONE;
    }
}

// Hata geçmişine kayıt ekle
static void recordError(int pistonIdx, uint16_t faultType, uint8_t faultDetail = PFD_NONE) {
    // Hatalı vites bitmask'ını güncelle (her vites sadece bir kez işaretlenir)
    g_autoShiftV2Errors.faultyGearsMask |= (1 << (uint8_t)s_targetGear);
    
    if (g_autoShiftV2Errors.count >= MAX_ERROR_HISTORY) return;
    
    int16_t zMin, zMid, zMax;
    getCalibLimits(pistonIdx, zMin, zMid, zMax);
    
    AutoShiftV2ErrorEntry& entry = g_autoShiftV2Errors.entries[g_autoShiftV2Errors.count];
    entry.gear = s_targetGear;
    entry.pistonIdx = pistonIdx;
    entry.repeatIdx = s_repeatIdx;
    entry.faultType = faultType;
    entry.timestampMs = g_autoShiftV2Pub.elapsedMs;
    entry.hallValue = g_tmagData[PISTON_TMAG[pistonIdx]].z;
    entry.expectedMin = zMin;
    entry.expectedMax = zMax;
    entry.expectedPos = (pistonIdx < 4) ? s_targetPos[pistonIdx] : 0;  // Beklenen pozisyon
    entry.faultDetail = (faultDetail != PFD_NONE) ? faultDetail
                                                  : computePistonFaultDetail(pistonIdx);
    entry.valveIdx    = (pistonIdx >= 0 && pistonIdx < 4) ? PISTON_VALVE[pistonIdx] : 0xFF;
    
    g_autoShiftV2Errors.count++;
}
// Hata nedeniyle sonraki vitese atla
static bool s_skipToNextGear = false;

// Tüm pistonların hedef pozisyonda olup olmadığını kontrol et
// NOT: Retry kaldırıldı - PistonMonitor hataları izliyor
static bool allPistonsAtTarget() {
    static uint32_t lastFailLog = 0;
    static uint32_t lastInvalidLog = 0;
    bool allOk = true;
    bool anyInvalid = false;
    int failIdx = -1;
    bool invalidPosDetected = false;
    
    // Önce geçersiz pozisyon kontrolü yap
    // Piston açık, kapalı veya ortada değilse mekatronik arızası var
    for (int i = 0; i < 4; i++) {
        if (!isPistonAtValidPosition(i)) {
            invalidPosDetected = true;
            failIdx = i;
            
            // Her 2 saniyede bir geçersiz pozisyon logla
            if ((millis() - lastInvalidLog > 2000)) {
                lastInvalidLog = millis();
                int16_t hall = g_tmagData[PISTON_TMAG[i]].z;
                int16_t zMin, zMid, zMax;
                getCalibLimits(i, zMin, zMid, zMax);
                char msg[120];
                snprintf(msg, sizeof(msg), "[WARN] P%d gecersiz pozisyon! hall=%d (min=%d mid=%d max=%d)", 
                         i, hall, zMin, zMid, zMax);
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
            }
            break;
        }
    }
    
    // Her piston için hedef kontrolü
    // ORTA konum için kararlılık kontrolü ekle - piston PISTON_STABLE_TIME_MS kadar hedefte kalmalı
    for (int i = 0; i < 4; i++) {
        bool atTarget = isPistonAtTarget(i, s_targetPos[i]);
        
        if (s_targetPos[i] == POS_MID) {
            // Orta konum için kararlılık kontrolü
            if (atTarget) {
                // Hedepte - kararlılık süresini başlat veya devam et
                if (s_pistonStableStartMs[i] == 0) {
                    s_pistonStableStartMs[i] = millis();
                }
                // Yeterince süre hedefte kaldı mı?
                if (millis() - s_pistonStableStartMs[i] < PISTON_STABLE_TIME_MS) {
                    // Henüz kararlı değil - hedepte sayılmaz
                    allOk = false;
                    if (failIdx < 0) failIdx = i;
                } else {
                    s_pistonStable[i] = true;
                }
            } else {
                // Hedeften çıktı - kararlılık süresini sıfırla
                s_pistonStableStartMs[i] = 0;
                s_pistonStable[i] = false;
                allOk = false;
                if (failIdx < 0) failIdx = i;
            }
        } else {
            // Açık/Kapalı için basit kontrol
            if (!atTarget) {
                allOk = false;
                if (failIdx < 0) failIdx = i;
            }
        }
    }
    
    if (!allOk) {
        // İlk fail anını kaydet
        if (s_pistonFailStartMs == 0) {
            s_pistonFailStartMs = millis();
            s_pistonFaultReported = false;
        }
        
        // Kalibre edilmiş timeout kullan (varsayılan 3000ms)
        // NOT: Retry kaldırıldı - PistonMonitor hataları izliyor
        uint16_t pistonTimeout = 3000;  // Sabit 3sn timeout (eski AutoCalibration_GetTimeout)
        if (!s_pistonFaultReported && (millis() - s_pistonFailStartMs > pistonTimeout)) {
            // Hata bildir ve sonraki vitese geç (retry yok)
            // NOT: (1 << failIdx) kaldırıldı - pressure fault bitleriyle çakışıyordu
            // Piston hataları pistonState[]=0xFF ile takip ediliyor
            g_autoShiftV2Pub.faults |= FAULT_PISTON_UNEXPECTED;
            s_pistonFaultReported = true;
            
            // Hatayı geçmişe kaydet
            recordError(failIdx, FAULT_PISTON_UNEXPECTED);
            g_autoShiftV2Errors.skippedGears++;
            
            {
                char msg[100];
                snprintf(msg, sizeof(msg), "[SKIP] Piston %d timeout - sonraki vitese atlaniyor", failIdx);
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
            }
            
            // Sonraki vitese atla
            s_skipToNextGear = true;
            resetPistonFailTracking();
        }
    } else {
        // Tüm pistonlar hedefte - sayacı sıfırla
        s_pistonFailStartMs = 0;
        s_pistonFaultReported = false;
        s_pistonRetryCount = 0;  // Başarılı olunca retry sayacını sıfırla
    }
    
    return allOk;
}

// Piston retry işlemini yürüt
// Pistonu kapatıp açarak kurtarmaya çalışır
static void processPistonRetry() {
    if (!s_pistonRetryInProgress || s_pistonRetryIdx < 0) return;
    
    uint32_t elapsed = millis() - s_pistonRetryStartMs;
    uint8_t valve = PISTON_VALVE[s_pistonRetryIdx];
    
    switch (s_pistonRetryPhase) {
        case 0: // Kapat
            g_valveTargetDuty[valve] = PWM_CLOSE;
            if (elapsed > PISTON_RETRY_DELAY) {
                s_pistonRetryPhase = 1;
                s_pistonRetryStartMs = millis();
            }
            break;
            
        case 1: // Bekle
            g_valveTargetDuty[valve] = getCalibHoldDuty(s_pistonRetryIdx);
            if (elapsed > PISTON_RETRY_DELAY) {
                s_pistonRetryPhase = 2;
                s_pistonRetryStartMs = millis();
            }
            break;
            
        case 2: // Hedef pozisyona git
            {
                uint16_t targetPwm = getCalibHoldDuty(s_pistonRetryIdx);
                if (s_targetPos[s_pistonRetryIdx] == POS_OPEN) {
                    targetPwm = PWM_OPEN;
                } else if (s_targetPos[s_pistonRetryIdx] == POS_CLOSED) {
                    targetPwm = PWM_CLOSE;
                }
                g_valveTargetDuty[valve] = targetPwm;
                
                if (elapsed > PISTON_RETRY_DELAY * 2) {
                    s_pistonRetryPhase = 3;
                    s_pistonRetryStartMs = millis();
                }
            }
            break;
            
        case 3: // Kontrol et
            {
                char msg[60];
                snprintf(msg, sizeof(msg), "[RETRY] Piston %d deneme %d tamamlandi", 
                         s_pistonRetryIdx, s_pistonRetryCount);
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
            }
            s_pistonRetryInProgress = false;
            s_pistonFailStartMs = millis();
            break;
    }
}

// Vites pozisyonunu uygula
static void applyGearTargets(GearState gear) {
    if (gear >= GEAR_COUNT) return;

    // Vites geçişi başlangıcında basınç ve önceki hedefleri kaydet
    s_gearPressureAtStart = readPressure();
    for (int i = 0; i < 4; i++) s_prevTargetPos[i] = s_targetPos[i];

    const GearPosition& pos = GEAR_POSITIONS[gear];
    s_targetPos[0] = pos.p5_7;
    s_targetPos[1] = pos.p1_3;
    s_targetPos[2] = pos.p2_4;
    s_targetPos[3] = pos.p6_R;
    
    // PD hold kontrolü aktive et + başlangıç PWM ayarla
    // setPistonValve Convention B sırası kullanır (0=P1-3, 1=P5-7)
    // s_targetPos Convention A sırası kullanır (0=P5-7, 1=P1-3)
    setPistonValve(0, pos.p1_3);
    setPistonValve(1, pos.p5_7);
    setPistonValve(2, pos.p2_4);
    setPistonValve(3, pos.p6_R);
    
    // Piston izleyiciye hedefleri bildir (sıralama: P5-7, P1-3, P2-4, P6-R)
    // Kavrama kontrolü
    setClutch(pos.clutch);
}

// Sıralı testte bir sonraki vitese geç
// Akış: N → D1 → D2 → D3 → D4 → D5 → D6 → D7 → D6 → D5 → D4 → D3 → D2 → D1 → N (tekrar)
// Bir döngü N'ye döndüğünde tamamlanır ve s_repeatIdx artar
static GearState getNextGear(GearState current) {
    if (s_gearDirectionUp) {
        // Yukarı: N → D1 → D2 → D3 → D4 → D5 → D6 → D7
        switch (current) {
            case GEAR_N:  return GEAR_D1;
            case GEAR_D1: return GEAR_D2;
            case GEAR_D2: return GEAR_D3;
            case GEAR_D3: return GEAR_D4;
            case GEAR_D4: return GEAR_D5;
            case GEAR_D5: return GEAR_D6;
            case GEAR_D6: return GEAR_D7;
            case GEAR_D7: 
                s_gearDirectionUp = false;  // D7'ye ulaştı, şimdi aşağı in
                return GEAR_D6;
            default:      return GEAR_D1;
        }
    } else {
        // Aşağı: D7 → D6 → D5 → D4 → D3 → D2 → D1 → N
        switch (current) {
            case GEAR_D7: return GEAR_D6;
            case GEAR_D6: return GEAR_D5;
            case GEAR_D5: return GEAR_D4;
            case GEAR_D4: return GEAR_D3;
            case GEAR_D3: return GEAR_D2;
            case GEAR_D2: return GEAR_D1;
            case GEAR_D1: 
                // N'ye döndüğünde bir döngü tamamlandı
                s_repeatIdx++;
                s_gearDirectionUp = true;
                
                // Tekrar sayısına ulaşıldı mı kontrol et
                if (s_repeatIdx >= g_autoShiftV2Cfg.repeatCount) {
                    s_testCompleted = true;
                    {
                        char msg[64];
                        snprintf(msg, sizeof(msg), "[AutoV2] Test tamamlandi: %u tekrar", s_repeatIdx);
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                    }
                }
                return GEAR_N;
            case GEAR_N:
                // N'deyken ve test tamamlanmadıysa devam et
                if (!s_testCompleted) {
                    s_gearDirectionUp = true;
                    return GEAR_D1;
                }
                return GEAR_N;  // Test bitti, N'de kal
            default:      return GEAR_N;
        }
    }
}

// ============================================================================
// Hızlı Stroke Kontrolü (test başlamadan önce)
// Her piston kapalı→açık hareket eder, hall farkı yetersizse ARIZALI işaretler
// ============================================================================
static const char* QC_VALVE_NAMES[4] = {"N434", "N433", "N437", "N438"};
static const uint16_t QC_OPEN_PWM     = 1500;   // Piston açma PWM
static const uint16_t QC_MIN_STROKE   = 500;    // Minimum hall farkı (arıza eşiği)
static const uint32_t QC_MOVE_WAIT_MS = 700;    // Hareket bekleme süresi (ms)
static const uint32_t QC_PRESSURE_WAIT_MS = 8000; // Basınç bekleme timeout
static const float QC_MIN_PRESSURE_BAR = 35.0f; // Test için minimum basınç

static uint8_t  s_qcState        = 0;
static uint8_t  s_qcPiston       = 0;
static uint32_t s_qcStateMs      = 0;
static int16_t  s_qcHallClosed[4]= {0, 0, 0, 0};
static int16_t  s_qcHallOpen[4]  = {0, 0, 0, 0};
static bool     s_qcFault[4]     = {false, false, false, false};
static bool     s_quickCalibDone = false;

static bool runQuickCalib() {
    enum QCS : uint8_t {
        QCS_START=0, QCS_WAIT_PRESSURE, QCS_CLOSE_ALL, QCS_WAIT_CLOSE,
        QCS_SAVE_CLOSED, QCS_OPEN_P, QCS_WAIT_OPEN, QCS_SAVE_OPEN,
        QCS_CLOSE_P, QCS_WAIT_CLOSE2, QCS_NEXT_P, QCS_SEND, QCS_DONE
    };
    uint32_t now = millis();

    switch (s_qcState) {
        case QCS_START:
            startPumpAuto();
            openMainValves();
            s_qcStateMs = now;
            s_qcState = QCS_WAIT_PRESSURE;
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[QC] Hizli stroke kontrol basliyor...");
            break;

        case QCS_WAIT_PRESSURE:
            if (readPressure() >= QC_MIN_PRESSURE_BAR || now - s_qcStateMs > QC_PRESSURE_WAIT_MS) {
                s_qcState = QCS_CLOSE_ALL;
            }
            break;

        case QCS_CLOSE_ALL:
            for (int i = 0; i < 4; i++) g_valveTargetDuty[PISTON_VALVE[i]] = 0;
            s_qcStateMs = now;
            s_qcState = QCS_WAIT_CLOSE;
            break;

        case QCS_WAIT_CLOSE:
            if (now - s_qcStateMs >= QC_MOVE_WAIT_MS) {
                s_qcState = QCS_SAVE_CLOSED;
            }
            break;

        case QCS_SAVE_CLOSED:
            for (int i = 0; i < 4; i++) s_qcHallClosed[i] = g_tmagData[PISTON_TMAG[i]].z;
            s_qcPiston = 0;
            s_qcState = QCS_OPEN_P;
            break;

        case QCS_OPEN_P:
            g_valveTargetDuty[PISTON_VALVE[s_qcPiston]] = QC_OPEN_PWM;
            s_qcStateMs = now;
            s_qcState = QCS_WAIT_OPEN;
            break;

        case QCS_WAIT_OPEN:
            if (now - s_qcStateMs >= QC_MOVE_WAIT_MS) s_qcState = QCS_SAVE_OPEN;
            break;

        case QCS_SAVE_OPEN: {
            s_qcHallOpen[s_qcPiston] = g_tmagData[PISTON_TMAG[s_qcPiston]].z;
            int16_t diff = (int16_t)abs((int)s_qcHallOpen[s_qcPiston] - (int)s_qcHallClosed[s_qcPiston]);
            s_qcFault[s_qcPiston] = (diff < (int16_t)QC_MIN_STROKE);

            // Geçerli ölçümü g_pistonCalibData'ya yaz (yalnızca mevcut kalibrasyon yoksa)
            if (!s_qcFault[s_qcPiston] &&
                !g_pistonCalibData[s_qcPiston].calibrated &&
                g_sharedMutex &&
                xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                int16_t zMin = (s_qcHallClosed[s_qcPiston] < s_qcHallOpen[s_qcPiston])
                               ? s_qcHallClosed[s_qcPiston] : s_qcHallOpen[s_qcPiston];
                int16_t zMax = (s_qcHallClosed[s_qcPiston] > s_qcHallOpen[s_qcPiston])
                               ? s_qcHallClosed[s_qcPiston] : s_qcHallOpen[s_qcPiston];
                g_pistonCalibData[s_qcPiston].min_raw  = (uint16_t)zMin;
                g_pistonCalibData[s_qcPiston].max_raw  = (uint16_t)zMax;
                g_pistonCalibData[s_qcPiston].mid_raw  = (uint16_t)((zMin + zMax) / 2);
                if (g_pistonCalibData[s_qcPiston].duty_hold < 500) {
                    g_pistonCalibData[s_qcPiston].duty_hold = PWM_HOLD; // Varsayılan hold PWM
                }
                g_pistonCalibData[s_qcPiston].calibrated = true;
                xSemaphoreGive(g_sharedMutex);
            }

            {
                char msg[96];
                snprintf(msg, sizeof(msg), "[QC] p%d(%s) kapali=%d acik=%d fark=%d %s",
                         s_qcPiston, QC_VALVE_NAMES[s_qcPiston],
                         s_qcHallClosed[s_qcPiston], s_qcHallOpen[s_qcPiston],
                         diff, s_qcFault[s_qcPiston] ? "ARIZALI" : "OK");
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
            }
            s_qcState = QCS_CLOSE_P;
            break;
        }

        case QCS_CLOSE_P:
            g_valveTargetDuty[PISTON_VALVE[s_qcPiston]] = 0;
            s_qcStateMs = now;
            s_qcState = QCS_WAIT_CLOSE2;
            break;

        case QCS_WAIT_CLOSE2:
            if (now - s_qcStateMs >= QC_MOVE_WAIT_MS) s_qcState = QCS_NEXT_P;
            break;

        case QCS_NEXT_P:
            s_qcPiston++;
            s_qcState = (s_qcPiston < 4) ? QCS_OPEN_P : QCS_SEND;
            break;

        case QCS_SEND: {
            int16_t d0 = (int16_t)abs((int)s_qcHallOpen[0] - (int)s_qcHallClosed[0]);
            int16_t d1 = (int16_t)abs((int)s_qcHallOpen[1] - (int)s_qcHallClosed[1]);
            int16_t d2 = (int16_t)abs((int)s_qcHallOpen[2] - (int)s_qcHallClosed[2]);
            int16_t d3 = (int16_t)abs((int)s_qcHallOpen[3] - (int)s_qcHallClosed[3]);
            char buf[220];
            snprintf(buf, sizeof(buf),
                "{\"_t\":\"QC\",\"results\":["
                "{\"p\":0,\"v\":\"%s\",\"ok\":%s,\"diff\":%d},"
                "{\"p\":1,\"v\":\"%s\",\"ok\":%s,\"diff\":%d},"
                "{\"p\":2,\"v\":\"%s\",\"ok\":%s,\"diff\":%d},"
                "{\"p\":3,\"v\":\"%s\",\"ok\":%s,\"diff\":%d}]}",
                QC_VALVE_NAMES[0], s_qcFault[0] ? "false":"true", d0,
                QC_VALVE_NAMES[1], s_qcFault[1] ? "false":"true", d1,
                QC_VALVE_NAMES[2], s_qcFault[2] ? "false":"true", d2,
                QC_VALVE_NAMES[3], s_qcFault[3] ? "false":"true", d3);
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
            s_qcState = QCS_DONE;
            break;
        }

        case QCS_DONE:
        default:
            return true;
    }
    return false;
}

static void userTestFunction() {
    // Önce hızlı stroke kontrolü yap
    // NOT: Phase 7/8 kalibrasyonu yapıldıysa QC atla (zaten kalibre edildi)
    if (!s_quickCalibDone) {
        bool allCalibrated = true;
        for (int i = 0; i < 4; i++) {
            if (!g_pistonCalibData[i].calibrated) { allCalibrated = false; break; }
        }
        if (allCalibrated) {
            s_quickCalibDone = true;
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, "[AutoV2] Phase 7/8 kalibrasyonu mevcut - QC atlandi");
        } else if (runQuickCalib()) {
            s_quickCalibDone = true;
        } else {
            return;
        }
    }

    // İlk çağrıda sistemi başlat
    if (!s_systemInitialized) {
        s_systemInitialized = true;
        s_gearReached = false;
        s_gearChangeStartMs = millis();
        s_repeatIdx = 0;           // Tekrar sayacını sıfırla
        s_testCompleted = false;   // Test tamamlandı flag'ini sıfırla
        resetPistonFailTracking(); // Piston fail takibini sıfırla
        // Per-piston istatistik ve pompa timeout sıfırla
        memset(g_pistonStats, 0, sizeof(g_pistonStats));
        s_gearPressureAtStart = 0.0f;
        for (int i = 0; i < 4; i++) s_prevTargetPos[i] = POS_CLOSED;
        s_pumpLowPressStartMs = 0;
        s_pumpTimeoutReported = false;
        g_leakRecheckNeeded   = false;
        
        // Pompayı başlat
        startPumpAuto();
        
        // Ana basınç valflerini aç
        openMainValves();
        
        // Test modunu belirle
        if (g_autoShiftV2Cfg.manualMode) {
            s_testMode = MODE_MANUAL_GEAR;
            s_targetGear = g_autoShiftV2Cfg.targetGear;
        } else if (g_autoShiftV2Cfg.repeatCount > 0) {
            s_testMode = MODE_SEQUENTIAL_TEST;
            s_targetGear = GEAR_D1;
            s_gearDirectionUp = true;  // Başlangıçta yukarı yönde
        } else {
            s_testMode = MODE_HOLD_MID;
            s_targetGear = GEAR_N;
        }
        
        // Hedef pozisyonları ayarla
        applyGearTargets(s_targetGear);
        
        {
            char msg[64];
            snprintf(msg, sizeof(msg), "[GEAR] Mod=%d Hedef=%s", 
                     s_testMode, GearStateToString(s_targetGear));
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        }
    }
    
    // Manuel modda GUI'den gelen hedef vitesi kontrol et
    if (s_testMode == MODE_MANUAL_GEAR) {
        GearState guiTarget = g_autoShiftV2Req.targetGear;
        if (guiTarget != s_targetGear && guiTarget < GEAR_COUNT) {
            s_targetGear = guiTarget;
            s_gearReached = false;
            s_gearChangeStartMs = millis();
            applyGearTargets(s_targetGear);
            
            {
                char msg[64];
                snprintf(msg, sizeof(msg), "[GEAR] Yeni hedef: %s", 
                         GearStateToString(s_targetGear));
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
            }
        }
    }
    
    // Her piston için bang-bang kontrol uygula
    for (int i = 0; i < 4; i++) {
        controlPiston(i, s_targetPos[i]);
    }
    
    // Tüm pistonlar hedefe ulaştı mı kontrol et
    bool allReached = allPistonsAtTarget();
    
    // Hata nedeniyle sonraki vitese atla (DSG mantığı: başarısız vitesi atla, sonrakine git)
    if (s_skipToNextGear && s_testMode == MODE_SEQUENTIAL_TEST && !s_testCompleted) {
        s_skipToNextGear = false;
        GearState failedGear = s_targetGear;  // Başarısız olan vites
        s_targetGear = getNextGear(s_targetGear);  // Başarısız vitesten sonrakine atla
        s_gearReached = false;
        s_gearChangeStartMs = millis();
        applyGearTargets(s_targetGear);
        
        {
            char msg[100];
            snprintf(msg, sizeof(msg), "[SKIP] %s arızalı, atlandi -> %s", 
                     GearStateToString(failedGear), GearStateToString(s_targetGear));
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        }
        return;  // Bu döngüde daha fazla işlem yapma
    }
    
    if (allReached && !s_gearReached) {
        s_gearReached = true;
        s_gearHoldStartMs = millis();
        s_currentGear = s_targetGear;
        
        // Per-piston istatistik güncelle
        uint32_t moveMs    = millis() - s_gearChangeStartMs;
        float    pressNow  = readPressure();
        float    pressDrop = (s_gearPressureAtStart > pressNow) ? (s_gearPressureAtStart - pressNow) : 0.0f;

        for (int i = 0; i < 4; i++) {
            // Sadece hedef değişen pistonlar için kayıt yap
            if (s_targetPos[i] == s_prevTargetPos[i]) continue;

            PerPistonStats& st = g_pistonStats[i];
            st.totalMoves++;

            bool isOpen  = (s_targetPos[i] == POS_OPEN  || s_targetPos[i] == POS_MID);
            bool isClose = (s_targetPos[i] == POS_CLOSED);

            if (isOpen) {
                st.openMoves++;
                st.totalOpenMs    += moveMs;
                st.totalPressDropOpen += pressDrop;
                if (moveMs  > st.maxOpenMs)       st.maxOpenMs       = moveMs;
                if (pressDrop > st.maxPressDropOpen)  st.maxPressDropOpen  = pressDrop;
                if (moveMs  > PISTON_SLOW_OPEN_MS) st.slowOpenCount++;
            } else if (isClose) {
                st.closeMoves++;
                st.totalCloseMs    += moveMs;
                st.totalPressDropClose += pressDrop;
                if (moveMs    > st.maxCloseMs)        st.maxCloseMs        = moveMs;
                if (pressDrop > st.maxPressDropClose) st.maxPressDropClose = pressDrop;
                if (moveMs    > PISTON_SLOW_CLOSE_MS) st.slowCloseCount++;
            }
        }

        // Publish güncelle
        g_autoShiftV2Pub.currentGear = s_currentGear;
        g_autoShiftV2Pub.targetGear = s_targetGear;
        
        {
            char msg[64];
            snprintf(msg, sizeof(msg), "[GEAR] %s konumuna ulasti (%lu ms, dP=%.1f bar)", 
                     GearStateToString(s_currentGear), 
                     (unsigned long)moveMs, pressDrop);
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        }
    }
    
    // Sıralı testte: belirli süre bekle, sonra sonraki vitese geç
    if (s_testMode == MODE_SEQUENTIAL_TEST && s_gearReached && !s_testCompleted) {
        uint32_t holdMs = g_autoShiftV2Cfg.gearHoldMs;
        if (holdMs < 1000) holdMs = 2000;  // Minimum 2 saniye
        
        if (millis() - s_gearHoldStartMs >= holdMs) {
            s_targetGear = getNextGear(s_currentGear);
            s_gearReached = false;
            s_gearChangeStartMs = millis();
            applyGearTargets(s_targetGear);
            
            {
                char msg[64];
                snprintf(msg, sizeof(msg), "[GEAR] Sonraki vites: %s (tekrar %u/%u)", 
                         GearStateToString(s_targetGear), s_repeatIdx, g_autoShiftV2Cfg.repeatCount);
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
            }
        }
    }
    
    // Test tamamlandıysa running flag'ini kapat (GUI'ye bildir)
    static bool s_completedLogged = false;
    if (s_testCompleted && s_testMode == MODE_SEQUENTIAL_TEST) {
        // Sadece bir kez ayarla
        if (!s_completedLogged) {
            s_completedLogged = true;
            g_autoShiftV2Pub.running = false;
            g_autoShiftV2Pub.phase = PHASE_COMPLETED;
            snprintf(g_autoShiftV2Pub.statusText, sizeof(g_autoShiftV2Pub.statusText), "Tamamlandi");
            
            {
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, "[AutoV2] PHASE_COMPLETED ayarlandi - GUI rapor gostermeli");
            }
        }
    } else {
        // Test yeniden başladığında flag'i sıfırla
        s_completedLogged = false;
    }
    
    // Publish güncelle
    g_autoShiftV2Pub.targetGear = s_targetGear;
    g_autoShiftV2Pub.repeatIdx = s_repeatIdx;  // Tekrar sayacını GUI'ye gönder
    
    // Piston durumlarını güncelle (0xFF sadece 5 saniye sonra - hata onaylandığında)
    for (int i = 0; i < 4; i++) {
        bool atTarget = isPistonAtTarget(i, s_targetPos[i]);
        if (atTarget) {
            g_autoShiftV2Pub.pistonState[i] = s_targetPos[i];
        } else if (s_pistonFaultReported) {
            // 5 saniye geçti, hata onaylandı
            g_autoShiftV2Pub.pistonState[i] = 0xFF;
        }
        // else: Henüz 5 saniye geçmedi, önceki değeri koru
    }
    
    // Pompa timeout izleme — basınç düşükken pompa uzun süre çalışıyorsa kaçak şüphesi
    if (s_testMode == MODE_SEQUENTIAL_TEST && !s_pumpTimeoutReported) {
        const float  MIN_PRESS_BAR    = 40.0f;
        const uint32_t PUMP_FILL_MAX_MS = 15000;  // 15 saniyeden fazla dolduramazsa → kaçak
        float curPress = readPressure();
        bool pumpOn = (g_pumpCmd.cmd == PUMP_CMD_AUTO);

        if (pumpOn && curPress < MIN_PRESS_BAR) {
            if (s_pumpLowPressStartMs == 0) s_pumpLowPressStartMs = millis();
            if (millis() - s_pumpLowPressStartMs > PUMP_FILL_MAX_MS) {
                s_pumpTimeoutReported = true;
                g_autoShiftV2Pub.faults |= FAULT_PUMP_TIMEOUT;
                g_leakRecheckNeeded = true;  // TaskAutoTest'e Faz 10 tetikle
                s_testCompleted = true;       // Testi durdur
                g_autoShiftV2Pub.running = false;
                g_autoShiftV2Pub.phase = PHASE_ERROR;
                snprintf(g_autoShiftV2Pub.statusText, sizeof(g_autoShiftV2Pub.statusText), "KACAK SUPE!");
                {
                    char msg[80];
                    snprintf(msg, sizeof(msg),
                             "[PUMP] Timeout: %lu ms boyunca %.1f bar alti! Kacak test tetiklendi.",
                             (unsigned long)(millis() - s_pumpLowPressStartMs), curPress);
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                }
            }
        } else {
            s_pumpLowPressStartMs = 0;  // Basınç normale döndüyse sayacı sıfırla
        }
    }

    // Debug: Her saniye hall değerlerini ve hedefleri göster
    static uint32_t lastDebugMs = 0;
    if (millis() - lastDebugMs > 1000) {
        lastDebugMs = millis();
        {
            char dbg[140];
            snprintf(dbg, sizeof(dbg), "[POS] H0=%d(%c) H1=%d(%c) H2=%d(%c) H3=%d(%c) G=%s", 
                     g_tmagData[PISTON_TMAG[0]].z, "CMO"[s_targetPos[0]],
                     g_tmagData[PISTON_TMAG[1]].z, "CMO"[s_targetPos[1]],
                     g_tmagData[PISTON_TMAG[2]].z, "CMO"[s_targetPos[2]],
                     g_tmagData[PISTON_TMAG[3]].z, "CMO"[s_targetPos[3]],
                     GearStateToString(s_targetGear));
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, dbg);
        }
    }
}

// ============================================================================
// Ana Task - Basitleştirilmiş versiyon
// ============================================================================
void TaskAutoShiftV2(void *pvParameters) {
    (void)pvParameters;
    
    static uint32_t lastReqSeq = 0;
    static bool running = false;
    static bool initialized = false;
    
    // Başlangıç durumu
    g_autoShiftV2Pub.running = false;
    g_autoShiftV2Pub.phase = PHASE_IDLE;
    g_autoShiftV2Pub.faults = FAULT_NONE;
    
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(20));  // 50Hz kontrol döngüsü
        
        // İstek kontrolü
        if (g_autoShiftV2ReqSeq != lastReqSeq) {
            lastReqSeq = g_autoShiftV2ReqSeq;
            
            if (g_autoShiftV2Req.start && !running) {
                // BAŞLA komutu geldi - Request'teki değerleri Config'e kopyala
                g_autoShiftV2Cfg.manualMode = g_autoShiftV2Req.manualMode;
                g_autoShiftV2Cfg.targetGear = g_autoShiftV2Req.targetGear;
                g_autoShiftV2Cfg.gearHoldMs = g_autoShiftV2Req.gearHoldMs;
                g_autoShiftV2Cfg.repeatCount = g_autoShiftV2Req.repeatCount;
                
                running = true;
                initialized = false;
                g_autoShiftV2Pub.running = true;
                g_autoShiftV2Pub.phase = PHASE_INIT;
                g_autoShiftV2Pub.faults = FAULT_NONE;  // Hataları sıfırla
                
                // Hata geçmişini sıfırla
                memset(&g_autoShiftV2Errors, 0, sizeof(g_autoShiftV2Errors));
                s_skipToNextGear = false;
                
                // Kalibrasyon durumunu logla
                if (g_pistonCalibData[0].calibrated) {
                    {
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, "[AutoV2] Phase 7/8 kalibrasyonu kullaniliyor");
                    }
                } else {
                    {
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, "[AutoV2] UYARI: Kalibrasyon yapilmamis - varsayilan degerler kullanilacak");
                    }
                }
                
                snprintf(g_autoShiftV2Pub.statusText, sizeof(g_autoShiftV2Pub.statusText), "Test basliyor...");
                
                {
                    char msg[80];
                    snprintf(msg, sizeof(msg), "[AutoV2] BASLA: manuel=%d, hedef=%d, hold=%u, tekrar=%u",
                             g_autoShiftV2Cfg.manualMode, g_autoShiftV2Cfg.targetGear,
                             g_autoShiftV2Cfg.gearHoldMs, g_autoShiftV2Cfg.repeatCount);
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                }
                
            } else if (!g_autoShiftV2Req.start && running) {
                // DURDUR komutu geldi
                running = false;
                initialized = false;
                s_systemInitialized = false;  // Test durumunu sıfırla
                s_quickCalibDone = false;     // Bir sonraki test için QC sıfırla
                s_qcState = 0;
                s_qcPiston = 0;
                
                // Temizlik
                disableAllPistonHold();
                closeAllValves();
                closeMainValves();
                stopPump();
                
                g_autoShiftV2Pub.running = false;
                // Test tamamlandıysa PHASE_COMPLETED'ı koru, aksi halde PHASE_IDLE yap
                if (g_autoShiftV2Pub.phase != PHASE_COMPLETED) {
                    g_autoShiftV2Pub.phase = PHASE_IDLE;
                    snprintf(g_autoShiftV2Pub.statusText, sizeof(g_autoShiftV2Pub.statusText), "Durduruldu");
                }
                
                {
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, "[AutoV2] DURDUR komutu alindi");
                }
            }
        }
        
        // Basınç okuma
        float pressure = readPressure();
        g_autoShiftV2Pub.pressure = pressure;
        
        // Piston durumlarını güncelle
        for (uint8_t i = 0; i < 4; i++) {
            g_autoShiftV2Pub.pistonState[i] = determinePistonState(i);
        }
        
        // Çalışıyorsa kullanıcı fonksiyonunu çağır
        if (running) {
            // İlk başlatma (bir kez)
            if (!initialized) {
                initialized = true;
                resetPistonHoldState();
                resetPistonFailTracking();  // Retry sayacını sıfırla
                g_autoShiftV2Pub.phase = PHASE_GEAR_ACTIVE;
                snprintf(g_autoShiftV2Pub.statusText, sizeof(g_autoShiftV2Pub.statusText), "Calisiyor...");
            }
            
            // NOT: Eski retry mantığı kaldırıldı - PistonMonitor hata izliyor
            // processPistonRetry();
            
            // Pre-position fazindaki pistonlari surekli guncelle (~50Hz)
            updatePistonPrePos();

            // Kullanıcı test fonksiyonunu çağır
            userTestFunction();
            
            // Test durdurulduysa (tamamlandı veya hata) temizlik yap
            if (!g_autoShiftV2Pub.running && running) {
                running = false;
                initialized = false;
                s_systemInitialized = false;
                s_quickCalibDone = false;     // Bir sonraki test için QC sıfırla
                s_qcState = 0;
                s_qcPiston = 0;
                
                disableAllPistonHold();
                closeAllValves();
                closeMainValves();
                stopPump();
                
                snprintf(g_autoShiftV2Pub.statusText, sizeof(g_autoShiftV2Pub.statusText), "Test bitti");
            }
        }
    }
}

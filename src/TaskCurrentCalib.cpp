// TaskCurrentCalib.cpp — Piston Kalibrasyon Gorevi
//
// Faz sirasi:
//   P0-P3: Pompa(0) → Stroke(1) → AcilmaEsigi(2) → KapanmaEsigi(3) → Hold(4) → Kaydet(5)
//   K1/K2: Pompa(0) → Stroke(1) → AcilmaEsigi(2) → KapanmaEsigi(3) → Kaydet(4)
//
// Temel duzeltme: ccSetCurrent artik mA degerini gercek PWM duty'ye cevirir.
//   duty = (mA/1000 * R_COIL / V_SUPPLY) * 4095
//   Onceki kod her non-zero mA icin sabit 2000 yaziyordu — esik arama calismiyordu.
//
// Hareket tespiti: her adimda onceki adima gore delta karsilastirir (ilk ref degil).
//
// JSON Protokol:
//   Baslat (tek): {"current_calib":{"piston":0}}
//   Baslat (hep): {"current_calib":{"all":true}}
//   Durdur:       {"current_calib":{"abort":true}}
// Ilerleme: {"_t":"CC","p":N,"phase":0-5,"prog":0-100,"mm":X.X,"mA":X}
// Sonuc:    {"_t":"CC","p":N,"done":true,"ok":true,"min_mm":X,"max_mm":X,
//            "hold_mA":X,"open_mA":X,"close_mA":X}

#include <Arduino.h>
#include "Tasks.h"
#include "Shared.h"
#include "PistonCalib.h"

// --- TMAG kanal esleme (Convention A) ---
// P0=5/7, P1=1/3, P2=2/4, P3=6/R  (K1/K2 hall raw saklamiyor)
static const uint8_t CC_PISTON_TMAG[4] = {
    TMAG_CH_5_7, TMAG_CH_1_3, TMAG_CH_2_4, TMAG_CH_6_R
};

// Belirli piston icin anlik hall ham (z) degerini oku
static int16_t ccReadHallZ(uint8_t p) {
    if (p >= 4) return 0;
    int16_t z = 0;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        z = g_tmagData[CC_PISTON_TMAG[p]].z;
        xSemaphoreGive(g_sharedMutex);
    }
    return z;
}

// --- Valf / destek valf esleme ---
// P0=N434(v2), P1=N433(v0), P2=N437(v7), P3=N438(v4), K1=N435(v3), K2=N439(v6)
static const uint8_t CC_VALVE_IDX[6]   = {2, 0, 7, 4, 3, 6};
// P0,P1,K1 → N436(idx1);  P2,P3,K2 → N440(idx5)
static const uint8_t CC_SUPPORT_IDX[6] = {1, 1, 5, 5, 1, 5};

// --- Sabitler ---
static const float CC_R_COIL       = 6.0f;   // Bobin direnci (olculen: 5.5-6.2 ohm, ort ~6)
static const float CC_V_SUPPLY     = 12.0f;  // Besleme gerilimi (V)
static const float CC_FULL_OPEN_MA = 700.0f; // Tam acma akimi (mA)
static const float CC_PUMP_MIN_BAR = 42.0f;  // Minimum calisme basinci (bar)
static const float CC_PUMP_TGT_BAR = 60.0f;  // Pompa hedef basinci (bar)
static const float CC_STEP_MA      = 10.0f;  // Arama adim buyuklugu (mA)
static const float CC_MOVE_THRESH  = 1.5f;   // Hareket tespiti esigi (mm) — gürültüye dayanıklı
static const float CC_STEP_WAIT_MS = 800.0f; // Her adim bekleme suresi (ms)
static const float    CC_HOLD_TOL_MM    = 0.8f;  // Bang-bang yaklaşım tol (mm)
static const uint32_t CC_HOLD_SETTLE_MS = 800;   // Midpoint uygulandıktan sonra Iact settle (ms)

// ----------------------------------------------------------------
// Yardimci: mA → PWM duty (V = I * R modeli)
//   Besleme gerilimini g_tele.mainV'den okur (yoksa 12V varsayilan).
//   Orn: batarya 13.8V ise 12V varsayimiyla PWM %13 dusuk kalir.
// ----------------------------------------------------------------
static uint16_t ccMaToDuty(float mA) {
    if (mA <= 0.0f) return 0;
    float vSup = CC_V_SUPPLY;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
        float v = g_tele.mainV;
        xSemaphoreGive(g_sharedMutex);
        if (v >= 8.0f && v <= 18.0f) vSup = v;  // gecerli aralikta ise kullan
    }
    float d = (mA / 1000.0f * CC_R_COIL / vSup) * 4095.0f;
    if (d > 4095.0f) d = 4095.0f;
    return (uint16_t)d;
}

// ----------------------------------------------------------------
// Yardimci: valf akimini ayarla
//   closedLoop=true  → g_valveCustomCurrent_mA → valve_current_reg_step (TaskValveControl)
//   closedLoop=false → Sabit V=IR duty, integral YOK (eşik arama — windup önleme)
// ----------------------------------------------------------------
static void ccSetCurrent(uint8_t valveIdx, float mA, bool closedLoop = true) {
    float targetMa = (mA > 0.0f) ? mA : 0.0f;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (closedLoop) {
            g_valveCustomCurrent_mA[valveIdx] = targetMa;
            g_valveTargetDuty[valveIdx]       = 0;
        } else {
            g_valveCustomCurrent_mA[valveIdx] = 0.0f;
            g_valveTargetDuty[valveIdx]       = ccMaToDuty(targetMa);
        }
        xSemaphoreGive(g_sharedMutex);
    }
}

// ----------------------------------------------------------------
// Yardimci: piston pozisyonunu oku (mm)
// ----------------------------------------------------------------
static float ccReadMm(uint8_t p) {
    float mm = 0.0f;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        mm = g_pistonHallmm[p];
        xSemaphoreGive(g_sharedMutex);
    }
    return mm;
}

// ----------------------------------------------------------------
// Yardimci: abort kontrollu bekleme
//   INA PI artık TaskValveControl'da (valve_current_reg_step) çalışıyor.
//   closedLoop=true ise g_valveCustomCurrent_mA hedefi otomatik takip edilir.
// ----------------------------------------------------------------
static bool ccWait(uint32_t ms) {
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        if (!g_currentCalibRunning) return false;
        // Abort isteği geldiyse iç döngülerden de hemen çık
        if (g_currentCalibReq.abort) {
            g_currentCalibRunning = false;
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return true;
}

// ----------------------------------------------------------------
// Yardimci: JSON ilerleme gonder
// ----------------------------------------------------------------
static void ccSendProgress(uint8_t p, uint8_t phase, uint8_t prog, float mm, float mA) {
        char buf[96];
    snprintf(buf, sizeof(buf),
        "{\"_t\":\"CC\",\"p\":%d,\"phase\":%d,\"prog\":%d,\"mm\":%.1f,\"mA\":%.0f}",
        p, phase, prog, mm, mA);
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
}

// ----------------------------------------------------------------
// Yardimci: log mesaji
// ----------------------------------------------------------------
static void ccLog(const char *msg) {
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char *)msg);
}

// ================================================================
// Ana kalibrasyon: tek piston
// Donus: true=basarili, false=iptal/hata
// ================================================================
static bool ccCalibratePiston(uint8_t p) {
    const bool    isK1K2   = (p >= 4);
    const uint8_t valveIdx = CC_VALVE_IDX[p];
    const uint8_t suppIdx  = CC_SUPPORT_IDX[p];
    // K1/K2 daha yuksek PCV akimi gerektirir
    const float   pcvMa    = isK1K2 ? 750.0f : 650.0f;

    // Hold kontrolunu devre disi birak
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_pistonRuntime[p].hold_mid_enable  = false;
        g_pistonRuntime[p].hold_init_needed = true;
        xSemaphoreGive(g_sharedMutex);
    }

    {
        char buf[64];
        snprintf(buf, sizeof(buf), "[CCAL] START P%d v%d supp%d K=%d pcv=%.0fmA",
            p, valveIdx, suppIdx, (int)isK1K2, pcvMa);
        ccLog(buf);
    }

    // ================================================================
    // PHASE 0: PCV ac, pompa baslat, basinci 42-60 bar araligina getir
    // ================================================================
    ccSendProgress(p, 0, 0, ccReadMm(p), 0);

    // Pompaya AUTO komutu
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_pumpCmd.cmd = PUMP_CMD_AUTO;
        g_pumpCmd.seq++;
        xSemaphoreGive(g_sharedMutex);
    }

    // PCV'yi pcvMa ile ac
    ccSetCurrent(suppIdx, pcvMa);

    // 42 bar'a ulasana kadar bekle (max 20s)
    uint32_t t0 = millis();
    while (millis() - t0 < 20000) {
        if (!g_currentCalibRunning) { ccSetCurrent(valveIdx, 0); ccSetCurrent(suppIdx, 0); return false; }
        if (g_pumpPub.bar >= CC_PUMP_MIN_BAR) break;
        ccSendProgress(p, 0, (uint8_t)((millis()-t0)/200), ccReadMm(p), g_pumpPub.bar);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    if (g_pumpPub.bar < CC_PUMP_MIN_BAR) {
        ccLog("[CCAL] FAIL: pressure too low");
        ccSetCurrent(valveIdx, 0); ccSetCurrent(suppIdx, 0); return false;
    }
    ccSendProgress(p, 0, 10, ccReadMm(p), g_pumpPub.bar);

    // ================================================================
    // PHASE 1: Strok olcumu
    //   0mA → 500ms bekle → min_mm
    //   700mA → 500ms bekle → max_mm
    // ================================================================
    ccLog("[CCAL] STROKE");
    ccSendProgress(p, 1, 10, ccReadMm(p), 0);

    // Kapat (yay kuvveti)
    ccSetCurrent(valveIdx, 0.0f);
    if (!ccWait(500)) { ccSetCurrent(suppIdx, 0); return false; }
    // Kararlasinca oku
    if (!ccWait(500)) { ccSetCurrent(suppIdx, 0); return false; }
    float   minMm    = ccReadMm(p);
    int16_t minHallZ = isK1K2 ? 0 : ccReadHallZ(p);  // Hall ham (raw) — kapali konum

    // Tam ac
    ccSetCurrent(valveIdx, CC_FULL_OPEN_MA);
    if (!ccWait(500)) { ccSetCurrent(valveIdx, 0); ccSetCurrent(suppIdx, 0); return false; }
    if (!ccWait(500)) { ccSetCurrent(valveIdx, 0); ccSetCurrent(suppIdx, 0); return false; }
    float   maxMm    = ccReadMm(p);
    int16_t maxHallZ = isK1K2 ? 0 : ccReadHallZ(p);  // Hall ham (raw) — acik konum

    float strokeMm = maxMm - minMm;
    {
        char buf[64];
        snprintf(buf, sizeof(buf), "[CCAL] min=%.1f max=%.1f stroke=%.1f", minMm, maxMm, strokeMm);
        ccLog(buf);
    }
    ccSendProgress(p, 1, 30, maxMm, CC_FULL_OPEN_MA);

    if (strokeMm < 3.0f) {
        ccLog("[CCAL] FAIL: stroke too small");
        ccSetCurrent(valveIdx, 0); ccSetCurrent(suppIdx, 0); return false;
    }

    float targetMm = minMm + strokeMm * 0.5f;  // Orta nokta

    // ================================================================
    // PHASE 2: Acilma esigi bul (open_mA)
    //   Piston kapali konumdan basla, 400mA'dan 20mA adimlarla yuksel.
    //   Her adimda onceki pozisyona gore delta > CC_MOVE_THRESH → acildi.
    // ================================================================
    ccLog("[CCAL] FIND_OPEN_THRESH");
    ccSendProgress(p, 2, 30, minMm, 0);

    // Once tam kapat — pistonun yaya karsi tam yerlesebilmesi icin yeterli sure
    ccSetCurrent(valveIdx, 0.0f);
    if (!ccWait(1500)) { ccSetCurrent(suppIdx, 0); return false; }

    float openMa   = 600.0f;  // varsayilan
    bool  openFound = false;
    float prevMm    = ccReadMm(p);

    for (float testMa = 500.0f; testMa <= 700.0f && !openFound && g_currentCalibRunning; testMa += CC_STEP_MA) {
        ccSetCurrent(valveIdx, testMa, false);  // Sabit duty — INA integral YOK (windup onleme)
        uint8_t prog = (uint8_t)(30.0f + (testMa - 400.0f) / 300.0f * 20.0f);
        if (!ccWait((uint32_t)CC_STEP_WAIT_MS)) { ccSetCurrent(valveIdx, 0); ccSetCurrent(suppIdx, 0); return false; }
        float mm = ccReadMm(p);
        {
            char buf[80];
            uint16_t dbgDuty = 0;
            if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
                dbgDuty = g_valveTargetDuty[valveIdx];
                xSemaphoreGive(g_sharedMutex);
            }
            snprintf(buf, sizeof(buf), "[CCAL] open_step mA=%.0f duty=%u mm=%.2f prev=%.2f delta=%.2f",
                testMa, dbgDuty, mm, prevMm, mm - prevMm);
            ccLog(buf);
        }
        ccSendProgress(p, 2, prog, mm, testMa);
        if (mm > prevMm + CC_MOVE_THRESH) {
            openMa = testMa;
            openFound = true;
        }
        prevMm = mm;
    }

    if (!openFound) {
        ccLog("[CCAL] open thresh not found, using default");
    } else {
        char buf[48];
        snprintf(buf, sizeof(buf), "[CCAL] open_mA=%.0f", openMa);
        ccLog(buf);
    }

    // ================================================================
    // PHASE 3: Kapanma esigi bul (close_mA)
    //   Piston tam acik konumdan basla, open_mA'dan 20mA adimlarla dus.
    //   Her adimda onceki pozisyona gore delta < -CC_MOVE_THRESH → kapandi.
    // ================================================================
    ccLog("[CCAL] FIND_CLOSE_THRESH");
    ccSendProgress(p, 3, 50, maxMm, CC_FULL_OPEN_MA);

    // Once tam ac — piston tam acik konuma ulassin
    ccSetCurrent(valveIdx, CC_FULL_OPEN_MA);
    if (!ccWait(1500)) { ccSetCurrent(valveIdx, 0); ccSetCurrent(suppIdx, 0); return false; }

    float closeMa   = 300.0f;  // varsayilan
    bool  closeFound = false;
    prevMm = ccReadMm(p);

    for (float testMa = 500.0f; testMa >= 200.0f && !closeFound && g_currentCalibRunning; testMa -= CC_STEP_MA) {
        ccSetCurrent(valveIdx, testMa, false);  // Sabit duty — INA integral YOK (windup onleme)
        uint8_t prog = (uint8_t)(50.0f + (openMa - testMa) / openMa * 20.0f);
        if (!ccWait((uint32_t)CC_STEP_WAIT_MS)) { ccSetCurrent(valveIdx, 0); ccSetCurrent(suppIdx, 0); return false; }
        float mm = ccReadMm(p);
        {
            char buf[80];
            uint16_t dbgDuty = 0;
            if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
                dbgDuty = g_valveTargetDuty[valveIdx];
                xSemaphoreGive(g_sharedMutex);
            }
            snprintf(buf, sizeof(buf), "[CCAL] close_step mA=%.0f duty=%u mm=%.2f prev=%.2f delta=%.2f",
                testMa, dbgDuty, mm, prevMm, mm - prevMm);
            ccLog(buf);
        }
        ccSendProgress(p, 3, prog < 70 ? prog : 70, mm, testMa);
        if (mm < prevMm - CC_MOVE_THRESH) {
            closeMa = testMa;
            closeFound = true;
        }
        prevMm = mm;
    }

    if (!closeFound) {
        ccLog("[CCAL] close thresh not found, using default");
    } else {
        char buf[48];
        snprintf(buf, sizeof(buf), "[CCAL] close_mA=%.0f", closeMa);
        ccLog(buf);
    }

    float holdMa = (openMa + closeMa) * 0.5f;

    // ================================================================
    // PHASE 4: Hold kalibrasyonu — sadece P0-P3
    //   4a: Tam aç (bilinen başlangıç)
    //   4b: 5 seviyeli bang-bang ile hedef konuma yaklaş:
    //         pos > tgt+tol  → closeMa        (kaba kapama)
    //         pos < tgt-tol  → openMa         (kaba açma)
    //         pos > tgt      → closeMa+20     (ince kapama)
    //         pos < tgt      → openMa-20      (ince açma)
    //         |pos-tgt|≤tol/2 → hedefe ulaşıldı
    //   4c: holdMa = (openMa+closeMa)/2 — FORMEL midpoint (convergence search yok)
    //         Saha testi: gerçek hold 510-540 mA, midpoint ±20 mA isabetli.
    //         Çalışma anı PD kontrolör (piston_ctrl_step) ±0.3 mm dengeyi yakalar.
    // ================================================================
    if (!isK1K2) {
        ccLog("[CCAL] HOLD_CALIB");
        ccSendProgress(p, 4, 70, ccReadMm(p), holdMa);

        // --- 4a: Tam kapat — bilinen başlangıç konumu 0 olmalı---
        ccSetCurrent(valveIdx, 0.0f);  
        if (!ccWait(1500)) { ccSetCurrent(valveIdx, 0); ccSetCurrent(suppIdx, 0); return false; }
        {
            char buf[64];
            snprintf(buf, sizeof(buf), "[CCAL] hold_open pos=%.1f (max=%.1f)", ccReadMm(p), maxMm);
            ccLog(buf);
        }

        // --- 4b: Orantılı yaklaşım ile hedef konuma git ---
        //   Uzakta: tam closeMa/openMa (hızlı hareket)
        //   Yakında: holdMa'ya doğru karıştır (yavaş, hassas hareket)
        //
        //   cmdMa = holdMa + ratio * (eşik - holdMa)
        //   ratio = clamp(|hata| / ZONE_MM, 0, 1)
        //
        //   Örnek (closeMa=460, holdMa=550, openMa=640, zone=10mm):
        //     10mm uzakta: ratio=1.0 → cmdMa=460  (tam kapatma hızı)
        //      5mm uzakta: ratio=0.5 → cmdMa=505  (yarı hız)
        //      2mm uzakta: ratio=0.2 → cmdMa=532  (yavaş)
        //    0.5mm içinde: nearTarget → dur
        const float BB_ZONE_MM = (maxMm - minMm) * 0.4f;  // stroke'un %40'ı = tam hız bölgesi

        bool nearTarget = false;
        for (int bb = 0; bb < 40 && !nearTarget && g_currentCalibRunning; bb++) {
            float pos = ccReadMm(p);
            float err = pos - targetMm;

            if (fabsf(err) <= CC_HOLD_TOL_MM * 0.5f) {
                nearTarget = true;
                break;
            }

            float ratio = (BB_ZONE_MM > 0.1f) ? fminf(fabsf(err) / BB_ZONE_MM, 1.0f) : 1.0f;
            float cmdMa;
            if (err > 0.0f) {
                cmdMa = holdMa + ratio * (closeMa - holdMa);  // fazla açık → closeMa yönünde
            } else {
                cmdMa = holdMa + ratio * (openMa  - holdMa);  // fazla kapalı → openMa yönünde
            }

            ccSetCurrent(valveIdx, cmdMa, false);  // Sabit duty — hızlı yanıt
            {
                char buf[96];
                snprintf(buf, sizeof(buf), "[CCAL] bang bb=%d pos=%.1f tgt=%.1f err=%.1f ratio=%.2f mA=%.0f",
                    bb, pos, targetMm, err, ratio, cmdMa);
                ccLog(buf);
            }
            ccSendProgress(p, 4, 72, pos, cmdMa);
            if (!ccWait(600)) { ccSetCurrent(valveIdx, 0); ccSetCurrent(suppIdx, 0); return false; }
        }

        {
            char buf[72];
            snprintf(buf, sizeof(buf), "[CCAL] bang_done near=%d pos=%.1f tgt=%.1f",
                (int)nearTarget, ccReadMm(p), targetMm);
            ccLog(buf);
        }

        // --- 4c: holdMa = midpoint (formel sonuç) + tek seferlik bilgi-amaçlı örnekleme ---
        //
        // Geçmişte burada Kp=5 mA/mm'lik iteratif convergence loop vardı (10x verify,
        // pos0/pos1 drift, INA PI vs.). Saha verisi gösterdi ki:
        //   * Gerçek hold akımı tüm pistonlarda 510-540 mA bandında
        //   * (openMa + closeMa) / 2 bunu ±20 mA içinde tutturuyor
        //   * Loop, mekanik histerezis + Hall gürültüsü yüzünden tipik olarak
        //     "not converged" ile sonlanıyor ve zaten midpoint'e düşüyordu
        //
        // Sonuç: convergence aramasını çıkardık. Midpoint formel hold_mA.
        // PD kontrolör (piston_ctrl_step) çalışma anında gerçek dengeyi zaten
        // ±0.3 mm içinde yakalıyor (HOLD_TUNING_TODO.md saha test sonuçları).
        holdMa = (openMa + closeMa) * 0.5f;

        // Bilgi amaçlı tek örnekleme (kalibrasyon raporuna giren "hold_verify" kaydı)
        ccSetCurrent(valveIdx, holdMa);  // closedLoop=true
        if (!ccWait(CC_HOLD_SETTLE_MS)) { ccSetCurrent(valveIdx, 0); ccSetCurrent(suppIdx, 0); return false; }
        float verifyPos = ccReadMm(p);
        {
            char buf[112];
            snprintf(buf, sizeof(buf),
                "[CCAL] hold_midpoint mA=%.0f pos=%.1f tgt=%.1f drift=%.1f (formel: midpoint, verify yok)",
                holdMa, verifyPos, targetMm, verifyPos - targetMm);
            ccLog(buf);
        }
        ccSendProgress(p, 4, 92, verifyPos, holdMa);
    }

    // ================================================================
    // PHASE 5 (P0-P3) / PHASE 4 (K1/K2): Kaydet
    // ================================================================
    uint8_t savePhase = isK1K2 ? 4 : 5;
    ccSendProgress(p, savePhase, 94, ccReadMm(p), holdMa);
    ccLog("[CCAL] SAVING");

    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_pistonCalibData[p].hold_mA    = holdMa;
        g_pistonCalibData[p].open_mA    = openMa;
        g_pistonCalibData[p].close_mA   = closeMa;
        if (isK1K2) {
            // K1/K2 hall raw kullanmaz — mm*10 (eski davraniş korunur)
            g_pistonCalibData[p].min_raw = (uint16_t)(minMm    * 10.0f);
            g_pistonCalibData[p].max_raw = (uint16_t)(maxMm    * 10.0f);
            g_pistonCalibData[p].mid_raw = (uint16_t)(targetMm * 10.0f);
        } else {
            // P0-P3: AutoShiftV2 getCalibLimits hall HAM degeri bekler
            // (eski Phase 7 API'si de hall raw yaziyor — uyum icin ayni format)
            // Hall acik/kapali konumlarda buyuk-kucuk sirasi pistona gore degisir;
            // AutoShiftV2 stroke = zMax - zMin >= 0 bekler → sirala.
            int16_t zLo = (minHallZ < maxHallZ) ? minHallZ : maxHallZ;
            int16_t zHi = (minHallZ < maxHallZ) ? maxHallZ : minHallZ;
            // uint16_t cast int16_t→uint16_t bit korur (negatif → high bit set),
            // AutoShiftV2 tekrar (int16_t) cast ederek isareti geri alir.
            g_pistonCalibData[p].min_raw = (uint16_t)zLo;
            g_pistonCalibData[p].max_raw = (uint16_t)zHi;
            g_pistonCalibData[p].mid_raw = (uint16_t)(int16_t)((zLo + zHi) / 2);
        }
        // P0-P3 holdcontrol_V2 / piston_ctrl_step icin PWM esiklerini de doldur.
        // TaskCurrentCalib akim esikleri bulur; bunlari ccMaToDuty ile PWM'e cevir.
        uint16_t dHold  = ccMaToDuty(holdMa);
        uint16_t dOpen  = ccMaToDuty(openMa);
        uint16_t dClose = ccMaToDuty(closeMa);
        g_pistonCalibData[p].duty_hold       = dHold;
        g_pistonCalibData[p].duty_open_thresh  = dOpen;
        g_pistonCalibData[p].duty_close_thresh = dClose;
        g_pistonCalibData[p].duty_breakaway    = dOpen;
        g_pistonCalibData[p].duty_min          = dClose;
        g_pistonCalibData[p].duty_max          = dOpen;
        g_pistonCalibData[p].direction         = 1;  // +1: duty artinca x artar
        g_pistonCalibData[p].version    = PISTON_CALIB_VERSION;  // NVS Load() bunu kontrol eder
        g_pistonCalibData[p].calibrated = true;
        xSemaphoreGive(g_sharedMutex);
    }
    if (!isK1K2) {
        char hbuf[96];
        snprintf(hbuf, sizeof(hbuf),
                 "[CCAL] hall_raw min=%d max=%d mid=%d",
                 minHallZ, maxHallZ, (int16_t)((minHallZ + maxHallZ) / 2));
        ccLog(hbuf);
    }
    PistonCalibStorage_Save(p, g_pistonCalibData[p]);

    {
        char msg[192];
        snprintf(msg, sizeof(msg),
            "{\"_t\":\"CC\",\"p\":%d,\"done\":true,\"ok\":true"
            ",\"min_mm\":%.1f,\"max_mm\":%.1f"
            ",\"hold_mA\":%.0f,\"open_mA\":%.0f,\"close_mA\":%.0f}",
            p, minMm, maxMm, holdMa, openMa, closeMa);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
    }

    // Valfleri kapat
    ccSetCurrent(valveIdx, 0.0f);
    ccSetCurrent(suppIdx, 0.0f);
    ccSendProgress(p, savePhase, 100, ccReadMm(p), 0);
    return true;
}

// ================================================================
// TaskCurrentCalib — FreeRTOS task
// ================================================================
void TaskCurrentCalib(void *pvParameters) {
    (void)pvParameters;

    volatile uint32_t lastSeq = g_currentCalibReqSeq;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));

        if (g_currentCalibReqSeq == lastSeq) continue;
        lastSeq = g_currentCalibReqSeq;

        if (g_currentCalibReq.abort) {
            g_currentCalibRunning = false;
            ccLog("[CCAL] ABORTED");
            continue;
        }

        g_currentCalibRunning = true;

        uint8_t startP = g_currentCalibReq.piston;
        bool    doAll  = g_currentCalibReq.all;
        uint8_t endP   = doAll ? 5 : startP;

        {
            char buf[48];
            snprintf(buf, sizeof(buf), "[CCAL] REQUEST p=%d all=%d", startP, (int)doAll);
            ccLog(buf);
        }

        for (uint8_t p = startP; p <= endP; p++) {
            if (!g_currentCalibRunning) break;

            bool ok = ccCalibratePiston(p);

            if (!ok && g_currentCalibRunning) {
                char msg[64];
                snprintf(msg, sizeof(msg),
                    "{\"_t\":\"CC\",\"p\":%d,\"done\":true,\"ok\":false}", p);
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
            }

            if (p < endP && g_currentCalibRunning)
                vTaskDelay(pdMS_TO_TICKS(1000));
        }

        g_currentCalibRunning = false;
        ccLog("[CCAL] ALL_DONE");
    }
}

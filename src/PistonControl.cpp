#include "PistonControl.h"
#include "PistonCalib.h"
#include "Tasks.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <string.h>
#include <math.h>

const PistonAxisConfig kPistonAxisConfig[PISTON_CHANNEL_COUNT] = {
    {"N434", PISTON_5_7, 2, 1, 0}, // Grup-1
    {"N433", PISTON_1_3, 0, 1, 0}, // Grup-1
    {"N437", PISTON_2_4, 7, 5, 1}, // Grup-2
    {"N438", PISTON_6_R, 4, 5, 1}, // Grup-2
    {"N435", PISTON_K1,  3, 1, 0}, // K1 kavrama - Grup-1
    {"N439", PISTON_K2,  6, 5, 1}  // K2 kavrama - Grup-2
};

static constexpr uint8_t  PCV_INDEX[2] = {1,5};
static constexpr float    PRESSURE_LIMIT_BAR = 70.0f;   // hard stop, ?zeri durumlar? h?zl? bo?alt
static constexpr float    CAL_TARGET_BAR     = 40.0f;
static constexpr float    CAL_PRESSURE_WIN   = 5.0f;
static constexpr uint32_t CAL_PRESSURE_TIMEOUT_MS = 15000;
static constexpr uint32_t CAL_STALL_MS       = 800;
static constexpr float    CAL_STALL_THRESH   = 0.0050f; // ~5 mV ham gerilim degisimi
static constexpr uint16_t CAL_PULSE_STEP     = 120;   // breakaway aramada daha b?y?k adim
static constexpr uint16_t CAL_PULSE_MS       = 100;   // pulse s?resi
static constexpr uint16_t CAL_SETTLE_MS      = 150;   // bekleme s?resi
static constexpr uint16_t CAL_SEEK_MARGIN    = 0;     // min/max taramada breakaway ekleme yok
static constexpr uint32_t CAL_TOTAL_TIMEOUT_MS = 12000;
static constexpr uint32_t CAL_SEEK_FALLBACK_MS = 4500;

static constexpr float    HOLD_SET_BAR = 35.0f;
static constexpr float    MOVE_SET_BAR = 45.0f;
static constexpr float    PRESS_KP     = 45.0f;
static constexpr float    PRESS_KI     = 3.5f;
static constexpr uint16_t PRESS_CMD_MAX= 3200;

static constexpr float    X_ALPHA      = 0.18f;
static constexpr float    V_ALPHA      = 0.12f;
static constexpr float    E_FAST       = 0.08f;
static constexpr float    E_HOLD       = 0.02f;
static constexpr float    E_DB         = 0.006f;
static constexpr float    E_LEARN      = 0.01f;
static constexpr float    V_LEARN      = 0.004f;
static constexpr float    PREF_BAR     = 40.0f;
static constexpr float    KP0_FAST     = 2200.0f;
static constexpr float    KP0_SLOW     = 1200.0f;
static constexpr float    KD0_FAST     = 160.0f;
static constexpr float    KD0_SLOW     = 340.0f;
static constexpr uint16_t DUTY_RATE_LIMIT = 220;
static constexpr uint16_t DUTY_SAFE_MAX   = 4095;
static constexpr uint32_t PULSE_AND_WAIT_MS = 80;
static constexpr uint16_t PCV_DRAIN_DUTY   = 3000;
static constexpr uint16_t SUPPORT_VALVE_DUTY = 2500;

// ============================================================================
// Hold PWM Bulma Sabitleri (selo1_espnow.cpp'den)
// ============================================================================
static constexpr uint16_t HOLD_PWM_MAX      = 2000;   // Kesinlikle aşılmamalı!
static constexpr uint16_t HOLD_DUTY_WINDOW  = 400;    // Başlangıç arama penceresi
static constexpr uint16_t HOLD_DITHER_DELTA = 80;     // Dither genliği
static constexpr uint16_t HOLD_DITHER_PULSE_MS = 40;  // Dither pulse süresi
static constexpr uint8_t  HOLD_DITHER_CYCLES = 5;     // Dither döngü sayısı
static constexpr float    HOLD_DRIFT_EPS    = 0.08f;  // Kabul edilebilir drift (mm/s)
static constexpr uint8_t  HOLD_BISECT_ITERS = 6;      // Bisection iterasyon sayısı

// Hold PWM bulma state
struct HoldFindState {
    bool     active;
    uint8_t  piston;
    uint8_t  step;       // 0=idle, 1=park, 2=measure_lo, 3=measure_hi, 4=bisect, 5=done
    uint32_t stepTs;
    uint16_t d0;         // Başlangıç tahmini
    int      lo, hi;     // Arama penceresi
    float    v_lo, v_hi; // Drift değerleri
    uint16_t result;     // Bulunan hold PWM
};
static HoldFindState s_holdFind{};

enum CalibStep : uint8_t {
    CAL_IDLE = 0,
    CAL_WAIT_PRESSURE,
    CAL_SENSOR_CHECK,
    CAL_FIND_DIR,
    CAL_BREAKAWAY,
    CAL_SEEK_MIN,
    CAL_SEEK_MAX,
    CAL_VERIFY_MID,
    CAL_FF_SWEEP,
    CAL_DONE,
    CAL_ERROR
};

struct CalibRuntime {
    bool     active;
    uint8_t  piston;
    uint8_t  group;
    CalibStep step;
    uint32_t stepTs;
    uint32_t overallTs;
    float    lastRaw;
    float    refRaw;
    uint16_t pulseDuty;
    uint16_t breakaway;
    float    minRaw;
    float    maxRaw;
    uint8_t  ffIdx;
    float    ffAcc;
    uint16_t ffSamples;
    char     err[16];
    uint16_t lastSeekDuty;
    uint32_t seekStartMs;
};

static CalibRuntime         s_cal{};
static uint32_t             s_lastCalibSeq = 0;
static PistonCalibProgress  s_calProg{};
static float                s_prevX[PISTON_CHANNEL_COUNT] = {0};
static bool                 s_calAllPending = false;
static uint8_t              s_calAllNext = 0;
static bool                 s_pumpKickActive = false;

static inline void sendLog(const char *msg){
    if (msg) {
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
    }
}

static inline void requestPumpStart(){
    if (!g_sharedMutex) return;
    if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
        g_pumpCmd.cmd = PUMP_CMD_START;
        g_pumpCmd.seq++;
        xSemaphoreGive(g_sharedMutex);
    }
}

static inline float clampf(float v, float lo, float hi){
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline uint16_t clampDuty(uint16_t duty, const PistonCalibData &cal){
    uint16_t lo = cal.duty_min;
    uint16_t hi = cal.duty_max ? cal.duty_max : DUTY_SAFE_MAX;
    if (hi > DUTY_SAFE_MAX) hi = DUTY_SAFE_MAX;
    if (duty < lo) duty = lo;
    if (duty > hi) duty = hi;
    return duty;
}

static inline float interpFF(const PistonCalibData &cal, float p_bar){
    float p0 = cal.p_bins[0];
    if (p_bar <= p0) return cal.u_ff_map[0];
    for (size_t i = 1; i < PISTON_FF_BINS; ++i) {
        float p1 = cal.p_bins[i];
        if (p_bar <= p1) {
            float t = (p_bar - p0) / (p1 - p0 + 1e-6f);
            return cal.u_ff_map[i-1] + t * (cal.u_ff_map[i] - cal.u_ff_map[i-1]);
        }
        p0 = p1;
    }
    return cal.u_ff_map[PISTON_FF_BINS-1];
}

static inline void copyDefaultsIfEmpty(PistonCalibData &d) {
    if (d.p_bins[0] == 0.0f && d.p_bins[1] == 0.0f && d.p_bins[2] == 0.0f) {
        d.p_bins[0] = 30.0f;
        d.p_bins[1] = 40.0f;
        d.p_bins[2] = 50.0f;
    }
    if (d.direction != 1 && d.direction != -1) d.direction = 1;
    if (d.duty_max == 0) d.duty_max = DUTY_SAFE_MAX;
    d.version = PISTON_CALIB_VERSION;
}

// ============================================================================
// Hold PWM Bulma Fonksiyonları (selo1_espnow.cpp'den uyarlandı)
// ============================================================================

// PWM'i güvenli sınırla (max 2000!)
static inline uint16_t clampHoldDuty(int duty) {
    if (duty < 0) return 0;
    if (duty > HOLD_PWM_MAX) return HOLD_PWM_MAX;
    return (uint16_t)duty;
}

// duty650'den hold PWM tahmini (yaklaşık %63)
static inline uint16_t guessHoldFromDuty650(uint16_t duty650) {
    if (duty650 == 0) return 950;
    int g = (int)(0.63f * duty650);
    if (g < 650) g = 650;
    if (g > 1900) g = 1900;
    return (uint16_t)g;
}

// Piston pozisyonunu oku (mm)
static inline float readPistonPosMm(uint8_t piston) {
    if (piston >= PISTON_CHANNEL_COUNT) return 0.0f;
    return g_pistonHallmm[piston];
}

// Valf PWM ayarla
static inline void setValvePWM(uint8_t piston, uint16_t duty) {
    if (piston >= PISTON_CHANNEL_COUNT) return;
    uint8_t valveIdx = kPistonAxisConfig[piston].valveIdx;
    g_valveTargetDuty[valveIdx] = clampHoldDuty(duty);
}

// Dither ile drift ölç (mm/s) - selo1_espnow.cpp: measure_drift_dither
// Küçük simetrik PWM sarsması ile stiction kırılır ve net hareket ölçülür
static float measureDriftDither(uint8_t piston, int dutyBase, 
                                 int delta = HOLD_DITHER_DELTA,
                                 int pulseMs = HOLD_DITHER_PULSE_MS,
                                 int cycles = HOLD_DITHER_CYCLES) {
    uint8_t valveIdx = kPistonAxisConfig[piston].valveIdx;
    
    // Başlangıç konumu
    float x0 = readPistonPosMm(piston);
    
    // Küçük simetrik sarsma ile stiction'ı kır
    for (int i = 0; i < cycles; i++) {
        g_valveTargetDuty[valveIdx] = clampHoldDuty(dutyBase + delta);
        vTaskDelay(pdMS_TO_TICKS(pulseMs));
        g_valveTargetDuty[valveIdx] = clampHoldDuty(dutyBase - delta);
        vTaskDelay(pdMS_TO_TICKS(pulseMs));
    }
    
    // Ölçüm sonunda net konum değişimi
    float x1 = readPistonPosMm(piston);
    
    // Toplam süre (s)
    float dt = cycles * 2.0f * (pulseMs / 1000.0f);
    
    // Kanalı serbest bırak
    g_valveTargetDuty[valveIdx] = 0;
    
    return (x1 - x0) / (dt > 1e-3f ? dt : 1.0f);
}

// Pistonu ortaya park et
static void parkToMid(uint8_t piston, uint16_t holdGuess) {
    float L = 12.0f;  // Varsayılan strok (mm) - kalibrasyondan alınabilir
    float mid = 0.5f * L;
    uint16_t dExt = clampHoldDuty(holdGuess + 250);
    uint16_t dRet = clampHoldDuty(holdGuess - 250);
    
    uint8_t valveIdx = kPistonAxisConfig[piston].valveIdx;
    uint32_t t0 = millis();
    float prev = readPistonPosMm(piston);
    
    while (millis() - t0 < 1500) {
        float x = readPistonPosMm(piston);
        float e = mid - x;
        if (fabsf(e) <= 0.6f) break;
        
        if (e > 0) {
            g_valveTargetDuty[valveIdx] = dExt;
        } else {
            g_valveTargetDuty[valveIdx] = dRet;
        }
        vTaskDelay(pdMS_TO_TICKS(60));
        
        // Hareket tıkandıysa çık
        float dx = readPistonPosMm(piston) - prev;
        prev = readPistonPosMm(piston);
        if (fabsf(dx) < 0.02f) break;
    }
    g_valveTargetDuty[valveIdx] = 0;
}

// Hold PWM bul - Dither yöntemi (selo1_espnow.cpp: find_hold_dither)
static uint16_t findHoldDither(uint8_t piston, uint16_t d0) {
    // Hold aramadan önce ortala
    parkToMid(piston, d0);
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Başlangıç penceresi
    int lo = (int)d0 - HOLD_DUTY_WINDOW;
    int hi = (int)d0 + HOLD_DUTY_WINDOW;
    if (lo < 0) lo = 0;
    if (hi > HOLD_PWM_MAX) hi = HOLD_PWM_MAX;
    
    float v_lo = measureDriftDither(piston, lo);
    float v_hi = measureDriftDither(piston, hi);
    
    // İşaret değişimi yoksa pencereyi genişlet
    for (int k = 0; k < 2 && !(v_lo < -HOLD_DRIFT_EPS && v_hi > HOLD_DRIFT_EPS); k++) {
        lo = (lo > 150) ? lo - 150 : 0;
        hi = (hi < HOLD_PWM_MAX - 150) ? hi + 150 : HOLD_PWM_MAX;
        v_lo = measureDriftDither(piston, lo);
        v_hi = measureDriftDither(piston, hi);
    }
    
    // Orta değer
    float v0 = measureDriftDither(piston, d0);
    if (fabsf(v0) <= HOLD_DRIFT_EPS) return d0;
    
    if (!(v_lo < -HOLD_DRIFT_EPS && v_hi > HOLD_DRIFT_EPS)) {
        // Gradyan adımla güvenli yaklaş
        int d = d0;
        for (int i = 0; i < 8; i++) {
            d += (v0 > 0) ? -60 : +60;
            d = clampHoldDuty(d);
            v0 = measureDriftDither(piston, d);
            if (fabsf(v0) <= HOLD_DRIFT_EPS) return (uint16_t)d;
        }
        return (uint16_t)d;
    }
    
    // Bisection (drift işareti ile)
    int a = lo, b = hi;
    for (int it = 0; it < HOLD_BISECT_ITERS; ++it) {
        int m = (a + b) / 2;
        float vm = measureDriftDither(piston, m);
        if (fabsf(vm) <= HOLD_DRIFT_EPS) return (uint16_t)m;
        if (vm > 0) b = m; else a = m;
    }
    return (uint16_t)((a + b) / 2);
}

// Hold PWM bulma işlemini başlat
static void startFindHold(uint8_t piston) {
    s_holdFind.active = true;
    s_holdFind.piston = piston;
    s_holdFind.step = 1;  // park
    s_holdFind.stepTs = millis();
    s_holdFind.d0 = guessHoldFromDuty650(1500);  // Varsayılan tahmin
    s_holdFind.result = 0;
    
    char msg[48];
    snprintf(msg, sizeof(msg), "[HOLD] Piston %d: Finding hold PWM...", piston);
    sendLog(msg);
}

// Hold PWM bulma state machine (non-blocking)
static void updateFindHold() {
    if (!s_holdFind.active) return;
    
    uint32_t now = millis();
    uint8_t p = s_holdFind.piston;
    
    switch (s_holdFind.step) {
        case 1: // Park to mid
            parkToMid(p, s_holdFind.d0);
            s_holdFind.step = 2;
            s_holdFind.stepTs = now;
            break;
            
        case 2: // Measure low
            s_holdFind.lo = (int)s_holdFind.d0 - HOLD_DUTY_WINDOW;
            if (s_holdFind.lo < 0) s_holdFind.lo = 0;
            s_holdFind.v_lo = measureDriftDither(p, s_holdFind.lo);
            s_holdFind.step = 3;
            break;
            
        case 3: // Measure high
            s_holdFind.hi = (int)s_holdFind.d0 + HOLD_DUTY_WINDOW;
            if (s_holdFind.hi > HOLD_PWM_MAX) s_holdFind.hi = HOLD_PWM_MAX;
            s_holdFind.v_hi = measureDriftDither(p, s_holdFind.hi);
            s_holdFind.step = 4;
            break;
            
        case 4: // Bisection
            {
                // İşaret değişimi var mı?
                if (s_holdFind.v_lo < -HOLD_DRIFT_EPS && s_holdFind.v_hi > HOLD_DRIFT_EPS) {
                    int a = s_holdFind.lo, b = s_holdFind.hi;
                    for (int it = 0; it < HOLD_BISECT_ITERS; ++it) {
                        int m = (a + b) / 2;
                        float vm = measureDriftDither(p, m);
                        if (fabsf(vm) <= HOLD_DRIFT_EPS) {
                            s_holdFind.result = (uint16_t)m;
                            break;
                        }
                        if (vm > 0) b = m; else a = m;
                    }
                    if (s_holdFind.result == 0) {
                        s_holdFind.result = (uint16_t)((a + b) / 2);
                    }
                } else {
                    // Gradyan adım
                    int d = s_holdFind.d0;
                    float v0 = measureDriftDither(p, d);
                    for (int i = 0; i < 8; i++) {
                        d += (v0 > 0) ? -60 : +60;
                        d = clampHoldDuty(d);
                        v0 = measureDriftDither(p, d);
                        if (fabsf(v0) <= HOLD_DRIFT_EPS) break;
                    }
                    s_holdFind.result = (uint16_t)d;
                }
            }
            s_holdFind.step = 5;
            break;
            
        case 5: // Done
            {
                // Sonucu kaydet
                if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    g_pistonCalibData[p].duty_hold = s_holdFind.result;
                    xSemaphoreGive(g_sharedMutex);
                }
                
                char msg[64];
                snprintf(msg, sizeof(msg), "[HOLD] Piston %d: Found hold PWM = %u", p, s_holdFind.result);
                sendLog(msg);
                
                // Kalibrasyon verisini kaydet
                PistonCalibStorage_Save(p, g_pistonCalibData[p]);
                
                s_holdFind.active = false;
                s_holdFind.step = 0;
            }
            break;
    }
}

static void publishCalibJSON(uint8_t piston, bool ok, const char *err){
        PistonCalibData snap{};
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        snap = g_pistonCalibData[piston];
        xSemaphoreGive(g_sharedMutex);
    }
    JsonDocument doc;
    doc["r"] = true;
    doc["ok"] = ok;
    doc["cmd"] = "piston_calib";
    doc["p"] = kPistonAxisConfig[piston].name;
    if (!ok && err) doc["err"] = err;
    doc["min"] = snap.min_raw;
    doc["max"] = snap.max_raw;
    doc["mid"] = snap.mid_raw;
    doc["dir"] = snap.direction;
    doc["break"] = snap.duty_breakaway;
    JsonArray uff = doc["uff"].to<JsonArray>();
    JsonArray pb  = doc["pbins"].to<JsonArray>();
    for (size_t i = 0; i < PISTON_FF_BINS; ++i) {
        uff.add(snap.u_ff_map[i]);
        pb.add(snap.p_bins[i]);
    }
    char buf[512];
    size_t n = serializeJson(doc, buf, sizeof(buf) - 1);
    if (n >= sizeof(buf) - 1) n = sizeof(buf) - 2;
    buf[n] = 0;
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
}

static void startCalib(uint8_t piston, uint32_t nowMs){
    s_cal = {};
    s_cal.active = true;
    s_cal.piston = piston;
    s_cal.group  = PistonControl_GroupOf((PistonChannel)piston);
    s_cal.step   = CAL_WAIT_PRESSURE;
    s_cal.stepTs = nowMs;
        s_cal.overallTs = nowMs;
        s_calProg.running = true;
        s_calProg.piston  = piston;
        s_calProg.step    = s_cal.step;
        s_calProg.err[0]  = 0;
        s_pumpKickActive = false;

        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            g_pistonRuntime[piston].state = PistonRuntimeState::CALIB;
            xSemaphoreGive(g_sharedMutex);
        }
    {
        char msg[80];
        snprintf(msg, sizeof(msg), "[CALIB] start %s", kPistonAxisConfig[piston].name);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
    }
}

static void finishCalib(bool ok, const char *err, uint32_t nowMs){
    if (!s_cal.active) return;
    uint8_t piston = s_cal.piston;
    s_calProg.running = false;
    if (!ok && err) {
        strlcpy(s_calProg.err, err, sizeof(s_calProg.err));
    } else {
        s_calProg.err[0] = 0;
    }
    if (ok) {
        PistonCalibData d{};
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            d = g_pistonCalibData[piston];
            xSemaphoreGive(g_sharedMutex);
        }
        d.calibrated = true;
        d.min_raw = (uint16_t)(s_cal.minRaw * 10000.0f);
        d.max_raw = (uint16_t)(s_cal.maxRaw * 10000.0f);
        d.mid_raw = (uint16_t)(0.5f * (d.min_raw + d.max_raw));
        if (d.duty_breakaway == 0) d.duty_breakaway = s_cal.breakaway ? s_cal.breakaway : 0;
        copyDefaultsIfEmpty(d);
        PistonCalibStorage_Save(piston, d);
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            g_pistonCalibData[piston] = d;
            g_pistonRuntime[piston].state = PistonRuntimeState::IDLE;
            xSemaphoreGive(g_sharedMutex);
        }
    } else {
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            g_pistonRuntime[piston].state = PistonRuntimeState::IDLE;
            xSemaphoreGive(g_sharedMutex);
        }
    }
    publishCalibJSON(piston, ok, err);
    s_cal = {};
    if (s_calAllPending) {
        if (!ok) {
            s_calAllPending = false;
            s_calAllNext = 0;
            return;
        }
        s_calAllNext++;
        if (s_calAllNext < PISTON_CHANNEL_COUNT) {
            startCalib(s_calAllNext, nowMs);
        } else {
            s_calAllPending = false;
            s_calAllNext = 0;
        }
    }
}

static void updateCalib(const PistonControlInput &in, uint16_t targetDuty[8]){
    if (!s_cal.active) return;
    uint8_t piston = s_cal.piston;
    uint8_t valveIdx = PistonControl_ValveIdx((PistonChannel)piston);
    uint8_t group = s_cal.group;
    if (valveIdx >= 8) {
        finishCalib(false, "VALVE", in.now_ms);
        return;
    }
    float raw = in.hall_raw[piston];
    uint32_t now = in.now_ms;
    s_calProg.step = s_cal.step;
    g_pressureGroupState[group].p_ref = CAL_TARGET_BAR;

    // Genel timeout
    if ((now - s_cal.overallTs) > CAL_TOTAL_TIMEOUT_MS) {
        finishCalib(false, "TIMEOUT", now);
        return;
    }

    switch (s_cal.step) {
        case CAL_WAIT_PRESSURE: {
            // Üretici modunda basınç kontrolü bypass edilir
            if (g_manufacturerMode) {
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[MFG] Pressure check bypassed");
                s_cal.step = CAL_SENSOR_CHECK;
                s_cal.stepTs = now;
                s_cal.refRaw = raw;
                break;
            }
            float p = in.pressure_bar[group];
            bool ok = p >= (CAL_TARGET_BAR - CAL_PRESSURE_WIN);
            bool over = p > (CAL_TARGET_BAR + CAL_PRESSURE_WIN);
            bool timeout = (now - s_cal.stepTs) > CAL_PRESSURE_TIMEOUT_MS;
            if (ok || timeout || over) {
                {
                    char msg[96];
                    snprintf(msg, sizeof(msg), "[CALIB] pressure %.1f bar (target %.1f)", p, CAL_TARGET_BAR);
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                }
                s_cal.step = CAL_SENSOR_CHECK;
                s_cal.stepTs = now;
                s_cal.refRaw = raw;
                break;
            }
            if (!s_pumpKickActive || ((now - s_cal.stepTs) > 800)) {
                requestPumpStart();
                s_pumpKickActive = true;
            }
            break;
        }
        case CAL_SENSOR_CHECK: {
            if (!isfinite(raw) || raw < 0.05f || raw > 3.5f) {
                finishCalib(false, "SENSOR", now);
                return;
            }
            s_cal.step = CAL_FIND_DIR;
            s_cal.stepTs = now;
            s_cal.refRaw = raw;
            // breakaway i?in daha y?ksek baslangi? duty
            s_cal.pulseDuty = clampDuty(900, g_pistonCalibData[piston]);
            sendLog("[CALIB] sensor OK, dir pulse");
            break;
        }
        case CAL_FIND_DIR: {
            targetDuty[valveIdx] = s_cal.pulseDuty;
            if ((now - s_cal.stepTs) >= CAL_PULSE_MS) {
                float delta = raw - s_cal.refRaw;
                if (fabsf(delta) > 0.0008f) {
                    int8_t dir = (delta > 0) ? 1 : -1;
                    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                        g_pistonCalibData[piston].direction = dir;
                        xSemaphoreGive(g_sharedMutex);
                    }
                    {
                        char msg[64];
                        snprintf(msg, sizeof(msg), "[CALIB] dir=%d", (int)g_pistonCalibData[piston].direction);
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                    }
                }
                s_cal.step = CAL_BREAKAWAY;
                s_cal.stepTs = now;
                s_cal.refRaw = raw;
                s_cal.pulseDuty = clampDuty(s_cal.pulseDuty, g_pistonCalibData[piston]);
                s_cal.breakaway = 0;
            }
            break;
        }
        case CAL_BREAKAWAY: {
            targetDuty[valveIdx] = s_cal.pulseDuty;
            if ((now - s_cal.stepTs) >= (CAL_PULSE_MS + CAL_SETTLE_MS)) {
                float delta = fabsf(raw - s_cal.refRaw);
                if (delta > 0.0012f) {
                    s_cal.breakaway = s_cal.pulseDuty;
                    int8_t dirSign = (raw >= s_cal.refRaw) ? 1 : -1;
                    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                        g_pistonCalibData[piston].direction = dirSign;
                        xSemaphoreGive(g_sharedMutex);
                    }
                    s_cal.step = CAL_SEEK_MIN;
                    s_cal.stepTs = now;
                    s_cal.lastRaw = raw;
                    s_cal.seekStartMs = now;
                    {
                        char msg[80];
                        snprintf(msg, sizeof(msg), "[CALIB] breakaway duty=%u", (unsigned)s_cal.breakaway);
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                    }
                } else {
                    s_cal.pulseDuty = clampDuty(s_cal.pulseDuty + CAL_PULSE_STEP, g_pistonCalibData[piston]);
                    s_cal.refRaw = raw;
                    s_cal.stepTs = now;
                    if (s_cal.pulseDuty >= g_pistonCalibData[piston].duty_max) {
                        finishCalib(false, "BREAK_TMO", now);
                        return;
                    }
                }
            }
            break;
        }
                case CAL_SEEK_MIN: {
            const PistonCalibData &cd = g_pistonCalibData[piston];
            uint16_t duty = (cd.direction > 0) ? cd.duty_min : cd.duty_max; // y?ne g?re tek limit
            duty = clampDuty(duty, cd);
            if (s_cal.breakaway) {
                uint16_t bw = clampDuty(s_cal.breakaway + CAL_SEEK_MARGIN, cd);
                if (bw > duty) duty = bw;
            }
            s_cal.lastSeekDuty = duty;
            targetDuty[valveIdx] = duty;
            if (fabsf(raw - s_cal.lastRaw) < CAL_STALL_THRESH) {
                if ((now - s_cal.stepTs) > CAL_STALL_MS) {
                    s_cal.minRaw = raw;
                    s_cal.step = CAL_SEEK_MAX;
                    s_cal.stepTs = now;
                    s_cal.seekStartMs = now;
                    {
                        char msg[96];
                        snprintf(msg, sizeof(msg), "[CALIB] min raw=%.4f duty=%u", raw, (unsigned)duty);
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                    }
                }
            } else {
                s_cal.lastRaw = raw;
                s_cal.stepTs = now;
            }
            break;
        }
        case CAL_SEEK_MAX: {
            const PistonCalibData &cd = g_pistonCalibData[piston];
            uint16_t duty = (cd.direction > 0) ? cd.duty_max : cd.duty_min; // ters limit
            duty = clampDuty(duty, cd);
            if (s_cal.breakaway) {
                uint16_t bw = clampDuty(s_cal.breakaway + CAL_SEEK_MARGIN, cd);
                if (cd.direction > 0 && bw > duty) duty = bw;
            }
            targetDuty[valveIdx] = duty;
            if (fabsf(raw - s_cal.lastRaw) < CAL_STALL_THRESH) {
                if ((now - s_cal.stepTs) > CAL_STALL_MS) {
                    s_cal.maxRaw = raw;
                    s_cal.step = CAL_VERIFY_MID;
                    s_cal.stepTs = now;
                    {
                        char msg[96];
                        snprintf(msg, sizeof(msg), "[CALIB] max raw=%.4f duty=%u", raw, (unsigned)duty);
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                    }
                }
            } else {
                s_cal.lastRaw = raw;
                s_cal.stepTs = now;
            }
            if ((now - s_cal.seekStartMs) > CAL_SEEK_FALLBACK_MS) {
                s_cal.maxRaw = raw;
                s_cal.step = CAL_VERIFY_MID;
                s_cal.stepTs = now;
                {
                    char msg[96];
                    snprintf(msg, sizeof(msg), "[CALIB] max fallback raw=%.4f duty=%u", raw, (unsigned)duty);
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                }
            }
            break;
        }
        case CAL_VERIFY_MID: {
            PistonCalibData cd{};
            if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                cd = g_pistonCalibData[piston];
                xSemaphoreGive(g_sharedMutex);
            }
            // E?er max < min ise ?l??mleri ters y?nde alm???z; swap + y?n? ?evir
            if (s_cal.maxRaw < s_cal.minRaw) {
                float tmp = s_cal.maxRaw;
                s_cal.maxRaw = s_cal.minRaw;
                s_cal.minRaw = tmp;
                cd.direction = -cd.direction;
                {
                    char msg[96];
                    snprintf(msg, sizeof(msg), "[CALIB] swapped min/max, dir=%d", (int)cd.direction);
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                }
            }
            float span = (s_cal.maxRaw - s_cal.minRaw);
            if (span < 0.010f) { // en az ~10 mV hareket bekle
                finishCalib(false, "SPAN", now);
                return;
            }
            float midRaw = s_cal.minRaw + 0.5f * span;
            cd.min_raw = (uint16_t)(s_cal.minRaw * 10000.0f);
            cd.max_raw = (uint16_t)(s_cal.maxRaw * 10000.0f);
            cd.mid_raw = (uint16_t)(midRaw * 10000.0f);
            cd.duty_breakaway = s_cal.breakaway ? s_cal.breakaway : cd.duty_breakaway;
            copyDefaultsIfEmpty(cd);
            if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                g_pistonCalibData[piston] = cd;
                xSemaphoreGive(g_sharedMutex);
            }
            s_cal.step = CAL_FF_SWEEP;
            s_cal.ffIdx = 0;
            s_cal.ffAcc = 0;
            s_cal.ffSamples = 0;
            s_cal.stepTs = now;
            break;
        }
        case CAL_FF_SWEEP: {
            PistonCalibData cd{};
            if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                cd = g_pistonCalibData[piston];
                xSemaphoreGive(g_sharedMutex);
            }
            if (s_cal.ffIdx >= PISTON_FF_BINS) {
                finishCalib(true, nullptr, now);
                return;
            }
            g_pressureGroupState[group].p_ref = cd.p_bins[s_cal.ffIdx];
            float span = (cd.max_raw - cd.min_raw) / 10000.0f;
            float midRaw = (cd.mid_raw) / 10000.0f;
            float x = clampf((raw - cd.min_raw / 10000.0f) / (span + 1e-6f), 0.0f, 1.0f);
            float e = 0.5f - x;
            float u_ff = cd.u_ff_map[s_cal.ffIdx];
            float u = u_ff + cd.direction * KP0_SLOW * e;
            uint16_t duty = clampDuty((uint16_t)u, cd);
            targetDuty[valveIdx] = duty;
            if (fabsf(e) < 0.02f) {
                s_cal.ffAcc += duty;
                s_cal.ffSamples++;
            }
            if ((now - s_cal.stepTs) > 700) {
                if (s_cal.ffSamples > 0) {
                    cd.u_ff_map[s_cal.ffIdx] = s_cal.ffAcc / (float)s_cal.ffSamples;
                }
                s_cal.ffIdx++;
                s_cal.ffAcc = 0;
                s_cal.ffSamples = 0;
                s_cal.stepTs = now;
                if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    g_pistonCalibData[piston] = cd;
                    xSemaphoreGive(g_sharedMutex);
                }
            }
            (void)midRaw;
            break;
        }
        case CAL_DONE:
        case CAL_ERROR:
        default:
            finishCalib(false, "UNK", now);
            break;
    }
}

static void updatePressureLoops(float /*dt*/){
    for (int g = 0; g < 2; ++g) {
        auto &pg = g_pressureGroupState[g];
        if (pg.p_ref < 0.0f) {
            pg.cmd = 0;
            pg.integ = 0;
            continue;
        }
        float over = pg.p_meas - pg.p_ref;   // pozitifse bas?n? fazla
        float cmd = 0.0f;
        if (over > 0.0f && pg.p_ref >= 0.0f) {
            cmd = PRESS_KP * over;
        }
        if (pg.p_ref <= 0.1f) cmd = PRESS_KP * pg.p_meas; // tamamen bo?alt
        if (cmd < 0.0f) cmd = 0.0f;
        if (cmd > PRESS_CMD_MAX) cmd = PRESS_CMD_MAX;
        pg.integ = 0.0f;
        pg.cmd = (uint16_t)cmd;
    }
}

static void updateHoldControllers(const PistonControlInput &in, uint16_t targetDuty[8], bool suppressLegacyHold){
    if (suppressLegacyHold) return;
    for (int i = 0; i < PISTON_CHANNEL_COUNT; ++i) {
        PistonRuntimeState rt{};
        PistonCalibData cd{};
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
            rt = g_pistonRuntime[i];
            cd = g_pistonCalibData[i];
            xSemaphoreGive(g_sharedMutex);
        }
        if (!rt.hold_mid_enable) continue;
        if (!cd.calibrated || cd.max_raw <= cd.min_raw) continue;
        float span = (cd.max_raw - cd.min_raw) / 10000.0f;
        float raw = in.hall_raw[i];
        float x = clampf((raw - (cd.min_raw / 10000.0f)) / (span + 1e-6f), 0.0f, 1.0f);
        float prev = s_prevX[i];
        if (prev == 0.0f) prev = x;
        float dx = (x - prev) / (in.dt_s > 1e-4f ? in.dt_s : 1e-3f);
        s_prevX[i] = x;
        rt.x_filt += X_ALPHA * (x - rt.x_filt);
        rt.v_est = rt.v_est + V_ALPHA * (dx - rt.v_est);
        rt.x_ref = 0.5f;
        rt.e = rt.x_ref - rt.x_filt;
        float absE = fabsf(rt.e);
        if (absE > E_FAST) rt.state = PistonRuntimeState::MOVE_FAST;
        else if (absE > E_HOLD) rt.state = PistonRuntimeState::MOVE_SLOW;
        else rt.state = PistonRuntimeState::HOLD;

        float p_meas = in.pressure_bar[PistonControl_GroupOf((PistonChannel)i)];
        float kpBase = (rt.state == PistonRuntimeState::MOVE_FAST) ? KP0_FAST : KP0_SLOW;
        float kdBase = (rt.state == PistonRuntimeState::MOVE_FAST) ? KD0_FAST : KD0_SLOW;
        float pForGain = fmaxf(p_meas, 10.0f);
        float kp = kpBase * (PREF_BAR / (pForGain));
        float kd = kdBase * (pForGain / (PREF_BAR + 1.0f));
        float u_ff = interpFF(cd, p_meas);
        float u = u_ff + cd.direction * (kp * rt.e - kd * rt.v_est);
        uint16_t duty = clampDuty((uint16_t)u, cd);
        if (cd.duty_breakaway > 0 && absE > E_HOLD) {
            if (cd.direction * (rt.e) > 0 && duty < cd.duty_breakaway) {
                duty = cd.duty_breakaway;
            }
        }
        // rate limit
        if (duty > rt.u_cmd && (duty - rt.u_cmd) > DUTY_RATE_LIMIT) duty = rt.u_cmd + DUTY_RATE_LIMIT;
        if (duty < rt.u_cmd && (rt.u_cmd - duty) > DUTY_RATE_LIMIT) duty = rt.u_cmd - DUTY_RATE_LIMIT;
        uint8_t vIdx = PistonControl_ValveIdx((PistonChannel)i);
        if (vIdx < 8) {
            targetDuty[vIdx] = duty;
        }

        rt.u_cmd = duty;
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
            g_pistonRuntime[i] = rt;
            xSemaphoreGive(g_sharedMutex);
        }

        // Adapt u_ff map
        if (rt.state == PistonRuntimeState::HOLD && absE < E_LEARN && fabsf(rt.v_est) < V_LEARN) {
            float alpha = 0.01f;
            float pbin0 = cd.p_bins[0];
            size_t bin = 0;
            for (size_t b=1; b<PISTON_FF_BINS; ++b){
                if (p_meas <= cd.p_bins[b]) { bin = b-1; break; }
                bin = b;
            }
            float updated = (1.0f - alpha) * cd.u_ff_map[bin] + alpha * duty;
            cd.u_ff_map[bin] = updated;
            if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
                g_pistonCalibData[i].u_ff_map[bin] = updated;
                xSemaphoreGive(g_sharedMutex);
            }
            (void)pbin0;
        }
    }
}

void PistonControl_Init() {
    PistonCalibStorage_Begin();
    PistonCalibStorage_LoadAll(g_pistonCalibData);
    for (int i = 0; i < PISTON_CHANNEL_COUNT; ++i) {
        copyDefaultsIfEmpty(g_pistonCalibData[i]);
        g_pistonRuntime[i].x_ref = 0.5f;
        g_pistonRuntime[i].state = PistonRuntimeState::IDLE;
    }
}

int PistonControl_IndexFromName(const char *name) {
    if (!name) return -1;
    for (int i = 0; i < PISTON_CHANNEL_COUNT; ++i) {
        if (!strcasecmp(name, kPistonAxisConfig[i].name)) return i;
    }
    return -1;
}

uint8_t PistonControl_GroupOf(PistonChannel ch) {
    if (ch >= PISTON_CHANNEL_COUNT) return 0;
    return kPistonAxisConfig[ch].pressureGroup;
}

uint8_t PistonControl_ValveIdx(PistonChannel ch) {
    if (ch >= PISTON_CHANNEL_COUNT) return 0xFF;
    return kPistonAxisConfig[ch].valveIdx;
}

uint8_t PistonControl_SupportIdx(PistonChannel ch) {
    if (ch >= PISTON_CHANNEL_COUNT) return 0xFF;
    return kPistonAxisConfig[ch].supportIdx;
}

void PistonControl_HandleCalibCommand(const PistonCalibCommand &cmd) {
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        g_pistonCalibCmd = cmd;
        xSemaphoreGive(g_sharedMutex);
        portENTER_CRITICAL(&g_portMux);
        g_pistonCalibCmdSeq++;
        portEXIT_CRITICAL(&g_portMux);
    }
}

void PistonControl_Update(const PistonControlInput &in, uint16_t targetDuty[8], bool suppressLegacyHold) {
    // 1) g_pistonCalibCmd taleplerini yakala
    if (g_pistonCalibCmdSeq != s_lastCalibSeq) {
        PistonCalibCommand cmd{};
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
            cmd = g_pistonCalibCmd;
            xSemaphoreGive(g_sharedMutex);
        }
        s_lastCalibSeq = g_pistonCalibCmdSeq;
        if (cmd.action == PistonCalibCommand::START_ONE && !s_cal.active) {
            startCalib(cmd.piston, in.now_ms);
        } else if (cmd.action == PistonCalibCommand::START_ALL && !s_cal.active) {
            s_calAllPending = true;
            s_calAllNext = 0;
            startCalib(0, in.now_ms);
        } else if (cmd.action == PistonCalibCommand::CLEAR_ALL) {
            PistonCalibStorage_ClearAll();
        } else if (cmd.action == PistonCalibCommand::CLEAR_ONE) {
            PistonCalibStorage_Clear(cmd.piston);
        } else if (cmd.action == PistonCalibCommand::STOP) {
            // Kalibrasyonu durdur
            s_cal.active = false;
            s_calAllPending = false;
            sendLog("[CALIB] Stopped by user");
        } else if (cmd.action == PistonCalibCommand::FIND_HOLD) {
            // Hold PWM bulma - Dither yöntemi ile
            if (!s_holdFind.active) {
                startFindHold(cmd.piston);
            }
        }
    }

    // 2) basin? ?l??lerini yay?nla
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        g_pressureGroupState[0].p_meas = in.pressure_bar[0];
        g_pressureGroupState[1].p_meas = in.pressure_bar[1];
        xSemaphoreGive(g_sharedMutex);
    }
    bool overLimit[2] = { in.pressure_bar[0] > PRESSURE_LIMIT_BAR, in.pressure_bar[1] > PRESSURE_LIMIT_BAR };
    if (overLimit[0] || overLimit[1]) {
        // H?zl? bo?alt: ilgili PCV'yi a?, referans? s?f?rla ama state machine'i ?ld?rme
        for (int g = 0; g < 2; ++g) {
            if (overLimit[g]) {
                g_pressureGroupState[g].cmd = PCV_DRAIN_DUTY;
                g_pressureGroupState[g].p_ref = 0.0f;
                g_pressureGroupState[g].integ = 0.0f;
                targetDuty[PCV_INDEX[g]] = PCV_DRAIN_DUTY;
            }
        }
        sendLog("[CALIB] over-pressure -> PCV drain");
    }

    // 3) Kalibrasyon state machine
    updateCalib(in, targetDuty);
    
    // 3.5) Hold PWM bulma state machine
    updateFindHold();

    // 4) Hold mid kontrol
    updateHoldControllers(in, targetDuty, suppressLegacyHold);

    // 5) basinc setpoint secimi
    float pRef[2] = {0.0f, 0.0f};
    // mevcut referanslari koru ki diger testleri etkilemeyelim
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        pRef[0] = g_pressureGroupState[0].p_ref;
        pRef[1] = g_pressureGroupState[1].p_ref;
        xSemaphoreGive(g_sharedMutex);
    }
    if (s_cal.active) {
        pRef[s_cal.group] = CAL_TARGET_BAR;
    }
    for (int i = 0; i < PISTON_CHANNEL_COUNT; ++i) {
        PistonRuntimeState rt{};
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            rt = g_pistonRuntime[i];
            xSemaphoreGive(g_sharedMutex);
        }
        if (!rt.hold_mid_enable) continue;
        uint8_t g = PistonControl_GroupOf((PistonChannel)i);
        if (rt.state == PistonRuntimeState::MOVE_FAST || rt.state == PistonRuntimeState::MOVE_SLOW) {
            if (pRef[g] < MOVE_SET_BAR) pRef[g] = MOVE_SET_BAR;
        } else {
            if (pRef[g] < HOLD_SET_BAR) pRef[g] = HOLD_SET_BAR;
        }
    }
    for (int g = 0; g < 2; ++g) {
        g_pressureGroupState[g].p_ref = pRef[g];
    }

    // 6) basinc PI
    updatePressureLoops(in.dt_s);
    // PCV komutlarini targetDuty'ye uygula
    for (int g = 0; g < 2; ++g) {
        uint16_t cmd = g_pressureGroupState[g].cmd;
        if (overLimit[g]) cmd = PCV_DRAIN_DUTY;
        if (s_cal.active && g == s_cal.group) {
            cmd = SUPPORT_VALVE_DUTY; // kalibrasyonda PCV sabit acik
        }
        targetDuty[PCV_INDEX[g]] = cmd;
    }
}

void PistonControl_TelemetrySnapshot(PistonRuntimeState (&out)[PISTON_CHANNEL_COUNT], PressureGroupState (&pg)[2]) {
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        for (int i = 0; i < PISTON_CHANNEL_COUNT; ++i) out[i] = g_pistonRuntime[i];
        for (int i = 0; i < 2; ++i) pg[i] = g_pressureGroupState[i];
        xSemaphoreGive(g_sharedMutex);
    }
}

void PistonControl_GetCalibProgress(PistonCalibProgress &out){
    out = s_calProg;
}


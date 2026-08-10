#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "Tasks.h"
#include "Shared.h"
#include "ValveCurrentControl.h"
#include <Preferences.h>

// ===============================
// PWM konfig
// ===============================
static const int PWM_FREQ = 3000;
static const int PWM_RES  = 12;
static const uint16_t DUTY_MAX = (1u << PWM_RES) - 1;
static const uint16_t BTN_TOGGLE_DUTY = 2000;   // butona basınca verilecek PWM
// ==== Hızlı DRAIN ayarları (gerekirse GUI'den override edilebilir) ====
static const uint16_t DRAIN_PCv_DUTY = 2000;   // N436, N440 için "tam açık" PWM
static const uint16_t DRAIN_PAIR_DUTY = 2000;  // Diğer valf çiftleri için PWM
static const uint32_t DRAIN_STEP_MS   = 1000;   // Her adımda yeni çift ekleme aralığı
static const uint32_t DRAIN_CHECK_MS  = 500;   // Basınç kontrol periyodu
static const uint32_t DRAIN_TIMEOUT_MS= 15000; // Maks. toplam süre
static const float    DRAIN_TARGET_BAR= 10.0f; // Hedef eşik

//static constexpr float PISTON_DEFAULT_STROKE_MM = 26.0f;
static constexpr float PISTON_K1_K2_STROKE_MM = 15.0f;  // K1/K2 kavrama stroke
static const float PISTON_STROKE_MM[PISTON_CHANNEL_COUNT] = {
    PISTON_DEFAULT_STROKE_MM,  // PISTON_5_7
    PISTON_DEFAULT_STROKE_MM,  // PISTON_1_3
    PISTON_DEFAULT_STROKE_MM,  // PISTON_2_4
    PISTON_DEFAULT_STROKE_MM,  // PISTON_6_R
    PISTON_K1_K2_STROKE_MM,    // PISTON_K1 (N435)
    PISTON_K1_K2_STROKE_MM     // PISTON_K2 (N439)
};

static constexpr uint16_t DEFAULT_CAL_PWM = 2000;
static constexpr uint16_t DEFAULT_CAL_PERIOD_MS = 100;
static constexpr uint16_t DEFAULT_CAL_SETTLE_MS = 3000;
static constexpr float    CAL_PRESSURE_TARGET_BAR = 50.0f;
static constexpr float    CAL_PRESSURE_WINDOW_BAR = 10.0f;
static constexpr uint32_t PUMP_RETRY_INTERVAL_MS  = 8000;
static constexpr uint32_t CAL_PRESSURE_WAIT_TIMEOUT_MS = 15000;
static constexpr uint32_t CAL_NOCHANGE_TIMEOUT_MS = 1500;
static constexpr uint32_t CAL_MIN_SAMPLE_MS = 1200;
static constexpr uint16_t SUPPORT_VALVE_DUTY = 1553;  // ~650mA: (0.65 * 7Ω / 12V) * 4095
static constexpr uint16_t HOLD_PWM_FAST_FORWARD  = 1500;  // manuel a?ma
static constexpr uint16_t HOLD_PWM_FAST_REVERSE  = 950;   // manuel kapama
static constexpr uint16_t HOLD_PWM_FINE_FORWARD  = 1500;  // basit kontrol i?in ayn?
static constexpr uint16_t HOLD_PWM_FINE_REVERSE  = 950;
static constexpr uint16_t HOLD_PWM_NEUTRAL       = 1100;  // ortada tutma
static constexpr float    HOLD_DEFAULT_TOL_MM    = 0.5f;
static constexpr float    HOLD_NEAR_BAND_MM      = 1.5f;
static constexpr float    HOLD_FILTER_ALPHA      = 0.2f;
static constexpr uint32_t HOLD_FORCE_DURATION_MS = 600;
static constexpr uint32_t HOLD_CENTER_TIMEOUT_MS = 5000;

struct PistonCalRuntime {
    // Kalibrasyon akışı:
    // 1. WAIT_PRESSURE: Önce basınç doldur (tüm valfler kapalı)
    // 2. OPEN_VALVES: Basınç valflerini aç, 1sn bekle (pistonlar kapansın)
    // 3. READ_CLOSED: Kapalı konum oku
    // 4. OPEN_PISTON: Pistonu aç
    // 5. WAIT_STABLE: Değer değişimi durana kadar bekle
    // 6. READ_OPEN: Açık konum oku
    // 7. FIND_HOLD_PARK: Pistonu ortaya konumlandır (hold PWM bulmak için)
    // 8. FIND_HOLD_MEASURE: Drift ölç ve PWM ayarla
    // 9. COMPLETE: Bitir
    enum Stage : uint8_t { 
        IDLE = 0, 
        WAIT_PRESSURE,      // Önce basınç doldur (tüm valfler kapalı)
        OPEN_VALVES,        // Basınç valflerini aç, bekle (pistonlar kapansın)
        READ_CLOSED,        // Kapalı konum oku
        OPEN_PISTON,        // Pistonu aç
        WAIT_STABLE,        // Değer stabilize olana kadar bekle
        READ_OPEN,          // Açık konum oku
        CLOSE_FOR_SCAN,         // PWM eşik taraması için pistonu kapat
        FIND_PWM_OPEN_THRESH,   // PWM'i yükselterek açılma eşiğini bul
        FIND_PWM_CLOSE_THRESH,  // PWM'i düşürerek kapanma eşiğini bul
        FIND_HOLD_PARK,     // Pistonu ortaya konumlandır
        FIND_HOLD_MEASURE,  // Drift ölç ve binary search
        COMPLETE 
    } stage = IDLE;
    bool     active = false;
    PistonChannel piston = PISTON_5_7;
    int      valveIdx = -1;
    int      supportIdx = -1;  // Destek valfi (N436 veya N440)
    uint16_t pwmDuty = DEFAULT_CAL_PWM;
    uint16_t settleMs = DEFAULT_CAL_SETTLE_MS;
    uint32_t stageStartMs = 0;
    float    closedRaw = 0.0f;  // Kapalı konum TMAG değeri - sensör 1
    float    openRaw = 0.0f;    // Açık konum TMAG değeri - sensör 1
    float    closedRaw2 = 0.0f; // Kapalı konum TMAG değeri - sensör 2 (K1/K2)
    float    openRaw2 = 0.0f;   // Açık konum TMAG değeri - sensör 2 (K1/K2)
    float    lastRaw = 0.0f;    // Son okunan değer (stabilite kontrolü için)
    uint32_t lastChangeMs = 0;  // Son değişim zamanı
    float    strokeMm = PISTON_DEFAULT_STROKE_MM;
    bool     pumpRequested = false;
    float    pressureTargetBar = CAL_PRESSURE_TARGET_BAR;
    uint32_t lastPumpReqMs = 0;
    // Hold PWM bulma için ek alanlar
    bool     findHold = false;      // Hold PWM bulma aktif mi?
    uint16_t holdPwmLo = 800;       // Düşük PWM (geri çekme)
    uint16_t holdPwmHi = 1400;      // Yüksek PWM (ileri itme)
    uint16_t holdPwmTest = 1000;    // Test edilen hold PWM
    uint16_t holdPwmResult = 0;     // Bulunan hold PWM
    float    midRaw = 0.0f;         // Hesaplanan orta konum (non-lineer, kullanılmıyor)
    float    measuredMidRaw = 0.0f; // ÖLÇÜLEN gerçek orta konum (hold PWM bulunduğunda)
    float    holdLastRaw = 0.0f;    // Son okunan değer
    uint32_t holdStableStartMs = 0; // Stabilite başlangıç zamanı
    bool     holdAtMid = false;     // Ortaya ulaştı mı?
    uint8_t  holdIteration = 0;     // İterasyon sayısı
    // PWM eşik bulma için yeni alanlar
    uint16_t pwmOpenThresh = 0;     // Açılma başlangıç PWM'i (bu değerin üstünde açılır)
    uint16_t pwmCloseThresh = 0;    // Kapanma başlangıç PWM'i (bu değerin altında kapanır)
    uint16_t pwmScanCurrent = 0;    // Tarama sırasında mevcut PWM
    float    threshStartRaw = 0.0f; // Eşik araması başlangıç pozisyonu
    bool     threshMovementDetected = false; // Hareket algılandı mı?
    uint32_t lastPwmStepMs = 0;     // Son PWM adım zamanı
    // Ardışık kalibrasyon için
    bool     calibrateAll = false;  // Tüm pistonları kalibre et modu
    uint8_t  nextPiston = 0;        // Sonraki piston indeksi
};

static PistonCalRuntime s_pistonCal{};
struct PistonHoldRuntime {
    enum Stage : uint8_t {
        STAGE_IDLE = 0,
        STAGE_FORCE_OPEN,
        STAGE_APPROACH,
        STAGE_CENTER,
        STAGE_DIRECT_OPEN,
        STAGE_DIRECT_CLOSE
    } stage = STAGE_IDLE;
    bool active = false;
    PistonChannel piston = PISTON_5_7;
    float   targetRaw = 0.0f;
    float   targetMm = NAN;
    float   tolerance = HOLD_DEFAULT_TOL_MM;   // raw fallback i??in kullan??lacak
    float   toleranceMm = HOLD_DEFAULT_TOL_MM;
    float   strokeMm = PISTON_MANUAL_STROKE_MM;
    int     valveIdx = -1;
    int     supportIdx = -1;
    bool    directionPositive = true;
    float   filteredRaw = 0.0f;
    float   refOpen = 0.0f;
    float   refClosed = 0.0f;
    uint8_t targetState = PISTON_REF_MID;
    uint32_t stageStartMs = 0;
    uint32_t commandStartMs = 0;
    bool faultNotified = false;
};
static PistonHoldRuntime s_pistonHold[PISTON_CHANNEL_COUNT]{};

static inline void pistonHoldSetStage(PistonHoldRuntime &ctrl, PistonHoldRuntime::Stage st){
    ctrl.stage = st;
    ctrl.stageStartMs = millis();
}

static inline void publishPistonState(PistonChannel piston, uint8_t state){
    if (!g_sharedMutex) return;
    uint8_t mapped = 0;
    switch (state) {
        case PISTON_REF_CLOSED: mapped = 1; break;
        case PISTON_REF_MID:    mapped = 2; break;
        case PISTON_REF_OPEN:   mapped = 3; break;
        default: mapped = state;
    }
    if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        g_pistonState[piston] = mapped;
        xSemaphoreGive(g_sharedMutex);
    }
}

// Manuel referanslardan (CLOSED/MID/OPEN) ham hall degerini mm'ye cevir
static bool manualRawToMm(const PistonManualReference &ref, float raw, float strokeMm, float &outMm) {
    uint8_t mask = ref.validMask;
    if (((mask & (1u << PISTON_REF_CLOSED)) == 0) || ((mask & (1u << PISTON_REF_OPEN)) == 0)) return false;
    float closed = ref.raw[PISTON_REF_CLOSED];
    float open   = ref.raw[PISTON_REF_OPEN];
    float span = open - closed;
    if (fabsf(span) < 1e-4f) return false;
    float mid = (mask & (1u << PISTON_REF_MID)) ? ref.raw[PISTON_REF_MID] : (closed + open) * 0.5f;
    if (raw <= closed) { outMm = 0.0f; return true; }
    if (raw >= open)   { outMm = strokeMm; return true; }
    if (raw <= mid) {
        float denom = mid - closed;
        if (fabsf(denom) < 1e-4f) denom = span;
        outMm = 0.5f * strokeMm * (raw - closed) / denom;
    } else {
        float denom = open - mid;
        if (fabsf(denom) < 1e-4f) denom = span;
        outMm = 0.5f * strokeMm + 0.5f * strokeMm * (raw - mid) / denom;
    }
    return true;
}

#ifndef PRESS_BAR_PER_UNIT
#define PRESS_BAR_PER_UNIT 1.0f  // Senin projende bar ise 1.0, Volt›bar ise 10.0 gibi ayarla
#endif

extern float g_pressure0_V;      // mevcut telemetri değişkenin
static inline float readPressureBar() {
    // Eğer g_pressure0_V zaten bar ise PRESS_BAR_PER_UNIT=1.0 yap
    g_pressure0_V=g_pumpPub.bar;
    return g_pressure0_V * PRESS_BAR_PER_UNIT;
}

// Test kodundaki sıra ile aynı:
// 0:N433  1:N436  2:N434  3:N435  4:N438  5:N440  6:N439  7:N437
static const uint8_t VALVE_PIN[8] = {
    N433_PWM_OUT, N436_PWM_OUT, N434_PWM_OUT, N435_PWM_OUT,
    N438_PWM_OUT, N440_PWM_OUT, N439_PWM_OUT, N437_PWM_OUT
};
static const char *VALVE_NAME[8] = {
    "N433","N436","N434","N435","N438","N440","N439","N437"
};
static const uint8_t VALVE_CH[8] = {0,1,2,3,4,5,6,7};
static uint16_t s_lastDuty[8] = {0};

static const int PISTON_VALVE_INDEX[PISTON_CHANNEL_COUNT] = {
    2,  // PISTON_5_7 -> N434
    0,  // PISTON_1_3 -> N433
    7,  // PISTON_2_4 -> N437
    4,  // PISTON_6_R -> N438
    3,  // PISTON_K1  -> N435
    6   // PISTON_K2  -> N439
};
static const int PISTON_SUPPORT_VALVE_INDEX[PISTON_CHANNEL_COUNT] = {
    1,  // PISTON_5_7 destek: N436
    1,  // PISTON_1_3 destek: N436
    5,  // PISTON_2_4 destek: N440
    5,  // PISTON_6_R destek: N440
    1,  // PISTON_K1  destek: N436
    5   // PISTON_K2  destek: N440
};

// Piston -> TMAG kanal eşleşmesi (DRV yerine TMAG kullanılıyor!)
// K1/K2 için TMAG-2 sensörleri birincil olarak kullanılıyor (TMAG_CH_K1_2, TMAG_CH_K2_2)
static const uint8_t PISTON_TO_TMAG[PISTON_CHANNEL_COUNT] = {
    TMAG_CH_5_7,   // PISTON_5_7 -> TMAG_CH_5_7 (1)
    TMAG_CH_1_3,   // PISTON_1_3 -> TMAG_CH_1_3 (0)
    TMAG_CH_2_4,   // PISTON_2_4 -> TMAG_CH_2_4 (2)
    TMAG_CH_6_R,   // PISTON_6_R -> TMAG_CH_6_R (3)
    TMAG_CH_K1_2,  // PISTON_K1  -> TMAG_CH_K1_2 (5) [birincil]
    TMAG_CH_K2_2   // PISTON_K2  -> TMAG_CH_K2_2 (7) [birincil]
};
// Piston -> TMAG ikinci sensör kanal (-1 = sensör 2 yok, sadece K1/K2 için)
static const int8_t PISTON_TO_TMAG2[PISTON_CHANNEL_COUNT] = {
    -1,                    // PISTON_5_7 -> sensör 2 yok
    -1,                    // PISTON_1_3
    -1,                    // PISTON_2_4
    -1,                    // PISTON_6_R
    (int8_t)TMAG_CH_K1_1,  // PISTON_K1  -> TMAG_CH_K1_1 (4) [ikincil]
    (int8_t)TMAG_CH_K2_1   // PISTON_K2  -> TMAG_CH_K2_1 (6) [ikincil]
};
// GUI phase indexi (manual_control_page / auto_control_page phase_names dizisi)
static const uint8_t PISTON_TO_GUI_PHASE[PISTON_CHANNEL_COUNT] = {
    3,  // PISTON_5_7 -> P5-7
    2,  // PISTON_1_3 -> P1-3
    4,  // PISTON_2_4 -> P2-4
    5,  // PISTON_6_R -> P6-R
    6,  // PISTON_K1  -> K1
    7   // PISTON_K2  -> K2
};

struct StoredPistonRefEntry {
    float   raw[3];       // CLOSED, MID, OPEN raw values
    uint8_t validMask;
    uint8_t reserved[3];
    // V2: kalibrasyon verileri
    uint16_t min_raw;
    uint16_t max_raw;
    uint16_t mid_raw;
    uint16_t duty_hold;
    uint8_t  calibrated;
    uint8_t  pad[3];
};

static Preferences s_pistonRefPrefs;
static bool        s_pistonRefPrefsReady = false;

static bool PistonRefPrefs_Begin(){
    if (!s_pistonRefPrefsReady) {
        s_pistonRefPrefsReady = s_pistonRefPrefs.begin("pistonref", false);
    }
    return s_pistonRefPrefsReady;
}

static void PistonRefPrefs_LoadAll(){
    if (!PistonRefPrefs_Begin()) {
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[REF] prefs init failed");
        return;
    }
    int loadedCount = 0;
    for (int i = 0; i < PISTON_CHANNEL_COUNT; ++i) {
        StoredPistonRefEntry entry{};
        char key[8];
        snprintf(key, sizeof(key), "p%d", i);
        size_t len = s_pistonRefPrefs.getBytes(key, &entry, sizeof(entry));
        if (len >= 16 && entry.validMask != 0) {  // En az eski format
            if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                for (int st = 0; st < 3; ++st) g_pistonManualRef[i].raw[st] = entry.raw[st];
                g_pistonManualRef[i].validMask = entry.validMask;
                
                // Yeni format: kalibrasyon verileri
                if (len == sizeof(entry) && entry.calibrated) {
                    g_pistonCalibData[i].calibrated = true;
                    g_pistonCalibData[i].min_raw = entry.min_raw;
                    g_pistonCalibData[i].max_raw = entry.max_raw;
                    g_pistonCalibData[i].mid_raw = entry.mid_raw;
                    g_pistonCalibData[i].duty_hold = entry.duty_hold;
                    loadedCount++;
                }
                xSemaphoreGive(g_sharedMutex);
            }
        }
    }
    {
        char msg[64];
        snprintf(msg, sizeof(msg), "[REF] loaded %d calibrated pistons", loadedCount);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
    }
}

static void PistonRefPrefs_Save(int idx){
    if (!PistonRefPrefs_Begin()) return;
    if (idx < 0 || idx >= PISTON_CHANNEL_COUNT) return;
    StoredPistonRefEntry entry{};
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        for (int st = 0; st < 3; ++st) entry.raw[st] = g_pistonManualRef[idx].raw[st];
        entry.validMask = g_pistonManualRef[idx].validMask;
        // Kalibrasyon verilerini de kaydet
        entry.calibrated = g_pistonCalibData[idx].calibrated ? 1 : 0;
        entry.min_raw = g_pistonCalibData[idx].min_raw;
        entry.max_raw = g_pistonCalibData[idx].max_raw;
        entry.mid_raw = g_pistonCalibData[idx].mid_raw;
        entry.duty_hold = g_pistonCalibData[idx].duty_hold;
        xSemaphoreGive(g_sharedMutex);
    } else {
        return;
    }
    char key[8];
    snprintf(key, sizeof(key), "p%d", idx);
    s_pistonRefPrefs.putBytes(key, &entry, sizeof(entry));
}

static const char* pistonRefStateName(uint8_t state){
    switch (state) {
        case PISTON_REF_CLOSED: return "CLOSED";
        case PISTON_REF_MID:    return "MID";
        case PISTON_REF_OPEN:   return "OPEN";
        default: return "?";
    }
}

static void handlePistonReferenceRequest(const PistonReferenceRequest &req){
    if (req.piston >= PISTON_CHANNEL_COUNT) return;
    if (req.state > PISTON_REF_OPEN) return;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_pistonManualRef[req.piston].raw[req.state] = req.rawValue;
        g_pistonManualRef[req.piston].validMask |= (1u << req.state);
        xSemaphoreGive(g_sharedMutex);
    } else {
        return;
    }
    PistonRefPrefs_Save(req.piston);
    {
        char msg[120];
        snprintf(msg, sizeof(msg), "[REF] stored piston=%d state=%s raw=%.4f",
                 req.piston, pistonRefStateName(req.state), req.rawValue);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
    }
}

static void closeGroupPcv(uint8_t piston) {
    // P0/P1/K1 -> N436 (idx 1); P2/P3/K2 -> N440 (idx 5)
    uint8_t pcvIdx = (piston == 0 || piston == 1 || piston == 4) ? 1 : 5;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
        g_valveCustomCurrent_mA[pcvIdx] = 0.0f;
        g_valveTargetDuty[pcvIdx] = 0;
        g_valveCustomMode[pcvIdx] = 0;  // pcv_pi_step'e kontrolu birak
        xSemaphoreGive(g_sharedMutex);
    }
    char msg[64];
    snprintf(msg, sizeof(msg), "[HOLD] pcv=%d cleared for piston=%d", pcvIdx, piston);
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
}

static void handlePistonHoldRequest(const PistonHoldRequest &req){
    if (req.piston >= PISTON_CHANNEL_COUNT) {
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[HOLD] invalid piston idx");
        return;
    }

    auto &ctrl = s_pistonHold[req.piston];

    // K1/K2 (piston index 4,5): yay sistemi var, hold kontrolü kararsız.
    // Enable isteği reddedilir; disable her zaman geçer (temizlik amaçlı).
    // Bkz. HOLD_TUNING_TODO.md "K1/K2 Karar" bölümü.
    if (req.enable && req.piston >= 4) {
        {
            char msg[80];
            snprintf(msg, sizeof(msg), "[HOLD] K1/K2 hold disabled (piston=%d ignored)", req.piston);
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        }
        return;
    }

    if (!req.enable) {
        bool gotMutex = false;
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_pistonRuntime[req.piston].hold_mid_enable = false;
            g_pistonRuntime[req.piston].state = PistonRuntimeState::IDLE;
            gotMutex = true;
            xSemaphoreGive(g_sharedMutex);
        }
        {
            char msg[80];
            snprintf(msg, sizeof(msg), "[HOLD] STOP piston=%d mutex=%d", req.piston, gotMutex ? 1 : 0);
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        }
        publishPistonState((PistonChannel)req.piston, PISTON_REF_CLOSED);
        closeGroupPcv(req.piston);
        ctrl = {};
        return;
    }

    // PISTON_REF_MID veya PISTON_REF_OPEN için hold kontrolü
    if (req.state == PISTON_REF_MID || req.state == PISTON_REF_OPEN) {
        bool calibrated = false;
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
            calibrated = g_pistonCalibData[req.piston].calibrated;
            xSemaphoreGive(g_sharedMutex);
        }
        if (!calibrated && req.piston < 4) {
            // P0-P3: kalibrasyon zorunlu
            {
                char msg[64];
                snprintf(msg, sizeof(msg), "[HOLD] calib missing piston=%d", req.piston);
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
            }
            return;
        }
        // K1/K2 (piston>=4): kalibre edilmemisse varsayilan akimlarla devam et
        // x_ref: 0.5 = orta, 1.0 = tam açık
        float xRef = (req.state == PISTON_REF_OPEN) ? 1.0f : 0.5f;
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
            g_pistonRuntime[req.piston].hold_mid_enable = true;
            g_pistonRuntime[req.piston].hold_init_needed = true;  // state makinesini sıfırla
            g_pistonRuntime[req.piston].x_ref = xRef;
            xSemaphoreGive(g_sharedMutex);
        }
        ctrl = {};
        {
            char msg[96];
            snprintf(msg, sizeof(msg), "[HOLD] control enabled piston=%d x_ref=%.1f", req.piston, xRef);
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        }
        return;
    }

    PistonManualReference ref{};
    PistonCalibrationTable cal{};
    float currentRaw = 0.0f;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        ref = g_pistonManualRef[req.piston];
        cal = g_pistonCalTable[req.piston];
        currentRaw = g_pistonHallRaw[req.piston];
        xSemaphoreGive(g_sharedMutex);
    } else {
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[HOLD] mutex busy");
        return;
    }

    uint8_t mask = ref.validMask;
    bool haveClosed = (mask & (1u << PISTON_REF_CLOSED)) != 0;
    bool haveOpen   = (mask & (1u << PISTON_REF_OPEN)) != 0;
    if (req.state == PISTON_REF_MID && ((mask & (1u << PISTON_REF_MID)) == 0)) {
        {
            char msg[96];
            snprintf(msg, sizeof(msg), "[HOLD] missing ref piston=%d state=%s", req.piston, pistonRefStateName(req.state));
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        }
        return;
    }
    if (req.state == PISTON_REF_MID && (!haveClosed || !haveOpen)) {
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[HOLD] refs missing for MID");
        return;
    }
    if (req.state == PISTON_REF_OPEN && !haveOpen) {
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[HOLD] refs missing for OPEN");
        return;
    }
    if (req.state == PISTON_REF_CLOSED && !haveClosed) {
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[HOLD] refs missing for CLOSED");
        return;
    }

    float strokeMm = cal.valid ? cal.stroke_mm : PISTON_MANUAL_STROKE_MM;
    float targetRaw = ref.raw[req.state];
    float targetMm = NAN;
    manualRawToMm(ref, targetRaw, strokeMm, targetMm);

    ctrl = {};
    ctrl.active = true;
    ctrl.piston = (PistonChannel)req.piston;
    ctrl.targetRaw = targetRaw;
    ctrl.targetMm = targetMm;
    ctrl.toleranceMm = (req.tolerance > 0.0005f) ? req.tolerance : HOLD_DEFAULT_TOL_MM;
    ctrl.tolerance = ctrl.toleranceMm; // raw fallback
    ctrl.strokeMm = strokeMm;
    ctrl.valveIdx = PISTON_VALVE_INDEX[req.piston];
    ctrl.supportIdx = PISTON_SUPPORT_VALVE_INDEX[req.piston];
    ctrl.refClosed = ref.raw[PISTON_REF_CLOSED];
    ctrl.refOpen = ref.raw[PISTON_REF_OPEN];
    ctrl.filteredRaw = currentRaw;
    ctrl.targetState = req.state;
    ctrl.commandStartMs = millis();
    ctrl.faultNotified = false;
    publishPistonState(ctrl.piston, ctrl.targetState);

    if (haveClosed && haveOpen) {
        ctrl.directionPositive = (ctrl.refOpen >= ctrl.refClosed);
    } else if (haveClosed) {
        ctrl.directionPositive = (targetRaw >= ctrl.refClosed);
    } else {
        ctrl.directionPositive = true;
    }

    if (req.state == PISTON_REF_MID) {
        pistonHoldSetStage(ctrl, PistonHoldRuntime::STAGE_FORCE_OPEN);
    } else {
        pistonHoldSetStage(ctrl, PistonHoldRuntime::STAGE_APPROACH);
    }

    {
        char msg[160];
        snprintf(msg, sizeof(msg), "[HOLD] enable piston=%d state=%s targetRaw=%.4f tol_mm=%.2f", req.piston, pistonRefStateName(req.state), ctrl.targetRaw, ctrl.toleranceMm);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
    }
}

// ============================================================================
// Piston Ortada Tutma - Yaklaşma + Frenleme + Adaptif Hold
// ============================================================================
//  Mantık:
//   Kapalıdan yaklaşma : yüksek PWM → frenleme bölgesi → holdPwm
//   Açıktan yaklaşma   : düşük PWM  → frenleme bölgesi → holdPwm
//   Hedefe varınca      : holdPwm sabit tut
//   Kayma algılanınca   : holdPwm yavaşça +/- 1 güncelle (ısı adaptasyonu)
// ============================================================================

// tmagZToMm tersini kullanarak hedef raw değerini hesapla (13mm = yarı strok)
// tmagZToMm: pos = strokeMm * (|zMin|^-n - |z|^-n) / (|zMin|^-n - |zMax|^-n)
// Ters: norm=0.5 → |z|^-n = 0.5*(|zMin|^-n + |zMax|^-n) → z = pow(...)^(-1/n)
static float holdComputeTargetRaw(const TMAGPistonCalib& cal, float midRawFallback) {
    if (!cal.valid) return (midRawFallback > 100.0f) ? midRawFallback : 5000.0f;
    float zMinF = (float)cal.zMin;
    float zMaxF = (float)cal.zMax;
    // Parçalı linear: zMid zaten tam 13mm'in raw değeri
    float zMidF = (float)cal.zMid;
    if (cal.zMid != 0 && fabsf(zMidF - zMinF) >= 10.0f && fabsf(zMaxF - zMidF) >= 10.0f) {
        return zMidF;
    }
    // Güç yasası ters çevirimi (n=0.39)
    float zA_min = fabsf(zMinF);
    float zA_max = fabsf(zMaxF);
    if (zA_min > 5.0f && zA_max > 5.0f) {
        const float n = 0.39f;
        float pw_min = powf(zA_min, -n);
        float pw_max = powf(zA_max, -n);
        float pw_mid = 0.5f * (pw_min + pw_max);
        if (pw_mid > 1e-10f) {
            float zA_mid = powf(pw_mid, -1.0f / n);
            return (zMaxF > zMinF) ? zA_mid : -zA_mid;
        }
    }
    // Fallback: lineer orta
    return (midRawFallback > 100.0f) ? midRawFallback : (zMinF + zMaxF) * 0.5f;
}

// Yaklaşma / frenleme bölgesi parametreleri
static constexpr float    HOLD_BRAKE_ZONE_MM    = 7.0f;   // Frenleme başlangıcı (mm)
static constexpr float    HOLD_DEAD_ZONE_MM     = 1.5f;   // Ölü bölge ±(mm)
static constexpr float    HOLD_NOMINAL_STROKE   = 26.0f;  // Nominal strok (mm) - raw/mm için
static constexpr uint16_t HOLD_BRAKE_MARGIN     = 60;     // Fren bölgesi kenarındaki PWM marjı
static constexpr uint16_t HOLD_PWM_ABS_MIN      = 600;    // Mutlak alt limit
static constexpr uint16_t HOLD_PWM_CTRL_MIN     = 902;    // Akım kontrolörü HOLD mod sınırı (≤900 = CLOSE mod devreye girer!)
static constexpr uint16_t HOLD_PWM_ABS_MAX      = 1750;   // Mutlak üst limit
// Isı adaptasyonu
static constexpr uint16_t HOLD_ADAPT_LIMIT      = 150;    // holdPwm kayabilir max ±PWM
static constexpr uint32_t HOLD_ADAPT_PERIOD_MS  = 200;    // En hızlı güncelleme aralığı
// Yaklaşma PWM kenetleme (holdPwm'den max sapma - akım sınırlama)
static constexpr uint16_t HOLD_APPROACH_OPEN_DELTA       = 400; // BR_O: fren bölgesinde açma max (+PWM) - yüksek threshold'lu pistonlar için (P2: openThresh~1440)
static constexpr uint16_t HOLD_APPROACH_OPEN_DELTA_FAST  = 300; // F_OP: uzak yaklaşmada açma max (+PWM)
static constexpr uint16_t HOLD_APPROACH_ABS_MAX          = 1200; // F_OP mutlak üst sınır - VALVE_MODE_HOLD sınırı ile AYNI (tutarlı)

// ============================================================================
// PID Controller Parametreleri (Yeni)
// ============================================================================
static constexpr float    HOLD_PID_KP_OPEN    = 30.0f;   // Açılma yönü Kp (PWM/mm hata)
static constexpr float    HOLD_PID_KP_CLOSE   = 40.0f;   // Kapanma yönü Kp (daha agresif, yay geri çeker)
static constexpr float    HOLD_PID_KI         = 1.5f;    // Integral katsayısı (düşürüldü - daha yavaş birikim)
static constexpr float    HOLD_PID_KD         = 12.0f;   // Derivative katsayısı (artırıldı - aşımı bastır)
static constexpr float    HOLD_PID_INTEGRAL_MAX = 30.0f; // Integral anti-windup sınırı (düşürüldü - 100 çok büyüktü)
static constexpr uint16_t HOLD_PWM_RATE_LIMIT   = 80;     // Max PWM değişim hızı (per period)
static constexpr uint16_t HOLD_PID_PERIOD_MS    = 50;     // PID güncelleme periyodu

// Hall sensör verisi doğrulama
static constexpr float    HOLD_MIN_VALID_RANGE  = 500.0f;  // Minimum geçerli raw range (kalibrasyon kontrolü)
static constexpr uint16_t HOLD_APPROACH_CLOSE_DELTA      = 100; // BR_C: fren bölgesinde kapama max (-PWM) - HOLD mod sınırında kalmalı
static constexpr uint16_t HOLD_APPROACH_CLOSE_DELTA_FAST = 130; // F_CL: uzak kapama max (-PWM) - HOLD mod sınırında kalmalı
// Sıcaklık kompanzasyonu (bobin direnci ısıyla artar → daha fazla PWM gerekir)
static constexpr float    HOLD_TEMP_REF_C    = 25.0f;  // Kalibrasyon referans sıcaklığı (°C)
static constexpr float    HOLD_TEMP_K        = 1.5f;   // Kompanzasyon katsayısı (PWM/°C)
static constexpr int16_t  HOLD_TEMP_MAX_COMP = 100;    // Maksimum kompanzasyon ±PWM

// ============================================================================
// AKIM-TABANLI HOLD KONTROL PARAMETRELERI (Yeni - Kullanıcı test değerleri)
// ============================================================================
// Piston başına kalibre edilmiş akım değerleri (mA)
// Test edilen değerler (23°C): 
//   P2-4: Açma=580, Kapatma=460, Tutma=500
//   P6-R: Açma=560, Kapatma=420, Tutma=500  
//   P1-3: Açma=590, Kapatma=470, Tutma=540
//   P5-7: Açma=550, Kapatma=435, Tutma=500
struct CurrentHoldParams {
    float openCurrent_mA;      // Hızlı açma akımı
    float closeCurrent_mA;     // Hızlı kapatma akımı
    float holdCurrent_mA;      // Tutma akımı (orta pozisyon)
    float slowOpenCurrent_mA;  // Yavaş açma (yaklaşma)
    float slowCloseCurrent_mA; // Yavaş kapatma (yaklaşma)
    float deadZone_mm;         // Ölü bölge ±mm
    float brakeZone_mm;        // Frenleme başlangıcı ±mm
    uint16_t valveIndex;       // Kontrol edilen valf indeksi
};

// Sıcaklık kompanzasyonu için akım katsayısı (bobin direnci artınca akım düşer, PWM artmalı)
// Yaklaşık: 10°C artış → %4 direnç artışı → +20-30mA gerekebilir
static constexpr float    HOLD_CURRENT_TEMP_K    = 1.0f;   // mA/°C kompanzasyon
static constexpr float    HOLD_CURRENT_MAX_COMP  = 30.0f;   // Maksimum ±30mA kompanzasyon

// Durum makinesi
enum HoldApproachState : uint8_t {
    HAS_INIT        = 0,  // Başlangıç - yön belirleniyor
    HAS_FAST_OPEN,        // Hızlı açılma (kapalı taraftan hedefe)
    HAS_BRAKE_OPEN,       // Frenleme (açılırken yavaşlatma)
    HAS_FAST_CLOSE,       // Hızlı kapanma (açık taraftan hedefe)
    HAS_BRAKE_CLOSE,      // Frenleme (kapanırken yavaşlatma)
    HAS_HOLDING,          // Hedede sabit tutma
    // ---- Hızlı Kalibrasyon (QC) - kalibre edilmemiş pistonlar için ----
    HAS_QC_PUMP_WAIT,     // Pompa basınç bekliyor
    HAS_QC_OPEN,          // Tam açık → max_raw öğren
    HAS_QC_CLOSE,         // Tam kapalı → min_raw öğren
    HAS_QC_FIND_HOLD,     // Hold PWM binary search
};

static HoldApproachState s_hasState[6]      = {};
static uint16_t          s_holdPwmCurrent[6]= {};  // Aktif hold PWM (adaptasyonla değişir)
static float             s_holdPwmBase[6]   = {};  // Kalibrasyon değeri (adaptasyon referansı)
static uint32_t          s_adaptLastMs[6]   = {};
static uint32_t          s_holdDbgMs[6]     = {};
static float             s_prevValidMm[6]   = {};  // Son geçerli mm ölçümü (sensör doyma koruması)
static uint8_t           s_holdOutCount[6]  = {};  // HOLD state'de ardışık büyük hata sayısı
static uint8_t           s_satCount[6]      = {};  // Ardışık sensör doyma (raw>=32700) sayacı
static uint16_t          s_holdPwmLastAdapted[PISTON_CHANNEL_COUNT] = {}; // Son uyarlanmış hold PWM (hold yeniden başlatılınca kullanılır)

// PID controller state (Yeni)
static float             s_pidIntegral[6]   = {};  // PID integral birikimi
static float             s_pidLastError[6]  = {};  // PID son hata (derivative için)
static uint32_t          s_pidLastMs[6]     = {};  // PID son çalışma zamanı
static uint16_t          s_lastPwmOut[6]    = {};  // Son PWM çıkışı (rate limiting için)
static float             s_filteredError[6] = {};  // Filtrelenmiş hata (gürültü azaltma)
// Hızlı Kalibrasyon (QC) değişkenleri
static float    s_qcMinRaw[4]   = {};
static float    s_qcMaxRaw[4]   = {};
static uint16_t s_qcFindPwm[4] = {};
static uint16_t s_qcFindLo[4]  = {};
static uint16_t s_qcFindHi[4]  = {};
static uint32_t s_qcTimer[4]   = {};
static uint8_t  s_qcStable[4]  = {};
static float    s_qcLastRaw[4] = {};

static void applyPistonHoldControl(uint16_t target[8], bool suppressed) {
    if (suppressed) return;

    for (int p = 0; p < PISTON_CHANNEL_COUNT; p++) {
        if (p >= 4) continue;  // K1/K2 hariç

        bool     holdEnabled = false;
        uint16_t holdPwm     = 1000;
        uint16_t openThresh  = 1400;
        uint16_t closeThresh = 700;
        float    minRaw = 0, maxRaw = 0, midRaw = 0, currentRaw = 0, currentMm = 0;
        int      valveIdx   = PISTON_VALVE_INDEX[p];
        int      supportIdx = PISTON_SUPPORT_VALVE_INDEX[p];
        TMAGPistonCalib tmagCal = {};

        if (valveIdx < 0) continue;

        bool initNeeded = false;
        bool calibrated = false;
        bool gotMutex = false;
        float xRef = 0.5f;  // Varsayılan: orta (0.0=kapalı, 0.5=orta, 1.0=açık)
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            holdEnabled = g_pistonRuntime[p].hold_mid_enable;
            initNeeded  = g_pistonRuntime[p].hold_init_needed;
            calibrated  = g_pistonCalibData[p].calibrated;
            xRef        = g_pistonRuntime[p].x_ref;
            if (initNeeded) g_pistonRuntime[p].hold_init_needed = false;  // hemen temizle
            if (holdEnabled && calibrated) {
                holdPwm = g_pistonCalibData[p].duty_hold;
                minRaw  = g_pistonCalibData[p].min_raw;
                maxRaw  = g_pistonCalibData[p].max_raw;
                midRaw  = g_pistonCalibData[p].mid_raw;
                if (g_pistonCalibData[p].duty_open_thresh  > 0) openThresh  = g_pistonCalibData[p].duty_open_thresh;
                if (g_pistonCalibData[p].duty_close_thresh > 0) closeThresh = g_pistonCalibData[p].duty_close_thresh;
                
                // P2 (piston 2) için kalibrasyon düzeltmesi: default değerler yerine
                // diğer pistonların tipik değerlerini kullan (gerçek threshold bulunamamış)
                static bool p2CalibFixed = false;
                if (p == 2 && !p2CalibFixed) {
                    if (openThresh == 1400 && closeThresh == 700) {
                        // Default değerler tespit edildi - P2'nin gerçek değerlerini kullan
                        openThresh = 1440;   // P1 ve P3 ile aynı
                        closeThresh = 800;   // P2'nin kapalı kalibrasyonu daha düşük
                        holdPwm = 1150;      // Orta pozisyon için tipik hold PWM (lineer FF yetersiz)
                        p2CalibFixed = true;
                        {
                            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[HOLD] p2 kalibrasyon duzeltmesi uygulandi");
                        }
                    }
                }
            }
            tmagCal = g_tmagPistonCalib[p];  // Ters dönüşüm için TMAG kalibrasyon verisi
            uint8_t tmagCh = PISTON_TO_TMAG[p];
            currentRaw = (float)g_tmagData[tmagCh].z;
            currentMm  = g_pistonHallmm[p];          // tmagZToMm() ile doğru nonlinear dönüşüm
            gotMutex = true;
            xSemaphoreGive(g_sharedMutex);
        }
        if (!gotMutex) continue;

        // Hold devre dışı → durumu sıfırla
        if (!holdEnabled) {
            s_hasState[p]       = HAS_INIT;
            s_holdPwmCurrent[p] = 0;
            continue;
        }

        // QC modunda (kalibre edilmemiş veya QC state aktif) midRaw/maxRaw kontrolü atla
        bool inQC = !calibrated || (s_hasState[p] >= HAS_QC_PUMP_WAIT);
        if (!inQC) {
            // Hall sensör verisi doğrulama: Kalibrasyon range kontrolü
            float rawRange = fabsf(maxRaw - minRaw);
            if (rawRange < HOLD_MIN_VALID_RANGE) {
                // Kalibrasyon verisi geçersiz - QC moduna geç
                {
                    char msg[96];
                    snprintf(msg, sizeof(msg), "[HOLD] p%d Kalibrasyon bozuk! range=%.0f < %.0f → QC basliyor", 
                             p, rawRange, HOLD_MIN_VALID_RANGE);
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                }
                inQC = true;
                s_hasState[p] = HAS_QC_PUMP_WAIT;
                s_qcMinRaw[p] = 0.0f;
                s_qcMaxRaw[p] = 0.0f;
                s_qcStable[p] = 0;
                s_qcTimer[p] = millis();
            }
            if (midRaw < 1.0f && maxRaw > minRaw + 200.0f) midRaw = (minRaw + maxRaw) * 0.5f;
            if (midRaw < 1.0f || maxRaw < 1.0f) continue;
        }

        // Destek valfini aç
        if (supportIdx >= 0 && supportIdx < 8) target[supportIdx] = 2000;

        // İlk çalıştırma veya hold yeniden aktifleşince: state makinesini sıfırla
        if (s_holdPwmCurrent[p] == 0 || initNeeded) {
            // PID state sıfırla
            s_pidIntegral[p] = 0.0f;
            s_pidLastError[p] = 0.0f;
            s_filteredError[p] = 0.0f;
            s_lastPwmOut[p] = 0;
            
            // holdPwmBase: DAIMA kalibrasyon değeri (adaptasyon sınırını sabitler - kaymaı önler)
            // holdPwmCurrent: önceki turda bulunan son değerden başla (sıcaklığa hızlı uyarlanır)
            uint16_t startPwm = holdPwm;
            if (s_holdPwmLastAdapted[p] >= HOLD_PWM_ABS_MIN && s_holdPwmLastAdapted[p] <= HOLD_PWM_ABS_MAX) {
                startPwm = s_holdPwmLastAdapted[p];
            }
            s_holdPwmCurrent[p] = startPwm;
            s_holdPwmBase[p]    = (float)holdPwm;  // Adaptasyon merkezi her zaman kalibrasyonda sabit kalır
            if (!calibrated) {
                // Kalibrasyon yok → Hızlı Kalibrasyon başlat
                s_hasState[p]  = HAS_QC_PUMP_WAIT;
                s_qcMinRaw[p]  = 0.0f;
                s_qcMaxRaw[p]  = 0.0f;
                s_qcStable[p]  = 0;
                s_qcTimer[p]   = millis();
                portENTER_CRITICAL(&g_portMux);
                g_pumpCmd.cmd = PUMP_CMD_AUTO;
                g_pumpCmd.seq++;
                portEXIT_CRITICAL(&g_portMux);
                {
                    char qm[64];
                    snprintf(qm, sizeof(qm), "[HOLD] QC p%d basliyor - pompa AUTO", p);
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, qm);
                }
            } else {
                s_hasState[p] = HAS_INIT;
            }
        }
        
        // Feed-forward PWM: Hedef pozisyona göre başlangıç tahmini
        // openThresh/closeThresh zaten kalibrasyon verisinden güncellenmiş (for-loop başında)
        // xRef = 0.0 (kapalı) → closeThresh, xRef = 1.0 (açık) → openThresh
        float feedForwardPwm;
        // P2 için özel: lineer interpolasyon yetersiz kalıyor, kalibre edilmiş hold değerini kullan
        // openThresh==1440 kontrolü ile P2 kalibrasyon düzeltmesinin uygulandığını anlarız
        if (p == 2 && openThresh == 1440 && fabsf(xRef - 0.5f) < 0.1f) {
            feedForwardPwm = (float)holdPwm;  // Düzeltilmiş değer: 1150
        } else {
            feedForwardPwm = (float)closeThresh + (float)(openThresh - closeThresh) * xRef;
        }
        
        // Sıcaklık kompanzasyonu uygula
        float tempCompFF = (g_temp2_C - HOLD_TEMP_REF_C) * HOLD_TEMP_K;
        if (tempCompFF > (float)HOLD_TEMP_MAX_COMP) tempCompFF = (float)HOLD_TEMP_MAX_COMP;
        if (tempCompFF < -(float)HOLD_TEMP_MAX_COMP) tempCompFF = -(float)HOLD_TEMP_MAX_COMP;
        feedForwardPwm += tempCompFF;
        
        // Güvenlik sınırları
        if (feedForwardPwm < HOLD_PWM_CTRL_MIN) feedForwardPwm = HOLD_PWM_CTRL_MIN;
        // Üst sınır: openThresh'in biraz üstü (pistonu açma eşiğine ulaşsın ama tam açmasın)
        uint16_t ffMax = openThresh + 50;
        if (feedForwardPwm > ffMax) feedForwardPwm = ffMax;

        // Bölge sınırları doğrudan mm cinsinden (nonlinear dönüşüm g_pistonHallmm'de zaten yapıldı)
        // Sensör doyma koruması: TMAG max değeri (32752 ≈ 0x7FF0) → geçici overrange
        // Piston hızlı hareket sırasında manyetik alan ölçüm aralığını aşabilir.
        // Bu durumda son geçerli mm değerini kullan; sert F_CL pulsunu engelle.
        bool sensorSaturated = (currentRaw >= 32700.0f);
        if (!sensorSaturated) {
            s_prevValidMm[p] = currentMm;
            s_satCount[p]    = 0;
        } else {
            // raw=32752 = kalibre edilmiş AÇIK uç nokta → piston gerçekte tam açık konumda.
            // prevValidMm (eski geçerli konum) YANLIŞTIR; strokeMm (tam açık mm) kullan.
            // Bu sayede error = strokeMm - targetMm > 0 → F_CL uygulanır, piston kapanır.
            float satMm = (tmagCal.valid && tmagCal.strokeMm > 5.0f) ? tmagCal.strokeMm : 26.0f;
            currentMm = satMm;
            if (s_satCount[p] < 255)     s_satCount[p]++;
        }
        // Senaryo A (artık pasif): currentMm=strokeMm olduğundan error>0 → state F_CL olur,
        // BRAKE_OPEN/FAST_OPEN koşulu asla sağlanmaz. Güvenlik için bırakıldı.
        if (sensorSaturated && s_satCount[p] >= 3 &&
            (s_hasState[p] == HAS_BRAKE_OPEN || s_hasState[p] == HAS_FAST_OPEN)) {
            s_hasState[p] = HAS_FAST_CLOSE;
            {
                char sm[80];
                snprintf(sm, sizeof(sm),
                    "[HOLD] p%d SAT-STUCK: sensor sınırı aşıldı, HOLD kabul edildi (%.2fmm)",
                    p, currentMm);
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, sm);
            }
        }

        float brakeZone = HOLD_BRAKE_ZONE_MM;   // 4.0mm
        float deadZone  = HOLD_DEAD_ZONE_MM;    // 0.8mm

        float targetRaw = holdComputeTargetRaw(tmagCal, midRaw);  // Log için
        float strokeMm_ = (tmagCal.valid && tmagCal.strokeMm > 5.0f) ? tmagCal.strokeMm : 26.0f;
        float targetMm  = strokeMm_ * xRef;  // x_ref: 0.0=kapalı, 0.5=orta, 1.0=açık
        float error     = currentMm - targetMm;  // mm: + = hedefin üstünde (açık tarafta)
        float absError  = fabsf(error);

        // Gürültü filtreleme: EMA (Exponential Moving Average) ile hata sinyalini yumuşat
        // Alfa = 0.3: Yeni değer %30, eski %70 - gürültü azaltma ve hız arası denge
        static constexpr float EMA_ALPHA = 0.3f;
        s_filteredError[p] = EMA_ALPHA * error + (1.0f - EMA_ALPHA) * s_filteredError[p];
        float filteredAbsError = fabsf(s_filteredError[p]);
        
        // HAS_INIT → yön belirle
        // FIZIK: Yüksek PWM → valf açılır → hidrolik → piston uzar → raw ARTAR
        //        Düşük PWM → valf kapanır → yay geri çeker → piston kısalır → raw AZALIR
        // Sonuç: error<0 (cur<tgt, raw düşük) → piston uzatmak lazım → YÜKSEK PWM → F_OP
        //         error>0 (cur>tgt, raw yüksek) → piston kısaltmak lazım → DÜŞÜK PWM → F_CL
        if (s_hasState[p] == HAS_INIT) {
            if (filteredAbsError <= deadZone)        s_hasState[p] = HAS_HOLDING;
            else if (error < 0)              s_hasState[p] = (filteredAbsError > brakeZone) ? HAS_FAST_OPEN  : HAS_BRAKE_OPEN;
            else                             s_hasState[p] = (filteredAbsError > brakeZone) ? HAS_FAST_CLOSE : HAS_BRAKE_CLOSE;
        }

        uint16_t pwmOut = s_holdPwmCurrent[p];
        uint16_t hpwm   = s_holdPwmCurrent[p];  // kısa erişim

        switch (s_hasState[p]) {

        // ---- Yaklaşma: Orantılı (P) kontrol ----
        // FAST (uzak): basePwm +/- DELTA_FAST  → piston harekete geçebilecek güç
        // BRAKE (yakın): basePwm +/- DELTA      → kontrollü yaklaşma, aşım önlenir
        case HAS_FAST_OPEN:
        case HAS_BRAKE_OPEN:
        case HAS_FAST_CLOSE:
        case HAS_BRAKE_CLOSE: {
            // Yaklaşma hesabı DAIMA s_holdPwmBase kullanır (adaptasyon cascade etkisini önler)
            float basePwm   = s_holdPwmBase[p];
            float Kp_open   = (float)(openThresh  - (uint16_t)basePwm) / brakeZone;
            float Kp_close  = (float)((uint16_t)basePwm - closeThresh) / brakeZone;
            if (Kp_open  < 80.0f / brakeZone) Kp_open  = 80.0f / brakeZone;
            if (Kp_close < 80.0f / brakeZone) Kp_close = 80.0f / brakeZone;
            float Kp   = (error < 0.0f) ? Kp_open : Kp_close;
            float pwmF = basePwm - Kp * error;
            // FAST state (uzak): daha geniş delta → piston başlayabilir
            // BRAKE state (yakın): dar delta → kontrollü yavaşlama
            bool isFast  = (s_hasState[p] == HAS_FAST_OPEN || s_hasState[p] == HAS_FAST_CLOSE);
            float openDelta  = isFast ? (float)HOLD_APPROACH_OPEN_DELTA_FAST
                                      : (float)HOLD_APPROACH_OPEN_DELTA;
            float closeDelta = isFast ? (float)HOLD_APPROACH_CLOSE_DELTA_FAST
                                      : (float)HOLD_APPROACH_CLOSE_DELTA;
            // F_OP mutlak üst sınır: pistonun openThresh'ine göre dinamik
            // P5-7 için 1440, P1-3 için ~1433 gerekiyor. 1200 çok düşük!
            uint16_t dynamicAbsMax = openThresh + 100;  // openThresh + 100 PWM marj
            float pwmMax = min(basePwm + openDelta, (float)dynamicAbsMax);
            float pwmMin = max(basePwm - closeDelta, (float)HOLD_PWM_CTRL_MIN);
            if (pwmF > pwmMax) pwmF = pwmMax;
            if (pwmF < pwmMin) pwmF = pwmMin;
            
            // Rate limiting: Yaklaşmada da PWM değişim hızını sınırla
            uint16_t approachPwm = (uint16_t)pwmF;
            if (s_lastPwmOut[p] > 0) {
                int16_t delta = (int16_t)approachPwm - (int16_t)s_lastPwmOut[p];
                if (delta > (int16_t)(HOLD_PWM_RATE_LIMIT * 2)) {  // Yaklaşmada 2x daha hızlı izin ver
                    approachPwm = s_lastPwmOut[p] + HOLD_PWM_RATE_LIMIT * 2;
                } else if (delta < -(int16_t)(HOLD_PWM_RATE_LIMIT * 2)) {
                    approachPwm = s_lastPwmOut[p] - HOLD_PWM_RATE_LIMIT * 2;
                }
            }
            
            pwmOut = approachPwm;
            s_lastPwmOut[p] = pwmOut;
            
            // State geçişi: Filtrelenmiş hatayı kullan (gürültü önleme)
            if      (filteredAbsError <= deadZone) s_hasState[p] = HAS_HOLDING;
            else if (s_filteredError[p] < 0)  s_hasState[p] = (filteredAbsError > brakeZone) ? HAS_FAST_OPEN  : HAS_BRAKE_OPEN;
            else                              s_hasState[p] = (filteredAbsError > brakeZone) ? HAS_FAST_CLOSE : HAS_BRAKE_CLOSE;
            break;
        }

        // ---- Tutma: PID Controller + Adaptasyon + Rate Limiting ----
        case HAS_HOLDING: {
            uint32_t nowMs = millis();
            
            // Büyük sapma → yaklaşma moduna geri dön (filtrelenmiş hatayı kullan)
            // Sensör doyuk iken HOLD'dan ÇIKMA: SAT-STUCK tarafından zorlanan HOLD'un
            // hemen BR_O döngüsüne geri dönmesini önler.
            if (!sensorSaturated) {
                if (filteredAbsError > brakeZone * 1.5f) {
                    if (++s_holdOutCount[p] >= 2) {
                        s_hasState[p] = (s_filteredError[p] < 0) ? HAS_FAST_OPEN : HAS_FAST_CLOSE;
                        s_holdOutCount[p] = 0;
                        s_pidIntegral[p] = 0.0f;  // Integral sıfırla (wind-up önleme)
                    }
                } else {
                    s_holdOutCount[p] = 0;
                    if (filteredAbsError > deadZone) {
                        s_hasState[p] = (s_filteredError[p] < 0) ? HAS_BRAKE_OPEN : HAS_BRAKE_CLOSE;
                        s_pidIntegral[p] *= 0.5f;  // Integral yarıya indir (soft reset)
                    }
                }
            } else {
                s_holdOutCount[p] = 0;
            }

            // PID Controller (sadece PID periyodu dolduğunda çalıştır - CPU yükünü azalt)
            if (nowMs - s_pidLastMs[p] >= HOLD_PID_PERIOD_MS) {
                s_pidLastMs[p] = nowMs;
                
                // Feed-forward baz PWM (kalibrasyon + pozisyon bazlı tahmin)
                float basePwm = s_holdPwmCurrent[p];
                
                // Integral limit kontrolü (anti-windup)
                if (s_pidIntegral[p] > HOLD_PID_INTEGRAL_MAX) s_pidIntegral[p] = HOLD_PID_INTEGRAL_MAX;
                if (s_pidIntegral[p] < -HOLD_PID_INTEGRAL_MAX) s_pidIntegral[p] = -HOLD_PID_INTEGRAL_MAX;
                
                // Derivative (hata değişim hızı)
                float derivative = (s_filteredError[p] - s_pidLastError[p]) / (HOLD_PID_PERIOD_MS / 1000.0f);
                s_pidLastError[p] = s_filteredError[p];
                
                // PID çıkışı (adaptasyon disabled ise Ki=0 olur)
                float Ki = g_autoTestParams.adaptiveHoldEnabled ? HOLD_PID_KI : 0.0f;
                float Kp = (s_filteredError[p] < 0.0f) ? HOLD_PID_KP_OPEN : HOLD_PID_KP_CLOSE;
                
                float pidP = Kp * s_filteredError[p];
                float pidI = Ki * s_pidIntegral[p];
                float pidD = HOLD_PID_KD * derivative;
                
                float pidOut = basePwm - pidP - pidI - pidD;  // - çünkü error>0 PWM azaltmalı
                
                // Sıcaklık kompanzasyonu (PID sonrası)
                float tempComp = (g_temp2_C - HOLD_TEMP_REF_C) * HOLD_TEMP_K;
                if (tempComp > (float)HOLD_TEMP_MAX_COMP) tempComp = (float)HOLD_TEMP_MAX_COMP;
                if (tempComp < -(float)HOLD_TEMP_MAX_COMP) tempComp = -(float)HOLD_TEMP_MAX_COMP;
                pidOut += tempComp;
                
                // PWM sınırları
                if (pidOut < (float)HOLD_PWM_CTRL_MIN) pidOut = (float)HOLD_PWM_CTRL_MIN;
                // Üst sınır: openThresh + 50, ama max 1500 (valf güvenliği)
                uint16_t holdMax = openThresh + 50;
                if (holdMax > 1500) holdMax = 1500;
                if (pidOut > (float)holdMax) pidOut = (float)holdMax;
                
                // Rate limiting: PWM değişim hızını sınırla (sarsıntı önleme)
                uint16_t targetPwm = (uint16_t)pidOut;
                if (s_lastPwmOut[p] > 0) {  // İlk çalıştırma değilse
                    int16_t delta = (int16_t)targetPwm - (int16_t)s_lastPwmOut[p];
                    if (delta > (int16_t)HOLD_PWM_RATE_LIMIT) {
                        targetPwm = s_lastPwmOut[p] + HOLD_PWM_RATE_LIMIT;
                    } else if (delta < -(int16_t)HOLD_PWM_RATE_LIMIT) {
                        targetPwm = s_lastPwmOut[p] - HOLD_PWM_RATE_LIMIT;
                    }
                }
                
                pwmOut = targetPwm;
                s_lastPwmOut[p] = pwmOut;
                
                // Integral birikimi (adaptasyon etkinse)
                if (g_autoTestParams.adaptiveHoldEnabled && filteredAbsError > deadZone * 0.5f) {
                    // Hata işareti değişmeden integral biriktir (stable drift düzeltmesi)
                    s_pidIntegral[p] += s_filteredError[p] * (HOLD_PID_PERIOD_MS / 1000.0f);
                    
                    // Kalibrasyon bazlı adaptasyon limitleri (kalibrasyon kayması önleme)
                    float lo = s_holdPwmBase[p] - (float)HOLD_ADAPT_LIMIT;
                    float hi = s_holdPwmBase[p] + (float)HOLD_ADAPT_LIMIT;
                    float effectivePwm = (float)s_holdPwmCurrent[p] - s_pidIntegral[p] * Ki;  // Integral etkisi
                    
                    // Integral aşırı büyüdüyse sınırla (adaptasyon limiti aşılmasın)
                    if (effectivePwm < lo) {
                        s_pidIntegral[p] = (s_holdPwmCurrent[p] - lo) / Ki;
                    } else if (effectivePwm > hi) {
                        s_pidIntegral[p] = (s_holdPwmCurrent[p] - hi) / Ki;
                    }
                    
                    // Son uyarlanmış değeri kaydet (sıcaklık hariç)
                    s_holdPwmLastAdapted[p] = (uint16_t)(s_holdPwmCurrent[p] - s_pidIntegral[p] * Ki);
                }
            } else {
                // PID periyodu dolmadan önceki son değeri koru
                pwmOut = s_lastPwmOut[p];
            }
            break;
        }

        // ---- Hızlı Kalibrasyon: Pompa Bekle ----
        case HAS_QC_PUMP_WAIT:
            pwmOut = 0;  // Valf kapalı
            if (g_pumpPub.bar >= 25.0f || millis() - s_qcTimer[p] > 10000) {
                s_qcTimer[p] = millis();
                s_hasState[p] = HAS_QC_OPEN;
                {
                    char qm[64];
                    snprintf(qm, sizeof(qm), "[HOLD] QC p%d %.1fbar → TAM AC", p, g_pumpPub.bar);
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, qm);
                }
            }
            break;

        // ---- Hızlı Kalibrasyon: Tam Aç → max_raw ----
        case HAS_QC_OPEN:
            pwmOut = 1900;
            if (millis() - s_qcTimer[p] >= 2000) {
                s_qcMaxRaw[p] = currentRaw;
                s_qcTimer[p]  = millis();
                s_hasState[p] = HAS_QC_CLOSE;
                {
                    char qm[64];
                    snprintf(qm, sizeof(qm), "[HOLD] QC p%d maxRaw=%.0f → TAM KAPAT", p, s_qcMaxRaw[p]);
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, qm);
                }
            }
            break;

        // ---- Hızlı Kalibrasyon: Tam Kapat → min_raw ----
        case HAS_QC_CLOSE:
            pwmOut = 400;
            if (millis() - s_qcTimer[p] >= 2000) {
                s_qcMinRaw[p] = currentRaw;
                float qcMid = (s_qcMinRaw[p] + s_qcMaxRaw[p]) * 0.5f;
                if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
                    g_pistonCalibData[p].min_raw = s_qcMinRaw[p];
                    g_pistonCalibData[p].max_raw = s_qcMaxRaw[p];
                    g_pistonCalibData[p].mid_raw = qcMid;
                    xSemaphoreGive(g_sharedMutex);
                }
                s_qcFindLo[p]  = HOLD_PWM_ABS_MIN;
                s_qcFindHi[p]  = (uint16_t)(openThresh + 150);
                s_qcFindPwm[p] = (s_qcFindLo[p] + s_qcFindHi[p]) / 2;
                s_qcStable[p]  = 0;
                s_qcLastRaw[p] = currentRaw;
                s_qcTimer[p]   = millis();
                s_hasState[p]  = HAS_QC_FIND_HOLD;
                {
                    char qm[96];
                    snprintf(qm, sizeof(qm), "[HOLD] QC p%d min=%.0f max=%.0f pwm=%u → DENGE ARA",
                             p, s_qcMinRaw[p], s_qcMaxRaw[p], s_qcFindPwm[p]);
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, qm);
                }
            }
            break;

        // ---- Hızlı Kalibrasyon: Doğal Denge Noktasını Bul ----
        // Hedef: piston hareketinin sıfırlandığı (hız≈ 0) PWM'i bul.
        // O PWM = duty_hold, o andaki pozisyon = mid_raw.
        // Böylece sensor non-linearity'den bağımsız, fiziksel denge noktası öğrenilir.
        case HAS_QC_FIND_HOLD: {
            pwmOut = s_qcFindPwm[p];
            if (millis() - s_qcTimer[p] >= 800) {
                s_qcTimer[p] = millis();
                float velocity  = currentRaw - s_qcLastRaw[p];  // raw/800ms
                s_qcLastRaw[p]  = currentRaw;
                bool converged  = (s_qcFindHi[p] > s_qcFindLo[p]) &&
                                  ((s_qcFindHi[p] - s_qcFindLo[p]) <= 15);
                bool stable     = fabsf(velocity) < 200.0f;  // ~0.2mm/800ms
                if (stable || converged) {
                    if (++s_qcStable[p] >= 2) {
                        uint16_t found   = s_qcFindPwm[p];
                        float    midFound = currentRaw;
                        s_holdPwmCurrent[p] = found;
                        s_holdPwmBase[p]    = (float)found;
                        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
                            g_pistonCalibData[p].mid_raw    = midFound;
                            g_pistonCalibData[p].duty_hold  = found;
                            g_pistonCalibData[p].calibrated = true;
                            xSemaphoreGive(g_sharedMutex);
                        }
                        s_hasState[p] = HAS_HOLDING;
                        {
                            char qm[96];
                            snprintf(qm, sizeof(qm), "[HOLD] QC TAMAMLANDI p%d duty_hold=%u mid_raw=%.0f",
                                     p, found, midFound);
                            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, qm);
                        }
                    }
                } else {
                    s_qcStable[p] = 0;
                    // vel>0: piston uzuyor (fazla kuvvet) → PWM azalt
                    // vel<0: piston kısalıyor (az kuvvet) → PWM artır
                    if (velocity > 0) s_qcFindHi[p] = s_qcFindPwm[p];
                    else              s_qcFindLo[p]  = s_qcFindPwm[p];
                    s_qcFindPwm[p] = (s_qcFindLo[p] + s_qcFindHi[p]) / 2;
                    {
                        char qm[96];
                        snprintf(qm, sizeof(qm), "[HOLD] QC p%d vel=%.0f pwm=%u [%u..%u] pos=%.0f",
                                 p, velocity, s_qcFindPwm[p], s_qcFindLo[p], s_qcFindHi[p], currentRaw);
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, qm);
                    }
                }
            }
            break;
        }

        default:
            s_hasState[p] = HAS_INIT;
            pwmOut = hpwm;
            break;
        }

        target[valveIdx] = pwmOut;

        // g_pistonHoldDuty güncelle
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            g_pistonHoldDuty[p] = pwmOut;
            xSemaphoreGive(g_sharedMutex);
        }

        // Debug (1 saniyede bir)
        uint32_t nowDbg = millis();
        if (nowDbg - s_holdDbgMs[p] >= 1000) {
            s_holdDbgMs[p] = nowDbg;
            static const char* SN[] = {"INIT","F_OP","BR_O","F_CL","BR_C","HOLD","QC_PMP","QC_OP","QC_CL","QC_FH"};
            {
                char dbg[220];
                // Kalibrasyon ve PID bilgisi ekle
                snprintf(dbg, sizeof(dbg),
                    "[HOLD] p%d st=%s cur=%.2f tgt=%.2f e=%.2f fe=%.2f pwm=%u ff=%.0f I=%.1f ot=%u ct=%u cal=%d",
                    p, SN[s_hasState[p]], currentMm, targetMm, error, s_filteredError[p],
                    pwmOut, feedForwardPwm, s_pidIntegral[p], openThresh, closeThresh, calibrated ? 1 : 0);
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, dbg);
            }
        }
    }
}

// ============================================================================
// AKIM-TABANLI HOLD KONTROL (Yeni - Deneysel)
// ============================================================================
// Kullanıcının test ettiği akım değerlerini kullanarak pozisyon kontrolü.
// Mantık: Pozisyon hatasına göre hedef akım belirle, akım kontrolcüye gönder.
// 
// Piston → Valf eşlemesi:
//   P0 (5-7) → V2 (N434)
//   P1 (1-3) → V0 (N433)  
//   P2 (2-4) → V7 (N437)
//   P3 (6-R) → V4 (N438)
//
// Test edilen akımlar (23°C):
//   P2-4: 580mA(aç) 460mA(kapa) 500mA(tut)
//   P6-R: 560mA(aç) 420mA(kapa) 500mA(tut)
//   P1-3: 590mA(aç) 470mA(kapa) 540mA(tut)
//   P5-7: 550mA(aç) 435mA(kapa) 500mA(tut)

static const CurrentHoldParams CURRENT_HOLD_PARAMS[4] = {
    // P0: 5-7 (N434, idx=2)
    { .openCurrent_mA = 550.0f, .closeCurrent_mA = 435.0f, .holdCurrent_mA = 500.0f,
      .slowOpenCurrent_mA = 520.0f, .slowCloseCurrent_mA = 460.0f,
      .deadZone_mm = 1.5f, .brakeZone_mm = 4.0f, .valveIndex = 2 },
    // P1: 1-3 (N433, idx=0)
    { .openCurrent_mA = 590.0f, .closeCurrent_mA = 470.0f, .holdCurrent_mA = 540.0f,
      .slowOpenCurrent_mA = 560.0f, .slowCloseCurrent_mA = 500.0f,
      .deadZone_mm = 1.5f, .brakeZone_mm = 4.0f, .valveIndex = 0 },
    // P2: 2-4 (N437, idx=7)
    { .openCurrent_mA = 580.0f, .closeCurrent_mA = 460.0f, .holdCurrent_mA = 500.0f,
      .slowOpenCurrent_mA = 540.0f, .slowCloseCurrent_mA = 480.0f,
      .deadZone_mm = 1.5f, .brakeZone_mm = 4.0f, .valveIndex = 7 },
    // P3: 6-R (N438, idx=4)
    { .openCurrent_mA = 560.0f, .closeCurrent_mA = 420.0f, .holdCurrent_mA = 500.0f,
      .slowOpenCurrent_mA = 530.0f, .slowCloseCurrent_mA = 460.0f,
      .deadZone_mm = 1.5f, .brakeZone_mm = 4.0f, .valveIndex = 4 }
};

// Akım-tabanlı hold state (sadece 4 piston için)
enum CurrentHoldState { CHS_INIT = 0, CHS_FAST_OPEN, CHS_BRAKE_OPEN, CHS_HOLDING, CHS_BRAKE_CLOSE, CHS_FAST_CLOSE };
static CurrentHoldState s_currHoldState[4] = {CHS_INIT};
static float s_currHoldTargetCurrent[4] = {0};  // Hedef akım (mA)
static uint32_t s_currHoldLastMs[4] = {0};      // Son güncelleme zamanı
static bool s_currHoldActive[8]   = {false};    // Hangi valfler PI akım kontrolünde (valf indeksi)
static bool s_directPWMActive[8] = {false};    // Hangi valfler doğrudan PWM kontrolünde (holdcontrol_V2)

void applyCurrentBasedHoldControl(uint16_t target[], bool suppressed) {
    // Sadece 4 ana pistonu kontrol et (P0-P3)
    for (int p = 0; p < 4; p++) {
        const CurrentHoldParams& params = CURRENT_HOLD_PARAMS[p];
        uint8_t valveIdx = params.valveIndex;
        
        // Hold aktif mi kontrol et
        bool holdEnabled = false;
        float xRef = 0.5f;  // Varsayılan orta pozisyon
        float currentMm = 0.0f;
        bool gotMutex = false;
        
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            holdEnabled = g_pistonRuntime[p].hold_mid_enable;
            xRef = g_pistonRuntime[p].x_ref;
            currentMm = g_pistonHallmm[p];
            gotMutex = true;
            xSemaphoreGive(g_sharedMutex);
        }
        if (!gotMutex) continue;
        
        // Hold devre dışı → state sıfırla ve valfi kapat
        if (!holdEnabled || suppressed) {
            s_currHoldState[p] = CHS_INIT;
            s_currHoldTargetCurrent[p] = 0.0f;
            s_currHoldActive[params.valveIndex] = false;  // İşareti kaldır
            // Valfi kapat (mevcut target[] değeri korunur - dışarıdan atanmış olabilir)
            continue;
        }
        
        // Hedef pozisyon (mm cinsinden)
        float strokeMm = 26.0f;  // Nominal strok
        float targetMm = strokeMm * xRef;
        float error = currentMm - targetMm;  // + = hedefin üstünde (açık), - = kapalı
        float absError = fabsf(error);
        
        // Sıcaklık kompanzasyonu
        float tempComp = (g_temp2_C - HOLD_TEMP_REF_C) * HOLD_CURRENT_TEMP_K;
        if (tempComp > HOLD_CURRENT_MAX_COMP) tempComp = HOLD_CURRENT_MAX_COMP;
        if (tempComp < -HOLD_CURRENT_MAX_COMP) tempComp = -HOLD_CURRENT_MAX_COMP;
        
        // Basit orantili akim kontrolu: targetCurrent = holdCurrent - Kp * error
        // error > 0 (piston hedefin ustunde) → akimi duşur → piston kapanir
        // error < 0 (piston hedefin altında) → akimi artir → piston acilir
        // Kp=8 mA/mm: 13mm hata → 104mA duzeltme (hold+104 veya hold-104)
        static constexpr float CURR_KP = 3.0f;  // mA/mm (dusuruldu: 8→3, overshoot azaltmak icin)
        
        float targetCurrent = params.holdCurrent_mA - (CURR_KP * error) + tempComp;
        
        // Sinirla: closeCurrent ile openCurrent arasinda tut
        if (targetCurrent > params.openCurrent_mA) targetCurrent = params.openCurrent_mA;
        if (targetCurrent < params.closeCurrent_mA) targetCurrent = params.closeCurrent_mA;
        
        // State sadece log icin kullan (eski state machine kaldirıldı)
        CurrentHoldState& state = s_currHoldState[p];
        if (absError <= params.deadZone_mm)       state = CHS_HOLDING;
        else if (error < -params.brakeZone_mm)    state = CHS_FAST_OPEN;
        else if (error > params.brakeZone_mm)     state = CHS_FAST_CLOSE;
        else if (error < 0)                       state = CHS_BRAKE_OPEN;
        else                                      state = CHS_BRAKE_CLOSE;
        
        // Hedef akımı kaydet
        s_currHoldTargetCurrent[p] = targetCurrent;
        
        // Sadece ilk etkinlestirmede SetMode cagir (baslangic PWM ayarlanir, integral sifirlanir)
        if (!ValveCurrentControl_IsEnabled(valveIdx)) {
            ValveCurrentMode initMode;
            float holdCurr = params.holdCurrent_mA;
            if (targetCurrent > holdCurr * 1.05f)      initMode = VALVE_MODE_OPEN;
            else if (targetCurrent < holdCurr * 0.95f) initMode = VALVE_MODE_CLOSE;
            else                                        initMode = VALVE_MODE_HOLD;
            ValveCurrentControl_SetMode(valveIdx, initMode);
        }
        // Her dongude hedef akimi guncelle (PI integratoru korunur - Update yok burada!)
        // Gercek Update() ana dongudeki AKIM KONTROLU blogunda tek kez yapilir.
        ValveCurrentControl_SetTargetCurrent(valveIdx, targetCurrent);
        ValveCurrentControl_Enable(valveIdx, true);
        
        // Debug log (500ms'de bir)
        uint32_t nowMs = millis();
        if (nowMs - s_currHoldLastMs[p] >= 500) {
            s_currHoldLastMs[p] = nowMs;
            {
                char dbg[192];
                static const char* STATE_NAMES[] = {"INIT","F_OP","BR_O","HOLD","BR_C","F_CL"};
                static const uint8_t VALVE_TO_INA[8] = {0, 3, 1, 2, 5, 7, 6, 4};
                snprintf(dbg, sizeof(dbg),
                    "[CURR_HOLD] p%d st=%s cur=%.1f tgt=%.1f e=%.1f I_tgt=%.0f I_act=%.0f pwm=%u INA%d",
                    p, STATE_NAMES[state], currentMm, targetMm, error, targetCurrent,
                    ValveCurrentControl_GetMeasuredCurrent(valveIdx),
                    ValveCurrentControl_GetPWM(valveIdx),
                    VALVE_TO_INA[valveIdx]);
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, dbg);
            }
        }
        
        // Bu valfin yeni akim-tabanli kontrolde oldugunu isaretleme
        // Ana dongudeki blok Update() yapip target[]'i guncelleyecek
        s_currHoldActive[valveIdx] = true;
        
    }
}

// ============================================================================
// KAPALI ÇEVRİM KONTROL
// piston_ctrl_step : Hall geri beslemeli P pozisyon kontrolörü (tüm pistonlar)
// pcv_pi_step      : Basınç sensörü geri beslemeli PI basınç kontrolörü (N436/N440)
// ============================================================================

// --- Ortak sabitler ---
static constexpr float V_SUPPLY_CL     = 12.0f;   // Sürücü besleme voltajı (V)
static constexpr float PWM_RAMP_CL     = 25.0f;   // Slew rate (PWM birim/döngü, ~10ms)
static constexpr float PIST_STROKE_CL  = 26.0f;   // P0-P3 toplam strok (mm)
static constexpr float PIST_K12_STROK  = 15.0f;   // K1/K2 strok (mm)
static constexpr float PIST_KP_CL      = 25.0f;   // P kazancı (mA/mm) — Kp düşürüldü, 80mA margin ile 3.2mm bant
static constexpr float PIST_KD_CL      = 15.0f;   // D kazancı (mA·s/mm) — hız sönümleme
static constexpr float PIST_VEL_EMA    = 0.3f;    // velocity filter alpha (gürültüye karşı)

// --- PCV sabit duty (PI yerine — tek basınç sensörü yüksek tarafı ölçüyor) ---
// SUPPORT_VALVE_DUTY (2500) zaten tanımlı, burada ek sabit yok

// --- Piston kontrolör durumu ---
// Piston → valf indeksi: P0=N434(v2), P1=N433(v0), P2=N437(v7), P3=N438(v4), K1=N435(v3), K2=N439(v6)
static const uint8_t VALVE_IDX_CL[6]    = {2, 0, 7, 4, 3, 6};
// Valf indeksi → INA219 indeksi (fiziksel bağlantı)
static const uint8_t VALVE_TO_INA_V2[8] = {0, 3, 1, 2, 5, 7, 6, 4};

static bool     s_pistonWasActive[6] = {};
static uint32_t s_clDbgMs[6]     = {0};
static bool     s_pistonCtrlActive[8] = {};  // Hangi valf indekslerini bu kontrolör yönetiyor


// -----------------------------------------------------------------------
// piston_ctrl_step: Hall pozisyon geri beslemeli P kontrolörü
// Her piston için: CurOut = holdMa - Kp * (posMm - tgtMm)
// Akım → PWM dönüşümü: V=IR modeli, EMA ile bobin direnci adaptasyonu
// -----------------------------------------------------------------------
static void piston_ctrl_step(uint16_t target[8], bool suppressed) {
    //                                P0      P1      P2      P3
    static constexpr float DEF_HOLD[4]  = {500.f, 500.f, 500.f, 490.f};
    static constexpr float DEF_OPEN[4]  = {620.f, 620.f, 620.f, 620.f};
    static constexpr float DEF_CLOS[4]  = {420.f, 420.f, 420.f, 400.f};

    // Slew-rate limiter: CurOut bir döngüde max bu kadar değişebilir (mA).
    // Hold P-kontrolörü ani openMa↔closeMa zıplamalarını üretebilir; bunu yumuşat.
    // 30 mA/cycle @ ~10ms = ~3000 mA/s. Mekanik dinamikten hızlı, bang-bang'ı önler.
    static constexpr float PIST_SLEW_MA = 30.0f;
    static float s_pistonLastCurOut[6] = {};

    // PD kontrolör için pozisyon ve filtrelenmiş hız geçmişi
    static float    s_pistonLastPos[6]  = {};
    static float    s_pistonVelFilt[6]  = {};
    static uint32_t s_pistonLastMs[6]   = {};

    for (int p = 0; p < 6; p++) {
        uint8_t vi = VALVE_IDX_CL[p];

        bool   holdEnabled = false;
        float  xRef     = 0.5f;
        float  posMm    = 0.0f;
        float  iAct     = 0.0f;
        bool   calibOk  = false;
        float  holdMa   = 0.0f, openMa = 0.0f, closeMa = 0.0f;

        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
            holdEnabled = g_pistonRuntime[p].hold_mid_enable;
            xRef        = g_pistonRuntime[p].x_ref;
            posMm       = g_pistonHallmm[p];
            iAct        = fabsf(g_tele.inaI_mA[VALVE_TO_INA_V2[vi]]);
            calibOk     = g_pistonCalibData[p].calibrated;
            if (calibOk) {
                holdMa  = g_pistonCalibData[p].hold_mA;
                openMa  = g_pistonCalibData[p].open_mA;
                closeMa = g_pistonCalibData[p].close_mA;
            }
            xSemaphoreGive(g_sharedMutex);
        }

        if (!holdEnabled || suppressed) {
            if (s_pistonCtrlActive[vi]) {
                // Piston kontrolörü bu valfi yönetiyordu → hedefi sıfırla
                s_pistonCtrlActive[vi] = false;
                s_pistonWasActive[p]   = false;
                s_pistonLastCurOut[p]  = 0.0f;  // slew state reset
                s_pistonLastMs[p]      = 0;     // PD state reset
                s_pistonVelFilt[p]     = 0.0f;
                if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
                    g_valveCustomCurrent_mA[vi] = 0.0f;
                    xSemaphoreGive(g_sharedMutex);
                }
            } else {
                // Piston kontrolörü aktif değil → GUI hedefine dokunma
                s_pistonWasActive[p]  = false;
                s_pistonLastCurOut[p] = 0.0f;
                s_pistonLastMs[p]     = 0;
                s_pistonVelFilt[p]    = 0.0f;
            }
            continue;
        }

        // Kalibre değer yoksa veya geçersizse → varsayılanları kullan
        if (!calibOk || holdMa < 50.0f) {
            holdMa  = (p < 4) ? DEF_HOLD[p] : 500.0f;
            openMa  = (p < 4) ? DEF_OPEN[p] : 650.0f;
            closeMa = (p < 4) ? DEF_CLOS[p] : 420.0f;
        }

        float stroke = (p < 4) ? PIST_STROKE_CL : PIST_K12_STROK;
        float tgtMm  = xRef * stroke;
        float error  = posMm - tgtMm;  // + = hedefin üstünde → akım azalt (kapanır)

        // Hız hesabı (mm/s) — EMA filtreli, gürültüye karşı.
        // İlk cycle: hız=0 (state taze).
        uint32_t nowCtrlMs = millis();
        float vel = 0.0f;
        if (s_pistonLastMs[p] != 0) {
            uint32_t dtMs = nowCtrlMs - s_pistonLastMs[p];
            if (dtMs > 0 && dtMs < 200) {  // makul aralık; cold start atla
                float dtS = dtMs * 0.001f;
                float velRaw = (posMm - s_pistonLastPos[p]) / dtS;
                s_pistonVelFilt[p] = (1.0f - PIST_VEL_EMA) * s_pistonVelFilt[p]
                                     + PIST_VEL_EMA * velRaw;
                vel = s_pistonVelFilt[p];
            }
        }
        s_pistonLastPos[p] = posMm;
        s_pistonLastMs[p]  = nowCtrlMs;

        // Asimetrik clamp:
        //   OPEN yönü: max(openMa+guard, hold+margin)
        //     - Bazı pistonlarda kalibrasyon open_mA ≈ hold_mA buluyor (breakaway sorunu).
        //       Bu durumda saf openMa clamp'i kullanılırsa P kontrolör açma yönünde
        //       ekstra güç üretemiyor. hold+margin ile garanti yetki sağla.
        //   CLOSE yönü: closeMa - guard (fiziksel)
        //     - hold-margin floor kullanılırsa close eşiğine ulaşılamıyor → piston
        //       hedefin üstünde park ediyor (steady-state error). Tam fiziksel
        //       otoriteyle kapama gücü ver.
        static constexpr float HOLD_MARGIN = 80.0f;  // mA — open yönü için minimum margin
        static constexpr float HW_GUARD    = 30.0f;  // mA — fiziksel limit toleransı
        float CurMax = openMa + HW_GUARD;
        if (CurMax < holdMa + HOLD_MARGIN) CurMax = holdMa + HOLD_MARGIN;
        float CurMin = closeMa - HW_GUARD;
        if (CurMin < 350.0f) CurMin = 350.0f;  // mutlak alt güvenlik

        // PD kontrolörü:
        //   error > 0 (piston yüksek) → akım azalt
        //   vel > 0   (piston yükseliyor) → akım azalt (öngörü ile fren)
        float CurOut = holdMa - PIST_KP_CL * error - PIST_KD_CL * vel;
        if (CurOut > CurMax) CurOut = CurMax;
        if (CurOut < CurMin) CurOut = CurMin;

        // Slew-rate limit: ilk döngüde holdMa'dan başla, sonra ±PIST_SLEW_MA/cycle.
        // Bu, openMa↔closeMa anlık sıçramalarını engelleyip bang-bang osilasyonunu kırar.
        if (s_pistonLastCurOut[p] <= 0.0f) {
            s_pistonLastCurOut[p] = holdMa;  // sıfırdan hedef akıma yumuşak başlangıç
        }
        float prevCur = s_pistonLastCurOut[p];
        if      (CurOut > prevCur + PIST_SLEW_MA) CurOut = prevCur + PIST_SLEW_MA;
        else if (CurOut < prevCur - PIST_SLEW_MA) CurOut = prevCur - PIST_SLEW_MA;
        s_pistonLastCurOut[p] = CurOut;

        // INA kapalı çevrim regülatörüne akım hedefini ilet (V=IR YOK)
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
            g_valveCustomCurrent_mA[vi] = CurOut;
            xSemaphoreGive(g_sharedMutex);
        }
        s_pistonCtrlActive[vi] = true;
        s_pistonWasActive[p]   = true;

        // Debug log (500ms periyot)
        uint32_t nowMs = millis();
        if (nowMs - s_clDbgMs[p] >= 500) {
            s_clDbgMs[p] = nowMs;
            {
                char buf[160];
                snprintf(buf, sizeof(buf),
                    "[PCTRL] p%d pos=%.1f tgt=%.1f e=%.1f vel=%.1f Iout=%.0f Iact=%.0f",
                    p, posMm, tgtMm, error, vel, CurOut, iAct);
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
            }
        }
    }
}

// -----------------------------------------------------------------------
// valve_current_reg_step: INA geri beslemeli merkezi PI akım regülatörü
//   Kp=0.5 duty/mA, Ki=8 duty/(mA·s) — V=IR hesaplama YOK
//   g_valveCustomCurrent_mA[vi]>0 → PI çalışır; ==0 → target=0
// -----------------------------------------------------------------------
static void valve_current_reg_step(uint16_t target[8], float dt_s) {
    static float            s_vcInteg[8]    = {};
    static float            s_vcPrevTgt[8]  = {};  // warm-start için önceki hedef
    static float            s_vcRampTgt[8]  = {};  // soft-start rampa hedefi
    static uint8_t          s_vcSettle[8]   = {};  // L/R transient sayacı (PI döngüsü adımı)
    static constexpr float  VC_KP           = 0.5f;
    static constexpr float  VC_KI           = 6.0f;
    static constexpr float  VC_SEED_K       = 2.05f;  // duty/mA ön-tohum katsayısı (V=IR ≈ 1310/640)
    static constexpr uint8_t VC_SETTLE_N    = 15;     // ilk N döngüde integral windup'ı bastır (~150ms @10ms)
    static constexpr float  VC_RAMP_MA_PER_S= 6000.0f; // open/open_slow için 6 A/s soft-start

    float tgtMa[8] = {};
    float inaI[8]  = {};
    uint8_t mode[8] = {};
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
        for (int i = 0; i < 8; i++) {
            tgtMa[i] = g_valveCustomCurrent_mA[i];
            inaI[i]  = fabsf(g_tele.inaI_mA[i]);
            mode[i]  = g_valveCustomMode[i];
        }
        xSemaphoreGive(g_sharedMutex);
    }
    for (int vi = 0; vi < 8; vi++) {
        if (tgtMa[vi] <= 0.0f) {
            s_vcInteg[vi]   = 0.0f;
            s_vcPrevTgt[vi] = 0.0f;
            s_vcRampTgt[vi] = 0.0f;
            s_vcSettle[vi]  = 0;
            target[vi]      = 0;
            continue;
        }
        // Soft-start: manuel open/open_slow modlarda hedefi yavas yavas artir.
        if (mode[vi] == 1 || mode[vi] == 3) {  // open / open_slow
            float dTgt = tgtMa[vi] - s_vcRampTgt[vi];
            float step = VC_RAMP_MA_PER_S * dt_s;
            if (fabsf(dTgt) <= step) {
                s_vcRampTgt[vi] = tgtMa[vi];
            } else {
                s_vcRampTgt[vi] += copysignf(step, dTgt);
            }
        } else {
            s_vcRampTgt[vi] = tgtMa[vi];
        }

        // Warm-start: 0→nonzero geçişinde, ilk çıkış tam V=IR noktasında olsun.
        //   d = VC_KP*err + integ; t=0'da err = tgtMa
        //   d_init = VC_KP*tgtMa + integ_seed = tgtMa*VC_SEED_K
        //   → integ_seed = tgtMa * (VC_SEED_K - VC_KP)
        // Böylece P aşması yok; çıkış doğrudan denge noktasında başlar.
        //
        // Ek: Hold kontrolü hedefi sürekli openMa↔closeMa arasında zıplattığında
        // integral eski denge noktasında takılı kalıyordu (KI*err ile unwind çok yavaş,
        // ~700ms). Bu süre içinde piston bir uçtan diğer uca uçuyordu.
        // Çözüm: target büyük adımla değiştiğinde integral'i yeni denge noktasına
        // FEEDFORWARD ile kaydır → komut anında doğru duty'yi versin.
        static constexpr float VC_DTGT_THRES = 30.0f;  // mA — bu üstü büyük adım sayılır
        float dTgt = tgtMa[vi] - s_vcPrevTgt[vi];
        if (s_vcPrevTgt[vi] <= 0.0f) {
            // Soğuk başlangıç (0 → nonzero)
            s_vcInteg[vi]  = tgtMa[vi] * (VC_SEED_K - VC_KP);
            s_vcSettle[vi] = VC_SETTLE_N;
        } else if (fabsf(dTgt) >= VC_DTGT_THRES) {
            // Sıçramalı hedef değişimi — integral'i feedforward kaydır
            s_vcInteg[vi] += dTgt * (VC_SEED_K - VC_KP);
            s_vcSettle[vi] = VC_SETTLE_N;  // L/R transient anti-windup penceresi
        }
        if (s_vcInteg[vi] < 0.0f)    s_vcInteg[vi] = 0.0f;
        if (s_vcInteg[vi] > 4095.0f) s_vcInteg[vi] = 4095.0f;
        s_vcPrevTgt[vi] = tgtMa[vi];

        // Regulator setpoint: rampa hedefi; bu sayede akim hedefi aniden atlamaz
        float err = s_vcRampTgt[vi] - inaI[VALVE_TO_INA_V2[vi]];

        // L/R transient sırasında integral windup'ı önle:
        // Akım hala yükseliyor (err > 0) ve daha settle dolmadıysa, integral biriktirme.
        // Negatif err (aşma) sırasında her zaman aşağı al — hızlı toparlama.
        bool settling = (s_vcSettle[vi] > 0);
        if (settling) s_vcSettle[vi]--;
        if (!(settling && err > 0.0f)) {
            s_vcInteg[vi] += VC_KI * err * dt_s;
        }
        if (s_vcInteg[vi] < 0.0f)    s_vcInteg[vi] = 0.0f;
        if (s_vcInteg[vi] > 4095.0f) s_vcInteg[vi] = 4095.0f;

        float d = VC_KP * err + s_vcInteg[vi];
        if (d < 0.0f)    d = 0.0f;
        if (d > 4095.0f) d = 4095.0f;
        target[vi] = (uint16_t)d;
    }
}

// -----------------------------------------------------------------------
// pcv_pi_step: PCV için g_valveCustomCurrent_mA hedefini yazar
//   Hold modu → 650mA; dışardan (kalibrasyon/GUI) ayarlanmışsa değiştirme
//   Gerçek PI artık valve_current_reg_step'te
// -----------------------------------------------------------------------
static void pcv_pi_step(uint16_t target[8], float dt_s) {
    (void)target; (void)dt_s;
    static const uint8_t PCV_VIDX[2]    = {1, 5};
    static const uint8_t GRP_PIST[2][3] = { {0, 1, 4}, {2, 3, 5} };
    static const float   PCV_TARGET_MA  = 650.0f;

    for (int g = 0; g < 2; g++) {
        uint8_t vi = PCV_VIDX[g];
        bool anyPistonActive = false;
        for (int i = 0; i < 3; i++) {
            uint8_t pp = GRP_PIST[g][i];
            if (pp < PISTON_CHANNEL_COUNT && g_pistonRuntime[pp].hold_mid_enable) {
                anyPistonActive = true; break;
            }
        }
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
            // Dışardan ayarlanmışsa (kalibrasyon/GUI) değiştirme
            if (g_valveCustomCurrent_mA[vi] <= 0.0f) {
                g_valveCustomCurrent_mA[vi] = anyPistonActive ? PCV_TARGET_MA : 0.0f;
            }
            xSemaphoreGive(g_sharedMutex);
        }
    }
}

// -----------------------------------------------------------------------
// autoStopCloseCurrent: current_ctrl close/close_slow sonrasi piston
// kapali konuma gelince akimi otomatik kes (surucu/sarj isinmasini onler)
// -----------------------------------------------------------------------
static void autoStopCloseCurrent() {
    static const int8_t VALVE_TO_PISTON[8] = { 1, -1, 0, 4, 3, -1, 5, 2 };
    static const float  CLOSE_MM = 2.0f;  // bu alti kapali kabul

    for (int vi = 0; vi < 8; vi++) {
        uint8_t mode = 0;
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            mode = g_valveCustomMode[vi];
            xSemaphoreGive(g_sharedMutex);
        }
        if (mode != 2 && mode != 4) continue;  // sadece close / close_slow

        int8_t pi = VALVE_TO_PISTON[vi];
        if (pi < 0) continue;  // PCV veya mapping yok

        float posMm = 0.0f;
        bool got = false;
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            posMm = g_pistonHallmm[pi];
            got = true;
            xSemaphoreGive(g_sharedMutex);
        }
        if (!got) continue;

        float strokeMm = (pi < 4) ? PISTON_DEFAULT_STROKE_MM : PIST_K12_STROK;
        if (g_tmagPistonCalib[pi].valid && g_tmagPistonCalib[pi].strokeMm > 5.0f) {
            strokeMm = g_tmagPistonCalib[pi].strokeMm;
        }

        if (posMm <= CLOSE_MM) {
            if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
                g_valveCustomCurrent_mA[vi] = 0.0f;
                g_valveCustomMode[vi] = 0;  // off
                xSemaphoreGive(g_sharedMutex);
            }
            char msg[80];
            snprintf(msg, sizeof(msg), "[CURR_CLOSE] vi=%d piston=%d pos=%.1fmm -> auto off", vi, (int)pi, posMm);
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        }
    }
}

// -----------------------------------------------------------------------
// holdcontrol_V2 — artık piston_ctrl_step'e ince kaplama (backward compat)
// -----------------------------------------------------------------------
void holdcontrol_V2(uint16_t target[], bool suppressed) {
    piston_ctrl_step(target, suppressed);
}

static void holdcontrol_V2_UNUSED_BODY_KEPT_FOR_REFERENCE(uint16_t target[], bool suppressed) {
    (void)target; (void)suppressed;  // Kullanılmıyor - referans için tutuldu
    static const uint8_t VALVE_IDX[6] = {2, 0, 7, 4, 3, 6};
    static uint32_t s_v2LastMs[6]    = {0};
    static float    s_v2RampedPWM[6] = {800.0f, 800.0f, 800.0f, 800.0f, 800.0f, 800.0f};
    static float    s_v2RCoil[6]     = {7.0f, 7.0f, 7.0f, 7.0f, 7.0f, 7.0f};
    static constexpr float V_SUPPLY = 12.0f;

    for (int p = 0; p < 6; p++) {
        uint8_t valveIdx = VALVE_IDX[p];
        uint8_t inaIdx   = VALVE_TO_INA_V2[valveIdx];  // Bu valfe ait INA219 indeksi

        // ---- VERİ TOPLAMA ----
        bool   holdEnabled = false;
        float  xRef        = 0.5f;   // Hedef pozisyon oranı (0.0-1.0)
        float  posMm       = 0.0f;   // Anlık piston konumu (mm)
        float  iAct        = 0.0f;   // Anlık valf akımı (mA)
        float  vCoil       = 0.0f;   // Valf bobinine uygulanan anlık voltaj (V)
        float  calibHoldMa = 0.0f, calibOpenMa = 0.0f, calibCloseMa = 0.0f;  // K1/K2 kalibre akimlar

        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            holdEnabled = g_pistonRuntime[p].hold_mid_enable;
            xRef        = g_pistonRuntime[p].x_ref;
            posMm       = g_pistonHallmm[p];
            iAct        = g_tele.inaI_mA[inaIdx];
            vCoil       = g_tele.inaV[inaIdx];     // Volt cinsinden
            if (p >= 4) {
                calibHoldMa  = g_pistonCalibData[p].hold_mA;
                calibOpenMa  = g_pistonCalibData[p].open_mA;
                calibCloseMa = g_pistonCalibData[p].close_mA;
            }
            xSemaphoreGive(g_sharedMutex);
        }

        // Hold devre dışı veya bastırıldıysa → valfi serbest bırak
        if (!holdEnabled || suppressed) {
            s_directPWMActive[valveIdx] = false;
            s_currHoldActive[valveIdx]  = false;
            ValveCurrentControl_Enable(valveIdx, false);
            s_v2RampedPWM[p] = (float)target[valveIdx];  // Mevcut değerden başla (yumuşak yeniden başlatma)
            continue;
        }

        // ---- DESTEK (PCV) VALFİNİ GARANTİLİ AÇ ----
        // Manuel hold aktifken ilgili piston grubunun support/PCV valfi deterministik şekilde açık tutulur.
        // Not: Burada sadece target[supportIdx] > 0 garanti edilir; akım PI kontrolü mevcut Task döngüsündeki
        // ValveCurrentControl bloğu tarafından (VALVE_MODE_PCV) yönetilmeye devam eder.
        int supportIdx = -1;
        if (p >= 0 && p < PISTON_CHANNEL_COUNT) {
            supportIdx = PISTON_SUPPORT_VALVE_INDEX[p];
            if (supportIdx >= 0 && supportIdx < 8) {
                target[supportIdx] = SUPPORT_VALVE_DUTY;  // PCV yolu açık kalmalı
            }
        }

        // ---- KONUM HESABI ----
        static constexpr float STROKE_MM = 26.0f;
        float tgtMm = (p >= 4) ? 10.0f : STROKE_MM * xRef;  // K1/K2: 10mm sabit, diger: xRef * stroke
        float error = posMm - tgtMm;       // + = hedefin üstünde, - = hedefin altında

        // ================================================================
        // KULLANICI KONTROL MANTIĞI - CurOut: valfe verilecek hedef akım (mA)
        // ================================================================
        float CurOut;

        // Orantılı akım kontrolü: CurOut = holdCurr - (Kp × error)
        // error > 0 → piston hedefin üstünde → akım azalt → kapanır
        // error < 0 → piston hedefin altında → akım artır → açılır
        // Kp=12 mA/mm: hedeften 4mm uzakta 48mA düzeltme
        //
        // PER-PİSTON KALİBRASYON DEĞERLERİ:
        // holdCurr  = pistonun hedef noktada (13mm) dengeleneceği akım
        // maxOpen   = maksimum açma akımı (holdCurr + ~150mA)
        // maxClose  = minimum kapama akımı (holdCurr - ~80mA)
        //
        // Kalibrasyon: Manuel sayfasından valfe sabit akım ver, pistonun
        // 13mm'de sabit kaldığı akımı bul → holdCurr olarak gir.
        //
        // P1-3/P2-4/P5-7/P6-R: YAY YOK - sadece N-valfi ile acilir/kapanir
        // Kapanma yonu: dusuk akim (250mA civarı) → N-valfi kapanir → piston kapanir
        // Acilma yonu: yuksek akim (850mA civarı) → N-valfi acilir → piston acilir
        // HOLD_CURR: hedef noktada (~13mm) denge akimi (pistona gore kalibre edilmeli)
        // MAX_CLOS_CURR: gercek kapanma akimi araligina indirildi (eski 410-480mA → 250-280mA)
        //                                          P0      P1      P2      P3
        static constexpr float HOLD_CURR[4]     = { 500.0f, 500.0f, 500.0f, 490.0f };
        static constexpr float MAX_OPEN_CURR[4] = { 600.0f, 610.0f, 610.0f, 620.0f };  // Azaltildi: daha yavas acilma, PWM ramp lag azaltildi
        static constexpr float MAX_CLOS_CURR[4] = { 450.0f, 450.0f, 450.0f, 420.0f };  // p0: 420→460 (ani kapanmayi fren - 420mA'da snapback gozlendi)
        static constexpr float CURR_KP_V2       = 50.0f;   // mA/mm: 75→50 (salınim azaltıldı, hızlı PWM_RAMP ile dengeli)

        // K1/K2 icin kalibre veya varsayilan akimlar
        float holdCurr, maxOpenCurr, maxClosCurr;
        if (p < 4) {
            holdCurr    = HOLD_CURR[p];
            maxOpenCurr = MAX_OPEN_CURR[p];
            maxClosCurr = MAX_CLOS_CURR[p];
        } else {
            holdCurr    = (calibHoldMa  > 100.0f) ? calibHoldMa  : 500.0f;
            maxOpenCurr = (calibOpenMa  > 100.0f) ? calibOpenMa  : holdCurr + 130.0f;
            maxClosCurr = (calibCloseMa > 100.0f) ? calibCloseMa : holdCurr - 80.0f;
        }

        CurOut = holdCurr - (CURR_KP_V2 * error);

        if      (CurOut > maxOpenCurr) CurOut = maxOpenCurr;
        else if (CurOut < maxClosCurr) CurOut = maxClosCurr;
        // ================================================================
        // KONTROL MANTIĞI SONU
        // ================================================================

        // ---- ÇIKIŞ UYGULAMASI (PI YOK - Doğrudan PWM rampa) ----
        // Hedef PWM: V = I * R  →  PWM = (I * R_est / V_supply) * 4095
        float tgtPWM = (CurOut / 1000.0f * s_v2RCoil[p] / V_SUPPLY) * 4095.0f;

        // ---- DİNAMİK DİRENÇ TAHMİNİ (EMA) ----
        // inaV PWM ortamında güvenilmez → V_supply ve bilinen PWM'den hesapla
        // Sadece rampa tamamlanmış (PWM kararlı) ve yeterli akım varken güncelle
        {
            bool pwmStable = (fabsf(s_v2RampedPWM[p] - tgtPWM) < 30.0f);
            if (pwmStable && iAct > 150.0f && s_v2RampedPWM[p] > 200.0f) {
                float v_est      = V_SUPPLY * (s_v2RampedPWM[p] / 4095.0f);
                float r_measured = v_est / (iAct / 1000.0f);  // Ω
                if (r_measured > 4.0f && r_measured < 12.0f) {
                    s_v2RCoil[p] = 0.95f * s_v2RCoil[p] + 0.05f * r_measured;  // EMA alfa=0.05
                }
            }
        }

        // Slew rate: her çağrıda max PWM_RAMP kadar değiş (~10ms döngü → 2000 PWM/s)
        static constexpr float PWM_RAMP = 25.0f;  // 20→150: PWM setpoint degisimini piston hizinda takip edebilmek icin
        if      (s_v2RampedPWM[p] < tgtPWM - PWM_RAMP) s_v2RampedPWM[p] += PWM_RAMP;
        else if (s_v2RampedPWM[p] > tgtPWM + PWM_RAMP) s_v2RampedPWM[p] -= PWM_RAMP;
        else                                             s_v2RampedPWM[p]  = tgtPWM;

        if (s_v2RampedPWM[p] < 0.0f)    s_v2RampedPWM[p] = 0.0f;
        if (s_v2RampedPWM[p] > 4095.0f) s_v2RampedPWM[p] = 4095.0f;

        // Doğrudan target[]'a yaz — PI devre dışı
        target[valveIdx] = (uint16_t)s_v2RampedPWM[p];
        s_directPWMActive[valveIdx] = true;   // AKIM KONTROLÜ bloğuna bu valfi atla dedirt
        s_currHoldActive[valveIdx]  = false;
        ValveCurrentControl_Enable(valveIdx, false);

        // ---- DEBUG LOG (500ms'de bir) ----
        uint32_t nowMs = millis();
        if (nowMs - s_v2LastMs[p] >= 500) {
            s_v2LastMs[p] = nowMs;
            {
                // KISA 1. SATIR: çekirdek kontrol bilgisi (kısa tut)
                char dbg1[160];
                snprintf(dbg1, sizeof(dbg1),
                    "[HOLD_V2] p%d pos=%.1f tgt=%.1f e=%.1f I_out=%.0f I_act=%.0f R=%.2f pwm=%u tgtPWM=%.0f",
                    p, posMm, tgtMm, error, CurOut, iAct, s_v2RCoil[p],
                    target[valveIdx], tgtPWM);
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, dbg1);

                // KISA 2. SATIR: PCV bilgisi (ayrı etiketle)
                char dbg2[96];
                const char* pcvName = (supportIdx >= 0 && supportIdx < 8) ? VALVE_NAME[supportIdx] : "-";
                uint16_t pcvTgt = (supportIdx >= 0 && supportIdx < 8) ? target[supportIdx] : 0;
                snprintf(dbg2, sizeof(dbg2),
                    "[HOLD_V2_PCV] p%d pcv=%s pcvTarget=%u ensured=%d",
                    p, pcvName, pcvTgt, (pcvTgt > 0) ? 1 : 0);
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, dbg2);
            }
        }
    }
}

// ===============================
// EFUSE (artık kullanılmıyor - pinler valf temizleme için ayrıldı)
// ===============================
static inline void efuse_on()  { /* no-op */ }
static inline void efuse_off() { /* no-op */ }
static inline int  efuse_pg()  { return 1; /* always OK */ }

// ===============================
// TCA9555 yardımcıları (I2C mutex korumalı)
// ===============================
static void tcaWrite8(uint8_t a, uint8_t r, uint8_t v){
    if (g_i2cMutex && xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        Wire.beginTransmission(a);
        Wire.write(r);
        Wire.write(v);
        Wire.endTransmission();
        xSemaphoreGive(g_i2cMutex);
    }
}

static uint8_t tcaRead8(uint8_t a, uint8_t r){
    uint8_t result = 0xFF;
    if (g_i2cMutex && xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        Wire.beginTransmission(a);
        Wire.write(r);
        Wire.endTransmission(false);
        Wire.requestFrom(a, (uint8_t)1);
        if (Wire.available())
            result = Wire.read();
        xSemaphoreGive(g_i2cMutex);
    }
    return result;
}

// P0 alt 4 bit çıkış, üst 4 bit giriş, P1 hepsi giriş
static void TCA_ConfigOutputs_P0x0to3(uint8_t a){
    tcaWrite8(a, TCA_REG_CONFIG0, 0xF0);  // P0[0..3]=0 (out), P0[4..7]=1 (in)
    tcaWrite8(a, TCA_REG_CONFIG1, 0xFF);  // P1 hepsi input
}

static uint8_t s_outP0_cache_20 = 0x0F;
static uint8_t s_outP0_cache_21 = 0x0F;

static void TCA_SetP0Bit(uint8_t a, uint8_t bit, bool level){
    uint8_t &c = (a == TCA0) ? s_outP0_cache_20 : s_outP0_cache_21;
    if (level) c |= (1u << bit);
    else       c &= ~(1u << bit);
    tcaWrite8(a, TCA_REG_OUTPUT0, c);
}

static void TCA_InitOne(uint8_t a){
    TCA_ConfigOutputs_P0x0to3(a);
    tcaWrite8(a, TCA_REG_OUTPUT0, (a == TCA0) ? s_outP0_cache_20 : s_outP0_cache_21);
    tcaWrite8(a, TCA_REG_OUTPUT1, 0xFF);
}

static void TCA_InitAll(){
    TCA_InitOne(TCA0);
    TCA_InitOne(TCA1);
}

// ===============================
// DRV8243
// ===============================
struct DrvMap { 
    uint8_t tca; uint8_t csBit; uint8_t offBit; uint8_t nFaultBit; 
};

static const DrvMap DRV[4] = {
    {TCA0, 0, 2, 4},  // DRV1 -> N433(Out1),N436(Out2)
    {TCA0, 1, 3, 5},  // DRV2 -> N434(Out1),N435(Out2)
    {TCA1, 0, 2, 4},  // DRV3 -> N439(Out1),N437(Out2)
    {TCA1, 1, 3, 5}   // DRV4 -> N438(Out1),N440(Out2)
};

// DRVOFF aktif-düşük
void DRV_EnableAll(bool en){
    for (int i = 0; i < 4; i++) {
        TCA_SetP0Bit(DRV[i].tca, DRV[i].offBit, !en);
    }
}

// --- bitbang SPI ---
static void spi_bb_begin(){
    pinMode(SPI_SCK,  OUTPUT);
    pinMode(SPI_MOSI, OUTPUT);
    pinMode(SPI_MISO, INPUT);
    digitalWrite(SPI_SCK, LOW);
    digitalWrite(SPI_MOSI, LOW);
}

static uint16_t bb16(uint16_t tx){
    uint16_t rx = 0;
    for (int i = 15; i >= 0; --i) {
        digitalWrite(SPI_SCK, LOW);
        digitalWrite(SPI_MOSI, (tx >> i) & 1);
        delayMicroseconds(1);
        digitalWrite(SPI_SCK, HIGH);
        delayMicroseconds(1);
        rx = (rx << 1) | (uint8_t)digitalRead(SPI_MISO);
    }
    digitalWrite(SPI_SCK, LOW);
    delayMicroseconds(1);
    return rx;
}

static inline uint16_t drv_xfer16(int idx, uint16_t frame){
    // CS LOW
    TCA_SetP0Bit(DRV[idx].tca, DRV[idx].csBit, false);
    delayMicroseconds(1);
    uint16_t r = bb16(frame);
    // CS HIGH
    TCA_SetP0Bit(DRV[idx].tca, DRV[idx].csBit, true);
    delayMicroseconds(1);
    return r;
}

static inline void DRV_WriteReg(int idx, uint8_t addr, uint8_t val){
    uint16_t cmd = ((addr & 0x3F) << 8) | (uint16_t)val;
    drv_xfer16(idx, cmd);
}

static inline uint8_t DRV_ReadReg(int idx, uint8_t addr){
    // read bit = bit14
    uint16_t cmd = (1u << 14) | ((addr & 0x3F) << 8);
    uint16_t r = drv_xfer16(idx, cmd);
    return (uint8_t)(r & 0xFF);
}

void DRV_FaultClear(int idx){
    DRV_WriteReg(idx, 0x08, 0x80);
}

void DRV_GetAllStatus(DRV8243Status status[4]){
    // Register adresleri (datasheet):
    // 0x01 = FAULT_SUMMARY
    // 0x02 = STATUS1: Bit7=OLA1, Bit6=OLA2, Bit3-0=OCP bitleri
    // 0x03 = STATUS2
    // 0x0C = CONFIG3: S_MODE bitleri — write-back SPI testi için kullanılır
    //
    // ok kriteri: SPI write-back testi (OLA/OCP valf bağlı olmasa da set olur,
    // gerçek DRV sağlığını yansıtmaz. CONFIG3'e test değeri yaz, oku, restore et.)
    static const uint8_t TEST_VAL    = 0x05;  // S_MODE=01 + rezerve bit — normal değerden farklı
    static const uint8_t RESTORE_VAL = 0x01;  // Normal: S_MODE=01 (Independent half-bridge)
    for (int i = 0; i < 4; i++) {
        status[i].st1 = DRV_ReadReg(i, 0x01);
        status[i].st2 = DRV_ReadReg(i, 0x02);
        status[i].flt = DRV_ReadReg(i, 0x03);
        // SPI write-back testi
        DRV_WriteReg(i, 0x0C, TEST_VAL);
        delayMicroseconds(50);
        uint8_t readback = DRV_ReadReg(i, 0x0C);
        DRV_WriteReg(i, 0x0C, RESTORE_VAL);  // Normal değere geri yükle
        status[i].ok = (readback == TEST_VAL);
    }
}

static inline void DRV_ApplyValvePreset(int idx){
    // COMMAND (0x08): 0x90 = CLR_FLT(1) + SPI_IN_LOCK(01) = fault temizle, SPI_IN kilitli
    DRV_WriteReg(idx, 0x08, 0x90);
    // CONFIG1 (0x0A): 0x80 = EN_OLA(1) + OCP_RETRY(0) + TSD_RETRY(0)
    // OCP_RETRY=0: kısa devre sonrası chip RETRY YAPMAZ — çıkış hi-Z kalır
    // TSD_RETRY=0: termal shutdown sonrası chip kendiliğinden AÇILMAZ — yanmayı önler
    // Yeniden açmak için: CLR_FLT + firmware g_drvOcpLatch reset (kullanıcı onayı)
    DRV_WriteReg(idx, 0x0A, 0x80);
    // CONFIG2 (0x0B): 0x64 = S_DIAG(11) + S_ITRIP(100)
    DRV_WriteReg(idx, 0x0B, 0x64);
    // CONFIG3 (0x0C): 0x01 = S_MODE(01) = Independent half-bridge mode
    DRV_WriteReg(idx, 0x0C, 0x01);
}

void DRV_PresetAll(){
    spi_bb_begin();
    // CS’leri idle high yap
    for (int i = 0; i < 4; i++)
        TCA_SetP0Bit(DRV[i].tca, DRV[i].csBit, true);

    delay(1);
    for (int i = 0; i < 4; i++) {
        DRV_FaultClear(i);
        delay(1);
        DRV_ApplyValvePreset(i);
        delay(1);
    }
}

// ===============================
// LEDC
// ===============================
static void PWM_AttachSafe(uint8_t pin, int ch){
    ledcDetachPin(pin);
    ledcSetup(ch, PWM_FREQ, PWM_RES);
    ledcWrite(ch, 0);
    ledcAttachPin(pin, ch);
}

static void PWM_InitAll(){
    for (int i = 0; i < 8; i++) {
        PWM_AttachSafe(VALVE_PIN[i], VALVE_CH[i]);
    }
}

static inline void PWM_WriteDuty(int idx, uint16_t duty){
    if (idx < 0 || idx > 7) return;
    if (duty > DUTY_MAX) duty = DUTY_MAX;
    ledcWrite(VALVE_CH[idx], duty);
    if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            g_valveDutyCounts[idx]=duty;
            xSemaphoreGive(g_sharedMutex);
        }
    

}

static inline void pumpSendCommand(PumpCmd cmd){
    if (!g_sharedMutex) return;
    if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_pumpCmd.cmd = cmd;
        g_pumpCmd.seq++;
        xSemaphoreGive(g_sharedMutex);
    }
}

static float getSystemPressureBar(){
    float bar = g_pumpPub.bar;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        bar = g_pumpPub.bar;
        xSemaphoreGive(g_sharedMutex);
    }
    return bar;
}

static void pistonCalReset(PistonCalRuntime &cal){
    cal.stage = PistonCalRuntime::IDLE;
    cal.active = false;
    g_pistonCalRunning = 0;
    g_pistonCalPhase   = 0;
    cal.valveIdx = -1;
    cal.supportIdx = -1;
    cal.stageStartMs = 0;
    cal.closedRaw = 0.0f;
    cal.openRaw = 0.0f;
    cal.lastRaw = 0.0f;
    cal.lastChangeMs = 0;
    cal.pumpRequested = false;
    cal.pressureTargetBar = CAL_PRESSURE_TARGET_BAR;
    cal.lastPumpReqMs = 0;
    // Hold PWM alanları
    cal.findHold = false;
    cal.holdPwmLo = 800;
    cal.holdPwmHi = 1400;
    cal.holdPwmTest = 1000;
    cal.holdPwmResult = 0;
    cal.midRaw = 0.0f;
    cal.holdLastRaw = 0.0f;
    cal.holdStableStartMs = 0;
    cal.holdAtMid = false;
    cal.holdIteration = 0;
    // Ardışık kalibrasyon - bunları reset etme (calibrateAll ve nextPiston korunmalı)
}


static bool readPistonRawSample(PistonChannel piston, float &raw){
    if (!g_sharedMutex) return false;
    if (piston >= PISTON_CHANNEL_COUNT) return false;
    
    // TMAG sensör kullan (DRV yerine!)
    uint8_t tmagCh = PISTON_TO_TMAG[piston];
    if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        // TMAG Z eksenini oku (int16 -> float)
        raw = (float)g_tmagData[tmagCh].z;
        xSemaphoreGive(g_sharedMutex);
        return g_tmagData[tmagCh].valid;
    }
    return false;
}

// K1/K2 için ikinci TMAG sensörü oku (-1 ise bu piston için sensör 2 yok)
static bool readPistonRawSample2(PistonChannel piston, float &raw){
    if (!g_sharedMutex) return false;
    int8_t tmagCh2 = PISTON_TO_TMAG2[piston];
    if (tmagCh2 < 0) return false;  // İkinci sensör yok
    if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
        raw = (float)g_tmagData[(uint8_t)tmagCh2].z;
        bool valid = g_tmagData[(uint8_t)tmagCh2].valid;
        xSemaphoreGive(g_sharedMutex);
        return valid;
    }
    return false;
}

// Eski çok noktalı kalibrasyon kaldırıldı - artık basit 2 noktalı kalibrasyon kullanılıyor
// pistonCalPublishSimple fonksiyonu kullanılıyor

// Yeni basit kalibrasyon için publish fonksiyonu
static void pistonCalPublishSimple(PistonCalRuntime &cal){
    float rawRange = fabsf(cal.openRaw - cal.closedRaw);
    
    // TMAG için minimum 2000 birim değişim beklenir
    if (rawRange < 2000.0f) {
        {
            char msg[120];
            snprintf(msg, sizeof(msg), "[CAL] ERROR piston %d - TMAG range too small: %.0f (closed:%.0f open:%.0f)",
                     (int)cal.piston, rawRange, cal.closedRaw, cal.openRaw);
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
            char json[160];
            snprintf(json, sizeof(json), 
                     "{\"cmd\":\"piston_cal_done\",\"p\":%u,\"ok\":0,\"err\":\"TMAG range %.0f < 2000\"}",
                     (unsigned)cal.piston, rawRange);
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, json);
        }
        return;
    }
    
    // Kalibrasyon verilerini kaydet
    float minRaw = (cal.closedRaw < cal.openRaw) ? cal.closedRaw : cal.openRaw;
    float maxRaw = (cal.closedRaw > cal.openRaw) ? cal.closedRaw : cal.openRaw;
    
    // zMid her zaman 0: piecewise path devre dışı, power-law (n=0.39) kullan
    // measuredMidRaw sadece hold PWM doğrulaması için saklanır, pozisyon hesabında KULLANILMAZ
    float midRaw = 0.0f;
    
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        // Manuel referansları kaydet
        g_pistonManualRef[cal.piston].raw[PISTON_REF_CLOSED] = cal.closedRaw;
        g_pistonManualRef[cal.piston].raw[PISTON_REF_OPEN] = cal.openRaw;
        g_pistonManualRef[cal.piston].raw[PISTON_REF_MID] = midRaw;
        g_pistonManualRef[cal.piston].validMask = 0x07;  // Hepsi geçerli
        
        // Kalibrasyon tablosunu da basitçe doldur (2 nokta)
        g_pistonCalTable[cal.piston].numPoints = 2;
        g_pistonCalTable[cal.piston].raw[0] = cal.closedRaw;
        g_pistonCalTable[cal.piston].raw[1] = cal.openRaw;
        g_pistonCalTable[cal.piston].mm[0] = 0.0f;
        g_pistonCalTable[cal.piston].mm[1] = cal.strokeMm;
        g_pistonCalSeq++;
        
        // TMAG kalibrasyon tablosunu da güncelle (pozisyon hesaplama için)
        g_tmagPistonCalib[cal.piston].zMin = (int16_t)cal.closedRaw;
        g_tmagPistonCalib[cal.piston].zMax = (int16_t)cal.openRaw;
        g_tmagPistonCalib[cal.piston].zMid = (int16_t)midRaw;  // Manuel orta değer
        g_tmagPistonCalib[cal.piston].strokeMm = cal.strokeMm;
        g_tmagPistonCalib[cal.piston].valid = true;
        // K1/K2 için ikinci sensör verilerini de kaydet (ratiometrik ölçüm için)
        if (cal.piston == PISTON_K1 || cal.piston == PISTON_K2) {
            bool s2Valid = (fabsf(cal.closedRaw2) > 10.0f && fabsf(cal.openRaw2) > 10.0f);
            g_tmagPistonCalib[cal.piston].zMin2 = (int16_t)cal.closedRaw2;
            g_tmagPistonCalib[cal.piston].zMax2 = (int16_t)cal.openRaw2;
            g_tmagPistonCalib[cal.piston].hasSensor2 = s2Valid;
        }
        g_tmagCalibSeq++;
        
        // PistonCalibData güncelle (hold için gerekli)
        g_pistonCalibData[cal.piston].calibrated = true;
        g_pistonCalibData[cal.piston].duty_hold = cal.holdPwmResult;
        g_pistonCalibData[cal.piston].duty_open_thresh = cal.pwmOpenThresh;
        g_pistonCalibData[cal.piston].duty_close_thresh = cal.pwmCloseThresh;
        if (cal.piston == PISTON_K1 || cal.piston == PISTON_K2) {
            // K1/K2: Phase 7 açık/kapalı hedefleri mm*10 formatında bekler.
            // Açık sensör pozisyonu 0..stroke mm arası; ham TMAG Z değil.
            g_pistonCalibData[cal.piston].min_raw = 0;
            g_pistonCalibData[cal.piston].max_raw = (uint16_t)(cal.strokeMm * 10.0f);
            g_pistonCalibData[cal.piston].mid_raw = (uint16_t)(cal.strokeMm * 5.0f);
        } else {
            g_pistonCalibData[cal.piston].min_raw = (uint16_t)minRaw;
            g_pistonCalibData[cal.piston].max_raw = (uint16_t)maxRaw;
            g_pistonCalibData[cal.piston].mid_raw = (uint16_t)midRaw;
        }
        
        xSemaphoreGive(g_sharedMutex);
    }
    
    // EEPROM'a kaydet
    PistonRefPrefs_Save((int)cal.piston);
    TMAGCalib_SavePiston(cal.piston);
    
    {
        char msg[140];
        snprintf(msg, sizeof(msg), "[CAL] piston %d OK: closed=%.0f open=%.0f range=%.0f hold=%u openPwm=%u closePwm=%u",
                 (int)cal.piston, cal.closedRaw, cal.openRaw, rawRange, 
                 cal.holdPwmResult, cal.pwmOpenThresh, cal.pwmCloseThresh);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        char json[256];
        snprintf(json, sizeof(json), 
                 "{\"cmd\":\"piston_cal_done\",\"p\":%u,\"ok\":1,\"closed\":%.0f,\"open\":%.0f,\"stroke\":%.1f,\"hold\":%u,\"openPwm\":%u,\"closePwm\":%u}",
                 (unsigned)cal.piston, cal.closedRaw, cal.openRaw, cal.strokeMm, 
                 cal.holdPwmResult, cal.pwmOpenThresh, cal.pwmCloseThresh);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, json);
    }
}

static void pistonCalStart(const PistonCalibrationRequest &req){
    if (req.piston >= PISTON_CHANNEL_COUNT) return;
    pistonCalReset(s_pistonCal);
    g_pistonCalRunning = 1;
    g_pistonCalPhase   = 1;  // Basinc / bekleniyor
    s_pistonCal.piston = (PistonChannel)req.piston;
    s_pistonCal.valveIdx = PISTON_VALVE_INDEX[req.piston];
    s_pistonCal.supportIdx = PISTON_SUPPORT_VALVE_INDEX[req.piston];
    if (s_pistonCal.valveIdx < 0) return;
    s_pistonCal.pwmDuty = req.pwmDuty ? req.pwmDuty : DEFAULT_CAL_PWM;
    s_pistonCal.settleMs = req.settleMs ? req.settleMs : DEFAULT_CAL_SETTLE_MS;
    s_pistonCal.strokeMm = PISTON_STROKE_MM[req.piston];
    s_pistonCal.pressureTargetBar = (req.pressureTargetBar >= 5.0f) ? req.pressureTargetBar : CAL_PRESSURE_TARGET_BAR;
    s_pistonCal.findHold = req.findHold;  // Hold PWM bulma aktif mi?
    s_pistonCal.calibrateAll = req.calibrateAll;  // Tüm pistonları kalibre et modu
    s_pistonCal.nextPiston = req.calibrateAll ? (req.piston + 1) : 0;  // Sonraki piston
    // Yeni akış: önce basınç doldur
    s_pistonCal.stage = PistonCalRuntime::WAIT_PRESSURE;
    s_pistonCal.stageStartMs = millis();
    s_pistonCal.active = true;
    {
        char msg[140];
        snprintf(msg, sizeof(msg), "[CAL] start piston %d valve=%d support=%d duty=%u target=%.1fbar findHold=%d",
                 req.piston, s_pistonCal.valveIdx, s_pistonCal.supportIdx, 
                 s_pistonCal.pwmDuty, s_pistonCal.pressureTargetBar, req.findHold);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
    }
}

// Stabilite için sabitler
static constexpr float    CAL_STABLE_THRESHOLD = 50.0f;   // TMAG değişim eşiği
static constexpr uint32_t CAL_STABLE_TIME_MS   = 500;     // Bu süre değişim yoksa stabil
static constexpr uint32_t CAL_STABLE_TIMEOUT_MS = 5000;   // Maksimum bekleme

static void pistonCalProcess(PistonCalRuntime &cal, uint16_t target[8]){
    if (cal.stage == PistonCalRuntime::IDLE || cal.valveIdx < 0) return;
    uint32_t now = millis();
    
    // Valf kontrolü stage'e göre yapılıyor
    // WAIT_PRESSURE: Tüm valfler kapalı (basınç doldurulurken)
    // OPEN_VALVES ve sonrası: Basınç valfleri açık
    // OPEN_PISTON ve sonrası: Hedef piston valfi de açık
    // FIND_HOLD_*: Hold PWM test değeri uygulanıyor
    for (int i = 0; i < 8; i++) {
        if (cal.stage == PistonCalRuntime::WAIT_PRESSURE) {
            // Basınç doldururken TÜM valfler kapalı
            target[i] = 0;
        } else if (i == 1 || i == 5) {
            // Basınç valfleri açık (OPEN_VALVES ve sonrası)
            target[i] = 2000;
        } else if (cal.stage == PistonCalRuntime::CLOSE_FOR_SCAN && i == cal.valveIdx) {
            // Tarama öncesi pistonu kapat (PWM=0)
            target[i] = 0;
        } else if ((cal.stage == PistonCalRuntime::FIND_PWM_OPEN_THRESH || 
                    cal.stage == PistonCalRuntime::FIND_PWM_CLOSE_THRESH) && i == cal.valveIdx) {
            // PWM eşik taraması: scan PWM değerini uygula
            target[i] = cal.pwmScanCurrent;
        } else if (cal.stage == PistonCalRuntime::FIND_HOLD_PARK && i == cal.valveIdx) {
            // PARK: Bang-bang kontrol - eşiği aşacak PWM uygula
            // Orantılı kontrol yetersiz: piston kapalıyken normDiff küçük kalır
            // ve hesaplanan PWM açılma eşiğinin altında çıkar → piston hareket etmez
            float raw;
            if (readPistonRawSample(cal.piston, raw)) {
                float diff = raw - cal.midRaw;
                float tolerance = fabsf(cal.openRaw - cal.closedRaw) * 0.05f;
                if (tolerance < 100.0f) tolerance = 100.0f;

                if (diff < -tolerance) {
                    // Ortanın gerisinde (kapalıya yakın) → açılma eşiğini geç
                    target[i] = cal.holdPwmHi + 100;
                } else if (diff > tolerance) {
                    // Ortanın ilerisinde (açığa yakın) → kapanma eşiğinin altına in
                    target[i] = (cal.holdPwmLo > 100) ? cal.holdPwmLo - 100 : 0;
                } else {
                    // Tolerans içinde → hold PWM uygula
                    target[i] = cal.holdPwmTest;
                }
            } else {
                target[i] = cal.holdPwmTest;
            }
        } else if (cal.stage == PistonCalRuntime::FIND_HOLD_MEASURE && i == cal.valveIdx) {
            // MEASURE: Test PWM değeri uygula
            target[i] = cal.holdPwmTest;
        } else if (cal.stage >= PistonCalRuntime::OPEN_PISTON && 
                   cal.stage <= PistonCalRuntime::READ_OPEN && i == cal.valveIdx) {
            // Piston açma aşamasında sadece hedef piston açık
            target[i] = cal.pwmDuty;
        } else {
            target[i] = 0;
        }
    }
    
    switch (cal.stage) {
        case PistonCalRuntime::WAIT_PRESSURE: {
            // Üretici modunda basınç kontrolü bypass edilir
            if (g_manufacturerMode) {
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[MFG] Pressure check bypassed");
                cal.stage = PistonCalRuntime::OPEN_VALVES;
                cal.stageStartMs = now;
                cal.pumpRequested = false;
                break;
            }
            // Önce basınç doldur (tüm valfler kapalı)
            float bar = getSystemPressureBar();
            bool enough = (bar >= cal.pressureTargetBar);
            bool timeout = (now - cal.stageStartMs) >= CAL_PRESSURE_WAIT_TIMEOUT_MS;
            
            if (enough || timeout) {
                if (timeout) {
                    char msg[96];
                    snprintf(msg, sizeof(msg), "[CAL] pressure timeout -> %.1f/%.1f bar", bar, cal.pressureTargetBar);
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                }
                // Basınç valflerini aç
                cal.stage = PistonCalRuntime::OPEN_VALVES;
                cal.stageStartMs = now;
                cal.pumpRequested = false;
                {
                    char msg[96];
                    snprintf(msg, sizeof(msg), "[CAL] pressure OK %.1fbar, opening valves...", bar);
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                }
                break;
            }
            
            if (!cal.pumpRequested || (now - cal.lastPumpReqMs) > PUMP_RETRY_INTERVAL_MS) {
                pumpSendCommand(PUMP_CMD_START);
                cal.pumpRequested = true;
                cal.lastPumpReqMs = now;
                {
                    char msg[96];
                    snprintf(msg, sizeof(msg), "[CAL] filling pressure %.1f/%.1f bar", bar, cal.pressureTargetBar);
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                }
            }
            break;
        }
        
        case PistonCalRuntime::OPEN_VALVES: {
            // Basınç valflerini aç, 1 saniye bekle (pistonlar kapansın)
            if (now - cal.stageStartMs >= 1000) {
                cal.stage = PistonCalRuntime::READ_CLOSED;
                cal.stageStartMs = now;
                {
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[CAL] reading closed position...");
                }
            }
            break;
        }
        
        case PistonCalRuntime::READ_CLOSED: {
            // Kapalı konumu oku
            float raw;
            if (readPistonRawSample(cal.piston, raw)) {
                cal.closedRaw = raw;
                // K1/K2 için ikinci sensörü de oku
                float raw2 = 0.0f;
                if (readPistonRawSample2(cal.piston, raw2)) {
                    cal.closedRaw2 = raw2;
                }
                cal.lastRaw = raw;
                cal.lastChangeMs = now;
                cal.stage = PistonCalRuntime::OPEN_PISTON;
                cal.stageStartMs = now;
                {
                    char msg[120];
                    if (cal.closedRaw2 != 0.0f) {
                        snprintf(msg, sizeof(msg), "[CAL] closed s1=%.0f s2=%.0f, opening piston duty=%u...", cal.closedRaw, cal.closedRaw2, cal.pwmDuty);
                    } else {
                        snprintf(msg, sizeof(msg), "[CAL] closed=%.0f, opening piston duty=%u...", cal.closedRaw, cal.pwmDuty);
                    }
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                }
            }
            break;
        }
        
        case PistonCalRuntime::OPEN_PISTON: {
            // Pistonu aç ve değer değişimini izle
            
            float raw;
            if (readPistonRawSample(cal.piston, raw)) {
                // Değer değişti mi kontrol et
                if (fabsf(raw - cal.lastRaw) > CAL_STABLE_THRESHOLD) {
                    cal.lastRaw = raw;
                    cal.lastChangeMs = now;
                }
            }
            
            // Stabilite kontrolü veya timeout
            bool stable = (now - cal.lastChangeMs) >= CAL_STABLE_TIME_MS;
            bool timeout = (now - cal.stageStartMs) >= CAL_STABLE_TIMEOUT_MS;
            
            if (stable || timeout) {
                cal.stage = PistonCalRuntime::READ_OPEN;
                cal.stageStartMs = now;
                {
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[CAL] reading open position...");
                }
            }
            break;
        }
        
        case PistonCalRuntime::READ_OPEN: {
            // Açık konumu oku
            float raw;
            if (readPistonRawSample(cal.piston, raw)) {
                cal.openRaw = raw;
                // 13mm = %50 strok → güç yasası tersiyle hesapla (n=0.39)
                // Lineer orta (raw/2) TMAG nonlineerliği nedeniyle 13mm'ye denk GELMEZ
                {
                    float zA = fabsf(cal.closedRaw);
                    float zB = fabsf(cal.openRaw);
                    if (zA > 5.0f && zB > 5.0f) {
                        const float n = 0.39f;
                        float pw_a   = powf(zA, -n);
                        float pw_b   = powf(zB, -n);
                        float pw_mid = 0.5f * (pw_a + pw_b);
                        float zMid   = powf(pw_mid, -1.0f / n);
                        cal.midRaw = (cal.openRaw > cal.closedRaw) ? zMid : -zMid;
                    } else {
                        cal.midRaw = (cal.closedRaw + cal.openRaw) * 0.5f;  // Fallback
                    }
                }
                
                // K1/K2 için ikinci sensörü de oku
                float raw2 = 0.0f;
                if (readPistonRawSample2(cal.piston, raw2)) {
                    cal.openRaw2 = raw2;
                }
                {
                    char msg[120];
                    if (cal.openRaw2 != 0.0f) {
                        snprintf(msg, sizeof(msg), "[CAL] open s1=%.0f s2=%.0f mid13mm=%.0f range=%.0f", cal.openRaw, cal.openRaw2, cal.midRaw, fabsf(cal.openRaw - cal.closedRaw));
                    } else {
                        snprintf(msg, sizeof(msg), "[CAL] open=%.0f mid13mm=%.0f range=%.0f", cal.openRaw, cal.midRaw, fabsf(cal.openRaw - cal.closedRaw));
                    }
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                }
                
                // Hold PWM bulma aktifse oraya git, değilse bitir
                bool isClutchPiston = (cal.piston == PISTON_K1 || cal.piston == PISTON_K2);
                float earlyRange = fabsf(cal.openRaw - cal.closedRaw);
                if (earlyRange < 2000.0f) {
                    // Piston açılmamış - gereksiz tarama adımlarını atla, direkt hata ver
                    {
                        char msg[96];
                        snprintf(msg, sizeof(msg), "[CAL] range=%.0f < 2000 - hold scan atlanıyor (piston açılmadı?)", earlyRange);
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                    }
                    cal.stage = PistonCalRuntime::COMPLETE;
                    cal.stageStartMs = now;
                } else if (cal.findHold && !isClutchPiston) {
                    // Vites pistonları: önce kapat, sonra PWM eşiklerini bul
                    cal.stage = PistonCalRuntime::CLOSE_FOR_SCAN;
                    cal.stageStartMs = now;
                    cal.pwmOpenThresh = 0;
                    cal.pwmCloseThresh = 0;
                    {
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[CAL] Closing piston for PWM scan...");
                    }
                } else if (cal.findHold && isClutchPiston) {
                    // Kavrama pistonları: hızlı kapalı döngü ile orantısal bölge ara
                    float strokeSafe = (cal.strokeMm > 0.0f) ? cal.strokeMm : 24.0f;
                    float holdTargetRaw = cal.closedRaw + (cal.openRaw - cal.closedRaw) * (10.0f / strokeSafe);
                    cal.midRaw = holdTargetRaw;
                    cal.holdPwmLo = 400;
                    cal.holdPwmHi = cal.pwmDuty;
                    cal.holdPwmTest = (uint16_t)(cal.pwmDuty * 0.85f);  // Eşik bölgesine yakın başla
                    cal.holdIteration = 0;
                    cal.holdAtMid   = false;
                    cal.holdLastRaw = raw;
                    cal.holdStableStartMs = now;
                    cal.lastPwmStepMs = now;
                    cal.stage = PistonCalRuntime::FIND_HOLD_MEASURE;
                    cal.stageStartMs = now;
                    {
                        char msg[128];
                        snprintf(msg, sizeof(msg),
                            "[CAL] K1/K2 fast loop: target=%.0f (10mm/%.1fmm) initPwm=%u",
                            holdTargetRaw, strokeSafe, cal.holdPwmTest);
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                    }
                } else {
                    cal.stage = PistonCalRuntime::COMPLETE;
                }
            }
            break;
        }
        
        case PistonCalRuntime::CLOSE_FOR_SCAN: {
            // PWM eşik taraması için pistonu kapat
            // Piston kapalı konuma yaklaşana kadar bekle
            float raw;
            if (readPistonRawSample(cal.piston, raw)) {
                float distToClosed = fabsf(raw - cal.closedRaw);
                float range = fabsf(cal.openRaw - cal.closedRaw);
                float threshold = range * 0.05f;  // %5 tolerans
                if (threshold < 100.0f) threshold = 100.0f;
                
                bool timeout = (now - cal.stageStartMs) >= 3000;  // 3sn max
                bool atClosed = (distToClosed < threshold);
                
                if (atClosed || timeout) {
                    // Piston kapandı veya timeout, açılma taramasına başla
                    cal.stage = PistonCalRuntime::FIND_PWM_OPEN_THRESH;
                    cal.stageStartMs = now;
                    cal.pwmScanCurrent = 900;  // 900'den başla
                    cal.threshStartRaw = raw;  // MEVCUT pozisyondan başla
                    cal.threshMovementDetected = false;
                    cal.lastPwmStepMs = now;
                    {
                        char msg[96];
                        snprintf(msg, sizeof(msg), "[CAL] PWM scan: closed at %.0f, starting open scan PWM=900", raw);
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                    }
                }
            }
            break;
        }
        
        case PistonCalRuntime::FIND_PWM_OPEN_THRESH: {
            // PWM'i 900'den 1500'e kademeli artırarak açılma eşiğini bul
            // Piston kapalıyken hangi PWM'de açılmaya başlıyor?
            float raw;
            if (readPistonRawSample(cal.piston, raw)) {
                float movement = raw - cal.threshStartRaw;  // Pozitif = açılma yönünde
                float moveThresh = fabsf(cal.openRaw - cal.closedRaw) * 0.03f;  // %3 hareket algılama
                if (moveThresh < 50.0f) moveThresh = 50.0f;
                
                bool timeout = (now - cal.stageStartMs) >= 10000;  // 10sn max
                
                // Hareket algılandı mı?
                if (!cal.threshMovementDetected && movement > moveThresh) {
                    cal.threshMovementDetected = true;
                    cal.pwmOpenThresh = cal.pwmScanCurrent;
                    {
                        char msg[96];
                        snprintf(msg, sizeof(msg), "[CAL] OPEN threshold found: PWM=%u (moved %.0f)", 
                                 cal.pwmOpenThresh, movement);
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                    }
                }
                
                // PWM'i artır veya sonraki aşamaya geç
                if (cal.threshMovementDetected || timeout || cal.pwmScanCurrent >= 1500) {
                    // Açılma eşiği bulundu veya tarama bitti, kapanma taramasına geç
                    if (!cal.threshMovementDetected) {
                        cal.pwmOpenThresh = 1400;  // Bulunamadı, varsayılan değer
                        {
                            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[CAL] OPEN threshold not found");
                        }
                    }
                    
                    // Pistonu açık konuma getir (kapanma taraması için)
                    // 1600'den başla - piston tam açılsın, sonra PWM düşürülecek
                    cal.stage = PistonCalRuntime::FIND_PWM_CLOSE_THRESH;
                    cal.stageStartMs = now;
                    cal.pwmScanCurrent = 1600;  // Tam açık PWM'den başla
                    cal.threshStartRaw = 0.0f;  // 0 = bekleme sonrasında okunacak
                    cal.threshMovementDetected = false;
                    cal.lastPwmStepMs = now + 2000;  // 2sn bekle (piston açılsın)
                    {
                        char msg[96];
                        snprintf(msg, sizeof(msg), "[CAL] OPEN thresh=%u, opening piston for close scan...", cal.pwmOpenThresh);
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                    }
                } else {
                    // PWM'i artır (her 150ms'de 15 PWM)
                    if ((now - cal.lastPwmStepMs) >= 150) {
                        cal.pwmScanCurrent += 15;
                        cal.lastPwmStepMs = now;
                    }
                }
            }
            break;
        }
        
        case PistonCalRuntime::FIND_PWM_CLOSE_THRESH: {
            // PWM'i 1600'den 500'e kademeli düşürerek kapanma eşiğini bul
            // Piston açıkken hangi PWM'de kapanmaya başlıyor?
            float raw;
            if (readPistonRawSample(cal.piston, raw)) {
                // İlk 2sn bekle, piston açılsın, sonra başlangıç pozisyonunu kaydet
                if (cal.threshStartRaw < 0.1f && (now >= cal.lastPwmStepMs)) {
                    cal.threshStartRaw = raw;  // Mevcut pozisyonu başlangıç olarak kaydet
                    {
                        char msg[96];
                        snprintf(msg, sizeof(msg), "[CAL] Close scan starting, pos=%.0f, PWM=%u", raw, cal.pwmScanCurrent);
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                    }
                }
                
                float movement = cal.threshStartRaw - raw;  // Pozitif = kapanma yönünde
                float moveThresh = fabsf(cal.openRaw - cal.closedRaw) * 0.03f;  // %3 hareket algılama
                if (moveThresh < 50.0f) moveThresh = 50.0f;
                
                bool timeout = (now - cal.stageStartMs) >= 15000;  // 15sn max (2sn bekleme dahil)
                
                // Hareket algılandı mı? (sadece bekleme bittikten sonra)
                if (!cal.threshMovementDetected && movement > moveThresh && cal.threshStartRaw > 0.1f) {
                    cal.threshMovementDetected = true;
                    cal.pwmCloseThresh = cal.pwmScanCurrent;
                    {
                        char msg[96];
                        snprintf(msg, sizeof(msg), "[CAL] CLOSE threshold found: PWM=%u (moved %.0f)", 
                                 cal.pwmCloseThresh, movement);
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                    }
                }
                
                // PWM'i düşür veya sonraki aşamaya geç
                if (cal.threshMovementDetected || timeout || cal.pwmScanCurrent <= 500) {
                    // Kapanma eşiği bulundu veya tarama bitti
                    if (!cal.threshMovementDetected) {
                        cal.pwmCloseThresh = 700;  // Bulunamadı, varsayılan değer
                        {
                            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[CAL] CLOSE threshold not found");
                        }
                    }
                    
                    // Hold PWM'i hesapla: açılma ve kapanma eşiklerinin ortası
                    cal.holdPwmTest = (cal.pwmOpenThresh + cal.pwmCloseThresh) / 2;
                    cal.holdPwmLo = cal.pwmCloseThresh;
                    cal.holdPwmHi = cal.pwmOpenThresh;
                    
                    {
                        char msg[120];
                        snprintf(msg, sizeof(msg), "[CAL] PWM thresholds: open=%u close=%u -> hold=%u", 
                                 cal.pwmOpenThresh, cal.pwmCloseThresh, cal.holdPwmTest);
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                    }
                    
                    // Hold PWM ince ayar için FIND_HOLD_PARK'a geç
                    cal.stage = PistonCalRuntime::FIND_HOLD_PARK;
                    cal.stageStartMs = now;
                    cal.holdIteration = 0;
                    cal.holdAtMid = false;
                    cal.holdStableStartMs = 0;
                    {
                        char msg[96];
                        snprintf(msg, sizeof(msg), "[CAL] hold search: mid=%.0f, parking...", cal.midRaw);
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                    }
                } else {
                    // PWM'i düşür (her 150ms'de 15 PWM)
                    if ((now - cal.lastPwmStepMs) >= 150) {
                        cal.pwmScanCurrent -= 15;
                        cal.lastPwmStepMs = now;
                    }
                }
            }
            break;
        }
        
        case PistonCalRuntime::FIND_HOLD_PARK: {
            // Pistonu ortaya konumlandır
            // Ortanın ilerisinde → düşük PWM (700)
            // Ortanın gerisinde → yüksek PWM (1350)
            float raw;
            if (readPistonRawSample(cal.piston, raw)) {
                float diff = raw - cal.midRaw;  // Pozitif = ortanın ilerisinde (açığa yakın)
                float tolerance = fabsf(cal.openRaw - cal.closedRaw) * 0.05f;  // %5 tolerans
                if (tolerance < 100.0f) tolerance = 100.0f;
                
                bool atMid = fabsf(diff) < tolerance;
                bool timeout = (now - cal.stageStartMs) >= 5000;
                
                if (atMid) {
                    // Ortaya ulaştı, hold test aşamasına geç
                    cal.holdAtMid = true;
                    cal.stage = PistonCalRuntime::FIND_HOLD_MEASURE;
                    cal.stageStartMs = now;
                    cal.holdLastRaw = raw;
                    cal.holdStableStartMs = now;
                    {
                        char msg[96];
                        snprintf(msg, sizeof(msg), "[CAL] at mid=%.0f, testing PWM=%u", raw, cal.holdPwmTest);
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                    }
                } else if (timeout) {
                    // Timeout - mevcut test PWM ile devam et
                    cal.stage = PistonCalRuntime::FIND_HOLD_MEASURE;
                    cal.stageStartMs = now;
                    cal.holdLastRaw = raw;
                    cal.holdStableStartMs = now;
                    {
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[CAL] park timeout");
                    }
                }
                // PWM ayarı valf kontrolünde yapılıyor
            }
            break;
        }
        
        case PistonCalRuntime::FIND_HOLD_MEASURE: {
            float raw;
            if (readPistonRawSample(cal.piston, raw)) {
                bool isClutch = (cal.piston == PISTON_K1 || cal.piston == PISTON_K2);
                float diff  = raw - cal.midRaw;               // Pozisyon hatası
                float drift = raw - cal.holdLastRaw;           // Hız (son okumadan fark)
                float tolerance = fabsf(cal.openRaw - cal.closedRaw) * 0.02f;
                if (tolerance < 30.0f) tolerance = 30.0f;
                
                bool stable = fabsf(drift) < tolerance;
                uint32_t stableTime = now - cal.holdStableStartMs;
                bool timeout = (now - cal.stageStartMs) >= (isClutch ? 30000u : 15000u);
                
                if (isClutch) {
                    // ---- Kavrama pistonu: hızlı kapalı döngü (30ms, ±5 PWM) ----
                    // Orantısal valf bölgesini bulmak için küçük adımlarla hızlı kontrol
                    const uint32_t STEP_MS = 30;
                    bool stepReady = (now - cal.lastPwmStepMs) >= STEP_MS;
                    
                    if (stepReady) {
                        cal.lastPwmStepMs = now;
                        float openDir = (cal.openRaw > cal.closedRaw) ? 1.0f : -1.0f;
                        float physDiff = (raw - cal.midRaw) * openDir;  // >0: çok açık, <0: çok kapalı
                        float posTol = fabsf(cal.openRaw - cal.closedRaw) * 0.08f;
                        if (posTol < 80.0f) posTol = 80.0f;
                        
                        if (physDiff > posTol) {
                            // Çok açık → PWM azalt
                            if (cal.holdPwmTest > cal.holdPwmLo + 5) cal.holdPwmTest -= 5;
                            cal.holdStableStartMs = now;
                        } else if (physDiff < -posTol) {
                            // Çok kapalı → PWM artır
                            if (cal.holdPwmTest < cal.holdPwmHi - 5) cal.holdPwmTest += 5;
                            cal.holdStableStartMs = now;
                        }
                        
                        cal.holdIteration++;
                        if ((cal.holdIteration % 10) == 0) {
                            char msg[96];
                            snprintf(msg, sizeof(msg), "[CAL] K1/K2 fast PWM=%u pos=%.0f target=%.0f diff=%.0f",
                                     cal.holdPwmTest, raw, cal.midRaw, physDiff);
                            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                        }
                    }
                    // Tolerans içinde ise stableTimer büyür → 3s sonra COMPLETE
                } else {
                    // ---- Vites pistonu (çift yönlü): drift tabanlı kontrol ----
                    if (!stable) {
                        cal.holdStableStartMs = now;
                        if (drift > tolerance) {
                            if (cal.holdPwmTest > 800) cal.holdPwmTest -= 8;
                        } else if (drift < -tolerance) {
                            if (cal.holdPwmTest < 1250) cal.holdPwmTest += 8;
                        }
                        cal.holdIteration++;
                        if ((cal.holdIteration % 10) == 0) {
                            char msg[96];
                            snprintf(msg, sizeof(msg), "[CAL] iter=%u PWM=%u drift=%.1f pos=%.0f",
                                     cal.holdIteration, cal.holdPwmTest, drift, raw);
                            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                        }
                    }
                }
                
                cal.holdLastRaw = raw;
                
                // Tamamlanma: K1/K2 → 3sn, vites → 5sn sabit + timeout
                uint32_t doneMs = isClutch ? 3000 : 5000;
                if (stableTime >= doneMs || timeout) {
                    cal.holdPwmResult = cal.holdPwmTest;
                    cal.measuredMidRaw = raw;
                    cal.stage = PistonCalRuntime::COMPLETE;
                    {
                        char msg[120];
                        snprintf(msg, sizeof(msg), "[CAL] hold PWM found: %u (stable %ums, iter=%u, pos=%.0f)",
                                 cal.holdPwmResult, stableTime, cal.holdIteration, cal.measuredMidRaw);
                        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                    }
                }
                
                // Yeniden park: sadece vites pistonları için
                if (!isClutch) {
                    static uint8_t s_reparkCount[6] = {0};
                    float midTolerance = fabsf(cal.openRaw - cal.closedRaw) * 0.4f;
                    if (fabsf(diff) > midTolerance && !timeout && s_reparkCount[cal.piston] < 3) {
                        s_reparkCount[cal.piston]++;
                        cal.stage = PistonCalRuntime::FIND_HOLD_PARK;
                        cal.stageStartMs = now;
                        {
                            char msg[64];
                            snprintf(msg, sizeof(msg), "[CAL] drift %.0f > tol %.0f, re-park %u/3",
                                     fabsf(diff), midTolerance, s_reparkCount[cal.piston]);
                            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                        }
                    } else if (fabsf(diff) > midTolerance && s_reparkCount[cal.piston] >= 3) {
                        cal.holdPwmResult = cal.holdPwmTest;
                        cal.measuredMidRaw = raw;
                        cal.stage = PistonCalRuntime::COMPLETE;
                        s_reparkCount[cal.piston] = 0;
                        {
                            char msg[120];
                            snprintf(msg, sizeof(msg), "[CAL] max re-park reached, accepting PWM=%u pos=%.0f",
                                     cal.holdPwmResult, cal.measuredMidRaw);
                            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                        }
                    }
                }
            }
            break;
        }
        
        case PistonCalRuntime::COMPLETE: {
            // Sonuçları yayınla
            pistonCalPublishSimple(cal);
            
            // Ardışık kalibrasyon: sonraki pistona geç
            bool wasCalibAll = cal.calibrateAll;
            bool wasFindHold = cal.findHold;
            float targetPressure = cal.pressureTargetBar;
            uint16_t pwmDuty = cal.pwmDuty;
            uint8_t nextIdx = cal.nextPiston;
            
            pistonCalReset(cal);
            
            if (wasCalibAll && nextIdx < PISTON_CHANNEL_COUNT) {
                // Sonraki pistonu başlat
                cal.piston = (PistonChannel)nextIdx;
                cal.valveIdx = PISTON_VALVE_INDEX[nextIdx];
                cal.supportIdx = PISTON_SUPPORT_VALVE_INDEX[nextIdx];
                cal.pwmDuty = pwmDuty;
                cal.strokeMm = PISTON_STROKE_MM[nextIdx];
                cal.pressureTargetBar = targetPressure;
                cal.findHold = wasFindHold;
                cal.calibrateAll = true;
                cal.nextPiston = nextIdx + 1;
                cal.stage = PistonCalRuntime::WAIT_PRESSURE;
                cal.stageStartMs = now;
                cal.active = true;
                
                {
                    char msg[96];
                    snprintf(msg, sizeof(msg), "[CAL] next piston %d valve=%d", nextIdx, cal.valveIdx);
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
                }
            } else {
                // Tüm kalibrasyon bitti
                if (wasCalibAll) {
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[CAL] ALL PISTONS DONE");
                }
            }
            
            // Tüm valfler kapalı olsun (basınç valfleri hariç - onlar açık kalabilir)
            for (int i = 0; i < 8; i++) {
                if (i != 1 && i != 5) target[i] = 0;
            }
            break;
        }
        
        default:
            break;
    }

    // Global kalibrasyon durumunu telemetry S mesajina yansit
    g_pistonCalRunning = cal.active ? 1 : 0;
    if (!cal.active) {
        g_pistonCalPhase = 0;
    } else {
        switch (cal.stage) {
            case PistonCalRuntime::WAIT_PRESSURE:
            case PistonCalRuntime::OPEN_VALVES:
                g_pistonCalPhase = 1; break;
            case PistonCalRuntime::READ_CLOSED:
            case PistonCalRuntime::OPEN_PISTON:
            case PistonCalRuntime::WAIT_STABLE:
            case PistonCalRuntime::READ_OPEN:
            case PistonCalRuntime::CLOSE_FOR_SCAN:
            case PistonCalRuntime::FIND_PWM_OPEN_THRESH:
            case PistonCalRuntime::FIND_PWM_CLOSE_THRESH:
                g_pistonCalPhase = (cal.piston < PISTON_CHANNEL_COUNT) ? PISTON_TO_GUI_PHASE[cal.piston] : 0;
                break;
            case PistonCalRuntime::FIND_HOLD_PARK:
            case PistonCalRuntime::FIND_HOLD_MEASURE:
                g_pistonCalPhase = 8; break;
            case PistonCalRuntime::COMPLETE:
                g_pistonCalPhase = 9; break;
            default:
                g_pistonCalPhase = 0; break;
        }
    }
}

// ===============================
// İsimden index bul
// ===============================
static int valveNameToIndex(const char *name){
    for (int i = 0; i < 8; i++) {
        if (strcasecmp(name, VALVE_NAME[i]) == 0) return i;
    }
    return -1;
}


// DRAIN sırası: önce PCV'ler (N436 idx=1, N440 idx=5), sonra çift çift diğerleri
static const int PAIRS[][2] = {
    {0,2},  // N433 + N435
    {4,7},  // N438 + N437
};
static const int NUM_PAIRS = 2;//sizeof(PAIRS)/sizeof(PAIRS[0]);

static void setValveDutyIdx(int idx, uint16_t duty) {
    PWM_WriteDuty(idx, duty);
    s_lastDuty[idx] = duty;
}

static void allValvesOff(){
    for (int i=0;i<8;i++){ setValveDutyIdx(i, 0); }
}

// Hızlı basınç düşürme: P ? targetBar olana veya timeout'a kadar
static bool fastDrain(uint16_t pcvDuty, uint16_t pairDuty,
                      float targetBar, uint32_t timeoutMs)
{
    uint32_t t0 = millis();

    // 0) Güvenli başlangıç: her şeyi kapat
    allValvesOff();
    vTaskDelay(pdMS_TO_TICKS(50));

    // 1) PCV'leri aç (N436 idx=1, N440 idx=5)
    setValveDutyIdx(1, pcvDuty);
    setValveDutyIdx(5, pcvDuty);
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[DRAIN] PCV open (N436,N440)");

    // 2) Basınç kontrol + gerektiğinde çift ekleme
    int pair_i = 0;
    bool state_valves=false;
    uint32_t lastStep = millis();

    for (;;) {
        // Basınç oku
        float p = readPressureBar();
        if (p > targetBar+20 && !state_valves) {
           pair_i=0;
           allValvesOff();
           setValveDutyIdx(1, pcvDuty);
           setValveDutyIdx(5, pcvDuty);
           setValveDutyIdx(0, pcvDuty);
           setValveDutyIdx(2, pcvDuty);
           setValveDutyIdx(4, pcvDuty);
           setValveDutyIdx(7, pcvDuty);
           state_valves=true;
        }else if(p > targetBar+20 && state_valves)
        {
            allValvesOff();
           setValveDutyIdx(1, pcvDuty);
           setValveDutyIdx(5, pcvDuty);
           state_valves=false;

        }
        if (p > targetBar+10 && p < targetBar+20 && !state_valves) {
           pair_i=0;
           allValvesOff();
           setValveDutyIdx(1, pcvDuty);
           setValveDutyIdx(5, pcvDuty);
           //setValveDutyIdx(0, pcvDuty);
           //setValveDutyIdx(2, pcvDuty);
           setValveDutyIdx(4, pcvDuty);
           //setValveDutyIdx(7, pcvDuty);
           state_valves=true;
        }else if(p > targetBar+10 && p < targetBar+20 && state_valves)
        {
            allValvesOff();
           setValveDutyIdx(1, pcvDuty);
           setValveDutyIdx(5, pcvDuty);
           state_valves=false;

        }
        static uint32_t prevCtlMs = millis();
        uint32_t nowMs = millis();
        float dt_s = (nowMs - prevCtlMs) * 0.001f;
        prevCtlMs = nowMs;

        
        if (p <= targetBar) {
            {
                char b[96]; snprintf(b, sizeof(b), "[DSCH] OK, P=%.1f bar", p);
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, b);
            }
            break;
        }

        // Zaman aşımı
        if (millis() - t0 > timeoutMs) {
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[DSCH] TIMEOUT");
            break;
        }


        vTaskDelay(pdMS_TO_TICKS(DRAIN_CHECK_MS));
    }
    

    // 3) Hepsini kapat
    allValvesOff();
    return (readPressureBar() <= targetBar);
}


// ===============================
// Asıl Task
// ===============================
void TaskValveControl(void *pvParameters){
    (void) pvParameters;

    // I2C'nin hazır olmasını bekle (TaskI2CMonitor başlatır)
    vTaskDelay(pdMS_TO_TICKS(200));

    // Gücü aç
    efuse_on();
    vTaskDelay(pdMS_TO_TICKS(10));
    int pg = efuse_pg();

    // TCA, DRV, PWM
    TCA_InitAll();
    DRV_EnableAll(true);
    DRV_PresetAll();
    PWM_InitAll();
    PistonRefPrefs_LoadAll();
    
    {
        char buf[96];
        snprintf(buf, sizeof(buf), "[VALVE] init ok. PG=%d CL-Control=piston_P+pcv_PI", pg);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
    }

    // Buton toggle durumları (8 valf için)
    bool valveToggled[8] = {false,false,false,false,false,false,false,false};

    // Önceki buton durumları (aktif-düşük varsayıyoruz -> beklenen idle = 1)
    uint8_t prev_in0_20 = 0xFF;
    uint8_t prev_in1_20 = 0xFF;
    uint8_t prev_in0_21 = 0xFF;
    uint8_t prev_in1_21 = 0xFF;
    uint32_t  lastDischargeSeq = 0;
    uint32_t  lastCalReqSeq = 0;
    uint32_t  lastHoldSeq[PISTON_CHANNEL_COUNT] = {0};
    uint32_t lastRefSeq       = 0; 

    // Ana döngü
    for (;;) {
        // -------------------------------------------------
        // 1) JSON'dan gelen otomatik basınç düşürme isteği
        // -------------------------------------------------
        if (g_valveDischargeSeq != lastDischargeSeq) {
            lastDischargeSeq = g_valveDischargeSeq;
            ValveDischargeCommand cmd{};
            if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                cmd = g_valveDischargeCmd;
                xSemaphoreGive(g_sharedMutex);
            }
            uint16_t pcvDuty  = cmd.pcvDuty  ? cmd.pcvDuty  : DRAIN_PCv_DUTY;
            uint16_t pairDuty = cmd.pairDuty ? cmd.pairDuty : DRAIN_PAIR_DUTY;
            float    target   = (cmd.targetBar > 0.0f) ? cmd.targetBar : DRAIN_TARGET_BAR;
            uint32_t tmoMs    = cmd.timeoutMs ? cmd.timeoutMs : DRAIN_TIMEOUT_MS;
            {
                char b[120];
                snprintf(b, sizeof(b), "[DISCH] start pcv=%u pair=%u target=%.1fbar timeout=%ums",
                         pcvDuty, pairDuty, target, (unsigned)tmoMs);
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, b);
            }
            bool ok = fastDrain(pcvDuty, pairDuty, target, tmoMs);
            {
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, ok ? (char*)"[DISCH] DONE" : (char*)"[DISCH] FAIL");
            }
        }

        if (g_pistonCalReqSeq != lastCalReqSeq) {
            lastCalReqSeq = g_pistonCalReqSeq;
            PistonCalibrationRequest req{};
            if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                req = g_pistonCalReq;
                xSemaphoreGive(g_sharedMutex);
            }
            if (req.start) {
                pistonCalStart(req);
            } else if (s_pistonCal.stage != PistonCalRuntime::IDLE) {
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[CAL] aborted by request");
                int idx = s_pistonCal.valveIdx;
                pistonCalReset(s_pistonCal);
                if (idx >= 0) PWM_WriteDuty(idx, 0);
            }
        }

        // Her piston için ayrı hold request kontrolü
        for (int hp = 0; hp < PISTON_CHANNEL_COUNT; hp++) {
            if (g_pistonHoldReqSeq[hp] != lastHoldSeq[hp]) {
                lastHoldSeq[hp] = g_pistonHoldReqSeq[hp];
                PistonHoldRequest req{};
                if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    req = g_pistonHoldReq[hp];
                    xSemaphoreGive(g_sharedMutex);
                }
                handlePistonHoldRequest(req);
            }
        }

        if (g_pistonRefReqSeq != lastRefSeq) {
            lastRefSeq = g_pistonRefReqSeq;
            PistonReferenceRequest req{};
            if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                req = g_pistonRefReq;
                xSemaphoreGive(g_sharedMutex);
            }
            handlePistonReferenceRequest(req);
        }
        // -------------------------------------------------
        // 2) TCA giri?lerini tara (butonlar + faultlar)
        // -------------------------------------------------
        uint8_t in0_20 = tcaRead8(TCA0, TCA_REG_INPUT0);  // P0
        uint8_t in1_20 = tcaRead8(TCA0, TCA_REG_INPUT1);  // P1
        uint8_t in0_21 = tcaRead8(TCA1, TCA_REG_INPUT0);
        uint8_t in1_21 = tcaRead8(TCA1, TCA_REG_INPUT1);

        // --- BTN'ler aktif-düşük varsayımıyla edge tespiti ---
        // 0x20: P06, P07, P10, P11
        // P06 -> N434 (idx=2)
        if ( ( (prev_in0_20 & (1<<6)) != 0 ) && ( (in0_20 & (1<<6)) == 0 ) ) {
            // buton basıldı
            int idx = 2; // N434
            if (!valveToggled[idx]) {
                PWM_WriteDuty(idx, BTN_TOGGLE_DUTY);
                valveToggled[idx] = true;
            } else {
                PWM_WriteDuty(idx, 0);
                valveToggled[idx] = false;
            }
        }
        // P07 -> N433 (idx=0)
        if ( ( (prev_in0_20 & (1<<7)) != 0 ) && ( (in0_20 & (1<<7)) == 0 ) ) {
            int idx = 0; // N433
            if (!valveToggled[idx]) {
                PWM_WriteDuty(idx, BTN_TOGGLE_DUTY);
                valveToggled[idx] = true;
            } else {
                PWM_WriteDuty(idx, 0);
                valveToggled[idx] = false;
            }
        }
        // P10 -> N436 (idx=1)
        if ( ( (prev_in1_20 & (1<<0)) != 0 ) && ( (in1_20 & (1<<0)) == 0 ) ) {
            int idx = 1; // N436
            if (!valveToggled[idx]) {
                PWM_WriteDuty(idx, BTN_TOGGLE_DUTY);
                valveToggled[idx] = true;
            } else {
                PWM_WriteDuty(idx, 0);
                valveToggled[idx] = false;
            }
        }
        // P11 -> N435 (idx=3)
        if ( ( (prev_in1_20 & (1<<1)) != 0 ) && ( (in1_20 & (1<<1)) == 0 ) ) {
            int idx = 3; // N435
            if (!valveToggled[idx]) {
                PWM_WriteDuty(idx, BTN_TOGGLE_DUTY);
                valveToggled[idx] = true;
            } else {
                PWM_WriteDuty(idx, 0);
                valveToggled[idx] = false;
            }
        }

        // 0x21: P06, P07, P10, P11
        // P06 -> N438 (idx=4)
        if ( ( (prev_in0_21 & (1<<6)) != 0 ) && ( (in0_21 & (1<<6)) == 0 ) ) {
            int idx = 4;
            if (!valveToggled[idx]) {
                PWM_WriteDuty(idx, BTN_TOGGLE_DUTY);
                valveToggled[idx] = true;
            } else {
                PWM_WriteDuty(idx, 0);
                valveToggled[idx] = false;
            }
        }
        // P07 -> N437 (idx=7)
        if ( ( (prev_in0_21 & (1<<7)) != 0 ) && ( (in0_21 & (1<<7)) == 0 ) ) {
            int idx = 7;
            if (!valveToggled[idx]) {
                PWM_WriteDuty(idx, BTN_TOGGLE_DUTY);
                valveToggled[idx] = true;
            } else {
                PWM_WriteDuty(idx, 0);
                valveToggled[idx] = false;
            }
        }
        // P10 -> N439 (idx=6)
        if ( ( (prev_in1_21 & (1<<0)) != 0 ) && ( (in1_21 & (1<<0)) == 0 ) ) {
            int idx = 6;
            if (!valveToggled[idx]) {
                PWM_WriteDuty(idx, BTN_TOGGLE_DUTY);
                valveToggled[idx] = true;
            } else {
                PWM_WriteDuty(idx, 0);
                valveToggled[idx] = false;
            }
        }
        // P11 -> N440 (idx=5)
        if ( ( (prev_in1_21 & (1<<1)) != 0 ) && ( (in1_21 & (1<<1)) == 0 ) ) {
            int idx = 5;
            if (!valveToggled[idx]) {
                PWM_WriteDuty(idx, BTN_TOGGLE_DUTY);
                valveToggled[idx] = true;
            } else {
                PWM_WriteDuty(idx, 0);
                valveToggled[idx] = false;
            }
        }

        // --- DRV FAULT takibi: OCP (kısa devre) → acil kapatma; OLA → sadece log ---
        // STATUS1 (st2): Bit7=OLA1, Bit6=OLA2, Bit4=ACTIVE, Bit3-0=OCP_H1/L1/H2/L2
        // OCP=0x0F maskesi | OLA=0xC0 maskesi
        // g_drvOcpLatch: Shared.h/Shared.cpp'de tanımlı — serial komutla sıfırlanabilir

        // nFAULT pin mappings (active low):
        // 0x20 P04 -> DRV1,  0x20 P05 -> DRV2
        // 0x21 P04 -> DRV3,  0x21 P05 -> DRV4
        static const uint8_t nFAULT_MASK[4] = {(1<<4),(1<<5),(1<<4),(1<<5)};

        // Her DRV için nFAULT giriş verisi
        uint8_t nfaultIn[4] = {in0_20, in0_20, in0_21, in0_21};

        // OCP log throttle (spam önleme — OCP kritik, her seferinde logla)
        static uint32_t s_ocpLogMs[4]  = {0, 0, 0, 0};
        static const uint32_t OCP_LOG_INTERVAL_MS = 0;

        uint32_t nowFault = millis();

        for (int di = 0; di < 4; di++) {
            if ((nfaultIn[di] & nFAULT_MASK[di]) != 0) continue;  // nFAULT HIGH = normal

            uint8_t st1 = DRV_ReadReg(di, 0x01);
            uint8_t st2 = DRV_ReadReg(di, 0x02);
            uint8_t flt = DRV_ReadReg(di, 0x03);

            bool isOcp = (st2 & 0x0F) != 0;  // OCP: H-bridge overcurrent (bit3-0)
            bool isTsd = (flt & 0x04) != 0;  // TSD: thermal shutdown

            if (isOcp || isTsd) {
                // --- ACİL KAPATMA ---
                for (int vi = 0; vi < 8; vi++) PWM_WriteDuty(vi, 0);
                DRV_EnableAll(false);
                g_drvOcpLatch = true;
                g_drvLastFault[di].st1 = st1;
                g_drvLastFault[di].st2 = st2;
                g_drvLastFault[di].flt = flt;
                g_drvLastFault[di].ok  = false;

                if ((nowFault - s_ocpLogMs[di] >= OCP_LOG_INTERVAL_MS)) {
                    s_ocpLogMs[di] = nowFault;
                    char b[112];
                    const char* reason = isTsd ? "TSD" : "OCP";
                    snprintf(b, sizeof(b),
                             "[DRV] !!! %s HATA DRV%d: ST1=0x%02X ST2=0x%02X FLT=0x%02X - TUM VALFLER KAPATILDI !!!",
                             reason, di + 1, st1, st2, flt);
                    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, b);
                }
                // Fault clear YAPMA — kilit mod, kullanıcı müdahalesi gerekir

            } else {
                // OLA veya diğer minor fault → sessizce temizle, log YOK
                // (Boşta DRV'lerde OLA kaçınılmaz — spam yapmasın)
                DRV_FaultClear(di);
                g_drvLastFault[di].ok = true;
            }
        }

        // OCP kilit modunda tüm görev süresince PWM=0 ve DRVOFF LOW tut
        if (g_drvOcpLatch) {
            for (int vi = 0; vi < 8; vi++) PWM_WriteDuty(vi, 0);
            // DRVOFF zaten LOW — burada tekrar çekmeye gerek yok ama güvenlik için
            DRV_EnableAll(false);
        }

        // önceki girişleri güncelle
        prev_in0_20 = in0_20;
        prev_in1_20 = in1_20;
        prev_in0_21 = in0_21;
        prev_in1_21 = in1_21;

        static uint32_t prevCtlMs = millis();
        uint32_t nowMs = millis();
        float dt_s = (nowMs - prevCtlMs) * 0.001f;
        if (dt_s < 0.001f) dt_s = 0.001f;
        if (dt_s > 0.1f)   dt_s = 0.1f;
        prevCtlMs = nowMs;

        // ---- Akım tabanlı kapalı çevrim kontrol ----
        bool holdSuppressed = (s_pistonCal.stage != PistonCalRuntime::IDLE) ||
                               g_currentCalibRunning;

        // 1. Pozisyon/PCV kontrolcüleri → g_valveCustomCurrent_mA hedeflerini yazar
        holdcontrol_V2(nullptr, holdSuppressed);
        if (!holdSuppressed) pcv_pi_step(nullptr, dt_s);

        // 2. Manuel close/close_slow: piston kapandiginda akimi kes
        autoStopCloseCurrent();

        // 3. INA PI regülatörü: g_valveCustomCurrent_mA → PWM (V=IR YOK)
        uint16_t target[8] = {};
        valve_current_reg_step(target, dt_s);

        // 4. Geriye dönük uyumluluk: g_valveCustomCurrent_mA=0 iken g_valveTargetDuty>0
        //    → doğrudan duty override (TaskAutoShiftV2 vb. eski kodlar)
        {
            if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                for (int i = 0; i < 8; i++) {
                    if (target[i] == 0 && g_valveTargetDuty[i] > 0)
                        target[i] = g_valveTargetDuty[i];
                }
                xSemaphoreGive(g_sharedMutex);
            }
        }

        // 5. Eski PistonCal override
        pistonCalProcess(s_pistonCal, target);
        if (s_pistonCal.stage != PistonCalRuntime::IDLE && s_pistonCal.valveIdx >= 0) {
            for (int i = 0; i < 8; i++) {
                if (i != s_pistonCal.valveIdx) target[i] = 0;
            }
            int supportIdx = -1, pcvIdx = -1;
            if (s_pistonCal.piston < PISTON_CHANNEL_COUNT) {
                supportIdx = PISTON_SUPPORT_VALVE_INDEX[s_pistonCal.piston];
                pcvIdx = (supportIdx == 1) ? 1 : (supportIdx == 5 ? 5 : -1);
            }
            if (supportIdx >= 0 && supportIdx < 8) target[supportIdx] = SUPPORT_VALVE_DUTY;
            if (pcvIdx >= 0 && pcvIdx < 8)         target[pcvIdx]     = SUPPORT_VALVE_DUTY;
        }
        
        // Değişen kanallara PWM yaz
        for (int i = 0; i < 8; i++) {
            if (target[i] != s_lastDuty[i]) {
                PWM_WriteDuty(i, target[i]);
                s_lastDuty[i]  = target[i];
                valveToggled[i] = (target[i] > 0); // buton toggles ile senkron kalsın
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}



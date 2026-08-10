#include <Arduino.h>
#include <math.h>
#include <algorithm>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include "Tasks.h"
#include "Shared.h"

// ---- Eski kart geriye dönük uyumluluk: ADS1115 @ 0x48 ----
// Yeni kartlarda ADS yok, IO2/IO10 direkt kullanılıyor.
// Başlangıçta otomatik tespit yapılır.
static Adafruit_ADS1115 s_ads;
static bool s_hasADS1115 = false;

// Hall/pozisyon filtreleri
static constexpr float HALL_ALPHA_FAST = 0.15f;  // kontrol icin daha yumusak hizli tepki
static constexpr float HALL_ALPHA_SLOW = 0.08f;  // gosterge/telemetri
static constexpr float HALL_DELTA_CLAMP = 0.05f; // ardil ornekler arasinda max adim (V)
static constexpr float VEL_ALPHA       = 0.10f;  // son hiz EMA (pozisyon filtresi sonrasi hafif)
static constexpr float POS_ALPHA       = 0.35f;  // pozisyon on-filtreleme (turev gürültüsünü azaltir)
static constexpr float VEL_MAX_ABS     = 2000.0f; // mm/s spike engeli (DQ200 max ~1300mm/s)
static constexpr float DT_MIN_S        = 0.001f;
static constexpr float DT_MAX_S        = 0.200f;

// NTC sıcaklık filtresi
static constexpr float TEMP_ALPHA      = 0.08f;   // EMA katsayısı (50Hz'de ~250ms zaman sabiti)
static constexpr int   TEMP_OVERSAMPLE = 4;        // MCU ADC aşırı örnekleme (2x SNR kazanımı)

// 26 mm strok icin kabaca voltaj araligi (sahada ayarlanir)
static constexpr float HALL26_V_MIN = 1.0f;   // sensor en uzaktayken
static constexpr float HALL26_V_MAX = 2.3f;   // sensor en yakindayken
static constexpr float HALL26_STROKE_MM = 26.0f;

// 30 mm strok icin (N435 / N439)
static constexpr float HALL30_STROKE_MM = 20.0f;
static constexpr float MULTI_HALL_SPACING_MM = 20.0f;   // sensorler arasindaki mesafe

// NTC hesabi icin sabitler
static constexpr float NTC_VSUPPLY   = 3.3f;
static constexpr float NTC_R_BOTTOM  = 10000.0f;  // ADS tarafindaki 10k
static constexpr float NTC_R0        = 10000.0f;  // 25C'de 10k
static constexpr float NTC_BETA      = 3950.0f;
static constexpr float NTC_T0        = 298.15f;   // 25C = 298.15K

static uint32_t t_prev_ms = 0;
static float s_v_t1_filt = -1.0f;  // NTC voltaj EMA filtresi (< 0 = başlatılmamış)
static float s_v_t2_filt = -1.0f;


float pos_13=0, pos_57=0, pos_24=0, pos_6R=0, pos_K1=0, pos_K2=0;  // onceki filtreli pozisyon (turev icin)
float posf_13=0, posf_57=0, posf_24=0, posf_6R=0, posf_K1=0, posf_K2=0;  // EMA filtreli mevcut pozisyon
float v_13=0,  v_57=0,  v_24=0,  v_6R=0,  v_K1=0,  v_K2=0;

static PistonCalibrationTable s_pistonCalCache[PISTON_CHANNEL_COUNT]{};
static uint32_t s_pistonCalSeqCache = 0;
struct ManualRefCache {
    float raw[3];
    uint8_t mask;
};
static ManualRefCache s_manualRefCache[PISTON_CHANNEL_COUNT]{};
static const int PISTON_VALVE_INDEX[PISTON_CHANNEL_COUNT] = {
    2,  // 5_7
    0,  // 1_3
    7,  // 2_4
    4   // 6_R
};
static const int PISTON_SUPPORT_VALVE_INDEX[PISTON_CHANNEL_COUNT] = {
    1, 1, 5, 5
};

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

static inline float lpf(float prev, float input, float alpha) {
    return prev + alpha * (input - prev);
}

static inline float clamp_step(float prev, float current, float maxStep) {
    float delta = current - prev;
    if (delta > maxStep) delta = maxStep;
    else if (delta < -maxStep) delta = -maxStep;
    return prev + delta;
}

static inline float median3(float a, float b, float c) {
    if (a > b) std::swap(a, b);
    if (b > c) std::swap(b, c);
    if (a > b) std::swap(a, b);
    return b;
}


// 10k NTC sicaklik hesaplama
static float ntcVoltageToCelsius(float v_ntc) {
    if (v_ntc <= 0.01f || v_ntc >= (NTC_VSUPPLY - 0.01f)) {
        return -273.0f; // sapma
    }
    float r_ntc = (NTC_VSUPPLY * NTC_R_BOTTOM / v_ntc) - NTC_R_BOTTOM;
    float inv_T = (1.0f / NTC_T0) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R0);
    float T = 1.0f / inv_T;       // Kelvin
    return T - 273.15f;           // Celsius
}

// ============ TMAG5173 TABANLI POZİSYON HESAPLAMA ============

// TMAG kanal -> piston mapping
// TMAG_CH_1_3 (0) -> PISTON_1_3 (1)
// TMAG_CH_5_7 (1) -> PISTON_5_7 (0)
// TMAG_CH_2_4 (2) -> PISTON_2_4 (2)
// TMAG_CH_6_R (3) -> PISTON_6_R (3)
static const uint8_t TMAG_TO_PISTON[4] = {PISTON_1_3, PISTON_5_7, PISTON_2_4, PISTON_6_R};
static const uint8_t PISTON_TO_TMAG[4] = {TMAG_CH_5_7, TMAG_CH_1_3, TMAG_CH_2_4, TMAG_CH_6_R};

// TMAG Z değerini mm'e çevir (kalibrasyon tablosuna göre)
// Parçalı linear interpolasyon: 0->mid=13mm, mid->max=13mm (toplam 26mm)
static float tmagZToMm(PistonChannel ch, int16_t z) {
    const TMAGPistonCalib& cal = g_tmagPistonCalib[ch];
    
    if (!cal.valid) {
        // Kalibrasyon yoksa varsayılan aralık kullan
        // TMAG5173 tipik aralık: -2000 ile +2000 arası
        float norm = (float)(z + 2000) / 4000.0f;
        return clampf(norm * PISTON_DEFAULT_STROKE_MM, 0.0f, PISTON_DEFAULT_STROKE_MM);
    }
    
    float zMinF = (float)cal.zMin;
    float zMaxF = (float)cal.zMax;
    float zMidF = (float)cal.zMid;
    float zF = (float)z;
    
    // zMid geçerli değilse küpkök modeli kullan (Bz ∝ 1/r³ yaklaşımı)
    // A = strokeMm / (1/|zMax|^(1/3) - 1/|zMin|^(1/3))
    // pos = A / |z|^(1/3) + B,   B = -A / |zMin|^(1/3)
    if (cal.zMid == 0 || fabsf(zMidF - zMinF) < 10.0f || fabsf(zMaxF - zMidF) < 10.0f) {
        float zA_min = fabsf(zMinF);
        float zA_max = fabsf(zMaxF);
        float zA    = fabsf(zF);
        if (zA < 1.0f) zA = 1.0f;
        // Güç yasası modeli: Bz ∝ 1/r^(1/n_phys), ölçümle belirlenen n=0.39
        // DQ200 piston verileri (z@0,6.5,13,19.5,26mm = 1550,2550,4650,10100,32725):
        //   n=1/3 (küpkök): maks hata 0.62mm | n=0.39: maks hata 0.16mm
        // Formül her iki geometride çalışır (closed>open veya closed<open):
        //   pos = strokeMm * (zA_min^-n - zA^-n) / (zA_min^-n - zA_max^-n)
        if (zA_min > 5.0f && zA_max > 5.0f) {
            const float HAL_PWR = 0.39f;
            float pow_min = powf(zA_min, -HAL_PWR);
            float pow_max = powf(zA_max, -HAL_PWR);
            float pow_z   = powf(zA,     -HAL_PWR);
            float denom = pow_min - pow_max;
            if (fabsf(denom) > 1e-10f) {
                float norm = (pow_min - pow_z) / denom;
                return clampf(norm * cal.strokeMm, 0.0f, cal.strokeMm);
            }
        }
        // Fallback: basit lineer interpolasyon
        float span = zMaxF - zMinF;
        if (fabsf(span) < 10.0f) return 0.0f;
        float norm = (zF - zMinF) / span;
        return clampf(norm * cal.strokeMm, 0.0f, cal.strokeMm);
    }
    
    // Parçalı linear interpolasyon (non-lineerlik düzeltmesi)
    // zMin -> zMid = 0mm -> 13mm
    // zMid -> zMax = 13mm -> 26mm
    float halfStroke = cal.strokeMm * 0.5f;  // 13mm
    
    if (zMinF < zMaxF) {
        // Normal yön: zMin < zMid < zMax
        if (zF <= zMidF) {
            // 0mm - 13mm arası
            float span1 = zMidF - zMinF;
            float norm = (zF - zMinF) / span1;
            norm = clampf(norm, 0.0f, 1.0f);
            return norm * halfStroke;
        } else {
            // 13mm - 26mm arası
            float span2 = zMaxF - zMidF;
            float norm = (zF - zMidF) / span2;
            norm = clampf(norm, 0.0f, 1.0f);
            return halfStroke + norm * halfStroke;
        }
    } else {
        // Ters yön: zMin > zMid > zMax
        if (zF >= zMidF) {
            // 0mm - 13mm arası
            float span1 = zMinF - zMidF;
            float norm = (zMinF - zF) / span1;
            norm = clampf(norm, 0.0f, 1.0f);
            return norm * halfStroke;
        } else {
            // 13mm - 26mm arası
            float span2 = zMidF - zMaxF;
            float norm = (zMidF - zF) / span2;
            norm = clampf(norm, 0.0f, 1.0f);
            return halfStroke + norm * halfStroke;
        }
    }
}

// Kavrama pozisyonu TMAG'den hesapla - işaretli lineer interpolasyon
// K1/K2 için TMAG_CH_K1_1 / TMAG_CH_K2_1 (açık pozisyon sensörü) kullanılır.
// Daha önce TMAG_CH_K1_2 / K2_2 (kapalı sensör) kullanılıyordu; saha verisinde
// açık sensör K1/K2 hareketini daha doğru takip ediyor.
// z1 hareket boyunca işaret değiştirdiğinden (+ → 0 → -) |z1| tabanlı
// ratiometrik formül orta bölgede donmaya yol açar. Bunun yerine
// kalibrasyon değerleri (zMin2=kapalı, zMax2=açık) arasında doğrusal norm.
static float tmagKavramaMm(int idx) {
    // Açık pozisyon sensörü (ikincil sensör kanalı)
    uint8_t ch1 = (idx == 0) ? TMAG_CH_K1_1 : TMAG_CH_K2_1;
    PistonChannel piston = (idx == 0) ? PISTON_K1 : PISTON_K2;

    if (!g_tmagData[ch1].valid) return 0.0f;

    const TMAGPistonCalib& cal = g_tmagPistonCalib[piston];

    // İkinci sensör kalibrasyonu varsa onu kullan; yoksa birincil sensör kalibrasyonuna
    // geri dön (sistemde yalnızca bir sensör varsa).
    bool useSensor2 = cal.valid && cal.hasSensor2 &&
                      (fabsf((float)cal.zMax2 - (float)cal.zMin2) >= 10.0f);

    if (!cal.valid) {
        // Kalibrasyon yoksa varsayılan aralık kullan (tek sensör)
        float norm = (float)(g_tmagData[ch1].z + 2000) / 4000.0f;
        return clampf(norm * 24.0f, 0.0f, 24.0f);
    }

    float span, zMin;
    if (useSensor2) {
        // Açık sensör kalibrasyonu (zMin2=kapalı, zMax2=açık)
        span  = (float)cal.zMax2 - (float)cal.zMin2;
        zMin  = (float)cal.zMin2;
    } else {
        // Birincil sensör kalibrasyonu (zMin=kapalı, zMax=açık)
        span  = (float)cal.zMax - (float)cal.zMin;
        zMin  = (float)cal.zMin;
    }
    if (fabsf(span) < 10.0f) return 0.0f;
    float norm = ((float)g_tmagData[ch1].z - zMin) / span;
    return clampf(norm, 0.0f, 1.0f) * cal.strokeMm;
}

// TMAG okumalarını cache'le (mutex dışında kullanmak için)
static TMAG5173_Reading s_tmagCache[TMAG_CH_COUNT] = {};
static TMAGPistonCalib s_tmagCalCache[PISTON_CHANNEL_COUNT] = {};
static TMAGKavramaCalib s_tmagKavCalCache[2] = {};
static uint32_t s_tmagCalSeqCache = 0;

// Yardimci: tek hall -> 0..stroke mm (kalibrasyon yoksa basit fallback)
static float hallVoltageToStroke26mm(float v_hall) {
    float mm = (v_hall - HALL26_V_MIN) * (HALL26_STROKE_MM / (HALL26_V_MAX - HALL26_V_MIN));
    return clampf(mm, 0.0f, HALL26_STROKE_MM);
}

// Yardimci: 3 hall'li (N435, N439) piston pozisyon tahmini
// DRV5055'de yan gecen magnet parabolik/duz olmayan alan olusturuyor, bu yuzden normalize edilmis agirlik merkezi kullaniyoruz.
static float multiHallToStroke30mm(const float hallV[3]) {
    float v1 = fmaxf(hallV[0], 0.0f);
    float v2 = fmaxf(hallV[1], 0.0f);
    float v3 = fmaxf(hallV[2], 0.0f);
    float maxv = fmaxf(v1, fmaxf(v2, v3));
    if (maxv < 0.02f) return 0.0f;
    float w1 = v1 / maxv;
    float w2 = v2 / maxv;
    float w3 = v3 / maxv;
    float sum = w1 + w2 + w3;
    if (sum < 1e-3f) return 0.0f;
    float pos = (0.0f * w1 + MULTI_HALL_SPACING_MM * w2 + (2.0f * MULTI_HALL_SPACING_MM) * w3) / sum;
    return clampf(pos, 0.0f, HALL30_STROKE_MM);
}

// Kalibre edilmis kavrama mm hesabi - sadece Hall2 ve Hall3 kullanir (Hall1 degismiyor)
static float kavramaCalibratedMm(int idx, const float hallV[3]) {
    const KavramaCalibData& cal = g_kavramaCalib[idx];
    
    // Hall2 ve Hall3 icin normalize et (Hall1 atla - degismiyor)
    float v2 = hallV[1];
    float v3 = hallV[2];
    
    if (cal.valid) {
        // Kalibre edilmis hesaplama
        float range2 = cal.openV[1] - cal.closedV[1];
        float range3 = cal.openV[2] - cal.closedV[2];
        
        float norm2 = 0.0f, norm3 = 0.0f;
        if (fabsf(range2) > 0.05f) {
            norm2 = (v2 - cal.closedV[1]) / range2;
            norm2 = clampf(norm2, 0.0f, 1.0f);
        }
        if (fabsf(range3) > 0.05f) {
            norm3 = (v3 - cal.closedV[2]) / range3;
            norm3 = clampf(norm3, 0.0f, 1.0f);
        }
        
        // Hall2 = 0mm pozisyonunda, Hall3 = 20mm pozisyonunda
        // Basit lineer interpolasyon: norm3 arttikca 20mm'e yaklasir
        float pos = norm3 * 20.0f;
        return clampf(pos, 0.0f, cal.strokeMm);
    }
    
    // Kalibrasyon yoksa basit formul: Hall3 yuksekse acik, Hall2 yuksekse kapali
    float maxv = fmaxf(v2, v3);
    if (maxv < 0.1f) return 0.0f;
    
    float ratio = v3 / (v2 + v3 + 0.01f);  // 0=kapali, 1=acik
    return ratio * 20.0f;
}

// Kavrama kalibrasyon state machine
static uint32_t s_kcalStartMs[2] = {0, 0};
static uint32_t s_kcalLastSeq = 0;

static void updateKavramaCalib(int idx, const float hallV[3]) {
    KavramaCalibState& state = (KavramaCalibState&)g_kavramaCalibState[idx];
    
    switch (state) {
        case KCAL_IDLE:
            break;
            
        case KCAL_WAIT_CLOSED:
            // 500ms bekle (filtre stabilize olsun)
            if (millis() - s_kcalStartMs[idx] > 500) {
                state = KCAL_SAMPLE_CLOSED;
            }
            break;
            
        case KCAL_SAMPLE_CLOSED:
            // Kapali pozisyon degerlerini kaydet
            for (int i = 0; i < 3; i++) {
                g_kavramaCalib[idx].closedV[i] = hallV[i];
            }
            // Valfi ac (N435 veya N439 + basinc valfi)
            if (idx == 0) {
                // K1: N435 + N436
                g_valveTargetDuty[3] = 2000;  // N435
                g_valveTargetDuty[1] = 2000;  // N436
            } else {
                // K2: N439 + N440
                g_valveTargetDuty[6] = 2000;  // N439
                g_valveTargetDuty[5] = 2000;  // N440
            }
            s_kcalStartMs[idx] = millis();
            state = KCAL_OPENING;
            break;
            
        case KCAL_OPENING:
            // 2 saniye bekle (piston acilsin)
            if (millis() - s_kcalStartMs[idx] > 2000) {
                state = KCAL_WAIT_OPEN;
                s_kcalStartMs[idx] = millis();
            }
            break;
            
        case KCAL_WAIT_OPEN:
            // 500ms daha bekle (stabilize)
            if (millis() - s_kcalStartMs[idx] > 500) {
                state = KCAL_SAMPLE_OPEN;
            }
            break;
            
        case KCAL_SAMPLE_OPEN:
            // Acik pozisyon degerlerini kaydet
            for (int i = 0; i < 3; i++) {
                g_kavramaCalib[idx].openV[i] = hallV[i];
            }
            g_kavramaCalib[idx].strokeMm = 20.0f;
            g_kavramaCalib[idx].valid = true;
            
            // Valfleri kapat
            if (idx == 0) {
                g_valveTargetDuty[3] = 0;  // N435
                g_valveTargetDuty[1] = 0;  // N436
            } else {
                g_valveTargetDuty[6] = 0;  // N439
                g_valveTargetDuty[5] = 0;  // N440
            }
            
            state = KCAL_DONE;
            break;
            
        case KCAL_DONE:
        case KCAL_FAILED:
            // Bitti, idle'a don
            break;
    }
}

static float tableToMm(PistonChannel ch, float rawV) {
    const auto &table = s_pistonCalCache[ch];
    if (!table.valid || table.numPoints < 2) {
        return hallVoltageToStroke26mm(rawV);
    }
    if (rawV <= table.raw[0]) return table.mm[0];
    if (rawV >= table.raw[table.numPoints - 1]) return table.mm[table.numPoints - 1];
    for (uint16_t i = 1; i < table.numPoints; i++) {
        float r0 = table.raw[i - 1];
        float r1 = table.raw[i];
        if (rawV <= r1) {
            float ratio = (rawV - r0) / (r1 - r0 + 1e-6f);
            float mm0 = table.mm[i - 1];
            float mm1 = table.mm[i];
            return mm0 + ratio * (mm1 - mm0);
        }
    }
    return table.mm[table.numPoints - 1];
}

static float manualToMm(PistonChannel ch, float rawV) {
    const auto &ref = s_manualRefCache[ch];
    uint8_t mask = ref.mask;
    if (((mask & (1u << PISTON_REF_CLOSED)) == 0) || ((mask & (1u << PISTON_REF_OPEN)) == 0)) {
        return NAN;
    }
    float closed = ref.raw[PISTON_REF_CLOSED];
    float open   = ref.raw[PISTON_REF_OPEN];
    float span = open - closed;
    if (fabsf(span) < 1e-4f) return NAN;
    float strokeMm = s_pistonCalCache[ch].valid ? s_pistonCalCache[ch].stroke_mm : PISTON_MANUAL_STROKE_MM;
    float t = (rawV - closed) / span;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return strokeMm * t;
}

static float pistonToMm(PistonChannel ch, float rawV) {
    // Oncelik: manuel referans -> kalibrasyon tablosu -> varsayilan 26mm harita
    float manual = manualToMm(ch, rawV);
    if (!isnan(manual)) return manual;
    return tableToMm(ch, rawV);
}

static inline void updateHallFilters(float &fast, float &slow, float sample) {
    fast = lpf(fast, sample, HALL_ALPHA_FAST);
    slow = lpf(slow, sample, HALL_ALPHA_SLOW);
}

static inline float updateVelocity(float pos, float &posPrev, float &vel, float dt) {
    float inst = (pos - posPrev) / dt;
    posPrev = pos;
    vel = lpf(vel, inst, VEL_ALPHA);
    return clampf(vel, -VEL_MAX_ABS, VEL_MAX_ABS);
}

float k1r[3] = {0};
float k2r[3] = {0};


void TaskADSMonitor(void *pvParameters) {
    (void) pvParameters;

    // I2C'nin hazır olmasını bekle (TaskI2CMonitor mutex'i oluşturur)
    vTaskDelay(pdMS_TO_TICKS(500));

    // IO2 ve IO10: analogRead için başlatma (ADC pin modu varsayılan)
    analogSetAttenuation(ADC_11db);

    // ---- ADS1115 otomatik tespiti (eski kart uyumluluğu) ----
    // Eski kartlar: ADS1115 @ 0x48, AIN2=sensör1, AIN3=sensör2
    // Yeni kartlar: ADS yok, IO10=sensör1, IO2=sensör2
    {
        bool probeOk = false;
        if (g_i2cMutex && xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
            Wire.beginTransmission(0x48);
            probeOk = (Wire.endTransmission() == 0);
            if (probeOk) {
                s_ads.begin(0x48, &Wire);
                s_ads.setGain(GAIN_ONE);              // ±4.096V - 3.3V NTC için yeterli
                s_ads.setDataRate(RATE_ADS1115_860SPS); // en hızlı mod (~1.2ms/örnek)
            }
            xSemaphoreGive(g_i2cMutex);
        }
        s_hasADS1115 = probeOk;
        {
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, probeOk ? (char*)"[TEMP] ADS1115@0x48 bulundu - eski kart (AIN2/AIN3)"
                        : (char*)"[TEMP] ADS1115 yok - yeni kart (IO10/IO2)");
        }
    }

    // Başlangıç log kaldırıldı - queue spam azaltıldı

    for (;;) {

        uint32_t now = millis();
        float dt = (t_prev_ms==0) ? 0.01f : (now - t_prev_ms) * 0.001f;
        t_prev_ms = now;
        dt = clampf(dt, DT_MIN_S, DT_MAX_S);
        // NTC sıcaklık okuması: IO2=sensör1, IO10=sensör2
        auto adcToVolt = [](int raw)->float { return (raw / 4095.0f) * 3.3f; };
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            if (s_pistonCalSeqCache != g_pistonCalSeq) {
                s_pistonCalSeqCache = g_pistonCalSeq;
                for (int i=0;i<PISTON_CHANNEL_COUNT;i++) {
                    s_pistonCalCache[i] = g_pistonCalTable[i];
                }
            }
            // TMAG kalibrasyon cache güncelle
            if (s_tmagCalSeqCache != g_tmagCalibSeq) {
                s_tmagCalSeqCache = g_tmagCalibSeq;
                for (int i=0;i<PISTON_CHANNEL_COUNT;i++) {
                    s_tmagCalCache[i] = g_tmagPistonCalib[i];
                }
                s_tmagKavCalCache[0] = g_tmagKavramaCalib[0];
                s_tmagKavCalCache[1] = g_tmagKavramaCalib[1];
            }
            // TMAG verilerini cache'le
            for (int i=0; i<TMAG_CH_COUNT; i++) {
                s_tmagCache[i] = g_tmagData[i];
            }
            for (int i=0; i<PISTON_CHANNEL_COUNT; ++i) {
                s_manualRefCache[i].mask = g_pistonManualRef[i].validMask;
                for (int st=0; st<3; ++st) {
                    s_manualRefCache[i].raw[st] = g_pistonManualRef[i].raw[st];
                }
            }
            xSemaphoreGive(g_sharedMutex);
        }

        // ============ TMAG5173 TABANLI POZİSYON HESAPLAMA ============
        // Sadece TMAG sensörleri kullanılıyor (DRV5055 devre dışı)
        float p57, p13, p24, p6R;
        
        // TMAG Z değerlerinden pozisyon hesapla
        p13 = tmagZToMm(PISTON_1_3, s_tmagCache[TMAG_CH_1_3].z);
        p57 = tmagZToMm(PISTON_5_7, s_tmagCache[TMAG_CH_5_7].z);
        p24 = tmagZToMm(PISTON_2_4, s_tmagCache[TMAG_CH_2_4].z);
        p6R = tmagZToMm(PISTON_6_R, s_tmagCache[TMAG_CH_6_R].z);

        // Hiz (mm/s) - Her zaman hesaplanir: on-filtreli pozisyon turevinden
        // 1. Pozisyon EMA (gürültü bastirma, turevi almadan once)
        // 2. Filtreli pozisyonun 1. dereceden turevi
        // 3. Hafif son EMA (VEL_ALPHA=0.10)
        posf_57 = lpf(posf_57, p57, POS_ALPHA);
        v_57    = updateVelocity(posf_57, pos_57, v_57, dt);

        posf_13 = lpf(posf_13, p13, POS_ALPHA);
        v_13    = updateVelocity(posf_13, pos_13, v_13, dt);

        posf_24 = lpf(posf_24, p24, POS_ALPHA);
        v_24    = updateVelocity(posf_24, pos_24, v_24, dt);

        posf_6R = lpf(posf_6R, p6R, POS_ALPHA);
        v_6R    = updateVelocity(posf_6R, pos_6R, v_6R, dt);

        // K1/K2 (Kavrama) ölçümü - Sadece TMAG5173 kullanılıyor
        float pK1 = tmagKavramaMm(0);
        float pK2 = tmagKavramaMm(1);

        posf_K1 = lpf(posf_K1, pK1, POS_ALPHA);
        v_K1    = updateVelocity(posf_K1, pos_K1, v_K1, dt);

        posf_K2 = lpf(posf_K2, pK2, POS_ALPHA);
        v_K2    = updateVelocity(posf_K2, pos_K2, v_K2, dt);

        // =========================
        // Sıcaklık sensörleri (otomatik kart tespiti)
        // Eski kart: ADS1115@0x48 - AIN2=sensör1, AIN3=sensör2
        // Yeni kart: MCU ADC      - IO10=sensör1, IO2=sensör2
        // =========================
        float v_t1 = 0.0f, v_t2 = 0.0f;
        if (s_hasADS1115) {
            // ADS1115 (16-bit): tek örnek yeterli
            if (g_i2cMutex && xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                int16_t raw2 = s_ads.readADC_SingleEnded(2);  // AIN2: sensör 1
                int16_t raw3 = s_ads.readADC_SingleEnded(3);  // AIN3: sensör 2
                xSemaphoreGive(g_i2cMutex);
                v_t1 = s_ads.computeVolts(raw2);
                v_t2 = s_ads.computeVolts(raw3);
            }
        } else {
            // MCU ADC (12-bit, gürültcü): TEMP_OVERSAMPLE örnek ortalaması → 2x SNR kazanımı
            float sum1 = 0.0f, sum2 = 0.0f;
            for (int i = 0; i < TEMP_OVERSAMPLE; i++) {
                sum1 += adcToVolt(analogRead(TEMP_SENSOR_1_PIN));
                sum2 += adcToVolt(analogRead(TEMP_SENSOR_2_PIN));
            }
            v_t1 = sum1 / TEMP_OVERSAMPLE;
            v_t2 = sum2 / TEMP_OVERSAMPLE;
        }

        // EMA filtresi: voltaj üzerine uygula, sonra NTC dönüşümü yap
        // (non-lineer log dönüşümü öncesi filtrelemek matematik açısından doğru)
        if (s_v_t1_filt < 0.0f) {
            s_v_t1_filt = v_t1;  // İlk çalışmada anlık başlat
            s_v_t2_filt = v_t2;
        } else {
            s_v_t1_filt = lpf(s_v_t1_filt, v_t1, TEMP_ALPHA);
            s_v_t2_filt = lpf(s_v_t2_filt, v_t2, TEMP_ALPHA);
        }

        float temp1_C = ntcVoltageToCelsius(s_v_t1_filt);
        float temp2_C = ntcVoltageToCelsius(s_v_t2_filt);

        // -------- ISITICI GÜVENLİK KONTROLÜ --------
        // Yağ sıcaklığı 90°C'yi aşarsa ısıtıcıyı otomatik kapat
        if (temp2_C >= 90.0f && g_ssrDesired != 0) {
            g_ssrDesired = 0;
            {
                kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[HEATER] SAFETY OFF - Oil temp >= 90C");
            }
        }

        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_temp1_C     = temp1_C;
            g_temp2_C     = temp2_C;
            // Piston ham pozisyon: TMAG mm değerleri kullanılıyor (MCU ADC hall sensörler kaldırıldı)
            g_pistonHallRaw[PISTON_5_7] = pos_57;
            g_pistonHallRaw[PISTON_1_3] = pos_13;
            g_pistonHallRaw[PISTON_2_4] = pos_24;
            g_pistonHallRaw[PISTON_6_R] = pos_6R;
            g_piston_1_3_mm = pos_13; g_piston_5_7_mm = pos_57;
            g_piston_2_4_mm = pos_24; g_piston_6_R_mm = pos_6R;
            g_v_1_3_mms = v_13; g_v_5_7_mms = v_57; g_v_2_4_mms = v_24; g_v_6_R_mms = v_6R;

            

            g_n435_stroke_mm = pK1;  // TMAG bazlı kavrama 1
            g_v_K1_mms = v_K1;

            g_n439_stroke_mm = pK2;  // TMAG bazlı kavrama 2
            g_v_K2_mms = v_K2;

            g_pistonHallmm[0]=pos_57;
            g_pistonHallmm[1]=pos_13;
            g_pistonHallmm[2]=pos_24;
            g_pistonHallmm[3]=pos_6R;
            g_pistonHallmm[4]=g_n435_stroke_mm;
            g_pistonHallmm[5]=g_n439_stroke_mm;

            xSemaphoreGive(g_sharedMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz - hall sensör için yeterli (CPU optimize)
    }
}

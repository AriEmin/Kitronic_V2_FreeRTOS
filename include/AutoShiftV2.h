// include/AutoShiftV2.h
#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ============================================================================
// DQ200 Otomatik Vites Değiştirme V2 - Veri Yapıları
// ============================================================================

// --- Piston Pozisyonları ---
enum PistonPos : uint8_t {
    POS_CLOSED = 0,   // Kapalı
    POS_MID    = 1,   // Orta
    POS_OPEN   = 2    // Açık
};

// --- Vites Durumları ---
enum GearState : uint8_t {
    GEAR_P  = 0,   // Park
    GEAR_R  = 1,   // Geri
    GEAR_N  = 2,   // Nötr
    GEAR_D1 = 3,   // 1. vites
    GEAR_D2 = 4,   // 2. vites
    GEAR_D3 = 5,   // 3. vites
    GEAR_D4 = 6,   // 4. vites
    GEAR_D5 = 7,   // 5. vites
    GEAR_D6 = 8,   // 6. vites
    GEAR_D7 = 9,   // 7. vites
    GEAR_COUNT = 10
};

// --- Kavrama Durumu ---
enum ClutchState : uint8_t {
    CLUTCH_NONE = 0,  // Her iki kavrama kapalı
    CLUTCH_K1   = 1,  // K1 açık (tek vitesler: 1,3,5,7)
    CLUTCH_K2   = 2   // K2 açık (çift vitesler: 2,4,6 + R)
};

// --- Test Aşamaları ---
enum AutoShiftV2Phase : uint8_t {
    PHASE_IDLE = 0,           // Beklemede
    PHASE_INIT,               // Başlatılıyor
    PHASE_PRESSURE_BUILDUP,   // Basınç oluşturuluyor
    PHASE_GEAR_PRESELECT,     // Sonraki vites hazırlanıyor
    PHASE_CLUTCH_ENGAGE,      // Kavrama değiştiriliyor
    PHASE_GEAR_ACTIVE,        // Vites aktif, bekleme
    PHASE_SHIFT_TRANSITION,   // Vites geçişi
    PHASE_COMPLETED,          // Test tamamlandı
    PHASE_ERROR,              // Hata durumu
    PHASE_ABORTING            // İptal ediliyor
};

// --- Hata Kodları ---
enum AutoShiftV2Fault : uint16_t {
    FAULT_NONE              = 0x0000,
    FAULT_LOW_PRESSURE      = 0x0001,  // Basınç düşük
    FAULT_HIGH_PRESSURE     = 0x0002,  // Basınç yüksek
    FAULT_PRESSURE_DROP     = 0x0004,  // Beklenmeyen basınç düşüşü
    FAULT_PISTON_STUCK      = 0x0008,  // Piston hareket etmiyor
    FAULT_PISTON_UNEXPECTED = 0x0010,  // Beklenmeyen piston hareketi
    FAULT_VALVE_OPEN        = 0x0020,  // Valf açık devre
    FAULT_VALVE_SHORT       = 0x0040,  // Valf kısa devre
    FAULT_CLUTCH_ERROR      = 0x0080,  // Kavrama hatası
    FAULT_TIMEOUT           = 0x0100,  // Zaman aşımı
    FAULT_PUMP_ERROR        = 0x0200,  // Pompa hatası
    FAULT_SENSOR_ERROR      = 0x0400,  // Sensör hatası
    FAULT_PUMP_TIMEOUT      = 0x0800,  // Pompa belirli süreden fazla çalıştı → kaçak şüphesi
    FAULT_ABORT             = 0x8000   // Kullanıcı iptal
};

// --- Detaylı Piston Hata Türleri (AutoShiftV2ErrorEntry.faultDetail) ---
// Faz 9'da hangi pistonun hangi durumda başarısız olduğunu netleştirir.
enum PistonFaultDetail : uint8_t {
    PFD_NONE          = 0,
    PFD_NOT_OPENED    = 1,   // Hedef OPEN, piston açılmadı (kapalı/orta kaldı)
    PFD_NOT_CLOSED    = 2,   // Hedef CLOSED, piston kapanmadı (orta/açık kaldı)
    PFD_NOT_HELD_MID  = 3,   // Hedef MID, piston ortada tutulamadı (kapanan/açılan tarafa kaydı)
    PFD_OUT_OF_RANGE  = 4    // Hall değeri kalibrasyon aralığı dışı (sensör/kalibrasyon arızası)
};

// İnsanca hata adı (log için)
inline const char* PistonFaultDetailToStr(uint8_t fd) {
    switch (fd) {
        case PFD_NOT_OPENED:   return "ACILMADI";
        case PFD_NOT_CLOSED:   return "KAPANMADI";
        case PFD_NOT_HELD_MID: return "ORTADA-TUTULAMADI";
        case PFD_OUT_OF_RANGE: return "ARALIK-DISI";
        default:               return "BILINMEYEN";
    }
}

// --- Vites Pozisyon Tablosu ---
// Her vites için 4 piston pozisyonu (1-3, 5-7, 2-4, 6-R) + kavrama durumu
struct GearPosition {
    PistonPos p1_3;       // Piston 1-3
    PistonPos p5_7;       // Piston 5-7
    PistonPos p2_4;       // Piston 2-4
    PistonPos p6_R;       // Piston 6-R
    ClutchState clutch;   // Kavrama durumu
};

// --- Valf İndeksleri ---
// Sıra: 0:N433  1:N436  2:N434  3:N435  4:N438  5:N440  6:N439  7:N437
enum ValveIndex : uint8_t {
    VALVE_1_3  = 0,   // N433: Piston 1-3 kontrolü
    VALVE_MAIN1= 1,   // N436: Ana basınç valfi 1 (PCV)
    VALVE_5_7  = 2,   // N434: Piston 5-7 kontrolü
    VALVE_K1   = 3,   // N435: Kavrama K1
    VALVE_6_R  = 4,   // N438: Piston 6-R kontrolü
    VALVE_MAIN2= 5,   // N440: Ana basınç valfi 2 (PCV)
    VALVE_K2   = 6,   // N439: Kavrama K2
    VALVE_2_4  = 7    // N437: Piston 2-4 kontrolü
};

// --- Piston Valf Kalibrasyon Yapısı ---
// selo1_espnow.cpp'deki SpoolMap benzeri
// Ortada tutma için kalibre edilmiş PWM değerleri
struct PistonValveCalib {
    uint16_t dutyHold;        // Ortada tutma PWM (kritik - kalibrasyon gerektirir)
    uint16_t dutyExtendBase;  // Yavaş açma başlangıç PWM
    uint16_t dutyRetractBase; // Yavaş kapama başlangıç PWM
    uint16_t dutyMax;         // Maksimum PWM (kesinlikle 2000'i geçmemeli!)
    float    kDutyPerMm;      // mm başına duty artışı
    float    deadbandMm;      // Tolerans (mm)
    bool     calibrated;      // Kalibrasyon yapıldı mı
};

// --- Test Konfigürasyonu ---
struct AutoShiftV2Config {
    uint16_t gearHoldMs;         // Viteste bekleme süresi (ms) [1500, 2000, 5000]
    uint16_t clutchTransMs;      // Kavrama geçiş süresi (ms)
    uint16_t preselectMs;        // Pre-select bekleme süresi (ms)
    float    minPressureBar;     // Minimum basınç (bar) [42]
    float    maxPressureBar;     // Maksimum basınç (bar) [60]
    float    pressureDropWarn;   // Uyarı için basınç düşüşü (bar)
    float    pressureDropFault;  // Hata için basınç düşüşü (bar)
    uint16_t valveCurrentMin;    // Minimum valf akımı (mA)
    uint16_t valveCurrentMax;    // Maksimum valf akımı (mA)
    uint16_t mainValveCurrent;   // Ana valf akımı (mA) [~1000]
    bool     autoRepeat;         // Otomatik tekrar
    uint8_t  repeatCount;        // Tekrar sayısı
    GearState targetGear;        // Hedef vites (oto mod için)
    bool     manualMode;         // Manuel mod aktif mi
};

// --- Adım Tanılama Verisi ---
struct AutoShiftV2StepDiag {
    GearState gear;              // Hangi vites
    uint32_t  startMs;           // Başlangıç zamanı
    uint32_t  endMs;             // Bitiş zamanı
    float     pressureStart;     // Başlangıç basıncı
    float     pressureEnd;       // Bitiş basıncı
    float     pressureDrop;      // Basınç düşüşü
    float     valveCurrent[8];   // Valf akımları
    float     pistonPos[4];      // Piston pozisyonları (mm)
    uint8_t   pistonState[4];    // Piston durumları (POS_*)
    uint16_t  faults;            // Bu adımdaki hatalar
    bool      success;           // Adım başarılı mı
};

// --- Hata Geçmişi Kaydı ---
// Her hata oluştuğunda kaydedilir, test sonunda GUI'ye gönderilir
#define MAX_ERROR_HISTORY 20  // Maksimum hata kaydı sayısı

struct AutoShiftV2ErrorEntry {
    GearState gear;              // Hangi viteste
    uint8_t   pistonIdx;         // Hangi piston (0-3)
    uint8_t   repeatIdx;         // Kaçıncı tekrarda
    uint16_t  faultType;         // Hata tipi (AutoShiftV2Fault)
    uint32_t  timestampMs;       // Hata zamanı (test başından itibaren)
    int16_t   hallValue;         // O anki hall değeri
    int16_t   expectedMin;       // Beklenen min
    int16_t   expectedMax;       // Beklenen max
    uint8_t   expectedPos;       // Beklenen pozisyon (0=kapalı, 1=orta, 2=açık)
    uint8_t   faultDetail;       // PistonFaultDetail — neden başarısız oldu (PFD_*)
    uint8_t   valveIdx;          // Hangi valf (0-7, ValveIndex enum)
};

struct AutoShiftV2ErrorHistory {
    AutoShiftV2ErrorEntry entries[MAX_ERROR_HISTORY];
    uint8_t  count;              // Kayıtlı hata sayısı
    uint8_t  skippedGears;       // Atlanan vites sayısı (hata nedeniyle)
    uint16_t faultyGearsMask;    // Hatalı vitesler bitmask (bit 0=P, 1=R, 2=N, 3=D1, ... 9=D7)
};

extern AutoShiftV2ErrorHistory g_autoShiftV2Errors;

// --- Per-Piston Hareket İstatistikleri (Faz 9 süresince birikir) ---
// 4 piston: 0=P5-7, 1=P1-3, 2=P2-4, 3=P6-R  (Convention A)
struct PerPistonStats {
    uint32_t totalMoves;          // Toplam hareket sayısı (açma + kapama)
    uint32_t openMoves;           // Açma hareketi sayısı
    uint32_t closeMoves;          // Kapama hareketi sayısı
    uint32_t totalOpenMs;         // Toplam açma süresi (ms birikimi)
    uint32_t totalCloseMs;        // Toplam kapama süresi (ms birikimi)
    uint32_t maxOpenMs;           // Maksimum açma süresi (ms)
    uint32_t maxCloseMs;          // Maksimum kapama süresi (ms)
    float    totalPressDropOpen;  // Toplam basınç düşümü - açma (bar birikimi)
    float    totalPressDropClose; // Toplam basınç düşümü - kapama (bar birikimi)
    float    maxPressDropOpen;    // Max basınç düşümü - açma (bar)
    float    maxPressDropClose;   // Max basınç düşümü - kapama (bar)
    uint16_t slowOpenCount;       // Yavaş açma sayısı (> eşik ms)
    uint16_t slowCloseCount;      // Yavaş kapama sayısı (> eşik ms)
};

extern PerPistonStats g_pistonStats[4];  // Faz 9 süresince doldurulan istatistikler

// --- Test Raporu ---
struct AutoShiftV2Report {
    uint32_t  testStartMs;       // Test başlangıç zamanı
    uint32_t  testEndMs;         // Test bitiş zamanı
    uint16_t  totalShifts;       // Toplam vites değişimi
    uint16_t  successfulShifts;  // Başarılı vites değişimi
    uint16_t  failedShifts;      // Başarısız vites değişimi
    
    // Bileşen durumları
    bool      pumpOk;            // Pompa sağlığı
    bool      pressureSensorOk;  // Basınç sensörü
    bool      valvesOk[8];       // Her valf için durum
    bool      pistonsOk[4];      // Her piston için durum
    bool      clutchK1Ok;        // K1 kavrama
    bool      clutchK2Ok;        // K2 kavrama
    
    // İstatistikler
    float     avgPressureDrop;   // Ortalama basınç düşüşü
    float     maxPressureDrop;   // Maksimum basınç düşüşü
    float     avgShiftTime;      // Ortalama vites geçiş süresi
    
    // Hata özeti
    uint16_t  faultMask;         // Tüm hatalar (OR)
    uint8_t   faultCount;        // Toplam hata sayısı
    char      recommendation[64];// Tamir önerisi
    
    bool      testPassed;        // Genel sonuç
};

// --- İstek Yapısı ---
struct AutoShiftV2Request {
    bool      start;             // true: başlat, false: durdur
    bool      manualMode;        // Manuel mod
    GearState targetGear;        // Hedef vites
    uint16_t  gearHoldMs;        // Viteste bekleme süresi
    uint8_t   repeatCount;       // Tekrar sayısı
};

// --- Yayın Yapısı (Telemetri) ---
struct AutoShiftV2Publish {
    bool              running;       // Çalışıyor mu
    AutoShiftV2Phase  phase;         // Mevcut aşama
    GearState         currentGear;   // Mevcut vites
    GearState         targetGear;    // Hedef vites
    ClutchState       clutch;        // Kavrama durumu
    uint16_t          stepIdx;       // Adım indeksi
    uint16_t          repeatIdx;     // Tekrar indeksi
    float             pressure;      // Mevcut basınç
    uint16_t          faults;        // Aktif hatalar
    uint8_t           pistonState[4];// Piston durumları
    uint32_t          elapsedMs;     // Geçen süre
    char              statusText[24];// Durum metni
};

// --- Global Değişkenler ---
extern volatile uint32_t     g_autoShiftV2ReqSeq;
extern AutoShiftV2Request    g_autoShiftV2Req;
extern AutoShiftV2Publish    g_autoShiftV2Pub;
extern AutoShiftV2Config     g_autoShiftV2Cfg;
extern AutoShiftV2Report     g_autoShiftV2Report;
extern AutoShiftV2StepDiag   g_autoShiftV2StepDiag;
extern PistonValveCalib      g_pistonCalib[4];  // 4 piston valfi için kalibrasyon

// --- Sabit Vites Pozisyon Tablosu ---
// [GEAR_P ... GEAR_D7] için piston pozisyonları
extern const GearPosition GEAR_POSITIONS[GEAR_COUNT];

// --- Task Prototipi ---
void TaskAutoShiftV2(void *pvParameters);

// --- Yardımcı Fonksiyonlar ---
const char* GearStateToString(GearState gear);
const char* PhaseToString(AutoShiftV2Phase phase);
void AutoShiftV2_GenerateReport();

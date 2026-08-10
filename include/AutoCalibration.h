// include/AutoCalibration.h
#ifndef AUTO_CALIBRATION_H
#define AUTO_CALIBRATION_H

#include <Arduino.h>
#include "AutoShiftV2.h"

// ============================================================================
// Otomatik Kalibrasyon Sistemi
// Test öncesi mekatronik karakteristiklerini ölçer ve timeout değerlerini ayarlar
// ============================================================================

// --- Kalibrasyon Aşamaları ---
enum CalibPhase : uint8_t {
    CALIB_IDLE = 0,           // Beklemede
    CALIB_PRESSURE_BUILDUP,   // Basınç oluşturuluyor
    CALIB_PISTON_1_3,         // Piston 1-3 kalibrasyonu
    CALIB_PISTON_5_7,         // Piston 5-7 kalibrasyonu
    CALIB_PISTON_2_4,         // Piston 2-4 kalibrasyonu
    CALIB_PISTON_6_R,         // Piston 6-R kalibrasyonu
    CALIB_CLUTCH_K1,          // Kavrama K1 kalibrasyonu
    CALIB_CLUTCH_K2,          // Kavrama K2 kalibrasyonu
    CALIB_CALCULATE,          // Timeout değerlerini hesapla
    CALIB_COMPLETED,          // Kalibrasyon tamamlandı
    CALIB_ERROR               // Hata durumu
};

// --- Uyarı Bayrakları (arıza tespiti için) ---
enum CalibWarning : uint16_t {
    WARN_NONE           = 0x0000,
    WARN_HALL_MIN_LOW   = 0x0001,  // Hall min çok düşük
    WARN_HALL_MIN_HIGH  = 0x0002,  // Hall min çok yüksek
    WARN_HALL_MAX_LOW   = 0x0004,  // Hall max çok düşük
    WARN_HALL_MAX_HIGH  = 0x0008,  // Hall max çok yüksek
    WARN_STROKE_SHORT   = 0x0010,  // Strok çok kısa
    WARN_STROKE_LONG    = 0x0020,  // Strok çok uzun
    WARN_MOVE_SLOW      = 0x0040,  // Hareket yavaş
    WARN_MOVE_FAST      = 0x0080,  // Hareket çok hızlı
    WARN_HOLD_LOW       = 0x0100,  // Hold duty çok düşük
    WARN_HOLD_HIGH      = 0x0200,  // Hold duty çok yüksek
};

// --- Vites Pistonu Kalibrasyon Verisi ---
struct GearPistonCalibData {
    // Hall sensör değerleri
    int16_t hallMin;              // Kapalı pozisyon hall değeri
    int16_t hallMid;              // Orta pozisyon hall değeri
    int16_t hallMax;              // Açık pozisyon hall değeri
    
    // Hareket süreleri (ms)
    uint16_t timeCloseToOpen;     // Kapalı → Açık süresi
    uint16_t timeOpenToClose;     // Açık → Kapalı süresi
    uint16_t timeMidToOpen;       // Orta → Açık süresi
    uint16_t timeMidToClose;      // Orta → Kapalı süresi
    uint16_t timeCloseToMid;      // Kapalı → Orta süresi
    uint16_t timeOpenToMid;       // Açık → Orta süresi
    
    // Kararlılık
    uint16_t midStabilityMs;      // Orta pozisyonda kararlılık süresi
    
    // Hesaplanan değerler
    uint16_t calculatedTimeout;   // Hesaplanan timeout (max süre × 1.5)
    uint16_t holdDuty;            // Orta pozisyon tutma duty değeri (eski, geriye uyumluluk)
    
    // Akım tabanlı kalibrasyon değerleri (mA)
    uint16_t openCurrent_mA;      // Açma akımı (pistonu açarken ölçülen)
    uint16_t holdCurrent_mA;      // Tutma akımı (ortada stabil tutan)
    uint16_t closeCurrent_mA;     // Kapatma akımı (pistonu kapatırken ölçülen)
    bool currentCalibValid;       // Akım kalibrasyonu geçerli mi
    
    // Durum
    bool valid;                   // Kalibrasyon geçerli mi
    uint8_t errorCode;            // Hata kodu (0 = başarılı)
    uint16_t warnings;            // Uyarı bayrakları (CalibWarning)
};

// --- Kavrama Pistonu Kalibrasyon Verisi ---
struct ClutchCalibData {
    // Hall sensör değerleri
    int16_t hallOpen;             // Açık pozisyon hall değeri
    int16_t hallClosed;           // Kapalı pozisyon hall değeri
    int16_t hallKissPoint;        // Öpüşme noktası hall değeri
    
    // PWM değerleri
    uint16_t pwmKissPoint;        // Öpüşme noktası PWM değeri
    uint16_t pwmFullEngage;       // Tam kapanma PWM değeri
    
    // Hareket süreleri (ms)
    uint16_t timeOpenToClose;     // Açık → Kapalı süresi
    uint16_t timeCloseToOpen;     // Kapalı → Açık süresi
    uint16_t fillTime;            // Dolum süresi (0'dan tam kapanmaya)
    
    // Hesaplanan değerler
    uint16_t calculatedTimeout;   // Hesaplanan timeout
    
    // Durum
    bool valid;                   // Kalibrasyon geçerli mi
    uint8_t errorCode;            // Hata kodu (0 = başarılı)
};

// --- Sistem Kalibrasyon Verisi ---
struct SystemCalibData {
    // Basınç karakteristiği
    uint16_t pressureBuildupTime; // Basınç oluşturma süresi (ms)
    float    stablePressure;      // Kararlı basınç değeri (bar)
    float    pressureDropRate;    // Basınç düşüş hızı (bar/s)
    
    // Pompa
    bool     pumpResponsive;      // Pompa tepki veriyor mu
    
    // Durum
    bool valid;
};

// --- Ana Kalibrasyon Yapısı ---
struct AutoCalibrationData {
    // Vites pistonları (0=1-3, 1=5-7, 2=2-4, 3=6-R)
    GearPistonCalibData gearPistons[4];
    
    // Kavrama pistonları (0=K1, 1=K2)
    ClutchCalibData clutches[2];
    
    // Sistem
    SystemCalibData system;
    
    // Genel durum
    bool calibrationValid;        // Tüm kalibrasyon geçerli mi
    uint32_t calibrationTime;     // Toplam kalibrasyon süresi (ms)
    uint32_t timestamp;           // Kalibrasyon zamanı (millis)
};

// --- Kalibrasyon Yayın Yapısı (GUI için) ---
struct CalibrationPublish {
    bool running;                 // Kalibrasyon çalışıyor mu
    CalibPhase phase;             // Mevcut aşama
    uint8_t progress;             // İlerleme yüzdesi (0-100)
    uint8_t currentPiston;        // Şu an kalibre edilen piston (0-5)
    char statusText[32];          // Durum metni
    uint32_t elapsedMs;           // Geçen süre
    
    // Son ölçülen değerler (anlık gösterim için)
    int16_t lastHallValue;
    uint16_t lastMoveTime;
};

// --- Kalibrasyon İstek Yapısı ---
struct CalibrationRequest {
    bool start;                   // true: başlat, false: iptal
    bool skipIfValid;             // Geçerli kalibrasyon varsa atla
};

// --- Hata Kodları ---
enum CalibErrorCode : uint8_t {
    CALIB_OK = 0,
    CALIB_ERR_PRESSURE,           // Basınç oluşturulamadı
    CALIB_ERR_PISTON_STUCK,       // Piston hareket etmiyor
    CALIB_ERR_PISTON_SLOW,        // Piston çok yavaş
    CALIB_ERR_HALL_INVALID,       // Hall sensör değeri geçersiz
    CALIB_ERR_NO_KISS_POINT,      // Kavrama öpüşme noktası bulunamadı
    CALIB_ERR_TIMEOUT,            // Zaman aşımı
    CALIB_ERR_ABORTED,            // Kullanıcı iptal etti
    CALIB_ERR_HOLD_FAILED         // Hold duty bulunamadı
};

// --- Global Değişkenler ---
extern AutoCalibrationData g_calibrationData;
extern CalibrationPublish g_calibrationPub;
extern volatile bool g_calibrationRequested;
extern volatile bool g_calibrationAbort;

// --- Fonksiyon Prototipleri ---

// Ana kalibrasyon fonksiyonu (test öncesi çağrılır)
// Blocking: true ise kalibrasyon bitene kadar bekler
// Timeout: maksimum kalibrasyon süresi (ms), 0 = sınırsız
bool AutoCalibration_Run(bool blocking = true, uint32_t timeoutMs = 120000);

// Kalibrasyon iptal
void AutoCalibration_Abort();

// Kalibrasyon geçerli mi kontrol et
bool AutoCalibration_IsValid();

// Kalibre edilmiş timeout değerini al
uint16_t AutoCalibration_GetTimeout(uint8_t pistonIdx);

// Kalibre edilmiş hall limitlerini al
void AutoCalibration_GetHallLimits(uint8_t pistonIdx, int16_t& minVal, int16_t& midVal, int16_t& maxVal);

// Kavrama öpüşme noktasını al
int16_t AutoCalibration_GetClutchKissPoint(uint8_t clutchIdx);

// Kalibrasyon verilerini sıfırla
void AutoCalibration_Reset();

// Kalibrasyon task'ı (FreeRTOS)
void TaskAutoCalibration(void *pvParameters);

// Aşama adını string olarak al
const char* CalibPhaseToString(CalibPhase phase);

#endif

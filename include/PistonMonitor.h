#ifndef PISTON_MONITOR_H
#define PISTON_MONITOR_H

#include <Arduino.h>
#include "AutoShiftV2.h"

// ============================================================================
// Piston İzleme Sistemi
// ============================================================================
// Her piston için bağımsız izleme:
// - Hedef pozisyona ulaşma süresi
// - Beklenmeyen hareket tespiti
// - Hata kaydı (test sonunda rapora aktarılır)

// Piston türleri
enum PistonType : uint8_t {
    PISTON_TYPE_GEAR = 0,    // Vites pistonu (4 adet)
    PISTON_TYPE_CLUTCH = 1   // Kavrama pistonu (2 adet)
};

// Piston durumu
enum PistonStatus : uint8_t {
    PISTON_STATUS_IDLE = 0,       // Beklemede
    PISTON_STATUS_MOVING = 1,     // Hareket ediyor
    PISTON_STATUS_AT_TARGET = 2,  // Hedefe ulaştı
    PISTON_STATUS_ERROR = 3       // Hata (hedefe ulaşamadı)
};

// Hata türleri (her piston için)
enum PistonErrorType : uint16_t {
    PERR_NONE = 0,
    PERR_TIMEOUT = 0x0001,           // Hedefe zamanında ulaşamadı
    PERR_UNEXPECTED_MOVE = 0x0002,   // Beklenmeyen hareket
    PERR_STUCK = 0x0004,             // Takılı kaldı (hiç hareket etmedi)
    PERR_OVERSHOOT = 0x0008,         // Hedefi geçti
    PERR_OSCILLATION = 0x0010,       // Salınım yapıyor
    PERR_SENSOR_INVALID = 0x0020     // Sensör verisi geçersiz
};

// Tek piston için izleme verisi
struct PistonMonitorData {
    // Tanımlama
    uint8_t     index;           // Piston indeksi (0-5)
    PistonType  type;            // Vites veya kavrama
    const char* name;            // İsim ("P5-7", "K1", vb.)
    
    // Mevcut durum
    PistonStatus status;         // Mevcut durum
    PistonPos    targetPos;      // Hedef pozisyon
    PistonPos    currentPos;     // Mevcut pozisyon (tespit edilen)
    int16_t      hallValue;      // Mevcut hall değeri
    
    // Zamanlama
    uint32_t     moveStartMs;    // Hareket başlangıç zamanı
    uint32_t     targetReachedMs;// Hedefe ulaşma zamanı
    uint32_t     lastMoveMs;     // Son hareket zamanı (ms)
    
    // İstatistikler (bu test için)
    uint16_t     totalMoves;     // Toplam hareket sayısı
    uint16_t     successfulMoves;// Başarılı hareket sayısı
    uint16_t     failedMoves;    // Başarısız hareket sayısı
    uint32_t     totalMoveTimeMs;// Toplam hareket süresi
    uint32_t     minMoveTimeMs;  // En kısa hareket süresi
    uint32_t     maxMoveTimeMs;  // En uzun hareket süresi
    
    // Hata
    uint16_t     errorMask;      // Hata bayrakları (PistonErrorType)
    uint8_t      errorCount;     // Toplam hata sayısı
};

// Piston hatası kaydı (detaylı)
#define MAX_PISTON_ERRORS 30  // Maksimum hata kaydı

struct PistonErrorEntry {
    uint8_t      pistonIdx;      // Piston indeksi (0-5)
    GearState    gear;           // Hangi viteste
    uint8_t      repeatIdx;      // Kaçıncı tekrarda
    uint16_t     errorType;      // Hata türü (PistonErrorType)
    uint32_t     timestampMs;    // Hata zamanı
    int16_t      hallValue;      // O anki hall değeri
    int16_t      expectedHall;   // Beklenen hall değeri
    uint32_t     moveTimeMs;     // Hareket süresi (varsa)
};

struct PistonErrorHistory {
    PistonErrorEntry entries[MAX_PISTON_ERRORS];
    uint8_t count;
};

// Global değişkenler (extern)
extern PistonMonitorData g_pistonMonitor[6];  // 4 vites + 2 kavrama
extern PistonErrorHistory g_pistonErrors;
extern bool g_pistonMonitorEnabled;

// Piston isimleri
extern const char* PISTON_NAMES[6];

// ============================================================================
// API Fonksiyonları
// ============================================================================

// Monitor başlat/durdur
void pistonMonitorInit();
void pistonMonitorStart();  // Test başladığında çağır
void pistonMonitorStop();   // Test bittiğinde çağır
void pistonMonitorReset();  // İstatistikleri sıfırla

// Hedef ayarla (TaskAutoShiftV2'den çağrılır)
void pistonMonitorSetTarget(uint8_t pistonIdx, PistonPos target, GearState gear);

// Kavrama hedefi ayarla
void pistonMonitorSetClutch(ClutchState clutch, GearState gear);

// Task fonksiyonu
void TaskPistonMonitor(void* param);

// Yardımcı fonksiyonlar
const char* getPistonStatusStr(PistonStatus status);
const char* getPistonErrorStr(uint16_t errorType);

#endif // PISTON_MONITOR_H

#ifndef VALVE_CURRENT_CONTROL_H
#define VALVE_CURRENT_CONTROL_H

#include <Arduino.h>

// ============================================================================
// Valf Akım Kontrolü - Solenoid valfler için akım geri beslemeli PWM kontrol
// ============================================================================

// Valf çalışma modları
enum ValveCurrentMode : uint8_t {
    VALVE_MODE_OFF = 0,      // Valf kapalı (0 mA)
    VALVE_MODE_OPEN,         // Açma (yüksek akım)
    VALVE_MODE_HOLD,         // Tutma (orta akım)
    VALVE_MODE_CLOSE,        // Kapatma (düşük akım)
    VALVE_MODE_MANUAL,        // Manuel PWM (akım kontrolü yok)
    VALVE_MODE_OPEN_SLOW,    // Yavaş açma (600 mA)
    VALVE_MODE_CLOSE_SLOW,   // Yavaş kapatma (400 mA)
    VALVE_MODE_PCV           // Basınç kontrol valfi (N436/N440) - 650 mA
};

// Hedef akım değerleri (mA)
struct ValveCurrentTargets {
    float openCurrent_mA;    // Açma akımı (750-800 mA)
    float holdCurrent_mA;    // Tutma akımı (450-500 mA)
    float closeCurrent_mA;   // Kapatma akımı (250-300 mA)
    float slowopenCurrent_mA;    // Yavaş açma akımı (600 mA)
    float slowcloseCurrent_mA;   // Yavaş kapatma akımı (400 mA)
    float pcvCurrent_mA;         // Basınç kontrol valfi akımı (650 mA)
};

// Varsayılan hedef akımlar
static const ValveCurrentTargets DEFAULT_CURRENT_TARGETS = {
    .openCurrent_mA  = 850.0f,   // Açma: 750-800 mA ortası
    .holdCurrent_mA  = 500.0f,   // Hold: 450-500 mA ortası
    .closeCurrent_mA = 250.0f,    // Kapatma: 250-300 mA ortası
    .slowopenCurrent_mA = 630.0f,  // Yavaş açma: 600mA
    .slowcloseCurrent_mA = 380.0f,  // Yavaş kapatma: 400mA
    .pcvCurrent_mA = 650.0f         // PCV (N436/N440): 650mA
};

// PI kontrolör parametreleri
struct ValveCurrentPIParams {
    float Kp;                // Oransal kazanç
    float Ki;                // İntegral kazanç
    float integralMax;       // İntegral sınırı (anti-windup)
    float outputMin;         // PWM minimum
    float outputMax;         // PWM maksimum
};

// Varsayılan PI parametreleri (düşük kazanç - kararlılık için)
static const ValveCurrentPIParams DEFAULT_PI_PARAMS = {
    .Kp = 0.3f,           // Düşük oransal kazanç
    .Ki = 0.05f,          // Çok düşük integral kazanç
    .integralMax = 200.0f,
    .outputMin = 0.0f,
    .outputMax = 4000.0f
};

// Tek valf için kontrolör durumu
struct ValveCurrentController {
    ValveCurrentMode mode;       // Çalışma modu
    float targetCurrent_mA;      // Hedef akım
    float measuredCurrent_mA;    // Ölçülen akım
    float error;                 // Hata (target - measured)
    float integral;              // İntegral birikimi
    uint16_t pwmOutput;          // PWM çıkışı (0-2500)
    bool enabled;                // Akım kontrolü aktif mi
    uint32_t lastUpdateMs;       // Son güncelleme zamanı
};

// Global akım kontrol sistemi durumu
struct ValveCurrentSystem {
    ValveCurrentController valves[8];  // 8 valf kontrolörü
    ValveCurrentTargets targets;        // Hedef akımlar
    ValveCurrentPIParams piParams;      // PI parametreleri
    bool systemEnabled;                 // Sistem aktif mi
    float temperature_C;                // Sıcaklık (kompanzasyon için)
};

// ============================================================================
// Fonksiyon Prototipleri
// ============================================================================

// Sistemi başlat
void ValveCurrentControl_Init();

// Valf modunu ayarla (hedef akım otomatik belirlenir)
void ValveCurrentControl_SetMode(uint8_t valveIdx, ValveCurrentMode mode);

// Valf modunu ve başlangıç PWM'ini ayarla (orijinal hedef duty'den başla)
void ValveCurrentControl_SetModeWithInitialPWM(uint8_t valveIdx, ValveCurrentMode mode, uint16_t initialPWM);

// Manuel hedef akım ayarla
void ValveCurrentControl_SetTargetCurrent(uint8_t valveIdx, float current_mA);

// Manuel PWM modu (akım kontrolü bypass)
void ValveCurrentControl_SetManualPWM(uint8_t valveIdx, uint16_t pwm);

// Akım kontrolünü aktif/pasif yap
void ValveCurrentControl_Enable(uint8_t valveIdx, bool enable);

// Tüm sistemi aktif/pasif yap
void ValveCurrentControl_EnableSystem(bool enable);

// Periyodik güncelleme (TaskValveControl'dan çağrılır)
// Ölçülen akımı alır, PI kontrolör çalıştırır, PWM çıkışı üretir
uint16_t ValveCurrentControl_Update(uint8_t valveIdx, float measuredCurrent_mA);

// Hedef akımları ayarla
void ValveCurrentControl_SetTargets(const ValveCurrentTargets& targets);

// PI parametrelerini ayarla
void ValveCurrentControl_SetPIParams(const ValveCurrentPIParams& params);

// Sıcaklık güncelle (kompanzasyon için)
void ValveCurrentControl_SetTemperature(float temp_C);

// Durum sorgulama
ValveCurrentMode ValveCurrentControl_GetMode(uint8_t valveIdx);
float ValveCurrentControl_GetTargetCurrent(uint8_t valveIdx);
float ValveCurrentControl_GetMeasuredCurrent(uint8_t valveIdx);
uint16_t ValveCurrentControl_GetPWM(uint8_t valveIdx);
bool ValveCurrentControl_IsEnabled(uint8_t valveIdx);

// Debug: Tüm kontrolör durumunu al
const ValveCurrentSystem& ValveCurrentControl_GetSystem();

#endif

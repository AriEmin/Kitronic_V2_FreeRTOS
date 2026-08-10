#include "Shared.h"
#include "AutoDiag.h"
#include <Preferences.h>
#include "modules/tca9548a.h"
#include "modules/tmag5173.h"

// RTOS
SemaphoreHandle_t g_sharedMutex     = nullptr;
SemaphoreHandle_t g_i2cMutex        = nullptr;  // Global I2C bus mutex
portMUX_TYPE      g_portMux         = portMUX_INITIALIZER_UNLOCKED;


float             g_pumpRunCurrent_A = 4000.0f;
float             g_pumpRunRpm       = 4000.0f;

// Güç izleme
float g_mainPwrVoltage_V = 0.0f, g_mainPwrCurrent_A = 0.0f;
float g_vescPwrVoltage_V = 0.0f, g_vescPwrCurrent_A = 0.0f;

// SSR / Isıtıcı
uint8_t  g_ssrDesired      = 0;
uint16_t g_heater_on_ms    = 10000; // Varsayılan: 10s ON
uint16_t g_heater_off_ms   = 5000;  // Varsayılan: 5s OFF  → %67 duty
float    g_heater_setpoint  = 0.0f; // 0 = setpoint devre dışı

// Valf ölçü/duty
float    g_valveCurrent_mA[8]   = {0};
float    g_valveBusVoltage_V[8] = {0};
uint16_t g_valveDutyCounts[8]   = {0};
uint16_t g_valveTargetDuty[8]         = {0};
float    g_valveCustomCurrent_mA[8]   = {0}; // Per-valf özel akım hedefi (0=devre dışı)
uint8_t  g_valveCustomMode[8]         = {0}; // 0=off,1=open,2=close,3=open_slow,4=close_slow,5=pcv

// ADS/Analog
float g_pressure0_V = 0.0f, g_pressure1_V = 0.0f;
float g_temp1_C = 0.0f, g_temp2_C = 0.0f;
float g_piston_1_3_mm = 0.0f, g_piston_5_7_mm = 0.0f, g_piston_2_4_mm = 0.0f, g_piston_6_R_mm = 0.0f;
float g_v_1_3_mms = 0.0f, g_v_5_7_mms = 0.0f,g_v_2_4_mms = 0.0f,g_v_6_R_mms = 0.0f,g_v_K1_mms = 0.0f,g_v_K2_mms = 0.0f;
float g_pistonHallRaw[PISTON_CHANNEL_COUNT] = {0.0f};
float g_pistonHallmm[6] = {0.0f};
PistonCalibrationTable   g_pistonCalTable[PISTON_CHANNEL_COUNT] = {};
volatile uint32_t        g_pistonCalSeq = 0;
volatile uint32_t        g_pistonCalReqSeq = 0;
PistonCalibrationRequest g_pistonCalReq = {};
volatile uint8_t         g_pistonCalRunning = 0;
volatile uint8_t         g_pistonCalPhase = 0;
volatile uint32_t        g_pistonRefReqSeq = 0;
PistonReferenceRequest   g_pistonRefReq = {};
PistonManualReference    g_pistonManualRef[PISTON_CHANNEL_COUNT] = {};
volatile uint32_t        g_pistonHoldReqSeq[PISTON_CHANNEL_COUNT] = {0};
PistonHoldRequest        g_pistonHoldReq[PISTON_CHANNEL_COUNT] = {};
uint8_t                  g_pistonState[PISTON_CHANNEL_COUNT] = {0};
uint16_t                 g_pistonHoldDuty[PISTON_CHANNEL_COUNT] = {0};
float                    g_pistonHoldJitter[PISTON_CHANNEL_COUNT] = {0};
uint8_t                  g_pistonHoldWarn[PISTON_CHANNEL_COUNT] = {0};
float                    g_pistonHoldError[PISTON_CHANNEL_COUNT] = {0};
PistonDutyTuning         g_pistonDutyTuning[PISTON_CHANNEL_COUNT] = {};
PistonCalibData          g_pistonCalibData[PISTON_CHANNEL_COUNT] = {};
PistonRuntimeState       g_pistonRuntime[PISTON_CHANNEL_COUNT] = {};
PressureGroupState       g_pressureGroupState[2] = {};
volatile uint32_t        g_pistonCalibCmdSeq = 0;
PistonCalibCommand       g_pistonCalibCmd{};
CurrentCalibRequest      g_currentCalibReq    = {};
volatile uint32_t        g_currentCalibReqSeq = 0;
volatile bool            g_currentCalibRunning = false;

float g_n435_hall_V[3] = {0};
float g_n439_hall_V[3] = {0};
float g_n435_stroke_mm = 0.0f, g_n439_stroke_mm = 0.0f;

float g_pot1_V = 0.0f, g_pot2_V = 0.0f;

// Kavrama (K1/K2) kalibrasyon
KavramaCalibData g_kavramaCalib[2] = {};
volatile KavramaCalibState g_kavramaCalibState[2] = {KCAL_IDLE, KCAL_IDLE};
volatile uint32_t g_kavramaCalibSeq = 0;

Telemetry          g_tele{};        // TEK tanım
VescStatus         g_vescStatus{};
PumpPublic         g_pumpPub{};
PumpCommand        g_pumpCmd{ PUMP_CMD_NONE, 0, 0.0f, 0.0f, 0.0f, 0 };
Presure_Drain       g_PresureDrain{};
volatile uint32_t  g_valveDischargeSeq = 0;
ValveDischargeCommand  g_valveDischargeCmd{};

volatile bool g_diagAbortFlag = false;

volatile uint32_t g_oilCheckRequestSeq = 0;
OilCheckPublic    g_oilCheck{};

//--------------------------------------------------------------------------------------

// AUTO SHIFT globals
volatile uint32_t g_autoShiftReqSeq = 0;
AutoShiftReq      g_autoShiftReq{ false, 1, 1000 };
AutoShiftPub      g_autoShiftPub{ false, 0, 0, 1000, "" };


AutoStepDiag g_autoStepDiag{};

// -------- Hızlı Sağlık Kontrolü --------
volatile uint32_t  g_quickHealthReqSeq = 0;

// Default konfig (istersen sonra GUI’den override edeceğiz)
QuickHealthConfig  g_quickHealthCfg{
  60.0f,          // target_bar  (60 bar)
  30.0f,          // hold_s      (30 saniye)
  0.5f / 60.0f,   // maxLeak_barps = 0.5 bar/dk
  3000.0f,        // pump RPM
  20000           // fillTimeout_ms
};

// Sonuç yapısı (başta her şey sıfır / false)
QuickHealthResult  g_quickHealthRes{};


// -------- Sistem Kaçak Testi --------
volatile uint32_t g_leakReqSeq = 0;

LeakTestConfig g_leakCfg{
  10.0f,        // preDrainAbove_bar
  5.0f,         // preDrainTo_bar
  8000,         // preDrainTimeout_ms

  60.0f,        // target_bar
  20.0f,        // settle_s  (senin gözleminle uyumlu)
  10.0f,        // hold_s
  0.5f / 60.0f, // maxLeak_barps = 0.5 bar/dk

  3000.0f,      // pump RPM
  20000         // fillTimeout_ms
};

LeakTestResult g_leakRes{};

// -------- Motor/Pompa Testi --------
volatile uint32_t   g_motorTestReqSeq = 0;
MotorPumpTestConfig g_motorTestCfg{
  {500, 1000, 2000, 3000, 3500},
  5000,           // 5 sn
  0.3f,           // min dp bar/s
  2.0f,           // low current A
  30.0f           // high current A
};
MotorPumpTestResult g_motorTestRes{};
volatile uint32_t   g_valvePistonReqSeq = 0;
ValvePistonResult   g_valvePistonRes{};
volatile uint32_t   g_pistonMidReqSeq = 0;
PistonMidTestConfig g_pistonMidCfg{};
PistonMidTestResult g_pistonMidRes{};
volatile uint32_t   g_pistonGraphReqSeq = 0;
PistonGraphConfig   g_pistonGraphCfg{1800, 2400, 150, 150, 100};
PistonGraphState    g_pistonGraphState{};

// Dinamik Vites Geçiş Testi
volatile uint32_t   g_dynGearTestReqSeq = 0;

// Valf Adaptasyonu
ValveAdaptation     g_valveAdapt{};
volatile uint32_t   g_valveAdaptReqSeq = 0;


// PID Parametreleri (varsayılan değerler)
PIDParams g_pidParams = {
  .Kp = 12.0f,
  .Ki = 0.05f,
  .Kd = 3.5f,
  .maxIntegral = 200.0f,
  .deadband = 0.5f,
  .controlMs = 40,
  .holdTimeMs = 30000
};

// PID Auto-Tune
PIDTuneRequest g_pidTuneReq{};
volatile uint32_t g_pidTuneReqSeq = 0;

// TMAG5173 manyetik sensör verileri
TMAG5173_Reading g_tmagData[TMAG_CH_COUNT] = {};

// TMAG5173 piston kalibrasyonu
TMAGPistonCalib g_tmagPistonCalib[PISTON_CHANNEL_COUNT] = {};
TMAGKavramaCalib g_tmagKavramaCalib[2] = {};
volatile uint32_t g_tmagCalibSeq = 0;

// Valf temizleme (2 bağımsız kanal)
ValveCleanConfig g_valveClean = {{{false, 100}, {false, 100}}};

// DRV8243 son fault durumları (volatile - multi-task erişim için)
volatile DRV8243Status g_drvLastFault[4] = {};
volatile bool          g_drvOcpLatch     = false;  // OCP/TSD kilit: true iken tüm valfler kapalı

// Valf Diagnostik Testi
volatile uint32_t    g_valveDiagReqSeq = 0;
ValveDiagRequest     g_valveDiagReq{};
volatile bool        g_valveDiagRunning = false;

// PCV Characterization Test globals
volatile uint32_t   g_pcvCharReqSeq = 0;
PCVCharRequest      g_pcvCharReq{};
volatile bool       g_pcvCharRunning = false;
volatile bool       g_pcvCharAbortFlag = false;

// Hall Stabilite Testi
volatile uint32_t    g_hallStabilityReqSeq = 0;
HallStabilityRequest g_hallStabilityReq{};
volatile bool        g_hallStabilityRunning = false;

// -------- TMAG5173 GLOBAL OBJELER --------
TCA9548A g_mux(TCA9548A_ADDR);              // TCA9548A I2C multiplexer
TMAG5173 g_tmag[TMAG_CH_COUNT];             // 8 kanal TMAG5173 sensör
bool g_tmagOk[TMAG_CH_COUNT] = {false};       // Sensör başlatma durumları

// Üretici Modu (Manufacturer Mode) - Safety check bypass
volatile bool        g_manufacturerMode = false;

// -------- 9-FAZLI OTOMATİK TEST --------
AutoTestParams g_autoTestParams = {
    .coilMinCurrentMa       = 150.0f,
    .targetBar              = 60.0f,
    .pumpFillMaxSec         = 20.0f,
    .pumpFillTimeoutMs      = 30000,
    .pressRiseMaxBarPerSec  = 20.0f,
    .movementThreshold    = 2000,
    .leakCheckWaitMs      = 2500,
    .oilLeakMaxDrop_bar   = 30.0f,
    .oilLeakHoldSec       = 20,
    .calPwm               = 1500,
    .calTimeoutMs         = 8000,
    .holdMidTolPct        = 20.0f,
    .holdStableMs         = 2000,
    .autoShiftRepeats     = 3,
    .gearHoldMs           = 1500,
    .pumpFillTimeoutFaz9Ms = 15000,
    .leakRecheckHoldSec   = 20,
    .leakRecheckMaxDrop_bar = 5.0f,
    .adaptiveHoldEnabled  = true,
    .adaptivePwmMaxOffset = 200,
    .adaptThreshMm        = 2.0f,
};
AutoTestResult    g_autoTestResult{};
volatile uint32_t g_autoTestReqSeq  = 0;
volatile bool     g_autoTestStop    = false;
volatile bool     g_leakRecheckNeeded = false;

static Preferences s_atpPref;

void AutoTestParams_LoadNVS() {
    if (!s_atpPref.begin("atp", true)) return;
    g_autoTestParams.coilMinCurrentMa      = s_atpPref.getFloat( "coilMa",150.0f);
    g_autoTestParams.targetBar             = s_atpPref.getFloat( "tBar",  60.0f);
    g_autoTestParams.pumpFillMaxSec        = s_atpPref.getFloat( "pfMax", 20.0f);
    g_autoTestParams.pumpFillTimeoutMs     = s_atpPref.getUInt(  "pfTmo", 30000);
    g_autoTestParams.pressRiseMaxBarPerSec = s_atpPref.getFloat( "prMax", 30.0f);
    g_autoTestParams.movementThreshold    = s_atpPref.getUShort("mvThr", 2000);
    g_autoTestParams.leakCheckWaitMs      = s_atpPref.getUInt(  "lkWt",  2500);
    g_autoTestParams.oilLeakMaxDrop_bar   = s_atpPref.getFloat( "olDrp", 30.0f);
    g_autoTestParams.oilLeakHoldSec       = s_atpPref.getUInt(  "olHld", 20);
    g_autoTestParams.calPwm               = s_atpPref.getUShort("cPwm",  1500);
    g_autoTestParams.calTimeoutMs         = s_atpPref.getUInt(  "cTmo",  8000);
    g_autoTestParams.holdMidTolPct        = s_atpPref.getFloat( "hTol",  20.0f);
    g_autoTestParams.holdStableMs         = s_atpPref.getUInt(  "hStbl", 2000);
    g_autoTestParams.autoShiftRepeats     = s_atpPref.getUShort("asRep", 3);
    g_autoTestParams.gearHoldMs           = s_atpPref.getUInt(  "gHld",  1500);
    g_autoTestParams.pumpFillTimeoutFaz9Ms = s_atpPref.getUInt( "pf9Tmo", 15000);
    g_autoTestParams.leakRecheckHoldSec   = s_atpPref.getUInt(  "lrHld", 20);
    g_autoTestParams.leakRecheckMaxDrop_bar = s_atpPref.getFloat("lrDrp", 5.0f);
    g_autoTestParams.adaptiveHoldEnabled  = s_atpPref.getBool(  "adEn",  true);
    g_autoTestParams.adaptivePwmMaxOffset = s_atpPref.getUShort("adMx",  200);
    g_autoTestParams.adaptThreshMm        = s_atpPref.getFloat( "adThr", 2.0f);
    s_atpPref.end();
}

void AutoTestParams_SaveNVS() {
    if (!s_atpPref.begin("atp", false)) return;
    s_atpPref.putFloat( "coilMa",g_autoTestParams.coilMinCurrentMa);
    s_atpPref.putFloat( "tBar",  g_autoTestParams.targetBar);
    s_atpPref.putFloat( "pfMax", g_autoTestParams.pumpFillMaxSec);
    s_atpPref.putUInt(  "pfTmo", g_autoTestParams.pumpFillTimeoutMs);
    s_atpPref.putFloat( "prMax", g_autoTestParams.pressRiseMaxBarPerSec);
    s_atpPref.putUShort("mvThr", g_autoTestParams.movementThreshold);
    s_atpPref.putUInt(  "lkWt",  g_autoTestParams.leakCheckWaitMs);
    s_atpPref.putFloat( "olDrp", g_autoTestParams.oilLeakMaxDrop_bar);
    s_atpPref.putUInt(  "olHld", g_autoTestParams.oilLeakHoldSec);
    s_atpPref.putUShort("cPwm",  g_autoTestParams.calPwm);
    s_atpPref.putUInt(  "cTmo",  g_autoTestParams.calTimeoutMs);
    s_atpPref.putFloat( "hTol",  g_autoTestParams.holdMidTolPct);
    s_atpPref.putUInt(  "hStbl", g_autoTestParams.holdStableMs);
    s_atpPref.putUShort("asRep", g_autoTestParams.autoShiftRepeats);
    s_atpPref.putUInt(  "gHld",  g_autoTestParams.gearHoldMs);
    s_atpPref.putUInt(  "pf9Tmo",g_autoTestParams.pumpFillTimeoutFaz9Ms);
    s_atpPref.putUInt(  "lrHld", g_autoTestParams.leakRecheckHoldSec);
    s_atpPref.putFloat( "lrDrp", g_autoTestParams.leakRecheckMaxDrop_bar);
    s_atpPref.putBool(  "adEn",  g_autoTestParams.adaptiveHoldEnabled);
    s_atpPref.putUShort("adMx",  g_autoTestParams.adaptivePwmMaxOffset);
    s_atpPref.putFloat( "adThr", g_autoTestParams.adaptThreshMm);
    s_atpPref.end();
}

// TMAG kalibrasyon flash storage
static Preferences s_tmagCalibPref;
static bool s_tmagPrefReady = false;

static void TMAGCalib_EnsureReady() {
    if (!s_tmagPrefReady) {
        s_tmagPrefReady = s_tmagCalibPref.begin("tmag_cal", false);
    }
}

void TMAGCalib_SavePiston(uint8_t pistonIdx) {
    if (pistonIdx >= PISTON_CHANNEL_COUNT) return;
    TMAGCalib_EnsureReady();
    if (!s_tmagPrefReady) return;
    
    char key[8];
    snprintf(key, sizeof(key), "p%u", (unsigned)pistonIdx);
    s_tmagCalibPref.putBytes(key, &g_tmagPistonCalib[pistonIdx], sizeof(TMAGPistonCalib));
}

void TMAGCalib_SaveKavrama(uint8_t kavramaIdx) {
    if (kavramaIdx >= 2) return;
    TMAGCalib_EnsureReady();
    if (!s_tmagPrefReady) return;
    
    char key[8];
    snprintf(key, sizeof(key), "k%u", (unsigned)kavramaIdx);
    s_tmagCalibPref.putBytes(key, &g_tmagKavramaCalib[kavramaIdx], sizeof(TMAGKavramaCalib));
}

void TMAGCalib_LoadAll() {
    TMAGCalib_EnsureReady();
    if (!s_tmagPrefReady) return;
    
    char key[8];
    // Pistonlar
    for (uint8_t i = 0; i < PISTON_CHANNEL_COUNT; i++) {
        snprintf(key, sizeof(key), "p%u", (unsigned)i);
        size_t n = s_tmagCalibPref.getBytes(key, &g_tmagPistonCalib[i], sizeof(TMAGPistonCalib));
        if (n != sizeof(TMAGPistonCalib)) {
            g_tmagPistonCalib[i] = {};  // Veri yoksa sıfırla
        }
    }
    // Kavramalar
    for (uint8_t i = 0; i < 2; i++) {
        snprintf(key, sizeof(key), "k%u", (unsigned)i);
        size_t n = s_tmagCalibPref.getBytes(key, &g_tmagKavramaCalib[i], sizeof(TMAGKavramaCalib));
        if (n != sizeof(TMAGKavramaCalib)) {
            g_tmagKavramaCalib[i] = {};  // Veri yoksa sıfırla
        }
    }
}

void Shared_Init() {
  if (!g_sharedMutex) g_sharedMutex = xSemaphoreCreateMutex();
  
  // TMAG kalibrasyonunu flash'tan yükle
  TMAGCalib_LoadAll();
  
  // AutoTest parametrelerini NVS'den yükle
  AutoTestParams_LoadNVS();
  
  for (int i = 0; i < PISTON_CHANNEL_COUNT; ++i) {
    if (g_pistonCalibData[i].p_bins[0] == 0.0f && g_pistonCalibData[i].p_bins[1] == 0.0f) {
      g_pistonCalibData[i].p_bins[0] = 30.0f;
      g_pistonCalibData[i].p_bins[1] = 40.0f;
      g_pistonCalibData[i].p_bins[2] = 50.0f;
    }
    g_pistonRuntime[i].x_ref = 0.5f;
    g_pistonRuntime[i].state = PistonRuntimeState::IDLE;
  }
}

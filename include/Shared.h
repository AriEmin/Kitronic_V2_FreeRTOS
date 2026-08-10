#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// -------- I2C Module Class Declarations --------
class TCA9548A;
class TMAG5173;

// -------- Firmware Versiyon --------
#define FW_VERSION_MAJOR 1
#define FW_VERSION_MINOR 4
#define FW_VERSION_PATCH 1
#define FW_VERSION_STR "1.4.1"

// -------- RTOS Objeleri --------
extern SemaphoreHandle_t  g_sharedMutex;
extern SemaphoreHandle_t  g_i2cMutex;      // Global I2C bus mutex
extern portMUX_TYPE       g_portMux;

// -------- PUMP/PWR paylaşılanlar --------

extern float              g_pumpRunCurrent_A;   // Çalışma akımı
extern float              g_pumpRunRpm;         // Çalışma rpm hedefi

extern float              g_mainPwrVoltage_V, g_mainPwrCurrent_A;
extern float              g_vescPwrVoltage_V, g_vescPwrCurrent_A;

// SSR (ısıtıcı) isteği
extern uint8_t            g_ssrDesired;
extern uint16_t           g_heater_on_ms;    // ON süresi (ms), 0 = kesintisiz mod
extern uint16_t           g_heater_off_ms;   // OFF süresi (ms)
extern float              g_heater_setpoint;  // Hedef sıcaklık (°C), 0 = devre dışı

// -------- Valf ölçü/duty durumları --------
extern float              g_valveCurrent_mA[8];
extern float              g_valveBusVoltage_V[8];
extern uint16_t           g_valveDutyCounts[8];
extern uint16_t           g_valveTargetDuty[8];
extern float              g_valveCustomCurrent_mA[8]; // Per-valf özel akım hedefi (0=devre dışı)
extern uint8_t            g_valveCustomMode[8];   // 0=off,1=open,2=close,3=open_slow,4=close_slow,5=pcv

// -------- ADS/Analog türevleri (TaskADSMonitor kullandıkları) --------
extern float g_pressure0_V, g_pressure1_V;
extern float g_temp1_C, g_temp2_C;
extern float g_piston_1_3_mm, g_piston_5_7_mm, g_piston_2_4_mm, g_piston_6_R_mm;
extern float g_v_1_3_mms, g_v_5_7_mms,g_v_2_4_mms,g_v_6_R_mms,g_v_K1_mms,g_v_K2_mms;

// --- DRV5055 ham verileri / kalibrasyon ---
enum PistonChannel : uint8_t {
  PISTON_5_7 = 0,
  PISTON_1_3 = 1,
  PISTON_2_4 = 2,
  PISTON_6_R = 3,
  PISTON_K1  = 4,  // N435 - K1 kavrama
  PISTON_K2  = 5,  // N439 - K2 kavrama
  PISTON_CHANNEL_COUNT = 6
};

static constexpr uint16_t PISTON_CAL_TABLE_POINTS = 96;
static constexpr float    PISTON_DEFAULT_STROKE_MM = 26.0f;
static constexpr float    PISTON_MANUAL_STROKE_MM  = 28.0f;

typedef struct {
  bool     valid;
  uint16_t numPoints;
  float    stroke_mm;
  float    raw[PISTON_CAL_TABLE_POINTS];
  float    mm[PISTON_CAL_TABLE_POINTS];
} PistonCalibrationTable;

// TMAG5173 tabanlı piston kalibrasyonu (Z ekseni)
typedef struct {
  bool     valid;
  int16_t  zMin;        // Kapalı pozisyon Z değeri (0mm) - sensör 1
  int16_t  zMax;        // Açık pozisyon Z değeri (26mm) - sensör 1
  int16_t  zMid;        // Orta pozisyon Z değeri (13mm) - manuel ayarlanan
  float    strokeMm;    // Strok mesafesi (mm) - toplam 26mm
  // İkinci sensör (K1/K2 kavrama pistonları için - sensörler 10mm aralıklı)
  int16_t  zMin2;       // Kapalı pozisyon Z değeri - sensör 2
  int16_t  zMax2;       // Açık pozisyon Z değeri - sensör 2
  bool     hasSensor2;  // İkinci sensör kalibre edildi mi?
} TMAGPistonCalib;

// Kavrama TMAG kalibrasyonu (2 sensör: açık ve kapalı)
typedef struct {
  bool     valid;
  int16_t  sensor1_closed;  // Sensör 1 kapalı pozisyon
  int16_t  sensor1_open;    // Sensör 1 açık pozisyon
  int16_t  sensor2_closed;  // Sensör 2 kapalı pozisyon
  int16_t  sensor2_open;    // Sensör 2 açık pozisyon
  float    strokeMm;
} TMAGKavramaCalib;

extern TMAGPistonCalib  g_tmagPistonCalib[PISTON_CHANNEL_COUNT];
extern TMAGKavramaCalib g_tmagKavramaCalib[2];  // K1, K2
extern volatile uint32_t g_tmagCalibSeq;

// TMAG kalibrasyon kaydetme/yükleme
void TMAGCalib_SavePiston(uint8_t pistonIdx);
void TMAGCalib_SaveKavrama(uint8_t kavramaIdx);
void TMAGCalib_LoadAll();

typedef struct {
  uint8_t  piston;           // PistonChannel
  uint16_t pwmDuty;          // PWM duty (0-4095)
  uint16_t samplePeriodMs;   // örnekleme periyodu
  uint16_t settleMs;         // başlangıç bekleme
  uint16_t maxSamples;       // maksimum kayıt sayısı
  bool     start;            // true->başlat, false->durdur
  float    pressureTargetBar;// kalibrasyon öncesi basınç hedefi
  bool     findHold;         // true->hold PWM de bul
  bool     calibrateAll;     // true->tüm pistonları sırayla kalibre et
} PistonCalibrationRequest;

enum PistonRefState : uint8_t {
  PISTON_REF_CLOSED = 0,
  PISTON_REF_MID    = 1,
  PISTON_REF_OPEN   = 2
};

typedef struct {
  uint8_t piston;
  uint8_t state;
  float   rawValue;
} PistonReferenceRequest;

typedef struct {
  float   raw[3];
  uint8_t validMask;
} PistonManualReference;

typedef struct {
  uint8_t piston;
  uint8_t state;
  float   tolerance;
  bool    enable;
} PistonHoldRequest;

extern float                     g_pistonHallRaw[PISTON_CHANNEL_COUNT];
extern float                     g_pistonHallmm[6];
extern PistonCalibrationTable    g_pistonCalTable[PISTON_CHANNEL_COUNT];
extern volatile uint32_t         g_pistonCalSeq;
extern volatile uint32_t         g_pistonCalReqSeq;
extern PistonCalibrationRequest  g_pistonCalReq;
// Manuel kalibrasyon ilerleme durumu (telemetry S mesajina yansitilir)
extern volatile uint8_t          g_pistonCalRunning;
extern volatile uint8_t          g_pistonCalPhase;    // 0=Bekle, 1=Basinc, 2..7=P0..P5, 8=Hesapla, 9=Tamam
extern volatile uint32_t         g_pistonRefReqSeq;
extern PistonReferenceRequest    g_pistonRefReq;
extern PistonManualReference     g_pistonManualRef[PISTON_CHANNEL_COUNT];
extern volatile uint32_t         g_pistonHoldReqSeq[PISTON_CHANNEL_COUNT];
extern PistonHoldRequest         g_pistonHoldReq[PISTON_CHANNEL_COUNT];
extern uint8_t                   g_pistonState[PISTON_CHANNEL_COUNT];
extern uint16_t                  g_pistonHoldDuty[PISTON_CHANNEL_COUNT];
extern float                     g_pistonHoldJitter[PISTON_CHANNEL_COUNT];
extern uint8_t                   g_pistonHoldWarn[PISTON_CHANNEL_COUNT];
extern float                     g_pistonHoldError[PISTON_CHANNEL_COUNT];
typedef struct {
  uint16_t openDuty;
  uint16_t closeDuty;
  uint16_t holdDuty;
  bool     valid;
} PistonDutyTuning;
extern PistonDutyTuning          g_pistonDutyTuning[PISTON_CHANNEL_COUNT];

// --- Yeni piston kalibrasyon/veri modelleri ---
static constexpr size_t PISTON_FF_BINS = 3;

struct PistonCalibData {
  bool     calibrated;
  uint16_t version;
  uint16_t min_raw;
  uint16_t max_raw;
  uint16_t mid_raw;
  int8_t   direction;       // +1: duty artinca x artar
  uint16_t duty_breakaway;
  uint16_t duty_min;
  uint16_t duty_max;
  uint16_t duty_hold;       // Ortada tutma PWM (find_hold_dither ile bulunur)
  uint16_t duty_open_thresh;  // Açılma başlangıç PWM'i (bu üstünde açılır)
  uint16_t duty_close_thresh; // Kapanma başlangıç PWM'i (bu altında kapanır)
  float    u_ff_map[PISTON_FF_BINS];
  float    p_bins[PISTON_FF_BINS];
  float    hold_mA;          // holdcontrol_V2: denge akimi (0=kalibre edilmemis)
  float    open_mA;          // holdcontrol_V2: acma esik akimi (MAX_OPEN_CURR)
  float    close_mA;         // holdcontrol_V2: kapama esik akimi (MAX_CLOS_CURR)
  uint32_t crc;
};

struct PistonRuntimeState {
  enum State : uint8_t { IDLE = 0, CALIB, MOVE_FAST, MOVE_SLOW, HOLD } state;
  bool      hold_mid_enable;
  bool      hold_init_needed;  // hold yeniden aktifleşince state makinesini INIT'e sıfırlar
  float     x_ref;
  float     x_filt;
  float     v_est;
  float     e;
  uint16_t  u_cmd;
  uint32_t  state_ts_ms;
};

struct PressureGroupState {
  float    p_meas;
  float    p_ref;
  uint16_t cmd;
  float    integ;
};

struct PistonCalibCommand {
  enum Action : uint8_t { NONE = 0, START_ONE, START_ALL, CLEAR_ONE, CLEAR_ALL, STATUS, STOP, FIND_HOLD } action;
  uint8_t piston;  // PistonChannel
};

extern PistonCalibData      g_pistonCalibData[PISTON_CHANNEL_COUNT];
extern PistonRuntimeState   g_pistonRuntime[PISTON_CHANNEL_COUNT];
extern PressureGroupState   g_pressureGroupState[2];
extern volatile uint32_t    g_pistonCalibCmdSeq;
extern PistonCalibCommand   g_pistonCalibCmd;

// --- Akım Kalibrasyon (holdcontrol_V2 için) ---
struct CurrentCalibRequest {
  bool    abort;     // true: devam eden kalibrasyonu durdur
  uint8_t piston;   // 0-3: kalibre edilecek piston
  bool    all;      // true: tum pistonlari kalibre et (piston=0'dan basla)
};
extern CurrentCalibRequest  g_currentCalibReq;
extern volatile uint32_t    g_currentCalibReqSeq;
extern volatile bool        g_currentCalibRunning;

extern float g_n435_hall_V[3];
extern float g_n439_hall_V[3];
extern float g_n435_stroke_mm, g_n439_stroke_mm;

extern float g_pot1_V, g_pot2_V;

// -------- Kavrama (K1/K2) Kalibrasyon --------
enum KavramaCalibState : uint8_t {
    KCAL_IDLE = 0,
    KCAL_WAIT_CLOSED,      // Kapalı pozisyon bekle
    KCAL_SAMPLE_CLOSED,    // Kapalı pozisyon örnekle
    KCAL_OPENING,          // Açılıyor
    KCAL_WAIT_OPEN,        // Açık pozisyon bekle
    KCAL_SAMPLE_OPEN,      // Açık pozisyon örnekle
    KCAL_DONE,             // Tamamlandı
    KCAL_FAILED            // Başarısız
};

struct KavramaCalibData {
    bool valid;
    float closedV[3];      // Kapalı pozisyondaki 3 sensör voltajı
    float openV[3];        // Açık pozisyondaki 3 sensör voltajı
    float strokeMm;        // Toplam strok (30mm)
};

extern KavramaCalibData g_kavramaCalib[2];  // [0]=K1, [1]=K2
extern volatile KavramaCalibState g_kavramaCalibState[2];
extern volatile uint32_t g_kavramaCalibSeq;

// -------- PUMP Komutları (tek kopya) --------
enum PumpCmd {
  PUMP_CMD_NONE = 0,
  PUMP_CMD_START,
  PUMP_CMD_AUTO,
  PUMP_CMD_STOP,
  PUMP_CMD_SET_CURR,
  PUMP_CMD_SET_RPM,
  PUMP_CMD_FILL,
  PUMP_CMD_DRAIN
};
typedef struct {
  PumpCmd   cmd;
  uint32_t  seq;
  float     setCurrentA;
  float     setRpm;
  float     fillCurrentA;
  uint32_t  fillDurationMs;
} PumpCommand;

typedef struct {
  uint16_t pcvDuty;
  uint16_t pairDuty;
  float    targetBar;
  uint32_t timeoutMs;
} ValveDischargeCommand;

typedef struct {
  bool     name; 
  uint32_t  Data;  
} Presure_Drain;

// -------- Telemetri snapshot (INA226 + INA219) --------
typedef struct {
  float mainV, mainI;
  float vescV, vescI;
  float inaV[8];     // 8 kanal INA219 bus V
  float inaI_mA[8];  // 8 kanal INA219 current mA
} Telemetry;

// -------- VESC ve Pompa halka açık durumları --------
typedef struct {
  int32_t rpm;
  float   duty;
  float   Im;      // motor akımı
  float   Iin;     // giriş akımı
  float   Tfet;
  float   Tmot;
  float   vin;
  int32_t tacho;
} VescStatus;


typedef struct {
  uint8_t mode;   // PumpMode veya basit int
  float   Icmd;
  float   rpmCmd;
  float   rpm;
  float   bar;
} PumpPublic;

// --- Global değişken BİLDİRİMLERİ (yalnızca extern!) ---
extern Telemetry          g_tele;
extern VescStatus         g_vescStatus;
extern PumpPublic         g_pumpPub;
extern PumpCommand        g_pumpCmd;
extern Presure_Drain      g_PresureDrain;
extern volatile uint32_t  g_valveDischargeSeq;
extern ValveDischargeCommand  g_valveDischargeCmd;

// Diag/test stop-all bayrağı
extern volatile bool g_diagAbortFlag;

// -------- OIL CHECK --------
typedef struct {
  bool     running;
  bool     present;             // Yağ tespit edildi mi
  float    p0_bar;              // başlangıç
  float    p1_bar;              // bitiş
  float    dp_bar;              // toplam artış
  uint32_t t_ms;                // toplam süre
  float    i_avg_A;             // ortalama pompa akımı
  float    i_rms_A;             // RMS pompa akımı
  float    dp_rate_barps;       // dP/dt (bar/s)
  float    leak_bar_per_s;      // durduktan sonraki düşüş hızı
  uint8_t  stage;               // 1:LOW_PRIME, 2:HIGH_PRIME, 3:DONE
  char     level_text[12];      // "EMPTY/LOW/OK"
  char     reason[24];          // "NO_DP_HIGH_I" vb.
} OilCheckPublic;

extern volatile uint32_t g_oilCheckRequestSeq;
extern OilCheckPublic    g_oilCheck;


// --- AUTO SHIFT (Yeni) ---
typedef struct {
  bool     start;
  uint16_t repeats;
  uint32_t gear_ms;
} AutoShiftReq;

typedef struct {
  bool     running;
  uint16_t step_idx;
  uint16_t repeat_idx;
  uint32_t gear_ms;
  char     step_name[8];
} AutoShiftPub;

extern volatile uint32_t g_autoShiftReqSeq;
extern AutoShiftReq      g_autoShiftReq;
extern AutoShiftPub      g_autoShiftPub;


typedef struct {
  // kimlik
  char     step[8];
  uint16_t activeMask;     // adımda aktif (OPEN/MID) sayılan valflerin maskesi

  // zaman
  uint32_t t_start_ms, t_end_ms, expect_ms;

  // basınç
  float P_pre, P_post, P_peak, dP;

  // enerji/besleme
  float Vbus_min;

  // pompa
  float I_pump_peak, I_pump_ss;

  // bobin akımları (mA)
  float I_coil_peak_mA[8];
  float I_coil_ss_mA[8];

  // bayraklar
  uint16_t faults;
} AutoStepDiag;

// global son ölçüm (TaskSerial için)
extern AutoStepDiag g_autoStepDiag;

void AutoDiag_End(uint32_t expect_ms, AutoStepDiag& out);

// -------- Hızlı Sağlık Kontrolü --------

// Sonuç tarafı (GUI buradan okuyacak)
struct QuickHealthResult {
  bool     running;      // test şu an çalışıyor mu
  bool     done;         // test bitti mi
  bool     pass;         // GENEL sonuç (tüm alt testler dahil)

  float    p0_bar;       // hold başlangıç basıncı
  float    p1_bar;       // hold bitiş basıncı
  float    dp_bar;       // fark (p1 - p0)
  float    leak_barps;   // bar/s (negatif beklenir)

  uint32_t t_fill_ms;    // hedef basınca varma süresi
  uint32_t t_hold_ms;    // leak test süresi

  // --- Alt birim durumları ---
  bool     valves_ok;        // 8 valfin bobin/INA kontrolü OK mi
  bool     pump_ok;          // pompa / motor start guard OK mi
  bool     pressure_ok;      // basınç ölçümü / sensör OK mi
  bool     pistons_ok;       // piston hall sensörleri OK mi

  uint16_t valve_open_mask;  // bit=1 -> düşük akım (open şüphesi)
  uint16_t valve_short_mask; // bit=1 -> çok yüksek akım (short şüphesi)
  uint8_t  piston_err_mask;  // bit=1 -> ilgili piston sensör arızası
  uint32_t piston_diag;      // debug: bit alanı (alt nibble: moved, üst nibble: returned)

  char     reason[16];       // "OK", "VALVE", "NO_PUMP", "LEAK", "SENSOR" vs.
};

// Konfigürasyon tarafı (GUI’den ayarlanabilir)
struct QuickHealthConfig {
  float    target_bar;      // doldurulacak hedef basınç
  float    hold_s;          // bekleme süresi [s]
  float    maxLeak_barps;   // izin verilen max kaçak [bar/s]
  float    pumpRpm;         // doldurma pompa hızı [rpm]
  uint32_t fillTimeout_ms;  // hedefe varış için timeout
};

extern volatile uint32_t  g_quickHealthReqSeq;
extern QuickHealthConfig  g_quickHealthCfg;
extern QuickHealthResult  g_quickHealthRes;

// -------- Yeni test yapıları: Sistem Kaçak Testi --------

struct LeakTestResult {
  bool     running;
  bool     done;
  bool     pass;

  float    p_start_bar;   // test başlangıcı
  float    p_fill_bar;    // target'a çıkış sonu
  float    p_base_bar;    // settle sonrası referans
  float    p_end_bar;     // hold sonu

  float    dp_bar;        // p_end - p_base
  float    leak_barps;    // bar/s (negatif beklenir)

  uint32_t t_fill_ms;
  uint32_t t_settle_ms;
  uint32_t t_hold_ms;

  char     reason[16];    // "OK","LEAK","NO_P","SENSOR","DRAIN_TMO"
};

struct LeakTestConfig {
  float    preDrainAbove_bar;   // başlangıçta bunun üstündeyse drain
  float    preDrainTo_bar;
  uint32_t preDrainTimeout_ms;

  float    target_bar;
  float    settle_s;            // "ilk gevşeme/oturma" süresi
  float    hold_s;              // gerçek kaçak ölçüm süresi
  float    maxLeak_barps;       // izin verilen max kaçak [bar/s]

  float    pumpRpm;             // doldurma pompa hızı [rpm]
  uint32_t fillTimeout_ms;
};

extern volatile uint32_t g_leakReqSeq;
extern LeakTestConfig    g_leakCfg;
extern LeakTestResult    g_leakRes;

// -------- Motor/Pompa Testi --------
struct MotorPumpTestConfig {
  float    rpms[5];        // test rpm listesi
  uint32_t duration_ms;    // her adım süresi
  float    min_dp_barps;   // minimum beklenen basınç artış hızı (bar/s)
  float    low_I_A;        // "çok düşük akım" eşiği
  float    high_I_A;       // "yüksek akım" eşiği
};

struct MotorPumpTestResult {
  bool     running;
  bool     done;
  bool     pass;
  float    rpm[5];
  float    dp_barps[5];
  float    I_avg[5];
  uint8_t  fail_mask;    // bit=i -> o adım başarısız
  char     reason[32];
};

extern volatile uint32_t   g_motorTestReqSeq;
extern MotorPumpTestConfig g_motorTestCfg;
extern MotorPumpTestResult g_motorTestRes;

// -------- Valf/Piston Testi --------
struct ValveResult {
  bool  open_fault;   // akım yok = open
  bool  short_fault;  // yüksek akım
  float R_est;        // ohm
  float inrush_mA;
  float hold_mA;
  float duty_prof;    // duty-akım profil hatası
};

struct PistonResult {
  float hall_raw;
  bool  moved;
  bool  err;
};

struct ValvePistonResult {
  bool   running;
  bool   done;
  bool   pass;
  ValveResult  valves[8];
  PistonResult pistons[6];
  char   reason[32];
};

extern volatile uint32_t   g_valvePistonReqSeq;
extern ValvePistonResult   g_valvePistonRes;

// -------- Piston Orta (Mid) Testi --------
struct PistonMidTestConfig {
  uint32_t duration_ms;    // her piston için tutma süresi
  float    drop_thresh_bar;// izin verilen max basınç düşüşü
};

struct PistonMidTestResult {
  bool     running;
  bool     done;
  bool     pass;
  uint8_t  fail_mask;      // bit=i -> piston i mid tutamadı
  float    max_drop_bar;
  uint8_t  current_piston; // running ise aktif piston
  uint32_t remaining_ms;   // tahmini kalan süre
  char     reason[24];
};

extern volatile uint32_t    g_pistonMidReqSeq;
extern PistonMidTestConfig  g_pistonMidCfg;
extern PistonMidTestResult  g_pistonMidRes;

// -------- Piston Grafik Testi --------
struct PistonGraphConfig {
  uint16_t duty_start;   // baslangic PWM
  uint16_t duty_end;     // hedef PWM
  uint16_t duty_step;    // adim
  uint16_t step_ms;      // her adim bekleme
  uint16_t sample_ms;    // telemetri ornekleme
};
struct PistonGraphState {
  bool     running;
  uint8_t  current_piston;
};
extern volatile uint32_t    g_pistonGraphReqSeq;
extern PistonGraphConfig    g_pistonGraphCfg;
extern PistonGraphState     g_pistonGraphState;

// -------- Dinamik Vites Geçiş Testi --------
extern volatile uint32_t    g_dynGearTestReqSeq;

// -------- Valf Adaptasyonu --------
struct ValveAdaptation {
  uint16_t openPWM[8];        // Açma touch point PWM
  uint16_t closePWM[8];       // Kapatma touch point PWM
  uint16_t holdMidPWM[8];     // Orta konumda tutma PWM
  uint16_t holdOpenPWM[8];    // Açık konumda tutma PWM
  uint16_t responseOpen_ms[8];  // Açma tepki süresi
  uint16_t responseClose_ms[8]; // Kapatma tepki süresi
  float    minPos_mm[8];      // Minimum konum (kapalı)
  float    midPos_mm[8];      // Orta konum
  float    maxPos_mm[8];      // Maximum konum (açık)
  float    travelMM[8];       // Toplam strok
  bool     calibrated;        // Kalibrasyon yapıldı mı
  uint32_t timestamp;         // Son kalibrasyon zamanı (epoch)
};
extern ValveAdaptation       g_valveAdapt;
extern volatile uint32_t     g_valveAdaptReqSeq;

// PID Parametreleri (GUI'den değiştirilebilir)
struct PIDParams {
  float Kp;           // Proportional gain
  float Ki;           // Integral gain
  float Kd;           // Derivative gain
  float maxIntegral;  // Anti-windup limit
  float deadband;     // Deadband (mm)
  uint16_t controlMs; // Kontrol periyodu (ms)
  uint32_t holdTimeMs;// Tutma süresi (ms)
};
extern PIDParams g_pidParams;

// PID Auto-Tune Request
struct PIDTuneRequest {
  uint8_t piston;
  float targetPos;
  bool active;
};
extern PIDTuneRequest g_pidTuneReq;
extern volatile uint32_t g_pidTuneReqSeq;

// -------- TMAG5173-Q1 Manyetik Sensörler --------
// TCA9548A üzerinden 8 kanal:
// ch0: Hall 1/3, ch1: Hall 5/7, ch2: Hall 2/4, ch3: Hall 6/R
// ch4: K1-1 (açık), ch5: K1-2 (kapalı), ch6: K2-1 (açık), ch7: K2-2 (kapalı)

struct TMAG5173_Reading {
    int16_t x;      // Raw X
    int16_t y;      // Raw Y
    int16_t z;      // Raw Z
    bool    valid;  // Sensör erişilebilir mi
};

// 8 kanal TMAG5173 okuma
extern TMAG5173_Reading g_tmagData[8];

// Kanal isimleri için enum
enum TMAGChannel : uint8_t {
    TMAG_CH_1_3  = 0,   // Piston 1/3 (vites)
    TMAG_CH_5_7  = 1,   // Piston 5/7 (vites)
    TMAG_CH_2_4  = 2,   // Piston 2/4 (vites)
    TMAG_CH_6_R  = 3,   // Piston 6/R (vites)
    TMAG_CH_K1_1 = 4,   // K1 sensör 1 (açık pozisyon)
    TMAG_CH_K1_2 = 5,   // K1 sensör 2 (kapalı pozisyon)
    TMAG_CH_K2_1 = 6,   // K2 sensör 1 (açık pozisyon)
    TMAG_CH_K2_2 = 7,   // K2 sensör 2 (kapalı pozisyon)
    TMAG_CH_COUNT = 8
};

// -------- VALF TEMİZLEME --------
struct ValveCleanChannel {
    volatile bool     active;        // Kanal aktif mi
    volatile uint16_t period_ms;     // Puls periyodu (100-2000ms)
};
struct ValveCleanConfig {
    ValveCleanChannel ch[2];  // 2 bağımsız kanal
};
extern ValveCleanConfig g_valveClean;

// -------- DRV8243 DURUM --------
struct DRV8243Status {
    volatile uint8_t st1;    // FAULT_SUMMARY (0x01)
    volatile uint8_t st2;    // STATUS1 (0x02) - OLA1/OLA2/OCP bitleri
    volatile uint8_t flt;    // STATUS2 (0x03)
    volatile bool    ok;     // Hata yok mu?
};
extern volatile DRV8243Status g_drvLastFault[4];  // Son fault durumları (telemetri için)
extern volatile bool g_drvOcpLatch;               // OCP/TSD kilit mod — tum valfler kapali
void DRV_GetAllStatus(DRV8243Status status[4]);
void DRV_FaultClear(int idx);
void DRV_EnableAll(bool en);
void DRV_PresetAll();  // FaultClear + ApplyPreset tüm DRV'ler için

// -------- VALF DİAGNOSTİK TESTİ --------
struct ValveDiagRequest {
    uint8_t  valveIdx;         // PWM index (0-7)
    uint16_t dutyMax;          // Rampa hedefi (varsayılan 2000)
    uint16_t dutyStep;         // Artış adımı (varsayılan 10)
    uint16_t stepMs;           // Adım periyodu (varsayılan 100ms)
    float    pressureTarget;   // Test öncesi basınç hedefi (bar)
    bool     start;            // true=başlat, false=durdur
};

extern volatile uint32_t    g_valveDiagReqSeq;
extern ValveDiagRequest     g_valveDiagReq;
extern volatile bool        g_valveDiagRunning;  // true iken normal telemetri durur

// -------- PCV Characterization Test --------
static constexpr uint8_t PCVCHAR_MAX_STEPS = 16;

struct PCVCharRequest {
  bool     start;              // true: başlat
  bool     abort;              // true: iptal
  uint8_t  pcvValveIdx;        // 1=N436, 5=N440
  uint8_t  pistonIdx;          // 0..3 (P0..P3)
  uint8_t  stepCount;          // pcv_currents_mA kaç adet
  uint16_t pcv_mA[PCVCHAR_MAX_STEPS]; // PCV akım listesi (mA)
  uint16_t open_mA;            // vites valfi açma akımı (mA)
  uint16_t close_mA;           // vites valfi kapama akımı (mA)
  float    mid_target_mm;      // örn: 13.0
  float    closed_thresh_mm;   // kapalı kabul eşiği (mm)
  float    full_open_thresh_mm;// tam açık kabul eşiği (mm)
  float    move_detect_mm;     // hareket algılama eşiği (mm)
  float    pressure_ready_bar; // başlangıç basınç eşiği (bar)
  float    pressure_min_bar;   // step sırasında minimum bar
  uint16_t settle_before_step_ms;
  uint16_t max_open_ms;
  uint16_t brake_observe_ms;
  uint16_t max_close_ms;
  uint8_t  repeats;            // her adımı kaç kez tekrarla
  // Per-step pressure equalization
  bool     wait_pressure_each_step;       // true: her step öncesi basınç hazırlığı yap
  float    step_ready_pressure_bar;       // hedef basınç (örn: 58.0)
  float    step_ready_pressure_window_bar;// kabul penceresi (örn: 2.0 → 56-60 bar)
  uint32_t step_pressure_wait_timeout_ms; // maksimum bekleme süresi (ms)
};

extern volatile uint32_t   g_pcvCharReqSeq;
extern PCVCharRequest      g_pcvCharReq;
extern volatile bool       g_pcvCharRunning;   // true iken normal telemetri durur
extern volatile bool       g_pcvCharAbortFlag; // acil durdurma

// -------- HALL STABILITE TESTI --------
struct HallStabilityRequest {
    uint8_t  pistonIdx;          // Piston index (0-5: P1-3, P5-7, P2-4, P6-R, K1, K2)
    uint16_t durationMs;         // Test süresi (ms)
    uint8_t  sampleIntervalMs;   // Örnekleme aralığı (ms)
    uint8_t  testType;           // 0=static, 1=emi, 2=dynamic
    bool     start;              // true=başlat, false=durdur
};

extern volatile uint32_t    g_hallStabilityReqSeq;
extern HallStabilityRequest g_hallStabilityReq;
extern volatile bool        g_hallStabilityRunning;  // true iken test çalışıyor

// Hall test sonuç paketi (her örnek için)
struct HallSample {
    uint32_t timestampMs;        // Test başlangıcından itibaren (ms)
    int16_t  rawValue;           // TMAG Z ham değer
    float    mmValue;            // Hesaplanan mm pozisyonu
};

// -------- TMAG5173 GLOBAL OBJELER --------
// TaskTMAG5173.cpp ve TaskHallTest.cpp tarafından paylaşılır
extern TCA9548A g_mux;               // TCA9548A I2C multiplexer
extern TMAG5173 g_tmag[TMAG_CH_COUNT];  // 8 kanal TMAG5173 sensör
extern bool g_tmagOk[TMAG_CH_COUNT];    // Sensör başlatma durumları

// -------- HOLD KALIBRASYONU --------
// TaskHoldCalibration.cpp'den çağrılır (TaskSerial.cpp kullanır)
bool startHoldCalibration(uint8_t pistonIdx);
void stopHoldCalibration();

// -------- ÜRETİCİ MODU (Manufacturer Mode) --------
// Safety check'leri bypass etmek için - kullanıcı GUI'den aktif edebilir
// true iken: min basınç kontrolü, valf sıralama kontrolü vb. atlanır
extern volatile bool g_manufacturerMode;

// -------- 9-FAZLI OTOMATİK TEST --------

struct AutoTestParams {
    // Faz 0: Elektriksel valf kontrolü
    float    coilMinCurrentMa;       // Min kabul edilen bobin akımı (mA) varsayılan: 150
                                     // GUI'deki "Faz 0 - Elektriksel Valf Kontrolü" eşiğiyle aynı
                                     // kaynaktan (config.json) beslenir; tek eşik burada kullanılır.

    // Faz 1: Pompa doldurma
    float    targetBar;              // Hedef basınç (bar)          varsayılan: 60
    float    pumpFillMaxSec;         // Rapor eşiği - üstünde arıza (sn) varsayılan: 20
    uint32_t pumpFillTimeoutMs;      // Max doldurma süresi (ms)    varsayılan: 30000
    float    pressRiseMaxBarPerSec;  // Max anlık basınç artış hızı (bar/sn) varsayılan: 20.0
                                     // Bu üstü = basınç tüpü (akümülatör) arızası

    // Faz 2-5: Kaçak testleri
    uint16_t movementThreshold;      // Hareket eşiği (hall birimi) varsayılan: 2000
    uint32_t leakCheckWaitMs;        // Valf açtıktan sonra bekleme varsayılan: 2500

    // Faz 6: Yağ kaçak testi
    float    oilLeakMaxDrop_bar;     // Max izin verilen düşüş (bar) varsayılan: 30
    uint32_t oilLeakHoldSec;         // Basınç tutma süresi (sn)    varsayılan: 20

    // Faz 7-8: Kalibrasyon
    uint16_t calPwm;                 // Kalibrasyon PWM             varsayılan: 1500
    uint32_t calTimeoutMs;           // Tek piston timeout (ms)     varsayılan: 8000
    float    holdMidTolPct;          // Hold toleransı %            varsayılan: 20.0
    uint32_t holdStableMs;           // Stabil kalma süresi (ms)    varsayılan: 2000

    // Faz 9: Otomatik vites testi
    uint16_t autoShiftRepeats;       // Vites döngüsü tekrar sayısı varsayılan: 3
    uint32_t gearHoldMs;             // Her viteste bekleme (ms)    varsayılan: 1500
    uint32_t pumpFillTimeoutFaz9Ms;  // Faz 9 içi pompa timeout (ms) varsayılan: 15000

    // Faz 10: Şartlı Kaçak Yeniden Testi (Faz 9'da pompa kaçak tetiklenirse)
    uint32_t leakRecheckHoldSec;     // Basınç tutma süresi (sn)    varsayılan: 20
    float    leakRecheckMaxDrop_bar; // Max izin verilen düşüş (bar) varsayılan: 5.0

    // Adaptif hold PWM
    bool     adaptiveHoldEnabled;    // Adaptif hold aktif mi       varsayılan: true
    uint16_t adaptivePwmMaxOffset;   // Max PWM adaptasyon ofseti   varsayılan: 200
    float    adaptThreshMm;          // Adaptasyon sapma eşiği (mm) varsayılan: 2.0
};

struct AutoTestPhaseResult {
    bool    done;
    bool    pass;
    char    detail[80];   // "OK" veya hata açıklaması
    float   measured;     // Faza göre: sn / bar / mm
    uint8_t faultMask;    // bit=i → piston/valf i arızalı (faz 2-5,7,8)
};

struct AutoTestResult {
    bool     running;
    bool     done;
    bool     pass;
    uint8_t  currentPhase;        // 0-8 (şu an çalışan faz indeksi)
    uint32_t startMs;
    uint32_t endMs;
    AutoTestPhaseResult phases[11]; // [0]=Faz0(Elektriksel) [1]=Faz1 ... [9]=Faz9 [10]=Faz10(Şartlı Kaçak)
};

enum AutoTestPhase : uint8_t {
    ATP_IDLE = 0,
    ATP_P1_PUMP_FILL,      // Faz 1: Pompa doldurma
    ATP_P2_LEAK_A1_V,      // Faz 2: Alan-1 vites valfleri kaçak
    ATP_P3_LEAK_A1_PCV,    // Faz 3: Alan-1 PCV (N436) kaçak
    ATP_P4_LEAK_A2_V,      // Faz 4: Alan-2 vites valfleri kaçak
    ATP_P5_LEAK_A2_PCV,    // Faz 5: Alan-2 PCV (N440) kaçak
    ATP_P6_OIL_LEAK,       // Faz 6: Mekatronik yağ kaçak
    ATP_P7_CALIB_OC,       // Faz 7: Açık/kapalı kalibrasyon
    ATP_P8_CALIB_HOLD,     // Faz 8: Hold kalibrasyon
    ATP_P9_AUTO_SHIFT,     // Faz 9: Otomatik vites testi
    ATP_P10_LEAK_RECHECK,  // Faz 10: Şartlı yağ kaçak yeniden testi (pompa timeout tetikler)
    ATP_DONE,
    ATP_ABORTED
};

extern AutoTestParams     g_autoTestParams;
extern AutoTestResult     g_autoTestResult;
extern volatile uint32_t  g_autoTestReqSeq;   // Arttırılınca test başlar
extern volatile bool      g_autoTestStop;      // true → testi durdur
extern volatile bool      g_leakRecheckNeeded; // Faz 9'da pompa timeout → Faz 10 tetikler

void AutoTestParams_LoadNVS();
void AutoTestParams_SaveNVS();

// Init
void Shared_Init();

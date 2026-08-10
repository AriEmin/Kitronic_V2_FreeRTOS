#include <Arduino.h>
#include <ArduinoJson.h>
#include <math.h>
#include "Shared.h"
#include "Tasks.h"

static inline float barFromUnit() { return g_pumpPub.bar; }

static inline void ensure_internal_path() {
#ifdef BLDC_SELECT
  pinMode(BLDC_SELECT, OUTPUT);
  digitalWrite(BLDC_SELECT, LOW); // LOW = internal pump (de-energised)
#endif
}

static bool start_guard_check(uint32_t ms_window, float I_threshold_A, float& Iavg_out, int min_hits=5) {
  uint32_t t0 = millis();
  float isum = 0.0f; uint32_t icnt = 0;
  int hits = 0;
  while (millis() - t0 < ms_window) {
    float I = g_vescStatus.Im;

    isum += I; icnt++;
    if (I >= I_threshold_A) hits++;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  Iavg_out = (icnt>0) ? (isum/(float)icnt) : 0.0f;
  return hits >= min_hits;
}

static void tlog(const char* s){
  kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, s);
}

// Pump command helpers
static void pump_set_rpm(float rpm){
  // Hedef rpm'i hemen paylaşılan değişkende güncelle
  if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    g_pumpRunRpm = rpm;
    xSemaphoreGive(g_sharedMutex);
  }
  // Ayrıca komut olarak da bildir (TaskBLDCPump durum senkronu için)
  PumpCommand pc{}; pc.cmd = PUMP_CMD_SET_RPM;
  pc.setRpm = rpm;
  portENTER_CRITICAL(&g_portMux);
  pc.seq = g_pumpCmd.seq + 1;
  g_pumpCmd = pc;
  portEXIT_CRITICAL(&g_portMux);
}

static void pump_start(){
  PumpCommand pc{}; pc.cmd = PUMP_CMD_AUTO;
  portENTER_CRITICAL(&g_portMux);
  pc.seq = g_pumpCmd.seq + 1;
  g_pumpCmd = pc;
  portEXIT_CRITICAL(&g_portMux);
}
static void pump_start_single(){
  PumpCommand pc{}; pc.cmd = PUMP_CMD_START;
  portENTER_CRITICAL(&g_portMux);
  pc.seq = g_pumpCmd.seq + 1;
  g_pumpCmd = pc;
  portEXIT_CRITICAL(&g_portMux);
}

static void pump_stop(){
  PumpCommand pc{}; pc.cmd = PUMP_CMD_STOP;
  portENTER_CRITICAL(&g_portMux);
  pc.seq = g_pumpCmd.seq + 1;
  g_pumpCmd = pc;
  portEXIT_CRITICAL(&g_portMux);
}

static void request_piston_hold(PistonChannel piston, uint8_t state, float tol, bool enable){
  PistonHoldRequest req{};
  req.piston = piston;
  req.state = state;
  req.tolerance = tol;
  req.enable = enable;
  portENTER_CRITICAL(&g_portMux);
  g_pistonHoldReq[piston] = req;
  g_pistonHoldReqSeq[piston]++;
  portEXIT_CRITICAL(&g_portMux);
}

static void setValveDutyIdx(int idx, uint16_t duty){
  if (idx < 0 || idx >= 8) return;
  if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    g_valveTargetDuty[idx] = duty;
    xSemaphoreGive(g_sharedMutex);
  }
}

// Quick Health helpers -------------------------------------------------------

// AutoDiag.cpp ile aynı INA mapping (valf index -> INA index)
static const uint8_t QH_VALVE_TO_INA[8] = {0,3,1,2,5,7,6,4};

#ifndef QH_I_COIL_OPEN_MAX_mA
# define QH_I_COIL_OPEN_MAX_mA   50.0f
#endif
#ifndef QH_I_COIL_SHORT_MIN_mA
# define QH_I_COIL_SHORT_MIN_mA  2500.0f
#endif

// Valf bobinlerini sırasıyla sürüp INA akımlarına göre open/short kontrolü
static void qh_test_valves(uint16_t& openMask, uint16_t& shortMask)
{
  openMask  = 0;
  shortMask = 0;

  Telemetry tele{};
  for (int v = 0; v < 8; ++v) {
    // Her valfi orta bir PWM ile kısa süre sür
    setValveDutyIdx(v, 2000);                         // 0..4095 arasında
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      tele = g_tele;
      xSemaphoreGive(g_sharedMutex);
    }

    uint8_t ina = QH_VALVE_TO_INA[v];
    float mA = 0.0f;
    if (ina < 8) {
      mA = fabsf(tele.inaI_mA[ina]);                  // mA cinsinden
    }

    if (mA < QH_I_COIL_OPEN_MAX_mA) {
      openMask  |= (1u << v);                         // Çok düşük akım -> open şüphesi
    } else if (mA > QH_I_COIL_SHORT_MIN_mA) {
      shortMask |= (1u << v);                         // aşırı akım -> short şüphesi
    }

    setValveDutyIdx(v, 0);                            // valfi kapat
    vTaskDelay(pdMS_TO_TICKS(40));
  }
}

// Piston hall sensörlerinin temel elektriksel kontrolü
static uint8_t qh_test_pistons()
{
  float hall[PISTON_CHANNEL_COUNT] = {0};

  if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    for (int i = 0; i < PISTON_CHANNEL_COUNT; ++i) {
      hall[i] = g_pistonHallRaw[i];
    }
    xSemaphoreGive(g_sharedMutex);
  }

  uint8_t mask = 0;
  for (int i = 0; i < PISTON_CHANNEL_COUNT; ++i) {
    float v = hall[i];
    // DRV5055 için beklenen ~1.0..2.3 V aralığının biraz dışının toleransla kabulü
    if (!isfinite(v) || v < 0.5f || v > 1.2f) {
      mask |= (1u << i); // sensör kopuk, kısa devre ya da saçma değer
    }
  }
  return mask;
}

static void request_fast_drain(float targetBar, uint32_t timeoutMs,
                               uint16_t pcvDuty = 0, uint16_t pairDuty = 0)
{
  if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    g_valveDischargeCmd.pcvDuty   = pcvDuty;   // 0 bırakılırsa TaskValveControl default kullanır
    g_valveDischargeCmd.pairDuty  = pairDuty;
    g_valveDischargeCmd.targetBar = targetBar;
    g_valveDischargeCmd.timeoutMs = timeoutMs;
    xSemaphoreGive(g_sharedMutex);
  }
  portENTER_CRITICAL(&g_portMux);
  g_valveDischargeSeq++;
  portEXIT_CRITICAL(&g_portMux);
}

// Quick Health parametreleri
static constexpr float    QH_PUMP_MAX_CURR_A    = 30.0f;
static constexpr float    QH_PUMP_MIN_VOLT_V    = 12.0f;
static constexpr float    QH_PUMP_MIN_DP_BAR    = 5.0f;
static constexpr uint32_t QH_PUMP_RAMP_MS       = 2500;
static constexpr float    QH_PISTON_MIN_BAR     = 20.0f;
static constexpr uint16_t QH_PISTON_DUTY_OPEN   = 2800;
static constexpr uint16_t QH_PISTON_DUTY_SUPP   = 2400;
static constexpr uint32_t QH_PISTON_MOVE_TIMEOUT_MS = 2000;
static constexpr uint32_t QH_PISTON_SAMPLE_MS  = 100;
static constexpr uint32_t QH_PISTON_SETTLE_MS  = 600;
static constexpr float    QH_PISTON_MIN_DELTA_V = 0.5f;
static constexpr float    QH_PISTON_RETURN_TOL  = 0.5f;
static constexpr float    QH_PISTON_REF_TOL_V   = 0.5f;
static constexpr uint8_t  QH_PISTON_COUNT       = 6;   // 4 temel piston + K1 + K2
static const int          QH_PISTON_VALVE_IDX[QH_PISTON_COUNT]   = {2, 0, 7, 4, 3, 6}; // N434,N433,N437,N438,K1->N435,K2->N439
static const int          QH_PISTON_SUPPORT_IDX[QH_PISTON_COUNT] = {1, 1, 5, 5, 1, 5}; // destek valfleri
static constexpr uint16_t PGRAF_SUPPORT_DUTY = 2500;
static const char*        PGRAF_VALVE_NAME[8] = {"N433","N436","N434","N435","N438","N440","N439","N437"};

typedef struct {
  int     OpenDuty;
  int     CloseDuty;
  int     Mid_Duty;
  
} DutyData;

DutyData DataofDuty[6];
static inline bool diag_should_abort(){
  return g_diagAbortFlag;
}


static inline void diag_clear_abort(){
  g_diagAbortFlag = false;
}

// ═══════════════════════════════════════════════════════════════════════════════
// ESKİ VERSİYON - YORUM SATIRINA ALINDI
// ═══════════════════════════════════════════════════════════════════════════════

static void run_piston_graph_test(const PistonGraphConfig& cfg)
{
  diag_clear_abort();
  tlog("[PGRAF] start");
  auto publishState = [&](bool running, int8_t piston){
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
      g_pistonGraphState.running = running;
      g_pistonGraphState.current_piston = (piston >= 0) ? (uint8_t)piston : 0xFF;
      xSemaphoreGive(g_sharedMutex);
    }
  };
  publishState(true, -1);

  // destek valflerini aç
  setValveDutyIdx(1, PGRAF_SUPPORT_DUTY);
  setValveDutyIdx(5, PGRAF_SUPPORT_DUTY);
  vTaskDelay(pdMS_TO_TICKS(200));

  // basinci sagla
  pump_set_rpm(4000.0);
  pump_start();
  uint32_t t0 = millis();
  while (millis() - t0 < 15000) {
    if (diag_should_abort()) { break; }
    if (barFromUnit() >= 50.0f) break;
    vTaskDelay(pdMS_TO_TICKS(200));
  }

  // destek valflerini aç
  setValveDutyIdx(1, PGRAF_SUPPORT_DUTY);
  setValveDutyIdx(5, PGRAF_SUPPORT_DUTY);
  vTaskDelay(pdMS_TO_TICKS(200));

  auto readHallMm = [&](int p) -> float {
    float raw = 0.0f;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      int idx = (p < 6) ? p : (6 - 1);
      raw = g_pistonHallmm[idx];
      xSemaphoreGive(g_sharedMutex);
    }
    return raw;
  };

  auto sendSample = [&](int p, int vIdx, uint16_t duty, float raw){
    {
      StaticJsonDocument<160> d;
      d["cmd"] = "pgraf";
      d["p"] = p;
      d["v"] = PGRAF_VALVE_NAME[vIdx];
      d["mm"] = raw;
      d["raw"] = raw;
      d["duty"] = duty;
      d["t"] = millis();
      char buf[160];
      size_t n = serializeJson(d, buf, sizeof(buf));
      buf[n] = 0;
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
    }
  };

  auto sendDutyEvent = [&](int p, int vIdx, int openDuty, int closeDuty){
    
    StaticJsonDocument<192> d;
    d["cmd"] = "pgraf_evt";
    d["p"] = p;
    d["v"] = PGRAF_VALVE_NAME[vIdx];
    d["open"] = openDuty;
    d["close"] = closeDuty;
    d["t"] = millis();
    char buf[192];
    size_t n = serializeJson(d, buf, sizeof(buf));
    buf[n] = 0;
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
  };

  auto movementDetected = [&](float base, float prev, float valve) -> bool {
    float cur=readHallMm(valve);
    constexpr float kStepThresh = 1.0f;
    float deltaStep = cur - prev;
    return (fabsf(deltaStep) >= kStepThresh);
  };

  const TickType_t sampleTicks = pdMS_TO_TICKS(cfg.sample_ms ? cfg.sample_ms : 100);
  const TickType_t stepTicks   = pdMS_TO_TICKS(cfg.step_ms   ? cfg.step_ms   : 120);

  for (int p = 0; p < QH_PISTON_COUNT; ++p) {
    if (diag_should_abort()) break;
    int vIdx = QH_PISTON_VALVE_IDX[p];
    int supp = QH_PISTON_SUPPORT_IDX[p];
    publishState(true, p);
    char msg[96];
    snprintf(msg, sizeof(msg), "[PGRAF] piston %d valveIdx %d", p, vIdx);
    tlog(msg);
    int openDutyDetected = -1;
    int closeDutyDetected = -1;

    // ramp duty up
    uint16_t duty = cfg.duty_start;
    TickType_t acc = 0;
    float baseHall = readHallMm(p);
    float prevHall = baseHall;
    while (true) {
      if (diag_should_abort()) break;
      setValveDutyIdx(supp, PGRAF_SUPPORT_DUTY);
      setValveDutyIdx(vIdx, duty);      
      vTaskDelay(sampleTicks);
      float hallNow = readHallMm(p);
      if (openDutyDetected < 0 && movementDetected(baseHall, prevHall, p)) {
        openDutyDetected = (int)duty;
      }
      prevHall = hallNow;
      sendSample(p, vIdx, duty, hallNow);
      if (duty >= cfg.duty_end ) break;
      acc += sampleTicks;
      if (acc >= stepTicks) {
        acc = 0;
        duty = (uint16_t)min<uint32_t>(cfg.duty_end, duty + cfg.duty_step);
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(500));

    // ramp duty down
    baseHall = readHallMm(p);
    prevHall = baseHall;
    duty = cfg.duty_end;
    acc = 0;
    while (true) {
      if (diag_should_abort()) break;
      setValveDutyIdx(supp, PGRAF_SUPPORT_DUTY);
      setValveDutyIdx(vIdx, duty);
      vTaskDelay(sampleTicks);
      float hallNow = readHallMm(p);      
      if (closeDutyDetected < 0 && movementDetected(baseHall, prevHall, p)) {
        closeDutyDetected = (int)duty;
      }
      prevHall = hallNow;
      sendSample(p, vIdx, duty, hallNow);
      if (duty <= cfg.duty_start ) break;     
      acc += sampleTicks;
      if (acc >= stepTicks) {
        acc = 0;
        duty = (uint16_t)max<uint32_t>(cfg.duty_start, duty - cfg.duty_step);
      }
    }

    sendDutyEvent(p, vIdx, openDutyDetected, closeDutyDetected);
    DataofDuty[p].OpenDuty=openDutyDetected;
    DataofDuty[p].CloseDuty=closeDutyDetected;
    DataofDuty[p].Mid_Duty=(openDutyDetected+closeDutyDetected)/2;

    // kapat ve bekle
    setValveDutyIdx(vIdx, 0);
    setValveDutyIdx(supp, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  setValveDutyIdx(1, 0);
  setValveDutyIdx(5, 0);
  pump_stop();
  publishState(false, -1);
  tlog("[PGRAF] done");
  diag_clear_abort();
}


// ═══════════════════════════════════════════════════════════════════════════════
// YENİ VERSİYON - PI KONTROLLÜ ORTA KONUM TUTMA
// ═══════════════════════════════════════════════════════════════════════════════

// PID Controller parametreleri artık g_pidParams'dan okunuyor (Shared.h)
// Varsayılan değerler: Kp=12, Ki=0.05, Kd=3.5, maxIntegral=200, deadband=0.5, controlMs=40, holdTimeMs=30000

/*static void run_piston_graph_test(const PistonGraphConfig& cfg)
{
  diag_clear_abort();
  tlog("[PGRAF] PID Control version start");
  
  // Durum yayınlama lambda'sı
  auto publishState = [&](bool running, int8_t piston){
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(3)) == pdTRUE) {
      g_pistonGraphState.running = running;
      g_pistonGraphState.current_piston = (piston >= 0) ? (uint8_t)piston : 0xFF;
      xSemaphoreGive(g_sharedMutex);
    }
  };
  publishState(true, -1);

  // Destek valflerini aç
  setValveDutyIdx(1, PGRAF_SUPPORT_DUTY);
  setValveDutyIdx(5, PGRAF_SUPPORT_DUTY);
  vTaskDelay(pdMS_TO_TICKS(200));

  // Basıncı sağla
  pump_set_rpm(4000.0);
  pump_start();
  uint32_t t0 = millis();
  while (millis() - t0 < 15000) {
    if (diag_should_abort()) { break; }
    if (barFromUnit() >= 50.0f) break;
    vTaskDelay(pdMS_TO_TICKS(200));
  }

  // Destek valflerini tekrar aç (emin olmak için)
  setValveDutyIdx(1, PGRAF_SUPPORT_DUTY);
  setValveDutyIdx(5, PGRAF_SUPPORT_DUTY);
  vTaskDelay(pdMS_TO_TICKS(200));

  // Hall sensör okuma
  auto readHallMm = [&](int p) -> float {
    float raw = 0.0f;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      int idx = (p < 6) ? p : (6 - 1);
      raw = g_pistonHallmm[idx];
      xSemaphoreGive(g_sharedMutex);
    }
    return raw;
  };

  // GUI'ye örnek gönderme
  auto sendSample = [&](int p, int vIdx, uint16_t duty, float hallMm, float targetMm = -1, float error = 0){
    {
      StaticJsonDocument<200> d;
      d["cmd"] = "pgraf";
      d["p"] = p;
      d["v"] = PGRAF_VALVE_NAME[vIdx];
      d["mm"] = hallMm;
      d["raw"] = hallMm;
      d["duty"] = duty;
      d["t"] = millis();
      if (targetMm >= 0) {
        d["target"] = targetMm;  // Hedef konum
        d["err"] = error;        // Hata değeri
      }
      char buf[200];
      size_t n = serializeJson(d, buf, sizeof(buf));
      buf[n] = 0;
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
    }
  };

  // Duty event gönderme
  auto sendDutyEvent = [&](int p, int vIdx, int openDuty, int closeDuty, float minHall = -1, float maxHall = -1){
    
    StaticJsonDocument<256> d;
    d["cmd"] = "pgraf_evt";
    d["p"] = p;
    d["v"] = PGRAF_VALVE_NAME[vIdx];
    d["open"] = openDuty;
    d["close"] = closeDuty;
    d["t"] = millis();
    if (minHall >= 0 && maxHall >= 0) {
      d["hall_min"] = minHall;
      d["hall_max"] = maxHall;
      d["hall_mid"] = (minHall + maxHall) / 2.0f;
    }
    char buf[256];
    size_t n = serializeJson(d, buf, sizeof(buf));
    buf[n] = 0;
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
  };

  // Hareket algılama
  auto movementDetected = [&](float base, float prev, int p) -> bool {
    float cur = readHallMm(p);
    constexpr float kStepThresh = 1.0f;
    float deltaStep = cur - prev;
    return (fabsf(deltaStep) >= kStepThresh);
  };

  const TickType_t sampleTicks = pdMS_TO_TICKS(cfg.sample_ms ? cfg.sample_ms : 100);
  const TickType_t stepTicks   = pdMS_TO_TICKS(cfg.step_ms   ? cfg.step_ms   : 120);

  // Her piston için test
  for (int p = 0; p < QH_PISTON_COUNT; ++p) {
    if (diag_should_abort()) break;
    
    int vIdx = QH_PISTON_VALVE_IDX[p];
    int supp = QH_PISTON_SUPPORT_IDX[p];
    publishState(true, p);
    
    char msg[96];
    snprintf(msg, sizeof(msg), "[PGRAF] piston %d valveIdx %d (PI Control)", p, vIdx);
    tlog(msg);
    
    int openDutyDetected = -1;
    int closeDutyDetected = -1;
    float minHallValue = 999.0f;  // Minimum hall değeri (kapalı konum)
    float maxHallValue = 0.0f;    // Maximum hall değeri (açık konum)

    // ═══════════════════════════════════════════════════════════════════════
    // RAMP UP: Duty'yi artırarak piston aç, min/max hall değerlerini kaydet
    // ═══════════════════════════════════════════════════════════════════════
    uint16_t duty = cfg.duty_start;
    TickType_t acc = 0;
    float baseHall = readHallMm(p);
    float prevHall = baseHall;
    minHallValue = baseHall;  // Başlangıç değeri (kapalı konum)
    
    while (true) {
      if (diag_should_abort()) break;
      setValveDutyIdx(supp, PGRAF_SUPPORT_DUTY);
      setValveDutyIdx(vIdx, duty);      
      vTaskDelay(sampleTicks);
      
      float hallNow = readHallMm(p);
      
      // Min/Max değerleri güncelle
      if (hallNow < minHallValue) minHallValue = hallNow;
      if (hallNow > maxHallValue) maxHallValue = hallNow;
      
      // Açılma duty'sini algıla
      if (openDutyDetected < 0 && movementDetected(baseHall, prevHall, p)) {
        openDutyDetected = (int)duty;
      }
      prevHall = hallNow;
      sendSample(p, vIdx, duty, hallNow);
      
      if (duty >= cfg.duty_end) break;
      
      acc += sampleTicks;
      if (acc >= stepTicks) {
        acc = 0;
        duty = (uint16_t)min<uint32_t>(cfg.duty_end, duty + cfg.duty_step);
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(500));

    // ═══════════════════════════════════════════════════════════════════════
    // RAMP DOWN: Duty'yi azaltarak piston kapat
    // ═══════════════════════════════════════════════════════════════════════
    baseHall = readHallMm(p);
    prevHall = baseHall;
    duty = cfg.duty_end;
    acc = 0;
    
    while (true) {
      if (diag_should_abort()) break;
      setValveDutyIdx(supp, PGRAF_SUPPORT_DUTY);
      setValveDutyIdx(vIdx, duty);
      vTaskDelay(sampleTicks);
      
      float hallNow = readHallMm(p);
      
      // Min/Max değerleri güncelle
      if (hallNow < minHallValue) minHallValue = hallNow;
      if (hallNow > maxHallValue) maxHallValue = hallNow;
      
      // Kapanma duty'sini algıla
      if (closeDutyDetected < 0 && movementDetected(baseHall, prevHall, p)) {
        closeDutyDetected = (int)duty;
      }
      prevHall = hallNow;
      sendSample(p, vIdx, duty, hallNow);
      
      if (duty <= cfg.duty_start) break;
      
      acc += sampleTicks;
      if (acc >= stepTicks) {
        acc = 0;
        duty = (uint16_t)max<uint32_t>(cfg.duty_start, duty - cfg.duty_step);
      }
    }

    // Duty ve Hall değerlerini kaydet
    DataofDuty[p].OpenDuty = openDutyDetected;
    DataofDuty[p].CloseDuty = closeDutyDetected;
    DataofDuty[p].Mid_Duty = (openDutyDetected + closeDutyDetected) / 2;
    
    // Hall min/max değerlerini de sakla (ileride kullanılabilir)
    // DataofDuty[p].MinHall = minHallValue;
    // DataofDuty[p].MaxHall = maxHallValue;
    
    sendDutyEvent(p, vIdx, openDutyDetected, closeDutyDetected, minHallValue, maxHallValue);
    
    // Kapat ve bekle
    setValveDutyIdx(vIdx, 0);
    setValveDutyIdx(supp, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // ═══════════════════════════════════════════════════════════════════════
    // PI KONTROL: Pistonu orta konumda tut (sadece ilk 4 piston için)
    // ═══════════════════════════════════════════════════════════════════════
    if (p < 4 && openDutyDetected > 0 && closeDutyDetected > 0) {
      
      // Hedef: Hall değerinin ortası
      float targetHall = (minHallValue + maxHallValue) / 2.0f;
      int midDuty = DataofDuty[p].Mid_Duty;
      
      char piMsg[128];
      snprintf(piMsg, sizeof(piMsg), "[PGRAF] PID Control: target=%.2f mm, midDuty=%d", targetHall, midDuty);
      tlog(piMsg);
      
      // Destek valfini aç
      setValveDutyIdx(supp, PGRAF_SUPPORT_DUTY);
      vTaskDelay(pdMS_TO_TICKS(200));
      
      // PID Controller değişkenleri
      float integral = 0.0f;
      float lastError = 0.0f;
      float lastHall = readHallMm(p);  // Derivative için önceki hall değeri
      float filteredDerivative = 0.0f;  // Derivative filtresi
      int prevDuty = midDuty;
      
      unsigned long holdStartTime = millis();
      unsigned long lastSendTime = millis();
      
      while (millis() - holdStartTime < g_pidParams.holdTimeMs) {
        if (diag_should_abort()) break;
        
        // Mevcut Hall değerini oku
        float currentHall = readHallMm(p);
        
        // Hata hesapla
        float error = targetHall - currentHall;
        
        // Derivative hesapla (hall değişim hızı) - low-pass filter ile
        float rawDerivative = currentHall - lastHall;
        filteredDerivative = filteredDerivative * 0.8f + rawDerivative * 0.2f;  // Güçlü filtre
        float derivative = filteredDerivative;
        lastHall = currentHall;
        
        // Deadband: Küçük hatalar için tepki verme
        if (fabsf(error) < g_pidParams.deadband) {
          error = 0.0f;
          integral *= 0.9f;  // Integral'i yavaşça sıfırla
        }
        
        // Integral (anti-windup ile)
        integral += error * 0.02f;  // Yavaş biriktir
        if (integral > g_pidParams.maxIntegral) integral = g_pidParams.maxIntegral;
        if (integral < -g_pidParams.maxIntegral) integral = -g_pidParams.maxIntegral;
        
        // PID kontrol çıkışı
        // P: Hataya orantılı tepki
        // I: Kalıcı hatayı düzeltir
        // D: Değişim hızını frenler (- işareti: hareket yönünün tersine)
        float output = (g_pidParams.Kp * error) + (g_pidParams.Ki * integral) - (g_pidParams.Kd * derivative);
        
        // Duty hesapla (güçlü yumuşatma)
        int rawDuty = midDuty + (int)output;
        int newDuty = (prevDuty * 6 + rawDuty * 4) / 10;  // %60 eski, %40 yeni (stabil)
        prevDuty = newDuty;
        
        // Duty limitlerini uygula
        if (newDuty < 0) newDuty = 0;
        if (newDuty > 4095) newDuty = 4095;
        
        // Valfi ayarla
        setValveDutyIdx(vIdx, (uint16_t)newDuty);
        
        // Her 100ms'de bir GUI'ye veri gönder
        if (millis() - lastSendTime >= 100) {
          sendSample(p, vIdx, (uint16_t)newDuty, currentHall, targetHall, error);
          lastSendTime = millis();
        }
        
        lastError = error;
        
        // Kontrol döngüsü periyodu
        vTaskDelay(pdMS_TO_TICKS(g_pidParams.controlMs));
      }
      
      tlog("[PGRAF] PID Control done for this piston");
    }
    
    // Kapat ve bekle
    setValveDutyIdx(vIdx, 0);
    setValveDutyIdx(supp, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // Tüm valfleri kapat ve pompayı durdur
  setValveDutyIdx(1, 0);
  setValveDutyIdx(5, 0);
  pump_stop();
  publishState(false, -1);
  tlog("[PGRAF] PI Control version done");
  diag_clear_abort();
}
*/
// ═══════════════════════════════════════════════════════════════════════════════
// PID AUTO-TUNE - Ziegler-Nichols Relay Feedback Yöntemi
// ═══════════════════════════════════════════════════════════════════════════════

static void run_pid_autotune()
{
  if (!g_pidTuneReq.active) return;
  g_pidTuneReq.active = false;
  
  tlog("[PID_TUNE] Starting auto-tune");
  diag_clear_abort();
  
  uint8_t piston = g_pidTuneReq.piston;
  float targetPos = g_pidTuneReq.targetPos;
  
  if (piston > 3) {
    tlog("[PID_TUNE] Invalid piston");
    return;
  }
  
  // Piston-valf mapping
  static const int PISTON_TO_VALVE[4] = {2, 0, 7, 4};  // P0->V2, P1->V0, P2->V7, P3->V4
  static const int PISTON_SUPPORT[4] = {1, 1, 5, 5};   // Destek valfi
  int vIdx = PISTON_TO_VALVE[piston];
  int supp = PISTON_SUPPORT[piston];
  
  // Hall okuma fonksiyonu (mm cinsinden - g_pistonHallmm kullan)
  auto readHall = [&]() -> float {
    float val = 0.0f;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      val = g_pistonHallmm[piston];
      xSemaphoreGive(g_sharedMutex);
    }
    return val;
  };
  
  // Pompa başlat (AUTO mode - basınç kontrolü yapar)
  tlog("[PID_TUNE] Starting pump...");
  pump_set_rpm(2500.0f);
  pump_start();  // AUTO mode
  vTaskDelay(pdMS_TO_TICKS(3000));  // Basınç oluşması için bekle
  
  { char logbuf[64]; snprintf(logbuf, sizeof(logbuf), "[PID_TUNE] Pressure=%.1f bar", barFromUnit()); tlog(logbuf); }
  
  // Destek valfini aç
  { char logbuf[64]; snprintf(logbuf, sizeof(logbuf), "[PID_TUNE] Opening support valve %d", supp); tlog(logbuf); }
  setValveDutyIdx(supp, 2500);
  vTaskDelay(pdMS_TO_TICKS(500));
  
  // Mevcut pozisyonu oku
  float startPos = readHall();
  { char logbuf[80]; snprintf(logbuf, sizeof(logbuf), "[PID_TUNE] P%d V%d start=%.2f target=%.2f", piston, vIdx, startPos, targetPos); tlog(logbuf); }
  
  // Hedef: mevcut pozisyonun 2mm üstü
  float realTarget = startPos + 2.0f;
  if (targetPos > 0.5f) realTarget = targetPos;
  
  // Relay feedback parametreleri
  const int openDuty = 1700;             // Açma duty (ileri)
  const int closeDuty = 600;               // Kapama duty (geri - yay ile döner)
  const float hysteresis = 0.3f;         // Deadband (mm)
  const int maxOscillations = 50;
  const uint32_t timeout = 30000;        // 30 saniye timeout
  
  float peakHigh = -50.0f, peakLow = 50.0f;
  uint32_t lastCrossTime = 0;
  float periodSum = 0.0f;
  int periodCount = 0;
  bool aboveTarget = false;
  
  tlog("[PID_TUNE] Running relay test...");
  
  uint32_t startTime = millis();
  
  while (millis() - startTime < timeout && periodCount < maxOscillations) {
    if (diag_should_abort()) break;
    
    // Basınç kontrolü - 40 bar altına düşerse pompayı başlat
    float pressure = barFromUnit();
    static uint32_t lastPumpLog = 0;
    if (pressure < 40.0f) {
      pump_set_rpm(2500.0f);
      pump_start();  // AUTO mode - daha güvenilir
      if (millis() - lastPumpLog > 2000) {
        { char logbuf[48]; snprintf(logbuf, sizeof(logbuf), "[PID_TUNE] Pump restart p=%.1f", pressure); tlog(logbuf); }
        lastPumpLog = millis();
      }
    }
    
    float currentPos = readHall();
    float error = realTarget - currentPos;
    
    // Relay control - basit aç/kapat
    int duty;
    if (error > hysteresis) {
      duty = openDuty;   // Hedefin altında → aç (ileri)
    } else if (error < -hysteresis) {
      duty = closeDuty;  // Hedefin üstünde → kapat (geri)
    } else {
      duty = closeDuty;  // Hysteresis içinde → kapat (yay geri çeksin)
    }
    
    setValveDutyIdx(vIdx, (uint16_t)duty);
    
    // Peak ve zero-crossing tespiti
    if (currentPos > peakHigh) peakHigh = currentPos;
    if (currentPos < peakLow) peakLow = currentPos;
    
    bool nowAbove = (currentPos > realTarget);
    if (nowAbove != aboveTarget) {
      // Zero crossing
      uint32_t now = millis();
      if (lastCrossTime > 0) {
        float halfPeriod = (now - lastCrossTime) / 1000.0f;
        periodSum += halfPeriod * 2.0f;
        periodCount++;
        { char logbuf[80]; snprintf(logbuf, sizeof(logbuf), "[PID_TUNE] Cross #%d pos=%.2f peak=%.2f/%.2f", periodCount, currentPos, peakLow, peakHigh); tlog(logbuf); }
      }
      lastCrossTime = now;
      aboveTarget = nowAbove;
      
      // Peak reset
      peakHigh = currentPos;
      peakLow = currentPos;
    }
    
    // Her 1 saniyede bir durum logla
    static uint32_t lastLog = 0;
    if (millis() - lastLog > 1000) {
      lastLog = millis();
      { char logbuf[80]; snprintf(logbuf, sizeof(logbuf), "[PID_TUNE] pos=%.2f err=%.2f duty=%d", currentPos, error, duty); tlog(logbuf); }
    }
    
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  
  // Valfi kapat
  setValveDutyIdx(vIdx, 0);
  setValveDutyIdx(supp, 0);
  pump_stop();
  
  // Sonuçları hesapla
  if (periodCount < 3) {
    tlog("[PID_TUNE] Not enough oscillations");
    // Hata JSON gönder
    {
      StaticJsonDocument<128> resp;
      resp["cmd"] = "pid_tune_result";
      resp["success"] = false;
      resp["error"] = "Not enough oscillations";
      char buf[128];
      serializeJson(resp, buf, sizeof(buf));
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
    }
    return;
  }
  
  float Tu = periodSum / periodCount;  // Ultimate period (s)
  float amplitude = (peakHigh - peakLow) / 2.0f;
  float Ku = (4.0f * (float)openDuty) / (3.14159f * amplitude);  // Ultimate gain
  
  // Ziegler-Nichols PID formülleri
  float newKp = 0.6f * Ku;
  float newKi = 1.2f * Ku / Tu;
  float newKd = 0.075f * Ku * Tu;
  
  // Sınırla
  if (newKp > 50.0f) newKp = 50.0f;
  if (newKp < 1.0f) newKp = 1.0f;
  if (newKi > 5.0f) newKi = 5.0f;
  if (newKi < 0.01f) newKi = 0.01f;
  if (newKd > 20.0f) newKd = 20.0f;
  if (newKd < 0.1f) newKd = 0.1f;
  
  // Global parametreleri güncelle
  g_pidParams.Kp = newKp;
  g_pidParams.Ki = newKi;
  g_pidParams.Kd = newKd;
  
  { char logbuf[80]; snprintf(logbuf, sizeof(logbuf), "[PID_TUNE] Done! Kp=%.2f Ki=%.3f Kd=%.2f Tu=%.3fs", newKp, newKi, newKd, Tu); tlog(logbuf); }
  
  // Sonuç JSON gönder
  {
    StaticJsonDocument<256> resp;
    resp["cmd"] = "pid_tune_result";
    resp["success"] = true;
    resp["kp"] = newKp;
    resp["ki"] = newKi;
    resp["kd"] = newKd;
    resp["Tu"] = Tu;
    resp["Ku"] = Ku;
    char buf[256];
    serializeJson(resp, buf, sizeof(buf));
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
  }
  
  diag_clear_abort();
}

// ═══════════════════════════════════════════════════════════════════════════════
// VALF ADAPTASYONU - Her valf için 3 konum (kapalı/orta/açık) PWM değerlerini öğrenir
// ═══════════════════════════════════════════════════════════════════════════════

// Valf -> Piston mapping (hangi valf hangi pistonu hareket ettirir)
// Valf 0: N433 -> Piston 1 (PISTON_1_3)
// Valf 1: N436 -> Destek valf (piston yok)
// Valf 2: N434 -> Piston 0 (PISTON_5_7)
// Valf 3: N435 -> Piston 5 (K1)
// Valf 4: N438 -> Piston 3 (PISTON_6_R)
// Valf 5: N440 -> Destek valf (piston yok)
// Valf 6: N439 -> Piston 4 (K2)
// Valf 7: N437 -> Piston 2 (PISTON_2_4)
static const int VALVE_TO_PISTON[8] = {1, -1, 0, 5, 3, -1, 4, 2};

// Adaptasyon parametreleri
#define ADAPT_PWM_START      400
#define ADAPT_PWM_MAX        3500
#define ADAPT_PWM_STEP       100
#define ADAPT_SETTLE_MS      80
#define ADAPT_SAMPLE_MS      50
#define ADAPT_MOVE_THRESHOLD 0.15f
#define ADAPT_SUPPORT_PWM    2500

// K1/K2 kavrama pistonları için özel parametreler (30mm strok, yay ile geri dönüş)
#define ADAPT_K_SETTLE_MS      500    // Daha uzun bekleme (yavaş hareket)
#define ADAPT_K_MOVE_THRESHOLD 0.3f   // Düşük eşik (hassas algılama)
#define ADAPT_K_RETURN_MS      1000   // Yay geri dönüş süresi (uzun)

static void run_valve_adaptation()
{
  diag_clear_abort();
  tlog("[ADAPT] Valve adaptation start");
  
  // JSON durum gönderme
  auto sendStatus = [](const char* status, int progress) {
    
    StaticJsonDocument<128> d;
    d["cmd"] = "adapt_status";
    d["status"] = status;
    d["progress"] = progress;
    d["t"] = millis();
    char buf[128];
    size_t n = serializeJson(d, buf, sizeof(buf));
    buf[n] = 0;
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
  };
  
  // JSON sonuç gönderme
  auto sendResult = [](int valve, const char* name, uint16_t openPWM, uint16_t closePWM, 
                       uint16_t holdMidPWM, float minPos, float midPos, float maxPos, 
                       uint16_t openMs, uint16_t closeMs) {
    
    StaticJsonDocument<256> d;
    d["cmd"] = "adapt_result";
    d["valve"] = valve;
    d["name"] = name;
    d["open_pwm"] = openPWM;
    d["close_pwm"] = closePWM;
    d["hold_mid_pwm"] = holdMidPWM;
    d["min_pos"] = minPos;
    d["mid_pos"] = midPos;
    d["max_pos"] = maxPos;
    d["open_ms"] = openMs;
    d["close_ms"] = closeMs;
    d["t"] = millis();
    char buf[256];
    size_t n = serializeJson(d, buf, sizeof(buf));
    buf[n] = 0;
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
  };
  
  sendStatus("Basınç sağlanıyor...", 0);
  
  // Destek valflerini aç
  setValveDutyIdx(1, ADAPT_SUPPORT_PWM);  // N436
  setValveDutyIdx(5, ADAPT_SUPPORT_PWM);  // N440
  vTaskDelay(pdMS_TO_TICKS(200));
  
  // Basıncı sağla
  pump_set_rpm(3000.0f);
  vTaskDelay(pdMS_TO_TICKS(50));
  pump_start_single();
  
  uint32_t t0 = millis();
  while (millis() - t0 < 15000) {
    if (diag_should_abort()) break;
    if (barFromUnit() >= 50.0f) break;
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  
  // Pompayı DURMA - düşük RPM ile basıncı koru!
  pump_set_rpm(1500.0f);  // Düşük RPM ile basıncı koru
  
  if (diag_should_abort() || barFromUnit() < 40.0f) {
    sendStatus("Basınç hatası!", -1);
    for (int v = 0; v < 8; v++) setValveDutyIdx(v, 0);
    pump_stop();
    tlog("[ADAPT] Pressure fail, abort");
    return;
  }
  
  tlog("[ADAPT] Pressure OK, starting adaptation");
  
  // Hall okuma - doğrudan ADC veya paylaşılan değişken
  static const int HALL_PINS[4] = {HALL_N434_PIN, HALL_N433_PIN, HALL_N437_PIN, HALL_N438_PIN};
  auto readHallDirect = [](int p) -> float {
    if (p < 0 || p > 5) return 0.0f;
    if (p < 4) {
      // Normal vites pistonları - doğrudan ADC oku
      int pin = HALL_PINS[p];
      int raw = analogRead(pin);
      float volt = (raw / 4095.0f) * 3.3f;
      return volt * 3.0f;
    }
    // K1/K2 kavrama pistonları - TaskADSMonitor'dan oku
    // ADS1115 güncelleme döngüsü için bekle (birden fazla okuma yap, ortalamasını al)
    float sum = 0.0f;
    int count = 0;
    for (int i = 0; i < 5; i++) {
      vTaskDelay(pdMS_TO_TICKS(50));  // Her okuma arasında 50ms bekle
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (p == 4) sum += g_n439_stroke_mm;  // K2
        else if (p == 5) sum += g_n435_stroke_mm;  // K1
        count++;
        xSemaphoreGive(g_sharedMutex);
      }
    }
    return (count > 0) ? (sum / count) : 0.0f;
  };
  
  // Valf isimleri
  static const char* VALVE_NAMES[8] = {"N433", "N436", "N434", "N435", "N438", "N440", "N439", "N437"};
  
  int adaptedCount = 0;
  
  // Her valf için adaptasyon
  for (int v = 0; v < 8; v++) {
    if (diag_should_abort()) break;
    
    int piston = VALVE_TO_PISTON[v];
    if (piston < 0) {
      // Destek valf - sadece varsayılan değer ata
      g_valveAdapt.openPWM[v] = 2000;
      g_valveAdapt.closePWM[v] = 0;
      g_valveAdapt.holdMidPWM[v] = 0;
      g_valveAdapt.holdOpenPWM[v] = 2000;
      continue;
    }
    
    char msg[64];
    snprintf(msg, sizeof(msg), "%s adaptasyonu...", VALVE_NAMES[v]);
    sendStatus(msg, 10 + (v * 80 / 8));
    
    // Basınç kontrolü - düşükse pompayı hızlandır
    float currentBar = barFromUnit();
    if (currentBar < 40.0f) {
      pump_set_rpm(3000.0f);
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    
    // Destek valflerini açık tut (test edilen valf değilse)
    if (v != 1) setValveDutyIdx(1, ADAPT_SUPPORT_PWM);  // N436
    if (v != 5) setValveDutyIdx(5, ADAPT_SUPPORT_PWM);  // N440
    
    { char logbuf[64]; snprintf(logbuf, sizeof(logbuf), "[ADAPT] Testing V%d (%s) -> P%d", v, VALVE_NAMES[v], piston); tlog(logbuf); }
    
    // K1/K2 kavrama pistonları için özel parametreler
    bool isClutch = (piston == 4 || piston == 5);  // K2=piston4, K1=piston5
    uint32_t settleMs = isClutch ? ADAPT_K_SETTLE_MS : ADAPT_SETTLE_MS;
    float moveThreshold = isClutch ? ADAPT_K_MOVE_THRESHOLD : ADAPT_MOVE_THRESHOLD;
    uint32_t returnMs = isClutch ? ADAPT_K_RETURN_MS : 300;
    
    // 1. Kapalı konum oku (valf kapalı)
    setValveDutyIdx(v, 0);
    vTaskDelay(pdMS_TO_TICKS(returnMs));  // K1/K2 için yay geri dönüş süresi
    float closedPos = readHallDirect(piston);
    g_valveAdapt.minPos_mm[v] = closedPos;
    
    // 2. Açma touch point bul
    uint16_t openTouchPWM = ADAPT_PWM_MAX;
    uint32_t openResponseMs = 0;
    float startPos = closedPos;
    
    for (uint16_t pwm = ADAPT_PWM_START; pwm <= ADAPT_PWM_MAX; pwm += ADAPT_PWM_STEP) {
      setValveDutyIdx(v, pwm);
      vTaskDelay(pdMS_TO_TICKS(settleMs));
      
      uint32_t moveStart = millis();
      float currentPos = readHallDirect(piston);
      float delta = fabsf(currentPos - startPos);
      
      if (delta >= moveThreshold) {
        openTouchPWM = pwm;
        openResponseMs = millis() - moveStart + settleMs;
        { char logbuf[48]; snprintf(logbuf, sizeof(logbuf), "[ADAPT] V%d open PWM=%d", v, pwm); tlog(logbuf); }
        break;
      }
    }
    
    g_valveAdapt.openPWM[v] = openTouchPWM;
    g_valveAdapt.responseOpen_ms[v] = openResponseMs;
    
    // 3. Tam açık konuma getir ve oku
    setValveDutyIdx(v, ADAPT_PWM_MAX);
    vTaskDelay(pdMS_TO_TICKS(isClutch ? 1000 : 500));  // K1/K2 için daha uzun
    float openPos = readHallDirect(piston);
    g_valveAdapt.maxPos_mm[v] = openPos;
    g_valveAdapt.travelMM[v] = fabsf(openPos - closedPos);
    
    // 4. Orta konumu hesapla
    float midPos = (closedPos + openPos) / 2.0f;
    g_valveAdapt.midPos_mm[v] = midPos;
    
    // 5. Orta konumda tutma PWM'i bul (basit yaklaşım: açma PWM + %50)
    g_valveAdapt.holdMidPWM[v] = (uint16_t)(openTouchPWM * 1.5f);
    g_valveAdapt.holdOpenPWM[v] = (uint16_t)(openTouchPWM * 1.3f);
    
    // 6. Kapatma touch point bul (K1/K2 için yay ile geri dönüş)
    uint16_t closeTouchPWM = 0;
    uint32_t closeResponseMs = 0;
    startPos = openPos;
    
    // PWM'i kademeli düşür
    for (uint16_t pwm = ADAPT_PWM_MAX; pwm >= ADAPT_PWM_STEP; pwm -= ADAPT_PWM_STEP) {
      setValveDutyIdx(v, pwm);
      vTaskDelay(pdMS_TO_TICKS(settleMs));
      
      uint32_t moveStart = millis();
      float currentPos = readHallDirect(piston);
      float delta = fabsf(currentPos - startPos);
      
      if (delta >= moveThreshold) {
        closeTouchPWM = pwm;
        closeResponseMs = millis() - moveStart + settleMs;
        { char logbuf[48]; snprintf(logbuf, sizeof(logbuf), "[ADAPT] V%d close PWM=%d", v, pwm); tlog(logbuf); }
        break;
      }
    }
    
    g_valveAdapt.closePWM[v] = closeTouchPWM;
    g_valveAdapt.responseClose_ms[v] = closeResponseMs;
    
    // Valfi kapat
    setValveDutyIdx(v, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    
    // Sonucu gönder
    sendResult(v, VALVE_NAMES[v], g_valveAdapt.openPWM[v], g_valveAdapt.closePWM[v],
               g_valveAdapt.holdMidPWM[v], g_valveAdapt.minPos_mm[v], g_valveAdapt.midPos_mm[v],
               g_valveAdapt.maxPos_mm[v], g_valveAdapt.responseOpen_ms[v], g_valveAdapt.responseClose_ms[v]);
    
    adaptedCount++;
  }
  
  // Tüm valfleri kapat
  for (int v = 0; v < 8; v++) setValveDutyIdx(v, 0);
  pump_stop();
  
  // Adaptasyon tamamlandı
  g_valveAdapt.calibrated = true;
  g_valveAdapt.timestamp = millis() / 1000;  // Basit timestamp
  
  // Final sonuç gönder
  {
    StaticJsonDocument<256> d;
    d["cmd"] = "adapt_final";
    d["success"] = true;
    d["valves_adapted"] = adaptedCount;
    d["t"] = millis();
    char buf[256];
    size_t n = serializeJson(d, buf, sizeof(buf));
    buf[n] = 0;
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
  }
  
  sendStatus("Adaptasyon tamamlandı", 100);
  { char logbuf[48]; snprintf(logbuf, sizeof(logbuf), "[ADAPT] Complete, %d valves", adaptedCount); tlog(logbuf); }
  diag_clear_abort();
}

// ═══════════════════════════════════════════════════════════════════════════════
// DİNAMİK VİTES GEÇİŞ TESTİ - Vites değişimi sırasındaki valf tepki sürelerini ölçer
// ═══════════════════════════════════════════════════════════════════════════════

// DQ200 Vites-Piston Mapping
// 1. vites: PISTON_1_3 (N433)
// 2. vites: PISTON_2_4 (N437)
// 3. vites: PISTON_1_3 (N433) - farklı basınç
// 4. vites: PISTON_2_4 (N437) - farklı basınç
// 5. vites: PISTON_5_7 (N434)
// 6. vites: PISTON_6_R (N438)
// 7. vites: PISTON_5_7 (N434) - farklı basınç
// R vites: PISTON_6_R (N438) - farklı basınç

// Test parametreleri
#define DYN_TEST_DUTY        2800     // Valf açma duty
#define DYN_TEST_SUPPORT     2500     // Destek valf duty
#define DYN_SETTLE_MS        500      // Başlangıç bekleme
#define DYN_TRANSITION_MS    600      // Geçiş için max bekleme
#define DYN_SAMPLE_MS        2        // Örnekleme periyodu (doğrudan ADC ile çok hızlı)
#define DYN_MIN_MOVE_MM      0.15f    // Minimum hareket eşiği (volt değişimi ~0.05V)
#define DYN_MAX_RESPONSE_MS  100      // Max kabul edilebilir tepki süresi

// Vites geçiş senaryoları
struct GearTransition {
  const char* name;
  int fromPiston;   // -1 = yok
  int toPiston;
  int support;      // Destek valf index
};

static const GearTransition GEAR_TRANSITIONS[] = {
  {"N->1", -1, 0, 1},  // Nötr -> 1. vites (PISTON_1_3, N436 destek)
  {"1->2", 0, 2, 5},   // 1 -> 2. vites (PISTON_2_4, N440 destek)
  {"2->3", 2, 0, 1},   // 2 -> 3. vites (PISTON_1_3, N436 destek)
  {"3->4", 0, 2, 5},   // 3 -> 4. vites (PISTON_2_4, N440 destek)
  {"4->5", 2, 1, 1},   // 4 -> 5. vites (PISTON_5_7, N436 destek)
  {"5->6", 1, 3, 5},   // 5 -> 6. vites (PISTON_6_R, N440 destek)
};
#define NUM_TRANSITIONS (sizeof(GEAR_TRANSITIONS) / sizeof(GEAR_TRANSITIONS[0]))

// Test sonucu yapısı
struct DynTestResult {
  bool tested;
  bool passed;
  uint32_t responseMs;    // Tepki süresi (ms)
  float startHall;        // Başlangıç hall değeri
  float endHall;          // Bitiş hall değeri
  float deltaHall;        // Hareket miktarı
};

static void run_dynamic_gear_test()
{
  diag_clear_abort();
  tlog("[DYNGEAR] Dynamic gear transition test start");
  
  // Sonuçları sakla
  DynTestResult results[NUM_TRANSITIONS] = {0};
  bool overallPass = true;
  
  // Hall okuma lambda - DOĞRUDAN ADC (TaskADSMonitor'u beklemeden anlık değer)
  // Piston index -> Hall pin mapping:
  // 0: PISTON_5_7 -> N434 (pin 12)
  // 1: PISTON_1_3 -> N433 (pin 2)
  // 2: PISTON_2_4 -> N437 (pin 18)
  // 3: PISTON_6_R -> N438 (pin 10)
  // 4: K2 (ADS) - shared değer kullan
  // 5: K1 (ADS) - shared değer kullan
  static const int HALL_PINS[4] = {HALL_N434_PIN, HALL_N433_PIN, HALL_N437_PIN, HALL_N438_PIN};
  
  auto readHallDirect = [](int p) -> float {
    if (p < 0 || p > 5) return 0.0f;
    
    // İlk 4 piston MCU ADC'den okunur
    if (p < 4) {
      int pin = HALL_PINS[p];
      int raw = analogRead(pin);
      float volt = (raw / 4095.0f) * 3.3f;
      // Basit mm dönüşümü (yaklaşık): 0-3.3V -> 0-10mm
      return volt * 3.0f;  // Kalibrasyona göre ayarlanabilir
    }
    
    // K1/K2 pistonları ADS'den - shared değer kullan
    float val = 0.0f;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      val = g_pistonHallmm[p];
      xSemaphoreGive(g_sharedMutex);
    }
    return val;
  };
  
  // JSON sonuç gönderme
  auto sendResult = [](int idx, const char* name, const DynTestResult& r) {
    
    StaticJsonDocument<256> d;
    d["cmd"] = "dyngear_result";
    d["idx"] = idx;
    d["name"] = name;
    d["tested"] = r.tested;
    d["pass"] = r.passed;
    d["response_ms"] = r.responseMs;
    d["start_hall"] = r.startHall;
    d["end_hall"] = r.endHall;
    d["delta"] = r.deltaHall;
    d["t"] = millis();
    char buf[256];
    size_t n = serializeJson(d, buf, sizeof(buf));
    buf[n] = 0;
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
  };
  
  // Başlangıç durumu gönder
  auto sendStatus = [](const char* status, int progress) {
    
    StaticJsonDocument<128> d;
    d["cmd"] = "dyngear_status";
    d["status"] = status;
    d["progress"] = progress;
    d["t"] = millis();
    char buf[128];
    size_t n = serializeJson(d, buf, sizeof(buf));
    buf[n] = 0;
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
  };
  
  sendStatus("Basınç sağlanıyor...", 0);
  
  // Destek valflerini aç
  setValveDutyIdx(1, DYN_TEST_SUPPORT);  // N436
  setValveDutyIdx(5, DYN_TEST_SUPPORT);  // N440
  vTaskDelay(pdMS_TO_TICKS(200));
  
  // Basıncı sağla - basit ve güvenilir yöntem
  pump_set_rpm(3000.0f);
  vTaskDelay(pdMS_TO_TICKS(50));  // RPM komutunun işlenmesini bekle
  pump_start_single();
  
  uint32_t t0 = millis();
  while (millis() - t0 < 15000) {
    if (diag_should_abort()) break;
    if (barFromUnit() >= 50.0f) break;
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  pump_stop();
  
  if (diag_should_abort() || barFromUnit() < 40.0f) {
    sendStatus("Basınç hatası!", -1);
    setValveDutyIdx(1, 0);
    setValveDutyIdx(5, 0);
    tlog("[DYNGEAR] Pressure fail, abort");
    return;
  }
  
  tlog("[DYNGEAR] Pressure OK");
  
  sendStatus("Test başlıyor...", 10);
  
  // Her geçiş senaryosunu test et
  for (int i = 0; i < (int)NUM_TRANSITIONS; i++) {
    if (diag_should_abort()) break;
    
    const GearTransition& trans = GEAR_TRANSITIONS[i];
    DynTestResult& res = results[i];
    res.tested = true;
    
    char msg[64];
    snprintf(msg, sizeof(msg), "[DYNGEAR] Testing %s", trans.name);
    tlog(msg);
    sendStatus(trans.name, 10 + (i * 80 / NUM_TRANSITIONS));
    
    // Destek valfini ayarla
    setValveDutyIdx(trans.support, DYN_TEST_SUPPORT);
    
    // Eğer önceki piston varsa, onu aç ve bekle
    if (trans.fromPiston >= 0) {
      int fromValve = QH_PISTON_VALVE_IDX[trans.fromPiston];
      setValveDutyIdx(fromValve, DYN_TEST_DUTY);
      vTaskDelay(pdMS_TO_TICKS(DYN_SETTLE_MS));
    }
    
    // Hedef pistonun başlangıç değerini oku
    res.startHall = readHallDirect(trans.toPiston);
    
    // Geçiş zamanlaması başlat
    uint32_t transStart = millis();
    
    // Önceki pistonu kapat (varsa) ve yeni pistonu aç - EŞ ZAMANLI
    if (trans.fromPiston >= 0) {
      int fromValve = QH_PISTON_VALVE_IDX[trans.fromPiston];
      setValveDutyIdx(fromValve, 0);
    }
    int toValve = QH_PISTON_VALVE_IDX[trans.toPiston];
    setValveDutyIdx(toValve, DYN_TEST_DUTY);
    
    // Hareket algılanana kadar bekle
    bool moved = false;
    uint32_t responseTime = 0;
    
    while ((millis() - transStart) < DYN_TRANSITION_MS) {
      float currentHall = readHallDirect(trans.toPiston);
      float delta = fabsf(currentHall - res.startHall);
      
      if (delta >= DYN_MIN_MOVE_MM) {
        moved = true;
        responseTime = millis() - transStart;
        res.endHall = currentHall;
        res.deltaHall = delta;
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(DYN_SAMPLE_MS));
    }
    
    if (!moved) {
      // Timeout - hareket algılanamadı
      res.endHall = readHallDirect(trans.toPiston);
      res.deltaHall = fabsf(res.endHall - res.startHall);
      responseTime = DYN_TRANSITION_MS;
    }
    
    res.responseMs = responseTime;
    res.passed = moved && (responseTime <= DYN_MAX_RESPONSE_MS);
    
    if (!res.passed) overallPass = false;
    
    // Sonucu gönder
    sendResult(i, trans.name, res);
    
    // Valfleri kapat
    setValveDutyIdx(toValve, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
  }
  
  // Tüm valfleri kapat
  for (int v = 0; v < 8; v++) {
    setValveDutyIdx(v, 0);
  }
  pump_stop();
  
  // Final sonuç gönder
  {
    StaticJsonDocument<512> d;
    d["cmd"] = "dyngear_final";
    d["pass"] = overallPass;
    d["t"] = millis();
    
    JsonArray arr = d.createNestedArray("faults");
    for (int i = 0; i < (int)NUM_TRANSITIONS; i++) {
      if (results[i].tested && !results[i].passed) {
        JsonObject fault = arr.createNestedObject();
        fault["name"] = GEAR_TRANSITIONS[i].name;
        fault["response_ms"] = results[i].responseMs;
        fault["valve"] = PGRAF_VALVE_NAME[QH_PISTON_VALVE_IDX[GEAR_TRANSITIONS[i].toPiston]];
        
        // Arıza nedeni
        if (results[i].deltaHall < DYN_MIN_MOVE_MM) {
          fault["reason"] = "NO_MOVEMENT";
        } else {
          fault["reason"] = "SLOW_RESPONSE";
        }
      }
    }
    
    char buf[512];
    size_t n = serializeJson(d, buf, sizeof(buf));
    buf[n] = 0;
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
  }
  
  sendStatus(overallPass ? "BAŞARILI" : "HATALI", 100);
  tlog(overallPass ? "[DYNGEAR] PASS" : "[DYNGEAR] FAIL");
  diag_clear_abort();
}

// Quick Health ---------------------------------------------------------------
static void run_quick_health(const QuickHealthConfig& cfg)
{
  tlog("[QH] Quick health start");

  // Başlangıç: sonucu temizle + running = true
  if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    QuickHealthResult z{};
    g_quickHealthRes = z;
    g_quickHealthRes.running = true;
    strlcpy(g_quickHealthRes.reason, "RUN", sizeof(g_quickHealthRes.reason));
    xSemaphoreGive(g_sharedMutex);
  }

  // Güvenlik: pompa stop, tüm valf PWM'leri sıfırla, iç/hazne hattı seç
  pump_stop();
  for (int v = 0; v < 8; ++v) {
    setValveDutyIdx(v, 0);
  }
  vTaskDelay(pdMS_TO_TICKS(50));
  ensure_internal_path();

  // 1) Valf bobin self-test
  uint16_t valveOpenMask  = 0;
  uint16_t valveShortMask = 0;
  qh_test_valves(valveOpenMask, valveShortMask);
  bool valves_ok = (valveOpenMask == 0 && valveShortMask == 0);
  uint8_t pistonErrMask = 0;
  bool pistons_ok = true;

  if (!valves_ok) {
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_quickHealthRes.running           = false;
      g_quickHealthRes.done              = true;
      g_quickHealthRes.pass              = false;
      g_quickHealthRes.valves_ok         = false;
      g_quickHealthRes.pistons_ok        = false;
      g_quickHealthRes.pump_ok           = false;
      g_quickHealthRes.pressure_ok       = false;
      g_quickHealthRes.valve_open_mask   = valveOpenMask;
      g_quickHealthRes.valve_short_mask  = valveShortMask;
      g_quickHealthRes.piston_err_mask   = 0;
      g_quickHealthRes.p0_bar            = 0.0f;
      g_quickHealthRes.p1_bar            = 0.0f;
      g_quickHealthRes.dp_bar            = 0.0f;
      g_quickHealthRes.leak_barps        = 0.0f;
      g_quickHealthRes.t_fill_ms         = 0;
      g_quickHealthRes.t_hold_ms         = 0;
      strlcpy(g_quickHealthRes.reason, "VALVE", sizeof(g_quickHealthRes.reason));
      xSemaphoreGive(g_sharedMutex);
    }
    tlog("[QH] abort: valve electrical fault");
    return;
  }

  // 3) Pompa / motor start guard
  pump_set_rpm(cfg.pumpRpm);
  vTaskDelay(pdMS_TO_TICKS(20));
  pump_start();

  float Iavg_guard = 0.0f;
  bool pump_ok = start_guard_check(2000, 0.5f, Iavg_guard, /*min_hits=*/5);
  float minMainV = 100.0f;
  float maxMainI = 0.0f;
  auto sample_supply = [&]() {
    float v = 0.0f, i = 0.0f;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      v = g_tele.mainV;
      i = g_tele.mainI;
      xSemaphoreGive(g_sharedMutex);
    }
    if (v > 0.01f && v < minMainV) minMainV = v;
    if (i > maxMainI) maxMainI = i;
  };
  sample_supply();

  if (!pump_ok) {
    pump_stop();
    vTaskDelay(pdMS_TO_TICKS(200));

    float p_now = barFromUnit();
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_quickHealthRes.running     = false;
      g_quickHealthRes.done        = true;
      g_quickHealthRes.pass        = false;

      g_quickHealthRes.valves_ok   = valves_ok;
      g_quickHealthRes.pistons_ok  = pistons_ok;
      g_quickHealthRes.pump_ok     = false;
      g_quickHealthRes.pressure_ok = false;

      g_quickHealthRes.valve_open_mask   = valveOpenMask;
      g_quickHealthRes.valve_short_mask  = valveShortMask;
      g_quickHealthRes.piston_err_mask   = pistonErrMask;

      g_quickHealthRes.p0_bar      = p_now;
      g_quickHealthRes.p1_bar      = p_now;
      g_quickHealthRes.dp_bar      = 0.0f;
      g_quickHealthRes.leak_barps  = 0.0f;
      g_quickHealthRes.t_fill_ms   = 0;
      g_quickHealthRes.t_hold_ms   = 0;
      strlcpy(g_quickHealthRes.reason, "NO_PUMP", sizeof(g_quickHealthRes.reason));
      xSemaphoreGive(g_sharedMutex);
    }
    tlog("[QH] abort: NO_PUMP");
    return;
  }

  // 4) Doldurma fazı (hedef basınca kadar)
  uint32_t t0 = millis();
  float    p_start = barFromUnit();
  float    p_now   = p_start;

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(50));
    p_now = barFromUnit();
    sample_supply();
    if (p_now >= cfg.target_bar) break;
    if (millis() - t0 > cfg.fillTimeout_ms) break;
  }

  pump_stop();
  uint32_t t_fill    = millis() - t0;
  float    p_fillEnd = barFromUnit();
  sample_supply();

  bool pressure_ok = true;
  if (!isfinite(p_fillEnd) || p_fillEnd < -1.0f || p_fillEnd > 200.0f) {
    pressure_ok = false;
  }
  float dp_fill = p_fillEnd - p_start;
  if (fabsf(dp_fill) < QH_PUMP_MIN_DP_BAR) {
    // Pompa çalışmasına rağmen basınç neredeyse hiç değişmemiş
    pressure_ok = false;
  }

  bool supply_ok = (minMainV >= QH_PUMP_MIN_VOLT_V) && (maxMainI <= QH_PUMP_MAX_CURR_A);
  pump_ok = pump_ok && supply_ok;

  bool reachedTarget = (p_fillEnd >= (cfg.target_bar - 10.0f));

  // Hedefe ulaşılamadıysa / sensör/güç saçmaysa leak fazına girme
  if (!pressure_ok || !reachedTarget || !pump_ok || !supply_ok) {
    float dp = dp_fill;

    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_quickHealthRes.running     = false;
      g_quickHealthRes.done        = true;
      g_quickHealthRes.pass        = false;

      g_quickHealthRes.valves_ok   = valves_ok;
      g_quickHealthRes.pistons_ok  = pistons_ok;
      g_quickHealthRes.pump_ok     = pump_ok;
      g_quickHealthRes.pressure_ok = pressure_ok;

      g_quickHealthRes.valve_open_mask   = valveOpenMask;
      g_quickHealthRes.valve_short_mask  = valveShortMask;
      g_quickHealthRes.piston_err_mask   = pistonErrMask;
      g_quickHealthRes.piston_diag       = 0;
      g_quickHealthRes.piston_diag       = 0;

      g_quickHealthRes.p0_bar      = p_start;
      g_quickHealthRes.p1_bar      = p_fillEnd;
      g_quickHealthRes.dp_bar      = dp;
      g_quickHealthRes.leak_barps  = 0.0f;
      g_quickHealthRes.t_fill_ms   = t_fill;
      g_quickHealthRes.t_hold_ms   = 0;

      if (!pressure_ok)       strlcpy(g_quickHealthRes.reason, "SENSOR", sizeof(g_quickHealthRes.reason));
      else if (!supply_ok)    strlcpy(g_quickHealthRes.reason, "SUPPLY", sizeof(g_quickHealthRes.reason));
      else if (!pump_ok)      strlcpy(g_quickHealthRes.reason, "NO_PUMP", sizeof(g_quickHealthRes.reason));
      else                    strlcpy(g_quickHealthRes.reason, "NO_P", sizeof(g_quickHealthRes.reason));
      xSemaphoreGive(g_sharedMutex);
    }

    char buf[120];
    snprintf(buf, sizeof(buf),
             "[QH] abort: %s p0=%.1f p=%.1f dp=%.1f V=%.1f I=%.1f",
             !pressure_ok ? "SENSOR" : (!supply_ok ? "SUPPLY" : (!pump_ok ? "NO_PUMP" : "NO_P")),
             p_start, p_fillEnd, dp_fill, minMainV, maxMainI);
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
    return;
  }

  // 5) Piston hareket testi (>=dyn threshold) + K1/K2
  /*float piston_press_thresh = QH_PISTON_MIN_BAR;
  if (cfg.target_bar > 5.0f) {
    // hedefe göre dinamik eşik (hedef-5, max 40)
    piston_press_thresh = fminf(QH_PISTON_MIN_BAR, cfg.target_bar - 5.0f);
    if (piston_press_thresh < 20.0f) piston_press_thresh = 20.0f;
  }
  bool piston_pressure_ok = (p_fillEnd >= piston_press_thresh);
  if (!piston_pressure_ok) {
    pistons_ok = false;
    pistonErrMask = (uint8_t)((1u << QH_PISTON_COUNT) - 1u); // tüm pistonlar için "basınç düşük" hatası
    {
      char buf[96];
      snprintf(buf, sizeof(buf), "[QH] piston skipped: low pressure p=%.2f thresh=%.2f", p_fillEnd, piston_press_thresh);
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
    }
  }
  if (pistons_ok) {
    float refOpen[QH_PISTON_COUNT] = {0};
    float refClose[QH_PISTON_COUNT] = {0};
    bool  refValid[QH_PISTON_COUNT] = {0};
    float raw0_arr[QH_PISTON_COUNT] = {0};
    float rawOpen_arr[QH_PISTON_COUNT] = {0};
    float rawClose_arr[QH_PISTON_COUNT] = {0};
    bool  moved_arr[QH_PISTON_COUNT] = {0};
    bool  returned_arr[QH_PISTON_COUNT] = {0};
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      for (int i = 0; i < PISTON_CHANNEL_COUNT && i < QH_PISTON_COUNT; ++i) {
        uint8_t mask = g_pistonManualRef[i].validMask;
        if ((mask & (1u << PISTON_REF_OPEN)) && (mask & (1u << PISTON_REF_CLOSED))) {
          refOpen[i] = g_pistonManualRef[i].raw[PISTON_REF_OPEN];
          refClose[i]= g_pistonManualRef[i].raw[PISTON_REF_CLOSED];
          refValid[i]= true;
        }
      }
      xSemaphoreGive(g_sharedMutex);
    }
    uint32_t pist_diag_bits = 0;

    auto is_close = [](float v, float target) {
      return fabsf(v - target) <= QH_PISTON_REF_TOL_V;
    };

    auto read_hall = [&](int idx, float &out) -> bool {
      bool ok = false;
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (idx < PISTON_CHANNEL_COUNT) {
          out = g_pistonHallRaw[idx];
        } else if (idx == 4) { // K1
          out = g_n435_stroke_mm;
        } else if (idx == 5) { // K2
          out = g_n439_stroke_mm;
        } else {
          out = 0.0f;
        }
        ok = isfinite(out);
        xSemaphoreGive(g_sharedMutex);
      }
      return ok;
    };

    // Önce sensör aralığına bak (ham değerler) sadece temel 4 piston için
    pistonErrMask = qh_test_pistons();
    if (pistonErrMask != 0) pistons_ok = false;

    for (int p = 0; p < QH_PISTON_COUNT; ++p) {
      int valveIdx = QH_PISTON_VALVE_IDX[p];
      int supportIdx = QH_PISTON_SUPPORT_IDX[p];
      float raw0 = 0.0f, rawOpen = 0.0f, rawClose = 0.0f;
      bool ok0 = read_hall(p, raw0);
      raw0_arr[p] = raw0;

      if (!ok0 || valveIdx < 0 || supportIdx < 0) {
        pistonErrMask |= (1u << p);
        pistons_ok = false;
        continue;
      }

      setValveDutyIdx(valveIdx, QH_PISTON_DUTY_OPEN);
      setValveDutyIdx(supportIdx, QH_PISTON_DUTY_SUPP);
      bool moved = false;
      uint32_t tOpen = millis();
      while ((millis() - tOpen) < QH_PISTON_MOVE_TIMEOUT_MS) {
        if (read_hall(p, rawOpen)) {
          rawOpen_arr[p] = rawOpen;
          if (refValid[p]) {
            // referanslı kontrol: open referansına yaklaş (delta + tolerans)
            float thr = QH_PISTON_MIN_DELTA_V;
            if (is_close(rawOpen, refOpen[p]) ||
                (refOpen[p] < refClose[p] ? (rawOpen <= (refClose[p] - thr))
                                          : (rawOpen >= (refClose[p] + thr)))) {
              moved = true;
              break;
            }
          } else {
            if (fabsf(rawOpen - raw0) >= QH_PISTON_MIN_DELTA_V) {
              moved = true;
              break;
            }
          }
        }
        vTaskDelay(pdMS_TO_TICKS(QH_PISTON_SAMPLE_MS));
      }

      setValveDutyIdx(valveIdx, 0);
      setValveDutyIdx(supportIdx, 0);
      vTaskDelay(pdMS_TO_TICKS(QH_PISTON_SETTLE_MS));
      read_hall(p, rawClose);
      rawClose_arr[p] = rawClose;

      float dRet  = rawClose - raw0;
      bool returned;
      if (refValid[p]) {
        returned = is_close(rawClose, refClose[p]) ||
                   fabsf(rawClose - refClose[p]) <= (QH_PISTON_REF_TOL_V + QH_PISTON_MIN_DELTA_V);
      } else {
        returned = fabsf(dRet)  <= (QH_PISTON_RETURN_TOL + QH_PISTON_MIN_DELTA_V);
      }
      moved_arr[p] = moved;
      returned_arr[p] = returned;
      if (moved)    pist_diag_bits |= (1u << p);
      if (returned) pist_diag_bits |= (1u << (p + 8));

      if (!(moved && returned)) {
        pistonErrMask |= (1u << p);
        pistons_ok = false;
      }
    }
    // Debug: seri loga piston hareket sonuçlarını bas
    {
      const char* names[QH_PISTON_COUNT] = {"5_7","1_3","2_4","6_R","K1","K2"};
      for (int p = 0; p < QH_PISTON_COUNT; ++p) {
        char buf[128];
        snprintf(buf, sizeof(buf), "[QH] pist %s mv=%d ret=%d raw0=%.3f open=%.3f close=%.3f",
                 names[p], moved_arr[p], returned_arr[p], raw0_arr[p], rawOpen_arr[p], rawClose_arr[p]);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
      }
    }
    // Publish diag bits
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_quickHealthRes.piston_diag = pist_diag_bits;
      xSemaphoreGive(g_sharedMutex);
    }
  }
*/
  // 6) Hold (kaçak) fazı
  float    p_before = barFromUnit();
  uint32_t hold_ms  = (uint32_t)(cfg.hold_s * 1000.0f);
  if (hold_ms < 200) hold_ms = 200;

  uint32_t holdStart = millis();
  while (millis() - holdStart < hold_ms) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  float p_after    = barFromUnit();
  float dp         = p_after - p_before;
  float leak_barps = (cfg.hold_s > 0.01f) ? (dp / cfg.hold_s) : 0.0f;

  bool leakOk = ((-leak_barps) <= cfg.maxLeak_barps);
  bool pass   = valves_ok && pump_ok && pressure_ok && leakOk; //&& pistons_ok 

  char why[16];
  if (!valves_ok)           strlcpy(why, "VALVE",   sizeof(why));
  //else if (!pistons_ok)     strlcpy(why, piston_pressure_ok ? "PISTON" : "P_LOW", sizeof(why));
  else if (!pump_ok)        strlcpy(why, "NO_PUMP", sizeof(why));
  else if (!pressure_ok)    strlcpy(why, "SENSOR",  sizeof(why));
  else if (!reachedTarget)  strlcpy(why, "NO_P",    sizeof(why));
  else if (!leakOk)         strlcpy(why, "LEAK",    sizeof(why));
  else                      strlcpy(why, "OK",      sizeof(why));

  if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    g_quickHealthRes.running           = false;
    g_quickHealthRes.done              = true;
    g_quickHealthRes.pass              = pass;

    g_quickHealthRes.p0_bar            = p_before;
    g_quickHealthRes.p1_bar            = p_after;
    g_quickHealthRes.dp_bar            = dp;
    g_quickHealthRes.leak_barps        = leak_barps;
    g_quickHealthRes.t_fill_ms         = t_fill;
    g_quickHealthRes.t_hold_ms         = hold_ms;

    g_quickHealthRes.valves_ok         = valves_ok;
    g_quickHealthRes.pistons_ok        = pistons_ok;
    g_quickHealthRes.pump_ok           = pump_ok;
    g_quickHealthRes.pressure_ok       = pressure_ok;

    g_quickHealthRes.valve_open_mask   = valveOpenMask;
    g_quickHealthRes.valve_short_mask  = valveShortMask;
    g_quickHealthRes.piston_err_mask   = pistonErrMask;
    g_quickHealthRes.piston_diag       = 0;
    g_quickHealthRes.piston_diag       = g_quickHealthRes.piston_diag; // diag daha önce set edildiyse koru

    strlcpy(g_quickHealthRes.reason, why, sizeof(g_quickHealthRes.reason));
    xSemaphoreGive(g_sharedMutex);
  }

  char buf[120];
  snprintf(buf, sizeof(buf),
           "[QH] %s p0=%.1f p1=%.1f leak=%.3f bar/s",
           pass ? "OK" : "FAIL",
           p_before, p_after, leak_barps);
  kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
}

// Sistem kaçak testi ---------------------------------------------------------
static void run_leak_test(const LeakTestConfig& cfg)
{
  // Başlangıç reset
  if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    g_leakRes = LeakTestResult{};
    g_leakRes.running = true;
    strlcpy(g_leakRes.reason, "RUN", sizeof(g_leakRes.reason));
    xSemaphoreGive(g_sharedMutex);
  }

  // Güvenlik: pompa stop + tüm valfler kapalı
  pump_stop();
  for (int v = 0; v < 8; ++v) setValveDutyIdx(v, 0);
  vTaskDelay(pdMS_TO_TICKS(50));
  ensure_internal_path();

  float p0 = barFromUnit();

  // 1) Pre-Drain
  if (p0 > cfg.preDrainAbove_bar) {
    request_fast_drain(cfg.preDrainTo_bar, cfg.preDrainTimeout_ms);

    uint32_t tD0 = millis();
    while (millis() - tD0 < cfg.preDrainTimeout_ms) {
      vTaskDelay(pdMS_TO_TICKS(100));
      float p = barFromUnit();
      if (p <= cfg.preDrainTo_bar) break;
    }

    float p_after_drain = barFromUnit();
    if (p_after_drain > (cfg.preDrainTo_bar + 1.0f)) {
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_leakRes.running = false;
        g_leakRes.done = true;
        g_leakRes.pass = false;
        g_leakRes.p_start_bar = p0;
        strlcpy(g_leakRes.reason, "DRAIN_TMO", sizeof(g_leakRes.reason));
        xSemaphoreGive(g_sharedMutex);
      }
      return;
    }
  }

  // Test başlangıç basıncını güncelle
  float p_start = barFromUnit();

  // 2) Fill to target
  pump_set_rpm(cfg.pumpRpm);
  vTaskDelay(pdMS_TO_TICKS(20));
  pump_start();

  uint32_t tF0 = millis();
  float p_now = p_start;

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(50));
    p_now = barFromUnit();
    if (p_now >= cfg.target_bar) break;
    if (millis() - tF0 > cfg.fillTimeout_ms) break;
  }

  pump_stop();

  uint32_t t_fill = millis() - tF0;
  float p_fill = barFromUnit();

  bool reachedTarget = (p_fill >= (cfg.target_bar - 1.0f));
  bool pressure_ok = isfinite(p_fill) && p_fill > -1.0f && p_fill < 200.0f;

  if (!pressure_ok) {
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_leakRes.running = false;
      g_leakRes.done = true;
      g_leakRes.pass = false;
      g_leakRes.p_start_bar = p_start;
      g_leakRes.p_fill_bar  = p_fill;
      g_leakRes.t_fill_ms   = t_fill;
      strlcpy(g_leakRes.reason, "SENSOR", sizeof(g_leakRes.reason));
      xSemaphoreGive(g_sharedMutex);
    }
    return;
  }

  if (!reachedTarget) {
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_leakRes.running = false;
      g_leakRes.done = true;
      g_leakRes.pass = false;
      g_leakRes.p_start_bar = p_start;
      g_leakRes.p_fill_bar  = p_fill;
      g_leakRes.t_fill_ms   = t_fill;
      strlcpy(g_leakRes.reason, "NO_P", sizeof(g_leakRes.reason));
      xSemaphoreGive(g_sharedMutex);
    }
    return;
  }

  // 3) Settle (ilk gevşeme)
  uint32_t tS0 = millis();
  uint32_t settle_ms = (uint32_t)(cfg.settle_s * 1000.0f);
  if (settle_ms < 1000) settle_ms = 1000;

  while (millis() - tS0 < settle_ms) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  float p_base = barFromUnit();
  uint32_t t_settle = millis() - tS0;

  // 4) Hold (gerçek kaçak ölçümü)
  uint32_t hold_ms = (uint32_t)(cfg.hold_s * 1000.0f);
  if (hold_ms < 2000) hold_ms = 2000;

  uint32_t tH0 = millis();
  while (millis() - tH0 < hold_ms) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  float p_end = barFromUnit();

  float dp = p_end - p_base;
  float leak_barps = (cfg.hold_s > 0.01f) ? (dp / cfg.hold_s) : 0.0f;

  bool leakOk = ((-leak_barps) <= cfg.maxLeak_barps);
  bool pass = leakOk;

  // 5) Result write
  if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    g_leakRes.running    = false;
    g_leakRes.done       = true;
    g_leakRes.pass       = pass;

    g_leakRes.p_start_bar= p_start;
    g_leakRes.p_fill_bar = p_fill;
    g_leakRes.p_base_bar = p_base;
    g_leakRes.p_end_bar  = p_end;

    g_leakRes.dp_bar     = dp;
    g_leakRes.leak_barps = leak_barps;

    g_leakRes.t_fill_ms  = t_fill;
    g_leakRes.t_settle_ms= t_settle;
    g_leakRes.t_hold_ms  = hold_ms;

    strlcpy(g_leakRes.reason, pass ? "OK" : "LEAK", sizeof(g_leakRes.reason));
    xSemaphoreGive(g_sharedMutex);
  }
}

// Motor/Pompa testi: çeşitli RPM'lerde 5 sn çalıştırıp dp/dt ve akım ölçer
static void run_motor_pump_test(const MotorPumpTestConfig& cfg)
{
  tlog("[MOTOR] test start");
  if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    g_motorTestRes = MotorPumpTestResult{};
    g_motorTestRes.running = true;
    strlcpy(g_motorTestRes.reason, "RUN", sizeof(g_motorTestRes.reason));
    xSemaphoreGive(g_sharedMutex);
  }

  ensure_internal_path();
  pump_stop();
  vTaskDelay(pdMS_TO_TICKS(50));

  MotorPumpTestResult res{};
  res.running = false;
  res.done = true;
  res.pass = true;
  res.fail_mask = 0;

  auto ensure_drain_if_high = [&]()->bool{
    float p = barFromUnit();
    if (p <= 20.0f) return true;
    request_fast_drain(/*targetBar=*/10.0f, /*timeoutMs=*/8000);
    uint32_t t0 = millis();
    while (millis() - t0 < 8000) {
      vTaskDelay(pdMS_TO_TICKS(200));
      float pb = barFromUnit();
      if (pb <= 15.0f) return true;
    }
    return false;
  };

  // başlangıçta basınç yüksekse boşalt
  if (!ensure_drain_if_high()) {
    res.pass = false;
    res.fail_mask = 0x1F;
    strlcpy(res.reason, "START_P_HIGH", sizeof(res.reason));
  }

  for (int i = 0; i < 5; ++i) {
    if (!res.pass && res.fail_mask == 0x1F) break; // drain başarısız ise çık

    // her adım öncesi basıncı düşür; başarısız olsa da ölçüme devam et
    if (!ensure_drain_if_high()) {
      res.pass = false;
      res.fail_mask |= (1u << i);
      strlcpy(res.reason, "P_HIGH", sizeof(res.reason));
    }

    float target_rpm = cfg.rpms[i];
    res.rpm[i] = target_rpm;

    pump_set_rpm(target_rpm);
     {
            char msg[64];
            snprintf(msg, sizeof(msg), "[DIAG] Target RPM = %f \n", target_rpm);
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        }
    vTaskDelay(pdMS_TO_TICKS(20));
    pump_start_single();

    float p0 = barFromUnit();
    float isum = 0.0f; int icnt = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < cfg.duration_ms) {
      float I = g_vescStatus.Im;
      isum += I; icnt++;
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    float p1 = barFromUnit();
    pump_stop();
    vTaskDelay(pdMS_TO_TICKS(500)); // adımlar arası bekleme

    float dur_s = cfg.duration_ms / 1000.0f;
    float dp_rate = (p1 - p0) / (dur_s > 0.01f ? dur_s : 1.0f);
    float Iavg = (icnt > 0) ? (isum / (float)icnt) : 0.0f;

    res.dp_barps[i] = dp_rate;
    res.I_avg[i] = Iavg;

    bool stepFail = false;
    if (dp_rate < cfg.min_dp_barps) {
      stepFail = true;
      res.pass = false;
      if (Iavg > cfg.high_I_A) {
        strlcpy(res.reason, "AKIM_YUKSEK_DP_DUSUK", sizeof(res.reason));
      } else if (Iavg < cfg.low_I_A) {
        strlcpy(res.reason, "AKIM_COK_DUSUK_DP_DUSUK", sizeof(res.reason));
      } else {
        strlcpy(res.reason, "DP_DUSUK", sizeof(res.reason));
      }
    }
    if (stepFail) res.fail_mask |= (1u << i);
  }

  if (res.pass) strlcpy(res.reason, "OK", sizeof(res.reason));

  if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    g_motorTestRes = res;
    xSemaphoreGive(g_sharedMutex);
  }
  tlog("[MOTOR] test done");
}

// Basit valf/piston testi: mevcut akım/hall değerlerine bakarak özet üretir
static float sample_valve_current(int idx, uint16_t duty, uint32_t ms, float& busVout) {
  // INA219 sırası: 0 N433,1 N434,2 N435,3 N436,4 N437,5 N438,6 N439,7 N440
  static const int INA_IDX[8] = {0, 3, 1, 2, 5, 7, 6, 4}; // valve idx -> INA idx
  const int stepMs = 200;
  int cnt = 0;
  float isum = 0.0f;
  float vbus = 0.0f;
  setValveDutyIdx(idx, duty);
  // INA219 güncellenmesi için kısa bekleme
  vTaskDelay(pdMS_TO_TICKS(20));
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(4)) == pdTRUE) {
      int ia = (idx >=0 && idx < 8) ? INA_IDX[idx] : idx;
      if (ia < 0 || ia >= 8) ia = 0;
      float I = g_tele.inaI_mA[ia];
      float V = g_tele.inaV[ia];
      // yön farkı/gürültü için mutlak değer, küçük akımı sıfırla
      if (fabsf(I) < 5.0f) I = 0.0f;
      if (I < 0) I = -I;
      isum += I;
      vbus += V;
      xSemaphoreGive(g_sharedMutex);
      cnt++;
    }
    vTaskDelay(pdMS_TO_TICKS(stepMs));
  }
  setValveDutyIdx(idx, 0);
  busVout = (cnt > 0) ? (vbus / (float)cnt) : 0.0f;
  return (cnt > 0) ? (isum / (float)cnt) : 0.0f;
}

static void run_valve_piston_test()
{
  tlog("[VP] test start");
  ensure_internal_path();
  pump_stop();
  if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    g_valvePistonRes = ValvePistonResult{};
    g_valvePistonRes.running = true;
    strlcpy(g_valvePistonRes.reason, "RUN", sizeof(g_valvePistonRes.reason));
    xSemaphoreGive(g_sharedMutex);
  }

  ValvePistonResult res{};
  res.running = false;
  res.done = true;
  res.pass = true;
  strlcpy(res.reason, "OK", sizeof(res.reason));

  // Valf akım testleri: kısa darbe + düşük duty
  for (int i=0;i<8;i++){
    float vbus = 0.0f;
    // Inrush: duty 2500, 400ms
    float inrush = sample_valve_current(i, 2500, 400, vbus);
    vTaskDelay(pdMS_TO_TICKS(40));
    // Hold: duty 1200, 300ms
    float hold = sample_valve_current(i, 1200, 300, vbus);

    ValveResult vr{};
    vr.open_fault  = fabsf(inrush) < 100.0f;       // 100 mA altı bağlılık yok
    vr.short_fault = fabsf(inrush) > 2500.0f;      // 2.5A üstü kısa devre kabul
    vr.R_est = (fabsf(inrush) > 20.0f && vbus > 0.1f) ? (vbus / (inrush/1000.0f)) : 0.0f;
    vr.inrush_mA = inrush;
    vr.hold_mA   = hold;
    vr.duty_prof = 1000.0f; // mevcut duty örneği

    if (vr.open_fault || vr.short_fault) {
      res.pass = false;
      strlcpy(res.reason, "VALVE", sizeof(res.reason));
    }
    res.valves[i] = vr;
    vTaskDelay(pdMS_TO_TICKS(10));
  }

    // Piston hareket testi (yavaş aç/kapa, hall delta ölçümü)
  auto read_hall = []() {
    float hallRaw[4] = {0};
    float k1=0, k2=0;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      memcpy(hallRaw, g_pistonHallRaw, sizeof(hallRaw));
      k1 = g_n435_stroke_mm;
      k2 = g_n439_stroke_mm;
      xSemaphoreGive(g_sharedMutex);
    }
    float out[6] = {hallRaw[0], hallRaw[1], hallRaw[2], hallRaw[3], k1, k2};
    return std::array<float,6>{out[0],out[1],out[2],out[3],out[4],out[5]};
  };

  // basınç 42 bar altındaysa pompayı kaldır, test sırasında düşerse tekrar kaldır
  auto ensure_pressure = [&]()->bool{
    float p = barFromUnit();
    if (p < 42.0f) {
      pump_set_rpm(2000.0f);
      pump_start();
      uint32_t t0 = millis();
      while (millis() - t0 < 15000) {
        vTaskDelay(pdMS_TO_TICKS(200));
        p = barFromUnit();
        if (p >= 42.0f) break;
      }
    }
    return p >= 42.0f;
  };

  if (!ensure_pressure()) {
    res.pass = false;
    strlcpy(res.reason, "P_LOW", sizeof(res.reason));
    auto halls = read_hall();
    for (int i=0;i<6;i++){
      PistonResult pr{}; pr.hall_raw = halls[i]; pr.moved = false; pr.err = true;
      res.pistons[i] = pr;
    }
  } else {
    // Basınç valflerini (N436 idx=1, N440 idx=5) önce aç
    setValveDutyIdx(1, 2000);
    setValveDutyIdx(5, 2000);
    vTaskDelay(pdMS_TO_TICKS(200));

    auto halls0 = read_hall();
    for (int p=0;p<6;p++){
      int valveIdx = QH_PISTON_VALVE_IDX[p];
      // test boyunca basınç 42 bar altına inerse pompayı tekrar kaldır
      if (!ensure_pressure()) { res.pass = false; strlcpy(res.reason, "P_LOW", sizeof(res.reason)); break; }
      setValveDutyIdx(valveIdx, 1350); vTaskDelay(pdMS_TO_TICKS(2000));
      setValveDutyIdx(valveIdx, 1000); vTaskDelay(pdMS_TO_TICKS(500));
      auto halls1 = read_hall();
      setValveDutyIdx(valveIdx, 850);  vTaskDelay(pdMS_TO_TICKS(2000));
      setValveDutyIdx(valveIdx, 1000); vTaskDelay(pdMS_TO_TICKS(200));
      auto halls2 = read_hall();

      float h0 = halls0[p], h1 = halls1[p], h2 = halls2[p];
      float d_open  = h0 - h1;
      float d_close = h2 - h1;
      bool open_ok  = (d_open  > 0.05f);   // daha düşük eşik
      bool close_ok = (d_close > 0.05f);
      float span    = fabsf(h0 - h2);

      PistonResult pr{}; pr.hall_raw = h2;
      if (p >= 4) { pr.moved = true; pr.err = false; }
      else {
        pr.moved = (open_ok && close_ok) || (span > 0.10f);
        pr.err = !pr.moved;
      }
      if (pr.err) { res.pass = false; strlcpy(res.reason, "PISTON", sizeof(res.reason)); }
      res.pistons[p] = pr;
    }

    // Test sonunda önce vites valflerini kapat, en son basınç valflerini
    for (int i=0;i<8;i++){
      if (i==1 || i==5) continue;
      setValveDutyIdx(i, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    setValveDutyIdx(1, 0);
    setValveDutyIdx(5, 0);

    
  }

  if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    g_valvePistonRes = res;
    xSemaphoreGive(g_sharedMutex);
  }
  tlog("[VP] test done");
}

static void run_piston_mid_test(const PistonMidTestConfig& cfg)
{
  tlog("[PMID] test start");
  auto publish = [&](const PistonMidTestResult& st){
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_pistonMidRes = st;
      xSemaphoreGive(g_sharedMutex);
    }
  };

  PistonMidTestResult res{};
  res.running = true;
  res.done = false;
  res.pass = true;
  res.fail_mask = 0;
  res.max_drop_bar = 0.0f;
  res.current_piston = 0xFF;
  res.remaining_ms = (cfg.duration_ms > 0) ? cfg.duration_ms : 30000;
  strlcpy(res.reason, "RUN", sizeof(res.reason));
  publish(res);

  auto ensure_pressure = [&]()->bool{
    float p = barFromUnit();
    if (p < 42.0f) {
      pump_set_rpm(3000.0f);
      pump_start_single();
      uint32_t t0 = millis();
      while (millis() - t0 < 10000) {
        vTaskDelay(pdMS_TO_TICKS(200));
        p = barFromUnit();
        if (p >= 42.0f) break;
      }
    }
    return p >= 42.0f;
  };

  if (!ensure_pressure()) {
    res.pass = false;
    strlcpy(res.reason, "P_LOW", sizeof(res.reason));
    res.running = false;
    res.done = true;
    res.remaining_ms = 0;
    publish(res);
    pump_stop();
  } else {
    // basinci valflerini ac
    setValveDutyIdx(1, 2000);
    setValveDutyIdx(5, 2000);
    vTaskDelay(pdMS_TO_TICKS(200));

    uint32_t holdMs = (cfg.duration_ms > 0) ? cfg.duration_ms : 30000;
    float dropThresh = (cfg.drop_thresh_bar > 0.1f) ? cfg.drop_thresh_bar : 5.0f;

    for (int p=0; p<4; ++p) {
      res.current_piston = (uint8_t)p;
      res.remaining_ms = holdMs;
      publish(res);

      if (!ensure_pressure()) { res.pass=false; strlcpy(res.reason,"P_LOW",sizeof(res.reason)); break; }
      float p_start = barFromUnit();
      request_piston_hold((PistonChannel)p, PISTON_REF_MID, 0.05f, true);
      uint32_t t0 = millis();
      while (millis() - t0 < holdMs) {
        vTaskDelay(pdMS_TO_TICKS(250));
        float p_now = barFromUnit();
        float drop = p_start - p_now;
        if (drop > res.max_drop_bar) res.max_drop_bar = drop;
        uint32_t elapsed = millis() - t0;
        res.remaining_ms = (elapsed < holdMs) ? (holdMs - elapsed) : 0;
        publish(res);
        if (drop > dropThresh) {
          res.fail_mask |= (1u << p);
          res.pass = false;
          strlcpy(res.reason, "MID_LEAK", sizeof(res.reason));
          break;
        }
      }
      request_piston_hold((PistonChannel)p, PISTON_REF_MID, 0.05f, false);
      res.current_piston = 0xFF;
      res.remaining_ms = 0;
      publish(res);
    }

    for (int p=0; p<4; ++p) {
      request_piston_hold((PistonChannel)p, PISTON_REF_MID, 0.1f, false);
    }

    for (int i=0;i<8;i++){
      if (i==1 || i==5) continue;
      setValveDutyIdx(i, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    setValveDutyIdx(1, 0);
    setValveDutyIdx(5, 0);
    pump_stop();
  }

  if (res.pass && strcmp(res.reason, "RUN") == 0) {
    strlcpy(res.reason, "OK", sizeof(res.reason));
  }
  res.running = false;
  res.done = true;
  res.remaining_ms = 0;
  publish(res);
  tlog("[PMID] test done");
}

// Task ----------------------------------------------------------------------
void TaskDiag(void *pvParameters){
  (void)pvParameters;
  tlog("[TEST] Diag task started.");

  uint32_t seenQh   = 0;
  uint32_t seenLeak = 0;
  uint32_t seenMotor= 0;
  uint32_t seenVp   = 0;
  uint32_t seenPMid = 0;
  uint32_t seenPGraf= 0;
  uint32_t seenDynGear = 0;
  uint32_t seenAdapt = 0;

  for(;;){
    uint32_t qhReq;
    portENTER_CRITICAL(&g_portMux); qhReq = g_quickHealthReqSeq; portEXIT_CRITICAL(&g_portMux);

    uint32_t leakReq;
    portENTER_CRITICAL(&g_portMux); leakReq = g_leakReqSeq; portEXIT_CRITICAL(&g_portMux);

    uint32_t motorReq;
    portENTER_CRITICAL(&g_portMux); motorReq = g_motorTestReqSeq; portEXIT_CRITICAL(&g_portMux);
    uint32_t vpReq;
    portENTER_CRITICAL(&g_portMux); vpReq = g_valvePistonReqSeq; portEXIT_CRITICAL(&g_portMux);
    uint32_t pmidReq;
    portENTER_CRITICAL(&g_portMux); pmidReq = g_pistonMidReqSeq; portEXIT_CRITICAL(&g_portMux);
    uint32_t pgrafReq;
    portENTER_CRITICAL(&g_portMux); pgrafReq = g_pistonGraphReqSeq; portEXIT_CRITICAL(&g_portMux);

    // Quick Health
    if (qhReq != seenQh) {
      seenQh = qhReq;
      QuickHealthConfig cfg{};
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        cfg = g_quickHealthCfg;
        xSemaphoreGive(g_sharedMutex);
      }
      run_quick_health(cfg);
    }

    // Sistem Kaçak Testi
    if (leakReq != seenLeak) {
      seenLeak = leakReq;
      LeakTestConfig cfg{};
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        cfg = g_leakCfg;
        xSemaphoreGive(g_sharedMutex);
      }
      run_leak_test(cfg);
    }

    // Motor/Pompa Testi
    if (motorReq != seenMotor) {
      seenMotor = motorReq;
      MotorPumpTestConfig cfg{};
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        cfg = g_motorTestCfg;
        xSemaphoreGive(g_sharedMutex);
      }
      run_motor_pump_test(cfg);
    }
    // Valf/Piston Testi
    if (vpReq != seenVp) {
      seenVp = vpReq;
      run_valve_piston_test();
    }
    
    // Piston Mid Testi
    if (pmidReq != seenPMid) {
      seenPMid = pmidReq;
      PistonMidTestConfig cfg{};
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        cfg = g_pistonMidCfg;
        xSemaphoreGive(g_sharedMutex);
      }
      run_piston_mid_test(cfg);
    }
    // Piston grafik testi
    if (pgrafReq != seenPGraf) {
      seenPGraf = pgrafReq;
      PistonGraphConfig cfg{};
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        cfg = g_pistonGraphCfg;
        xSemaphoreGive(g_sharedMutex);
      }
      run_piston_graph_test(cfg);
    }
    
    // Dinamik Vites Geçiş Testi
    uint32_t dynGearReq;
    portENTER_CRITICAL(&g_portMux); dynGearReq = g_dynGearTestReqSeq; portEXIT_CRITICAL(&g_portMux);
    if (dynGearReq != seenDynGear) {
      seenDynGear = dynGearReq;
      run_dynamic_gear_test();
    }
    
    // Valf Adaptasyonu
    uint32_t adaptReq;
    portENTER_CRITICAL(&g_portMux); adaptReq = g_valveAdaptReqSeq; portEXIT_CRITICAL(&g_portMux);
    if (adaptReq != seenAdapt) {
      seenAdapt = adaptReq;
      run_valve_adaptation();
    }
    
    // PID Auto-Tune
    static uint32_t seenPidTune = 0;
    uint32_t pidTuneReq;
    portENTER_CRITICAL(&g_portMux); pidTuneReq = g_pidTuneReqSeq; portEXIT_CRITICAL(&g_portMux);
    if (pidTuneReq != seenPidTune) {
      seenPidTune = pidTuneReq;
      run_pid_autotune();
    }

    vTaskDelay(pdMS_TO_TICKS(25));
  }
}



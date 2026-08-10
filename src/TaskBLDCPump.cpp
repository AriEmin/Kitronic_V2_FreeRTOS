#include <Arduino.h>
#include <driver/twai.h>
#include "Tasks.h"
#include "Shared.h"


#define REQUIRE_VESC_STATUS_RX 0   // 1 yaparsan tekrar RX zorunlu olur
#define VESC_TX_DEBUG          1   // 1 ise gÃ¶nderilen akÄ±mÄ± 1 snâ€™de bir loglar

// ===================== Konfig =====================
static const float    PUMP_TARGET_BAR     = 60.0f;
static const float    PUMP_RESTART_BAR    = 42.0f;
static const uint32_t PUMP_TIMEOUT_MS     = 15000;
static const int      PRESSURE_SAMPLES    = 10;
static const uint32_t VESC_CMD_PERIOD_MS  = 100;
static const float    PUMP_RAMP_RATE_RPM_S= 500.0f;   // ramp up/down hızı (rpm/sn)
static const float    PUMP_STEP_RPM       = 500.0f;   // kademeli artış miktarı
static const float    PUMP_FEEDBACK_RATIO = 0.75f;    // bir sonraki adıma geçmek için geri besleme oranı
static const uint32_t PUMP_STEP_TIMEOUT_MS= 800;      // geri besleme gelmezse bekleme süresi
constexpr int PUMP_POLE_PAIRS = 4;
static const float    PUMP_START_CURRENT_A    = 5.0f;   // Başlatma akımı (A)
static const float    PUMP_START_RPM_THRESHOLD= 500.0f; // Bu RPM'e ulaşınca RPM kontrole geç

// Soft landing & ramp-down parametreleri
static const float    PUMP_SOFT_START_BAR   = 45.0f;   // Bu basınçtan itibaren yavaşlamaya başla
static const float    PUMP_MIN_RPM          = 1000.0f;  // Minimum çalışma RPM
static const float    PUMP_RAMP_DOWN_RATE   = 2000.0f; // Durma rampası (rpm/sn)


// ===================== Pinler =====================
#ifndef BLDC_SELECT
#define BLDC_SELECT 3
#endif
#ifndef PRESSURE_PIN
#define PRESSURE_PIN 1
#endif
#ifndef LED_RUN
#define LED_RUN 4
#endif

#ifndef CAN_TX
#define CAN_TX 13
#define CAN_RX 14
#define CAN_RS 38
#endif

#define VESC_ID 10

bool Reset_Motor=false;
float pubRpmCmd =0.0f;

// BasÄ±nÃ§ okuma + kÃ¼Ã§Ã¼k filtre
float pressureLPF = 0;
// ===================== YardÄ±mcÄ±lar =====================
static float readPressureBarADC() {
  long sum=0; for (int i=0;i<PRESSURE_SAMPLES;i++){ sum += analogRead(PRESSURE_PIN); vTaskDelay(pdMS_TO_TICKS(2)); }
  float avg = sum / (float)PRESSURE_SAMPLES;
  float rawBar = (avg-138) * 0.024175f * 1.65; // 1.56f; // mevcut kalibrasyon katsayÄ±n
  if (pressureLPF == 0) pressureLPF = rawBar;
  pressureLPF = 0.25f * rawBar + 0.75f * pressureLPF;
  
  return pressureLPF+5.0;
}

/*static void ledBlinkError() {
  for (int i=0;i<10;i++){ digitalWrite(LED_RUN, HIGH); vTaskDelay(pdMS_TO_TICKS(100));
                          digitalWrite(LED_RUN, LOW);  vTaskDelay(pdMS_TO_TICKS(100)); }
}*/

static void vesc_send_current(float amps) {
  int32_t value = (int32_t)(amps * 1000.0f);
  uint8_t data[4] = {
    (uint8_t)(value>>24),
    (uint8_t)(value>>16),
    (uint8_t)(value>>8), 
    (uint8_t)value 
};
  uint32_t can_id = ((uint32_t)CAN_PACKET_SET_CURRENT << 8) | (uint32_t)VESC_ID;

  // EXT
  { twai_message_t m={0}; 
    m.identifier=can_id; 
    m.extd=1; 
    m.rtr=0; 
    m.data_length_code=4; 
    memcpy(m.data,data,4);
    if (twai_transmit(&m, pdMS_TO_TICKS(10)) == ESP_OK) {
      //Serial.printf("SET_CURRENT EXT sent: %.2f A  (EID=0x%08X)\n", amps, can_id);
    } /*else {
      Serial.println("SET_CURRENT EXT transmit failed!");
    } */
  }
  // STD
  { twai_message_t m={0}; 
    m.identifier=(can_id & 0x7FF); 
    m.extd=0; 
    m.rtr=0; 
    m.data_length_code=4; 
    memcpy(m.data,data,4);
    if (twai_transmit(&m, pdMS_TO_TICKS(10)) == ESP_OK) {
      //Serial.printf("SET_CURRENT STD sent: %.2f A  (SID=0x%03X)\n", amps, (int)(can_id & 0x7FF));
    } /*else {
      Serial.println("SET_CURRENT STD transmit failed!");
    } */
}
}

static void vesc_send_rpm(float rpm) {
  int32_t value = (int32_t)(rpm*PUMP_POLE_PAIRS);
  uint8_t data[4] = {
    (uint8_t)(value>>24),
    (uint8_t)(value>>16),
    (uint8_t)(value>>8),
    (uint8_t)value
  };
  uint32_t can_id = ((uint32_t)CAN_PACKET_SET_RPM << 8) | (uint32_t)VESC_ID;

  // EXT
  { twai_message_t m={0};
    m.identifier=can_id;
    m.extd=1;
    m.rtr=0;
    m.data_length_code=4;
    memcpy(m.data,data,4);
    twai_transmit(&m, pdMS_TO_TICKS(10));
  }
  // STD
  { twai_message_t m={0};
    m.identifier=(can_id & 0x7FF);
    m.extd=0;
    m.rtr=0;
    m.data_length_code=4;
    memcpy(m.data,data,4);
    twai_transmit(&m, pdMS_TO_TICKS(10));
  }
}

static inline void select_pressure_pump(){ digitalWrite(BLDC_SELECT, LOW); }
static inline void select_fill_pump()    { digitalWrite(BLDC_SELECT, HIGH); }

// Basınca göre RPM hesapla (soft landing)
static float calcPressureScaledRpm(float baseRpm, float bar) {
  if (bar < PUMP_SOFT_START_BAR) {
    return baseRpm;  // Tam hız
  }
  if (bar >= PUMP_TARGET_BAR) {
    return 0.0f;     // Hedefe ulaştı, dur
  }
  // Linear interpolasyon: SOFT_START -> TARGET arası
  float range = PUMP_TARGET_BAR - PUMP_SOFT_START_BAR;
  float progress = (bar - PUMP_SOFT_START_BAR) / range;  // 0..1
  float scale = 1.0f - progress;  // 1..0
  float rpm = PUMP_MIN_RPM + (baseRpm - PUMP_MIN_RPM) * scale;
  return fmaxf(rpm, PUMP_MIN_RPM);
}

// Rampalı RPM geçişi (ani değişim yerine yumuşak)
static float rampToTarget(float current, float target, float ratePerSec, uint32_t dtMs) {
  if (dtMs == 0) return current;
  float maxChange = ratePerSec * (dtMs / 1000.0f);
  float diff = target - current;
  if (fabsf(diff) <= maxChange) {
    return target;
  }
  return current + (diff > 0 ? maxChange : -maxChange);
}

// ===================== Durumlar =====================
enum PumpMode { PUMP_IDLE=0, PUMP_SINGLE, PUMP_AUTO, PUMP_FILL, PUMP_FAULT };

typedef struct {
  PumpMode  mode;
  uint32_t  startMs;
  uint32_t  lastCanRxMs;
  uint32_t  fillDurationMs;
  float     fillRpm;
  float     lastCommandRpm;
  float     rampRpm;           // Mevcut rampalı RPM
  uint32_t  rampLastMs;
  float     rampStageTarget;
  uint32_t  rampStageMs;
  bool      useCurrentForStart;
  float     startCurrentA;
  float     startRpmThreshold;
  float     targetRpm;         // Hedef RPM (soft landing sonrası)
  float     actualRpm;         // VESC'e gönderilen son RPM
  bool      inStartup;         // Başlatma modunda mı (akım kontrolü)
  uint32_t  startupBeginMs;    // Başlatma başlangıç zamanı
} PumpState;



// (Telemetri RX: opsiyonel â€” aynen bende bÄ±raktÄ±m, sadece status 9'u logluyor)
static void handle_vesc_rx(const twai_message_t &m, PumpState &st) {
  uint16_t id = m.identifier;
  uint8_t ctrlID = id & 0xFF;
  uint8_t packet = id >> 8;
  if (ctrlID != VESC_ID) return;
  st.lastCanRxMs = millis();
  //int32_t rpm =0;
  //int16_t currentX10 =0.0f;
  //int16_t dutyX1000  =0.0f;
  
  if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5))==pdTRUE) {  
      
    
    switch (packet) {

        // ---------- CAN_PACKET_STATUS (9) ----------
        case 9: {
          int32_t rpmX4 = (m.data[0]<<24)|(m.data[1]<<16)|(m.data[2]<<8)|m.data[3];
          int16_t currentX10 = (m.data[4]<<8)|m.data[5];
          int16_t dutyX1000  = (m.data[6]<<8)|m.data[7];
          int32_t rpm =rpmX4/4;
          float I_motor = currentX10 / 10.0f;         // A
          float duty    = dutyX1000 / 1000.0f;        // 0..1
          float Iin_est = I_motor * duty;             // ~ beklenen giri�Ÿ akÄ±mÄ±
          g_vescStatus.rpm=rpm;
          g_vescStatus.duty=duty;
          g_vescStatus.Im=I_motor;
          //Serial.printf("Check: I_motor=%.2f A, Duty=%.3f, I_in~%.2f A\n", I_motor, duty, Iin_est);
          break;
        }

        // ---------- CAN_PACKET_STATUS_4 (16) ----------
        case 16: {
          int16_t tempFetX10     = (m.data[0]<<8)|m.data[1];
          int16_t tempMotorX10   = (m.data[2]<<8)|m.data[3];
          int16_t currentInX10   = (m.data[4]<<8)|m.data[5];
          int16_t pidPosNowX50   = (m.data[6]<<8)|m.data[7];
          g_vescStatus.Iin = currentInX10/10.0f;
          g_vescStatus.Tfet = tempFetX10/10.0f;
          g_vescStatus.Tmot = tempMotorX10/10.0f;
          //Serial.printf("Status4: TempFET=%.1f Â°C  TempMotor=%.1f Â°C  Iin=%.2f A  PIDpos=%.2f deg\n",
          //              tempFetX10/10.0, tempMotorX10/10.0,
          //              currentInX10/10.0, pidPosNowX50/50.0);
          break;
        }

        // ---------- CAN_PACKET_STATUS_5 (27) ----------
        case 27: {
          int32_t tacho     = (m.data[0]<<24)|(m.data[1]<<16)|(m.data[2]<<8)|m.data[3];
          int16_t vinX10    = (m.data[4]<<8)|m.data[5];
          int16_t reserved  = (m.data[6]<<8)|m.data[7];
          g_vescStatus.vin = vinX10/10.0f;
          g_vescStatus.tacho = tacho;
          //Serial.printf("Status5: Tacho=%ld  Vin=%.2f V\n", tacho, vinX10/10.0);
          break;
        }

        default:
          // DiÄŸer paketler (10, 11 vb.) � Ÿimdilik yoksay
          break;
    }
      
      xSemaphoreGive(g_sharedMutex);
      
  }
}


// ===================== Task =====================
void TaskBLDCPump(void *pvParameters)
{
  (void)pvParameters;
  if (!g_sharedMutex) Shared_Init();
  
  

  pinMode(CAN_RS, OUTPUT);  digitalWrite(CAN_RS, LOW);
  pinMode(BLDC_SELECT, OUTPUT);
 // pinMode(LED_RUN, OUTPUT); digitalWrite(LED_RUN, LOW);


  // --- CAN init ---
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX,(gpio_num_t)CAN_RX,TWAI_MODE_NORMAL);
  twai_timing_config_t  t = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&g, &t, &f);
  if (err!=ESP_OK && err!=ESP_ERR_INVALID_STATE) { kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[PUMP] TWAI install FAIL"); vTaskDelete(NULL); }
  err = twai_start();
  if (err!=ESP_OK && err!=ESP_ERR_INVALID_STATE) { kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[PUMP] TWAI start FAIL"); vTaskDelete(NULL); }

  kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[PUMP] task started (serial-central)");

  PumpState st = {};
  st.mode = PUMP_IDLE;
  st.startMs = 0;
  st.lastCanRxMs = millis();
  st.fillDurationMs = 0;
  st.fillRpm = 0;
  st.lastCommandRpm = 0;
  st.rampRpm = 0;
  st.rampLastMs = millis();
  st.useCurrentForStart = false;
  st.startCurrentA = 0.0f;
  st.startRpmThreshold = PUMP_START_RPM_THRESHOLD;
  st.targetRpm = 0.0f;
  st.actualRpm = 0.0f;
  st.inStartup = false;
  st.startupBeginMs = 0;

  uint32_t lastVescCmdMs = 0;
  uint32_t seenSeq = 0;

  twai_message_t rxmsg;

  for(;;){
    uint32_t now = millis();
    float prevCommandRpm = st.lastCommandRpm;

    // ---- 1) Shared komut tÃ¼ketimi ----
    if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5))==pdTRUE) {
      PumpCommand pc = g_pumpCmd;
      xSemaphoreGive(g_sharedMutex);

      if (pc.seq != seenSeq) { // yeni komut var
        seenSeq = pc.seq;
        switch (pc.cmd) {
          case PUMP_CMD_START:
            if (pc.setRpm > 0) g_pumpRunRpm = pc.setRpm;
            select_pressure_pump();
            st.mode = PUMP_SINGLE; st.startMs = now;
            st.lastCommandRpm = g_pumpRunRpm;
            st.inStartup = false;  // Direkt RPM ile başlat
            st.startupBeginMs = now;
            st.actualRpm = 0.0f;  // Ramp'ı sıfırla
            { char b[64]; snprintf(b,sizeof(b),"[PUMP] START rpm=%.0f", g_pumpRunRpm); kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, b); }
            break;

          case PUMP_CMD_AUTO:
            if (pc.setRpm > 0) g_pumpRunRpm = pc.setRpm;
            select_pressure_pump();
            st.mode = PUMP_AUTO; st.startMs = now;
            st.lastCommandRpm = g_pumpRunRpm;
            st.inStartup = false;  // Direkt RPM ile başlat
            st.startupBeginMs = now;
            st.actualRpm = 0.0f;
            { char b[64]; snprintf(b,sizeof(b),"[PUMP] AUTO rpm=%.0f", g_pumpRunRpm); kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, b); }
            break;

          case PUMP_CMD_STOP:
            st.mode = PUMP_IDLE;
            st.lastCommandRpm = 0.0f;
            st.rampStageTarget = 0.0f;
            st.rampRpm = 0.0f;
            st.inStartup = false;
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[PUMP] STOP");
            break;

          case PUMP_CMD_SET_CURR:
            g_pumpRunRpm = pc.setCurrentA; // eski komut: rpm olarak yorumlanıyor
            st.lastCommandRpm = g_pumpRunRpm;
            st.rampStageTarget = min(PUMP_STEP_RPM, fabsf(st.lastCommandRpm));
            st.rampStageMs = now;
            { char b[64]; snprintf(b,sizeof(b),"[PUMP] RPM=%.0f", g_pumpRunRpm); kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, b); }
            break;

          case PUMP_CMD_SET_RPM:
            g_pumpRunRpm = pc.setRpm;
            st.lastCommandRpm = g_pumpRunRpm;
            st.rampStageTarget = min(PUMP_STEP_RPM, fabsf(st.lastCommandRpm));
            st.rampStageMs = now;
            { char b[64]; snprintf(b,sizeof(b),"[PUMP] RPM=%.0f", g_pumpRunRpm); kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, b); }
            break;

          case PUMP_CMD_FILL:
            select_fill_pump();
            st.mode = PUMP_FILL; st.startMs = now;
            st.fillDurationMs = pc.fillDurationMs;
            st.fillRpm  = pc.fillCurrentA;  // doldurma hÄ±zÄ± rpm olarak kullanÄ±lÄ±yor
            { char b[80]; snprintf(b,sizeof(b),"[PUMP] FILL rpm=%.0f t=%dms", st.fillRpm, st.fillDurationMs);
              kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, b); }
            break;

          case PUMP_CMD_DRAIN:
            select_fill_pump();
            st.mode = PUMP_FILL; st.startMs = now;
            st.fillDurationMs = pc.fillDurationMs;
            st.fillRpm  = -fabsf(pc.fillCurrentA);// - bo�Ÿaltma, rpm ters
            { char b[80]; snprintf(b,sizeof(b),"[PUMP] DRAIN rpm=%.0f t=%dms", st.fillRpm, st.fillDurationMs);
              kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, b); }
            break;

          default: break;
        }
      }
    }
    
    // ---- 2) CAN RX (telemetri) ----
    if (twai_receive(&rxmsg,pdMS_TO_TICKS(100)) == ESP_OK) {
      handle_vesc_rx(rxmsg, st);
    }

    // ---- 4) BasÄ±nÃ§ kontrol / durum makinesi ----
    float bar = readPressureBarADC();
    g_pumpPub.bar=bar;

    AutoShiftReq req{ false, 1, 1000 };

    if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        req.gear_ms = g_autoShiftReq.gear_ms;
        req.repeats = g_autoShiftReq.repeats;
        req.start =  g_autoShiftReq.start;
        
        xSemaphoreGive(g_sharedMutex);
      }
     
      
      
      
    switch (st.mode) {
      case PUMP_IDLE:
        st.lastCommandRpm = 0.0f;
        vTaskDelay(pdMS_TO_TICKS(100));
        select_pressure_pump();
        break;

      case PUMP_SINGLE:
        if (bar >= PUMP_TARGET_BAR) {
          st.mode = PUMP_IDLE;
          kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[PUMP] reached 60 bar -> stop");
        } else if ((now - st.startMs) > PUMP_TIMEOUT_MS) {
          st.mode = PUMP_IDLE; //ledBlinkError();
          kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[PUMP] timeout -> stop");
        } else {
          st.lastCommandRpm = g_pumpRunRpm;
        }
        break;

      case PUMP_AUTO:
        if (bar < PUMP_RESTART_BAR) {
          st.lastCommandRpm = g_pumpRunRpm;
        } else if (bar >= PUMP_TARGET_BAR) {
          st.lastCommandRpm = 0.0f;
        }
        break;

      case PUMP_FILL:
        if ((now - st.startMs) >= st.fillDurationMs) {
          st.mode = PUMP_IDLE;st.lastCommandRpm = 0.0f; vTaskDelay(pdMS_TO_TICKS(200)); select_pressure_pump();
          kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[PUMP] fill/drain done");
        } else {
          st.lastCommandRpm = st.fillRpm;
        }
        break;

      case PUMP_FAULT:
        st.lastCommandRpm = 0.0f; vTaskDelay(pdMS_TO_TICKS(100)); select_pressure_pump();

        break;
    }

    g_pumpPub.mode = st.mode;
    
    // ---- Başlatma modu kontrolü ----
    float currentRpmFeedback = fabsf(g_vescStatus.rpm);
    
    if (st.inStartup && st.lastCommandRpm > 0) {
      // Motor henüz dönmüyor, akım ile başlat
      if (currentRpmFeedback >= PUMP_START_RPM_THRESHOLD) {
        // Motor döndü! RPM kontrole geç
        st.inStartup = false;
        st.actualRpm = currentRpmFeedback;  // Mevcut hızdan devam et
        {
          char msg[64];
          snprintf(msg, sizeof(msg), "[PUMP] RPM reached %.0f -> RPM mode", currentRpmFeedback);
          kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        }
      } else if ((now - st.startupBeginMs) > 3000) {
        // 3 saniye içinde dönmedi, timeout
        st.inStartup = false;
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[PUMP] Startup timeout -> RPM mode");
      } else {
        // Akım göndermeye devam et
        vesc_send_current(PUMP_START_CURRENT_A);
        pubRpmCmd = 0;  // RPM henüz yok
        
        float rpmAbs = fabsf(g_vescStatus.rpm);
        g_pumpPub.rpmCmd = PUMP_START_CURRENT_A;  // Akım değerini göster
        g_pumpPub.rpm = g_vescStatus.rpm;
        
        vTaskDelay(pdMS_TO_TICKS(50));  // Başlatmada daha hızlı döngü
        continue;  // Ana döngünün geri kalanını atla
      }
    }
    
    // ---- Hedef RPM hesapla ----
    float targetRpm = 0.0f;
    
    if (st.lastCommandRpm > 0) {
      // Basınç pompası: soft landing uygula
      if (st.mode == PUMP_SINGLE || st.mode == PUMP_AUTO) {
        targetRpm = calcPressureScaledRpm(st.lastCommandRpm, bar);
      } else {
        // PUMP_FILL veya diğer modlar: direkt RPM
        targetRpm = st.lastCommandRpm;
      }
    } else if (st.lastCommandRpm < 0) {
      // Ters yön (drain): direkt RPM
      targetRpm = st.lastCommandRpm;
    } else {
      targetRpm = 0.0f;
      st.inStartup = false;  // Duruyorsa startup modundan çık
    }
    
    // ---- Rampalı geçiş uygula ----
    uint32_t dtMs = now - st.rampLastMs;
    st.rampLastMs = now;
    
    // Ramp hızını belirle: durma için daha yavaş, çalışma için daha hızlı
    float rampRate = (targetRpm == 0.0f) ? PUMP_RAMP_DOWN_RATE : PUMP_RAMP_RATE_RPM_S * 4;
    st.actualRpm = rampToTarget(st.actualRpm, targetRpm, rampRate, dtMs);
    
    // ---- VESC'e gönder ----
    vesc_send_rpm(st.actualRpm);
    pubRpmCmd = st.actualRpm;

    float rpmAbs = fabsf(g_vescStatus.rpm);
    g_pumpPub.rpmCmd = pubRpmCmd;
    g_pumpPub.rpm = g_vescStatus.rpm;
    

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
    











































































































































































































































































































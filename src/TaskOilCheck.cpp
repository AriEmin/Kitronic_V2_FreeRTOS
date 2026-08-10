#include <Arduino.h>
#include <math.h>
#include "Tasks.h"
#include "Shared.h"

// ------------ Ayarlar (sahada oynat) ------------
#ifndef PRESS_BAR_PER_UNIT
// g_pressure0_V VOLT ise 10.0 kullan; eğer zaten BAR ise 1.0 yap.
#define PRESS_BAR_PER_UNIT 10.0f
#endif

static const float    OIL_LOW_CURRENT_A         = 3.0f;   // 1. kademe
static const uint32_t OIL_LOW_MS                = 1000;   // 1. kademe süre
static const float    OIL_HIGH_CURRENT_A        = 4.5f;   // 2. kademe
static const uint32_t OIL_HIGH_MS               = 800;    // 2. kademe süre

// “Yağ VAR” kriteri (aynı)
static const float    PRESENT_MIN_DP_BAR        = 2.0f;   // toplam artış
static const float    PRESENT_MIN_PEND_BAR      = 5.0f;   // bitiş basıncı

// “YAĞ YOK” (kavitasyon) kriterleri
static const float    EMPTY_MAX_DP_BAR          = 0.15f;  // (0.20'ydi, biraz sıkılaştırdık)
static const float    CAV_I_MIN_A               = 2.0f;   // (2.5'ten düşürüldü)
static const float    CAV_RMS_RATIO_MIN         = 1.18f;  // Irms/Iavg ≥ 1.18 → kavitasyon
static const float    EMPTY_MAX_P_ABS_BAR       = 0.30f;  // p0 & p1 bu barın altı → pratikte boş

// Pompa durduktan sonra izleme
static const uint32_t LEAKDOWN_MS               = 1500;

static inline float barFromUnit(float v) { return v * PRESS_BAR_PER_UNIT; }

static void oc_log(const char* msg) {
  kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
}

static void cmd_pump_set_current(float a) {
  PumpCommand pc{}; pc.cmd = PUMP_CMD_SET_CURR; pc.setCurrentA = a;
  portENTER_CRITICAL(&g_portMux); pc.seq = g_pumpCmd.seq + 1; g_pumpCmd = pc; portEXIT_CRITICAL(&g_portMux);
}
static void cmd_pump_start() { PumpCommand pc{}; pc.cmd = PUMP_CMD_START; portENTER_CRITICAL(&g_portMux); pc.seq = g_pumpCmd.seq + 1; g_pumpCmd = pc; portEXIT_CRITICAL(&g_portMux); }
static void cmd_pump_stop()  { PumpCommand pc{}; pc.cmd = PUMP_CMD_STOP;  portENTER_CRITICAL(&g_portMux); pc.seq = g_pumpCmd.seq + 1; g_pumpCmd = pc; portEXIT_CRITICAL(&g_portMux); }

static void run_stage(float setI_A, uint32_t run_ms, uint8_t stage_id,
                      float& p_now, float& p_max,
                      float& i_sum, float& i2_sum, uint32_t& i_cnt)
{
  cmd_pump_set_current(setI_A);
  vTaskDelay(pdMS_TO_TICKS(50));
  cmd_pump_start();

  uint32_t t0 = millis(), lastPrint = t0;
  for (;;) {
    uint32_t now = millis();
    if (now - t0 >= run_ms) break;

    p_now =g_pumpPub.bar; // barFromUnit(g_pressure0_V);
    if (p_now > p_max) p_max = p_now;

    float iA = g_tele.vescI;
    i_sum += iA; i2_sum += iA * iA; i_cnt++;

    if (now - lastPrint > 200) {
      lastPrint = now;
      char line[96];
      snprintf(line, sizeof(line), "[OIL] stg=%u t=%lu p=%.2fbar I=%.2fA",
               stage_id, (unsigned long)(now - t0), p_now, iA);
      oc_log(line);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  cmd_pump_stop();
}

static void run_oil_check_once() {
  // Başlangıç
  float p0_bar =g_pumpPub.bar; // barFromUnit(g_pressure0_V);
  float p_now  = p0_bar, p_max = p0_bar;

  float i_sum = 0.0f, i2_sum = 0.0f; uint32_t i_cnt = 0;
  uint32_t total_ms = 0;
  uint8_t  stage_used = 0;

  // ---- 1) LOW prime
  stage_used = 1;
  run_stage(OIL_LOW_CURRENT_A, OIL_LOW_MS, stage_used, p_now, p_max, i_sum, i2_sum, i_cnt);
  total_ms += OIL_LOW_MS;
  float p_after_low = p_now;
  float dp_low = p_after_low - p0_bar;

  // Eğer hâlâ dP ~ 0 ise kısa bir HIGH denemesi yap
  if (dp_low < EMPTY_MAX_DP_BAR) {
    stage_used = 2;
    run_stage(OIL_HIGH_CURRENT_A, OIL_HIGH_MS, stage_used, p_now, p_max, i_sum, i2_sum, i_cnt);
    total_ms += OIL_HIGH_MS;
  }

  float p1_bar = p_now;
  float dp_bar = p1_bar - p0_bar;

  // Akım istatistikleri
  float i_avg = (i_cnt > 0) ? (i_sum / (float)i_cnt) : 0.0f;
  float i_rms = (i_cnt > 0) ? sqrtf(i2_sum / (float)i_cnt) : 0.0f;

  // dP/dt
  float dp_rate = (total_ms > 0) ? (dp_bar * 1000.0f / (float)total_ms) : 0.0f;

  // --- Yeni: RMS/AVG oranı
  float rms_ratio = (i_avg > 0.2f) ? (i_rms / i_avg) : 0.0f;

  // Leakdown
  float p_before = p1_bar;
  uint32_t leak_t0 = millis();
  while (millis() - leak_t0 < LEAKDOWN_MS) {
    p_now = g_pumpPub.bar;//barFromUnit(g_pressure0_V);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  float p_after = p_now;
  float leak_rate = (LEAKDOWN_MS > 0) ? ((p_after - p_before) * 1000.0f / (float)LEAKDOWN_MS) : 0.0f;

  // Karar
  bool present = (dp_bar >= PRESENT_MIN_DP_BAR) || (p1_bar >= PRESENT_MIN_PEND_BAR);

  // Boşluk tespitini iki kanallı yap:
    // 1) EMPTY_HARD: mutlak basınçlar taban seviyede ve hareket yok
    bool empty_hard = (!present) &&
                    (p0_bar <= EMPTY_MAX_P_ABS_BAR) &&
                    (p1_bar <= EMPTY_MAX_P_ABS_BAR) &&
                    (dp_bar <= 0.05f) &&
                    (fabsf(dp_rate) <= 0.05f);

    // 2) EMPTY_CAV: dP ≈ 0 ama akım yüksek VEYA akım dalgalanması yüksek
    bool empty_cav = (!present) &&
                    (dp_bar <= EMPTY_MAX_DP_BAR) &&
                    ( (i_avg >= CAV_I_MIN_A) || (rms_ratio >= CAV_RMS_RATIO_MIN) );

    bool empty = empty_hard || empty_cav;

  const char* lvl = "LOW";
    const char* why = "LOW_DP";
    if (present) {
    lvl = "OK";    why = "PRESSURE_OK";
    } else if (empty_hard) {
    lvl = "EMPTY"; why = "NO_P_ABS";
    } else if (empty_cav) {
    lvl = "EMPTY"; why = (rms_ratio >= CAV_RMS_RATIO_MIN) ? "CAV_RMS" : "NO_DP_HIGH_I";
    }
  // Paylaş
  if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    g_oilCheck.running        = false;
    g_oilCheck.present        = present;
    g_oilCheck.p0_bar         = p0_bar;
    g_oilCheck.p1_bar         = p1_bar;
    g_oilCheck.dp_bar         = dp_bar;
    g_oilCheck.t_ms           = total_ms;
    g_oilCheck.i_avg_A        = i_avg;
    g_oilCheck.i_rms_A        = i_rms;
    g_oilCheck.dp_rate_barps  = dp_rate;
    g_oilCheck.leak_bar_per_s = leak_rate;
    g_oilCheck.stage          = stage_used ? stage_used : 3;
    strlcpy(g_oilCheck.level_text, lvl, sizeof(g_oilCheck.level_text));
    strlcpy(g_oilCheck.reason,     why, sizeof(g_oilCheck.reason));
    xSemaphoreGive(g_sharedMutex);
  }

  char line[128];
  snprintf(line, sizeof(line),
           "[OIL] p0=%.2f p1=%.2f dP=%.2f (%.2f bar/s) Iavg=%.2f Irms=%.2f leak=%.2f -> %s (%s)",
           p0_bar, p1_bar, dp_bar, dp_rate, i_avg, i_rms, leak_rate, lvl, why);
  oc_log(line);
  oc_log(present ? "[OIL] RESULT: OIL PRESENT" : (empty ? "[OIL] RESULT: OIL EMPTY" : "[OIL] RESULT: OIL LOW/LEAK"));
}

void TaskOilCheck(void *pvParameters) {
  (void)pvParameters;
  oc_log("[OIL] Auto Oil Check task started.");

  uint32_t seenSeq = 0;
  for (;;) {
    uint32_t req;
    portENTER_CRITICAL(&g_portMux); req = g_oilCheckRequestSeq; portEXIT_CRITICAL(&g_portMux);
    if (req != seenSeq) {
      seenSeq = req;
      if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_oilCheck.running = true; xSemaphoreGive(g_sharedMutex);
      }
      oc_log("[OIL] Running oil check...");
      run_oil_check_once();
    }
    vTaskDelay(pdMS_TO_TICKS(25));
  }
}

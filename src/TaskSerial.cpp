// src/TaskSerial.cpp
#define KEEP_REAL_SERIAL
#include "SerialJSON.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <esp_heap_caps.h>
#include <math.h>
#include "Tasks.h"
#include "Shared.h"
#include "AutoDiag.h"
#include "AutoShiftV2.h"
#include "PistonControl.h"
#include "PistonMonitor.h"
#include "PistonCalib.h"
#include "ValveCurrentControl.h"
#include "Protocol.h"

// DRV8243 fonksiyonları (TaskValveControl.cpp'de tanımlı)
extern void DRV_FaultClear(int idx);
extern void DRV_EnableAll(bool en);

// ======= Ayarlar =======

static const size_t   RX_BUF_MAX   = 1024;   // tek komut JSON boyutu (satır)

// ======= YardÄ±mcÄ±lar =======
static inline int valveIndexByName(const char* s) {
  if (!s) return -1;
  // Projede kullandÄ±ÄŸÄ±n adlandÄ±rma: "N433","N436","N434","N435","N438","N440","N439","N437"
  if (!strcmp(s, "N433")) return 0;
  if (!strcmp(s, "N436")) return 1;
  if (!strcmp(s, "N434")) return 2;
  if (!strcmp(s, "N435")) return 3;
  if (!strcmp(s, "N438")) return 4;
  if (!strcmp(s, "N440")) return 5;
  if (!strcmp(s, "N439")) return 6;
  if (!strcmp(s, "N437")) return 7;
  return -1;
}

static inline void bumpSeq(volatile uint32_t &seq) {
  // seq++ (wrap Ã¶nemli deÄŸil, sadece deÄŸiÅŸmesi yeter)
  portENTER_CRITICAL(&g_portMux);
  seq++;
  portEXIT_CRITICAL(&g_portMux);
}

// ======= JSON GÖNDERİM SİSTEMİ (Mesaj Tiplerine Ayrılmış) =======
// Mesaj tipleri:
// "S" - Sensor: press, temp, hall, tmag (10 Hz)
// "P" - Piston: hold, speed, piston_pos, pc (5 Hz)
// "I" - INA: valf akımları (5 Hz, aktifse)
// "W" - Power: ina226, vesc, pump (2 Hz)
// "T" - Test: qh, leak, motor, vp, pmid sonuçları (event-driven)

static constexpr size_t MSG_BUF_SIZE = 512;   // Normal mesajlar için küçük buffer
static constexpr size_t MSG_BUF_LARGE = 4096; // Test sonuçları için büyük buffer

// Mesaj tipi sayaçları (frekans kontrolü)
static uint32_t g_msgCounter = 0;
static uint8_t  g_txSeq = 0;

// ======= Frame gönderim: USB CDC TX buffer müsaitse non-blocking yaz =======
static void sendFrame(const uint8_t* frame, size_t len) {
  if (len == 0) return;
  if (Serial.availableForWrite() >= (int)len) {
    Serial.write(frame, len);
  }
}

// ======= TİP S: Sensör Verileri (binary) =======
static void sendMsgSensor() {
  kitronic::TelemetrySensor s{};
  s.timestamp_ms = (uint32_t)millis();
  s.p0_bar = (int16_t)roundf(g_pressure0_V * 100.0f);
  s.p1_bar = (int16_t)roundf(g_pressure1_V * 100.0f);
  s.t1_C   = (int16_t)roundf(g_temp1_C * 100.0f);
  s.t2_C   = (int16_t)roundf(g_temp2_C * 100.0f);
  s.h13_mm = (int16_t)roundf(g_piston_1_3_mm * 10.0f);
  s.h57_mm = (int16_t)roundf(g_piston_5_7_mm * 10.0f);
  s.h24_mm = (int16_t)roundf(g_piston_2_4_mm * 10.0f);
  s.h6R_mm = (int16_t)roundf(g_piston_6_R_mm * 10.0f);
  s.hK1_mm = (int16_t)roundf(g_n435_stroke_mm * 10.0f);
  s.hK2_mm = (int16_t)roundf(g_n439_stroke_mm * 10.0f);
  s.tm13 = g_tmagData[TMAG_CH_1_3].valid ? g_tmagData[TMAG_CH_1_3].z : 0;
  s.tm57 = g_tmagData[TMAG_CH_5_7].valid ? g_tmagData[TMAG_CH_5_7].z : 0;
  s.tm24 = g_tmagData[TMAG_CH_2_4].valid ? g_tmagData[TMAG_CH_2_4].z : 0;
  s.tm6R = g_tmagData[TMAG_CH_6_R].valid ? g_tmagData[TMAG_CH_6_R].z : 0;
  s.tk1 = g_tmagData[TMAG_CH_K1_1].valid ? g_tmagData[TMAG_CH_K1_1].z : 0;
  s.tk2 = g_tmagData[TMAG_CH_K1_2].valid ? g_tmagData[TMAG_CH_K1_2].z : 0;
  s.tk3 = g_tmagData[TMAG_CH_K2_1].valid ? g_tmagData[TMAG_CH_K2_1].z : 0;
  s.tk4 = g_tmagData[TMAG_CH_K2_2].valid ? g_tmagData[TMAG_CH_K2_2].z : 0;
  s.calRunning  = g_pistonCalRunning;
  s.calPhase    = g_pistonCalPhase;
  s.calProgress = 0;  // GUI phase'den kendisi hesaplar

  static uint8_t frame[64];
  size_t n = kitronic::encodeTelemetry(frame, sizeof(frame), kitronic::FT_SENSOR, g_txSeq++, &s);
  if (n > 0) sendFrame(frame, n);
}

// ======= TİP P: Piston Kontrol (binary) =======
static void sendMsgPiston() {
  kitronic::TelemetryPiston p{};
  p.timestamp_ms = (uint32_t)millis();
  for (int i = 0; i < PISTON_CHANNEL_COUNT; i++) {
    p.holdDuty[i] = g_pistonHoldDuty[i];
    p.state[i] = g_pistonState[i];
  }
  p.speed[0] = (int16_t)roundf(g_v_5_7_mms * 10.0f);
  p.speed[1] = (int16_t)roundf(g_v_1_3_mms * 10.0f);
  p.speed[2] = (int16_t)roundf(g_v_2_4_mms * 10.0f);
  p.speed[3] = (int16_t)roundf(g_v_6_R_mms * 10.0f);
  p.speed[4] = (int16_t)roundf(g_v_K1_mms * 10.0f);
  p.speed[5] = (int16_t)roundf(g_v_K2_mms * 10.0f);

  static uint8_t frame[64];
  size_t n = kitronic::encodeTelemetry(frame, sizeof(frame), kitronic::FT_PISTON, g_txSeq++, &p);
  if (n > 0) sendFrame(frame, n);
}

// ======= TİP I: INA Valf Akımları (binary) =======
static void sendMsgINA() {
  // Sadece aktif valf varsa gönder
  bool anyActive = false;
  for (int i = 0; i < 8; i++) {
    if (fabsf(g_tele.inaI_mA[i]) > 1.0f || g_valveDutyCounts[i] > 0) {
      anyActive = true;
      break;
    }
  }
  if (!anyActive) return;

  kitronic::TelemetryINA ina{};
  ina.timestamp_ms = (uint32_t)millis();

  static const int dutyMap[8] = {0, 2, 3, 1, 7, 4, 6, 5};
  static const int drvMap[8]  = {0, 1, 1, 0, 2, 3, 2, 3};
  static const int outMap[8]  = {1, 1, 2, 2, 2, 1, 1, 2};
  for (int i = 0; i < 8; i++) {
    ina.v[i]   = (uint16_t)roundf(g_tele.inaV[i] * 10.0f);
    ina.i[i]   = (int16_t)roundf(g_tele.inaI_mA[i]);
    ina.d[i]   = g_valveDutyCounts[dutyMap[i]];

    uint8_t st2 = g_drvLastFault[drvMap[i]].st2;
    uint8_t status = 0;
    if (outMap[i] == 1) {
      if (st2 & 0x80) status = 1;
      if (st2 & 0x0C) status = 2;
    } else {
      if (st2 & 0x40) status = 1;
      if (st2 & 0x03) status = 2;
    }
    ina.drv[i] = status;
  }

  static uint8_t frame[80];
  size_t n = kitronic::encodeTelemetry(frame, sizeof(frame), kitronic::FT_INA, g_txSeq++, &ina);
  if (n > 0) sendFrame(frame, n);
}

// ======= TİP W: Güç Durumu (binary) =======
static void sendMsgPower() {
  kitronic::TelemetryPower pw{};
  pw.timestamp_ms = (uint32_t)millis();
  pw.mainV   = (uint16_t)roundf(g_tele.mainV * 10.0f);
  pw.mainI   = (uint16_t)roundf(g_tele.mainI * 100.0f);
  pw.vescV   = (uint16_t)roundf(g_tele.vescV * 10.0f);
  pw.vescI   = (uint16_t)roundf(g_tele.vescI * 100.0f);
  pw.rpm     = (int16_t)g_vescStatus.rpm;
  pw.vescTf  = (uint8_t)roundf(g_vescStatus.Tfet);
  pw.pumpMode = (uint8_t)g_pumpPub.mode;
  pw.pumpRpm  = (uint16_t)g_pumpPub.rpm;
  pw.pumpBar  = (uint16_t)roundf(g_pumpPub.bar * 100.0f);

  static uint8_t frame[64];
  size_t n = kitronic::encodeTelemetry(frame, sizeof(frame), kitronic::FT_POWER, g_txSeq++, &pw);
  if (n > 0) sendFrame(frame, n);
}

// ======= TİP V: Versiyon / Heartbeat (binary) =======
static void sendMsgVersion() {
  kitronic::TelemetryVersion v{};
  v.major = FW_VERSION_MAJOR;
  v.minor = FW_VERSION_MINOR;
  v.patch = FW_VERSION_PATCH;
  v.freeHeap = (uint16_t)(ESP.getFreeHeap() / 1024);
  v.ocpLatch = g_drvOcpLatch ? 1 : 0;

  static uint8_t frame[32];
  size_t n = kitronic::encodeTelemetry(frame, sizeof(frame), kitronic::FT_VERSION, g_txSeq++, &v);
  if (n > 0) sendFrame(frame, n);
}

// ======= MessagePack yardımcı: FT_RESULT frame gönder =======
static void sendMsgPackResult(const JsonDocument& doc) {
  static uint8_t payload[MSG_BUF_LARGE];
  size_t n = serializeMsgPack(doc, payload, sizeof(payload));
  if (n == 0 || n >= sizeof(payload)) return;
  static uint8_t frame[MSG_BUF_LARGE + 8];
  size_t fn = kitronic::encodeFrame(frame, sizeof(frame), kitronic::FT_RESULT, g_txSeq++, payload, (uint16_t)n);
  if (fn > 0) sendFrame(frame, fn);
}

// ======= MessagePack yardımcı: FT_LOG frame gönder =======
static void sendMsgPackLog(kitronic::MsgCode code, const JsonArray& params) {
  static JsonDocument doc;
  doc.clear();
  doc["c"] = (uint8_t)code;
  doc["p"] = params;
  static uint8_t payload[128];
  size_t n = serializeMsgPack(doc, payload, sizeof(payload));
  if (n == 0 || n >= sizeof(payload)) return;
  static uint8_t frame[160];
  size_t fn = kitronic::encodeFrame(frame, sizeof(frame), kitronic::FT_LOG, g_txSeq++, payload, (uint16_t)n);
  if (fn > 0) sendFrame(frame, fn);
}

// Convenience overload
static void sendMsgPackLog(kitronic::MsgCode code) {
  static JsonDocument doc;
  doc.clear();
  auto params = doc["p"].to<JsonArray>();
  doc["c"] = (uint8_t)code;
  static uint8_t payload[128];
  size_t n = serializeMsgPack(doc, payload, sizeof(payload));
  if (n == 0 || n >= sizeof(payload)) return;
  static uint8_t frame[160];
  size_t fn = kitronic::encodeFrame(frame, sizeof(frame), kitronic::FT_LOG, g_txSeq++, payload, (uint16_t)n);
  if (fn > 0) sendFrame(frame, fn);
}

// ======= TİP T: Test Sonuçları (MessagePack) =======
// Her testi ayrı FT_RESULT frame olarak gönder; autoV2 event-driven.
static void sendMsgTest() {
  static bool prevQhRunning = false;
  static bool prevLeakRunning = false;
  static bool prevMotorRunning = false;
  static bool prevVpRunning = false;
  static bool prevPmidRunning = false;

  bool qhActive = g_quickHealthRes.running || g_quickHealthRes.done;
  bool leakActive = g_leakRes.running || g_leakRes.done;
  bool motorActive = g_motorTestRes.running || g_motorTestRes.done;
  bool vpActive = g_valvePistonRes.running || g_valvePistonRes.done;
  bool pmidActive = g_pistonMidRes.running || g_pistonMidRes.done;

  if (qhActive) {
    bool changed = (g_quickHealthRes.running != prevQhRunning) || g_quickHealthRes.done;
    prevQhRunning = g_quickHealthRes.running;
    if (changed) {
      JsonDocument doc;
      doc["t"] = "qh";
      doc["r"] = g_quickHealthRes.running;
      doc["ok"] = g_quickHealthRes.pass;
      doc["p0"] = roundf(g_quickHealthRes.p0_bar * 10.0f) / 10.0f;
      doc["p1"] = roundf(g_quickHealthRes.p1_bar * 10.0f) / 10.0f;
      doc["dp"] = roundf(g_quickHealthRes.dp_bar * 10.0f) / 10.0f;
      doc["lk"] = roundf(g_quickHealthRes.leak_barps * 1000.0f) / 1000.0f;
      doc["tf"] = g_quickHealthRes.t_fill_ms;
      doc["th"] = g_quickHealthRes.t_hold_ms;
      doc["msg"] = g_quickHealthRes.reason;
      doc["v_ok"] = g_quickHealthRes.valves_ok;
      doc["v_open"] = g_quickHealthRes.valve_open_mask;
      doc["v_sh"] = g_quickHealthRes.valve_short_mask;
      doc["pmp_ok"] = g_quickHealthRes.pump_ok;
      doc["prs_ok"] = g_quickHealthRes.pressure_ok;
      doc["pst_ok"] = g_quickHealthRes.pistons_ok;
      doc["pst_err"] = g_quickHealthRes.piston_err_mask;
      doc["pst_diag"] = g_quickHealthRes.piston_diag;
      sendMsgPackResult(doc);
    }
  }

  if (leakActive) {
    bool changed = (g_leakRes.running != prevLeakRunning) || g_leakRes.done;
    prevLeakRunning = g_leakRes.running;
    if (changed) {
      JsonDocument doc;
      doc["t"] = "leak";
      doc["r"] = g_leakRes.running;
      doc["ok"] = g_leakRes.pass;
      doc["msg"] = g_leakRes.reason;
      doc["p0"] = roundf(g_leakRes.p_start_bar * 10.0f) / 10.0f;
      doc["p1"] = roundf(g_leakRes.p_end_bar * 10.0f) / 10.0f;
      doc["dp"] = roundf(g_leakRes.dp_bar * 10.0f) / 10.0f;
      doc["lk"] = roundf(g_leakRes.leak_barps * 1000.0f) / 1000.0f;
      doc["tf"] = g_leakRes.t_fill_ms;
      doc["ts"] = g_leakRes.t_settle_ms;
      doc["th"] = g_leakRes.t_hold_ms;
      sendMsgPackResult(doc);
    }
  }

  if (motorActive) {
    bool changed = (g_motorTestRes.running != prevMotorRunning) || g_motorTestRes.done;
    prevMotorRunning = g_motorTestRes.running;
    if (changed) {
      JsonDocument doc;
      doc["t"] = "motor_test";
      doc["r"] = g_motorTestRes.running;
      doc["ok"] = g_motorTestRes.pass;
      doc["msg"] = g_motorTestRes.reason;
      doc["fail_mask"] = g_motorTestRes.fail_mask;
      auto rpmArr = doc["rpm"].to<JsonArray>();
      for (int i = 0; i < 5; i++) rpmArr.add(g_motorTestRes.rpm[i]);
      auto dpArr = doc["dp"].to<JsonArray>();
      for (int i = 0; i < 5; i++) dpArr.add(g_motorTestRes.dp_barps[i]);
      auto IArr = doc["I"].to<JsonArray>();
      for (int i = 0; i < 5; i++) IArr.add(g_motorTestRes.I_avg[i]);
      sendMsgPackResult(doc);
    }
  }

  if (vpActive) {
    bool changed = (g_valvePistonRes.running != prevVpRunning) || g_valvePistonRes.done;
    prevVpRunning = g_valvePistonRes.running;
    if (changed) {
      JsonDocument doc;
      doc["t"] = "vp";
      doc["r"] = g_valvePistonRes.running;
      doc["ok"] = g_valvePistonRes.pass;
      doc["msg"] = g_valvePistonRes.reason;
      uint8_t open_mask = 0, short_mask = 0, move_mask = 0, err_mask = 0;
      auto RArr = doc["R"].to<JsonArray>();
      auto inArr = doc["in"].to<JsonArray>();
      auto hoArr = doc["ho"].to<JsonArray>();
      auto dpArr = doc["dp"].to<JsonArray>();
      for (int i = 0; i < 8; i++) {
        RArr.add(g_valvePistonRes.valves[i].R_est);
        inArr.add(g_valvePistonRes.valves[i].inrush_mA);
        hoArr.add(g_valvePistonRes.valves[i].hold_mA);
        dpArr.add(g_valvePistonRes.valves[i].duty_prof);
        if (g_valvePistonRes.valves[i].open_fault) open_mask |= (1u << i);
        if (g_valvePistonRes.valves[i].short_fault) short_mask |= (1u << i);
      }
      doc["vo"] = open_mask;
      doc["vs"] = short_mask;
      auto phArr = doc["ph"].to<JsonArray>();
      for (int i = 0; i < 6; i++) {
        phArr.add(g_valvePistonRes.pistons[i].hall_raw);
        if (g_valvePistonRes.pistons[i].moved) move_mask |= (1u << i);
        if (g_valvePistonRes.pistons[i].err) err_mask |= (1u << i);
      }
      doc["pm"] = move_mask;
      doc["pe"] = err_mask;
      sendMsgPackResult(doc);
    }
  }

  if (pmidActive) {
    bool changed = (g_pistonMidRes.running != prevPmidRunning) || g_pistonMidRes.done;
    prevPmidRunning = g_pistonMidRes.running;
    if (changed) {
      JsonDocument doc;
      doc["t"] = "pm";
      doc["r"] = g_pistonMidRes.running;
      doc["ok"] = g_pistonMidRes.pass;
      doc["msg"] = g_pistonMidRes.reason;
      doc["fail_mask"] = g_pistonMidRes.fail_mask;
      doc["max_drop"] = roundf(g_pistonMidRes.max_drop_bar * 100.0f) / 100.0f;
      doc["current_piston"] = g_pistonMidRes.current_piston;
      doc["remaining_ms"] = g_pistonMidRes.remaining_ms;
      sendMsgPackResult(doc);
    }
  }

  // AutoShiftV2 event-driven: phase, gear, errors, pistonStats değiştiğinde
  static uint8_t  prevAvPhase = 0;
  static uint8_t  prevAvGear = 0;
  static uint8_t  prevAvClutch = 0;
  static uint16_t prevAvFaults = 0;
  static bool     prevAvRunning = false;
  static uint8_t  prevErrorCount = 0;
  static bool     prevPumpTimeout = false;

  if (g_autoShiftV2Pub.running || g_autoShiftV2Pub.phase != 0 || g_autoShiftV2Errors.count > 0) {
    bool phaseChanged = (g_autoShiftV2Pub.phase != prevAvPhase);
    bool runningChanged = (g_autoShiftV2Pub.running != prevAvRunning);
    bool gearChanged = (g_autoShiftV2Pub.currentGear != prevAvGear || g_autoShiftV2Pub.targetGear != 0 /* compare later */);
    bool clutchChanged = (g_autoShiftV2Pub.clutch != prevAvClutch);
    bool faultsChanged = (g_autoShiftV2Pub.faults != prevAvFaults);
    bool errorsChanged = (g_autoShiftV2Errors.count != prevErrorCount);
    bool pumpTimeoutChanged = (g_leakRecheckNeeded != prevPumpTimeout);
    bool phaseDone = !g_autoShiftV2Pub.running && (g_autoShiftV2Pub.phase == PHASE_COMPLETED || g_autoShiftV2Pub.phase == PHASE_ERROR);

    if (phaseChanged || runningChanged || gearChanged || clutchChanged || faultsChanged ||
        errorsChanged || pumpTimeoutChanged || phaseDone) {
      prevAvPhase = g_autoShiftV2Pub.phase;
      prevAvRunning = g_autoShiftV2Pub.running;
      prevAvGear = g_autoShiftV2Pub.currentGear;
      prevAvClutch = g_autoShiftV2Pub.clutch;
      prevAvFaults = g_autoShiftV2Pub.faults;
      prevErrorCount = g_autoShiftV2Errors.count;
      prevPumpTimeout = g_leakRecheckNeeded;

      JsonDocument doc;
      doc["t"] = "autoV2";
      doc["running"] = g_autoShiftV2Pub.running;
      doc["phase"] = (uint8_t)g_autoShiftV2Pub.phase;
      doc["completed"] = phaseDone;
      doc["currentGear"] = (uint8_t)g_autoShiftV2Pub.currentGear;
      doc["targetGear"] = (uint8_t)g_autoShiftV2Pub.targetGear;
      doc["clutch"] = (uint8_t)g_autoShiftV2Pub.clutch;
      doc["stepIdx"] = g_autoShiftV2Pub.stepIdx;
      doc["repeatIdx"] = g_autoShiftV2Pub.repeatIdx;
      doc["pressure"] = roundf(g_autoShiftV2Pub.pressure * 10.0f) / 10.0f;
      doc["faults"] = g_autoShiftV2Pub.faults;
      doc["elapsedMs"] = g_autoShiftV2Pub.elapsedMs;
      doc["pumpTimeout"] = g_leakRecheckNeeded;
      doc["ok"] = phaseDone && (g_autoShiftV2Errors.count == 0) && (g_autoShiftV2Pub.faults == 0);
      doc["msg"] = doc["ok"] ? "OK" : "FAIL";

      auto ps = doc["pistonState"].to<JsonArray>();
      for (int i = 0; i < 4; i++) ps.add(g_autoShiftV2Pub.pistonState[i]);

      doc["errorCount"] = g_autoShiftV2Errors.count;
      doc["skippedGears"] = g_autoShiftV2Errors.skippedGears;
      doc["faultyGears"] = g_autoShiftV2Errors.faultyGearsMask;

      if (!g_autoShiftV2Pub.running && g_autoShiftV2Errors.count > 0) {
        auto errs = doc["errors"].to<JsonArray>();
        for (int i = 0; i < g_autoShiftV2Errors.count && i < 10; i++) {
          auto e = errs.add<JsonObject>();
          e["gear"] = g_autoShiftV2Errors.entries[i].gear;
          e["piston"] = g_autoShiftV2Errors.entries[i].pistonIdx;
          e["repeat"] = g_autoShiftV2Errors.entries[i].repeatIdx;
          e["fault"] = g_autoShiftV2Errors.entries[i].faultType;
          e["time"] = g_autoShiftV2Errors.entries[i].timestampMs;
          e["hall"] = g_autoShiftV2Errors.entries[i].hallValue;
          e["emin"] = g_autoShiftV2Errors.entries[i].expectedMin;
          e["emax"] = g_autoShiftV2Errors.entries[i].expectedMax;
          e["fd"] = g_autoShiftV2Errors.entries[i].faultDetail;
        }
      }

      if (phaseDone) {
        static const char* PSTAT_NAMES[4] = {"P5-7", "P1-3", "P2-4", "P6-R"};
        auto pmon = doc["pistonStats"].to<JsonArray>();
        for (int i = 0; i < 4; i++) {
          const PerPistonStats& ps = g_pistonStats[i];
          auto pm = pmon.add<JsonObject>();
          pm["name"] = PSTAT_NAMES[i];
          pm["omov"] = ps.openMoves;
          pm["cmov"] = ps.closeMoves;
          pm["aoms"] = (ps.openMoves > 0) ? (ps.totalOpenMs / ps.openMoves) : 0;
          pm["acms"] = (ps.closeMoves > 0) ? (ps.totalCloseMs / ps.closeMoves) : 0;
          pm["moms"] = ps.maxOpenMs;
          pm["mcms"] = ps.maxCloseMs;
          pm["adpo"] = (ps.openMoves > 0) ? (ps.totalPressDropOpen / ps.openMoves) : 0.0f;
          pm["adpc"] = (ps.closeMoves > 0) ? (ps.totalPressDropClose / ps.closeMoves) : 0.0f;
          pm["mdpo"] = ps.maxPressDropOpen;
          pm["mdpc"] = ps.maxPressDropClose;
          pm["slo"] = ps.slowOpenCount;
          pm["slc"] = ps.slowCloseCount;
        }
      }
      sendMsgPackResult(doc);
    }
  }
}

// ======= ANA TELEMETRİ GÖNDERİCİ =======
static void sendTelemetryMultiMsg() {
  // Valf diagnostik testi sırasında normal telemetri duraksatılır
  if (g_valveDiagRunning) return;

  g_msgCounter++;

  // Her çağrıda sensör gönder (10 Hz)
  sendMsgSensor();

  // Her 2 çağrıda piston gönder (5 Hz)
  if ((g_msgCounter % 2) == 0) {
    sendMsgPiston();
    sendMsgINA();
  }

  // Her 5 çağrıda güç gönder (2 Hz)
  if ((g_msgCounter % 5) == 0) {
    sendMsgPower();
  }

  // Her 10 çağrıda versiyon/heartbeat gönder (1 Hz)
  if ((g_msgCounter % 10) == 0) {
    sendMsgVersion();
  }

  // Her çağrıda test sonuçlarını kontrol et
  sendMsgTest();

  // Her 50 çağrıda (5 saniyede) heap durumunu logla (fragmentation tespiti)
  if ((g_msgCounter % 50) == 0) {
    size_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    size_t minHeap  = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
    if (freeHeap < 40000 || minHeap < 20000) {
      JsonDocument hdoc;
      auto p = hdoc["p"].to<JsonArray>();
      p.add((unsigned)freeHeap);
      p.add((unsigned)minHeap);
      sendMsgPackLog(kitronic::MsgCode::HEAP_WARNING, p);
    }
  }
}

// Eski fonksiyon adını koru (geriye uyumluluk)
#define sendTelemetryJSON sendTelemetryMultiMsg

// ======= ESKİ sendTelemetryJSON KODUNU YORUM OLARAK SAKLA =======
#if 0  // Eski tek-büyük-JSON sistemi (devre dışı)
static void sendTelemetryJSON_OLD() {
  uint8_t pistonStateSnap[PISTON_CHANNEL_COUNT] = {0};
  uint16_t holdDutySnap[PISTON_CHANNEL_COUNT] = {0};
  float holdJitterSnap[PISTON_CHANNEL_COUNT] = {0};
  uint8_t holdWarnSnap[PISTON_CHANNEL_COUNT] = {0};
  float holdErrSnap[PISTON_CHANNEL_COUNT] = {0};
  bool calibFlags[PISTON_CHANNEL_COUNT] = {false};
  PistonRuntimeState pcRt[PISTON_CHANNEL_COUNT]{};
  PressureGroupState pgSnap[2]{};
  PistonCalibProgress calibProg{};
  QuickHealthResult qhSnap{};
  LeakTestResult leakSnap{};
  MotorPumpTestResult motorSnap{};
  ValvePistonResult vpSnap{};
  PistonMidTestResult pmidSnap{};
  // Yayın kontrolü (test bitince 5 kez daha gönder, sonra dur)
  static uint64_t qhSigPrev = 0;
  static uint32_t qhHoldCnt = 0;
  static uint64_t leakSigPrev = 0;
  static uint32_t leakHoldCnt = 0;
  static uint64_t motorSigPrev = 0;
  static uint32_t motorHoldCnt = 0;
  static uint64_t vpSigPrev = 0;
  static uint32_t vpHoldCnt = 0;
  static uint64_t pmidSigPrev = 0;
  static uint32_t pmidHoldCnt = 0;

  auto hashReason = [](const char* s) -> uint32_t {
    if (!s) return 0;
    uint32_t h = 2166136261u;
    for (int i = 0; s[i] != 0 && i < 24; ++i) {
      h ^= (uint8_t)s[i];
      h *= 16777619u;
    }
    return h;
  };
  auto sig_qh = [&](const QuickHealthResult& qh) -> uint64_t {
    if (!(qh.running || qh.done)) return 0;
    uint32_t r = hashReason(qh.reason);
    uint32_t p = ((uint32_t)(qh.p0_bar * 100.0f) ^ (uint32_t)(qh.p1_bar * 100.0f) ^ (uint32_t)(qh.dp_bar * 100.0f));
    uint32_t t = ((uint32_t)qh.t_fill_ms ^ ((uint32_t)qh.t_hold_ms << 1));
    return ((uint64_t)r << 32) | (uint64_t)(p ^ t);
  };
  auto sig_leak = [&](const LeakTestResult& l) -> uint64_t {
    if (!(l.running || l.done)) return 0;
    uint32_t r = hashReason(l.reason);
    uint32_t p = ((uint32_t)(l.p_base_bar * 100.0f) ^ (uint32_t)(l.p_end_bar * 100.0f) ^ (uint32_t)(l.dp_bar * 100.0f));
    uint32_t t = ((uint32_t)l.t_fill_ms ^ (uint32_t)l.t_hold_ms ^ (uint32_t)l.t_settle_ms);
    return ((uint64_t)r << 32) | (uint64_t)(p ^ t);
  };
  auto sig_motor = [&](const MotorPumpTestResult& m) -> uint64_t {
    if (!(m.running || m.done)) return 0;
    uint32_t r = hashReason(m.reason);
    uint32_t p = 0;
    for (int i=0;i<5;i++){
      p ^= (uint32_t)(m.rpm[i]) ^ (uint32_t)(m.dp_barps[i] * 100.0f);
    }
    return ((uint64_t)r << 32) | (uint64_t)p;
  };
  auto sig_vp = [&](const ValvePistonResult& v) -> uint64_t {
    if (!(v.running || v.done)) return 0;
    uint32_t r = hashReason(v.reason);
    uint32_t p = 0;
    for (int i=0;i<8;i++){
      p ^= (uint32_t)(v.valves[i].open_fault) ^ ((uint32_t)(v.valves[i].short_fault)<<1);
    }
    return ((uint64_t)r << 32) | (uint64_t)p;
  };
  auto sig_pmid = [&](const PistonMidTestResult& m) -> uint64_t {
    if (!(m.running || m.done)) return 0;
    uint32_t r = hashReason(m.reason);
    uint32_t p = ((uint32_t)m.fail_mask << 16) ^ (uint32_t)(m.max_drop_bar * 100.0f);
    return ((uint64_t)r << 32) | (uint64_t)p;
  };



  // Snapshot al
  if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    mainV = g_tele.mainV;  mainI = g_tele.mainI;
    vescV = g_tele.vescV;  vescI = g_tele.vescI;
    for (int i=0;i<8;i++){ inaV[i]=g_tele.inaV[i]; inaI[i]=g_tele.inaI_mA[i]; duty[i]=g_valveDutyCounts[i]; }
    vs = g_vescStatus;
    ps = g_pumpPub;
     s = g_autoStepDiag;
    motorSnap = g_motorTestRes;
    for(int i=0;i<PISTON_CHANNEL_COUNT;i++){
      hallRaw[i] = g_pistonHallRaw[i];
      manualRefSnap[i] = g_pistonManualRef[i];
      pistonStateSnap[i] = g_pistonState[i];
      holdDutySnap[i] = g_pistonHoldDuty[i];
      holdJitterSnap[i] = g_pistonHoldJitter[i];
      holdWarnSnap[i] = g_pistonHoldWarn[i];
      holdErrSnap[i] = g_pistonHoldError[i];
      calibFlags[i] = g_pistonCalibData[i].calibrated;
    }
    qhSnap = g_quickHealthRes;
    leakSnap = g_leakRes;
    motorSnap = g_motorTestRes;
    vpSnap = g_valvePistonRes;
    pmidSnap = g_pistonMidRes;
    xSemaphoreGive(g_sharedMutex);
  } else {
    memset(inaV, 0, sizeof(inaV));
    memset(inaI, 0, sizeof(inaI));
    memset(duty, 0, sizeof(duty));
    memset(&vs, 0, sizeof(vs));
    memset(&ps, 0, sizeof(ps));
    mainV = mainI = vescV = vescI = 0;
    memset(&vpSnap, 0, sizeof(vpSnap));
    for(int i=0;i<PISTON_CHANNEL_COUNT;i++){
      hallRaw[i] = 0.0f;
      memset(&manualRefSnap[i], 0, sizeof(PistonManualReference));
      pistonStateSnap[i] = 0;
    }
  }

  PistonControl_TelemetrySnapshot(pcRt, pgSnap);
  PistonControl_GetCalibProgress(calibProg);

  // Yayın kontrol: yeni sonuç geldiyse sayaçları 5'e kur
  uint64_t qhSigCur = sig_qh(qhSnap);
  if (qhSigCur != qhSigPrev && (qhSnap.running || qhSnap.done)) {
    qhSigPrev = qhSigCur;
    qhHoldCnt = 5;
  }
  uint64_t leakSigCur = sig_leak(leakSnap);
  if (leakSigCur != leakSigPrev && (leakSnap.running || leakSnap.done)) {
    leakSigPrev = leakSigCur;
    leakHoldCnt = 5;
  }
  uint64_t motorSigCur = sig_motor(motorSnap);
  if (motorSigCur != motorSigPrev && (motorSnap.running || motorSnap.done)) {
    motorSigPrev = motorSigCur;
    motorHoldCnt = 5;
  }
  // Motor test yayın kontrolü basit: sadece done/running ise gönder
  // Valve/Piston yayın kontrolü
  uint64_t vpSigCur = sig_vp(vpSnap);
  if (vpSigCur != vpSigPrev && (vpSnap.running || vpSnap.done)) {
    vpSigPrev = vpSigCur;
    vpHoldCnt = 5;
  }
  uint64_t pmidSigCur = sig_pmid(pmidSnap);
  if (pmidSigCur != pmidSigPrev && (pmidSnap.running || pmidSnap.done)) {
    pmidSigPrev = pmidSigCur;
    pmidHoldCnt = 5;
  }

  doc["t"] = (uint32_t)millis();

  auto press = doc["press"].to<JsonObject>();
  press["p0"] = round(g_pressure0_V * 10.0) / 10.0;  // 1 decimal precision
  press["p1"] = round(g_pressure1_V * 10.0) / 10.0;

  auto temp = doc["temp"].to<JsonObject>();
  temp["t1"] = round(g_temp1_C * 10.0) / 10.0;
  temp["t2"] = round(g_temp2_C * 10.0) / 10.0;

  auto hall = doc["hall"].to<JsonObject>();
  hall["1_3"] = round(g_piston_1_3_mm * 10.0) / 10.0;
  hall["5_7"] = round(g_piston_5_7_mm * 10.0) / 10.0;
  hall["2_4"] = round(g_piston_2_4_mm * 10.0) / 10.0;
  hall["6_R"] = round(g_piston_6_R_mm * 10.0) / 10.0;
  hall["K1"] = round(g_n435_stroke_mm * 10.0) / 10.0;
  hall["K2"] = round(g_n439_stroke_mm * 10.0) / 10.0;

  const char* pistonNames[PISTON_CHANNEL_COUNT] = {"5_7","1_3","2_4","6_R"};
  
  // Test çalışıyor mu kontrol et (JSON boyutunu azaltmak için)
  bool anyTestRunning = qhSnap.running || leakSnap.running || motorSnap.running || vpSnap.running || pmidSnap.running;

  // Test dışında hall_raw ve piston_pos gönder
  if (!anyTestRunning) {
    auto hallRawObj = doc["hall_raw"].to<JsonObject>();
    hallRawObj[pistonNames[0]] = round(hallRaw[PISTON_5_7] * 10.0) / 10.0;
    hallRawObj[pistonNames[1]] = round(hallRaw[PISTON_1_3] * 10.0) / 10.0;
    hallRawObj[pistonNames[2]] = round(hallRaw[PISTON_2_4] * 10.0) / 10.0;
    hallRawObj[pistonNames[3]] = round(hallRaw[PISTON_6_R] * 10.0) / 10.0;

    // K1/K2 ham hall voltajlari (3 sensor/piston)
    auto k1Raw = doc["k1_raw"].to<JsonArray>();
    k1Raw.add(round(g_n435_hall_V[0] * 1000.0) / 1000.0);
    k1Raw.add(round(g_n435_hall_V[1] * 1000.0) / 1000.0);
    k1Raw.add(round(g_n435_hall_V[2] * 1000.0) / 1000.0);

    auto k2Raw = doc["k2_raw"].to<JsonArray>();
    k2Raw.add(round(g_n439_hall_V[0] * 1000.0) / 1000.0);
    k2Raw.add(round(g_n439_hall_V[1] * 1000.0) / 1000.0);
    k2Raw.add(round(g_n439_hall_V[2] * 1000.0) / 1000.0);

    // TMAG5173 manyetik sensör verileri
    // Vites pistonları için sadece Z ekseni
    auto tmagGear = doc["tmag_gear"].to<JsonObject>();
    tmagGear["1_3"] = g_tmagData[TMAG_CH_1_3].valid ? g_tmagData[TMAG_CH_1_3].z : 0;
    tmagGear["5_7"] = g_tmagData[TMAG_CH_5_7].valid ? g_tmagData[TMAG_CH_5_7].z : 0;
    tmagGear["2_4"] = g_tmagData[TMAG_CH_2_4].valid ? g_tmagData[TMAG_CH_2_4].z : 0;
    tmagGear["6_R"] = g_tmagData[TMAG_CH_6_R].valid ? g_tmagData[TMAG_CH_6_R].z : 0;

    // Kavrama pistonları için sadece Z (2 sensör/piston, 10mm aralıklı)
    // tmag_k1: [z1, z2], tmag_k2: [z1, z2] - kompakt format
    auto tmagK1 = doc["tmag_k1"].to<JsonArray>();
    tmagK1.add(g_tmagData[TMAG_CH_K1_1].valid ? g_tmagData[TMAG_CH_K1_1].z : 0);
    tmagK1.add(g_tmagData[TMAG_CH_K1_2].valid ? g_tmagData[TMAG_CH_K1_2].z : 0);

    auto tmagK2 = doc["tmag_k2"].to<JsonArray>();
    tmagK2.add(g_tmagData[TMAG_CH_K2_1].valid ? g_tmagData[TMAG_CH_K2_1].z : 0);
    tmagK2.add(g_tmagData[TMAG_CH_K2_2].valid ? g_tmagData[TMAG_CH_K2_2].z : 0);

    auto pistonStateObj = doc["piston_pos"].to<JsonObject>();
    pistonStateObj[pistonNames[0]] = pistonStateSnap[PISTON_5_7];
    pistonStateObj[pistonNames[1]] = pistonStateSnap[PISTON_1_3];
    pistonStateObj[pistonNames[2]] = pistonStateSnap[PISTON_2_4];
    pistonStateObj[pistonNames[3]] = pistonStateSnap[PISTON_6_R];
  }

  // Test çalışırken gereksiz alanları gönderme (buffer overflow önleme)
  if (!anyTestRunning) {
    auto holdObj = doc["hold"].to<JsonObject>();
    for (int i=0;i<PISTON_CHANNEL_COUNT;i++){
      auto h = holdObj[pistonNames[i]].to<JsonObject>();
      h["duty"] = holdDutySnap[i];
      h["jit"]  = holdJitterSnap[i];
      h["warn"] = (bool)holdWarnSnap[i];
      h["err"]  = holdErrSnap[i];
    }

    // Hall/piston örnek (kullandığın kadarını aç)
    //g_v_1_3_mms = v_13; g_v_5_7_mms = v_57; g_v_2_4_mms = v_24; g_v_6_R_mms = v_6R;
    auto speed = doc["speed"].to<JsonObject>();
    speed["1_3"] = round(g_v_1_3_mms * 10.0) / 10.0;
    speed["5_7"] = round(g_v_5_7_mms * 10.0) / 10.0;
    speed["2_4"] = round(g_v_2_4_mms * 10.0) / 10.0;
    speed["6_R"] = round(g_v_6_R_mms * 10.0) / 10.0;
    speed["K1"] = round(g_v_K1_mms * 10.0) / 10.0;
    speed["K2"] = round(g_v_K2_mms * 10.0) / 10.0;

    auto manualRefObj = doc["manualRef"].to<JsonObject>();
    for (int i=0;i<PISTON_CHANNEL_COUNT;i++){
      auto refEntry = manualRefObj[pistonNames[i]].to<JsonObject>();
      uint8_t mask = manualRefSnap[i].validMask;
      if (mask & (1u << PISTON_REF_CLOSED)) refEntry["closed"] = round(manualRefSnap[i].raw[PISTON_REF_CLOSED] * 10.0) / 10.0;
      if (mask & (1u << PISTON_REF_MID))    refEntry["mid"]    = round(manualRefSnap[i].raw[PISTON_REF_MID] * 10.0) / 10.0;
      if (mask & (1u << PISTON_REF_OPEN))   refEntry["open"]   = round(manualRefSnap[i].raw[PISTON_REF_OPEN] * 10.0) / 10.0;
    }

    // PC (Piston Controller) state - sadece test dışında gönder
    auto pc = doc["pc"].to<JsonObject>();
    JsonArray pcx = pc["x"].to<JsonArray>();
    JsonArray pce = pc["e"].to<JsonArray>();
    JsonArray pcv = pc["v"].to<JsonArray>();
    JsonArray pcu = pc["u"].to<JsonArray>();
    JsonArray pcs = pc["st"].to<JsonArray>();
    JsonArray pcc = pc["cal"].to<JsonArray>();
    for (int i=0;i<PISTON_CHANNEL_COUNT;i++){
      pcx.add(round(pcRt[i].x_filt * 10.0) / 10.0);
      pce.add(round(pcRt[i].e * 10.0) / 10.0);
      pcv.add(round(pcRt[i].v_est * 10.0) / 10.0);
      pcu.add(round(pcRt[i].u_cmd * 10.0) / 10.0);
      pcs.add(pcRt[i].state);
      pcc.add(calibFlags[i]);
    }
    auto pg = pc["pg"].to<JsonObject>();
    pg["p1"] = round(pgSnap[0].p_meas * 10.0) / 10.0;
    pg["p2"] = round(pgSnap[1].p_meas * 10.0) / 10.0;
    pg["r1"] = round(pgSnap[0].p_ref * 10.0) / 10.0;
    pg["r2"] = round(pgSnap[1].p_ref * 10.0) / 10.0;
    pg["u1"] = round(pgSnap[0].cmd * 10.0) / 10.0;
    pg["u2"] = round(pgSnap[1].cmd * 10.0) / 10.0;
    if (calibProg.running) {
      auto cj = pc["calib"].to<JsonObject>();
      cj["p"] = calibProg.piston;
      cj["step"] = calibProg.step;
      if (calibProg.err[0]) cj["err"] = calibProg.err;
    }
  }  // if (!anyTestRunning) sonu 

  // Test sırasında INA219'u kaldır (buffer overflow önleme)
  if (!anyTestRunning) {
    // INA219 sirası: 0 N433,1 N434,2 N435,3 N436,4 N437,5 N438,6 N439,7 N440
    // PWM/duty dizisi sirası (TaskValveControl): 0 N433,1 N436,2 N434,3 N435,4 N438,5 N440,6 N439,7 N437
    // DRV mapping: DRV1->N433,N436  DRV2->N434,N435  DRV3->N438,N440  DRV4->N439,N437
    
    // g_drvLastFault kullan - fault tespit edildiğinde kaydedilen değerler
    // (DRV_GetAllStatus kullanma - fault clear sonrası sıfırlanıyor)
    
    auto ina = doc["ina"].to<JsonArray>();
    struct Map { const char* name; int dutyIdx; int drvIdx; int outIdx; };  // outIdx: 1=OUT1, 2=OUT2
    static const Map map[8] = {
      {"N433", 0, 0, 1},  // DRV1 Out1
      {"N434", 2, 1, 1},  // DRV2 Out1
      {"N435", 3, 1, 2},  // DRV2 Out2
      {"N436", 1, 0, 2},  // DRV1 Out2
      {"N437", 7, 2, 2},  // DRV3 Out2
      {"N438", 4, 3, 1},  // DRV4 Out1
      {"N439", 6, 2, 1},  // DRV3 Out1
      {"N440", 5, 3, 2},  // DRV4 Out2
    };
    for (int i=0;i<8;i++){
      auto ch = ina.add<JsonObject>();
      ch["ch"] = map[i].name;
      ch["V"]  = round(inaV[i] * 10.0) / 10.0;  // 0.1V precision
      ch["I"]  = round(inaI[i] * 10.0) / 10.0;  // 0.1mA precision
      ch["duty"] = duty[map[i].dutyIdx];
      
      // DRV8243 durum: per-output kontrol
      uint8_t status1 = g_drvLastFault[map[i].drvIdx].st2;
      int status = 0;  // OK
      if (map[i].outIdx == 1) {
        // OUT1: OLA1 (bit7=0x80), OCP_H1/L1 (bit3,2=0x0C)
        if (status1 & 0x80) status = 1;  // OLA1 -> Open Load
        if (status1 & 0x0C) status = 2;  // OCP_H1/L1 -> Short Circuit
      } else {
        // OUT2: OLA2 (bit6=0x40), OCP_H2/L2 (bit1,0=0x03)
        if (status1 & 0x40) status = 1;  // OLA2 -> Open Load
        if (status1 & 0x03) status = 2;  // OCP_H2/L2 -> Short Circuit
      }
      ch["drv"] = status;
    }
  }  // !anyTestRunning
  
  // INA226 - test sırasında kaldır
  if (!anyTestRunning) {
    auto pwr = doc["ina226"].to<JsonObject>();
  auto main = pwr["main"].to<JsonObject>();
  main["V"] = round(mainV * 10.0) / 10.0; 
  main["I"] = round(mainI * 100.0) / 100.0;  // 0.01A precision
  auto vesc = pwr["vesc"].to<JsonObject>();
    vesc["V"] = round(vescV * 10.0) / 10.0; 
    vesc["I"] = round(vescI * 100.0) / 100.0;

    // VESC status
    auto vsj = doc["vesc_status"].to<JsonObject>();
    vsj["rpm"] = (int)vs.rpm;  // RPM integer yeter
    vsj["duty"]= round(vs.duty * 10.0) / 10.0;
    vsj["Im"]  = round(vs.Im * 10.0) / 10.0;
    vsj["Iin"] = round(vs.Iin * 10.0) / 10.0;
    vsj["Tfet"]= round(vs.Tfet * 10.0) / 10.0;
    vsj["Tmot"]= round(vs.Tmot * 10.0) / 10.0;
    vsj["vin"] = round(vs.vin * 10.0) / 10.0;
    vsj["tacho"]=(int)vs.tacho;  // Tacho integer
  }  // !anyTestRunning
  
  // DRV OCP kilit durumu (her zaman gönder - kritik güvenlik)
  doc["ocpLatch"] = (bool)g_drvOcpLatch;

  // Pompa (her zaman gönder - test için kritik)
  auto pump = doc["pump"].to<JsonObject>();
  pump["mode"] = (int)ps.mode;
  pump["Icmd"] = round(ps.Icmd * 100.0) / 100.0;
  pump["rpm_cmd"] = (int)ps.rpmCmd;
  pump["rpm"]  = (int)ps.rpm;
  pump["bar"]  = round(ps.bar * 10.0) / 10.0;

  // Oil check - test sırasında kaldır
  if (!anyTestRunning) {
    bool oilActive = g_oilCheck.running || g_oilCheck.stage != 0 ||
                     fabsf(g_oilCheck.p0_bar) > 0.01f || fabsf(g_oilCheck.p1_bar) > 0.01f ||
                     g_oilCheck.reason[0] != '\0';
    if (oilActive) {
    auto oil = doc["oil"].to<JsonObject>();
    oil["run"]   = g_oilCheck.running;
    oil["ok"]    = g_oilCheck.present;
    oil["lvl"]   = g_oilCheck.level_text;
    oil["reason"]= g_oilCheck.reason;
    oil["stage"] = g_oilCheck.stage;

    oil["p0"]    = round(g_oilCheck.p0_bar * 10.0) / 10.0;
    oil["p1"]    = round(g_oilCheck.p1_bar * 10.0) / 10.0;
    oil["dp"]    = round(g_oilCheck.dp_bar * 10.0) / 10.0;
    oil["dpr"]   = round(g_oilCheck.dp_rate_barps * 100.0) / 100.0;  // 0.01 bar/s
    oil["tms"]   = g_oilCheck.t_ms;

    oil["Iavg"]  = round(g_oilCheck.i_avg_A * 100.0) / 100.0;
    oil["Irms"]  = round(g_oilCheck.i_rms_A * 100.0) / 100.0;

    oil["leak"]  = round(g_oilCheck.leak_bar_per_s * 100.0) / 100.0;
    oil["Irr"] = (g_oilCheck.i_avg_A > 0.2f) ? round((g_oilCheck.i_rms_A / g_oilCheck.i_avg_A) * 100.0) / 100.0 : 0.0f;
    }  // oilActive
  }  // !anyTestRunning

  // Hızlı Sağlık Kontrolü sonucu (minimize JSON, kısa key isimleri)
  bool sendQh = qhSnap.running || (qhSnap.done && qhHoldCnt > 0);
  if (sendQh) {
    auto jqh = doc["qh"].to<JsonObject>();
    jqh["r"]   = qhSnap.running;      // running
    jqh["ok"]  = qhSnap.pass;         // genel sonuç
    jqh["p0"]  = round(qhSnap.p0_bar * 10.0) / 10.0;
    jqh["p1"]  = round(qhSnap.p1_bar * 10.0) / 10.0;
    jqh["dp"]  = round(qhSnap.dp_bar * 10.0) / 10.0;
    jqh["lk"]  = round(qhSnap.leak_barps * 100.0) / 100.0;  // 0.01 bar/s
    jqh["tf"]  = qhSnap.t_fill_ms;    // doldurma süresi
    jqh["th"]  = qhSnap.t_hold_ms;    // hold süresi
    jqh["msg"] = qhSnap.reason;       // "OK", "LEAK", "VALVE" vb.

    // Yeni alt-birim alanları
    jqh["v_ok"]    = qhSnap.valves_ok;
    jqh["v_open"]  = qhSnap.valve_open_mask;
    jqh["v_sh"]    = qhSnap.valve_short_mask;
    jqh["pmp_ok"]  = qhSnap.pump_ok;
    jqh["prs_ok"]  = qhSnap.pressure_ok;
    jqh["pst_ok"]  = qhSnap.pistons_ok;
    jqh["pst_err"] = qhSnap.piston_err_mask;
    jqh["pst_diag"]= qhSnap.piston_diag;

    if (qhSnap.done && !qhSnap.running && qhHoldCnt > 0) {
      qhHoldCnt--;
    }
  }

  

  bool sendLeak = leakSnap.running || (leakSnap.done && leakHoldCnt > 0);
  if (sendLeak) {
    auto jl = doc["leak"].to<JsonObject>();
    jl["r"]   = leakSnap.running;
    jl["ok"]  = leakSnap.pass;
    jl["p0"]  = round(leakSnap.p_base_bar * 10.0) / 10.0;
    jl["p1"]  = round(leakSnap.p_end_bar * 10.0) / 10.0;
    jl["dp"]  = round(leakSnap.dp_bar * 10.0) / 10.0;
    jl["lk"]  = round(leakSnap.leak_barps * 100.0) / 100.0;
    jl["tf"]  = leakSnap.t_fill_ms;
    jl["ts"]  = leakSnap.t_settle_ms;
    jl["th"]  = leakSnap.t_hold_ms;
    jl["msg"] = leakSnap.reason;

    if (leakSnap.done && !leakSnap.running && leakHoldCnt > 0) {
      leakHoldCnt--;
    }
  }

  // Motor/Pompa testi (running veya hold sayacı varsa gönder)
  bool sendMotor = motorSnap.running || (motorSnap.done && motorHoldCnt > 0);
  bool sendVp    = vpSnap.running    || (vpSnap.done    && vpHoldCnt > 0);
  bool sendPMid  = pmidSnap.running  || (pmidSnap.done  && pmidHoldCnt > 0);
  if (sendMotor) {
    auto jm = doc["motor_test"].to<JsonObject>();
    jm["run"] = motorSnap.running;
    jm["ok"]  = motorSnap.pass;
    jm["msg"] = motorSnap.reason;
    jm["fail_mask"] = motorSnap.fail_mask;
    JsonArray arRpm = jm["rpm"].to<JsonArray>();
    JsonArray arDp  = jm["dp"].to<JsonArray>();
    JsonArray arI   = jm["I"].to<JsonArray>();
    for (int i=0;i<5;i++){
      arRpm.add((int)motorSnap.rpm[i]);  // RPM integer yeter
      arDp.add(round(motorSnap.dp_barps[i] * 100.0) / 100.0);  // 0.01 bar/s
      arI.add(round(motorSnap.I_avg[i] * 100.0) / 100.0);  // 0.01A
    }
    if (motorSnap.done && !motorSnap.running && motorHoldCnt > 0) {
      motorHoldCnt--;
    }
  }

  if (sendVp) {
    auto jv = doc["vp"].to<JsonObject>();
    jv["run"] = vpSnap.running;
    jv["ok"]  = vpSnap.pass;
    jv["msg"] = vpSnap.reason;
    // Kompakt mask+array format (JSON boyutunu küçültmek için)
    uint16_t openMask = 0, shortMask = 0;
    JsonArray arrR   = jv["R"].to<JsonArray>();
    JsonArray arrIn  = jv["in"].to<JsonArray>();
    JsonArray arrHold= jv["ho"].to<JsonArray>();
    JsonArray arrDp  = jv["dp"].to<JsonArray>();
    for (int i=0;i<8;i++){
      if (vpSnap.valves[i].open_fault)  openMask  |= (1u<<i);
      if (vpSnap.valves[i].short_fault) shortMask |= (1u<<i);
      arrR.add(round(vpSnap.valves[i].R_est * 10.0) / 10.0);  // 0.1 Ohm
      arrIn.add(round(vpSnap.valves[i].inrush_mA * 10.0) / 10.0);  // 0.1 mA
      arrHold.add(round(vpSnap.valves[i].hold_mA * 10.0) / 10.0);  // 0.1 mA
      arrDp.add((int)vpSnap.valves[i].duty_prof);  // duty integer yeter
    }
    jv["vo"] = openMask;
    jv["vs"] = shortMask;
    // pistonlar
    JsonArray arrPh = jv["ph"].to<JsonArray>();
    uint8_t moveMask = 0, errMask = 0;
    for (int i=0;i<6;i++){
      arrPh.add(round(vpSnap.pistons[i].hall_raw * 10.0) / 10.0);
      if (vpSnap.pistons[i].moved) moveMask |= (1u<<i);
      if (vpSnap.pistons[i].err)   errMask  |= (1u<<i);
    }
    jv["pm"] = moveMask;
    jv["pe"] = errMask;

    if (vpSnap.done && !vpSnap.running && vpHoldCnt > 0) {
      vpHoldCnt--;
    }
  }

  if (sendPMid) {
    auto jm = doc["pmid"].to<JsonObject>();
    jm["run"] = pmidSnap.running;
    jm["ok"]  = pmidSnap.pass;
    jm["msg"] = pmidSnap.reason;
    jm["drop"]= round(pmidSnap.max_drop_bar * 10.0) / 10.0;  // 0.1 bar precision
    jm["fail_mask"] = pmidSnap.fail_mask;
    jm["cp"]  = pmidSnap.current_piston;
    jm["rem"] = pmidSnap.remaining_ms;
    if (pmidSnap.done && !pmidSnap.running && pmidHoldCnt > 0) {
      pmidHoldCnt--;
    }
  }


  bool autoActive = g_autoShiftPub.running; // || g_autoShiftPub.step_idx != 0 || g_autoShiftPub.repeat_idx != 0;
  if (autoActive) {
    auto jauto = doc["auto"].to<JsonObject>();
    jauto["run"] = g_autoShiftPub.running;
    jauto["rep"] = g_autoShiftPub.repeat_idx;
    jauto["step"] = g_autoShiftPub.step_idx;
    //jauto["tms"] = g_autoShiftPub.gear_ms;
    jauto["name"] = g_autoShiftPub.step_name;

  auto dx = jauto["diag"].to<JsonObject>();
  dx["step"] = s.step;
  dx["dP"]   = s.dP;
  dx["Vmin"] = s.Vbus_min;
  dx["IpPk"] = s.I_pump_peak;
  dx["ff"]   = s.faults;
  //JsonArray ca = dx["coil_pk_mA"].to<JsonArray>();
  //for (int i=0;i<8;i++) ca.add(s.I_coil_peak_mA[i]);
  }

  static char out[4096];
  size_t n = serializeJson(doc, out, sizeof(out) - 2);
  if (n >= sizeof(out) - 2) {
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[JSON] TOO_LONG");
  } else if (n > 0) {
    out[n++] = '\0';
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, out);
  }
}
#endif  // Eski tek-büyük-JSON sistemi sonu

// ======= JSON AL / KOMUT İŞLE =======
static void applyPumpCommand(const JsonObject& j) {
  // {"pump": {"mode":"start|auto|stop", "current": 3.5}}
  PumpCommand pc;
  memset(&pc, 0, sizeof(pc));

  const char* mode = j["mode"] | "";
  if      (!strcasecmp(mode, "start")) pc.cmd = PUMP_CMD_START;
  else if (!strcasecmp(mode, "auto"))  pc.cmd = PUMP_CMD_AUTO;
  else if (!strcasecmp(mode, "stop"))  pc.cmd = PUMP_CMD_STOP;

  if (j.containsKey("current")) {
    pc.setCurrentA = (float)j["current"].as<double>();
  }

  portENTER_CRITICAL(&g_portMux);
  pc.seq = g_pumpCmd.seq + 1;
  g_pumpCmd = pc;
  portEXIT_CRITICAL(&g_portMux);
}

static void applyFillDrain(const JsonObject& j) {
  // {"fill":{"current": 2.5, "ms": 10000, "dir":"fill|drain"}}
  PumpCommand pc;
  memset(&pc, 0, sizeof(pc));

  pc.fillCurrentA   = (float)j["current"].as<double>();
  pc.fillDurationMs = (uint32_t)j["ms"].as<uint32_t>();
  const char* dir   = j["dir"] | "fill";
  if (!strcasecmp(dir, "drain")) {
    pc.cmd = PUMP_CMD_DRAIN;
  } else {
    pc.cmd = PUMP_CMD_FILL;
  }

  portENTER_CRITICAL(&g_portMux);
  pc.seq = g_pumpCmd.seq + 1;
  g_pumpCmd = pc;
  portEXIT_CRITICAL(&g_portMux);
}

// JSON'da isim -> index eÅŸlemesi (senin kullandÄ±ÄŸÄ±n sÄ±ra)
static int valveNameToIndex(const char* s){
  if (!s) return -1;
  if (!strcasecmp(s,"N433")) return 0;
  if (!strcasecmp(s,"N436")) return 1;
  if (!strcasecmp(s,"N434")) return 2;
  if (!strcasecmp(s,"N435")) return 3;
  if (!strcasecmp(s,"N438")) return 4;
  if (!strcasecmp(s,"N440")) return 5;
  if (!strcasecmp(s,"N439")) return 6;
  if (!strcasecmp(s,"N437")) return 7;
  return -1;
}

static void applyValveCommand(const JsonObject &j){
  // 1) Ä°ndex veya isim Ã§Ã¶z
  int idx = -1;

  if (j["idx"].is<int>()) {
    idx = j["idx"].as<int>();
  } else {
    const char* nm = nullptr;
    if (j["name"].is<const char*>())      nm = j["name"].as<const char*>();
    else if (j["ch"].is<const char*>())   nm = j["ch"].as<const char*>();

    if (nm) {
      // "ALL" -> hepsini 0 yap ve çık
      if (!strcasecmp(nm, "ALL")) {
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          for (int i=0;i<8;i++) {
            g_valveTargetDuty[i] = 0;
            g_valveCustomCurrent_mA[i] = 0.0f;
          }
          xSemaphoreGive(g_sharedMutex);
        }
        return;
      }
      idx = valveNameToIndex(nm);
    }
  }

  // 2) Duty oku (varsayılan 0)
  uint16_t duty = 0;
  if (j["duty"].is<uint16_t>()) {
    duty = j["duty"].as<uint16_t>();
  } else if (j["duty"].is<int>()) {
    int d = j["duty"].as<int>();
    if (d < 0) d = 0; if (d > 4095) d = 4095;
    duty = (uint16_t)d;
  } else if (j["stop"].is<bool>() && j["stop"].as<bool>()) {
    duty = 0;
  }

  // 2b) mA field: INA kapalı çevrim regülatörüne akım hedefi ilet — V=IR YOK
  //     mA > 0  → g_valveCustomCurrent_mA hedefi, duty sıfırla
  //     mA == 0 → kapalı çevrim hedefi temizle
  if (j["mA"].is<float>() || j["mA"].is<int>()) {
    float mA = j["mA"].as<float>();
    if (mA < 0.0f) mA = 0.0f;
    if (mA > 2000.0f) mA = 2000.0f;
    if (idx >= 0 && idx < 8) {
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_valveCustomCurrent_mA[idx] = mA;
        g_valveTargetDuty[idx]       = 0;  // Eski duty çakışmasını önle
        xSemaphoreGive(g_sharedMutex);
      }
    }
    return;  // duty yazımını atla — kapalı çevrim regülatörü halleder
  }

  // 3) Güvenlik kontrolü: N433 (idx 0) açılmadan önce N436 (idx 1) PCV açık mı?
  // Grup 1 pistonları (N433, N434, N435) için PCV=N436 (idx 1) gerekli
  // Üretici modunda (g_manufacturerMode=true) bu kontrol atlanır
  if (idx == 0 && duty > 0 && !g_manufacturerMode) {
    float pcvMa = 0.0f;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      pcvMa = g_valveCustomCurrent_mA[1];  // N436 akım hedefi
      xSemaphoreGive(g_sharedMutex);
    }
    if (pcvMa < 50.0f) {
      // PCV kapalı, N433 açılamaz
      {
        char warn[80];
        snprintf(warn, sizeof(warn), "[SAFETY] N436 (PCV) önce açılmalı! CMD: valve N436 duty=2000");
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, warn);
      }
      return;  // Komutu iptal et
    }
  }

  // 4) Uygula
  if (idx >= 0 && idx < 8) {
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_valveTargetDuty[idx] = duty;
      xSemaphoreGive(g_sharedMutex);
    }
  }
}

static inline int pistonNameToIndex(const char* s){
  if (!s) return -1;
  int mapIdx = PistonControl_IndexFromName(s);
  if (mapIdx >= 0) return mapIdx;
  if (!strcasecmp(s,"5_7") || !strcasecmp(s,"57")) return PISTON_5_7;
  if (!strcasecmp(s,"1_3") || !strcasecmp(s,"13")) return PISTON_1_3;
  if (!strcasecmp(s,"2_4") || !strcasecmp(s,"24")) return PISTON_2_4;
  if (!strcasecmp(s,"6_R") || !strcasecmp(s,"6r") || !strcasecmp(s,"6-r")) return PISTON_6_R;
  if (!strcasecmp(s,"N435") || !strcasecmp(s,"K1") || !strcasecmp(s,"k1")) return PISTON_K1;
  if (!strcasecmp(s,"N439") || !strcasecmp(s,"K2") || !strcasecmp(s,"k2")) return PISTON_K2;
  return -1;
}

static const char* pistonIndexToName(int idx){
  switch (idx) {
    case PISTON_5_7: return "5_7";
    case PISTON_1_3: return "1_3";
    case PISTON_2_4: return "2_4";
    case PISTON_6_R: return "6_R";
    case PISTON_K1:  return "K1";
    case PISTON_K2:  return "K2";
    default: return "?";
  }
}

static const char* pistonIndexToValveName(int idx){
  if (idx >= 0 && idx < PISTON_CHANNEL_COUNT) {
    return kPistonAxisConfig[idx].name;
  }
  return "?";
}

static int pistonRefStateFromString(const char* s){
  if (!s) return -1;
  if (!strcasecmp(s,"closed") || !strcasecmp(s,"kapali") || !strcasecmp(s,"close")) return PISTON_REF_CLOSED;
  if (!strcasecmp(s,"mid") || !strcasecmp(s,"orta") || !strcasecmp(s,"center")) return PISTON_REF_MID;
  if (!strcasecmp(s,"open") || !strcasecmp(s,"acik")) return PISTON_REF_OPEN;
  return -1;
}

static void applyPistonCalibrationRequest(const JsonObject &j){
  int idx = -1;
  if (j["idx"].is<int>()) {
    idx = j["idx"].as<int>();
  } else {
    const char* name = nullptr;
    if (j["piston"].is<const char*>()) name = j["piston"].as<const char*>();
    else if (j["target"].is<const char*>()) name = j["target"].as<const char*>();
    else if (j["name"].is<const char*>()) name = j["name"].as<const char*>();
    idx = pistonNameToIndex(name);
  }
  if (idx < 0 || idx >= PISTON_CHANNEL_COUNT) {
    {
      char msg[64];
      snprintf(msg, sizeof(msg), "[CAL] invalid piston idx=%d", idx);
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
    }
    return;
  }

  PistonCalibrationRequest req{};
  req.piston = (uint8_t)idx;
  req.pwmDuty = j["duty"].is<uint16_t>() ? j["duty"].as<uint16_t>() : 1350;
  req.samplePeriodMs = j["periodMs"].is<uint16_t>() ? j["periodMs"].as<uint16_t>() :
                       j["period"].is<uint16_t>()   ? j["period"].as<uint16_t>()   : 15;
  req.settleMs = j["settleMs"].is<uint16_t>() ? j["settleMs"].as<uint16_t>() : 400;
  req.maxSamples = j["samples"].is<uint16_t>() ? j["samples"].as<uint16_t>() : PISTON_CAL_TABLE_POINTS;
  
  if (req.maxSamples < 4) req.maxSamples = 4;
  
  if (req.maxSamples > PISTON_CAL_TABLE_POINTS) req.maxSamples = PISTON_CAL_TABLE_POINTS;
 
  if (j["pressure"].is<float>()) {
    req.pressureTargetBar = j["pressure"].as<float>();
  } else if (j["pressure"].is<int>()) {
    req.pressureTargetBar = (float)j["pressure"].as<int>();
  } else if (j["bar"].is<float>()) {
    req.pressureTargetBar = j["bar"].as<float>();
  } else if (j["bar"].is<int>()) {
    req.pressureTargetBar = (float)j["bar"].as<int>();
  } else {
    req.pressureTargetBar = 50.0f;
  }
  
  if (req.pressureTargetBar < 5.0f) req.pressureTargetBar = 5.0f;
  const char* action = j["action"].is<const char*>() ? j["action"].as<const char*>() : "start";
  
  if (j["start"].is<bool>()) {
    req.start = j["start"].as<bool>();
  } else {
    req.start = strcasecmp(action, "stop") != 0;
  }
  
  // Hold PWM bulma (findHold veya hold parametresi)
  req.findHold = j["findHold"].is<bool>() ? j["findHold"].as<bool>() : 
                 j["hold"].is<bool>() ? j["hold"].as<bool>() : false;

  if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    g_pistonCalReq = req;
    xSemaphoreGive(g_sharedMutex);
    bumpSeq(g_pistonCalReqSeq);
  }
}

static const char* pistonRefStateName(uint8_t state){
    switch (state) {
        case PISTON_REF_CLOSED: return "CLOSED";
        case PISTON_REF_MID:    return "MID";
        case PISTON_REF_OPEN:   return "OPEN";
        default: return "?";
    }
}

static void applyPistonReferenceCommand(const JsonObject &j){
  int idx = -1;
  if (j["idx"].is<int>()) {
    idx = j["idx"].as<int>();
  } else if (j["piston"].is<const char*>()) {
    idx = pistonNameToIndex(j["piston"].as<const char*>());
  }
  if (idx < 0 || idx >= PISTON_CHANNEL_COUNT) {
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[REF] invalid piston");
    return;
  }

  int state = -1;
  if (j["state"].is<const char*>()) {
    state = pistonRefStateFromString(j["state"].as<const char*>());
  } else if (j["pos"].is<const char*>()) {
    state = pistonRefStateFromString(j["pos"].as<const char*>());
  }
  if (state < 0 || state > PISTON_REF_OPEN) {
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[REF] invalid state");
    return;
  }

  float raw = 0.0f;
  bool haveRaw = false;
  if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    raw = g_pistonHallRaw[idx];
    haveRaw = true;
    xSemaphoreGive(g_sharedMutex);
  }
  if (!haveRaw) {
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[REF] unable to read sensor");
    return;
  }

  PistonReferenceRequest req{};
  req.piston = (uint8_t)idx;
  req.state  = (uint8_t)state;
  req.rawValue = raw;

  if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    g_pistonRefReq = req;
    xSemaphoreGive(g_sharedMutex);
    bumpSeq(g_pistonRefReqSeq);
  } else {
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[REF] unable to queue request");
    return;
  }

  {
    const char* st = (state==PISTON_REF_CLOSED)?"CLOSED":(state==PISTON_REF_MID?"MID":"OPEN");
    char msg[120];
    snprintf(msg, sizeof(msg), "[REF] queued piston=%s state=%s raw=%.4f", pistonIndexToName(idx), st, raw);
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
  }
}

static void applyPistonHoldCommand(const JsonObject &j){
  int idx = -1;
  if (j["idx"].is<int>())            idx = j["idx"].as<int>();
  else if (j["piston"].is<int>())    idx = j["piston"].as<int>();          // GUI'den gelen integer index (0-3)
  else if (j["piston"].is<const char*>()) idx = pistonNameToIndex(j["piston"].as<const char*>());
  if (idx < 0 || idx >= PISTON_CHANNEL_COUNT) {
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[HOLD] invalid piston");
    return;
  }

  bool enable = true;
  if (j["enable"].is<bool>()) enable = j["enable"].as<bool>();
  else if (j["start"].is<bool>()) enable = j["start"].as<bool>();
  else if (j["active"].is<bool>()) enable = j["active"].as<bool>();
  else if (j["action"].is<const char*>()) {
    const char* action = j["action"].as<const char*>();
    enable = (strcmp(action, "start") == 0);
  }

  int state = PISTON_REF_MID;
  if (j["state"].is<const char*>()) state = pistonRefStateFromString(j["state"].as<const char*>());
  else if (j["pos"].is<const char*>()) state = pistonRefStateFromString(j["pos"].as<const char*>());

  PistonHoldRequest req{};
  req.piston = (uint8_t)idx;
  req.enable = enable;
  req.tolerance = j["tol"].is<float>() ? j["tol"].as<float>() :
                  j["tolerance"].is<float>() ? j["tolerance"].as<float>() : 0.01f;
  if (req.tolerance < 0.001f) req.tolerance = 0.001f;

  if (enable) {
    if (state < 0 || state > PISTON_REF_OPEN) {
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[HOLD] invalid state");
      return;
    }
    req.state = (uint8_t)state;
  }

  if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    g_pistonHoldReq[idx] = req;
    xSemaphoreGive(g_sharedMutex);
    bumpSeq(g_pistonHoldReqSeq[idx]);
  } else {
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[HOLD] unable to queue request");
    return;
  }

  {
    char msg[120];
    if (enable) {
      snprintf(msg, sizeof(msg), "[HOLD] enable piston=%s state=%s tol=%.4f",
               pistonIndexToName(idx), pistonRefStateName(req.state), req.tolerance);
    } else {
      snprintf(msg, sizeof(msg), "[HOLD] disable piston=%s", pistonIndexToName(idx));
    }
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
  }
}

static void publishPistonCalibSnapshot(int idx, bool ok, const char *err){
    if (idx < 0 || idx >= PISTON_CHANNEL_COUNT) return;
  PistonCalibData d{};
  if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    d = g_pistonCalibData[idx];
    xSemaphoreGive(g_sharedMutex);
  }
  StaticJsonDocument<512> doc;
  doc["r"] = true;
  doc["ok"] = ok;
  doc["cmd"] = "piston_calib";
  doc["p"] = pistonIndexToValveName(idx);
  if (!ok && err) doc["err"] = err;
  doc["min"] = d.min_raw;
  doc["max"] = d.max_raw;
  doc["mid"] = d.mid_raw;
  doc["dir"] = d.direction;
  doc["break"] = d.duty_breakaway;
  JsonArray uff = doc["uff"].to<JsonArray>();
  JsonArray pb = doc["pbins"].to<JsonArray>();
  for (size_t i = 0; i < PISTON_FF_BINS; ++i) {
    uff.add(d.u_ff_map[i]);
    pb.add(d.p_bins[i]);
  }
  char buf[512];
  size_t n = serializeJson(doc, buf, sizeof(buf) - 1);
  if (n >= sizeof(buf) - 1) n = sizeof(buf) - 2;
  buf[n] = 0;
  kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
}

static void handlePistonCalibStatus(int idx){
  if (idx >= 0) {
    publishPistonCalibSnapshot(idx, true, nullptr);
    return;
  }
  for (int i = 0; i < PISTON_CHANNEL_COUNT; ++i) {
    publishPistonCalibSnapshot(i, g_pistonCalibData[i].calibrated, nullptr);
  }
}

static void handlePistonCalibClear(int idx){
  if (idx < 0) {
    PistonCalibStorage_ClearAll();
    for (int i = 0; i < PISTON_CHANNEL_COUNT; ++i) {
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_pistonCalibData[i] = {};
        xSemaphoreGive(g_sharedMutex);
      }
      publishPistonCalibSnapshot(i, true, nullptr);
    }
  } else {
    PistonCalibStorage_Clear((uint8_t)idx);
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_pistonCalibData[idx] = {};
      xSemaphoreGive(g_sharedMutex);
    }
    publishPistonCalibSnapshot(idx, true, nullptr);
  }
}

// [DEPRECATED] Eski kalibrasyon sistemi - artık g_pistonCalReq kullanılıyor
// static void handlePistonCalibStart(int idx, bool all){
//   PistonCalibCommand cmd{};
//   cmd.action = all ? PistonCalibCommand::START_ALL : PistonCalibCommand::START_ONE;
//   cmd.piston = (idx >= 0) ? (uint8_t)idx : 0;
//   PistonControl_HandleCalibCommand(cmd);
//   {
//     char msg[80];
//     if (all) {
//       snprintf(msg, sizeof(msg), "[CALIB] start all pistons");
//     } else {
//       snprintf(msg, sizeof(msg), "[CALIB] start %s", pistonIndexToValveName(idx));
//     }
//     kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
//   }
// }
static void applySSR(const JsonVariant& v) {
  // {"ssr": true/false}
  bool on = v.as<bool>();
  g_ssrDesired = on ? 1 : 0; // TaskWorker/Status tarafÄ±nda uygulanÄ±yor olabilir
}

// {"pump":{"cmd":"start"}} / {"pump":{"cmd":"auto"}} / {"pump":{"cmd":"stop"}}
// {"pump":{"cmd":"set_curr","current":3.5}}
// {"fill":{"mode":"fill","current":3.0,"ms":8000}}
// {"valve":{"idx":0,"duty":1024}}  // idx 0..7
// {"ssr":true}  // Ä±sÄ±tÄ±cÄ±
// {"curr":3.2}  // kÄ±sayol: Ã§alÄ±ÅŸma akÄ±mÄ±
static void parseAndDispatch(uint8_t type, const uint8_t* payload, uint16_t len) {
  if (type != kitronic::FT_COMMAND && type != kitronic::FT_REQUEST) {
    return;
  }

  static JsonDocument doc;
  doc.clear();
  auto err = deserializeMsgPack(doc, payload, len);
  if (err){
    {
      JsonDocument pdoc;
      auto p = pdoc["p"].to<JsonArray>();
      p.add((int)err.code());
      sendMsgPackLog(kitronic::MsgCode::RX_ERROR, p);
    }
    return;
  }

  if (doc["cmd"].is<const char*>()) {
    const char* c = doc["cmd"].as<const char*>();
    auto resolvePiston = [&](const JsonVariant &v) -> int {
      if (v.is<const char*>()) return pistonNameToIndex(v.as<const char*>());
      if (v.is<int>()) return v.as<int>();
      return -1;
    };
    if (!strcasecmp(c, "piston_calib")) {
      int idx = resolvePiston(doc["p"]);
      if (idx < 0) idx = resolvePiston(doc["piston"]);
      if (idx < 0) idx = resolvePiston(doc["valve"]);
      if (idx >= 0 && idx < PISTON_CHANNEL_COUNT) {
        // Yeni sistem: g_pistonCalReq kullan
        PistonCalibrationRequest req{};
        req.piston = (uint8_t)idx;
        req.pwmDuty = 1600;
        req.start = true;
        req.findHold = true;
        req.calibrateAll = false;
        req.pressureTargetBar = 50.0f;
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          g_pistonCalReq = req;
          xSemaphoreGive(g_sharedMutex);
          bumpSeq(g_pistonCalReqSeq);
        }
      } else {
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[CALIB] invalid piston");
      }
    } else if (!strcasecmp(c, "piston_calib_all")) {
      // Yeni sistem: g_pistonCalReq kullan (tüm pistonlar)
      PistonCalibrationRequest req{};
      req.piston = 0;
      req.pwmDuty = 1600;
      req.start = true;
      req.findHold = true;
      req.calibrateAll = true;
      req.pressureTargetBar = 50.0f;
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_pistonCalReq = req;
        xSemaphoreGive(g_sharedMutex);
        bumpSeq(g_pistonCalReqSeq);
      }
    } else if (!strcasecmp(c, "piston_calib_clear")) {
      int idx = resolvePiston(doc["p"]);
      if (idx < 0) idx = resolvePiston(doc["piston"]);
      if (idx < 0) idx = resolvePiston(doc["valve"]);
      handlePistonCalibClear(idx);
    } else if (!strcasecmp(c, "piston_calib_status")) {
      int idx = resolvePiston(doc["p"]);
      if (idx < 0) idx = resolvePiston(doc["piston"]);
      if (idx < 0) idx = resolvePiston(doc["valve"]);
      handlePistonCalibStatus(idx);
    } else if (!strcasecmp(c, "kavrama_calib")) {
      // Kavrama (K1/K2) otomatik kalibrasyon baslat
      g_kavramaCalibSeq++;
      {
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[KCAL] Kavrama kalibrasyon baslatildi");
      }
    } else if (!strcasecmp(c, "tmag_status")) {
      // TMAG5173 sensör durumunu raporla
      {
        char msg[80];
        snprintf(msg, sizeof(msg), "[TMAG] Status: gear={1_3:%d, 5_7:%d, 2_4:%d, 6_R:%d}",
                 g_tmagData[0].z, g_tmagData[1].z, g_tmagData[2].z, g_tmagData[3].z);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        snprintf(msg, sizeof(msg), "[TMAG] K1: s1={%d,%d,%d} s2={%d,%d,%d}",
                 g_tmagData[4].x, g_tmagData[4].y, g_tmagData[4].z,
                 g_tmagData[5].x, g_tmagData[5].y, g_tmagData[5].z);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        snprintf(msg, sizeof(msg), "[TMAG] K2: s1={%d,%d,%d} s2={%d,%d,%d}",
                 g_tmagData[6].x, g_tmagData[6].y, g_tmagData[6].z,
                 g_tmagData[7].x, g_tmagData[7].y, g_tmagData[7].z);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        snprintf(msg, sizeof(msg), "[TMAG] valid: %d%d%d%d %d%d%d%d",
                 g_tmagData[0].valid, g_tmagData[1].valid, g_tmagData[2].valid, g_tmagData[3].valid,
                 g_tmagData[4].valid, g_tmagData[5].valid, g_tmagData[6].valid, g_tmagData[7].valid);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
      }
    } else if (!strcasecmp(c, "tmag_calib")) {
      // TMAG5173 kalibrasyon: {"cmd":"tmag_calib","piston":"0","state":"closed"}
      // piston: 0-3 (vites), "k1", "k2" (kavrama)
      // state: "closed" veya "open"
      const char* pistonStr = doc["piston"] | "";
      const char* state = doc["state"] | "closed";
      bool isOpen = !strcasecmp(state, "open");
      
      int pistonIdx = -1;
      bool isKavrama = false;
      
      if (!strcasecmp(pistonStr, "k1")) {
        pistonIdx = 0;
        isKavrama = true;
      } else if (!strcasecmp(pistonStr, "k2")) {
        pistonIdx = 1;
        isKavrama = true;
      } else {
        pistonIdx = atoi(pistonStr);
        if (pistonIdx < 0 || pistonIdx >= PISTON_CHANNEL_COUNT) pistonIdx = -1;
      }
      
      if (pistonIdx >= 0) {
        if (isKavrama) {
          // Kavrama kalibrasyonu
          uint8_t ch1 = (pistonIdx == 0) ? TMAG_CH_K1_1 : TMAG_CH_K2_1;
          uint8_t ch2 = (pistonIdx == 0) ? TMAG_CH_K1_2 : TMAG_CH_K2_2;
          
          if (isOpen) {
            g_tmagKavramaCalib[pistonIdx].sensor1_open = g_tmagData[ch1].z;
            g_tmagKavramaCalib[pistonIdx].sensor2_open = g_tmagData[ch2].z;
            g_tmagKavramaCalib[pistonIdx].strokeMm = 30.0f;  // 30mm strok
            g_tmagKavramaCalib[pistonIdx].valid = true;
          } else {
            g_tmagKavramaCalib[pistonIdx].sensor1_closed = g_tmagData[ch1].z;
            g_tmagKavramaCalib[pistonIdx].sensor2_closed = g_tmagData[ch2].z;
          }
          g_tmagCalibSeq++;
          
          // Flash'a kaydet
          TMAGCalib_SaveKavrama(pistonIdx);
          
          char msg[80];
          snprintf(msg, sizeof(msg), "[TMAG_CAL] K%d %s: s1=%d s2=%d SAVED", 
                   pistonIdx+1, isOpen ? "OPEN" : "CLOSED",
                   g_tmagData[ch1].z, g_tmagData[ch2].z);
          kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        } else {
          // Vites pistonu kalibrasyonu
          uint8_t tmagCh = (pistonIdx == PISTON_1_3) ? TMAG_CH_1_3 :
                           (pistonIdx == PISTON_5_7) ? TMAG_CH_5_7 :
                           (pistonIdx == PISTON_2_4) ? TMAG_CH_2_4 : TMAG_CH_6_R;
          
          TMAGPistonCalib& cal = g_tmagPistonCalib[pistonIdx];
          int16_t currentZ = g_tmagData[tmagCh].z;
          
          if (isOpen) {
            cal.zMax = currentZ;
            cal.strokeMm = PISTON_DEFAULT_STROKE_MM;
            // Her iki değer de set edilmişse valid yap
            if (cal.zMin != cal.zMax) cal.valid = true;
          } else {
            cal.zMin = currentZ;
            // Her iki değer de set edilmişse valid yap
            if (cal.zMin != cal.zMax) cal.valid = true;
          }
          g_tmagCalibSeq++;
          
          // Flash'a kaydet
          TMAGCalib_SavePiston(pistonIdx);
          
          char msg[120];
          snprintf(msg, sizeof(msg), "[TMAG_CAL] P%d %s: z=%d (zMin=%d, zMax=%d, valid=%d) SAVED", 
                   pistonIdx, isOpen ? "OPEN" : "CLOSED", currentZ,
                   cal.zMin, cal.zMax, cal.valid);
          kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        }
      }
    } else if (!strcasecmp(c, "i2c_scan")) {
      // I2C bus tarama - hangi adresler cevap veriyor
      if (g_i2cMutex && xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[I2C] Scanning...");
        char msg[64];
        int found = 0;
        for (uint8_t addr = 1; addr < 127; addr++) {
          Wire.beginTransmission(addr);
          if (Wire.endTransmission() == 0) {
            snprintf(msg, sizeof(msg), "[I2C] Found: 0x%02X", addr);
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
            found++;
          }
        }
        snprintf(msg, sizeof(msg), "[I2C] Scan done, %d devices found", found);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        xSemaphoreGive(g_i2cMutex);
      }
    } else if (!strcasecmp(c, "mux_scan")) {
      // TCA9548A mux kanallarını tek tek tara
      if (g_i2cMutex && xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        char msg[80];
        const uint8_t MUX_ADDR = 0x70;
        
        // Önce mux'u kapat (tüm kanallar kapalı)
        Wire.beginTransmission(MUX_ADDR);
        Wire.write(0x00);
        Wire.endTransmission();
        
        for (uint8_t ch = 0; ch < 8; ch++) {
          // Kanalı aç
          Wire.beginTransmission(MUX_ADDR);
          Wire.write(1 << ch);
          if (Wire.endTransmission() != 0) {
            snprintf(msg, sizeof(msg), "[MUX] ch%d: MUX select FAIL", ch);
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
            continue;
          }
          
          vTaskDelay(pdMS_TO_TICKS(5));
          
          // 0x35 (TMAG5173) var mı kontrol et
          Wire.beginTransmission(0x35);
          uint8_t err = Wire.endTransmission();
          
          if (err == 0) {
            snprintf(msg, sizeof(msg), "[MUX] ch%d: TMAG5173 (0x35) FOUND", ch);
          } else {
            snprintf(msg, sizeof(msg), "[MUX] ch%d: no device (err=%d)", ch, err);
          }
          kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        }
        
        // Mux'u kapat
        Wire.beginTransmission(MUX_ADDR);
        Wire.write(0x00);
        Wire.endTransmission();
        
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[MUX] Scan done");
        xSemaphoreGive(g_i2cMutex);
      }
    } else if (!strcasecmp(c, "tmag_read_id")) {
      // Her mux kanalından TMAG5173 device ID oku
      if (g_i2cMutex && xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        char msg[80];
        const uint8_t MUX_ADDR = 0x70;
        const uint8_t TMAG_ADDR = 0x35;
        const uint8_t DEVICE_ID_REG = 0x0D;
        
        for (uint8_t ch = 0; ch < 8; ch++) {
          // Kanalı aç
          Wire.beginTransmission(MUX_ADDR);
          Wire.write(1 << ch);
          Wire.endTransmission();
          vTaskDelay(pdMS_TO_TICKS(5));
          
          // Device ID register oku
          Wire.beginTransmission(TMAG_ADDR);
          Wire.write(DEVICE_ID_REG);
          if (Wire.endTransmission(false) != 0) {
            snprintf(msg, sizeof(msg), "[TMAG] ch%d: read fail", ch);
            kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
            continue;
          }
          
          Wire.requestFrom(TMAG_ADDR, (uint8_t)1);
          if (Wire.available()) {
            uint8_t id = Wire.read();
            snprintf(msg, sizeof(msg), "[TMAG] ch%d: DEVICE_ID=0x%02X %s", 
                     ch, id, (id == 0x49) ? "(OK)" : "(unexpected)");
          } else {
            snprintf(msg, sizeof(msg), "[TMAG] ch%d: no response", ch);
          }
          kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        }
        
        // Mux kapat
        Wire.beginTransmission(MUX_ADDR);
        Wire.write(0x00);
        Wire.endTransmission();
        xSemaphoreGive(g_i2cMutex);
      }
    } else if (!strcasecmp(c, "tmag_dump")) {
      // İlk mux kanalından register dump al
      if (g_i2cMutex && xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
        char msg[128];
        const uint8_t MUX_ADDR = 0x70;
        const uint8_t TMAG_ADDR = 0x35;
        
        // Kanal 0 aç
        Wire.beginTransmission(MUX_ADDR);
        Wire.write(0x01);
        Wire.endTransmission();
        vTaskDelay(pdMS_TO_TICKS(10));
        
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[TMAG] Register dump ch0:");
        
        // İlk 32 register oku
        for (uint8_t reg = 0; reg < 32; reg += 8) {
          char line[80];
          int pos = snprintf(line, sizeof(line), "  0x%02X:", reg);
          
          for (uint8_t i = 0; i < 8; i++) {
            Wire.beginTransmission(TMAG_ADDR);
            Wire.write(reg + i);
            if (Wire.endTransmission(false) == 0) {
              Wire.requestFrom(TMAG_ADDR, (uint8_t)1);
              if (Wire.available()) {
                pos += snprintf(line + pos, sizeof(line) - pos, " %02X", Wire.read());
              } else {
                pos += snprintf(line + pos, sizeof(line) - pos, " --");
              }
            } else {
              pos += snprintf(line + pos, sizeof(line) - pos, " XX");
            }
          }
          kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, line);
        }
        
        // Mux kapat
        Wire.beginTransmission(MUX_ADDR);
        Wire.write(0x00);
        Wire.endTransmission();
        xSemaphoreGive(g_i2cMutex);
      }
    }
  }

  // PUMP
  if (doc["pump"].is<JsonObject>()) {
    auto j = doc["pump"].as<JsonObject>();
    const char* c = j["cmd"] | "";
    PumpCmd cmd = PUMP_CMD_NONE;

    if      (!strcasecmp(c,"start"))    cmd = PUMP_CMD_START;
    else if (!strcasecmp(c,"auto"))     cmd = PUMP_CMD_AUTO;
    else if (!strcasecmp(c,"stop"))     cmd = PUMP_CMD_STOP;
    else if (!strcasecmp(c,"set_curr")) cmd = PUMP_CMD_SET_RPM;  // eski isim -> rpm set
    else if (!strcasecmp(c,"set_rpm"))  cmd = PUMP_CMD_SET_RPM;

    if (cmd != PUMP_CMD_NONE) {
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_pumpCmd.cmd = cmd;
        g_pumpCmd.seq++;
        if (j["current"].is<float>()) g_pumpCmd.setCurrentA = j["current"].as<float>();
        if (j["rpm"].is<float>())     g_pumpCmd.setRpm      = j["rpm"].as<float>();
        xSemaphoreGive(g_sharedMutex);
      }
    }
    // rpm alanı varsa ve cmd verilmediyse de rpm set et
    else if (j["rpm"].is<float>()) {
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_pumpCmd.cmd = PUMP_CMD_SET_RPM;
        g_pumpCmd.seq++;
        g_pumpCmd.setRpm = j["rpm"].as<float>();
        xSemaphoreGive(g_sharedMutex);
      }
    }
  }

  // FILL / DRAIN
  if (doc["fill"].is<JsonObject>()) {
    auto f = doc["fill"].as<JsonObject>();
    const char* mode = f["mode"] | "fill";
    float  cur = f["current"] | 3000.0f;
    uint32_t ms = (f["ms"] | 60)*1000;

    PumpCmd cmd = (!strcasecmp(mode,"drain")) ? PUMP_CMD_DRAIN : PUMP_CMD_FILL;

    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      g_pumpCmd.cmd = cmd;
      g_pumpCmd.seq++;
      g_pumpCmd.fillCurrentA  = cur;
      g_pumpCmd.fillDurationMs= ms;
      xSemaphoreGive(g_sharedMutex);
    }
  }

  // Valve fast discharge (basï¿½ï¿½nï¿½ï¿½ dï¿½ï¿½ï¿½rme)
  if (doc["discharge"].is<JsonObject>()) {
    auto d = doc["discharge"].as<JsonObject>();
    ValveDischargeCommand cmd{};
    if (d["pcv"].is<int>())  cmd.pcvDuty  = (uint16_t)d["pcv"].as<int>();
    else if (d["pcvDuty"].is<int>()) cmd.pcvDuty = (uint16_t)d["pcvDuty"].as<int>();
    if (d["pair"].is<int>()) cmd.pairDuty = (uint16_t)d["pair"].as<int>();
    else if (d["pairDuty"].is<int>()) cmd.pairDuty = (uint16_t)d["pairDuty"].as<int>();
    if (d["target"].is<float>()) cmd.targetBar = d["target"].as<float>();
    else if (d["bar"].is<float>()) cmd.targetBar = d["bar"].as<float>();
    if (d["timeout"].is<uint32_t>()) cmd.timeoutMs = d["timeout"].as<uint32_t>();
    else if (d["ms"].is<uint32_t>()) cmd.timeoutMs = d["ms"].as<uint32_t>();
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_valveDischargeCmd = cmd;
      xSemaphoreGive(g_sharedMutex);
      bumpSeq(g_valveDischargeSeq);
    }
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[DRAIN] request queued");
  } else if (doc["discharge"].is<bool>() && doc["discharge"].as<bool>()) {
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      memset(&g_valveDischargeCmd, 0, sizeof(g_valveDischargeCmd));
      xSemaphoreGive(g_sharedMutex);
      bumpSeq(g_valveDischargeSeq);
    }
  }
  //{"valve": {"ch": "N433","duty": 2000}}
  // VALVE duty
  if (doc["valve"].is<JsonObject>()) {
    applyValveCommand(doc["valve"].as<JsonObject>());
  }

  // MANUEL KONTROL - GUI current_ctrl komutu
  // {"current_ctrl": {"valve": 0, "mode": "open"/"close"/"open_slow"/"close_slow"/"off"/"pcv"}}
  // Akım hedefleri (mA) — INA kapalı çevrim regülatörüne iletilir, V=IR YOK
  // Kalibre edilmiş openMa/closeMa kullanılır; yoksa varsayılan sabitler.
  if (doc["current_ctrl"].is<JsonObject>()) {
    auto j = doc["current_ctrl"].as<JsonObject>();
    int idx = j["valve"] | -1;
    const char* modeStr = j["mode"] | "off";

    if (idx >= 0 && idx < 8) {
      // Valf indeksi → piston indeksi (kalibrasyon lookup için)
      // VALVE_IDX_CL = {2,0,7,4,3,6} ters çevirisi; PCV (1,5) → -1
      static const int8_t VALVE_TO_PISTON[8] = { 1, -1, 0, 4, 3, -1, 5, 2 };
      const int8_t pistonIdx = VALVE_TO_PISTON[idx];

      // Kalibre değerleri oku (varsa)
      float calOpen = 0.0f, calClose = 0.0f;
      bool  calOk   = false;
      if (pistonIdx >= 0 && g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        if (g_pistonCalibData[pistonIdx].calibrated) {
          calOpen  = g_pistonCalibData[pistonIdx].open_mA;
          calClose = g_pistonCalibData[pistonIdx].close_mA;
          calOk    = (calOpen > 50.0f && calClose > 50.0f && calOpen > calClose);
        }
        xSemaphoreGive(g_sharedMutex);
      }

      // Güncel hedef akımlarını sistemden oku (GUI'den veya varsayılan)
      const ValveCurrentTargets& tgt = ValveCurrentControl_GetSystem().targets;

      uint8_t modeCode = 0;  // off
      float targetMa = 0.0f;
      if      (!strcasecmp(modeStr, "open"))       { modeCode = 1; targetMa = calOk ? calOpen : tgt.openCurrent_mA; }
      else if (!strcasecmp(modeStr, "close"))      { modeCode = 2; targetMa = calOk ? calClose : tgt.closeCurrent_mA; }
      else if (!strcasecmp(modeStr, "open_slow"))  { modeCode = 3; targetMa = tgt.slowopenCurrent_mA; }
      else if (!strcasecmp(modeStr, "close_slow")) { modeCode = 4; targetMa = tgt.slowcloseCurrent_mA; }
      else if (!strcasecmp(modeStr, "pcv"))        { modeCode = 5; targetMa = tgt.pcvCurrent_mA; }
      // "off" -> targetMa = 0, modeCode = 0

      // GUI'den gönderilen özel akım hedefi varsa kullan (deneme/override)
      float customMa = j["mA"] | 0.0f;
      if (customMa > 0.0f) {
        targetMa = customMa;
      }

      // Güvenlik kontrolü: N433 (idx 0) açılmadan önce N436 (PCV) açık mı?
      if (idx == 0 && targetMa > 0.0f && !g_manufacturerMode) {
        float pcvMa = 0.0f;
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          pcvMa = g_valveCustomCurrent_mA[1];  // N436 akım hedefi
          xSemaphoreGive(g_sharedMutex);
        }
        if (pcvMa < 50.0f) {
          {
            JsonDocument pdoc;
            auto p = pdoc["p"].to<JsonArray>();
            sendMsgPackLog(kitronic::MsgCode::UNKNOWN_COMMAND, p);
          }
          return;
        }
      }

      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_valveCustomCurrent_mA[idx] = targetMa;
        g_valveTargetDuty[idx]       = 0;  // Eski duty çakışmasını önle
        g_valveCustomMode[idx]       = modeCode;
        xSemaphoreGive(g_sharedMutex);
      }
    }

    // Akim hedeflerini guncelle: {"current_ctrl":{"targets":{"open":775,...}}}
    if (j["targets"].is<JsonObject>()) {
      auto t_obj = j["targets"].as<JsonObject>();
      ValveCurrentTargets t = ValveCurrentControl_GetSystem().targets;
      if (!t_obj["open"].isNull())       t.openCurrent_mA      = t_obj["open"].as<float>();
      if (!t_obj["hold"].isNull())       t.holdCurrent_mA      = t_obj["hold"].as<float>();
      if (!t_obj["close"].isNull())      t.closeCurrent_mA     = t_obj["close"].as<float>();
      if (!t_obj["slow_open"].isNull())  t.slowopenCurrent_mA  = t_obj["slow_open"].as<float>();
      if (!t_obj["slow_close"].isNull()) t.slowcloseCurrent_mA = t_obj["slow_close"].as<float>();
      if (!t_obj["pcv"].isNull())        t.pcvCurrent_mA       = t_obj["pcv"].as<float>();
      ValveCurrentControl_SetTargets(t);
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[CURR_TGT] Targets updated OK");
    }

    // Akim hedeflerini oku: {"current_ctrl":{"get_targets":true}}
    if (!j["get_targets"].isNull() && j["get_targets"].as<bool>()) {
      const ValveCurrentTargets& t = ValveCurrentControl_GetSystem().targets;
      static char tbuf[256];
      snprintf(tbuf, sizeof(tbuf),
        "{\"curr_tgt\":{\"open\":%.0f,\"hold\":%.0f,\"close\":%.0f,"
        "\"slow_open\":%.0f,\"slow_close\":%.0f,\"pcv\":%.0f}}",
        t.openCurrent_mA, t.holdCurrent_mA, t.closeCurrent_mA,
        t.slowopenCurrent_mA, t.slowcloseCurrent_mA, t.pcvCurrent_mA);
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, tbuf);
    }
  }

  if (doc["pistonCal"].is<JsonObject>()) {
    applyPistonCalibrationRequest(doc["pistonCal"].as<JsonObject>());
  } else if (doc["piston_cal"].is<JsonObject>()) {
    applyPistonCalibrationRequest(doc["piston_cal"].as<JsonObject>());
  }

  if (doc["pistonRef"].is<JsonObject>()) {
    applyPistonReferenceCommand(doc["pistonRef"].as<JsonObject>());
  }

  if (doc["pistonHold"].is<JsonObject>()) {
    applyPistonHoldCommand(doc["pistonHold"].as<JsonObject>());
  }
  if (doc["piston_hold"].is<JsonObject>()) {
    applyPistonHoldCommand(doc["piston_hold"].as<JsonObject>());
  }
  // MANUEL ORTA NOKTA KAYDETME: {"saveMidPos":{"piston":"N433"}}
  if (doc["saveMidPos"].is<JsonObject>()) {
    auto j = doc["saveMidPos"].as<JsonObject>();
    int idx = -1;
    if (j["idx"].is<int>()) idx = j["idx"].as<int>();
    else if (j["piston"].is<const char*>()) idx = pistonNameToIndex(j["piston"].as<const char*>());
    
    if (idx >= 0 && idx < 4) {  // Sadece vites pistonları (K1/K2 hariç)
      // TMAG kanal eşleşmesi: 0->1, 1->0, 2->2, 3->3 (PISTON_5_7, PISTON_1_3, PISTON_2_4, PISTON_6_R)
      static const uint8_t kPistonToTmag[4] = {1, 0, 2, 3};
      uint8_t tmagCh = kPistonToTmag[idx];
      int16_t currentRaw = 0;
      
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        currentRaw = g_tmagData[tmagCh].z;
        
        // mid_raw olarak kaydet (hold kontrol için)
        g_pistonCalibData[idx].mid_raw = (uint16_t)currentRaw;
        g_pistonManualRef[idx].raw[PISTON_REF_MID] = (float)currentRaw;
        
        // TMAG kalibrasyon struct'ını da güncelle (mesafe hesaplama için)
        g_tmagPistonCalib[idx].zMid = currentRaw;
        
        xSemaphoreGive(g_sharedMutex);
      }
      
      // EEPROM'a kaydet
      TMAGCalib_SavePiston(idx);
      
      {
        char msg[120];
        snprintf(msg, sizeof(msg), "[MID] saved piston=%s mid_raw=%d tmag=%d (saved to EEPROM)", 
                 pistonIndexToName(idx), currentRaw, tmagCh);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
      }
    } else {
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[MID] invalid piston (only gear pistons)");
    }
  }

  // INA219 TARAMA: {"ina_scan":true} - tum 8 INA degerini aninda goster (fiziksel mapping tespiti icin)
  if (doc["ina_scan"].is<bool>() || doc["ina_scan"].is<int>()) {
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      char msg[200];
      snprintf(msg, sizeof(msg),
        "[INA_SCAN] 0=%.0f 1=%.0f 2=%.0f 3=%.0f 4=%.0f 5=%.0f 6=%.0f 7=%.0f mA",
        g_tele.inaI_mA[0], g_tele.inaI_mA[1], g_tele.inaI_mA[2], g_tele.inaI_mA[3],
        g_tele.inaI_mA[4], g_tele.inaI_mA[5], g_tele.inaI_mA[6], g_tele.inaI_mA[7]);
      xSemaphoreGive(g_sharedMutex);
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
    }
  }

  // TOPLU PİSTON KALİBRASYON: {"pistonCalAll":{"action":"start","pressure":50,"findHold":true}}
  // Yeni sistem: g_pistonCalReq üzerinden ilk pistonu başlat, ardışık pistonlar TaskValveControl'da yönetilecek
  if (doc["pistonCalAll"].is<JsonObject>()) {
    auto j = doc["pistonCalAll"].as<JsonObject>();
    const char* action = j["action"] | "start";
    
    if (strcasecmp(action, "start") == 0) {
      // İlk piston (0) için kalibrasyon başlat - yeni sistem kullan
      PistonCalibrationRequest req{};
      req.piston = 0;  // İlk piston
      req.pwmDuty = j["pwmOpen"].is<uint16_t>() ? j["pwmOpen"].as<uint16_t>() : 1600;
      req.settleMs = j["settleMs"].is<uint16_t>() ? j["settleMs"].as<uint16_t>() : 2000;
      req.start = true;
      req.findHold = j["findHold"].is<bool>() ? j["findHold"].as<bool>() : false;
      req.calibrateAll = true;  // Tüm pistonları sırayla kalibre et
      
      // Basınç hedefi
      if (j["pressure"].is<float>()) {
        req.pressureTargetBar = j["pressure"].as<float>();
      } else if (j["pressure"].is<int>()) {
        req.pressureTargetBar = (float)j["pressure"].as<int>();
      } else {
        req.pressureTargetBar = 50.0f;
      }
      
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_pistonCalReq = req;
        xSemaphoreGive(g_sharedMutex);
        bumpSeq(g_pistonCalReqSeq);
      }
      
      {
        char msg[96];
        snprintf(msg, sizeof(msg), "[CALIB_ALL] Starting piston 0, pressure=%.0f, findHold=%d", 
                 req.pressureTargetBar, req.findHold);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
      }
    } else if (strcasecmp(action, "stop") == 0) {
      // Kalibrasyonu durdur - yeni sistemde start=false gönder
      PistonCalibrationRequest req{};
      req.start = false;
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_pistonCalReq = req;
        xSemaphoreGive(g_sharedMutex);
        bumpSeq(g_pistonCalReqSeq);
      }
      {
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[CALIB_ALL] Stopped");
      }
    }
  }

  // KALİBRASYON (CalibrationDialog): {"calibration":{"start":true}} / {"calibration":{"abort":true}}
  if (doc["calibration"].is<JsonObject>()) {
    auto j = doc["calibration"].as<JsonObject>();
    if (j["start"].is<bool>() && j["start"].as<bool>()) {
      PistonCalibrationRequest req{};
      req.piston = 0;
      req.pwmDuty = 1600;
      req.settleMs = 2000;
      req.start = true;
      req.findHold = j["findHold"].is<bool>() ? j["findHold"].as<bool>() : true;
      req.calibrateAll = true;
      req.pressureTargetBar = j["pressure"].is<float>() ? j["pressure"].as<float>() :
                              j["pressure"].is<int>()   ? (float)j["pressure"].as<int>() : 50.0f;
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_pistonCalReq = req;
        xSemaphoreGive(g_sharedMutex);
        bumpSeq(g_pistonCalReqSeq);
      }
      {
        char msg[80];
        snprintf(msg, sizeof(msg), "[CALIB] start all pistons, pressure=%.0f", req.pressureTargetBar);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
      }
    } else if (j["abort"].is<bool>() && j["abort"].as<bool>()) {
      PistonCalibrationRequest req{};
      req.start = false;
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_pistonCalReq = req;
        xSemaphoreGive(g_sharedMutex);
        bumpSeq(g_pistonCalReqSeq);
      }
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[CALIB] aborted");
    }
  }

  // HOLD PWM BULMA: {"findHold":{"piston":"N433","method":"dither"}}
  if (doc["findHold"].is<JsonObject>()) {
    auto j = doc["findHold"].as<JsonObject>();
    int idx = -1;
    if (j["piston"].is<const char*>()) idx = pistonNameToIndex(j["piston"].as<const char*>());
    else if (j["piston"].is<int>()) idx = j["piston"].as<int>();
    if (idx >= 0 && idx < PISTON_CHANNEL_COUNT) {
      // Hold PWM bulma isteği - PistonControl'e yönlendir
      PistonCalibCommand cmd{};
      cmd.action = PistonCalibCommand::FIND_HOLD;
      cmd.piston = (uint8_t)idx;
      PistonControl_HandleCalibCommand(cmd);
      {
        char msg[48];
        snprintf(msg, sizeof(msg), "[FIND_HOLD] Piston %d started", idx);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
      }
    }
  }

  // SSR (hem bool hem int kabul et)
  if (doc.containsKey("ssr")) {
    if (doc["ssr"].is<bool>()) {
      g_ssrDesired = doc["ssr"].as<bool>() ? 1 : 0;
    } else if (doc["ssr"].is<int>()) {
      g_ssrDesired = doc["ssr"].as<int>() ? 1 : 0;
    }
    {
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, g_ssrDesired ? (char*)"[SSR] ON" : (char*)"[SSR] OFF");
    }
  }

  // VALF TEMİZLEME: {"vclean":{"ch":0,"on":true,"period":100}} veya {"vclean":{"ch":1,"on":true,"period":100}}
  // ch: 0 veya 1 (kanal index), on: true/false, period: 20-1000ms
  if (doc["vclean"].is<JsonObject>()) {
    auto vc = doc["vclean"].as<JsonObject>();
    int ch = vc["ch"] | 0;
    bool on = vc["on"] | false;
    uint16_t period = vc["period"] | 100;
    
    // Sınırla
    if (ch < 0) ch = 0;
    if (ch > 1) ch = 1;
    if (period < 20) period = 20;
    if (period > 1000) period = 1000;
    
    g_valveClean.ch[ch].active = on;
    g_valveClean.ch[ch].period_ms = period;
    
    char msg[64];
    if (on) {
      snprintf(msg, sizeof(msg), "[VCLEAN] CH%d ON period=%dms", ch+1, period);
    } else {
      snprintf(msg, sizeof(msg), "[VCLEAN] CH%d OFF", ch+1);
    }
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
  }

  // PIN TESTİ: {"pin_test":{"pin":16,"state":1}} veya {"pin_test":{"pin":17,"state":0}}
  // Direkt GPIO kontrolü - donanım testi için
  if (doc["pin_test"].is<JsonObject>()) {
    auto pt = doc["pin_test"].as<JsonObject>();
    int pin = pt["pin"] | -1;
    int state = pt["state"] | 0;
    if (pin == 16 || pin == 17) {
      pinMode(pin, OUTPUT);
      digitalWrite(pin, state ? HIGH : LOW);
      char msg[64];
      snprintf(msg, sizeof(msg), "[PIN_TEST] GPIO%d = %d", pin, state);
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
    }
  }

  // DONANIM TESTİ: {"hw_test": true}
  // Tüm I2C cihazlarını tarar ve JSON sonuç döner
  if (doc["hw_test"].is<bool>() && doc["hw_test"].as<bool>()) {
    if (!g_i2cMutex) return;

    if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
      
      JsonDocument result;
      result["_t"] = "HW_TEST";
      
      // I2C cihaz listesi
      JsonObject i2c = result["i2c"].to<JsonObject>();
      
      // INA219 (Valf akım sensörleri) 0x40-0x47
      JsonArray ina219 = i2c["ina219"].to<JsonArray>();
      const uint8_t ina219_addrs[] = {0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47};
      const char* ina219_names[] = {"N433", "N434", "N435", "N436", "N437", "N438", "N439", "N440"};
      for (int i = 0; i < 8; i++) {
        Wire.beginTransmission(ina219_addrs[i]);
        bool ok = (Wire.endTransmission() == 0);
        JsonObject dev = ina219.add<JsonObject>();
        dev["addr"] = ina219_addrs[i];
        dev["name"] = ina219_names[i];
        dev["ok"] = ok;
      }
      
      // INA226 (Güç izleme) 0x4C, 0x4D
      JsonArray ina226 = i2c["ina226"].to<JsonArray>();
      const uint8_t ina226_addrs[] = {0x4C, 0x4D};
      const char* ina226_names[] = {"MAIN_PWR", "VESC_PWR"};
      for (int i = 0; i < 2; i++) {
        Wire.beginTransmission(ina226_addrs[i]);
        bool ok = (Wire.endTransmission() == 0);
        JsonObject dev = ina226.add<JsonObject>();
        dev["addr"] = ina226_addrs[i];
        dev["name"] = ina226_names[i];
        dev["ok"] = ok;
      }
      
      // TCA9555 (GPIO Expander) 0x20, 0x21
      JsonArray tca9555 = i2c["tca9555"].to<JsonArray>();
      const uint8_t tca_addrs[] = {0x20, 0x21};
      const char* tca_names[] = {"DRV_1", "DRV_2"};
      for (int i = 0; i < 2; i++) {
        Wire.beginTransmission(tca_addrs[i]);
        bool ok = (Wire.endTransmission() == 0);
        JsonObject dev = tca9555.add<JsonObject>();
        dev["addr"] = tca_addrs[i];
        dev["name"] = tca_names[i];
        dev["ok"] = ok;
      }
      
      // TCA9548A (I2C Mux) 0x70
      Wire.beginTransmission(0x70);
      bool mux_ok = (Wire.endTransmission() == 0);
      i2c["tca9548a"] = mux_ok;
      
      // TMAG5173 (Mux üzerinden) - her kanal için kontrol
      if (mux_ok) {
        JsonArray tmag = i2c["tmag5173"].to<JsonArray>();
        const char* tmag_names[] = {"1_3", "5_7", "2_4", "6_R", "K1_1", "K1_2", "K2_1", "K2_2"};
        for (uint8_t ch = 0; ch < 8; ch++) {
          // Kanalı seç
          Wire.beginTransmission(0x70);
          Wire.write(1 << ch);
          Wire.endTransmission();
          vTaskDelay(pdMS_TO_TICKS(2));
          
          // TMAG5173 kontrol (0x35)
          Wire.beginTransmission(0x35);
          bool ok = (Wire.endTransmission() == 0);
          
          JsonObject dev = tmag.add<JsonObject>();
          dev["ch"] = ch;
          dev["name"] = tmag_names[ch];
          dev["ok"] = ok;
        }
        // Mux'u kapat
        Wire.beginTransmission(0x70);
        Wire.write(0x00);
        Wire.endTransmission();
      }
      
      xSemaphoreGive(g_i2cMutex);
      
      // DRV8243 Motor Sürücüleri (SPI)
      DRV8243Status drvStatus[4];
      DRV_GetAllStatus(drvStatus);
      
      JsonArray drv8243 = result["drv8243"].to<JsonArray>();
      const char* drv_names[] = {"DRV1 (N433/N436)", "DRV2 (N434/N435)", "DRV3 (N439/N437)", "DRV4 (N438/N440)"};
      for (int i = 0; i < 4; i++) {
        JsonObject dev = drv8243.add<JsonObject>();
        dev["idx"] = i + 1;
        dev["name"] = drv_names[i];
        dev["st1"] = drvStatus[i].st1;
        dev["st2"] = drvStatus[i].st2;
        dev["flt"] = drvStatus[i].flt;
        dev["ok"] = drvStatus[i].ok;
      }
      
      // JSON'u seri porta gönder
      static char buf[4096];  // Queue item boyutuyla eşleşmeli
      size_t len = serializeJson(result, buf, sizeof(buf));
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
    }
  }

  // VALF BAĞLANTI TESTİ: {"valve_check": true}
  // Tüm valfleri sırayla kısa süre açarak DRV8243 fault durumunu kontrol eder
  if (doc["valve_check"].is<bool>() && doc["valve_check"].as<bool>()) {
    JsonDocument result;
    result["_t"] = "VALVE_CHECK";
    
    // Valf isimleri ve PWM index mapping
    // PWM sırası: 0=N433, 1=N436, 2=N434, 3=N435, 4=N438, 5=N440, 6=N439, 7=N437
    // INA sırası: 0=N433, 1=N434, 2=N435, 3=N436, 4=N437, 5=N438, 6=N439, 7=N440
    static const char* valveNames[8] = {"N433", "N436", "N434", "N435", "N438", "N440", "N439", "N437"};
    static const int drvIdx[8] = {0, 0, 1, 1, 3, 3, 2, 2};  // Valf -> DRV index
    static const int outIdx[8] = {1, 2, 1, 2, 1, 2, 1, 2};  // Valf -> OUT1/OUT2
    static const int inaIdx[8] = {0, 3, 1, 2, 5, 7, 6, 4};  // PWM idx -> INA idx
    
    const float CURRENT_THRESHOLD_MA = 100.0f;  // Bağlı kabul etmek için minimum akım
    
    JsonArray valves = result["valves"].to<JsonArray>();
    
    // Her valfi sırayla test et
    for (int i = 0; i < 8; i++) {
      // DRV8243 fault register'larını temizle (CLR_FLT komutu)
      DRV_FaultClear(drvIdx[i]);
      vTaskDelay(pdMS_TO_TICKS(10));  // Temizleme için bekleme
      
      // g_drvLastFault'u temizle
      g_drvLastFault[drvIdx[i]].st1 = 0;
      g_drvLastFault[drvIdx[i]].st2 = 0;
      g_drvLastFault[drvIdx[i]].flt = 0;
      g_drvLastFault[drvIdx[i]].ok = true;
      
      // Valfi aç (yüksek duty ile akım ölçümü için)
      g_valveDutyCounts[i] = 2000;  // ~50% duty
      vTaskDelay(pdMS_TO_TICKS(150));  // INA219 ve DRV8243 güncellenmesi için bekle
      
      // INA219'dan akım oku (g_tele.inaI_mA INA sırasıyla dolu)
      float current_mA = g_tele.inaI_mA[inaIdx[i]];
      
      g_valveDutyCounts[i] = 0;  // Kapat
      vTaskDelay(pdMS_TO_TICKS(50));  // Fault register güncellemesi için bekle
      
      // 2 AŞAMALI KONTROL:
      // 1. Aşama: DRV8243 fault durumu
      uint8_t st2 = g_drvLastFault[drvIdx[i]].st2;
      bool drvOCP = false;  // Kısa devre
      bool drvOLA = false;  // Açık yük (DRV'ye göre)
      
      if (outIdx[i] == 1) {
        drvOLA = (st2 & 0x80) != 0;  // OLA1
        drvOCP = (st2 & 0x0C) != 0;  // OCP_H1/L1
      } else {
        drvOLA = (st2 & 0x40) != 0;  // OLA2
        drvOCP = (st2 & 0x03) != 0;  // OCP_H2/L2
      }
      
      // 2. Aşama: INA219 akım kontrolü
      bool hasCurrentFlow = (current_mA >= CURRENT_THRESHOLD_MA);
      
      // Sonuç belirleme:
      // - OCP varsa -> Kısa Devre (2)
      // - Akım akıyorsa (>100mA) -> Bağlı (0), DRV OLA'yı görmezden gel
      // - Akım akmıyorsa ve DRV OLA varsa -> Bağlı Değil (1)
      // - Akım akmıyorsa ve DRV OLA yoksa -> Bağlı Değil (1) (muhtemelen bağlantı yok)
      int status = 0;  // Varsayılan: Bağlı
      if (drvOCP) {
        status = 2;  // Kısa Devre
      } else if (!hasCurrentFlow) {
        status = 1;  // Bağlı Değil (akım akmıyor)
      }
      // else: hasCurrentFlow = true -> Bağlı (status=0)
      
      JsonObject v = valves.add<JsonObject>();
      v["name"] = valveNames[i];
      v["status"] = status;  // 0=Bağlı, 1=Bağlı Değil, 2=Kısa Devre
      v["current_mA"] = (int)current_mA;  // Debug için akım değeri
    }
    
    char buf[512];
    size_t len = serializeJson(result, buf, sizeof(buf));
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
  }

  // Kısayol: çalışma akımı
  if (doc["curr"].is<float>()) {
    float a = doc["curr"].as<float>();
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_pumpRunCurrent_A = a;   // doldurma pompası için eski akım seti
      g_pumpRunRpm = a;         // yeni: rpm alias
      g_pumpCmd.cmd = PUMP_CMD_SET_RPM;
      g_pumpCmd.seq++;
      g_pumpCmd.setRpm = a;
      xSemaphoreGive(g_sharedMutex);
    }
  }

  if (doc["rpm"].is<float>()) {
    float r = doc["rpm"].as<float>();
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_pumpCmd.cmd = PUMP_CMD_SET_RPM;
      g_pumpCmd.seq++;
      g_pumpCmd.setRpm = r;
      xSemaphoreGive(g_sharedMutex);
    }
  }

  // DRV OCP/TSD kilit sıfırlama  {"drv_fault_reset": true}
  // UYARI: Kısa devre giderilmeden bu komutu çalıştırmayın!
  if (doc["drv_fault_reset"].is<bool>() && doc["drv_fault_reset"].as<bool>()) {
    g_drvOcpLatch = false;
    DRV_EnableAll(true);   // DRVOFF aç — SPI erişebilsin
    delay(2);
    DRV_PresetAll();       // FaultClear + CONFIG1/2/3 tüm DRV'lere yeniden uygula
    for (int i = 0; i < 4; i++) {
      g_drvLastFault[i].st1 = 0;
      g_drvLastFault[i].st2 = 0;
      g_drvLastFault[i].flt = 0;
      g_drvLastFault[i].ok  = true;
    }
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, "[DRV] OCP kilit sifirlandi - preset yeniden yuklendi");
  }

  // DRV debug dump  {"drv_debug":true}
  if (doc["drv_debug"].is<bool>() && doc["drv_debug"].as<bool>()) {
    {
      char b[96];
      snprintf(b, sizeof(b), "[DRV_DBG] ocpLatch=%d", (int)g_drvOcpLatch);
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, b);
      // TCA output register durumunu oku (DRVOFF pin durumu)
      DRV_EnableAll(true);  // Zorla aç — sonra status oku
      delay(2);
      DRV8243Status st[4];
      DRV_GetAllStatus(st);
      for (int i = 0; i < 4; i++) {
        snprintf(b, sizeof(b), "[DRV_DBG] DRV%d st1=0x%02X st2=0x%02X flt=0x%02X ok=%d",
                 i+1, st[i].st1, st[i].st2, st[i].flt, (int)st[i].ok);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, b);
      }
      snprintf(b, sizeof(b), "[DRV_DBG] mA[4..7]=%.0f/%.0f/%.0f/%.0f",
               (float)g_valveCustomCurrent_mA[4], (float)g_valveCustomCurrent_mA[5],
               (float)g_valveCustomCurrent_mA[6], (float)g_valveCustomCurrent_mA[7]);
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, b);
    }
  }

  // Motor/Pompa test isteği  {"motor":{"cmd":"run","rpms":[500,1000], "dur":5, "min_dp":0.3, "Ilo":2, "Ihi":30}}
  if (doc["motor"].is<JsonObject>()) {
    auto m = doc["motor"].as<JsonObject>();
    const char* c = m["cmd"] | "run";
    if (!strcasecmp(c,"run")) {
      MotorPumpTestConfig cfg{};
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        cfg = g_motorTestCfg;
        xSemaphoreGive(g_sharedMutex);
      }
      if (m["rpms"].is<JsonArray>()) {
        JsonArray arr = m["rpms"].as<JsonArray>();
        int i=0;
        for (JsonVariant v : arr) {
          if (i>=5) break;
          cfg.rpms[i++] = v.as<float>();
        }
      }
      if (m["dur"].is<float>()) cfg.duration_ms = (uint32_t)(m["dur"].as<float>() * 1000.0f);
      else if (m["dur_ms"].is<uint32_t>()) cfg.duration_ms = m["dur_ms"].as<uint32_t>();
      if (m["min_dp"].is<float>()) cfg.min_dp_barps = m["min_dp"].as<float>();
      if (m["Ilo"].is<float>()) cfg.low_I_A = m["Ilo"].as<float>();
      if (m["Ihi"].is<float>()) cfg.high_I_A = m["Ihi"].as<float>();

      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_motorTestCfg = cfg;
        xSemaphoreGive(g_sharedMutex);
        portENTER_CRITICAL(&g_portMux);
        g_motorTestReqSeq++;
        portEXIT_CRITICAL(&g_portMux);
      }
    }
  }

  // OIL CHECK   {"oil":{"cmd":"check"}}
  if (doc["oil"].is<JsonObject>()) {
    auto j = doc["oil"].as<JsonObject>();
    const char* c = j["cmd"] | "";
    if (!strcasecmp(c,"check")) {
      portENTER_CRITICAL(&g_portMux);
      g_oilCheckRequestSeq++;
      portEXIT_CRITICAL(&g_portMux);
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[OIL] CHECK requested");
    }
  }

  // VALVE/PISTON TEST  {"vp":{"cmd":"run"}}
  if (doc["vp"].is<JsonObject>()) {
    auto j = doc["vp"].as<JsonObject>();
    const char* c = j["cmd"] | "run";
    if (!strcasecmp(c,"run")) {
      portENTER_CRITICAL(&g_portMux);
      g_valvePistonReqSeq++;
      portEXIT_CRITICAL(&g_portMux);
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[VP] request");
    }
  }

  // PISTON MID TEST {"pmid":{"dur":30,"drop":5.0}}
  if (doc["pmid"].is<JsonObject>()) {
    auto j = doc["pmid"].as<JsonObject>();
    PistonMidTestConfig cfg{};
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      cfg = g_pistonMidCfg;
      xSemaphoreGive(g_sharedMutex);
    }
    if (j["dur"].is<uint32_t>()) cfg.duration_ms = j["dur"].as<uint32_t>() * 1000u;
    if (j["drop"].is<float>())   cfg.drop_thresh_bar = j["drop"].as<float>();
    if (cfg.duration_ms == 0) cfg.duration_ms = 30000;
    if (cfg.drop_thresh_bar <= 0) cfg.drop_thresh_bar = 5.0f;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_pistonMidCfg = cfg;
      xSemaphoreGive(g_sharedMutex);
      portENTER_CRITICAL(&g_portMux);
      g_pistonMidReqSeq++;
      portEXIT_CRITICAL(&g_portMux);
    }
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[PMID] request");
  }

  // PISTON GRAFIK TEST {"pgraf":{"ds":1800,"de":2400,"step":150,"ms":150,"samp":100}}
  if (doc["pgraf"].is<JsonObject>()) {
    auto j = doc["pgraf"].as<JsonObject>();
    PistonGraphConfig cfg{};
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      cfg = g_pistonGraphCfg;
      xSemaphoreGive(g_sharedMutex);
    }
    if (j["ds"].is<uint16_t>()) cfg.duty_start = j["ds"].as<uint16_t>();
    if (j["de"].is<uint16_t>()) cfg.duty_end   = j["de"].as<uint16_t>();
    if (j["step"].is<uint16_t>()) cfg.duty_step = j["step"].as<uint16_t>();
    if (j["ms"].is<uint16_t>()) cfg.step_ms = j["ms"].as<uint16_t>();
    if (j["samp"].is<uint16_t>()) cfg.sample_ms = j["samp"].as<uint16_t>();
    if (cfg.duty_end < cfg.duty_start) cfg.duty_end = cfg.duty_start + 400;
    if (cfg.duty_step == 0) cfg.duty_step = 150;
    if (cfg.step_ms == 0) cfg.step_ms = 150;
    if (cfg.sample_ms == 0) cfg.sample_ms = 100;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_pistonGraphCfg = cfg;
      xSemaphoreGive(g_sharedMutex);
      portENTER_CRITICAL(&g_portMux);
      g_pistonGraphReqSeq++;
      portEXIT_CRITICAL(&g_portMux);
    }
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[PGRAF] request");
  }

  // DİNAMİK VİTES GEÇİŞ TESTİ {"dyngear":{"start":true}}
  if (doc["dyngear"].is<JsonObject>()) {
    auto j = doc["dyngear"].as<JsonObject>();
    if (j["start"].is<bool>() && j["start"].as<bool>()) {
      portENTER_CRITICAL(&g_portMux);
      g_dynGearTestReqSeq++;
      portEXIT_CRITICAL(&g_portMux);
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[DYNGEAR] request");
    }
  }
  
  // VALF ADAPTASYONU {"adapt":{"start":true}}
  if (doc["adapt"].is<JsonObject>()) {
    auto j = doc["adapt"].as<JsonObject>();
    if (j["start"].is<bool>() && j["start"].as<bool>()) {
      portENTER_CRITICAL(&g_portMux);
      g_valveAdaptReqSeq++;
      portEXIT_CRITICAL(&g_portMux);
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[ADAPT] request");
    }
  }
  
  // PID PARAMETRELER {"pid":{"kp":12,...}} veya {"pid":"get"}
  if (doc.containsKey("pid")) {
    if (doc["pid"].is<JsonObject>()) {
      // Set parametreleri
      auto pj = doc["pid"].as<JsonObject>();
      if (pj["kp"].is<float>()) g_pidParams.Kp = pj["kp"].as<float>();
      if (pj["ki"].is<float>()) g_pidParams.Ki = pj["ki"].as<float>();
      if (pj["kd"].is<float>()) g_pidParams.Kd = pj["kd"].as<float>();
      if (pj["max_i"].is<float>()) g_pidParams.maxIntegral = pj["max_i"].as<float>();
      if (pj["db"].is<float>()) g_pidParams.deadband = pj["db"].as<float>();
      if (pj["ctrl_ms"].is<int>()) g_pidParams.controlMs = pj["ctrl_ms"].as<int>();
      if (pj["hold_ms"].is<int>()) g_pidParams.holdTimeMs = pj["hold_ms"].as<int>();
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[PID] params updated");
    }
    // Mevcut değerleri text olarak gönder (JSON çakışmasını önlemek için)
    {
      char buf[128];
      snprintf(buf, sizeof(buf), "[PID] Kp=%.2f Ki=%.3f Kd=%.2f maxI=%.0f db=%.2f ctrl=%d hold=%lu",
        g_pidParams.Kp, g_pidParams.Ki, g_pidParams.Kd, 
        g_pidParams.maxIntegral, g_pidParams.deadband,
        g_pidParams.controlMs, g_pidParams.holdTimeMs);
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
    }
  }
  
  // PID AUTO-TUNE {"pid_tune":{"piston":0,"target":2.0}}
  if (doc["pid_tune"].is<JsonObject>()) {
    auto tj = doc["pid_tune"].as<JsonObject>();
    g_pidTuneReq.piston = tj["piston"].as<int>();
    g_pidTuneReq.targetPos = tj["target"].as<float>();
    g_pidTuneReq.active = true;
    portENTER_CRITICAL(&g_portMux);
    g_pidTuneReqSeq++;
    portEXIT_CRITICAL(&g_portMux);
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[PID_TUNE] request received");
  }

  // Genel test stop
  if (doc["test"].is<JsonObject>()) {
    auto tj = doc["test"].as<JsonObject>();
    if (tj["stop"].is<bool>() && tj["stop"].as<bool>()) {
      g_diagAbortFlag = true;
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[TEST] stop request");
    }
  }
  if (doc["stop"].is<bool>() && doc["stop"].as<bool>()) {
    g_diagAbortFlag = true;
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[TEST] stop request");
  }
  // GUI abort komutu ({"abort": true})
  if (doc["abort"].is<bool>() && doc["abort"].as<bool>()) {
    g_diagAbortFlag = true;
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[TEST] abort request");
  }

  // -------- ISITICI (HEATER) KONTROLÜ --------
  // {"heater":{"on":true,"setpoint":40,"on_sec":10,"off_sec":5}}
  // on_sec / off_sec: duty cycle süresi (saniye). 0 = kesintisiz mod.
  // setpoint: hedef yağ sıcaklığı (°C). 0 veya yoksa devre dışı.
  if (doc["heater"].is<JsonObject>()) {
    auto hj = doc["heater"].as<JsonObject>();

    // Duty cycle parametreleri (opsiyonel — gönderilmezse mevcut değer korunur)
    if (!hj["on_sec"].isNull()) {
      float s = hj["on_sec"].as<float>();
      g_heater_on_ms = (s > 0.0f) ? (uint16_t)(s * 1000.0f) : 0;
    }
    if (!hj["off_sec"].isNull()) {
      float s = hj["off_sec"].as<float>();
      g_heater_off_ms = (s > 0.0f) ? (uint16_t)(s * 1000.0f) : 0;
    }

    // Setpoint (opsiyonel)
    if (!hj["setpoint"].isNull()) {
      g_heater_setpoint = hj["setpoint"].as<float>();
    }

    bool heaterOn = hj["on"].as<bool>();
    if (heaterOn) {
      g_ssrDesired = 1;
      {
        char buf[80];
        snprintf(buf, sizeof(buf), "[HEATER] ON | duty=%us/%us | setpt=%.0fC",
                 (unsigned)(g_heater_on_ms / 1000), (unsigned)(g_heater_off_ms / 1000),
                 g_heater_setpoint);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
      }
    } else {
      g_ssrDesired = 0;
      g_heater_setpoint = 0.0f;
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[HEATER] OFF");
    }
  }

  // -------- DIAG TEST CONTROL --------
  /*
  {"test":{"cmd":"smoke","I":3.0,"t":5,"pmin":3,"hold":5,"pmax":70}}
  {"test":{"cmd":"pumpaccu","I":4.0,"p1":30,"p2":60,"hold":10,"timeout":20,"pmax":70}}
  */
 #if 0
 #endif

// -------- Hızlı Sağlık Kontrolü (Quick Health) --------
  // Örnek:
  // {"qh":{"p":60,"hold":30,"leakMax":0.5,"rpm":3000}}
  //  p / bar  : hedef basınç [bar]
  //  hold     : bekleme süresi [s]
  //  leakMax  : izin verilen max kaçak [bar/dk]
  //  rpm      : pompa hızı [rpm] (eski I/curr alanları rpm olarak kabul edilir)
  if (doc["qh"].is<JsonObject>()) {
    auto j = doc["qh"].as<JsonObject>();
    QuickHealthConfig cfg{};

    // Mevcut ayarları baz al
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      cfg = g_quickHealthCfg;
      xSemaphoreGive(g_sharedMutex);
    }

    if (j["p"].is<float>())        cfg.target_bar = j["p"].as<float>();
    else if (j["bar"].is<float>()) cfg.target_bar = j["bar"].as<float>();

    if (j["hold"].is<float>())     cfg.hold_s = j["hold"].as<float>();
    else if (j["t"].is<float>())   cfg.hold_s = j["t"].as<float>();

    if (j["leakMax"].is<float>()) {
      float leak_bar_per_min = j["leakMax"].as<float>();
      cfg.maxLeak_barps = leak_bar_per_min / 60.0f * 5.0f;
    }

    if (j["rpm"].is<float>())      cfg.pumpRpm = j["rpm"].as<float>();
    else if (j["I"].is<float>())   cfg.pumpRpm = j["I"].as<float>();
    else if (j["curr"].is<float>())cfg.pumpRpm = j["curr"].as<float>();

    if (j["fillTimeout"].is<uint32_t>()) cfg.fillTimeout_ms = j["fillTimeout"].as<uint32_t>();
    else if (j["tout"].is<uint32_t>())   cfg.fillTimeout_ms = j["tout"].as<uint32_t>();

    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_quickHealthCfg = cfg;
      xSemaphoreGive(g_sharedMutex);
      portENTER_CRITICAL(&g_portMux);
      g_quickHealthReqSeq++;
      portEXIT_CRITICAL(&g_portMux);
    }

    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[QH] request");
  }

  // -------- Sistem Kaçak Testi --------
// Örnek:
// {"leak":{"p":60,"settle":20,"hold":10,"leakMax":0.5,"rpm":3000,
//          "preAbove":10,"preTo":5,"preTout":8000}}
  if (doc["leak"].is<JsonObject>()) {
    auto j = doc["leak"].as<JsonObject>();
    LeakTestConfig cfg{};

    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      cfg = g_leakCfg;
      xSemaphoreGive(g_sharedMutex);
    }

    if (j["p"].is<float>())        cfg.target_bar = j["p"].as<float>();
    if (j["settle"].is<float>())   cfg.settle_s   = j["settle"].as<float>();
    if (j["hold"].is<float>())     cfg.hold_s     = j["hold"].as<float>();
    if (j["leakMax"].is<float>())  cfg.maxLeak_barps = j["leakMax"].as<float>() / 60.0f; // bar/dk -> bar/s
    if (j["rpm"].is<float>())      cfg.pumpRpm = j["rpm"].as<float>();
    else if (j["I"].is<float>())   cfg.pumpRpm = j["I"].as<float>();

    if (j["tout"].is<uint32_t>())  cfg.fillTimeout_ms = j["tout"].as<uint32_t>();

    if (j["preAbove"].is<float>()) cfg.preDrainAbove_bar = j["preAbove"].as<float>();
    if (j["preTo"].is<float>())    cfg.preDrainTo_bar    = j["preTo"].as<float>();
    if (j["preTout"].is<uint32_t>()) cfg.preDrainTimeout_ms = j["preTout"].as<uint32_t>();

    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_leakCfg = cfg;
      xSemaphoreGive(g_sharedMutex);
      portENTER_CRITICAL(&g_portMux);
      g_leakReqSeq++;
      portEXIT_CRITICAL(&g_portMux);
    }

    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[LEAK] request");
  }

  //--------------AUTO Control---------------
  // {"auto":{"start":true, "repeat":1, "gear_ms":1200}} (1 tur, her adÄ±m 1.2 sn):
  // {"auto":{"start":true, "repeat":3, "gear_s":0.5}} (3 tur, her adÄ±m min. 1 sn)
  //{"auto":{"start":false}} (Durdur) 

  if (doc["auto"].is<JsonObject>())
  {
    auto J = doc["auto"].as<JsonObject>();
    AutoShiftReq r{};
    r.start   = J["start"] | false;
    r.repeats = J["repeat"] | J["repeats"] | 1;
    if (r.repeats == 0) r.repeats = 1;

    uint32_t ms = 0;
    if (J.containsKey("gear_ms")) ms = J["gear_ms"].as<uint32_t>();
    else if (J.containsKey("gear_s")) ms = (uint32_t)(J["gear_s"].as<float>() * 1000.0f);
    if (ms < 1000) ms = 1000;
    r.gear_ms = ms;

    if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE){
      g_autoShiftReq = r;
      xSemaphoreGive(g_sharedMutex);
      // seq++
      portENTER_CRITICAL(&g_portMux);
      g_autoShiftReqSeq++;
      portEXIT_CRITICAL(&g_portMux);
    }
  }

  //--------------AUTO V2 Control (Yeni Vites Kontrol Sistemi)---------------
  // {"autoV2":{"start":true, "manualMode":false, "targetGear":9, "gearHoldMs":2000, "repeatCount":1}}
  // {"autoV2":{"start":false}} (Durdur)
  // {"autoV2":{"targetGear":5}} (Manuel modda hedef vites değiştir)
  if (doc["autoV2"].is<JsonObject>())
  {
    auto J = doc["autoV2"].as<JsonObject>();
    
    if (J.containsKey("start")) {
      AutoShiftV2Request r{};
      r.start = J["start"] | false;
      r.manualMode = J["manualMode"] | false;
      r.targetGear = (GearState)(J["targetGear"].as<uint8_t>());
      r.gearHoldMs = J["gearHoldMs"] | 2000;
      r.repeatCount = J["repeatCount"] | 1;
      
      if (r.gearHoldMs < 500) r.gearHoldMs = 500;
      if (r.repeatCount == 0) r.repeatCount = 1;
      
      if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE){
        g_autoShiftV2Req = r;
        xSemaphoreGive(g_sharedMutex);
        bumpSeq(g_autoShiftV2ReqSeq);
      }
    } else if (J.containsKey("targetGear")) {
      // Sadece hedef vites güncellemesi (manuel mod için)
      if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE){
        g_autoShiftV2Req.targetGear = (GearState)(J["targetGear"].as<uint8_t>());
        xSemaphoreGive(g_sharedMutex);
        bumpSeq(g_autoShiftV2ReqSeq);
      }
    }
  }

  //--------------VALF DİAGNOSTİK TESTİ---------------
  // {"valve_test":{"valve":0,"duty":2000,"step":10,"ms":100,"pressure":50}}
  if (doc["valve_test"].is<JsonObject>())
  {
    auto J = doc["valve_test"].as<JsonObject>();
    ValveDiagRequest r{};
    r.valveIdx       = J["valve"]    | (uint8_t)0;
    r.dutyMax        = J["duty"]     | (uint16_t)2000;
    r.dutyStep       = J["step"]     | (uint16_t)10;
    r.stepMs         = J["ms"]       | (uint16_t)100;
    r.pressureTarget = J["pressure"] | 50.0f;
    r.start          = true;

    if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_valveDiagReq = r;
      xSemaphoreGive(g_sharedMutex);
      bumpSeq(g_valveDiagReqSeq);
    }
  }

  //--------------PCV CHARACTERIZATION TEST---------------
  // {"pcv_char_test": { "cmd":"start"|"abort", ... }}
  if (doc["pcv_char_test"].is<JsonObject>()){
    auto J = doc["pcv_char_test"].as<JsonObject>();
    const char* cmd = J["cmd"] | "";
    if (!strcasecmp(cmd, "abort")){
      if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE){
        g_pcvCharReq.abort = true;
        xSemaphoreGive(g_sharedMutex);
      }
      bumpSeq(g_pcvCharReqSeq);
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[PCV_CHAR] abort req accepted");
    } else if (!strcasecmp(cmd, "start")){
      PCVCharRequest r{};
      const char* pcvName = J["pcv"] | "";
      r.pcvValveIdx = (!strcasecmp(pcvName, "N436")) ? 1 : (!strcasecmp(pcvName, "N440") ? 5 : 255);
      r.pistonIdx = J["piston"] | 255;
      if (J["pcv_currents_mA"].is<JsonArray>()){
        auto arr = J["pcv_currents_mA"].as<JsonArray>();
        uint8_t n = 0; for (auto v : arr){ if (n < PCVCHAR_MAX_STEPS) { r.pcv_mA[n++] = (uint16_t)(int)v; } }
        r.stepCount = n;
      }
      r.open_mA = J["open_current_mA"] | 610;
      r.close_mA = J["close_current_mA"] | 450;
      r.mid_target_mm = J["mid_target_mm"] | 13.0f;
      r.closed_thresh_mm = J["closed_threshold_mm"] | 1.0f;
      r.full_open_thresh_mm = J["full_open_threshold_mm"] | 25.0f;
      r.move_detect_mm = J["move_detect_mm"] | 1.0f;
      r.pressure_ready_bar = J["pressure_ready_bar"] | 55.0f;
      r.pressure_min_bar = J["pressure_min_bar"] | 42.0f;
      r.settle_before_step_ms = J["settle_before_step_ms"] | 500;
      r.max_open_ms = J["max_open_ms"] | 3000;
      r.brake_observe_ms = J["brake_observe_ms"] | 1200;
      r.max_close_ms = J["max_close_ms"] | 3000;
      r.repeats = J["repeats"] | 1;
      r.wait_pressure_each_step       = J["wait_pressure_each_step"]        | false;
      r.step_ready_pressure_bar       = J["step_ready_pressure_bar"]        | 58.0f;
      r.step_ready_pressure_window_bar= J["step_ready_pressure_window_bar"] | 2.0f;
      r.step_pressure_wait_timeout_ms = J["step_pressure_wait_timeout_ms"]  | (uint32_t)15000;
      r.start = true;
      r.abort = false;
      if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE){
        g_pcvCharReq = r;
        xSemaphoreGive(g_sharedMutex);
      }
      bumpSeq(g_pcvCharReqSeq);
      {
        char buf[96];
        snprintf(buf, sizeof(buf), "[PCV_CHAR] start req accepted pcv=%s piston=%u steps=%u", 
                 pcvName, (unsigned)r.pistonIdx, (unsigned)r.stepCount);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
      }
    }
  }

  //--------------HALL STABILITE TESTİ---------------
  // {"hall_test":{"start":true,"piston":0,"duration_ms":1000,"sample_interval_ms":1,"test_type":"static"}}
  // {"hall_test":{"stop":true}}
  if (doc["hall_test"].is<JsonObject>())
  {
    auto J = doc["hall_test"].as<JsonObject>();
    HallStabilityRequest r{};
    
    if (J["stop"].is<bool>() && J["stop"].as<bool>()) {
      r.start = false;
    } else {
      r.start          = J["start"]             | false;
      r.pistonIdx      = J["piston"]            | (uint8_t)0;
      r.durationMs     = J["duration_ms"]       | (uint16_t)1000;
      r.sampleIntervalMs = J["sample_interval_ms"] | (uint8_t)1;
      
      // test_type: "static"=0, "emi"=1, "dynamic"=2
      const char* typeStr = J["test_type"] | "static";
      if (strcmp(typeStr, "emi") == 0) r.testType = 1;
      else if (strcmp(typeStr, "dynamic") == 0) r.testType = 2;
      else r.testType = 0;
    }

    if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      g_hallStabilityReq = r;
      xSemaphoreGive(g_sharedMutex);
      bumpSeq(g_hallStabilityReqSeq);
      
      {
        char buf[64];
        if (r.start) {
          snprintf(buf, sizeof(buf), "[HS] REQ received: piston=%d duration=%d interval=%d", 
                   r.pistonIdx, r.durationMs, r.sampleIntervalMs);
        } else {
          snprintf(buf, sizeof(buf), "[HS] REQ received: STOP");
        }
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
      }
    }
  }

  //--------------HOLD KALIBRASYONU---------------
  // {"hold_cal":{"piston":0,"start":true}}
  // {"hold_cal":{"stop":true}}
  if (doc["hold_cal"].is<JsonObject>())
  {
    auto J = doc["hold_cal"].as<JsonObject>();
    
    if (J["stop"].is<bool>() && J["stop"].as<bool>()) {
      stopHoldCalibration();
      {
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[HCAL] STOP requested");
      }
    } else if (J["start"].is<bool>() && J["start"].as<bool>()) {
      uint8_t pistonIdx = J["piston"] | (uint8_t)0;
      if (startHoldCalibration(pistonIdx)) {
        {
          char buf[48];
          snprintf(buf, sizeof(buf), "[HCAL] START requested: piston=%d", pistonIdx);
          kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
        }
      } else {
        {
          kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[HCAL] START failed (already running or invalid piston)");
        }
      }
    }
  }

  // =====================================================================
  // AKIM KALİBRASYON: {"current_calib":{"piston":0}} veya {"current_calib":{"all":true}}
  //                   {"current_calib":{"abort":true}}
  if (doc.containsKey("current_calib")) {
    auto cc = doc["current_calib"].as<JsonObject>();
    if (cc["abort"].is<bool>() && cc["abort"].as<bool>()) {
      // Direkt flag'i kapat — ccWait() icinden aninda gorunur
      g_currentCalibRunning = false;
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_currentCalibReq.abort  = true;
        g_currentCalibReq.all    = false;
        g_currentCalibReq.piston = 0;
        xSemaphoreGive(g_sharedMutex);
      }
      bumpSeq(g_currentCalibReqSeq);
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[CCAL] Abort requested");
    } else {
      bool doAll = cc["all"].is<bool>() && cc["all"].as<bool>();
      int  pistonIdx = 0;
      if (!doAll) {
        if (cc["piston"].is<int>()) {
          pistonIdx = cc["piston"].as<int>();
        } else if (cc["piston"].is<const char*>()) {
          const char *ps = cc["piston"].as<const char*>();
          if      (!strcmp(ps, "P5-7") || !strcmp(ps, "0")) pistonIdx = 0;
          else if (!strcmp(ps, "P1-3") || !strcmp(ps, "1")) pistonIdx = 1;
          else if (!strcmp(ps, "P2-4") || !strcmp(ps, "2")) pistonIdx = 2;
          else if (!strcmp(ps, "P6-R") || !strcmp(ps, "3")) pistonIdx = 3;
        }
        if (pistonIdx < 0 || pistonIdx > 3) pistonIdx = 0;
      }
      if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_currentCalibReq.abort  = false;
        g_currentCalibReq.piston = doAll ? 0 : (uint8_t)pistonIdx;
        g_currentCalibReq.all    = doAll;
        xSemaphoreGive(g_sharedMutex);
      }
      bumpSeq(g_currentCalibReqSeq);
      {
        char buf[56];
        snprintf(buf, sizeof(buf), "[CCAL] Start requested: p=%d all=%d", pistonIdx, (int)doAll);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
      }
    }
  }

  // ÜRETİCİ MODU (Manufacturer Mode): {"mfg":true} / {"mfg":false}
  // true iken: safety check'ler (basınç, valf sıralaması) bypass edilir
  if (doc["mfg"].is<bool>()) {
    bool ena = doc["mfg"].as<bool>();
    portENTER_CRITICAL(&g_portMux);
    g_manufacturerMode = ena;
    portEXIT_CRITICAL(&g_portMux);
    {
      char buf[48];
      snprintf(buf, sizeof(buf), "[MFG] Manufacturer mode %s", ena ? "ENABLED" : "disabled");
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, buf);
    }
  }

  //--------------9-FAZLI OTOMATİK TEST---------------
  // {"at_start":true}          → Testi başlat
  // {"at_stop":true}           → Testi durdur
  // {"at_params_get":true}     → Parametreleri oku ve gönder
  // {"at_params_set":{...}}    → Parametreleri yaz + NVS'e kaydet

  if (doc["at_start"].is<bool>() && doc["at_start"].as<bool>()) {
    // Opsiyonel parametreler: gearHoldMs, repeatCount
    if (doc["gearHoldMs"].is<uint32_t>()) {
      uint32_t ms = doc["gearHoldMs"].as<uint32_t>();
      if (ms >= 500 && ms <= 10000) g_autoTestParams.gearHoldMs = ms;
    }
    if (doc["repeatCount"].is<uint8_t>() || doc["repeatCount"].is<int>()) {
      int rep = doc["repeatCount"].as<int>();
      if (rep >= 1 && rep <= 250) g_autoTestParams.autoShiftRepeats = (uint16_t)rep;
    }
    portENTER_CRITICAL(&g_portMux);
    g_autoTestStop = false;
    g_autoTestReqSeq++;
    portEXIT_CRITICAL(&g_portMux);
    {
      char msg[64];
      snprintf(msg, sizeof(msg), "[AT] 9-faz test baslatildi: %u tekrar, %ums",
               (unsigned)g_autoTestParams.autoShiftRepeats, (unsigned)g_autoTestParams.gearHoldMs);
      kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
    }
  }

  if (doc["at_stop"].is<bool>() && doc["at_stop"].as<bool>()) {
    portENTER_CRITICAL(&g_portMux);
    g_autoTestStop = true;
    portEXIT_CRITICAL(&g_portMux);
    kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, (char*)"[AT] Test durdurma istegi");
  }

  if (doc["at_params_get"].is<bool>() && doc["at_params_get"].as<bool>()) {
    StaticJsonDocument<512> resp;
    resp["_t"] = "ATP";
    const auto& p = g_autoTestParams;
    resp["coilMa"] = p.coilMinCurrentMa;
    resp["tBar"]   = p.targetBar;
    resp["pfMax"]  = p.pumpFillMaxSec;
    resp["pfTmo"]  = p.pumpFillTimeoutMs;
    resp["mvThr"]  = p.movementThreshold;
    resp["lkWt"]   = p.leakCheckWaitMs;
    resp["olDrp"]  = p.oilLeakMaxDrop_bar;
    resp["olHld"]  = p.oilLeakHoldSec;
    resp["cPwm"]   = p.calPwm;
    resp["cTmo"]   = p.calTimeoutMs;
    resp["hTol"]   = p.holdMidTolPct;
    resp["hStbl"]  = p.holdStableMs;
    resp["asRep"]  = p.autoShiftRepeats;
    resp["gHld"]   = p.gearHoldMs;
    resp["adEn"]   = p.adaptiveHoldEnabled;
    resp["adMx"]   = p.adaptivePwmMaxOffset;
    resp["adThr"]  = p.adaptThreshMm;
    static char atpBuf[512];
    size_t n = serializeJson(resp, atpBuf, sizeof(atpBuf) - 1);
    if (n > 0) { atpBuf[n] = '\0'; kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, atpBuf); }
  }

  if (doc["at_params_set"].is<JsonObject>()) {
    auto J = doc["at_params_set"].as<JsonObject>();
    auto& p = g_autoTestParams;
    if (J["coilMa"].is<float>()) p.coilMinCurrentMa     = J["coilMa"].as<float>();
    if (J["tBar"].is<float>())   p.targetBar            = J["tBar"].as<float>();
    if (J["pfMax"].is<float>())  p.pumpFillMaxSec       = J["pfMax"].as<float>();
    if (J["pfTmo"].is<uint32_t>()) p.pumpFillTimeoutMs  = J["pfTmo"].as<uint32_t>();
    if (J["mvThr"].is<int>())    p.movementThreshold    = (uint16_t)J["mvThr"].as<int>();
    if (J["lkWt"].is<uint32_t>()) p.leakCheckWaitMs     = J["lkWt"].as<uint32_t>();
    if (J["olDrp"].is<float>())  p.oilLeakMaxDrop_bar   = J["olDrp"].as<float>();
    if (J["olHld"].is<uint32_t>()) p.oilLeakHoldSec     = J["olHld"].as<uint32_t>();
    if (J["cPwm"].is<int>())     p.calPwm               = (uint16_t)J["cPwm"].as<int>();
    if (J["cTmo"].is<uint32_t>()) p.calTimeoutMs        = J["cTmo"].as<uint32_t>();
    if (J["hTol"].is<float>())   p.holdMidTolPct        = J["hTol"].as<float>();
    if (J["hStbl"].is<uint32_t>()) p.holdStableMs       = J["hStbl"].as<uint32_t>();
    if (J["asRep"].is<int>())    p.autoShiftRepeats      = (uint16_t)J["asRep"].as<int>();
    if (J["gHld"].is<uint32_t>()) p.gearHoldMs          = J["gHld"].as<uint32_t>();
    if (J["adEn"].is<bool>())    p.adaptiveHoldEnabled   = J["adEn"].as<bool>();
    if (J["adMx"].is<int>())     p.adaptivePwmMaxOffset  = (uint16_t)J["adMx"].as<int>();
    if (J["adThr"].is<float>())  p.adaptThreshMm         = J["adThr"].as<float>();
    AutoTestParams_SaveNVS();
    {
      JsonDocument pdoc;
      auto p = pdoc["p"].to<JsonArray>();
      sendMsgPackLog(kitronic::MsgCode::AT_PARAMS_SAVED, p);
    }
  }

  return;
}

// Binary frame alıcı (length-prefixed, CRC'li)
static bool tryReadOneFrame(uint8_t* outBuf, size_t outMax, size_t& outLen, uint8_t& outType) {
  static uint8_t rb[RX_BUF_MAX];
  static size_t rp = 0;
  static bool inFrame = false;
  static uint16_t expectedLen = 0;

  while (Serial.available()) {
    int c = Serial.read();
    if (c < 0) break;

    if (!inFrame) {
      if (c == kitronic::FRAME_START) {
        inFrame = true;
        rp = 0;
        rb[rp++] = (uint8_t)c;
      }
      continue;
    }

    if (rp < RX_BUF_MAX) {
      rb[rp++] = (uint8_t)c;
    } else {
      // overflow
      inFrame = false;
      rp = 0;
      continue;
    }

    if (rp >= 5) {
      expectedLen = (uint16_t)rb[2] | ((uint16_t)rb[3] << 8);
      size_t total = (size_t)expectedLen + kitronic::FRAME_OVERHEAD;
      if (total > RX_BUF_MAX) {
        inFrame = false;
        rp = 0;
        continue;
      }
      if (rp >= total) {
        kitronic::FrameHeader hdr;
        const uint8_t* payload;
        size_t off = kitronic::decodeFrame(rb, rp, hdr, payload);
        if (off > 0 && hdr.length <= outMax) {
          outType = hdr.type;
          outLen = hdr.length;
          memcpy(outBuf, payload, hdr.length);
          inFrame = false;
          rp = 0;
          return true;
        }
        // Geçersiz frame; sonraki START ara
        inFrame = false;
        size_t startIdx = 0;
        for (size_t i = 1; i < rp; i++) {
          if (rb[i] == kitronic::FRAME_START) {
            startIdx = i;
            break;
          }
        }
        if (startIdx > 0 && startIdx < rp) {
          memmove(rb, &rb[startIdx], rp - startIdx);
          rp = rp - startIdx;
          inFrame = true;
        } else {
          rp = 0;
        }
      }
    }
  }
  return false;
}

// ======= Ana Task =======
void TaskSerial(void *pvParameters) {
  (void)pvParameters;

  Serial.setTimeout(5);
  static uint8_t rx[RX_BUF_MAX];
  size_t rxLen = 0;
  uint8_t rxType = 0;

  uint32_t lastTx = 0;
  static const uint32_t TX_PERIOD_MS = 100;   // JSON telemetri gönderim periyodu (10 Hz - 5x hızlı)
  lastTx = millis() - TX_PERIOD_MS;   // ilk turda kesin gönder

  // Non-blocking satÄ±r toplayÄ±cÄ±
  Serial.setTimeout(2);  // emniyet

  for (;;) {
    // 1) Binary frame komut var mi?
    if (tryReadOneFrame(rx, sizeof(rx), rxLen, rxType)) {
      parseAndDispatch(rxType, rx, (uint16_t)rxLen);
    }
    // 2) Periyodik telemetri gönder
    uint32_t now = millis();
    if (now - lastTx >= TX_PERIOD_MS) {
      sendTelemetryJSON();
      lastTx = now;
    }

    vTaskDelay(pdMS_TO_TICKS(10));  // 10ms - TX queue hızlı boşalt (piston graph için)
  }
}

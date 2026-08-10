#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <ArduinoJson.h>

namespace kitronic {

// ============================================================
// Length-prefixed binary frame
// [START][TYPE][LEN_L][LEN_H][SEQ][PAYLOAD...][CRC_L][CRC_H][END]
// CRC16-CCITT covers [TYPE][LEN_L][LEN_H][SEQ][PAYLOAD...]
// ============================================================
constexpr uint8_t FRAME_START = 0xA5;
constexpr uint8_t FRAME_END   = 0x5A;
constexpr uint8_t FRAME_HEADER_SIZE = 5;  // START + TYPE + 2*LEN + SEQ
constexpr uint8_t FRAME_TRAILER_SIZE = 3; // CRC_L + CRC_H + END
constexpr uint8_t FRAME_OVERHEAD = FRAME_HEADER_SIZE + FRAME_TRAILER_SIZE;

enum FrameType : uint8_t {
  // Fixed binary telemetry (ESP -> PC)
  FT_SENSOR  = 0x01,
  FT_PISTON  = 0x02,
  FT_INA     = 0x03,
  FT_POWER   = 0x04,
  FT_VERSION = 0x05,

  // MessagePack payloads
  FT_COMMAND = 0x10, // PC -> ESP command
  FT_RESULT  = 0x11, // ESP -> PC test/result update
  FT_LOG     = 0x12, // ESP -> PC localized log message
  FT_EVENT   = 0x13, // ESP -> PC single-point event (VD, HS, CC, HCAL, PCV step)
  FT_REQUEST = 0x14, // PC -> ESP request that expects an ack
  FT_ACK     = 0x15, // ESP -> PC ack
};

// ============================================================
// Message codes (ESP -> PC localization)
// GUI translates these with parameters.
// ============================================================
enum class MsgCode : uint8_t {
  NONE = 0,

  // System
  SYS_BOOT,
  FIRMWARE_VERSION,
  HEAP_WARNING,
  RX_ERROR,
  UNKNOWN_COMMAND,
  TELEMETRY_RATE_SET,

  // Power / SSR
  SSR_ON,
  SSR_OFF,
  PUMP_STARTED,
  PUMP_STOPPED,
  PUMP_RPM_SET,
  HEATER_ON,
  HEATER_OFF,

  // Calibration
  CALIB_STARTED,
  CALIB_DONE,
  CALIB_ABORTED,
  HOLD_CALIB_STARTED,
  HOLD_CALIB_STOPPED,
  HOLD_CALIB_FAILED,
  CURRENT_CALIB_STARTED,
  CURRENT_CALIB_ABORTED,
  FIND_HOLD_STARTED,

  // Generic tests
  TEST_STARTED,
  TEST_DONE,
  TEST_ABORTED,
  TEST_FAILED,

  // Specific tests
  QH_STARTED,
  QH_DONE,
  QH_FAILED,
  LEAK_STARTED,
  LEAK_DONE,
  LEAK_FAILED,
  MOTOR_STARTED,
  MOTOR_DONE,
  MOTOR_FAILED,
  VP_STARTED,
  VP_DONE,
  VP_FAILED,
  PMID_STARTED,
  PMID_DONE,
  PMID_FAILED,
  AT_STARTED,
  AT_DONE,
  AT_ABORTED,
  AT_PARAMS_SAVED,
  VALVE_CHECK_STARTED,
  VALVE_CHECK_DONE,
  HW_TEST_STARTED,
  HW_TEST_DONE,
  VALVE_DIAG_STARTED,
  VALVE_DIAG_DONE,
  PCV_CHAR_STARTED,
  PCV_CHAR_DONE,
  PCV_CHAR_ABORTED,
  HALL_TEST_STARTED,
  HALL_TEST_STOPPED,
  HALL_TEST_DONE,
  AUTO_SHIFT_STARTED,
  AUTO_SHIFT_DONE,
  AUTO_SHIFT_ABORTED,
  DYNAMIC_GEAR_STARTED,
  VALVE_ADAPT_STARTED,
  PID_PARAMS_UPDATED,
  PID_TUNE_REQUESTED,

  // DRV / faults
  DRV_FAULT,
  DRV_OCP_RESET,
  DRV_DEBUG_DUMP,
  OCP_LATCHED,
  OCP_LATCH_CLEARED,

  // Manufacturer mode
  MFG_MODE_ENABLED,
  MFG_MODE_DISABLED,

  // Valf cleaning / pin test
  VALVE_CLEAN_ON,
  VALVE_CLEAN_OFF,
  PIN_TEST_STATE,
};

// ============================================================
// Fixed binary telemetry structs (packed, little-endian)
// ============================================================
struct __attribute__((packed)) TelemetrySensor {
  uint32_t timestamp_ms;
  int16_t  p0_bar;      // 0.01 bar
  int16_t  p1_bar;
  int16_t  t1_C;        // 0.01 C
  int16_t  t2_C;
  int16_t  h13_mm;      // 0.1 mm
  int16_t  h57_mm;
  int16_t  h24_mm;
  int16_t  h6R_mm;
  int16_t  hK1_mm;
  int16_t  hK2_mm;
  int16_t  tm13;        // TMAG raw Z
  int16_t  tm57;
  int16_t  tm24;
  int16_t  tm6R;
  int16_t  tk1;         // K1 sensor 1
  int16_t  tk2;         // K1 sensor 2
  int16_t  tk3;         // K2 sensor 1
  int16_t  tk4;         // K2 sensor 2
  uint8_t  calRunning;  // Manuel kalibrasyon aktif mi
  uint8_t  calPhase;    // Kalibrasyon fazı (0=Bekle, 1=Basinc, 2..7=P0..P5, 8=Hesapla, 9=Tamam)
  uint8_t  calProgress; // 0..100 (opsiyonel, 0=phase'den hesaplanir)
};
static_assert(sizeof(TelemetrySensor) == 43, "TelemetrySensor size mismatch");

struct __attribute__((packed)) TelemetryPiston {
  uint32_t timestamp_ms;
  uint16_t holdDuty[6];  // 0-4095
  int16_t  speed[6];     // 0.1 mm/s
  uint8_t  state[6];     // PistonRuntimeState::State
};
static_assert(sizeof(TelemetryPiston) == 34, "TelemetryPiston size mismatch");

struct __attribute__((packed)) TelemetryINA {
  uint32_t timestamp_ms;
  uint16_t v[8];         // 0.1 V
  int16_t  i[8];         // mA
  uint16_t d[8];         // duty
  uint8_t  drv[8];       // 0=OK, 1=Open, 2=Short
};
static_assert(sizeof(TelemetryINA) == 60, "TelemetryINA size mismatch");

struct __attribute__((packed)) TelemetryPower {
  uint32_t timestamp_ms;
  uint16_t mainV;        // 0.1 V
  uint16_t mainI;        // 0.01 A
  uint16_t vescV;        // 0.1 V
  uint16_t vescI;        // 0.01 A
  int16_t  rpm;
  uint8_t  vescTf;       // C
  uint8_t  pumpMode;
  uint16_t pumpRpm;
  uint16_t pumpBar;      // 0.01 bar
};
static_assert(sizeof(TelemetryPower) == 20, "TelemetryPower size mismatch");

struct __attribute__((packed)) TelemetryVersion {
  uint8_t  major;
  uint8_t  minor;
  uint8_t  patch;
  uint16_t freeHeap;     // KB
  uint8_t  ocpLatch;
};
static_assert(sizeof(TelemetryVersion) == 6, "TelemetryVersion size mismatch");

struct FrameHeader {
  uint8_t  type;
  uint16_t length;
  uint8_t  seq;
};

// ============================================================
// Helpers
// ============================================================
uint16_t crc16_ccitt(const uint8_t* data, size_t len);

// Encode a full frame. Returns total bytes written or 0 on failure.
size_t encodeFrame(uint8_t* out, size_t outMax,
                   uint8_t type, uint8_t seq,
                   const uint8_t* payload, uint16_t payloadLen);

// Encode a fixed binary telemetry struct.
template<typename T>
inline size_t encodeTelemetry(uint8_t* out, size_t outMax,
                              uint8_t type, uint8_t seq,
                              const T* data) {
  return encodeFrame(out, outMax, type, seq,
                     reinterpret_cast<const uint8_t*>(data), sizeof(T));
}

// Decode helpers.
// Returns payload offset (5) if a valid frame is present, 0 otherwise.
size_t decodeFrame(const uint8_t* buf, size_t len,
                   FrameHeader& hdr, const uint8_t*& payload);

// MessagePack helpers using ArduinoJson.
// size_t encodeMsgPack(const JsonDocument& doc, uint8_t* out, size_t outMax);
// bool decodeMsgPack(const uint8_t* data, size_t len, JsonDocument& doc);

// Global non-blocking frame transmit helpers (usable from any task).
// These write directly to USB CDC if the TX buffer has room.
void SerialTx_SendEvent(const JsonDocument& doc, bool reliable = false);
void SerialTx_SendLog(MsgCode code, const char* text);

} // namespace kitronic

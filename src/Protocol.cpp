#include "Protocol.h"
#include "Shared.h"
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace kitronic {

static uint8_t s_txSeq = 0;

static uint8_t nextSeq() {
  portENTER_CRITICAL(&g_portMux);
  uint8_t seq = s_txSeq++;
  portEXIT_CRITICAL(&g_portMux);
  return seq;
}

static void sendFrameNonBlock(const uint8_t* frame, size_t len) {
  if (len == 0) return;
  if (Serial.availableForWrite() >= (int)len) {
    Serial.write(frame, len);
  }
}

// TX buffer'i dolu olsa bile kareyi parca parca, kisa bir timeout ile gonderir.
// Kritik event kareleri (AT, test done) icin kullanilir; telemetri gibi yuksek
// frekansli veriler icin sendFrameNonBlock tercih edilir.
static void sendFrameReliable(const uint8_t* frame, size_t len, uint32_t timeoutMs = 200) {
  if (len == 0) return;
  uint32_t start = millis();
  size_t sent = 0;
  while (sent < len) {
    int avail = Serial.availableForWrite();
    if (avail > 0) {
      size_t chunk = len - sent;
      if ((size_t)avail < chunk) chunk = (size_t)avail;
      size_t written = Serial.write(frame + sent, chunk);
      if (written > 0) sent += written;
    }
    if (sent >= len) break;
    if ((uint32_t)(millis() - start) >= timeoutMs) break;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void SerialTx_SendEvent(const JsonDocument& doc, bool reliable) {
  uint8_t payload[256];
  size_t n = serializeMsgPack(doc, payload, sizeof(payload));
  if (n == 0 || n >= sizeof(payload)) return;
  uint8_t frame[280];
  size_t fn = encodeFrame(frame, sizeof(frame), FT_EVENT, nextSeq(), payload, (uint16_t)n);
  if (fn > 0) {
    if (reliable) sendFrameReliable(frame, fn);
    else sendFrameNonBlock(frame, fn);
  }
}

void SerialTx_SendLog(MsgCode code, const char* text) {
  StaticJsonDocument<1024> doc;
  doc["c"] = (uint8_t)code;
  auto p = doc["p"].to<JsonArray>();
  p.add(text);
  uint8_t payload[1024];
  size_t n = serializeMsgPack(doc, payload, sizeof(payload));
  if (n == 0 || n >= sizeof(payload)) return;
  uint8_t frame[1050];
  size_t fn = encodeFrame(frame, sizeof(frame), FT_LOG, nextSeq(), payload, (uint16_t)n);
  if (fn > 0) sendFrameNonBlock(frame, fn);
}

uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

size_t encodeFrame(uint8_t* out, size_t outMax,
                   uint8_t type, uint8_t seq,
                   const uint8_t* payload, uint16_t payloadLen) {
  if (outMax < (size_t)payloadLen + FRAME_OVERHEAD) {
    return 0;
  }
  out[0] = FRAME_START;
  out[1] = type;
  out[2] = payloadLen & 0xFF;
  out[3] = (payloadLen >> 8) & 0xFF;
  out[4] = seq;
  if (payloadLen > 0 && payload != nullptr) {
    memcpy(&out[5], payload, payloadLen);
  }
  uint16_t crc = crc16_ccitt(&out[1], payloadLen + 4);
  size_t pos = 5 + payloadLen;
  out[pos++] = crc & 0xFF;
  out[pos++] = (crc >> 8) & 0xFF;
  out[pos++] = FRAME_END;
  return pos;
}

size_t decodeFrame(const uint8_t* buf, size_t len,
                   FrameHeader& hdr, const uint8_t*& payload) {
  if (len < FRAME_OVERHEAD) {
    return 0;
  }
  if (buf[0] != FRAME_START) {
    return 0;
  }
  hdr.type = buf[1];
  hdr.length = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
  hdr.seq = buf[4];
  if (len < (size_t)hdr.length + FRAME_OVERHEAD) {
    return 0;
  }
  if (buf[hdr.length + FRAME_HEADER_SIZE + 2] != FRAME_END) {
    return 0;
  }
  uint16_t calc = crc16_ccitt(&buf[1], hdr.length + 4);
  uint16_t recv = (uint16_t)buf[hdr.length + FRAME_HEADER_SIZE]
                | ((uint16_t)buf[hdr.length + FRAME_HEADER_SIZE + 1] << 8);
  if (calc != recv) {
    return 0;
  }
  payload = &buf[FRAME_HEADER_SIZE];
  return FRAME_HEADER_SIZE;  // payload offset
}

} // namespace kitronic

#pragma once
#include <Arduino.h>

// ESP32-S3 TWAI (CAN) basit arayüz
bool  TWAI_Init(int rx_gpio, int tx_gpio, long baud = 500000, bool listen_only=false);
bool  TWAI_Send(uint32_t id, const uint8_t* data, uint8_t len, bool extended=false);
bool  TWAI_Read(uint32_t &id, uint8_t* data, uint8_t &len, bool &extended, uint32_t timeout_ms=0);
int   TWAI_Available();
void  TWAI_Deinit();

#pragma once
#include <Arduino.h>

void  INA219_InitAll();
void  INA219_UpdateAll();       // opsiyonel (şu an no-op)
float INA219_GetVoltage(uint8_t index); // Vbus (V)
float INA219_GetCurrent(uint8_t index); // akım (mA)
bool  INA219_IsPresent(uint8_t index);

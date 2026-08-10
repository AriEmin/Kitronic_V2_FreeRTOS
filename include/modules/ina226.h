#pragma once
#include <Arduino.h>

void  INA226_Init();                 // opsiyonel
void  INA226_Update();               // opsiyonel
float INA226_GetVoltage(bool vesc);  // false: MAIN (0x68), true: VESC (0x69)
bool  INA226_IsPresent(bool vesc);

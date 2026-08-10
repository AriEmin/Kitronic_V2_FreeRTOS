#include "ina226.h"
#include <Wire.h>
#include "config/i2c_map.h"

#ifndef INA226_MAIN_PWR_ADDR
  #define INA226_MAIN_PWR_ADDR 0x68
  #define INA226_VESC_PWR_ADDR 0x69
#endif

static bool s_hasMain = false, s_hasVesc = false;

static bool i2c_present(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

static uint16_t i2cReadWord(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr); Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)2);
  uint16_t v = ((uint16_t)Wire.read() << 8) | Wire.read();
  return v;
}

void INA226_Init() {
  s_hasMain = i2c_present(INA226_MAIN_PWR_ADDR);
  s_hasVesc = i2c_present(INA226_VESC_PWR_ADDR);
}

void INA226_Update() {
  // Şimdilik register polling ile anlık okuyoruz; caching yok.
}

float INA226_GetVoltage(bool vesc) {
  uint8_t addr = vesc ? INA226_VESC_PWR_ADDR : INA226_MAIN_PWR_ADDR;
  if (!i2c_present(addr)) return NAN;
  // Bus voltage reg: 0x02, LSB=1.25mV
  uint16_t raw = i2cReadWord(addr, 0x02);
  return (float)raw * 1.25f / 1000.0f;
}

bool INA226_IsPresent(bool vesc) {
  return vesc ? s_hasVesc : s_hasMain;
}

#include "ina219.h"
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "config/i2c_map.h"

#ifndef INA219_N433_ADDR
  #define INA219_N433_ADDR 0x40
  #define INA219_N434_ADDR 0x41
  #define INA219_N435_ADDR 0x42
  #define INA219_N436_ADDR 0x43
  #define INA219_N437_ADDR 0x44
  #define INA219_N438_ADDR 0x45
  #define INA219_N439_ADDR 0x46
  #define INA219_N440_ADDR 0x47
#endif

static const uint8_t kAddr[8] = {
  INA219_N433_ADDR, INA219_N434_ADDR, INA219_N435_ADDR, INA219_N436_ADDR,
  INA219_N437_ADDR, INA219_N438_ADDR, INA219_N439_ADDR, INA219_N440_ADDR
};

// Adresler kurucuda veriliyor (Adafruit kütüphanesi böyle çalışıyor)
static Adafruit_INA219 s_ina[8] = {
  Adafruit_INA219(INA219_N433_ADDR), Adafruit_INA219(INA219_N434_ADDR),
  Adafruit_INA219(INA219_N435_ADDR), Adafruit_INA219(INA219_N436_ADDR),
  Adafruit_INA219(INA219_N437_ADDR), Adafruit_INA219(INA219_N438_ADDR),
  Adafruit_INA219(INA219_N439_ADDR), Adafruit_INA219(INA219_N440_ADDR)
};

static bool s_present[8] = {false,false,false,false,false,false,false,false};

static bool i2c_present(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void INA219_InitAll() {
  for (int i=0;i<8;i++) {
    if (!i2c_present(kAddr[i])) { s_present[i] = false; continue; }
    // begin() sadece TwoWire* alır; adres zaten kurucuda verildi
    s_present[i] = s_ina[i].begin(&Wire);
    if (s_present[i]) {
      // Gerekirse buraya farklı kalibrasyon profilini koyarız
      s_ina[i].setCalibration_32V_2A();
    }
  }
}

void INA219_UpdateAll() {
  // Adafruit sürücüsü okuma anında ölçüyor; burada cache yok.
}

float INA219_GetVoltage(uint8_t i) {
  if (i>=8 || !s_present[i]) return NAN;
  return s_ina[i].getBusVoltage_V();
}

float INA219_GetCurrent(uint8_t i) {
  if (i>=8 || !s_present[i]) return NAN;
  return s_ina[i].getCurrent_mA();
}

bool INA219_IsPresent(uint8_t i) {
  return (i<8) ? s_present[i] : false;
}

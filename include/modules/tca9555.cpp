#include "tca9555.h"

// TCA9555 Registerleri
static constexpr uint8_t REG_INPUT   = 0x00; // 0x00: P0, 0x01: P1
static constexpr uint8_t REG_OUTPUT  = 0x02; // 0x02: P0, 0x03: P1
static constexpr uint8_t REG_POL     = 0x04; // 0x04: P0, 0x05: P1
static constexpr uint8_t REG_CONFIG  = 0x06; // 0x06: P0, 0x07: P1

static uint16_t s_outReg[2] = {0xFFFF, 0xFFFF};  // default HIGH
static uint16_t s_cfgReg[2] = {0xFFFF, 0xFFFF};  // default input

static inline uint8_t addrOf(uint8_t grp) { return (grp==0) ? TCA9555_SELO1_ADDR : TCA9555_SELO2_ADDR; }

static void write16(uint8_t addr, uint8_t reg, uint16_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val & 0xFF);         // P0 (low byte)
  Wire.write((val >> 8) & 0xFF);  // P1 (high byte)
  Wire.endTransmission();
}

static uint16_t read16(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr); Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)2);
  uint16_t v0 = Wire.read();
  uint16_t v1 = Wire.read();
  return v0 | (v1 << 8);
}

void TCA9555_InitAll() {
  // P00..P05 OUTPUT, diğerleri INPUT -> 0b0000 0011 1111 0000 = 0xFC00
  s_cfgReg[0] = 0x03F0;
  s_cfgReg[1] = 0x03F0;
  write16(TCA9555_SELO1_ADDR, REG_CONFIG, s_cfgReg[0]);
  write16(TCA9555_SELO2_ADDR, REG_CONFIG, s_cfgReg[1]);

  // Çıkışlar default HIGH (CS/DRVOFF pasif)
  s_outReg[0] = 0x0000;
  s_outReg[1] = 0x0000;
  write16(TCA9555_SELO1_ADDR, REG_OUTPUT, s_outReg[0]);
  write16(TCA9555_SELO2_ADDR, REG_OUTPUT, s_outReg[1]);

  // Polarite default
  write16(TCA9555_SELO1_ADDR, REG_POL, 0x0000);
  write16(TCA9555_SELO2_ADDR, REG_POL, 0x0000);
}

void TCA9555_SetOutput(uint8_t grp, uint8_t bit, uint8_t level) {
  uint8_t a = addrOf(grp);
  uint16_t cur = s_outReg[grp];
  if (level) cur |= (1u << bit);
  else       cur &= ~(1u << bit);
  s_outReg[grp] = cur;
  write16(a, REG_OUTPUT, cur);
}

bool TCA9555_ReadButton(uint8_t grp, uint8_t bit) {
  uint8_t a = addrOf(grp);
  uint16_t in = read16(a, REG_INPUT);
  return (in >> bit) & 0x1;
}

uint16_t TCA9555_ReadInputsRaw(uint8_t grp) {
  return read16(addrOf(grp), REG_INPUT);
}

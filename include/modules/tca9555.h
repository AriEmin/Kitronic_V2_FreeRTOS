#pragma once
#include <Arduino.h>
#include <Wire.h>

// Adresler (pins/i2c_map varsa onlar baskın gelir)
#ifndef TCA9555_SELO1_ADDR
  #define TCA9555_SELO1_ADDR 0x20
#endif
#ifndef TCA9555_SELO2_ADDR
  #define TCA9555_SELO2_ADDR 0x21
#endif

// Port bit indeksleri (SELO1: DRV1/DRV2, SELO2: DRV3/DRV4)
#ifndef DRV1_NSCS
  #define DRV1_NSCS   0
  #define DRV2_NSCS   1
  #define DRV1_DRVOFF 2
  #define DRV2_DRVOFF 3
  #define DRV1_NFAULT 4
  #define DRV2_NFAULT 5

  #define DRV3_NSCS   0
  #define DRV4_NSCS   1
  #define DRV3_DRVOFF 2
  #define DRV4_DRVOFF 3
  #define DRV3_NFAULT 4
  #define DRV4_NFAULT 5
#endif

void TCA9555_InitAll();                       // Port yönleri/config ayarla
void TCA9555_SetOutput(uint8_t grp, uint8_t bit, uint8_t level); // grp:0->0x20, 1->0x21
bool TCA9555_ReadButton(uint8_t grp, uint8_t bit);                // Input (buton / nFAULT)
uint16_t TCA9555_ReadInputsRaw(uint8_t grp);

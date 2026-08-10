#pragma once
#include <Arduino.h>

// Güvenli bring-up fonksiyonları (register yazmaz, sadece hat durumları ve pin sürüş)
// DRV8243’lerin CS/DRVOFF hatları TCA9555 üzerinden sürülür.
void DRV8243_InitAll();     // SPI.begin + CS high, DRVOFF high (enable) yapar
void DRV8243_CheckFaults(); // nFAULT pinlerini TCA9555 üzerinden okur ve Serial’e yazar

#include "drv8243.h"
#include <SPI.h>
#include "tca9555.h"
#include "config/pins.h"

// SPI pinleri pins.h’da tanımlıysa onları kullanır; yoksa Arduino default
#ifndef SPI_SCK
  #define SPI_SCK  SCK
#endif
#ifndef SPI_MISO
  #define SPI_MISO MISO
#endif
#ifndef SPI_MOSI
  #define SPI_MOSI MOSI
#endif

static bool s_spiBegun = false;

void DRV8243_InitAll() {
  if (!s_spiBegun) {
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    s_spiBegun = true;
  }

  // TCA9555 tarafı hazır değilse init et
  TCA9555_InitAll();

  // Tüm CS’ler HIGH (pasif), DRVOFF’ler HIGH (enable)
  TCA9555_SetOutput(0, DRV1_NSCS,   1);
  TCA9555_SetOutput(0, DRV2_NSCS,   1);
  TCA9555_SetOutput(1, DRV3_NSCS,   1);
  TCA9555_SetOutput(1, DRV4_NSCS,   1);

  TCA9555_SetOutput(0, DRV1_DRVOFF, 1);
  TCA9555_SetOutput(0, DRV2_DRVOFF, 1);
  TCA9555_SetOutput(1, DRV3_DRVOFF, 1);
  TCA9555_SetOutput(1, DRV4_DRVOFF, 1);

  
}

void DRV8243_CheckFaults() {
  bool f1 = TCA9555_ReadButton(0, DRV1_NFAULT);
  bool f2 = TCA9555_ReadButton(0, DRV2_NFAULT);
  bool f3 = TCA9555_ReadButton(1, DRV3_NFAULT);
  bool f4 = TCA9555_ReadButton(1, DRV4_NFAULT);
  Serial.printf("DRV8243 nFAULT: D1=%d D2=%d D3=%d D4=%d\n", f1, f2, f3, f4);
}

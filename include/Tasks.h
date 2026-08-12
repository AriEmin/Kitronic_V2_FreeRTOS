#pragma once

#include <Arduino.h>

#include <freertos/FreeRTOS.h>

#include <freertos/task.h>

#include <Shared.h>

#include "Protocol.h"


// ===============================

// KITRONIC ONEBOARD PIN MAP

// ===============================



//Hall N434 -   IO12

//Hall N433 -   IO2

//Hall N437 -   IO46

//Hall N438 -   I10



#define HALL_N434_PIN         12

#define HALL_N433_PIN         2

#define HALL_N437_PIN         18

#define HALL_N438_PIN         10



// --- Rezerve ---

#define LED_RUN              4



// --- DRV8243 Solenoid Outputs ---

#define N433_PWM_OUT           5   // U16 IN1

#define N436_PWM_OUT           6   // U16 IN2

#define N434_PWM_OUT           7   // U15 IN1

#define N435_PWM_OUT           15  // U15 IN2

#define N438_PWM_OUT           39  // U4 IN1

#define N440_PWM_OUT           40  // U4 IN2

#define N439_PWM_OUT           41  // U17 IN1

#define N437_PWM_OUT           42  // U17 IN2



// --- Valf Temizleme (MOSFET Sürücü) ---

// Not: EFUSE (TPS24770-Q1) artık kullanılmıyor, pinler valf temizleme için ayrıldı

#define VALVE_CLEAN_1          16  // IO16 - Valf temizleme çıkış 1

#define VALVE_CLEAN_2          17  // IO17 - Valf temizleme çıkış 2



// --- Pompa / Test / Role Kontrolleri ---

#define BLDC_SELECT            3   // BLDC role kontrol (seçici)

#define SSR_CONTROL            11  // Solid State Role (ısıtıcı PID kontrolü)



// --- Analog Girişler ---

#define PRESSURE_PIN           1   // Basınç ADC girişi



// --- I²C ---

#define I2C_SDA                8

#define I2C_SCL                9



// --- USB OTG ---

#define USB_DN                 19  // D-

#define USB_DP                 20  // D+



// --- UART2 / ESC ---

#define ESC_UART_RX            46  // ESC_DATA (UART2 RX)



// --- CAN Bus (SN65HVD230DR) ---

#define CAN_TX                 13  // Pin 1

#define CAN_RX                 14  // Pin 4

#define CAN_RS                 38  // Pin 8 (mode select)



// --- SPI Bus ---

#define SPI_MOSI               35

#define SPI_MISO               37

#define SPI_SCK                36



// --- IO Expanders (TCA9555) ---

#define SELO_2_IO_INT          21  // Selo 2 Interrupt

#define SELO_1_IO_INT          47  // Selo 1 Interrupt



// ===============================

// KITRONIC ONEBOARD CAN MAP

// ===============================



#define CAN_BAUDRATE 500E3



// Pinler (SN65HVD230 bağlantısı)

#define CAN_TX 13

#define CAN_RX 14

#define CAN_RS 38



// VESC parametreleri

#define VESC_ID 10

#define CAN_PACKET_SET_CURRENT 0

#define CAN_PACKET_SET_DUTY 0

#define CAN_PACKET_SET_RPM 3   // VESC CAN: 0=duty,1=current,2=brake-current,3=rpm



// ===============================

// KITRONIC ONEBOARD I2C MAP

// ===============================



// ========== SICAKLIK SENSÖRÜ PİNLERİ (MCU ADC) ==========

#define TEMP_SENSOR_1_PIN  10  // IO10: Sıcaklık sensörü 1 (ileride kullanılacak)

#define TEMP_SENSOR_2_PIN  2   // IO2:  Sıcaklık sensörü 2 (aktif, GUI'ye gönderilir)

// =============================================





// ========== TCA9555 - IO EXPANDERS ==========

#define TCA0  0x20

#define TCA1  0x21



#define TCA_REG_INPUT0   0x00

#define TCA_REG_INPUT1   0x01

#define TCA_REG_OUTPUT0  0x02

#define TCA_REG_OUTPUT1  0x03

#define TCA_REG_CONFIG0  0x06

#define TCA_REG_CONFIG1  0x07



// --- TCA9555 #1 (0x20) --- U16/U15 (N433–N436) ---

#define DRV1_NSCS        0   // P00

#define DRV2_NSCS        1   // P01

#define DRV1_DRVOFF      2   // P02

#define DRV2_DRVOFF      3   // P03

#define DRV1_NFAULT      4   // P04

#define DRV2_NFAULT      5   // P05

#define BTN_N433         6   // P06

#define BTN_N434         7   // P07

#define BTN_N435         8   // P10

#define BTN_N436         9   // P11



// --- TCA9555 #2 (0x21) --- U17/U4 (N437–N440) ---

#define DRV3_NSCS        0   // P00

#define DRV4_NSCS        1   // P01

#define DRV3_DRVOFF      2   // P02

#define DRV4_DRVOFF      3   // P03

#define DRV3_NFAULT      4   // P04

#define DRV4_NFAULT      5   // P05

#define BTN_N437         6   // P06

#define BTN_N438         7   // P07

#define BTN_N439         8   // P10

#define BTN_N440         9   // P11



// =============================================





// ========== INA219 - VALF AKIM ÖLÇÜMLERİ ==========

#define INA219_N433_ADDR  0x40

#define INA219_N434_ADDR  0x41

#define INA219_N435_ADDR  0x42

#define INA219_N436_ADDR  0x43

#define INA219_N437_ADDR  0x44

#define INA219_N438_ADDR  0x45

#define INA219_N439_ADDR  0x46

#define INA219_N440_ADDR  0x47



// =============================================





// ========== INA226 - GÜÇ HATLARI ==========

#define INA226_MAIN_PWR_ADDR  0x4C  // Ana giriş gücü

#define INA226_VESC_PWR_ADDR  0x4D   // VESC hattı gücü



// =============================================





// ========== TCA9548A - I2C MUX (TMAG5173 için) ==========

#define TCA9548A_ADDR         0x70



// =============================================





// I2C pinleri (config/pins.h içinden kullanılır)

#ifndef I2C_SDA

#define I2C_SDA 8

#define I2C_SCL 9

#endif







// --- Kuyruklar (eski) ---
// Seri iletişim artık doğrudan USB CDC üzerinden binary frame olarak yapılıyor.



// --- Task prototipleri ---

void TaskSerial(void *pvParameters);    // Seri haberleşme (PC ile)

void TaskBlink(void *pvParameters);     // LED task

void TaskAnalog(void *pvParameters);    // ADC okuma task

void TaskStatus(void *pvParameters);    // Sistem durumu üretip seri kuyruğa atar

void TaskWorker(void *pvParameters);    // Örnek iş yapan task

void OilFill(void *pvParameters);    // Yağ doldurma iş paketi

void TaskI2CMonitor(void *pvParameters);

void TaskADSMonitor(void *pvParameters);

void TaskValveControl(void *pvParameters);

void TaskBLDCPump(void *pvParameters);

void TaskOilCheck(void *pvParameters);

void TaskDiag(void *pvParameters);

void TaskTMAG5173(void *pvParameters);  // TMAG5173 manyetik sensör okuma

void TaskValveClean(void *pvParameters); // Valf temizleme PWM kontrolü


void TaskValveDiag(void *pvParameters);      // Valf diagnostik testi (PWM rampa)

void TaskHallTest(void *pvParameters);       // Hall sensör stabilite testi

void TaskCurrentCalib(void *pvParameters);   // holdcontrol_V2 akım kalibrasyon task'ı

void TaskAutoTest(void *pvParameters);       // 9-Fazlı Otomatik Test

void TaskOLED(void *pvParameters);           // I2C OLED durum ekranı



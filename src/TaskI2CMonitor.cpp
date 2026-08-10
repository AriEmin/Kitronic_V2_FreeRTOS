#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "Tasks.h"
#include "Shared.h"

// 8 tane INA219 için obje oluşturalım
// Sırayı senin adres sıralamanla tutuyorum:
static Adafruit_INA219 ina_n433(INA219_N433_ADDR); // 0x40
static Adafruit_INA219 ina_n434(INA219_N434_ADDR); // 0x41
static Adafruit_INA219 ina_n435(INA219_N435_ADDR); // 0x42
static Adafruit_INA219 ina_n436(INA219_N436_ADDR); // 0x43
static Adafruit_INA219 ina_n437(INA219_N437_ADDR); // 0x44
static Adafruit_INA219 ina_n438(INA219_N438_ADDR); // 0x45
static Adafruit_INA219 ina_n439(INA219_N439_ADDR); // 0x46
static Adafruit_INA219 ina_n440(INA219_N440_ADDR); // 0x47

// ========== INA226 Yardımcıları ==========
#define INA226_REG_CONFIG     0x00
#define INA226_REG_SHUNT_V    0x01
#define INA226_REG_BUS_V      0x02
#define INA226_REG_CURRENT    0x04
#define INA226_REG_CALIB      0x05

// INA226 register okuma
static uint16_t ina226_read16(uint8_t i2c_addr, uint8_t reg) {
    Wire.beginTransmission(i2c_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(i2c_addr, (uint8_t)2);
    if (Wire.available() < 2) return 0;
    uint16_t v = (Wire.read() << 8) | Wire.read();
    return v;
}

// INA226 register yazma
static void ina226_write16(uint8_t i2c_addr, uint8_t reg, uint16_t value) {
    Wire.beginTransmission(i2c_addr);
    Wire.write(reg);
    Wire.write((uint8_t)(value >> 8));
    Wire.write((uint8_t)(value & 0xFF));
    Wire.endTransmission();
}

// Basit konfigürasyon (ortalama + 1.1ms conv, shunt/bus enable)
// Cihaz varsa true döner
static bool ina226_init(uint8_t addr) {
    // Önce cihaz var mı kontrol et
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) {
        return false;  // Cihaz yok
    }
    
    // config: 0x4527 -> örnek: AVG=4, VBUS=1.1ms, VSH=1.1ms, mode=shunt+bus, cont
    ina226_write16(addr, INA226_REG_CONFIG, 0x4527);

    // Kalibrasyon: bu değeri sen sahada şunta göre ayarlarsın.
    // Şimdilik "1mA/bit" gibi düşünelim.
    // current = raw * current_LSB
    ina226_write16(addr, INA226_REG_CALIB, 1024);
    return true;
}

void TaskI2CMonitor(void *pvParameters) {
    (void) pvParameters;

    // Global I2C mutex oluştur (ilk task olarak)
    if (!g_i2cMutex) {
        g_i2cMutex = xSemaphoreCreateMutex();
    }

    // I2C başlat
    Wire.begin(I2C_SDA, I2C_SCL, 400000);   // 400kHz (1MHz yerine daha stabil)
    
    // INA219'ları başlat
    bool ok_40 = ina_n433.begin();
    bool ok_41 = ina_n434.begin();
    bool ok_42 = ina_n435.begin();
    bool ok_43 = ina_n436.begin();
    bool ok_44 = ina_n437.begin();
    bool ok_45 = ina_n438.begin();
    bool ok_46 = ina_n439.begin();
    bool ok_47 = ina_n440.begin();

    ina_n433.setCalibration_32V_2A();
    ina_n434.setCalibration_32V_2A();
    ina_n435.setCalibration_32V_2A();
    ina_n436.setCalibration_32V_2A();
    ina_n437.setCalibration_32V_2A();
    ina_n438.setCalibration_32V_2A();
    ina_n439.setCalibration_32V_2A();
    ina_n440.setCalibration_32V_2A();

    // INA226'ları başlat (cihaz yoksa false döner)
    bool ok_main = ina226_init(INA226_MAIN_PWR_ADDR);
    bool ok_vesc = ina226_init(INA226_VESC_PWR_ADDR);

    {
        char msg[128];
        snprintf(msg, sizeof(msg), "[I2C] INA219: %d %d %d %d %d %d %d %d",
                 ok_40, ok_41, ok_42, ok_43, ok_44, ok_45, ok_46, ok_47);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
        snprintf(msg, sizeof(msg), "[I2C] INA226: MAIN=%d VESC=%d", ok_main, ok_vesc);
        kitronic::SerialTx_SendLog(kitronic::MsgCode::UNKNOWN_COMMAND, msg);
    }

    for (;;) {
        // I2C mutex al
        if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            // ---- 1) INA219'lar ----
            // Okuyup Shared'e yazıyoruz
            float current_mA[8];
            float bus_V[8];
            
            // EMA filtreleme icin static buffer (alfa=0.3)
            static float s_filteredCurrent[8] = {0};
            /*static constexpr float EMA_ALPHA = 0.30f;*/
            
            //GPT önerisi-EMA için daha düşük alfa, daha fazla stabilite (ancak tepki süresi uzar)
            static constexpr float EMA_ALPHA = 0.15f;
            float raw_mA[8];
            
            //chatgpt önerisi boş değerler nan olsun, böylece filtrede sıfır etkisi olmaz
            if (ok_40) {
                raw_mA[0] = ina_n433.getCurrent_mA();
                bus_V[0]  = ina_n433.getBusVoltage_V();
            } else {
                raw_mA[0] = NAN;   // Geçersiz veri işareti
            }
            if (ok_41) {
                raw_mA[1] = ina_n434.getCurrent_mA();
                bus_V[1]  = ina_n434.getBusVoltage_V();
            } else {
                raw_mA[1] = NAN;
            }
            if (ok_42) {
                raw_mA[2] = ina_n435.getCurrent_mA();
                bus_V[2]  = ina_n435.getBusVoltage_V();
            } else {
                raw_mA[2] = NAN;
            }
            if (ok_43) {
                raw_mA[3] = ina_n436.getCurrent_mA();
                bus_V[3]  = ina_n436.getBusVoltage_V();
            } else {
                raw_mA[3] = NAN;
            }
            if (ok_44) {
                raw_mA[4] = ina_n437.getCurrent_mA();
                bus_V[4]  = ina_n437.getBusVoltage_V();
            } else {
                raw_mA[4] = NAN;
            }
            if (ok_45) {
                raw_mA[5] = ina_n438.getCurrent_mA();
                bus_V[5]  = ina_n438.getBusVoltage_V();
            } else {
                raw_mA[5] = NAN;
            }
            if (ok_46) {
                raw_mA[6] = ina_n439.getCurrent_mA();
                bus_V[6]  = ina_n439.getBusVoltage_V();
            } else {
                raw_mA[6] = NAN;
            }
            if (ok_47) {
                raw_mA[7] = ina_n440.getCurrent_mA();
                bus_V[7]  = ina_n440.getBusVoltage_V();
            } else {
                raw_mA[7] = NAN;
            }


            // Raw akim okuma
            /*
            raw_mA[0] = ok_40 ? ina_n433.getCurrent_mA() : 0.0f;
            bus_V[0]  = ok_40 ? ina_n433.getBusVoltage_V() : 0.0f;

            raw_mA[1] = ok_41 ? ina_n434.getCurrent_mA() : 0.0f;
            bus_V[1]      = ok_41 ? ina_n434.getBusVoltage_V() : 0.0f;

            raw_mA[2] = ok_42 ? ina_n435.getCurrent_mA() : 0.0f;
            bus_V[2]      = ok_42 ? ina_n435.getBusVoltage_V() : 0.0f;

            raw_mA[3] = ok_43 ? ina_n436.getCurrent_mA() : 0.0f;
            bus_V[3]      = ok_43 ? ina_n436.getBusVoltage_V() : 0.0f;

            raw_mA[4] = ok_44 ? ina_n437.getCurrent_mA() : 0.0f;
            bus_V[4]      = ok_44 ? ina_n437.getBusVoltage_V() : 0.0f;

            raw_mA[5] = ok_45 ? ina_n438.getCurrent_mA() : 0.0f;
            bus_V[5]      = ok_45 ? ina_n438.getBusVoltage_V() : 0.0f;

            raw_mA[6] = ok_46 ? ina_n439.getCurrent_mA() : 0.0f;
            bus_V[6]      = ok_46 ? ina_n439.getBusVoltage_V() : 0.0f;

            raw_mA[7] = ok_47 ? ina_n440.getCurrent_mA() : 0.0f;
            bus_V[7]      = ok_47 ? ina_n440.getBusVoltage_V() : 0.0f;*/

            //ChatGPT önerisi-EMA için ilk değer ataması yapalım, böylece ilk okuma sonrası sıfır etkisi olmaz
            static bool s_currentFilterInit[8] = {false};

            for (int i = 0; i < 8; i++) {
                if (!isnan(raw_mA[i])) {
                    if (!s_currentFilterInit[i]) {
                        s_filteredCurrent[i] = raw_mA[i];
                        s_currentFilterInit[i] = true;
                    } else {
                        s_filteredCurrent[i] =
                            EMA_ALPHA * raw_mA[i] +
                            (1.0f - EMA_ALPHA) * s_filteredCurrent[i];
                    }
                }

                current_mA[i] = s_filteredCurrent[i];
            }

            // ---- EMA Filtreleme uygula ----
            /*for (int i = 0; i < 8; i++) {
                s_filteredCurrent[i] = EMA_ALPHA * raw_mA[i] + (1.0f - EMA_ALPHA) * s_filteredCurrent[i];
                current_mA[i] = s_filteredCurrent[i];
            }*/

            // ---- 2) INA226'lar (sadece varsa oku) ----
            float main_bus_V = 0.0f, main_cur_A = 0.0f;
            float vesc_bus_V = 0.0f, vesc_cur_A = 0.0f;
            
            if (ok_main) {
                uint16_t main_bus_raw = ina226_read16(INA226_MAIN_PWR_ADDR, INA226_REG_BUS_V);
                uint16_t main_cur_raw = ina226_read16(INA226_MAIN_PWR_ADDR, INA226_REG_CURRENT);
                main_bus_V = main_bus_raw * 0.00125f; // 1.25mV
                main_cur_A = ((int16_t)main_cur_raw) * 0.01f;
            }
            
            if (ok_vesc) {
                uint16_t vesc_bus_raw = ina226_read16(INA226_VESC_PWR_ADDR, INA226_REG_BUS_V);
                uint16_t vesc_cur_raw = ina226_read16(INA226_VESC_PWR_ADDR, INA226_REG_CURRENT);
                vesc_bus_V = vesc_bus_raw * 0.00125f;
                vesc_cur_A = ((int16_t)vesc_cur_raw) * 0.01f;
            }

            // I2C mutex bırak
            xSemaphoreGive(g_i2cMutex);

            // ---- 3) Shared'e güvenli yaz ----
            if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                for (int i = 0; i < 8; i++) {
                    g_tele.inaI_mA[i]   = current_mA[i];
                    g_tele.inaV[i] = bus_V[i];
                }

                g_tele.mainV= main_bus_V;
                g_tele.mainI = main_cur_A;
                g_tele.vescV = vesc_bus_V;
                g_tele.vescI = vesc_cur_A;

                xSemaphoreGive(g_sharedMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // 0.1 sn'de bir oku
    }
}

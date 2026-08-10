// TMAG5173-Q1 3D Linear Hall-Effect Sensor Driver
// Texas Instruments - I2C Interface
#pragma once
#include <Arduino.h>
#include <Wire.h>

// Default I2C address (can be changed via DEVICE_CONFIG)
#define TMAG5173_DEFAULT_ADDR  0x35

// Register addresses
#define TMAG5173_REG_DEVICE_CONFIG_1   0x00
#define TMAG5173_REG_DEVICE_CONFIG_2   0x01
#define TMAG5173_REG_SENSOR_CONFIG_1   0x02
#define TMAG5173_REG_SENSOR_CONFIG_2   0x03
#define TMAG5173_REG_X_THR_CONFIG      0x04
#define TMAG5173_REG_Y_THR_CONFIG      0x05
#define TMAG5173_REG_Z_THR_CONFIG      0x06
#define TMAG5173_REG_T_CONFIG          0x07
#define TMAG5173_REG_INT_CONFIG_1      0x08
#define TMAG5173_REG_MAG_GAIN_CONFIG   0x09
#define TMAG5173_REG_MAG_OFFSET_CONFIG_1 0x0A
#define TMAG5173_REG_MAG_OFFSET_CONFIG_2 0x0B
#define TMAG5173_REG_I2C_ADDRESS       0x0C
#define TMAG5173_REG_DEVICE_ID         0x0D
#define TMAG5173_REG_MANUFACTURER_ID_LSB 0x0E
#define TMAG5173_REG_MANUFACTURER_ID_MSB 0x0F
#define TMAG5173_REG_T_MSB_RESULT      0x10
#define TMAG5173_REG_T_LSB_RESULT      0x11
#define TMAG5173_REG_X_MSB_RESULT      0x12
#define TMAG5173_REG_X_LSB_RESULT      0x13
#define TMAG5173_REG_Y_MSB_RESULT      0x14
#define TMAG5173_REG_Y_LSB_RESULT      0x15
#define TMAG5173_REG_Z_MSB_RESULT      0x16
#define TMAG5173_REG_Z_LSB_RESULT      0x17
#define TMAG5173_REG_CONV_STATUS       0x18
#define TMAG5173_REG_ANGLE_RESULT_MSB  0x19
#define TMAG5173_REG_ANGLE_RESULT_LSB  0x1A
#define TMAG5173_REG_MAGNITUDE_RESULT  0x1B
#define TMAG5173_REG_DEVICE_STATUS     0x1C

// Device ID expected value
#define TMAG5173_DEVICE_ID_VALUE       0x49

// Magnetic range options (SENSOR_CONFIG_2)
enum TMAG5173_Range {
    TMAG5173_RANGE_40mT  = 0x00,  // ±40 mT (most sensitive)
    TMAG5173_RANGE_80mT  = 0x01,  // ±80 mT
    TMAG5173_RANGE_133mT = 0x02,  // ±133 mT
    TMAG5173_RANGE_266mT = 0x03   // ±266 mT (least sensitive)
};

// Operating mode (DEVICE_CONFIG_2)
enum TMAG5173_OpMode {
    TMAG5173_OP_STANDBY     = 0x00,
    TMAG5173_OP_SLEEP       = 0x01,
    TMAG5173_OP_CONTINUOUS  = 0x02,
    TMAG5173_OP_TRIGGER     = 0x03
};

// Magnetic channel configuration (SENSOR_CONFIG_1)
enum TMAG5173_MagChannel {
    TMAG5173_MAG_NONE      = 0x00,
    TMAG5173_MAG_X         = 0x01,
    TMAG5173_MAG_Y         = 0x02,
    TMAG5173_MAG_Z         = 0x03,
    TMAG5173_MAG_XY        = 0x04,
    TMAG5173_MAG_XZ        = 0x05,
    TMAG5173_MAG_YZ        = 0x06,
    TMAG5173_MAG_XYZ       = 0x07,
    TMAG5173_MAG_XYXZ_YZYX = 0x08,
    TMAG5173_MAG_XY_XZ_YZ  = 0x09
};

struct TMAG5173_Data {
    int16_t x_raw;   // Raw X-axis value
    int16_t y_raw;   // Raw Y-axis value
    int16_t z_raw;   // Raw Z-axis value
    int16_t t_raw;   // Raw temperature value
    float   x_mT;    // X-axis in mT
    float   y_mT;    // Y-axis in mT
    float   z_mT;    // Z-axis in mT
    float   temp_C;  // Temperature in Celsius
    bool    valid;   // Data validity flag
};

class TMAG5173 {
public:
    TMAG5173(uint8_t addr = TMAG5173_DEFAULT_ADDR) 
        : _addr(addr), _range(TMAG5173_RANGE_40mT), _initialized(false) {}

    bool begin() {
        // Önce cihaz var mı kontrol et
        Wire.beginTransmission(_addr);
        if (Wire.endTransmission() != 0) {
            _initialized = false;
            return false;  // Cihaz yok
        }
        
        // Device ID kontrolü YAPMA - bazı varyantlarda farklı ID var
        // Direkt konfigüre et (çalışan koddan alındı)
        
        // DEVICE_CONFIG_1: 32x averaging
        writeReg(TMAG5173_REG_DEVICE_CONFIG_1, 0x40);
        
        // SENSOR_CONFIG_1: X,Y,Z aktif
        writeReg(TMAG5173_REG_SENSOR_CONFIG_1, 0x74);
        
        // T_CONFIG: Sıcaklık ölçümü aktif
        writeReg(TMAG5173_REG_T_CONFIG, 0x01);
        
        // DEVICE_CONFIG_2: low-noise + continuous mode
        writeReg(TMAG5173_REG_DEVICE_CONFIG_2, 0x12);
        
        _initialized = true;
        return true;
    }

    void setRange(TMAG5173_Range range) {
        _range = range;
        uint8_t cfg = readReg(TMAG5173_REG_SENSOR_CONFIG_2);
        cfg = (cfg & 0xFC) | (range & 0x03);
        writeReg(TMAG5173_REG_SENSOR_CONFIG_2, cfg);
    }

    // Read all magnetic axes and temperature
    TMAG5173_Data read() {
        TMAG5173_Data data = {0};
        
        if (!_initialized) {
            data.valid = false;
            return data;
        }

        // Read X, Y, Z results (6 bytes starting from X_MSB)
        uint8_t buf[6];
        if (!readRegs(TMAG5173_REG_X_MSB_RESULT, buf, 6)) {
            data.valid = false;
            return data;
        }

        data.x_raw = (int16_t)((buf[0] << 8) | buf[1]);
        data.y_raw = (int16_t)((buf[2] << 8) | buf[3]);
        data.z_raw = (int16_t)((buf[4] << 8) | buf[5]);

        // Convert to mT based on range
        float scale = getRangeScale();
        data.x_mT = data.x_raw * scale / 32768.0f;
        data.y_mT = data.y_raw * scale / 32768.0f;
        data.z_mT = data.z_raw * scale / 32768.0f;

        // Read temperature
        uint8_t tbuf[2];
        if (readRegs(TMAG5173_REG_T_MSB_RESULT, tbuf, 2)) {
            data.t_raw = (int16_t)((tbuf[0] << 8) | tbuf[1]);
            // Temperature formula from datasheet: T = TADCRES * (1/60.1) + 25
            data.temp_C = (float)data.t_raw / 60.1f + 25.0f;
        }

        data.valid = true;
        return data;
    }

    // Quick read - only Z axis (for gear pistons)
    int16_t readZ_raw() {
        uint8_t buf[2];
        if (!readRegs(TMAG5173_REG_Z_MSB_RESULT, buf, 2)) {
            return 0;
        }
        return (int16_t)((buf[0] << 8) | buf[1]);
    }

    // Read X, Y, Z raw values efficiently
    bool readXYZ_raw(int16_t &x, int16_t &y, int16_t &z) {
        uint8_t buf[6];
        if (!readRegs(TMAG5173_REG_X_MSB_RESULT, buf, 6)) {
            return false;
        }
        x = (int16_t)((buf[0] << 8) | buf[1]);
        y = (int16_t)((buf[2] << 8) | buf[3]);
        z = (int16_t)((buf[4] << 8) | buf[5]);
        return true;
    }

    bool isInitialized() const { return _initialized; }
    uint8_t getAddress() const { return _addr; }

private:
    uint8_t _addr;
    TMAG5173_Range _range;
    bool _initialized;

    float getRangeScale() const {
        switch (_range) {
            case TMAG5173_RANGE_40mT:  return 40.0f;
            case TMAG5173_RANGE_80mT:  return 80.0f;
            case TMAG5173_RANGE_133mT: return 133.0f;
            case TMAG5173_RANGE_266mT: return 266.0f;
            default: return 40.0f;
        }
    }

    uint8_t readReg(uint8_t reg) {
        Wire.beginTransmission(_addr);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) return 0;
        Wire.requestFrom(_addr, (uint8_t)1);
        if (Wire.available()) return Wire.read();
        return 0;
    }

    bool readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
        Wire.beginTransmission(_addr);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) return false;
        Wire.requestFrom(_addr, len);
        if (Wire.available() < len) return false;
        for (uint8_t i = 0; i < len; i++) {
            buf[i] = Wire.read();
        }
        return true;
    }

    void writeReg(uint8_t reg, uint8_t value) {
        Wire.beginTransmission(_addr);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }
};

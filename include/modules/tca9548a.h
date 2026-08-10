// TCA9548A I2C Multiplexer Driver
// 8-channel I2C switch for TMAG5173-Q1 sensors
#pragma once
#include <Arduino.h>
#include <Wire.h>

#define TCA9548A_ADDR  0x70

class TCA9548A {
public:
    TCA9548A(uint8_t addr = TCA9548A_ADDR) : _addr(addr), _currentChannel(0xFF) {}

    bool begin() {
        Wire.beginTransmission(_addr);
        return (Wire.endTransmission() == 0);
    }

    // Select a single channel (0-7), or 0xFF to disable all
    bool selectChannel(uint8_t channel) {
        if (channel == _currentChannel) return true;  // Already selected
        
        uint8_t mask = (channel < 8) ? (1 << channel) : 0;
        Wire.beginTransmission(_addr);
        Wire.write(mask);
        bool ok = (Wire.endTransmission() == 0);
        if (ok) _currentChannel = channel;
        return ok;
    }

    // Disable all channels
    bool disableAll() {
        return selectChannel(0xFF);
    }

    uint8_t currentChannel() const { return _currentChannel; }

private:
    uint8_t _addr;
    uint8_t _currentChannel;
};

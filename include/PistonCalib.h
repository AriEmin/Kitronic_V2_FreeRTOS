#pragma once
#include <Arduino.h>
#include "Shared.h"

// Kalibrasyon verisi saklama/yükleme yardımcıları
static constexpr uint16_t PISTON_CALIB_VERSION = 2;  // duty_hold eklendi

void PistonCalibStorage_Begin();
bool PistonCalibStorage_Load(uint8_t piston, PistonCalibData &out);
bool PistonCalibStorage_Save(uint8_t piston, const PistonCalibData &data);
void PistonCalibStorage_Clear(uint8_t piston);
void PistonCalibStorage_ClearAll();
void PistonCalibStorage_LoadAll(PistonCalibData (&out)[PISTON_CHANNEL_COUNT]);

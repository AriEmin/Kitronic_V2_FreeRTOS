#pragma once
#include <Arduino.h>
#include "Shared.h"

struct PistonAxisConfig {
    const char *name;
    PistonChannel piston;
    uint8_t valveIdx;
    uint8_t supportIdx;
    uint8_t pressureGroup; // 0: Grup-1 (N436), 1: Grup-2 (N440)
};

extern const PistonAxisConfig kPistonAxisConfig[PISTON_CHANNEL_COUNT];

struct PistonControlInput {
    uint32_t now_ms;
    float    dt_s;
    float    hall_raw[PISTON_CHANNEL_COUNT];
    float    vel[PISTON_CHANNEL_COUNT];
    float    pressure_bar[2];
};

struct PistonCalibProgress {
    bool     running;
    uint8_t  piston;
    uint8_t  step;
    char     err[16];
};

void PistonControl_Init();
int  PistonControl_IndexFromName(const char *name);
uint8_t PistonControl_GroupOf(PistonChannel ch);
uint8_t PistonControl_ValveIdx(PistonChannel ch);
uint8_t PistonControl_SupportIdx(PistonChannel ch);
void PistonControl_HandleCalibCommand(const PistonCalibCommand &cmd);
void PistonControl_Update(const PistonControlInput &in, uint16_t targetDuty[8], bool suppressLegacyHold);
void PistonControl_TelemetrySnapshot(PistonRuntimeState (&out)[PISTON_CHANNEL_COUNT], PressureGroupState (&pg)[2]);
void PistonControl_GetCalibProgress(PistonCalibProgress &out);

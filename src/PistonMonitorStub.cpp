// PistonMonitorStub.cpp
// TaskPistonMonitor silindi — global değişkenler ve no-op fonksiyonlar stub olarak sağlanıyor.
// TaskSerial.cpp'deki raporlama kodu bu değişkenleri okur.

#include "PistonMonitor.h"
#include <string.h>

PistonMonitorData g_pistonMonitor[6]  = {};
PistonErrorHistory g_pistonErrors     = {};
bool g_pistonMonitorEnabled           = false;

static const char* s_names[6] = {"P5-7", "P1-3", "P2-4", "P6-R", "K1", "K2"};
const char* PISTON_NAMES[6] = {"P5-7", "P1-3", "P2-4", "P6-R", "K1", "K2"};

void pistonMonitorInit()   {}
void pistonMonitorStart()  { g_pistonMonitorEnabled = true; }
void pistonMonitorStop()   { g_pistonMonitorEnabled = false; }
void pistonMonitorReset()  { memset(&g_pistonMonitor, 0, sizeof(g_pistonMonitor));
                              memset(&g_pistonErrors, 0, sizeof(g_pistonErrors)); }

void pistonMonitorSetTarget(uint8_t pistonIdx, PistonPos target, GearState gear) {
    (void)pistonIdx; (void)target; (void)gear;
}
void pistonMonitorSetClutch(ClutchState clutch, GearState gear) {
    (void)clutch; (void)gear;
}

const char* getPistonStatusStr(PistonStatus status) {
    switch (status) {
        case PISTON_STATUS_IDLE:      return "IDLE";
        case PISTON_STATUS_MOVING:    return "MOVING";
        case PISTON_STATUS_AT_TARGET: return "OK";
        case PISTON_STATUS_ERROR:     return "ERR";
        default:                      return "?";
    }
}
const char* getPistonErrorStr(uint16_t errorType) {
    if (errorType == PERR_NONE)             return "NONE";
    if (errorType & PERR_TIMEOUT)           return "TIMEOUT";
    if (errorType & PERR_UNEXPECTED_MOVE)   return "UNEXPECTED";
    if (errorType & PERR_STUCK)             return "STUCK";
    if (errorType & PERR_OVERSHOOT)         return "OVERSHOOT";
    if (errorType & PERR_OSCILLATION)       return "OSCILLATION";
    if (errorType & PERR_SENSOR_INVALID)    return "SENSOR";
    return "UNKNOWN";
}

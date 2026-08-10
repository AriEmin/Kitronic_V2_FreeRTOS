// PistonCalibStorage.cpp
// NVS (Preferences) tabanlı piston kalibrasyon verisi saklama/yükleme
// TaskCurrentCalib.cpp ve PistonControl.cpp tarafından kullanılır.

#include <Arduino.h>
#include <Preferences.h>
#include "PistonCalib.h"

static Preferences s_prefs;
static bool s_begun = false;

static const char* NVS_NS = "piston_cal2";

static void _begin_rw() {
    if (!s_begun) { s_prefs.begin(NVS_NS, false); s_begun = true; }
}
static void _begin_ro() {
    if (!s_begun) { s_prefs.begin(NVS_NS, true); s_begun = true; }
}

void PistonCalibStorage_Begin() {
    _begin_ro();
}

bool PistonCalibStorage_Load(uint8_t piston, PistonCalibData &out) {
    if (piston >= PISTON_CHANNEL_COUNT) return false;
    _begin_ro();
    char key[12];
    snprintf(key, sizeof(key), "p%d", piston);
    size_t len = s_prefs.getBytesLength(key);
    if (len != sizeof(PistonCalibData)) return false;
    s_prefs.getBytes(key, &out, sizeof(PistonCalibData));
    if (out.version != PISTON_CALIB_VERSION) { memset(&out, 0, sizeof(out)); return false; }
    return out.calibrated;
}

bool PistonCalibStorage_Save(uint8_t piston, const PistonCalibData &data) {
    if (piston >= PISTON_CHANNEL_COUNT) return false;
    s_begun = false;  // reopen r/w
    _begin_rw();
    char key[12];
    snprintf(key, sizeof(key), "p%d", piston);
    return s_prefs.putBytes(key, &data, sizeof(PistonCalibData)) == sizeof(PistonCalibData);
}

void PistonCalibStorage_Clear(uint8_t piston) {
    if (piston >= PISTON_CHANNEL_COUNT) return;
    s_begun = false;
    _begin_rw();
    char key[12];
    snprintf(key, sizeof(key), "p%d", piston);
    s_prefs.remove(key);
}

void PistonCalibStorage_ClearAll() {
    s_begun = false;
    _begin_rw();
    s_prefs.clear();
}

void PistonCalibStorage_LoadAll(PistonCalibData (&out)[PISTON_CHANNEL_COUNT]) {
    for (uint8_t i = 0; i < PISTON_CHANNEL_COUNT; i++) {
        if (!PistonCalibStorage_Load(i, out[i])) {
            memset(&out[i], 0, sizeof(PistonCalibData));
        }
    }
}

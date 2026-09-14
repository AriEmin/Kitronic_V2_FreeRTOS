// ValveCurrentControl.cpp
// 8 kanal valf akım geri beslemeli PI PWM kontrol modülü
//
// Kullanım:
//   TaskValveControl gibi periyodik bir görevden:
//     uint16_t pwm = ValveCurrentControl_Update(vi, measured_mA);
//     PWM_WriteDuty(vi, pwm);
//
//   Komut veren taraflar:
//     ValveCurrentControl_SetMode(vi, VALVE_MODE_OPEN);    // moda göre hedef akım
//     ValveCurrentControl_SetTargetCurrent(vi, 750.0f);    // doğrudan mA
//     ValveCurrentControl_SetManualPWM(vi, 2000);          // akım kontrolü bypass
//
#include "ValveCurrentControl.h"
#include "Shared.h"
#include <Arduino.h>
#include <math.h>

static ValveCurrentSystem s_sys{};

// Akım PI durum değişkenleri (kontrolör yapısında tutulmayan ek dahili durum)
static float   s_rampTarget[8] = {0};
static float   s_prevTarget[8] = {0};
static uint8_t s_settle[8]     = {0};

// Valf indeksi → INA219 sensör indeksi (fiziksel bağlantı)
static const uint8_t VALVE_TO_INA[8] = {0, 3, 1, 2, 5, 7, 6, 4};

// Akım kontrol sabitleri
static constexpr float VC_KP           = 0.5f;    // duty/mA
static constexpr float VC_KI           = 6.0f;    // duty/(mA·s)
static constexpr float VC_SEED_K       = 2.05f;   // duty/mA ön-tohum katsayısı
static constexpr uint8_t VC_SETTLE_N   = 15;      // anti-windup penceresi
static constexpr float VC_DTGT_THRES   = 30.0f;   // mA — büyük adım eşiği
static constexpr float VC_RAMP_MA_PER_S= 6000.0f; // open/open_slow soft-start

static float modeToCurrent(ValveCurrentMode mode, const ValveCurrentTargets& t) {
    switch (mode) {
        case VALVE_MODE_OPEN:      return t.openCurrent_mA;
        case VALVE_MODE_HOLD:      return t.holdCurrent_mA;
        case VALVE_MODE_CLOSE:     return t.closeCurrent_mA;
        case VALVE_MODE_OPEN_SLOW: return t.slowopenCurrent_mA;
        case VALVE_MODE_CLOSE_SLOW:return t.slowcloseCurrent_mA;
        case VALVE_MODE_PCV:       return t.pcvCurrent_mA;
        default:                   return 0.0f;
    }
}

void ValveCurrentControl_Init() {
    s_sys = {};
    s_sys.targets = DEFAULT_CURRENT_TARGETS;
    s_sys.piParams = DEFAULT_PI_PARAMS;
    s_sys.systemEnabled = true;
    s_sys.temperature_C = 25.0f;
    for (int i = 0; i < 8; i++) {
        s_sys.valves[i].enabled = true;
        s_sys.valves[i].mode = VALVE_MODE_OFF;
        s_sys.valves[i].lastUpdateMs = 0;
        s_rampTarget[i] = 0.0f;
        s_prevTarget[i] = 0.0f;
        s_settle[i] = 0;
    }
}

void ValveCurrentControl_SetMode(uint8_t idx, ValveCurrentMode mode) {
    if (idx >= 8) return;
    ValveCurrentController& c = s_sys.valves[idx];
    c.mode = mode;
    c.pwmOutput = 0;
    if (mode == VALVE_MODE_OFF) {
        c.targetCurrent_mA = 0.0f;
        c.integral = 0.0f;
        s_rampTarget[idx] = 0.0f;
        s_prevTarget[idx] = 0.0f;
        s_settle[idx] = 0;
    } else if (mode == VALVE_MODE_MANUAL) {
        // Manuel PWM veya hedef akım ayrıca set edilecek
        c.targetCurrent_mA = 0.0f;
    } else {
        c.targetCurrent_mA = modeToCurrent(mode, s_sys.targets);
    }
}

void ValveCurrentControl_SetModeWithInitialPWM(uint8_t idx, ValveCurrentMode mode, uint16_t initialPWM) {
    if (idx >= 8) return;
    ValveCurrentControl_SetMode(idx, mode);
    s_sys.valves[idx].pwmOutput = initialPWM;
}

void ValveCurrentControl_SetTargetCurrent(uint8_t idx, float mA) {
    if (idx >= 8) return;
    ValveCurrentController& c = s_sys.valves[idx];
    c.targetCurrent_mA = mA;
    if (mA <= 0.0f) {
        c.integral = 0.0f;
        s_rampTarget[idx] = 0.0f;
    }
}

void ValveCurrentControl_SetManualPWM(uint8_t idx, uint16_t pwm) {
    if (idx >= 8) return;
    ValveCurrentController& c = s_sys.valves[idx];
    c.mode = VALVE_MODE_MANUAL;
    c.pwmOutput = pwm;
    c.targetCurrent_mA = 0.0f;
}

void ValveCurrentControl_Enable(uint8_t idx, bool enable) {
    if (idx >= 8) return;
    s_sys.valves[idx].enabled = enable;
    if (!enable) {
        s_sys.valves[idx].pwmOutput = 0;
    }
}

void ValveCurrentControl_EnableSystem(bool enable) {
    s_sys.systemEnabled = enable;
    if (!enable) {
        for (int i = 0; i < 8; i++) {
            s_sys.valves[i].pwmOutput = 0;
        }
    }
}

void ValveCurrentControl_SetTargets(const ValveCurrentTargets& t)    { s_sys.targets = t; }
void ValveCurrentControl_SetPIParams(const ValveCurrentPIParams& p)  { s_sys.piParams = p; }
void ValveCurrentControl_SetTemperature(float temp_C)                { s_sys.temperature_C = temp_C; }

ValveCurrentMode ValveCurrentControl_GetMode(uint8_t idx)            { return (idx < 8) ? s_sys.valves[idx].mode : VALVE_MODE_OFF; }
float   ValveCurrentControl_GetTargetCurrent(uint8_t idx)            { return (idx < 8) ? s_sys.valves[idx].targetCurrent_mA : 0.0f; }
float   ValveCurrentControl_GetMeasuredCurrent(uint8_t idx)          { return (idx < 8) ? s_sys.valves[idx].measuredCurrent_mA : 0.0f; }
uint16_t ValveCurrentControl_GetPWM(uint8_t idx)                   { return (idx < 8) ? s_sys.valves[idx].pwmOutput : 0; }
bool    ValveCurrentControl_IsEnabled(uint8_t idx)                 { return (idx < 8) && s_sys.valves[idx].enabled; }
const ValveCurrentSystem& ValveCurrentControl_GetSystem()          { return s_sys; }

uint16_t ValveCurrentControl_Update(uint8_t idx, float measuredCurrent_mA) {
    if (idx >= 8) return 0;
    ValveCurrentController& c = s_sys.valves[idx];
    c.measuredCurrent_mA = measuredCurrent_mA;

    // Güvenlik: sistem veya valf devre dışı
    if (!s_sys.systemEnabled || !c.enabled) {
        c.pwmOutput = 0;
        c.integral = 0.0f;
        s_rampTarget[idx] = 0.0f;
        s_prevTarget[idx] = 0.0f;
        s_settle[idx] = 0;
        c.lastUpdateMs = millis();
        return 0;
    }

    // dt hesapla
    uint32_t nowMs = millis();
    if (c.lastUpdateMs == 0) c.lastUpdateMs = nowMs - 10; // ilk çağrıda ~10ms kabul et
    float dt_s = (nowMs - c.lastUpdateMs) * 0.001f;
    c.lastUpdateMs = nowMs;
    if (dt_s < 0.001f) dt_s = 0.001f;
    if (dt_s > 0.1f)   dt_s = 0.1f;

    // Manuel PWM modu: akım kontrolü bypass
    if (c.mode == VALVE_MODE_MANUAL && c.pwmOutput > 0 && c.targetCurrent_mA <= 0.0f) {
        return c.pwmOutput;
    }

    // Hedef akım belirleme önceliği:
    // 1. modül içindeki hedef (SetTargetCurrent / SetMode)
    // 2. geriye dönük: g_valveCustomCurrent_mA global override
    // 3. geriye dönük: g_valveCustomMode global
    float target = c.targetCurrent_mA;
    ValveCurrentMode mode = c.mode;
    if (c.mode != VALVE_MODE_MANUAL && target <= 0.0f) {
        if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            if (g_valveCustomCurrent_mA[idx] > 0.0f) {
                target = g_valveCustomCurrent_mA[idx];
                if (g_valveCustomMode[idx] > 0 && g_valveCustomMode[idx] < 6) {
                    mode = (ValveCurrentMode)g_valveCustomMode[idx];
                }
            } else if (g_valveCustomMode[idx] > 0 && g_valveCustomMode[idx] < 6) {
                mode = (ValveCurrentMode)g_valveCustomMode[idx];
                target = modeToCurrent(mode, s_sys.targets);
            }
            xSemaphoreGive(g_sharedMutex);
        }
    }
    c.mode = mode;

    if (target <= 0.0f) {
        c.pwmOutput = 0;
        c.integral = 0.0f;
        s_rampTarget[idx] = 0.0f;
        s_prevTarget[idx] = 0.0f;
        s_settle[idx] = 0;
        return 0;
    }

    // Soft-start: open / open_slow için hedefi yavaş yavaş artır
    if (c.mode == VALVE_MODE_OPEN || c.mode == VALVE_MODE_OPEN_SLOW) {
        float d = target - s_rampTarget[idx];
        float step = VC_RAMP_MA_PER_S * dt_s;
        if (fabsf(d) <= step) {
            s_rampTarget[idx] = target;
        } else {
            s_rampTarget[idx] += copysignf(step, d);
        }
    } else {
        s_rampTarget[idx] = target;
    }

    // Warm-start / feedforward: hedef değişince integral'i yeni denge noktasına kaydır
    float dTgt = s_rampTarget[idx] - s_prevTarget[idx];
    if (s_prevTarget[idx] <= 0.0f) {
        c.integral = s_rampTarget[idx] * (VC_SEED_K - VC_KP);
        s_settle[idx] = VC_SETTLE_N;
    } else if (fabsf(dTgt) >= VC_DTGT_THRES) {
        c.integral += dTgt * (VC_SEED_K - VC_KP);
        s_settle[idx] = VC_SETTLE_N;
    }
    if (c.integral < 0.0f)              c.integral = 0.0f;
    if (c.integral > s_sys.piParams.integralMax) c.integral = s_sys.piParams.integralMax;
    s_prevTarget[idx] = s_rampTarget[idx];

    // PI hesapla
    float err = s_rampTarget[idx] - c.measuredCurrent_mA;
    c.error = err;
    bool settling = (s_settle[idx] > 0);
    if (settling) s_settle[idx]--;
    if (!(settling && err > 0.0f)) {
        c.integral += s_sys.piParams.Ki * err * dt_s;
    }
    if (c.integral < 0.0f)              c.integral = 0.0f;
    if (c.integral > s_sys.piParams.integralMax) c.integral = s_sys.piParams.integralMax;

    float out = s_sys.piParams.Kp * err + c.integral;
    if (out < s_sys.piParams.outputMin) out = s_sys.piParams.outputMin;
    if (out > s_sys.piParams.outputMax) out = s_sys.piParams.outputMax;

    c.pwmOutput = (uint16_t)out;
    return c.pwmOutput;
}

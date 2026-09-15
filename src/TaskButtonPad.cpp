#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "Tasks.h"
#include "Shared.h"

// TCA9555 button panel I2C address
#define BTN_TCA_ADDR 0x22

// TCA9555 registers
#define TCA_REG_INPUT0   0x00
#define TCA_REG_INPUT1   0x01
#define TCA_REG_OUTPUT0  0x02
#define TCA_REG_OUTPUT1  0x03
#define TCA_REG_CONFIG0  0x06
#define TCA_REG_CONFIG1  0x07

// Button mapping: port bit index -> valve index / action
// Port 0 bits 0..7  -> buttons P00..P07
// Port 1 bits 0..7  -> buttons P10..P17
// Returns valve index [0..7] for valve toggles, -1 for non-valve buttons.
static int buttonToValveIndex(uint8_t port, uint8_t bit) {
    if (port == 0) {
        switch (bit) {
            case 0: return 3; // P00 - N435
            case 1: return 2; // P01 - N434
            case 2: return 1; // P02 - N436
            case 3: return 0; // P03 - N433
            default: return -1;
        }
    } else if (port == 1) {
        switch (bit) {
            case 4: return 7; // P14 - N437
            case 5: return 6; // P15 - N439
            case 6: return 5; // P16 - N440
            case 7: return 4; // P17 - N438
            default: return -1;
        }
    }
    return -1;
}

// Button action identifiers
enum ButtonAction {
    ACTION_NONE = 0,
    ACTION_VALVE_TOGGLE,
    ACTION_PUMP_START,
    ACTION_PUMP_STOP,
    ACTION_PUMP_AUTO,
    ACTION_OIL_FILL,
    ACTION_OIL_FILL_STOP,
    ACTION_DRAIN,
    ACTION_VALVE_CLEAN
};

static ButtonAction buttonToAction(uint8_t port, uint8_t bit) {
    int valve = buttonToValveIndex(port, bit);
    if (valve >= 0) return ACTION_VALVE_TOGGLE;

    if (port == 0) {
        switch (bit) {
            case 4: return ACTION_PUMP_START;
            case 5: return ACTION_PUMP_STOP;
            case 6: return ACTION_OIL_FILL;
            case 7: return ACTION_DRAIN;
            default: return ACTION_NONE;
        }
    } else if (port == 1) {
        switch (bit) {
            case 2: return ACTION_VALVE_CLEAN;
            case 3: return ACTION_PUMP_AUTO;
            default: return ACTION_NONE;
        }
    }
    return ACTION_NONE;
}

static bool tcaReadPort(uint8_t reg, uint8_t &out) {
    if (!g_i2cMutex) return false;
    if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;

    Wire.beginTransmission(BTN_TCA_ADDR);
    Wire.write(reg);
    uint8_t err = Wire.endTransmission(false);
    if (err != 0) {
        xSemaphoreGive(g_i2cMutex);
        return false;
    }
    Wire.requestFrom((uint8_t)BTN_TCA_ADDR, (uint8_t)1);
    if (Wire.available() < 1) {
        xSemaphoreGive(g_i2cMutex);
        return false;
    }
    out = (uint8_t)Wire.read();
    xSemaphoreGive(g_i2cMutex);
    return true;
}

static bool tcaWritePort(uint8_t reg, uint8_t value) {
    if (!g_i2cMutex) return false;
    if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(50)) != pdTRUE) return false;

    Wire.beginTransmission(BTN_TCA_ADDR);
    Wire.write(reg);
    Wire.write(value);
    uint8_t err = Wire.endTransmission();
    xSemaphoreGive(g_i2cMutex);
    return err == 0;
}

static bool tcaInit() {
    if (!g_i2cMutex) return false;
    if (xSemaphoreTake(g_i2cMutex, pdMS_TO_TICKS(200)) != pdTRUE) return false;
    Wire.beginTransmission(BTN_TCA_ADDR);
    bool present = (Wire.endTransmission() == 0);
    xSemaphoreGive(g_i2cMutex);
    if (!present) return false;

    return tcaWritePort(TCA_REG_CONFIG0, 0xFF) &&
           tcaWritePort(TCA_REG_CONFIG1, 0xFC) &&
           tcaWritePort(TCA_REG_OUTPUT1, 0xFF);
}

static void sendPumpCmd(PumpCmd cmd, float fillCurrentA = 0.0f, uint32_t fillDurationMs = 0) {
    PumpCommand pc;
    memset(&pc, 0, sizeof(pc));
    pc.cmd = cmd;
    pc.fillCurrentA = fillCurrentA;
    pc.fillDurationMs = fillDurationMs;

    portENTER_CRITICAL(&g_portMux);
    pc.seq = g_pumpCmd.seq + 1;
    g_pumpCmd = pc;
    portEXIT_CRITICAL(&g_portMux);
}

static bool toggleValve(int idx) {
    if (idx < 0 || idx >= 8) return false;

    uint8_t mode = 0;
    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        mode = g_valveCustomMode[idx];
        xSemaphoreGive(g_sharedMutex);
    }

    // Açık değilse aç, açıksa kapat
    bool turnOn = (mode == 0 || mode == 2);

    if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_valveTargetDuty[idx] = 0;
        if (turnOn) {
            g_valveCustomCurrent_mA[idx] = 700.0f;
            g_valveCustomMode[idx] = 1;           // open
        } else {
            g_valveCustomCurrent_mA[idx] = 0.0f;  // off
            g_valveCustomMode[idx] = 0;           // off
        }
        xSemaphoreGive(g_sharedMutex);
    }
    return turnOn;
}

static void publishButtonEvent(const char *button, const char *state) {
    if (!g_sharedMutex) return;
    if (xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        snprintf(g_buttonDisplayEvent.button, sizeof(g_buttonDisplayEvent.button), "%s", button);
        snprintf(g_buttonDisplayEvent.state, sizeof(g_buttonDisplayEvent.state), "%s", state);
        g_buttonDisplayEvent.timestampMs = millis();
        g_buttonDisplayEvent.seq++;
        xSemaphoreGive(g_sharedMutex);
    }
}

// Persistent oil-fill defaults (GUI may override via JSON)
static void loadOilFillDefaults(uint32_t &rpm, uint32_t &durationMs) {
    Preferences prefs;
    if (prefs.begin("btn_oil", true)) {
        rpm = prefs.getUInt("rpm", 3500);
        durationMs = prefs.getUInt("ms", 60000);
        prefs.end();
    } else {
        rpm = 3500;
        durationMs = 60000;
    }
}

static void doAction(ButtonAction action, int valveIdx) {
    static const char *VALVE_NAMES[8] = {"N433", "N436", "N434", "N435", "N438", "N440", "N439", "N437"};
    static bool oilFillActive = false;
    if (!g_controlSession.active) {
        publishButtonEvent("SISTEM", "GIRIS GEREKLI");
        return;
    }
    bool cleanActive = g_valveClean.ch[0].active || g_valveClean.ch[1].active;
    if (cleanActive && action != ACTION_VALVE_CLEAN) {
        publishButtonEvent("TEMIZLEME", "MESGUL");
        return;
    }
    switch (action) {
        case ACTION_VALVE_TOGGLE: {
            bool opened = toggleValve(valveIdx);
            publishButtonEvent(VALVE_NAMES[valveIdx], opened ? "ACIK" : "KAPALI");
            break;
        }
        case ACTION_PUMP_START:
            sendPumpCmd(PUMP_CMD_START);
            publishButtonEvent("POMPA", "ACIK");
            break;
        case ACTION_PUMP_STOP:
        case ACTION_OIL_FILL_STOP:
            sendPumpCmd(PUMP_CMD_STOP);
            oilFillActive = false;
            publishButtonEvent("POMPA", "KAPALI");
            break;
        case ACTION_PUMP_AUTO:
            sendPumpCmd(PUMP_CMD_AUTO);
            publishButtonEvent("POMPA", "OTOMATIK");
            break;
        case ACTION_OIL_FILL: {
            if (oilFillActive) {
                sendPumpCmd(PUMP_CMD_STOP);
                oilFillActive = false;
                publishButtonEvent("YAG DOLUM", "DURDU");
            } else {
                uint32_t rpm = 3500, durationMs = 60000;
                loadOilFillDefaults(rpm, durationMs);
                // Match GUI behavior: 'current' field carries RPM value, ms carries duration
                sendPumpCmd(PUMP_CMD_FILL, (float)rpm, durationMs);
                oilFillActive = true;
                publishButtonEvent("YAG DOLUM", "BASLADI");
            }
            break;
        }
        case ACTION_DRAIN:
            if (g_pumpPub.bar > 10.0f) {
                if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    g_valveDischargeCmd = {2000, 2000, 10.0f, 15000};
                    g_valveDischargeSeq++;
                    xSemaphoreGive(g_sharedMutex);
                }
                publishButtonEvent("BASINC", "BOSALTILIYOR");
            } else {
                publishButtonEvent("BASINC", "10 BAR ALTI");
            }
            break;
        case ACTION_VALVE_CLEAN: {
            bool start = !cleanActive;
            bool busy = false;
            if (start) {
                if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    busy = fabsf(g_pumpPub.rpm) > 50.0f;
                    for (int i = 0; i < 8 && !busy; i++) {
                        busy = g_valveCustomCurrent_mA[i] > 0.0f || g_valveTargetDuty[i] > 0;
                    }
                    xSemaphoreGive(g_sharedMutex);
                } else {
                    busy = true;
                }
            }
            if (start && busy) {
                publishButtonEvent("TEMIZLEME", "ISLEM AKTIF");
                break;
            }
            uint32_t now = millis();
            uint32_t endMs = (start) ? (now + 5U * 60U * 1000U) : 0;
            g_valveClean.ch[0].period_ms = 200;
            g_valveClean.ch[1].period_ms = 200;
            g_valveClean.ch[0].end_ms = endMs;
            g_valveClean.ch[1].end_ms = endMs;
            g_valveClean.ch[0].active = start;
            g_valveClean.ch[1].active = start;
            publishButtonEvent("TEMIZLEME", start ? "BASLADI" : "DURDU");
            break;
        }
        default:
            break;
    }
}

void TaskButtonPad(void *pvParameters) {
    (void) pvParameters;

    // Wait for TaskI2CMonitor to initialize I2C
    vTaskDelay(pdMS_TO_TICKS(1500));

    bool tca_ok = tcaInit();
    uint8_t readFailCount = 0;
    uint32_t lastProbeMs = millis();

    // 16-bit debounced button state: 1 = released, 0 = pressed (active low)
    uint16_t raw_prev = 0xFFFF;
    uint16_t debounced = 0xFFFF;
    bool syncState = true;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(25));

        if (!tca_ok) {
            uint32_t now = millis();
            if (now - lastProbeMs >= 1000) {
                lastProbeMs = now;
                tca_ok = tcaInit();
                syncState = true;
                readFailCount = 0;
            }
            continue;
        }

        uint8_t p0 = 0xFF, p1 = 0xFF;
        bool readOk = tcaReadPort(TCA_REG_INPUT0, p0) &&
                      tcaReadPort(TCA_REG_INPUT1, p1);
        if (!readOk) {
            if (++readFailCount >= 3) {
                tca_ok = false;
                lastProbeMs = millis();
            }
            continue;
        }
        readFailCount = 0;

        uint16_t raw = ((uint16_t)p1 << 8) | p0;
        if (syncState) {
            raw_prev = raw;
            debounced = raw;
            syncState = false;
            continue;
        }

        // Simple debounce: only accept state after it stays stable for ~50 ms
        if (raw == raw_prev) {
            uint16_t changed = debounced ^ raw;
            if (changed) {
                // Detect falling edges (1 -> 0) after debounce
                uint16_t pressed = changed & (~raw);
                for (int i = 0; i < 16; i++) {
                    if (pressed & (1u << i)) {
                        uint8_t port = (i < 8) ? 0 : 1;
                        uint8_t bit = (i < 8) ? (uint8_t)i : (uint8_t)(i - 8);
                        int valve = buttonToValveIndex(port, bit);
                        ButtonAction action = buttonToAction(port, bit);
                        if (action != ACTION_NONE) {
                            doAction(action, valve);
                        }
                    }
                }
                debounced = raw;
            }
        }
        raw_prev = raw;

        // Update status LEDs on P10 (green) and P11 (red)
        // Assume active low: 0 = LED on, 1 = LED off
        bool ocp = false;
        portENTER_CRITICAL(&g_portMux);
        ocp = g_drvOcpLatch;
        portEXIT_CRITICAL(&g_portMux);

        uint8_t out1 = 0xFF;
        if (ocp) {
            out1 &= ~(1u << 0); // red on
        } else {
            out1 &= ~(1u << 1); // green on
        }
        if (tca_ok) {
            tcaWritePort(TCA_REG_OUTPUT1, out1);
        }
    }
}

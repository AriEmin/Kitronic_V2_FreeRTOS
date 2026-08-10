// ValveCurrentControl.cpp
// Stub — tüm kontrol mantığı TaskValveControl.cpp içindeki
// piston_ctrl_step / pcv_pi_step fonksiyonlarına taşındı.
// Bu dosya sadece eski çağrı noktaları için sembol sağlar.

#include "ValveCurrentControl.h"

static ValveCurrentSystem s_sys{};

void    ValveCurrentControl_Init()                                      {}
void    ValveCurrentControl_SetMode(uint8_t, ValveCurrentMode)          {}
void    ValveCurrentControl_SetModeWithInitialPWM(uint8_t, ValveCurrentMode, uint16_t) {}
void    ValveCurrentControl_SetTargetCurrent(uint8_t, float)            {}
void    ValveCurrentControl_SetManualPWM(uint8_t, uint16_t)             {}
void    ValveCurrentControl_Enable(uint8_t, bool)                       {}
void    ValveCurrentControl_EnableSystem(bool)                          {}
uint16_t ValveCurrentControl_Update(uint8_t, float)                    { return 0; }
void    ValveCurrentControl_SetTargets(const ValveCurrentTargets& t)    { s_sys.targets = t; }
void    ValveCurrentControl_SetPIParams(const ValveCurrentPIParams&)    {}
void    ValveCurrentControl_SetTemperature(float)                       {}
ValveCurrentMode ValveCurrentControl_GetMode(uint8_t)                  { return VALVE_MODE_OFF; }
float   ValveCurrentControl_GetTargetCurrent(uint8_t)                  { return 0.0f; }
float   ValveCurrentControl_GetMeasuredCurrent(uint8_t)                { return 0.0f; }
uint16_t ValveCurrentControl_GetPWM(uint8_t)                           { return 0; }
bool    ValveCurrentControl_IsEnabled(uint8_t)                         { return false; }
const ValveCurrentSystem& ValveCurrentControl_GetSystem()              { return s_sys; }

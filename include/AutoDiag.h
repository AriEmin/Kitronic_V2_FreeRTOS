#pragma once
#include <Arduino.h>

// AutoStepDiag gövdesi Shared.h içinde tanımlı.
// Burada SADECE ileri bildirim yeterli:


// --- Fault bitleri ---
enum AutoFaultFlags : uint16_t {
  AFF_NONE       = 0,
  AFF_SLOW_RESP  = 1<<0,
  AFF_NO_DELTA_P = 1<<1,
  AFF_BUS_SAG    = 1<<2,
  AFF_COIL_OPEN  = 1<<3,
  AFF_COIL_SHORT = 1<<4,
  AFF_MECH_STUCK = 1<<5,
};

// --- Eşikler (isteğe göre build_flags ile override) ---
#ifndef AD_P_MIN_DELTA_BAR
# define AD_P_MIN_DELTA_BAR       0.15f
#endif
#ifndef AD_TT_MAX_MS
# define AD_TT_MAX_MS             400u
#endif
#ifndef AD_VBUS_MIN_V
# define AD_VBUS_MIN_V            21.0f
#endif
#ifndef AD_I_COIL_OPEN_MAX_mA
# define AD_I_COIL_OPEN_MAX_mA    50.0f
#endif
#ifndef AD_I_COIL_SHORT_MIN_mA
# define AD_I_COIL_SHORT_MIN_mA   2500.0f
#endif

// --- API ---
void AutoDiag_Begin(const char* stepName, uint16_t activeMask);
void AutoDiag_Sample();


#include "AutoDiag.h"
#include "Shared.h"
#include <string.h>
#include <math.h>

// VALVE index (0..7) -> INA index (0..7)
static const uint8_t VALVE_TO_INA[8] = {0,3,1,2,5,7,6,4};

//AutoStepDiag g_autoStepDiag{}; // tek kopya

// çalışma içi buffer
static AutoStepDiag cur{};
static bool started = false;

// paylaşımları güvenli oku (kısa mutex pencereleri)
static inline void snapshot(float& bar, float& vbus, float& ipump, float coil_mA[8]) {
  if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    bar   = g_pumpPub.bar;              // bar
    vbus  = g_tele.mainV;               // V
    ipump = g_tele.vescI;               // A
    for (int i=0;i<8;i++) coil_mA[i] = g_tele.inaI_mA[i]; // mA
    xSemaphoreGive(g_sharedMutex);
  }
}

void AutoDiag_Begin(const char* stepName, uint16_t activeMask){
  memset(&cur, 0, sizeof(cur));
  strncpy(cur.step, stepName?stepName:"", sizeof(cur.step));
  cur.activeMask = activeMask;
  cur.t_start_ms = millis();

  float bar, vbus, ipA; float cmA[8]{};
  snapshot(bar, vbus, ipA, cmA);

  cur.P_pre    = bar;
  cur.P_peak   = bar;
  cur.Vbus_min = vbus;
  cur.I_pump_peak = fabsf(ipA);
  cur.I_pump_ss   = fabsf(ipA);
  for (int i=0;i<8;i++){ cur.I_coil_peak_mA[i]=fabsf(cmA[i]); cur.I_coil_ss_mA[i]=fabsf(cmA[i]); }

  started = true;
}

void AutoDiag_Sample(){
  if (!started) return;
  float bar, vbus, ipA; float cmA[8]{};
  snapshot(bar, vbus, ipA, cmA);

  if (bar  > cur.P_peak)     cur.P_peak = bar;
  if (vbus < cur.Vbus_min)   cur.Vbus_min = vbus;

  float ipAbs = fabsf(ipA);
  if (ipAbs > cur.I_pump_peak) cur.I_pump_peak = ipAbs;
  cur.I_pump_ss = ipAbs;

  for (int i=0;i<8;i++){
    float a = fabsf(cmA[i]);
    if (a > cur.I_coil_peak_mA[i]) cur.I_coil_peak_mA[i] = a;
    cur.I_coil_ss_mA[i] = a;
  }
}

void AutoDiag_End(uint32_t expect_ms, AutoStepDiag& out){
  cur.t_end_ms  = millis();
  cur.expect_ms = expect_ms;

  float bar, vbus, ipA; float cmA[8]{};
  snapshot(bar, vbus, ipA, cmA);

  cur.P_post = bar;
  cur.dP     = cur.P_post - cur.P_pre;

  uint16_t ff = AFF_NONE;

  if (fabsf(cur.dP) < AD_P_MIN_DELTA_BAR) ff |= AFF_NO_DELTA_P;
  uint32_t dur = cur.t_end_ms - cur.t_start_ms;
  if (dur > AD_TT_MAX_MS && fabsf(cur.dP) < (2*AD_P_MIN_DELTA_BAR)) ff |= AFF_SLOW_RESP;

  if (cur.Vbus_min < AD_VBUS_MIN_V) ff |= AFF_BUS_SAG;

  // coil open/short: sadece aktif masktaki valflerin INA kanalları
  for (int v=0; v<8; v++){
    if (!(cur.activeMask & (1u<<v))) continue;
    uint8_t ina = VALVE_TO_INA[v];
    float ss = cur.I_coil_ss_mA[ina], pk = cur.I_coil_peak_mA[ina];
    if (ss < AD_I_COIL_OPEN_MAX_mA)    ff |= AFF_COIL_OPEN;
    if (pk > AD_I_COIL_SHORT_MIN_mA)   ff |= AFF_COIL_SHORT;
  }

  // mekanik stuck: bobin akımları normal (ne open ne short) ama ΔP yok
  bool anyActive=false, coils_ok=false;
  for (int v=0; v<8; v++){
    if (!(cur.activeMask & (1u<<v))) continue;
    anyActive = true;
    uint8_t ina = VALVE_TO_INA[v];
    float ss = cur.I_coil_ss_mA[ina], pk = cur.I_coil_peak_mA[ina];
    if (ss >= AD_I_COIL_OPEN_MAX_mA && pk <= (AD_I_COIL_SHORT_MIN_mA*0.9f)) coils_ok = true;
  }
  if (anyActive && coils_ok && fabsf(cur.dP) < AD_P_MIN_DELTA_BAR) ff |= AFF_MECH_STUCK;

  cur.faults = ff;

  // son ölçümü global’e aktar (TaskSerial okuyacak)
  if (g_sharedMutex && xSemaphoreTake(g_sharedMutex, pdMS_TO_TICKS(5)) == pdTRUE){
    g_autoStepDiag = cur;
    xSemaphoreGive(g_sharedMutex);
  }

  out = cur;
  started = false;
}

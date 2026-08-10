// include/AutoShift.h
#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
/*
// --- İstek ---
typedef struct {
  bool     start;         // true: başlat / false: durdur
  uint16_t repeats;       // kaç tur
  uint32_t gear_ms;       // her vites adımında bekleme (ms) [min 1000]
} AutoShiftReq;

// --- Durum / Telemetri ---
typedef struct {
  bool     running;
  uint16_t step_idx;
  uint16_t repeat_idx;
  uint32_t gear_ms;
  char     step_name[8];
} AutoShiftPub;

// --- Global paylaşımlar ---
extern volatile uint32_t g_autoShiftReqSeq;
extern AutoShiftReq      g_autoShiftReq;
extern AutoShiftPub      g_autoShiftPub;
*/
// Task prototipi
void TaskAutoShift(void *pvParameters);

// HoldCalibStub.cpp
// TaskHoldCalibration silindi — stub implementasyonlar.
// Yeni kalibrasyon akışı: TaskCurrentCalib ({"current_calib":...} komutuyla)

#include "Shared.h"

bool startHoldCalibration(uint8_t pistonIdx) {
    (void)pistonIdx;
    return false;  // Artık TaskCurrentCalib kullanılıyor
}

void stopHoldCalibration() {
    // no-op
}

# Hold Kontrolü & Kalibrasyon — Yapılacaklar

> Son güncelleme: 2026-05-16
> Hold kontrol stabilitesi ve kalibrasyon doğruluğu için yol haritası.

---

## 🧭 Mevcut Durum (2026-05-16, uzun süreli test sonrası)

**Test sonuçları:**

| Piston | Hedef | Gerçek | Sapma | Yorum |
|--------|-------|--------|-------|-------|
| P0 (5-7) | 13.0 | 13.2-13.4 | +0.3mm | ✅ Stabil |
| P1 (1-3) | 13.0 | 12.9-13.2 | ±0.1mm | ✅ Mükemmel |
| P2 (2-4) | 13.0 | 11.4-11.8 → 12.5-13.0 | -0.4mm | ⚠️ İlk-enable sorunu, re-enable iyileştirdi |
| P3 (6-R) | 13.0 | 13.1-13.4 | +0.2mm | ✅ Stabil |
| P4 (K1)  | 7.5  | 8.5 → 9.7 | +1.8mm drift | ❌ Hold uygun değil — yay sistemi var |
| P5 (K2)  | 7.5  | 4.6-7.6  | -2.8mm drift | ❌ Hold uygun değil — yay sistemi var |

**Karar:**
- P0/P1/P3 production-ready. P2 ilk-enable bug'ı sonra incelenecek.
- **K1/K2 hold kontrolü kaldırıldı** — bkz. aşağı.

---

## 🚫 K1/K2 Karar — Hold Kapatıldı

**Neden:** K1/K2 (debriyaj pistonları) **mekanik yay** ile geri itiliyor. Diğer pistonlar (1-3, 5-7, 2-4, 6-R) ana basınç valfinin (PCV) açıklığına göre konumlanır — ortada tutulabilir. K1/K2'de bu mekanizma yok, sadece yay vs hidrolik basınç dengesi var; "ortada tutma" mekanik olarak çok kararsız.

**Kalibrasyon:** K1/K2 zaten sadece `open/close/min/max` kalibre ediyor, hold akımı bulmuyor. Hold istemine "varsayılan" akımlarla cevap veriliyordu — yanlış davranış.

**Yapılan:**
- `manual_control_page.py`: K1/K2 (N435/N439) için otomatik `piston_hold:enable` gönderimi kaldırıldı.
- K1/K2 artık sadece **fast/slow open/close** olarak çalışıyor (current_ctrl).
- Pozisyon check (`_start_valve_check`) K1/K2 için tekrar etkin (hold devre dışı olduğu için sorun yok).
- **ESP32 firmware guard:** `handlePistonHoldRequest` (`@f:\Proje\Kitronic_Test_Cihazi\Kitronic_Tester_FreeRTOS\src\TaskValveControl.cpp:374-384`) `piston >= 4` için `enable=true` isteğini reddeder. Disable her zaman geçer. Defense-in-depth: GUI yanlış komut gönderse bile hold devreye girmez.

**Sonraki ihtiyaç:** Otomatik testte K1/K2 için "açık tut → arıza beklerken" gerekirse, sadece full-open current uygulaması yeterli (yay açıkta direnir, hidrolik kuvvet daha büyük olur).

---

## 🔜 Sıradaki Görev

**Otomatik kontrol — mekatronik hata tespiti** sayfasına geçiyoruz. Hold kalibrasyonu/kontrolü konusu rafa kaldırıldı.

İleride hold tarafına dönülürse açık eksikler:

1. **P4/P5 için hold gerekirse** — K1/K2 kalibrasyonuna hold-bulma fazı eklenmeli (yay direnci kompansasyonlu, asimetrik açma/kapama eşikleri ile).
2. **I-term ekle (PD → PID)** — P4/P5 olmasa da P0/P2/P3 ±0.3mm bias'ı sıfırlamak için 2-3 mA·s/mm yumuşak integral, anti-windup ile.
3. **P2 ilk-enable bug'ı** — İlk hold enable'da -1.4mm offset, kapat-aç sonrası -0.4mm. Muhtemel sebep: `piston_ctrl_step` static state'leri (`s_pistonLastPos`, `s_pistonVel`, `s_pistonLastCurOut`) ilk cycle'da `holdMa` baz alınmadan hesap yapıyor olabilir. `hold_init_needed` flag'i set olduğunda hepsini fresh init etmek gerekebilir.
4. **Fix 5 — `find_open_threshold` steady-state** (aşağıda detay var, hala valid).

---

## ✅ Tamamlananlar

### Fix 1 — Akım Regülatörü FF Reseed (`valve_current_reg_step`)
`@f:\Proje\Kitronic_Test_Cihazi\Kitronic_Tester_FreeRTOS\src\TaskValveControl.cpp:1466-1479`

- 0→nonzero geçişine ek olarak, hedef akım `>= 30 mA` değiştiğinde integral feedforward kaydırılıyor.
- L/R transient settle penceresi (`VC_SETTLE_N=5`) yeniden kuruluyor.
- Sonuç: 630↔420 mA hedef değişimlerinde integral artık 700ms unwind beklemiyor.

### Fix 2 — Hold P-Çıkış Slew Limit (`piston_ctrl_step`)
`@f:\Proje\Kitronic_Test_Cihazi\Kitronic_Tester_FreeRTOS\src\TaskValveControl.cpp:1409-1417`

- `PIST_SLEW_MA = 30 mA/cycle` — CurOut bir cycle'da en fazla bu kadar değişir.
- Disable durumunda `s_pistonLastCurOut[p] = 0` reset.
- Enable'da ilk değer `holdMa` (0'dan sıçrama yok).

### Bonus — `PistonCalibStorage` version bug fix
- Save'de `version = PISTON_CALIB_VERSION` set edilmiyordu → reboot sonrası kalibrasyon "geçersiz" sayılıyordu.
- Düzeltildi: `@f:\Proje\Kitronic_Test_Cihazi\Kitronic_Tester_FreeRTOS\src\TaskCurrentCalib.cpp:447-458`

---

## ✅ Tamamlananlar (devam)

### Fix 3 — Hold Asimetrik Margin Clamp
`@f:\Proje\Kitronic_Test_Cihazi\Kitronic_Tester_FreeRTOS\src\TaskValveControl.cpp:1437-1453`
- **OPEN yönü:** `max(openMa+30, holdMa+80)` — open ≈ hold sorununu çözer
- **CLOSE yönü:** `closeMa - 30` — fiziksel close eşiğine tam erişim (steady-state error'u önler)
- **İlk versiyon (simetrik 80mA)** steady-state error yaratıyordu: hold-80 floor close threshold'un üstündeydi → piston hedefin üstünde park ediyordu. Asimetriğe çevrildi.

### Fix 4 — Kp Düşür + D-Term
`@f:\Proje\Kitronic_Test_Cihazi\Kitronic_Tester_FreeRTOS\src\TaskValveControl.cpp:1318-1320,1411-1435`
- `PIST_KP_CL: 50 → 25` (doyum bandı 1.6mm → 3.2mm)
- `PIST_KD_CL = 15 mA·s/mm` (hız sönümleme)
- `PIST_VEL_EMA = 0.3` (gürültü filtresi)
- Velocity hesabı `millis()` farkıyla; ilk cycle'da vel=0.
- Disable'da PD state reset.
- Log'a `vel` eklendi.

### Fix 6 — Hold Verify İki Noktada Örnekleme (DEPRECATED — Fix 8 ile çıkarıldı)
- Eski: 1200ms bekle → tek örnek → `drift = pos-target`. Kısa pencerede yavaş kayma yakalanmıyor (P2 hold=open=620 hatası).
- Ara çözüm: `holdMa` uygula → 800ms settle → `pos0` → 3000ms gözlem → `pos1`, iki kriter convergence.
- Çıkarıldı çünkü Fix 8 (formel midpoint) bu loop'u tamamen gereksiz kıldı.

### Fix 9 — Pre-Position Approach Phase (REVERT EDİLDİ — Regresyon)
- **Denenen:** APPROACH state (`|err|>3mm` iken bang-bang full authority) + PD'ye `|err|<1.5 AND |vel|<5` ile devir.
- **Sonuç:** Tüm pistonlar `ph=APP`'de takılı kaldı, end-stop'tan end-stop'a savruldu (0→26→0→26 mm sürekli). Faz 8 verify ve Faz 9 tüm vitesler HATA.
- **Kök neden:** APPROACH modunda D-term (fren) yok. Saf bang-bang full current → piston hedefi 80+ mm/s ile geçiyor → exit koşulu `|err|<1.5 AND |vel|<5` aynı anda asla sağlanmıyor (hızlı geçişte `|vel|>>5`) → sonsuz salınım. PD modunun D-term'i en azından fren basıyordu.
- **Revert:** Tüm wiring kaldırıldı, eski PD-only davranışa döndürüldü.
- **Yeni yaklaşım (Fix 10):** Pre-position TaskAutoShiftV2 tarafında, manuel slow-open mantığıyla.

### Fix 10c — Pre-Position Sürekli Güncelleme (`updatePistonPrePos`)
`@f:\Proje\Kitronic_Test_Cihazi\Kitronic_Tester_FreeRTOS\src\TaskAutoShiftV2.cpp:367-437,1501-1502`
- **Saha sonucu (Fix 10b):** Hala tüm vitesler skip. Log: D1 girişte `H1=1728` (P1-3 kapalı) → bir sonraki snapshot `H1=32752` (P1-3 tam açık). Hiçbir kapatma sinyali gelmeden full open. P6-R aynı şekilde overshoot.
- **Kök neden:** `setPistonValve` sadece **gear değişim anında bir kez** çağrılıyor (`applyGearTargets` içinde). Pre-pos `g_valveCustomCurrent_mA = openMa+30` set edip dönüyor; PI loop sabit akımı sürekli uyguluyor → piston end-stop'a uçuyor. Hiçbir feedback re-evaluate yok.
- **Düzeltme:** Yeni `updatePistonPrePos()` fonksiyonu — sadece `s_pistonInPrePos[i]==true` olan pistonlar için, her loop iterasyonunda (50 Hz):
  1. Anlık `posMm`'i oku (Convention A indeksiyle).
  2. `|err| ≤ PREPOS_EXIT_MM` ise → `s_pistonInPrePos=false`, `queuePistonHold(pA, POS_MID)` ile PD hold devralt.
  3. Aksi halde error yönüne göre `g_valveCustomCurrent_mA` (veya fallback PWM duty) güncelle.
- **Çağrı:** `TaskAutoShiftV2` main loop'unda `userTestFunction()`'dan hemen önce. OPEN/CLOSED pistonlara dokunulmuyor (sadece pre-pos aktif olanlar).

### Fix 10b — Pre-Position Convention Fix + PI Akım Loop'u
`@f:\Proje\Kitronic_Test_Cihazi\Kitronic_Tester_FreeRTOS\src\TaskAutoShiftV2.cpp:289-360`
- **Saha sonucu (Fix 10 ilk versiyon):** Faz 9'da hiçbir vites tutmuyor — pistonlar timeout vuruyor.
- **Kök neden 1 (KRİTİK BUG):** `setPistonValve` Convention B (`0=P1-3, 1=P5-7`) kullanır, ama `g_pistonHallmm[]`, `g_tmagPistonCalib[]`, `g_pistonCalibData[]` hepsi Convention A (PistonChannel: `0=P5-7, 1=P1-3`). Pre-pos `pistonIdx=0`'da P1-3 valfini sürerken **P5-7'nin** pozisyonunu okuyup error hesaplıyordu! Yanlış yön ve yanlış miktar.
- **Kök neden 2:** Open-loop PWM ile `holdDuty(1100) ± 150` = 950–1250 PWM (≈ 463–610 mA). Open eşiği 620–630 mA → asla aşılmıyor → piston yavaş ya da hiç hareket etmiyor.
- **Düzeltme:**
  1. `pA = toHoldIdx[pistonIdx]` ile Convention A'ya çevir; tüm hall/calib erişimlerinde `pA` kullan.
  2. Faz 8 kalibrasyon (`g_pistonCalibData[pA].open_mA / close_mA / hold_mA`) varsa **PI akım döngüsü** üzerinden sür: `g_valveCustomCurrent_mA[valveIdx]` set et.
     - `err < -1 mm` → `openMa + 30` (eşik üstü güvenli aç)
     - `err > +1 mm` → `closeMa - 20` (eşik altı güvenli kapat)
     - deadband → `holdMa`
  3. Kalibrasyon yoksa fallback PWM open-loop, delta `±250` (525-1350 PWM ≈ 415-660 mA — geniş yelpaze).
- **Avantaj:** PI akım loop coil sıcaklığı/voltaj sapmalarına dirençli; calibre eşikler kesin sınır verir, piston kontrollü hızla hareket eder.

### Fix 10 — Pre-Position TaskAutoShiftV2'de (`setPistonValve`) [SUPERSEDED by 10b]
`@f:\Proje\Kitronic_Test_Cihazi\Kitronic_Tester_FreeRTOS\src\TaskAutoShiftV2.cpp:183-192,267-326`
- **Yapılan:** `setPistonValve(POS_MID)` artık state machine:
  1. **Fresh entry** (`prevTarget != POS_MID`): `s_pistonInPrePos[idx]=true`.
  2. **PRE-POS aktif AND |err|>2 mm:** PD hold OFF (`queuePistonHold(POS_CLOSED)`), valf direkt PWM ile sürülür:
     - `errMm < -1 mm` → `dutyHold + 150` PWM (yumuşak aç)
     - `errMm > +1 mm` → `dutyHold - 150` PWM (yumuşak kapat)
     - deadband içinde → sadece `dutyHold`
  3. **|err| ≤ 2 mm:** `s_pistonInPrePos=false`, `queuePistonHold(POS_MID)` ile PD devralır. PD küçük error gördüğü için saturate olmaz, salınım yok.
- **Hizmet:** Manuel davranışı birebir taklit eder (kullanıcı manuelde önce slow-open ile yaklaştırıp hold açar). PD'nin "0 mm'den 13 mm'e tek seferde uç" sorunu yok.
- **Faz 8 verify:** TaskAutoShiftV2 değil TaskCurrentCalib çalıştırıyor; o yüzden bu fix Faz 8'i doğrudan etkilemez. Faz 8'de pre-revert sonrası eski PD davranışı (P0/P2/P3 ~stable, P1 oynak) geri gelmeli.
- **Beklenti:** Faz 9 D1-D7 transition'larında piston end-stop'a uçmadan ~13 mm civarına yumuşak yaklaşır, PD ±0.5 mm bandında tutar.
- Reset: `resetPistonHoldState()` test başında her piston için `s_pistonInPrePos=false`, `s_lastSetTarget=POS_CLOSED`.

### Fix 8 — Hold Convergence Loop'u Çıkarıldı, Midpoint Formel Sonuç
`@f:\Proje\Kitronic_Test_Cihazi\Kitronic_Tester_FreeRTOS\src\TaskCurrentCalib.cpp:416-441`
- **Saha gözlemi:** Tüm pistonlarda gerçek hold akımı 510-540 mA bandında. `(openMa+closeMa)/2` bunu ±20 mA içinde tutturuyor. Iteratif convergence loop (10 iter, pos0/pos1, drift+rate, Kp=5) tipik olarak "not converged" sonlanıp zaten midpoint'e düşüyordu — boşa süre, gürültülü sonuç.
- **Yapılan:** `hold_verify` iter loop tamamen kaldırıldı. `holdMa = (openMa+closeMa)/2` formel sonuç. Tek seferlik bilgi-amaçlı `hold_midpoint` log kaydı (settle + tek örnek) raporlama için tutuldu.
- **Süre kazancı:** Faz 8 piston başına ~26 sn → ~3 sn (sadece settle). 4 piston için toplam ~90 sn tasarruf.
- **Çalışma anı:** PD kontrolör (`piston_ctrl_step`) gerçek dengeyi ±0.3 mm içinde yakaladığı için kalibrasyon-zamanı convergence'a gerek yok.
- Kullanılmayan sabitler temizlendi: `CC_HOLD_DRIFT_MM`, `CC_HOLD_OBSV_MS`, `CC_HOLD_STABLE_CNT`, `CC_HOLD_MAX_ITER`.

---

## 🟡 Sıradaki — Yazılım Tarafı

### Fix 3 — Hold Kontrolde Sabit Margin Clamp (ÖNCELİK 1) ✅ TAMAM

**Sorun:** `open_mA ≈ hold_mA` durumunda P-kontrolör açma yönünde güç üretemiyor.
- P2: open=hold=610 → error<0 olsa CurOut=610 (zaten open, ekstra kuvvet yok)
- P1: open-hold=35 → 0.7mm hata = doyum

**Çözüm:** `piston_ctrl_step` içinde `openMa/closeMa` clamp yerine hold etrafında sabit margin:

```cpp
// MEVCUT:
if (CurOut > openMa)  CurOut = openMa;
if (CurOut < closeMa) CurOut = closeMa;

// YENİ:
static constexpr float HOLD_MARGIN = 80.0f;  // mA
float CurMax = holdMa + HOLD_MARGIN;
float CurMin = holdMa - HOLD_MARGIN;
// Yine de fiziksel limitler aşılmasın:
if (CurMax > openMa + 30.0f) CurMax = openMa + 30.0f;
if (CurMin < closeMa - 30.0f) CurMin = closeMa - 30.0f;
if (CurOut > CurMax) CurOut = CurMax;
if (CurOut < CurMin) CurOut = CurMin;
```

Notlar:
- `HOLD_MARGIN=80` ampirik başlangıç. Test ile 60-100 arası ayarlanabilir.
- Margin hold etrafında **simetrik** olduğu için her iki yönde eşit kuvvet.
- Fiziksel `openMa+30/closeMa-30` üst sınır = donanım koruması.

### Fix 4 — Kp Düşür + D-Term Ekle (ÖNCELİK 2)

**Sorun:** Kp=50 mA/mm + 80 mA margin → doyum eşiği 80/50 = 1.6mm. Hala dar.
Damping yok → osilasyon.

**Çözüm:**
```cpp
static constexpr float PIST_KP_CL = 25.0f;   // 50 → 25 (yarıya)
static constexpr float PIST_KD_CL = 15.0f;   // mA / (mm/s)

// Hız hesabı (basit fark):
float dt = 0.01f;  // ~10ms control period
float vel = (posMm - s_pistonLastPos[p]) / dt;
s_pistonLastPos[p] = posMm;

// PD kontrolör:
float CurOut = holdMa - PIST_KP_CL * error - PIST_KD_CL * vel;
```

Notlar:
- `s_pistonLastPos[6]` yeni static array gerek.
- Disable'da reset.
- Hall pos gürültülüyse vel'i basit EMA ile filtrele: `vel_f = 0.7*vel_f + 0.3*vel`
- Kp=25 → doyum eşiği 80/25 = **3.2 mm** (geniş çalışma bandı).
- Kd*vel ile öngörü: piston hızlı ilerliyorsa fren basıyor → overshoot azalır.

---

## 🟠 Sonra — Kalibrasyon Algoritması

### Fix 5 — `find_open_threshold` Steady-State'e Çevir

**Sorun:** Mevcut algoritma piston tam kapalıyken **breakaway / cracking pressure** ölçüyor. Bu, steady-state opening current değil. Sonuç: open_mA gerçekten piston'u açan akımdan **yüksek** çıkıyor; bazen hold_mA'ya eşit.

**Yeni algoritma (öneri):**

1. **Yüksek akımla pistonu tam aç** (örn 700 mA, 1 sn) → piston açık konumda
2. **Akımı orta seviyeye düşür** (450 mA) ve piston'u orta civarına bekle
3. **Akımı kademeli azalt** (10 mA adım, her adım 500ms tut)
4. Her adımda piston pozisyonunu izle
5. **Piston düşmeye başladığı (negatif velocity)** ilk akım = steady-state hold sınırı
6. `open_mA` = bu sınır + 20 mA güvenlik payı (piston'u yukarı itecek minimum)

Lokasyon: `@f:\Proje\Kitronic_Test_Cihazi\Kitronic_Tester_FreeRTOS\src\TaskCurrentCalib.cpp` — `findOpenThreshold` veya benzer fonksiyon.

### Fix 6 — Hold Verify Güvenilirliği

**Sorun:** Aynı mA'da farklı drift değerleri (P0: 596mA → -2.8, 600mA → +12.9). Histerezis + kısa pencere.

**İyileştirmeler:**
- Verify öncesi piston'u her iterasyonda **aynı yöntemle** orta pozisyona getir (ör: full-open sonra yavaş kapatma).
- Drift ölçüm penceresi 2sn → 5sn.
- Drift kabul eşiği: `< 1.0mm` yerine `|drift| < 0.5mm` **VE** `|velocity| < 0.5 mm/s`.
- Max 10 iterasyon yetmiyorsa, **midpoint yerine** son 3 başarısız iterasyonun ortalamasını al.

### Fix 7 — Bang-Bang Faz Düzeltme

**Sorun:** P1, P2, P3 bang fazı `near=0` ile bitti — piston hedefe ulaşamadan kuruldu.

```
P3 bang_done near=0 pos=15.5 tgt=13.5  (2mm overshoot)
```

`bang` algoritması orantısal kapama (`mA = open - ratio*(open-close)`) kullanıyor ama overshoot sonrası yeterli kapatma yapmıyor.

İyileştirme: bang fazına basit hız sınırlama veya integral ekle, ya da `near` koşulunu pos hedefe yakınsa kabul et (mevcut: konum hedefin %5 içinde mi?).

---

## 🔵 Test Listesi

Her fix sonrası tekrarlanacak:

1. **Tam kalibrasyon** — `{"current_calib":{"all":true}}` → tüm pistonlar
   - Beklenen: hold-open farkı ≥ 50 mA, hold converge etti, drift < 1mm
2. **Reboot sonrası ortada tut** — kalibrasyon değerleri NVS'den yüklenmiş mi?
3. **Tek tek hold testi** — her piston için 30 sn ortada tut
   - Beklenen: pos osilasyonu ±3 mm içinde
4. **Manuel akım komutu transient** — `current_ctrl mode:open/pcv` arası geçiş
   - Beklenen: Iact, Iout'u <100ms'de takip eder, overshoot < %10

---

## 📌 Notlar

- **P0** sürekli converge etmiyor — histerezis ihtimali yüksek. Mekanik kontrol gerekebilir (yağ basıncı, valf temizliği).
- **K1/K2 (P4, P5)** için hold algoritması yok; sadece open/close kalibre ediliyor. Hold gerekirse benzer mantık eklenebilir.
- Slew limit (30 mA/cycle) test sonrası 20 veya 50'ye fine-tune edilebilir.
- Log periyodu (500ms) transient analizi için yetersiz; geçici olarak 100ms yapılabilir.

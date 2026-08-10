# Kitronic Tester — Donanım Referansı

Bu belge, mevcut firmware kaynaklarından çıkarılmış donanım envanteridir. Yeni firmware projesinde kart tanımları, çevrebirim başlatmaları ve sürücü eşleşmeleri için kaynak olarak kullanılmalıdır.

> Kapsam: `include/Tasks.h`, `platformio.ini` ve donanım task/sürücü kaynaklarındaki etkin uygulama. Eski kart uyumluluğu veya kullanılmayan tanımlar ayrıca işaretlenmiştir.

## 1. Hedef platform ve derleme ortamı

| Alan | Değer |
|---|---|
| MCU kartı | ESP32-S3-DevKitM-1 |
| PlatformIO platformu | `espressif32` |
| Framework | Arduino (FreeRTOS içerir) |
| Yükleme protokolü | `esptool` |
| Upload hızı | 115200 baud |
| Seri monitör | 115200 baud |
| PC haberleşmesi | USB CDC; `ARDUINO_USB_MODE=1`, `ARDUINO_USB_CDC_ON_BOOT=1` |
| USB kimliği | Ürün: `Kitronic Tester`; üretici: `Kitronic` |

### PlatformIO bağımlılıkları

```ini
lib_deps =
  adafruit/Adafruit ADS1X15@^2.6.0
  adafruit/Adafruit INA219@^1.2.3
  bblanchon/ArduinoJson @ ^7.4.2
  jandrassy/EthernetENC@^2.0.5
```

`EthernetENC` bağımlılığı yapılandırmada bulunuyor; incelenen aktif kaynaklarda Ethernet kullanımı yoktur. Yeni projeye yalnızca Ethernet işlevi eklenecekse alınmalıdır.

### SDK / framework başlıkları

- Arduino çekirdeği: `Arduino.h`, `Wire.h`, `SPI.h`, `Preferences.h`
- ESP-IDF TWAI sürücüsü: `driver/twai.h`
- FreeRTOS: `freertos/FreeRTOS.h`, `freertos/task.h`, `freertos/semphr.h`

## 2. ESP32-S3 GPIO pin haritası

| GPIO | Sembol | Yön / arayüz | Karttaki işlev | Durum |
|---:|---|---|---|---|
| 1 | `PRESSURE_PIN` | ADC giriş | Basınç sensörü | Etkin |
| 2 | `TEMP_SENSOR_2_PIN` | ADC giriş | NTC sıcaklık sensörü 2; GUI'ye gönderilir | Etkin |
| 3 | `BLDC_SELECT` | Dijital çıkış | Pompa/yağ doldurma seçici röle | Etkin |
| 4 | `LED_RUN` | Dijital çıkış | Çalışma LED'i | Etkin |
| 5 | `N433_PWM_OUT` | LEDC PWM | DRV8243 U16 IN1, N433 | Etkin |
| 6 | `N436_PWM_OUT` | LEDC PWM | DRV8243 U16 IN2, N436 | Etkin |
| 7 | `N434_PWM_OUT` | LEDC PWM | DRV8243 U15 IN1, N434 | Etkin |
| 8 | `I2C_SDA` | I2C | Ortak I2C veri hattı | Etkin |
| 9 | `I2C_SCL` | I2C | Ortak I2C saat hattı | Etkin |
| 10 | `TEMP_SENSOR_1_PIN` | ADC giriş | NTC sıcaklık sensörü 1 | Etkin |
| 11 | `SSR_CONTROL` | Dijital çıkış | Isıtıcı SSR | Etkin |
| 13 | `CAN_TX` | TWAI TX | SN65HVD230DR CAN transceiver | Etkin |
| 14 | `CAN_RX` | TWAI RX | SN65HVD230DR CAN transceiver | Etkin |
| 15 | `N435_PWM_OUT` | LEDC PWM | DRV8243 U15 IN2, N435 / K1 | Etkin |
| 16 | `VALVE_CLEAN_1` | Dijital çıkış | Valf temizleme MOSFET kanalı 1 | Etkin |
| 17 | `VALVE_CLEAN_2` | Dijital çıkış | Valf temizleme MOSFET kanalı 2 | Etkin |
| 18 | `HALL_N437_PIN` | Tanımlı GPIO | Eski/ayrı Hall tanımı | Etkin akışta kullanılmıyor |
| 19 | `USB_DN` | USB D- | USB OTG | Donanım tanımı |
| 20 | `USB_DP` | USB D+ | USB OTG | Donanım tanımı |
| 21 | `SELO_2_IO_INT` | Kesme girişi | TCA9555 #2 interrupt | Tanımlı, aktif akışta kullanılmıyor |
| 35 | `SPI_MOSI` | Yazılımsal SPI çıkışı | DRV8243 veri hattı | Etkin |
| 36 | `SPI_SCK` | Yazılımsal SPI çıkışı | DRV8243 saat hattı | Etkin |
| 37 | `SPI_MISO` | Yazılımsal SPI girişi | DRV8243 veri hattı | Etkin |
| 38 | `CAN_RS` | Dijital çıkış | SN65HVD230DR mod seçimi; `LOW` normal mod | Etkin |
| 39 | `N438_PWM_OUT` | LEDC PWM | DRV8243 U4 IN1, N438 | Etkin |
| 40 | `N440_PWM_OUT` | LEDC PWM | DRV8243 U4 IN2, N440 | Etkin |
| 41 | `N439_PWM_OUT` | LEDC PWM | DRV8243 U17 IN1, N439 / K2 | Etkin |
| 42 | `N437_PWM_OUT` | LEDC PWM | DRV8243 U17 IN2, N437 | Etkin |
| 46 | `ESC_UART_RX` | UART RX | ESC veri hattı | Tanımlı, incelenen aktif akışta kullanılmıyor |
| 47 | `SELO_1_IO_INT` | Kesme girişi | TCA9555 #1 interrupt | Tanımlı, aktif akışta kullanılmıyor |

### Eski veya çakışan tanımlar

- `HALL_N434_PIN=12`, `HALL_N433_PIN=2`, `HALL_N437_PIN=18`, `HALL_N438_PIN=10` tanımları bulunmaktadır; güncel pozisyon kontrolü bunları değil, I2C üzerindeki TMAG5173 sensörlerini kullanır.
- GPIO2 ve GPIO10 aynı zamanda güncel NTC ADC girişleri olarak kullanılır. Bu nedenle eski Hall tanımlarını yeni projeye aynen taşımak pin çakışması oluşturur.
- `USB_DN` ve `USB_DP` sembolleri bulunur; USB CDC yapı bayrakları USB işlevini etkinleştirir, uygulama doğrudan bu GPIO'ları sürmez.

## 3. Valf sürme altyapısı

Sekiz solenoid valf, dört adet çift kanallı **DRV8243** sürücüsü ile kontrol edilir. Her valf için ayrı ESP32 LEDC PWM çıkışı kullanılır. DRV sürücülerinin `nSCS`, `DRVOFF` ve `nFAULT` sinyalleri iki adet TCA9555 I2C genişletici üzerinden yönetilir.

### Valf / PWM / sürücü eşleşmesi

| PWM indeksi | Valf | ESP32 GPIO | LEDC kanal | DRV8243 | Kanal | Kontrol ettiği eleman |
|---:|---|---:|---:|---|---|---|
| 0 | N433 | 5 | 0 | U16 / DRV1 | OUT1 | Piston 1/3 |
| 1 | N436 | 6 | 1 | U16 / DRV1 | OUT2 | PCV, grup 1 |
| 2 | N434 | 7 | 2 | U15 / DRV2 | OUT1 | Piston 5/7 |
| 3 | N435 | 15 | 3 | U15 / DRV2 | OUT2 | K1 kavrama |
| 4 | N438 | 39 | 4 | U4 / DRV4 | OUT1 | Piston 6/R |
| 5 | N440 | 40 | 5 | U4 / DRV4 | OUT2 | PCV, grup 2 |
| 6 | N439 | 41 | 6 | U17 / DRV3 | OUT1 | K2 kavrama |
| 7 | N437 | 42 | 7 | U17 / DRV3 | OUT2 | Piston 2/4 |

> Dikkat: PWM indeks sırası, INA219 adres sırası veya valf numarasının doğal sırası değildir. Yeni yazılımda bu tablo tek otorite olmalıdır.

### LEDC PWM yapılandırması

| Parametre | Değer |
|---|---:|
| Frekans | 3000 Hz |
| Çözünürlük | 12 bit |
| Duty aralığı | 0–4095 |
| Kanal sayısı | 8 |
| Başlangıç duty | 0 |

Her çıkış sırasıyla `ledcSetup(channel, 3000, 12)`, `ledcWrite(channel, 0)` ve `ledcAttachPin(pin, channel)` ile başlatılır.

### DRV8243 SPI ve TCA9555 eşleşmesi

| DRV | TCA9555 adresi | `nSCS` biti | `DRVOFF` biti | `nFAULT` biti | Sürdüğü valfler |
|---|---:|---:|---:|---:|---|
| DRV1 / U16 | `0x20` | P00 | P02 | P04 | N433, N436 |
| DRV2 / U15 | `0x20` | P01 | P03 | P05 | N434, N435 |
| DRV3 / U17 | `0x21` | P00 | P02 | P04 | N439, N437 |
| DRV4 / U4 | `0x21` | P01 | P03 | P05 | N438, N440 |

- DRV8243 SPI haberleşmesi **bit-bang** ile GPIO35/36/37 üzerinde 16-bit frame olarak yapılır.
- `DRVOFF` aktif düşüktür: `LOW` sürücüyü kapatır, `HIGH` etkinleştirir.
- TCA9555 tarafında P00–P03 çıkış; P04–P07 ve P10–P17 giriş olarak yapılandırılır.
- Başlatma akışı: TCA9555 başlat → DRV etkinleştir → hata temizle/preset uygula → LEDC PWM başlat.
- DRV güvenlik preset'i: `COMMAND(0x08)=0x90`, `CONFIG1(0x0A)=0x80`, `CONFIG2(0x0B)=0x64`, `CONFIG3(0x0C)=0x01`.
- OCP veya TSD tespitinde bütün PWM çıkışları 0 yapılır ve bütün DRVOFF hatları kapalı konuma çekilir.

## 4. I2C veri yolu ve cihaz adresleri

Ortak I2C veri yolu GPIO8 (SDA), GPIO9 (SCL) üzerindedir ve **400 kHz** ile başlatılır. Birden çok FreeRTOS task'i aynı hattı kullandığından I2C erişimi global `g_i2cMutex` ile seri hale getirilmelidir.

| I2C adresi | Cihaz | İşlev |
|---:|---|---|
| `0x20` | TCA9555 #1 | DRV1/DRV2 `nSCS`, `DRVOFF`, `nFAULT`, N433–N436 düğmeleri |
| `0x21` | TCA9555 #2 | DRV3/DRV4 `nSCS`, `DRVOFF`, `nFAULT`, N437–N440 düğmeleri |
| `0x35` | TMAG5173-Q1 | Her TCA9548A kanalındaki manyetik konum sensörü |
| `0x40` | INA219 | N433 bobin akımı/gerilimi |
| `0x41` | INA219 | N434 bobin akımı/gerilimi |
| `0x42` | INA219 | N435 bobin akımı/gerilimi |
| `0x43` | INA219 | N436 bobin akımı/gerilimi |
| `0x44` | INA219 | N437 bobin akımı/gerilimi |
| `0x45` | INA219 | N438 bobin akımı/gerilimi |
| `0x46` | INA219 | N439 bobin akımı/gerilimi |
| `0x47` | INA219 | N440 bobin akımı/gerilimi |
| `0x48` | ADS1115 | Eski kartlarda iki NTC için isteğe bağlı / otomatik algılama |
| `0x4C` | INA226 | Ana giriş gücü izleme |
| `0x4D` | INA226 | VESC güç hattı izleme |
| `0x70` | TCA9548A | Sekiz kanallı TMAG5173 I2C multiplexer |

### INA219 bobin izleme

- Sekiz INA219, `Adafruit_INA219` ile başlatılır.
- Her biri `setCalibration_32V_2A()` kullanır.
- Okuma periyodu 100 ms; akım verisine `alpha=0.15` EMA uygulanır.
- INA219 dizi/eşleme sırası: `N433`, `N434`, `N435`, `N436`, `N437`, `N438`, `N439`, `N440`.

### INA226 güç izleme

- Kütüphane yerine doğrudan `Wire` register işlemleri kullanılır.
- Başlatma: `CONFIG(0x00)=0x4527`, `CALIB(0x05)=1024`.
- Bus voltaj dönüşümü: ham değer × `0.00125 V`.
- Akım dönüşümü: signed ham değer × `0.01 A`.
- `CALIB=1024` ve akım ölçeği gerçek şönt direncine göre doğrulanmalıdır; kaynakta geçici/saha ayarı olarak belirtilmiştir.

### TCA9548A + TMAG5173 konum algılama

Sekiz TMAG5173 aynı `0x35` adrese sahiptir; her biri TCA9548A'nın farklı kanalında bulunduğu için bağımsız erişilir.

| TCA9548A kanalı | TMAG adı | Bağlı mekanizma |
|---:|---|---|
| 0 | `TMAG_CH_1_3` | Piston 1/3 |
| 1 | `TMAG_CH_5_7` | Piston 5/7 |
| 2 | `TMAG_CH_2_4` | Piston 2/4 |
| 3 | `TMAG_CH_6_R` | Piston 6/R |
| 4 | `TMAG_CH_K1_1` | K1 sensör 1 / açık taraf |
| 5 | `TMAG_CH_K1_2` | K1 sensör 2 / kapalı taraf |
| 6 | `TMAG_CH_K2_1` | K2 sensör 1 / açık taraf |
| 7 | `TMAG_CH_K2_2` | K2 sensör 2 / kapalı taraf |

TMAG5173 başlatma ayarları:

| Register | Yazılan değer | Anlamı |
|---:|---:|---|
| `DEVICE_CONFIG_1 (0x00)` | `0x40` | 32× averaging |
| `SENSOR_CONFIG_1 (0x02)` | `0x74` | X/Y/Z kanalları açık |
| `T_CONFIG (0x07)` | `0x01` | Dahili sıcaklık ölçümü açık |
| `DEVICE_CONFIG_2 (0x01)` | `0x12` | Düşük gürültü, sürekli ölçüm |

- Varsayılan manyetik aralık ±40 mT'tir.
- TMAG okuma periyodu 100 ms'dir.
- Sensör kaybolursa sonraki döngülerde yeniden `begin()` denenir.
- Konum dönüşümü için uç/mid kalibrasyon değerleri NVS'te tutulur; yeni projede sabit lineer dönüşüm yerine kalibrasyon katmanı korunmalıdır.

## 5. Analog girişler ve sıcaklık ölçümü

### Basınç

| Alan | Değer |
|---|---|
| Giriş | GPIO1 / `PRESSURE_PIN` |
| ADC ham çözünürlüğü | Kaynakta 12-bit varsayımı, 0–4095 |
| Pompa task örnek sayısı | 10 |
| Örnekler arası bekleme | 2 ms |
| Basınç dönüşümü | `(avg - 138) * 0.024175 * 1.65 + 5.0` bar |
| Filtre | `0.25 * raw + 0.75 * previous` |

Bu dönüşüm saha kalibrasyonudur. Yeni kart/sensörde basınç transdüseri, ADC attenuation ve dönüşüm katsayıları yeniden doğrulanmadan kullanılmamalıdır.

### NTC sıcaklık sensörleri

| Sensör | Yeni kart bağlantısı | Eski kart bağlantısı |
|---|---:|---|
| Sıcaklık 1 | GPIO10 | ADS1115 `AIN2` |
| Sıcaklık 2 | GPIO2 | ADS1115 `AIN3` |

- Yeni kartta global ADC attenuation `ADC_11db` yapılır; her giriş 4 örnek ortalamasıyla okunur.
- ADS1115 `0x48` başlatmada bulunursa eski kart yolu seçilir: `GAIN_ONE` (±4.096 V), `RATE_ADS1115_860SPS`.
- NTC modeli: 3.3 V besleme, 10 kΩ alt direnç, 10 kΩ @ 25 °C, Beta=3950, EMA `alpha=0.08`.
- Isıtıcı güvenlik katmanı 90 °C'de SSR isteğini kapatır.

## 6. Pompa, VESC ve CAN

Pompa, bir **VESC** ile CAN/TWAI üzerinden sürülür. Fiziksel CAN transceiver **SN65HVD230DR**'dir.

| Alan | Değer |
|---|---|
| CAN hızı | 500 kbit/s |
| CAN modu | Normal |
| Filtre | Tüm çerçeveleri kabul et |
| VESC CAN ID | 10 |
| VESC kutup çifti | 4 |
| CAN TX / RX | GPIO13 / GPIO14 |
| Transceiver mode/select | GPIO38 / `CAN_RS`; başlatmada `LOW` |
| Pompa seçici röle | GPIO3 / `BLDC_SELECT` |
| Basınç hedefi | 60 bar |
| Otomatik yeniden başlatma eşiği | 42 bar |
| Pompa zaman aşımı | 15 saniye |

VESC çerçeve düzeni:

- Set current: paket türü `0`, 4-byte signed değer, amper × 1000.
- Set RPM: paket türü `3`, 4-byte signed değer, mekanik RPM × 4.
- Kod hem extended hem de standard CAN çerçevesi göndermeyi dener; yeni projede VESC yapılandırmasına göre tek doğru çerçeve türü seçilmelidir.

## 7. Yardımcı dijital çıkışlar

| İşlev | GPIO | Başlatma / çalışma biçimi |
|---|---:|---|
| Çalışma LED'i | 4 | `OUTPUT`, başlangıçta `LOW` |
| Isıtıcı SSR | 11 | `OUTPUT`, başlangıçta `LOW`; setpoint ve zaman oranlı aç/kapa |
| Valf temizleme 1 | 16 | `OUTPUT`, %50 duty toggle; periyot 100–2000 ms |
| Valf temizleme 2 | 17 | `OUTPUT`, %50 duty toggle; periyot 100–2000 ms |
| Pompa/yağ doldurma röle seçimi | 3 | `LOW`: basınç pompası, `HIGH`: yağ doldurma pompası |

## 8. Kalıcı bellek

ESP32 `Preferences` / NVS kullanılır. Yeni projede aşağıdaki donanıma bağlı kalibrasyonların kaydedilmesi gerekir:

- Piston açık/kapalı/orta konum referansları
- TMAG5173 ham Z ↔ mm kalibrasyonları
- K1/K2 iki sensör kalibrasyonu
- Piston valf duty / hold / eşik değerleri
- Otomatik test parametreleri

Kalibrasyon, mekanik ve sensöre özgüdür. Yeni proje başlangıcında eski NVS verilerini varsayılan doğru değer kabul etmeyin; sürümleme ve CRC ile doğrulayın.

## 9. Yeni proje için önerilen başlatma sırası

1. USB CDC seri haberleşmesini başlatın.
2. Paylaşılan mutex'leri (`g_sharedMutex`, `g_i2cMutex`) oluşturun.
3. I2C'yi GPIO8/9 ve 400 kHz ile başlatın.
4. TCA9555'leri güvenli çıkış durumuyla başlatın; DRV8243 `DRVOFF` hatlarını sürücüler etkinleşmeden önce kapalı tutun.
5. Yazılımsal DRV SPI pinlerini GPIO35/36/37 olarak başlatın, sürücü register preset'lerini uygulayın ve fault durumlarını okuyun.
6. Tüm valf LEDC kanallarını 3 kHz / 12-bit / duty 0 ile başlatın.
7. INA219 ve INA226 cihaz varlıklarını kontrol edin.
8. TCA9548A'yı ve sekiz TMAG5173 kanalını tek tek başlatın.
9. Basınç ve NTC ADC kalibrasyonlarını doğrulayın.
10. CAN/TWAI ve VESC bağlantısını başlatın; CAN_RS'i normal modda tutun.
11. Kalibrasyon verilerini NVS'ten yükleyip doğrulayın.
12. Ancak tüm fault/kalibrasyon kontrolleri geçtikten sonra valf ve pompa komutlarını kabul edin.

## 10. Taşıma sırasında dikkat edilmesi gerekenler

- `Tasks.h` hem pin haritası hem de task prototiplerini içeriyor. Yeni projede bu iki sorumluluğu ayırıp `config/pins.h`, `config/i2c_map.h` ve `config/hardware_config.h` altında toplamak daha güvenlidir.
- `include/modules/drv8243.cpp` içinde `config/pins.h` include'u bulunuyor, ancak mevcut projede bu dosya yok; aynı pin sembolleri `Tasks.h` üzerinden kullanılmaktadır. Yeni projede tek bir gerçek pin başlığı oluşturun.
- TCA9555 için iki farklı uygulama vardır: `include/modules/tca9555.cpp` ve `TaskValveControl.cpp` içindeki yerel TCA yardımcıları. Etkin valf kontrolü yerel uygulamayı kullanır. Yeni projede yalnızca bir TCA9555 sürücüsü bırakın.
- Piston konumunun güncel kaynağı TMAG5173'tür; eski analog Hall ve ADS1115 yolları yalnızca geriye dönük uyumluluktur.
- PWM, DRV8243 güvenlik mantığı, I2C mutex ve valf/piston eşleme tablosu donanım erişim katmanında merkezi olmalıdır.
- GPIO2/GPIO10 eski Hall tanımları ile güncel NTC ADC girişleri arasında çakışır. Yeni tasarımda yalnızca gerçek kullanılan işlevleri tanımlayın.

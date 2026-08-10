# 🚀 Kitronic DSG Tester GUI - Build Talimatları

Bu dosya, Kitronic DSG Tester GUI uygulamasını **çalıştırılabilir .exe** ve **kurulum paketi** olarak derleme talimatlarını içerir.

---

## 📋 Gereksinimler

### 1. Python 3.8+ 
- İndirin: https://www.python.org/downloads/
- Kurulum sırasında **"Add Python to PATH"** seçeneğini işaretleyin

### 2. Inno Setup (sadece kurulum paketi için)
- İndirin: https://jrsoftware.org/isdl.php
- Kurulum sırasında varsayılan ayarları kullanın

---

## 🔨 Yöntem 1: Tek Tıkla .exe Oluşturma (ÖNERİLEN)

### Windows:
1. `build.bat` dosyasına **çift tıklayın**
2. Script otomatik olarak:
   - Sanal ortam oluşturur
   - Bağımlılıkları yükler
   - .exe dosyasını derler
3. Tamamlandığında `dist\Kitronic_DSG_Tester.exe` dosyası oluşur

**Süre:** ~3-5 dakika (internet hızına bağlı)

---

## 📦 Yöntem 2: Kurulum Paketi Oluşturma

### Adım 1: .exe Oluşturun
```bash
build.bat
```

### Adım 2: Installer Oluşturun
```bash
build_installer.bat
```

Kurulum dosyası: `installer_output\Kitronic_DSG_Tester_Setup_v1.0.exe`

---

## 🖥️ Manuel Build (İleri Düzey)

### 1. Sanal Ortam Oluşturma
```bash
python -m venv venv
venv\Scripts\activate
```

### 2. Bağımlılıkları Yükleme
```bash
pip install -r requirements.txt
pip install pyinstaller
```

### 3. .exe Derleme
```bash
pyinstaller --clean build_exe.spec
```

### 4. Kurulum Paketi (Opsiyonel)
```bash
iscc setup.iss
```

---

## 📁 Çıktı Dosyaları

```
📂 Kitronic_Tester_FreeRTOS/
├── 📄 build.bat                    # Tek tıkla .exe build
├── 📄 build_installer.bat          # Kurulum paketi build
├── 📄 build_exe.spec               # PyInstaller ayarları
├── 📄 setup.iss                    # Inno Setup ayarları
├── 📄 requirements.txt             # Python bağımlılıkları
│
├── 📂 dist/
│   └── 📄 Kitronic_DSG_Tester.exe  # ✅ ÇALIŞAN UYGULAMA
│
└── 📂 installer_output/
    └── 📄 Kitronic_DSG_Tester_Setup_v1.0.exe  # ✅ KURULUM PAKETİ
```

---

## ✅ Başka Bilgisayarda Kullanım

### .exe Dosyası (Portable)
1. `dist\Kitronic_DSG_Tester.exe` dosyasını kopyalayın
2. **Çift tıklayın** - çalışır!
3. Python kurulu olmasına gerek **YOK**

### Kurulum Paketi
1. `Kitronic_DSG_Tester_Setup_v1.0.exe` dosyasını kopyalayın
2. Çalıştırın ve kurulum yapın
3. Başlat menüsünden veya masaüstü kısayolundan açın

---

## 🐛 Sorun Giderme

### "Python bulunamadı" Hatası
- Python'u PATH'e ekleyin veya yeniden kurun
- Komut: `python --version` çalışmalı

### "Inno Setup bulunamadı" Hatası
- Inno Setup'ı kurun
- Bilgisayarı yeniden başlatın

### .exe Çalışmıyor
- Windows Defender/Antivirus kontrol edin
- `dist` klasöründeki tüm dosyaları kopyalayın

### .exe Boyutu Büyük (~100-150 MB)
- Normal! PyQt5 ve bağımlılıkları içeriyor
- UPX ile sıkıştırılmış hali

---

## 📝 Özelleştirme

### Uygulama İkonu Ekleme
1. `icon.ico` dosyası oluşturun (256x256 px)
2. `build_exe.spec` dosyasında:
   ```python
   icon='icon.ico',
   ```

### Sürüm Numarası Değiştirme
1. `setup.iss` dosyasında:
   ```
   #define MyAppVersion "1.0"
   ```

---

## 📞 Destek

Sorun yaşarsanız:
1. `build.log` dosyasını kontrol edin
2. Terminal çıktısını kaydedin
3. Hata mesajını paylaşın

---

**Son Güncelleme:** 6 Ocak 2026

# EGE ROV — Uygulama Kılavuzu

> **Repo:** `ege-row/rov/`  
> **Hedef Platform:** NVIDIA Jetson Orin Nano (aarch64, Ubuntu 20.04 JetPack)

---

## 1. Kurulum (Jetson'da bir kez)

```bash
cd ~/ege-row/rov
pip3 install -r requirements.txt
```

> `opencv-python` Jetson'da GPU ivmeli değildir; tam CUDA desteği için
> kaynak kodundan derleme gerekir (`jetson-containers` projesi).
> Havuz testleri için mevcut sürüm yeterlidir.

---

## 2. İlk Açılış ve Donanım Kontrolü

### 2a. I2C cihazlarını kontrol et
```bash
sudo i2cdetect -y 7    # veya 8 — i2cdetect -l ile bul
```
Beklenen:
- `0x0c` → AK8963 manyetometre (ilk `Mpu9250()` sonrası görünür)
- `0x40` → PCA9685 PWM sürücü
- `0x68` → MPU-9250 IMU
- `0x76` → MS5837 derinlik sensörü

### 2b. `config.py` ayarlarını güncelle
```python
SIM_MODE = False         # ZORUNLU — donanımda
I2C_BUS  = 7            # i2cdetect çıktısına göre
FLUID_DENSITY = 997     # havuz (997) veya deniz (1025)
```

### 2c. IMU ve ESC Kalibrasyonu (İlk Kullanımdan Önce Bir Kez)

#### 1) IMU Kalibrasyonu
```bash
cd ege-row/rov
python3 calibrate_imu.py
```
Adımlar:
1. ROV sabit zeminde → ENTER → 5 sn bekleme (jiroskop bias)
2. ROV elinle yavaşça her yöne çevir (15 sn) → manyetometre kalibrasyonu
3. `config.py` otomatik güncellenir, eski hali `config.py.bak` olarak yedeklenir.

#### 2) ESC Nötr & Gaz Kalibrasyonu (Motor Titremesi ve Bip Seslerini Kesme)
Eğer ESC'ler kendi kendine bip sesi çıkarıyor, titriyor veya nötr sinyali 1700us/1670us gibi kayma yapıyorsa:
```bash
python3 calibrate_escs.py
```
Adımlar:
1. Pervaneleri sökün ve ESC güç fişini çekin.
2. Komutu çalıştırın → Jetson 2000us (Max Gaz) gönderecektir.
3. ESC bataryasını takın → Yüksek bip seslerini duyunca sistem otomatik olarak 1000us (Min Gaz) ve 1500us (Sabit Nötr) göndererek ESC EEPROM hafızasına tam 1500us nötr ayarını kilitleyecektir.

### 2d. Motor testi (pervane takmadan)
```bash
python3 main.py --test-motors
```
Her motor 2 sn %10 güçte döner. Doğru yön yoksa `config.py`'de `MOTOR_DIRECTION` düzelt.

### 2e. E-stop GPIO testi
```bash
python3 main.py --estop-test
```
`e` + Enter → yazılımsal tetikleme. Tüm motorlar durmalı. Donanımda mıknatısla test et.

---

## 3. Video Gösterimi Görevi (Görev Ön Koşulu)

### Çalıştırma
```bash
python3 main.py --mission video
```

### Görev akışı
```
COUNTDOWN (10sn) → DIVE → STRAIGHT1 (15sn) → TURN1 (+90°)
  → STRAIGHT2 (15sn) → CIRCLE (360°) → STRAIGHT3 (15sn)
  → TURN2 (+90°) → STRAIGHT4 (15sn) → FINISH
```

### Kontrol edilecekler
- Araç su altında kalıyor mu? (`target_depth_m = 0.6`)
- Başlangıç alanına (1×1m) dönüyor mu? → `run_sim.py` ile simüle et
- E-stop gösterimi: mıknatısı yaklaştır → tüm motorlar anında duruyor mu?

### Simülasyon (havuz öncesi test)
```bash
python3 run_sim.py
```
ASCII rota izini ve başarı/başarısız çıktı verir.

---

## 4. Yer İstasyonu Bağlantısı (Web GCS)

### 4a. Web Arayüzüne Erişim
`main.py` çalıştırıldığı anda **Web Tabanlı Yer İstasyonu (GCS)** otomatik olarak başlar.
Operatör laptopunda veya tabletinde herhangi bir web tarayıcısından (Chrome/Edge/Firefox) bağlanın:

```text
http://192.168.1.10:8000/
```

### 4b. Web GCS Özellikleri
- **Canlı FPV Kamera Akışı:** `/video_feed` üzerinden düşük gecikmeli MJPEG canlı yayın ve AR nişangah göstergeleri.
- **HUD PFD Göstergesi:** HTML5 Canvas ile 60FPS akıcı Yunuslama/Yatış (Pitch/Roll) yapay ufuk, dinamik Pusula (Heading) bandı ve Derinlik ölçeği.
- **6 Motor İtki Monitörü:** V_FL, V_FR, V_RL, V_RR, H_L, H_R motor çıktılarının canli % grafik çubukları.
- **Canlı PID Ayarlayıcı:** Derinlik, Heading, Roll ve Pitch PID katsayılarını ($K_p, K_i, K_d$) su altındayken kod yeniden başlatılmadan tarayıcıdan anlık güncelleme.
- **Acil Durum & Görev Kontrolleri:** Büyük Kırmızı **EMERGENCY ABORT** butonu, ARM/DISARM düğmesi, CLS3860MED Mini ROV vinç bırakma/çekme butonları.
- **Klavye Teleop Sürüşü:** Tarayıcı açıkken WASD / IJKL tuşlarıyla manuel sürüş desteği.

### 4c. Alternatif Terminal İstemcisi
İstenirse eski hafif terminal istemcisi de kullanılabilir:
```bash
python3 -m comms.client 192.168.1.10
```
| `u/o` | Yukarı / Aşağı heave teleop |
| `[space]` | Tüm eksenler nötr |
| `t` | Teleop'tan çık, göreve dön |
| `q` | Çıkış (abort gönderir) |

### Canlı FPV video
```bash
# Laptopta (Jetson'da otomatik başlar main.py ile):
python3 -m comms.video_stream --receive
```

---

## 5. Görev 1 — Hat Takibi + Mini ROV

### Ön koşullar
- Kamera çalışıyor: `CSI_CAMERA = True` (ya da `False` USB için)
- Hat rengi: beyaz (config.py `LINE_HSV_*` ayarları)
- Vinç servo PCA9685 CH6'ya bağlı

### Hat rengi kalibrasyonu
```bash
# Laptopta görüntü alarak HSV değerini bul:
python3 -c "
import cv2, numpy as np
cap = cv2.VideoCapture(0)
while True:
    _, f = cap.read()
    hsv = cv2.cvtColor(f, cv2.COLOR_BGR2HSV)
    # İmleç konumundaki HSV değerini yazdır
    h, w = f.shape[:2]
    print(hsv[h//2, w//2])  # merkez piksel
    cv2.imshow('f', f)
    if cv2.waitKey(1) == 27: break
"
```
Elde edilen H, S, V değerlerine göre `config.py`'de:
```python
LINE_HSV_LOW  = (H-15, S-30, V-40)
LINE_HSV_HIGH = (H+15, S+30, V+40)
```

### Çalıştırma
```bash
python3 main.py --mission line
```

### Mini ROV geri çekme sinyali (laptopta)
Mini ROV görevini tamamladığında yer istasyonundan:
```bash
# client.py üzerinden:
# 't' tuşuyla teleop'tan çık yoksa görev bağlamında:
# Veya doğrudan:
python3 -c "
from comms.client import CommsClient
c = CommsClient()
c.connect()
import json
c._send({'cmd': 'minrov_back'})
"
```

---

## 6. Görev 2 — Otonom Navigasyon

### Ön koşullar
- GPS: Quectel L80-R → Jetson Pin 8/10 (UART1, `/dev/ttyTHS1`)
- Ping Sonar: USB → `/dev/ttyUSB0`
- GPS anteni su üstünde açık gökyüzü görmeli

### GPS port doğrulama
```bash
ls /dev/ttyTHS*    # UART portları
ls /dev/ttyUSB*    # USB portları
```

### Test: GPS fix var mı?
```bash
python3 -c "
from sensors.gps import GPS
import time
g = GPS()
g.start()
time.sleep(20)
print(g.fix)
"
```

### Test: Sonar çalışıyor mu?
```bash
python3 -c "
from sensors.ping_sonar import PingSonar
import time
s = PingSonar()
s.start()
for _ in range(10):
    time.sleep(0.5)
    print(s.measurement)
"
```

### Çalıştırma
```bash
python3 main.py --mission nav
```

### Hedef mesafe ayarı
`config.py`'de `_target_dist_m` parametresini parkur boyutuna göre güncelle
(`nav_mission.py` içinde `self._target_dist_m = 20.0`).

---

## 7. Log Dosyaları

Tüm görev logları `logs/` klasörüne CSV olarak kaydedilir:
```
logs/video_demo_20260723_153000.csv
logs/line_follow_20260723_160000.csv
logs/nav_mission_20260723_163000.csv
```

Sütunlar: `t, type, state, heading, target_heading, depth, target_depth, roll, pitch, yaw_rate, surge, yaw, heave, note`

PID optimizasyonu için Excel veya Python/Pandas ile analiz et:
```python
import pandas as pd
df = pd.read_csv("logs/video_demo_*.csv")
df[df.type == "DATA"].plot(x="t", y=["heading", "target_heading"])
```

---

## 8. Proje Dosya Yapısı

```
ege-row/rov/
├── config.py                ← TÜM parametreler burada
├── main.py                  ← Giriş noktası (--mission, --test-motors, vb.)
├── calibrate_imu.py         ← IMU kalibrasyonu
├── test_motors.py           ← Motor yön testi
├── kanal_test.py            ← Ham kanal PWM testi
├── run_sim.py               ← Video demo simülasyonu
│
├── hal/
│   ├── thrusters.py         ← ESC PWM sürücü
│   ├── estop.py             ← E-stop GPIO izleyicisi
│   └── winch.py             ← CLS3860MED servo (Mini ROV vinç)
│
├── control/
│   ├── pid.py               ← Anti-windup PID
│   ├── mixer.py             ← 6-motor thrust mixer
│   ├── stabilizer.py        ← Depth/heading/roll/pitch PID katmanı
│   └── dead_reckoning.py    ← Heading+hız XY entegrasyonu
│
├── sensors/
│   ├── imu.py               ← MPU-9250 + AK8963 tilt-compensated heading
│   ├── depth.py             ← MS5837-30BA derinlik sensörü
│   ├── camera.py            ← CSI/USB kamera yöneticisi (GStreamer)
│   ├── gps.py               ← Quectel L80-R NMEA parser
│   └── ping_sonar.py        ← Blue Robotics Ping sonar
│
├── vision/
│   ├── enhance.py           ← LAB/CLAHE/USM/HSV iyileştirme
│   ├── line_tracker.py      ← HSV hat algılama + merkez hatası
│   └── pipe_aligner.py      ← Boru girişi Hough Circle hizalama
│
├── missions/
│   ├── video_demo.py        ← Video gösterimi (4 düz + daire)
│   ├── line_follow.py       ← Görev 1: hat takibi + Mini ROV
│   └── nav_mission.py       ← Görev 2: GPS/sonar şamandıra orbit
│
├── comms/
│   ├── server.py            ← TCP JSON komut sunucusu (Jetson'da)
│   ├── client.py            ← Klavye GCS istemcisi (laptopta)
│   └── video_stream.py      ← UDP JPEG video akışı
│
├── sim/
│   └── simulator.py         ← 3-DOF ROV simülatörü
│
├── utils/
│   └── logger.py            ← CSV görev logger
│
├── tests/
│   └── test01.py            ← Temel kanal duman testi
│
├── requirements.txt
├── HARDWARE_SETUP.md        ← Fiziksel kablolama kılavuzu
└── IMPLEMENTATION_GUIDE.md  ← Bu dosya
```

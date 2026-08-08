# EGE ROV — Durum Değerlendirmesi & Video Görevi Eylem Planı
**Tarih: 1 Ağustos 2026 — Yarışma günü**

---

## Genel Durum Özeti

Sistem çalışıyor ama stabilize değil. Motorlar çalışıyor, sensörlerden veri geliyor,
ama kalibrasyon sorunları kritik düzeyde. Bugün video görevi tamamlanmalı.

---

## 🔴 Kritik Sorunlar (Öncelik Sırası)

### 1. MOTOR_DIRECTION Tablosu — En Kritik Bug

**Dosya:** [config.py](file:///c:/Users/burak/Desktop/egerov/ege-row/rov/config.py#L62-L65)

```python
MOTOR_DIRECTION = {
    "V_FL": -1, "V_FR": -1, "V_RL": -1, "V_RR": -1,
    "H_L": -1, "H_R": -1,
}
```

> [!CAUTION]
> config.py'nin kendi yorumuna göre (L54-L61) bu tablo **yanlış!** 58.1 Hz PWM hatası döneminde
> ölçüldü. Frekans düzeltildi, ama tablo hâlâ hepsini -1 yapıyor. Bu demek oluyor ki:
> - Derinlik PID'i "dal" komutu verince ROV **yükseliyor**
> - Heading PID "sağa dön" verince ROV **sola dönüyor**
> - Stabilizasyon çalışmak yerine **karşı yönde salınım yaratıyor**
>
> **Bu sorun giderilmeden hiçbir kalibrasyon işe yaramaz.**

**Çözüm:** `kanal_test.py` ile her motoru tek tek test et. Gerçekten ters dönen motorlara -1 bırak, doğru dönenlerden -1 kaldır.

---

### 2. IMU Kalibrasyon Scripti — Yetersiz (Fiziksel Montaj Problemi)

**Dosya:** [calibrate_imu.py](file:///c:/Users/burak/Desktop/egerov/ege-row/rov/calibration/calibrate_imu.py)

Mevcut script **cihazın yatay durmasını varsayıyor**. ROV şu an biraz yukarı bakıyor
(pitch ≠ 0). Bu şu anlama geliyor:

**İvmeölçer (Accel) Problemi:**
- `calibrate_accel()` Z ekseninde 1g varsayıyor (`bias_z = avg_z - 1.0`)
- ROV eğikse Z'de 1g değil, cos(θ) * 1g var → bias yanlış hesaplanıyor
- Roll/Pitch referansı bozuk → stabilizasyon "düz" sanarak eğik kalibre ediyor

**Pusula Tilt Compensation Problemi:**
- `Orientation._mag_heading()` roll/pitch'i tilt-compensation için kullanıyor
- Eğer roll/pitch kalibrasyonu yanlışsa → heading hesabı da sapıyor
- ROV su altında "düz" duruyor ama kod bunu yanlış tanımlıyor

**Gyro Bias Problemi:**
- Gyro kalibrasyonu iyiydi ama IMU fiziksel açısı bilgiyi kirletiyor
- Gyro bias aslında koordinat sisteminden bağımsız — bu kısım görece sorunsuz

**Çözüm:** Kalibrasyonu "cihazın açısından bağımsız" hale getir.

---

### 3. PID Değerleri — Hiç Test Edilmedi (Gerçek Donanımda)

**Dosya:** [config.py](file:///c:/Users/burak/Desktop/egerov/ege-row/rov/config.py#L69-L72)

```python
PID_DEPTH   = dict(kp=2.0,  ki=0.1,  kd=0.8,  out_limit=1.0, i_limit=0.3)
PID_HEADING = dict(kp=0.02, ki=0.0,  kd=0.008, out_limit=0.6, i_limit=0.2)
PID_ROLL    = dict(kp=0.01, ki=0.0,  kd=0.004, out_limit=0.3, i_limit=0.1)
PID_PITCH   = dict(kp=0.01, ki=0.0,  kd=0.004, out_limit=0.3, i_limit=0.1)
```

Bu değerler teorik başlangıç değerleri. Gerçek tankta test edilmemiş.
MOTOR_DIRECTION düzeltilmeden test anlamsız.

---

### 4. Mixer Yönlendirme Tutarlılığı

**Dosya:** [mixer.py](file:///c:/Users/burak/Desktop/egerov/ege-row/rov/control/mixer.py#L21-L29)

Mixer mantığı doğru ama şu soru yanıtsız:
- `H_L: surge - yaw` ile `H_R: surge + yaw` → yaw+ = sağa mı sola mı?
- Bu MOTOR_DIRECTION ile birleşince son yön belirsiz

MOTOR_DIRECTION düzeltmesi sırasında mixer konvansiyonunu da doğrula.

---

### 5. Video Demo Görevi — Bazı Riskler

**Dosya:** [video_demo.py](file:///c:/Users/burak/Desktop/egerov/ege-row/rov/missions/video_demo.py)

- **Daire ölçümü:** `_circle_acc += abs(yaw_rate) * dt` — yaw_rate gürültülü olursa daire erkenden bitebilir veya hiç bitmeyebilir
- **TURN timeout:** 15 saniye bekleyip geçiyor — heading PID yanlışsa bu hep timeout'la geçecek
- **FINISH durumu:** `mix(0, 0, 0)` ama sonra `stab.compute()` ile heave uygulanıyor — bu birbirini eziyor (bug: `self.thr.command(mix(0, 0, 0))` satırı anlamsız)

---

## 📋 Bugünkü Eylem Planı (Öncelik Sırasına Göre)

### ⏱️ ADIM 1 — Motor Yönlerini Düzelt (~30 dk, havuzdan önce, kuru)
Önce pervane çıkar, sonra:
```bash
cd /home/burak/ege-row/rov
python3 tests/kanal_test.py
```
- Kanal 0,1,2,3,4,5 tek tek test et
- Her kanal için: "ileri" komutunda pervane hangi yönde döndü? Not al.
- Doğru dönenlerde MOTOR_DIRECTION'ı +1 yap
- Sadece gerçekten ters dönenlerde -1 bırak

**Beklenen sonuç:** V_FL/V_FR/V_RL/V_RR motorların tümü için "ileri (heave+)" = ROV'u aşağı iter olmalı (daha ağır = batma eğilimli tasarım için aksi de doğru olabilir — pervane yönüne bak).

---

### ⏱️ ADIM 2 — IMU Kalibrasyonu (Cihaz Açısından Bağımsız) (~20 dk)

Mevcut `calibrate_imu.py`'yi geliştir:

**a) Gyro Bias:** Mevcut yöntem yeterli — ROV hareketsizken 5 sn ölç.

**b) Accel Bias (Geliştirilmiş):**
- ROV'u tutmaya gerek yok. Matematiksel düzeltme yap:
- `ax, ay, az` ölç → `total_g = sqrt(ax² + ay² + az²)` → gerçek 1g'den sapma = `total_g - 1.0`
- Bu sayede ROV eğik duru ş ta bile bias hesaplanabilir
- Yeni kalibrasyon: ACCEL_BIAS, cihazın koordinat sisteminde değil, büyüklük olarak saklanır

**c) Pitch/Roll Referansı (Önemli):**
- Gerçek "düz" duruşu tanımla: IMU'dan okunan mevcut ax/ay/az değerleri ROV'un "doğal" duruşu
- Bu değerleri `MOUNT_PITCH_DEG` ve `MOUNT_ROLL_DEG` olarak config'e ekle
- `Orientation.update()` bu offset'leri acc_roll/acc_pitch'e uygula

**d) Mag Kalibrasyon:** Mevcut yöntem yeterli (her yöne çevir).

---

### ⏱️ ADIM 3 — Basit IMU Offset Workaround (Hızlı Çözüm) (~10 dk)

ADIM 2 için zaman yoksa bu daha hızlı:
1. `python3 tests/test_imu.py` çalıştır
2. ROV havada hareketsizken pitch/roll değerlerini oku
3. Örnek: pitch = +15°, roll = +3° çıkıyorsa
4. `stabilizer.py`'de PID hesaplamaya offset ekle:
   - `roll_error = 0 - (ori.roll - MOUNT_ROLL_DEG)` 
   - `pitch_error = 0 - (ori.pitch - MOUNT_PITCH_DEG)`
5. Bu değerleri config'e `MOUNT_PITCH_DEG = 15.0` ve `MOUNT_ROLL_DEG = 3.0` olarak ekle

---

### ⏱️ ADIM 4 — Derinlik PID Kalibrasyonu (Havuz, ~30 dk)

MOTOR_DIRECTION düzeltilmeden yapmayın.

```bash
python3 tests/pid_test_cal.py  # Seçim: 1 (Derinlik)
```

Prosedür:
1. `ki 0.0`, `kd 0.0` yap
2. `kp 0.5` ile başla, `step 0.3` (30 cm hedef)
3. Sistemi gözlemle: ROV dengeleniyor mu? Aşım var mı?
4. Kp'yi kademeli artır: 0.5 → 1.0 → 1.5 → 2.0
5. Salınım başlarsa Kd ekle: `kd 0.3` → `kd 0.5` → `kd 0.8`
6. Kalıcı hata varsa: `ki 0.05` → `ki 0.1`
7. Tatmin olunca `save`

**Başlangıç önerisi:** kp=1.5, ki=0.05, kd=0.5 (mevcut değerler biraz agresif)

---

### ⏱️ ADIM 5 — Heading PID (Havuz, ~20 dk)

```bash
python3 tests/pid_test_cal.py  # Seçim: 2 (Heading)
```

Heading PID girişi derece → çıkışı -1..+1 yaw komutu.
Mevcut kp=0.02 çok küçük. Tipik ROV değerleri:
- kp=0.008..0.015, ki=0.0, kd=0.002..0.005

Prosedür: `step 45` ile 45° dönüş iste, salınım yoksa kp artır.

---

### ⏱️ ADIM 6 — Hız Kalibrasyonu (straight_time vs mesafe)

config.py'de `straight_time_s = 16.0` sn @ `cruise_throttle = 0.35`.
Bu sürenin kaç metreye karşılık geldiğini ölç:
- ROV'u havuza koy, `cruise_throttle = 0.35` ile 16 sn sürdür
- Kaç metre ilerlediğini ölç
- Dikdörtgen iz çiziyorsa bu 4 köşeli olmalı (eşit kenar)

---

### ⏱️ ADIM 7 — Video Demo Son Kontrol

```bash
python3 video_main.py --check  # Önce kalibrasyon kontrolü
python3 video_main.py --sim    # Simülasyonda bir tur
python3 video_main.py          # Gerçek çalıştırma
```

---

## 🔧 Kod Düzeltmeleri Yapılacaklar

### A. calibrate_imu.py — Açı Bağımsız Accel Kalibrasyonu

`calibrate_accel()` fonksiyonunu değiştir:
- Z ekseninden 1g çıkarmak yerine, **vektör büyüklüğünü** normalize et
- `MOUNT_PITCH_DEG` ve `MOUNT_ROLL_DEG` değerlerini hesapla ve config'e yaz

### B. sensors/imu.py — Mount Offset Desteği

`Orientation.update()` içinde:
```python
# config'den MOUNT_PITCH_DEG ve MOUNT_ROLL_DEG oku
acc_roll  = acc_roll  - MOUNT_ROLL_DEG
acc_pitch = acc_pitch - MOUNT_PITCH_DEG
```

### C. missions/video_demo.py — FINISH Bug Düzeltmesi

```python
# Satır 145 — bu satırı kaldır (stabilizer'dan gelen heave'i eziyor):
# self.thr.command(mix(0, 0, 0))  # BUG: bu satır kaldırılmalı
axes = self.stab.compute(surge=0.0)  # derinliği tutmaya devam
self._apply(axes)
```

### D. config.py — Eksik Parametreler Ekle

```python
MOUNT_PITCH_DEG = 0.0   # IMU montaj pitch açısı (calibrate_imu.py ile doldurulacak)
MOUNT_ROLL_DEG  = 0.0   # IMU montaj roll açısı
```

---

## ❓ Açık Sorular (Senden Yanıt Gerekiyor)

> [!IMPORTANT]
> 1. **Motor yönleri:** `kanal_test.py` ile her motoru test ettin mi? Hangileri gerçekten ters dönüyor?
> 2. **ROV'un fiziksel montajı:** IMU (MPU-9250) ROV içinde hangi yöne bakıyor? X ekseni ileri mi?
> 3. **Havuz derinliği:** Bugün test yapacağınız havuz ne kadar derin? 0.6m hedef derinlik yeterli mi?
> 4. **Süre kısıtı:** Bugün kaç saatin var? Önceliği nasıl sıralayalım — "çalışır bir demo" mu yoksa "tam kalibrasyon" mu?
> 5. **Daire görevi:** Yarışma şartnamesinde dairenin en az 1m çap olması gerekiyor. `circle_throttle=0.25` ile bu sağlanıyor mu?

---

## 📊 Risk Matrisi

| Risk | Olasılık | Etki | Azaltma |
|------|-----------|------|---------|
| MOTOR_DIRECTION yanlış → ROV kontrol edilemiyor | **YÜKSEK** | Kritik | kanal_test.py ile düzelt |
| IMU pitch offset → stabilizasyon hatalı | **YÜKSEK** | Yüksek | Mount offset ekle |
| PID değerleri → aşım / salınım | Orta | Orta | Havuzda kademeli ayarla |
| Daire ≥1m çap sağlanamıyor | Orta | Yüksek | circle_throttle artır |
| Derinlik timeout (10 sn) → görev abortlanıyor | Düşük | Yüksek | dive_timeout_s artır |
| Heading drift → dikdörtgen bozuluyor | Orta | Orta | HEADING_FILTER_ALPHA ayarla |

---

## 🏁 Minimum Çalışan Demo İçin Şart Koşullar

Yarışma için mutlak minimum:
1. ✅ MOTOR_DIRECTION doğru → ROV komutlara doğru tepki veriyor
2. ✅ Derinlik PID çalışıyor → ROV 0.6m'de tutuluyor
3. ✅ Heading PID kabul edilebilir → 90° dönüş ±15° toleransla tamamlanıyor
4. ⚠️ Roll/Pitch stabilizasyonu → ROV ciddi yatmıyorsa PID olmadan da geçebilir (kp çok küçük)
5. ✅ straight_time_s yeterince uzun → 15+ saniye şartı karşılanıyor

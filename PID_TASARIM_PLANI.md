# EGE ROV — PID Tasarım Planı
**Kapsam:** Otonom video gösterimi görevi (şartname 2.4.3.3)
**Durum:** Havuza ~3 saat var. Bu doküman havuza kadar yapılacakları ve havuzda izlenecek ayar protokolünü tanımlar.

---

## 0. Ana karar

**PID katsayısı ayarlamadan önce geçilmesi gereken 3 kapı var.** Loglar, mevcut katsayıların
neden "işe yaramadığını" değil, **kontrol döngüsünün hiç anlamlı veri görmediğini** gösteriyor.
Bu kapılar geçilmeden havuzda harcanan her dakika boşa gider — çünkü ölçtüğün her tepki
sensör hatasının tepkisi olur, aracın değil.

| Kapı | Konu | Süre | Havuz öncesi zorunlu mu? |
|---|---|---|---|
| **K1** | IMU kalibrasyonu (ACCEL_BIAS geçersiz) | 20 dk | **EVET** |
| **K2** | Kontrol döngüsü 7.3 Hz → ≥30 Hz | 30 dk | **EVET** |
| **K3** | MOTOR_DIRECTION işaret doğrulaması | 20 dk | **EVET** (işaret hatası = pozitif geri besleme = diverjans) |

Kalan süre PID mimarisinin yeniden yazımına ve havuz protokolüne ayrılır.

---

## 1. Teşhis — kanıtlarla

### B1 — `ACCEL_BIAS` fiziksel olarak imkânsız; roll/pitch tamamen sahte  🔴 KRİTİK

```python
ACCEL_BIAS = (2.0, -2.0, -0.296)   # config.py
```

MPU-9250 varsayılan ±2 g aralığında çalışıyor (`/16384.0` ölçekleme). **2.0 g'lik bir sapma
sensörün tüm ölçüm aralığı kadar** — yani ölçüm doyuma girmişken (araç sallanırken)
kalibre edilmiş. Fiziksel olarak bir bias değeri olamaz.

**Kanıt (sayısal olarak birebir tutuyor):** araç düz dururken `(0, 0, 1) g` okuması bu bias ile
düzeltilirse tamamlayıcı filtrenin oturacağı sahte açılar:

```
acc_roll  = atan2(ay, az)             = 57.06°
acc_pitch = atan2(-ax, hypot(ay,az))  = 40.00°
```

Gerçek donanım logu `logs/video_demo_20260731_121017.csv`:

```
t=22.175  roll=-57.06  pitch=-34.80     ← roll TAM olarak 57.06'ya oturmuş
t=15.403  roll=-28.97  pitch=-17.56     ← oraya doğru düzgün yakınsıyor
```

Rastlantı değil. `roll` değeri sahte referansın virgülden sonraki basamağına kadar aynı.

**Etkileri (zincirleme):**
1. Roll/pitch PID'leri sürekli 57°/40° hata görüyor → 4 dikey motor birbirini iterek boşuna güç yakıyor.
2. `Orientation._mag_heading()` **tilt telafisi için bu sahte roll/pitch'i kullanıyor** →
   pusula okuması sahte açı yakınsarken beraberinde dönüyor. Aynı logda:
   `heading 154.27 → 98.03` (10 sn'de −56°), roll oturdukça sürüklenme duruyor.
   **Yani heading sürüklenmesi ayrı bir arıza değil, B1'in sonucu.** Tek kök neden.

**Düzeltme:** `calibrate_imu.py` yeniden çalıştırılacak — araç **görev pozisyonunda, düz zeminde,
tamamen hareketsiz**. `calibrate_accel()` içine bir doğrulama eklenecek:
`|bias| > 0.3 g` ise kaydetme, hata ver.

---

### B2 — Kontrol döngüsü 50 Hz değil **7.3 Hz**  🔴 KRİTİK

Log örnekleme aralığından (`LOG_EVERY_N = 5`) ölçülen:

| | Örnek arası | Döngü periyodu | Frekans |
|---|---|---|---|
| Simülasyon (`120625.csv`) | 0.103 s | 20.5 ms | **48.8 Hz** ✅ |
| Gerçek donanım (`121017.csv`) | 0.680 s | 136 ms | **7.3 Hz** ❌ |

**Neden:** `Ms5837.read_pressure_mbar()` iki ADC dönüşümü yapıyor, her biri `time.sleep(0.02)`
= **40 ms bloklama**. Ve bu, döngü başına **2–3 kez** çağrılıyor:

1. `Stabilizer.compute()` → `depth_error()` → `read_depth_m()`
2. `VideoDemoMission.step()` DIVE durumunda tekrar `self.stab.depth_error()`
3. `MissionLogger.sample()` içinde bir kez daha `stab.depth.read_depth_m()`

→ 80–120 ms bloklama + `time.sleep(1/LOOP_HZ)`. Ölçülen 136 ms ile tam uyumlu.

**Etkileri:**
- D terimi 7 Hz'de anlamsız (Nyquist ~3.6 Hz, ROV dinamiği bunun içinde).
- `dt` jitter'ı %30+ → `d_filter` her adımda farklı kesim frekansı uyguluyor.
- `SLEW_RATE = 2.0 birim/s` 136 ms'de 0.27 birim adım demek → gaz komutu sıçramalı.
- Kontrol gecikmesi 136 ms → kazanç payı ciddi düşüyor, aynı Kp ile salınım eşiği çok daha erken.

**Düzeltme (3 parça):**
1. **OSR düşür:** `CMD_CONV_D1 = 0x44`, `CMD_CONV_D2 = 0x54` (OSR 1024, 2.28 ms).
   Çözünürlük ~0.2 mbar ≈ **2 mm derinlik** — bize fazlasıyla yeter (hedef tolerans 5 cm).
2. **Sensörleri ayrı thread'e al:** derinlik 20 Hz, IMU 100 Hz kendi thread'lerinde okur,
   son değeri paylaşılan bir `state` nesnesine yazar. Kontrol döngüsü **asla bloklanmaz**,
   sadece son değeri okur.
3. **Döngüde tek okuma:** `Stabilizer.compute()` başında sensörler bir kez örneklenir,
   `self.last_depth` / `self.last_heading` içine yazılır; mission ve logger bu cache'i okur.
4. **Deadline tabanlı zamanlayıcı:** `time.sleep(1/LOOP_HZ)` yerine
   `next_t += dt; time.sleep(max(0, next_t - now))`.

---

### B3 — PWM ölü bandı kontrol otoritesini yiyor  🟠

`PWM_DEADBAND_US = 30`, `PWM_RANGE_US = 470` → normalize ölü bant = **0.0638**.
Bu değerin altındaki her komut motora **sıfır** olarak gidiyor.

| Eksen | Kp | İtki üretmek için gereken min hata |
|---|---|---|
| Heading | 0.02 | **3.19°** (`turn_tol_deg = 5.0`) |
| Roll | 0.01 | **6.38°** |
| Pitch | 0.01 | **6.38°** |

**Sonuç:** roll/pitch PID'leri pratikte hiç çalışmıyor — 6.4°'nin altında hiçbir düzeltme
uygulanmıyor, üstünde ise aniden itki başlıyor (limit-cycle üreten klasik yapı).
Heading'de de son 3.2°'lik yaklaşma ölü — dönüş "tam oturmuyor" davranışının kaynağı bu.

**Düzeltme:** mixer'a **ölü bant telafisi (deadband breakaway)**:
komut sıfır değilse çıktıyı ölü bandın hemen üstüne kaydır.

```
u_out = 0                              , |u| < eps          (gerçekten dur)
u_out = sign(u)*(DB + (1-DB)*|u|)      , |u| >= eps         (ölü bandı atla)
```
`DB = PWM_DEADBAND_US/PWM_RANGE_US = 0.0638`, `eps ≈ 0.01`.
Böylece komut–itki ilişkisi ölü bant boyunca **doğrusal ve sürekli** olur.

---

### B4 — Derinlik yolu: clamp + FF yokluğu + windup  🟠

```python
return max(0.0, (p - surface) * 100.0 / (FLUID_DENSITY * 9.81))   # depth.py
```

- **`max(0.0, ...)` kontrol için zararlı.** Araç referans yüzeyin üstüne çıkarsa derinlik
  0'da kalıyor; PID aşımın büyüdüğünü **göremiyor**, hata sabit `target`'ta donuyor.
  → `read_depth_m()` ham (işaretli) değeri döndürmeli; clamp sadece telemetri gösteriminde.
- **İleri besleme (feed-forward) yok.** ROV pozitif kaldırma kuvvetli (olması gerektiği gibi —
  arıza halinde yüzmeli). Sabit derinlikte durmak için **sürekli** aşağı itki gerekiyor;
  bunu şu an sadece I terimi sağlayabilir, `i_limit = 0.3` ile sınırlı ve yavaş.
  → `heave = FF_hover + PID(...)`. `FF_hover` havuzda 3 dakikada ölçülür (bkz. §6 Adım 2).
- **Windup:** logda `heave = 1.0` 10 sn boyunca doygun. Mevcut anti-windup sadece I'yi
  bağımsız olarak kırpıyor, **çıktı doygunluğuna bakmıyor**.

---

### B5 — `MOTOR_DIRECTION` tablosu güvenilmez  🔴 KRİTİK

`config.py`'nin kendi yorumu bunu zaten işaretlemiş: tablo 58.1 Hz PWM hatası varken
ölçülmüş, o sırada **her motor her testte geri dönüyordu** → hepsine `-1` yazılmış.
Frekans düzeltildi ama tablo düzeltilmedi.

**PID açısından anlamı:** tek bir işaret hatası, o eksende negatif geri beslemeyi
**pozitif geri beslemeye** çevirir. PID hatayı düzeltmek yerine büyütür ve araç doyuma gider.
Hiçbir Kp/Ki/Kd değeri bunu kurtaramaz.

**Düzeltme (havuz öncesi zorunlu, kuru):** `kanal_test.py` ile her motor tek tek,
doğru frekansta sürülecek; pervane dönüş yönü ve ürettiği itki yönü elle doğrulanacak.
Ardından eksen testi: `mix(surge=+0.3,0,0)` → 2 yatay motor ileri; `mix(0,0,heave=+0.3)`
→ 4 dikey motor **aşağı** iter.

---

### B6 — Daire açı sayacında `abs()` hatası  🟠

```python
self._circle_acc += abs(self.stab.ori.yaw_rate) * dt     # video_demo.py
```

`abs()` yüzünden **jiroskop gürültüsü pozitif olarak birikiyor.** Araç hiç dönmese bile
~0.5 dps RMS gürültü, 40 sn'lik dairede **~20° sahte açı** üretir. Ayrıca araç ters yöne
sapsa bile sayaç ileri gider.

**Düzeltme:** işaretli integrasyon + ölü bant:
```python
w = self.stab.ori.yaw_rate
if abs(w) > GYRO_NOISE_DPS:      # ~1.0 dps
    self._circle_acc += w * dt   # işaretli
```
Ve `CIRCLE` durumuna girişte `_circle_acc = 0.0`, `_prev_t = None` sıfırlanacak.

---

### B7 — `PID` sınıfının yapısal eksikleri  🟡

| Sorun | Etki | Çözüm |
|---|---|---|
| D terimi **hata** üzerinden | Hedef değişiminde türev tepesi (setpoint kick) — her dönüşte ani gaz | D'yi **ölçüm** üzerinden al |
| `d_filter` örnek başına sabit | Değişken `dt`'de kesim frekansı kayar | Zaman sabitli filtre: `a = dt/(tau+dt)` |
| Anti-windup çıktı doygunluğundan bağımsız | Doygunlukta I birikmeye devam eder | **Conditional integration** + back-calculation |
| İleri besleme (FF) girişi yok | Derinlik/hover için şart | `update(error, ff=0.0)` |
| Ki integral birikiminin **içinde** | Canlı ayar sırasında geçmiş birikim eski kazançta kalır | Ki'yi çıkışta uygula: `out = kp*e + ki*I + ...` |
| Türev/çıkış hız limiti yok | Motor komutu sıçraması | Opsiyonel çıkış slew (zaten HAL'de var) |

---

## 2. Tasarım hedefleri (kabul kriterleri)

Bunlar havuzdaki "geçti/kaldı" ölçütleri. Log CSV'sinden otomatik hesaplanacak.

| # | Eksen / davranış | Hedef | Ölçüt |
|---|---|---|---|
| H1 | Kontrol döngüsü | **≥ 30 Hz**, jitter < %20 | log `dt` sütunu |
| H2 | Derinlik tutma | 0.60 m, **±5 cm RMS** | 20 sn sabit seyirde |
| H3 | Derinlik adım yanıtı | aşım < 15 cm, yerleşme < 6 s | 0 → 0.6 m adım |
| H4 | **Yüzeye çıkmama** | derinlik hiçbir an < 0.25 m | tüm görev boyunca — eleme kriteri |
| H5 | Heading tutma (düz seyir) | **±3° RMS**, kalıcı sapma < 2° | 16 sn düz segment |
| H6 | 90° dönüş | süre < 6 s, aşım < 8°, ±2° yerleşme | TURN1/TURN2 |
| H7 | Roll / pitch | \|açı\| < 8° | tüm görev |
| H8 | Daire | 360° ± 10°, çap ≥ **1.2 m** | jiroskop toplamı + video |
| H9 | Rota kapanması | bitiş noktası başlangıç 1×1 m alanı içinde | video |

---

## 3. Kontrol mimarisi (hedef yapı)

```
 ┌───────────────┐   100 Hz thread      ┌──────────────────┐
 │ MPU-9250      │─────────────────────▶│                  │
 └───────────────┘                      │  RovState        │
 ┌───────────────┐    20 Hz thread      │  (paylaşılan,    │
 │ MS5837        │─────────────────────▶│   kilitli)       │
 └───────────────┘                      │  heading, roll,  │
                                        │  pitch, yaw_rate,│
                                        │  depth, d_depth  │
                                        └────────┬─────────┘
                                                 │ bloklamayan okuma
                                    ┌────────────▼──────────────┐
                                    │  Stabilizer (50 Hz)       │
                                    │                           │
   hedef derinlik ──▶ FF_hover + PID_depth  ────────▶ heave     │
   hedef heading ──▶ P_heading → ω_hedef                        │
                            └─▶ PI_rate(ω_hedef, gyro) ──▶ yaw  │
   0 ──────────────▶ PD_roll  (ölü bant telafili) ────▶ roll    │
   0 ──────────────▶ PD_pitch (ölü bant telafili) ────▶ pitch   │
   görev ──────────▶ surge (açık çevrim)          ────▶ surge   │
                                    └────────────┬──────────────┘
                                    ┌────────────▼──────────────┐
                                    │  mixer.mix()              │
                                    │  + grup normalizasyonu    │
                                    │  + ÖLÜ BANT TELAFİSİ      │
                                    │  + MOTOR_DIRECTION        │
                                    └────────────┬──────────────┘
                                    ┌────────────▼──────────────┐
                                    │  Thrusters (slew + limit) │
                                    └───────────────────────────┘
```

**Mimari kararlar ve gerekçeleri**

| # | Karar | Gerekçe |
|---|---|---|
| D1 | Sensörler ayrı thread, kontrol döngüsü cache okur | B2 — 7.3 Hz sorununun tek gerçek çözümü |
| D2 | Heading: **kaskad** (dış P → ω_hedef, iç PI → yaw komutu) | Dönüş hızını doğrudan sınırlar (aşım ↓); iç döngü gyroyu **ölçüm olarak** kullanır, türev almaya gerek kalmaz; daire görevi zaten ω hedefi istiyor — aynı iç döngü tekrar kullanılır |
| D3 | Derinlik: `FF_hover + PID`, D terimi **derinlik hızı** üzerinden | Kaldırma kuvvetini I'ye yıkmak yavaş ve windup üretiyor; FF ile I sadece artık hatayı toplar |
| D4 | Anti-windup: çıkış doygunsa ve hata aynı yöndeyse I dondurulur | B4 — 10 sn doygunlukta biriken I aşımın ana kaynağı |
| D5 | D filtresi zaman sabitli (`tau = 0.15 s`) | Değişken dt'de tutarlı kesim frekansı |
| D6 | Ölü bant telafisi mixer'da | B3 — roll/pitch/heading son yaklaşma otoritesi |
| D7 | Her PID canlı ayarlanabilir + tüm iç terimler loglanır | 3 saatlik havuz süresinde deneme sayısı = başarı |
| D8 | Derinlik ham (işaretli) okunur | B4 — aşım görünür olmalı |

---

## 4. Eksen bazlı algoritma tasarımı

### 4.1 Derinlik (heave)

```
e      = d_hedef − d_ölçülen                       (işaretli, clamp YOK)
ḋ      = filtreli derinlik hızı (m/s)
I     ← I + e·dt      [koşullu: çıkış doygun ve e aynı yönde değilse]
u     = FF_hover + Kp·e + Ki·I − Kd·ḋ
heave = clamp(u, −1, +1)
```

- **`FF_hover`**: havuzda ölçülür (§6 Adım 2). Beklenen aralık `0.15 … 0.40`.
- **D terimi neden `−Kd·ḋ`?** Hata türevi yerine ölçüm türevi → hedef değişiminde sıçrama yok
  ve fiziksel anlamı doğrudan **sönümleme** (dikey hıza karşı kuvvet).
- **`ḋ` nasıl:** ham türev + 1. derece LPF (`tau = 0.3 s`). 20 Hz derinlik verisi + 2 mm
  çözünürlük ile yeterli. (Gelişmiş: alfa-beta filtre — zaman kalırsa.)

**Başlangıç değerleri (havuzda ayarlanacak, §6 Adım 3-5):**
```python
PID_DEPTH = dict(kp=1.5, ki=0.15, kd=0.6, out_limit=1.0, i_limit=0.4, ff=0.0)
```

### 4.2 Heading (yaw) — kaskad

**Dış döngü (pozisyon):**
```
e_h     = angle_error_deg(hedef, ölçülen)          [−180 … +180]
ω_hedef = clamp(Kp_pos · e_h, ±ω_max)              ω_max = 30 °/s
```
**İç döngü (hız):**
```
e_ω  = ω_hedef − yaw_rate(gyro)
I   ← I + e_ω·dt        [koşullu]
u    = Kp_rate·e_ω + Ki_rate·I
yaw  = clamp(u, ±out_limit)
```

**Neden kaskad:**
- Dönüş hızı üst sınırı **doğrudan** ayarlanabilir → 90° dönüşte aşım kontrol altında.
- İç döngü gyroyu ölçüm olarak kullanır → türev gürültüsü yok, tepki hızlı.
- Aynı iç döngü daire görevinde `ω_hedef` sabit verilerek tekrar kullanılır (§4.4).
- Akıntı/motor asimetrisine karşı `Ki_rate` kalıcı hatayı siler (mevcut `ki=0` bunu yapamıyor).

**İki mod:**

| Mod | Ne zaman | ω_max | out_limit | Ki_rate |
|---|---|---|---|---|
| Seyir | `surge > 0` (düz segmentler) | 15 °/s | 0.35 | aktif |
| Dönüş | `surge = 0` (TURN1/TURN2) | 30 °/s | 0.60 | aktif, tolerans içinde sıfırlanır |

**Başlangıç değerleri:**
```python
HEADING_POS  = dict(kp=1.2, w_max_dps=30.0)                      # derece → °/s
HEADING_RATE = dict(kp=0.020, ki=0.010, out_limit=0.6, i_limit=0.25)   # °/s → komut
```
> `Kp_rate = 0.020` seçimi: 30 °/s hata → 0.6 komut (tam otorite). Mantıklı başlangıç.

### 4.3 Roll / Pitch

Görev için **kritik değil**, ama şu an ölü bant yüzünden hiç çalışmıyor (B3).

- Öncelik sırası: **mekanik trim > PID.** Havuzda ilk 10 dakika ağırlık/köpük dengesine
  ayrılacak; araç motorlar kapalıyken ±3° içinde durmalı.
- PID düşük otoriteyle, ölü bant telafili, ölü bölge (`|açı| < 2°` → çıkış 0) ile:
```python
PID_ROLL  = dict(kp=0.020, ki=0.0, kd=0.010, out_limit=0.25, i_limit=0.05, deadzone_deg=2.0)
PID_PITCH = dict(kp=0.020, ki=0.0, kd=0.010, out_limit=0.25, i_limit=0.05, deadzone_deg=2.0)
```
- D terimi gyro `gx`/`gy`'den (ölçüm türevi), tıpkı heading gibi.

### 4.4 Daire

**Şu anki yaklaşım:** sabit `yaw` **komutu** + sabit surge → yarıçap batarya voltajına,
sürüklenmeye, motor sıcaklığına göre değişir. Tekrarlanabilir değil.

**Yeni yaklaşım:** sabit **yaw_rate hedefi** (§4.2 iç döngüsü) + sabit surge.
Kapalı çevrim olduğu için yarıçap tekrarlanabilir olur.

```
r = v / ω        →        ω[°/s] = (2·v / D) · 57.3
```

| İstenen çap D | Seyir hızı v | Gereken ω | Tur süresi |
|---|---|---|---|
| 1.2 m | 0.25 m/s | **23.9 °/s** | 15.1 s |
| 1.2 m | 0.20 m/s | 19.1 °/s | 18.8 s |
| 1.5 m | 0.25 m/s | 19.1 °/s | 18.8 s |

**Öneri: D = 1.2 m** (şartname minimum 1 m — %20 pay), `v ≈ 0.25 m/s`, `ω = 24 °/s`.
`v`'yi düz segment kalibrasyonundan (§6 Adım 8) ölçüp bu tablodan ω seçilecek.

Açı sayacı: **işaretli** integrasyon + 1 dps ölü bant (B6). Hedef `370°` (pay).
Çıkışta `h0 + 180°` heading kilidi (mevcut mantık doğru, korunuyor).

### 4.5 Düz segmentler

- `surge = cruise_throttle` sabit (açık çevrim), heading kaskad kilitli, derinlik PID aktif.
- **Rota kapanması (H9) için kritik:** 4 düz segmentin **hız × süre** çarpımı eşit olmalı.
  Aynı `cruise_throttle` ve aynı `straight_time_s` kullanmak matematiksel olarak yeterli —
  ama batarya düşerken hız da düşer. → 4 segment arası batarya voltajı loglanacak;
  gerekirse son segmentin süresi log'dan kalibre edilir.

---

## 5. Dosya bazlı değişiklik listesi

| # | Dosya | Değişiklik | Neden | Risk |
|---|---|---|---|---|
| 1 | `sensors/depth.py` | OSR 8192 → 1024 (`0x44`/`0x54`); `max(0.0,…)` clamp kaldır; `read_depth_m_raw()` | B2, B4 | Düşük |
| 2 | `sensors/state.py` **(yeni)** | `RovState` + IMU/derinlik okuma thread'leri, kilitli paylaşım | B2 / D1 | Orta — thread güvenliği |
| 3 | `control/pid.py` | D-on-measurement, zaman sabitli filtre, koşullu integrasyon + back-calculation, `ff` girişi, `deadzone`, iç terimleri dışa aç (`last_p/i/d/sat`) | B7 | Düşük |
| 4 | `control/cascade.py` **(yeni)** | `HeadingController` (dış P + iç PI), mod değişimi, ω_max | D2 | Orta |
| 5 | `control/mixer.py` | Ölü bant telafisi (`deadband_compensate`) | B3 | Düşük |
| 6 | `control/stabilizer.py` | `RovState` cache kullan, tek örnekleme, `FF_hover`, roll/pitch ölü bölge, `set_mode('cruise'/'turn')` | B2, D3 | Orta |
| 7 | `missions/video_demo.py` | İşaretli daire integrasyonu + giriş sıfırlama; `depth_error()` cache'ten; `ω` tabanlı daire | B6 | Düşük |
| 8 | `utils/logger.py` | Cache'ten oku (blocking yok); yeni sütunlar: `dt, depth_rate, p, i, d, ff, sat, batt` | B2, D7 | Düşük |
| 9 | `main.py` | Deadline tabanlı döngü zamanlayıcı + gerçek `dt` ölçümü | B2 | Düşük |
| 10 | `calibrate_imu.py` | `\|accel_bias\| > 0.3 g` → kaydetme, hata ver | B1 | Düşük |
| 11 | `config.py` | Yeni PID blokları, `FF_HOVER`, `GYRO_NOISE_DPS`, `CIRCLE_*` | — | Düşük |
| 12 | `pid_tune.py` **(yeni)** | Havuz için canlı ayar konsolu: `kp/ki/kd/ff/step/hold/sweep/save`, otomatik adım-yanıtı analizi | D7 | Düşük |

> **Not:** `auto_pid.py` (röle/Ziegler-Nichols) mevcut haliyle **kullanılmamalı** —
> `MOTOR_DIRECTION` doğrulanmadan ve döngü 7.3 Hz'de iken ürettiği Ku/Tu anlamsız.
> K1–K3 kapıları geçildikten sonra, sadece derinlik ekseninde bir *başlangıç tahmini*
> aracı olarak kullanılabilir; nihai değer elle ayarlanacak.

---

## 6. Havuza kadar ~3 saatlik iş planı

| Süre | İş | Çıktı / geçme kriteri |
|---|---|---|
| **0:00–0:20** | **K1** — IMU yeniden kalibrasyonu (araç düz, hareketsiz) + `calibrate_imu.py` doğrulama kontrolü | `\|ACCEL_BIAS\| < 0.3 g`, `\|GYRO_BIAS\| < 3 dps`; `test_imu.py` 60 sn'de roll/pitch sürüklenmesi **< 2°** |
| **0:20–0:50** | **K2** — Derinlik OSR + `RovState` thread'leri + deadline zamanlayıcı | `main.py` boş döngüde **≥ 40 Hz**, log `dt` jitter < %20 |
| **0:50–1:10** | **K3** — `kanal_test.py` ile 6 motor yön doğrulaması + eksen testi (`surge/yaw/heave` işaretleri) | `MOTOR_DIRECTION` elle doğrulanmış; `heave=+` **aşağı** itiyor |
| **1:10–2:10** | `pid.py` yeniden yazımı + `cascade.py` + mixer ölü bant telafisi + stabilizer entegrasyonu | Simülasyonda tam görev hatasız dönüyor |
| **2:10–2:30** | `video_demo.py` daire düzeltmesi + genişletilmiş loglama | Sim logunda `circle_acc` gürültüde birikmiyor |
| **2:30–2:50** | `pid_tune.py` canlı ayar konsolu | Havuzda tek terminalden kp/ki/kd/ff değiştirilebiliyor |
| **2:50–3:00** | Havuz çantası: yedek batarya, laptop şarjı, protokol çıktısı, e-stop testi | Hazır |

**Zaman daralırsa öncelik sırası:** K1 → K3 → K2 → mixer ölü bant → `pid_tune.py` →
kaskad heading. (Kaskad olmadan da mevcut PID ile ilerlenebilir; K1/K3 olmadan **hiçbir şey** ilerlemez.)

---

## 7. Havuzda ayar protokolü

> Her adımın **geçme kriteri** var. Kriter sağlanmadan bir sonrakine geçilmez —
> bozuk bir alt katman üstündekini ayarlanamaz hale getirir.
> Her denemeden sonra log dosyası saklanır; hangi katsayı ile çekildiği not edilir.

### Adım 0 — Güvenlik (5 dk)
Sızdırmazlık kontrolü, e-stop testi (buton → tüm motorlar nötr, videoda gösterilecek),
güvenlik halatı bağlı, kurtarma planı.
**Geçme:** e-stop 1 sn içinde tüm motorları durduruyor.

### Adım 1 — Mekanik trim (10 dk) · *motorlar kapalı*
Araç suya bırakılır, serbest halde nerede durduğu gözlenir.
**Geçme:** araç **yavaşça yüzüyor** (pozitif kaldırma — arıza güvenliği), roll/pitch **< 3°**.
Değilse ağırlık/köpük ekle. *Bu adımı atlama — roll/pitch PID'i mekanik dengesizliği düzeltemez.*

### Adım 2 — `FF_hover` ölçümü (10 dk)
`pid_tune.py > hold` modu: PID kapalı, sabit `heave` komutu ver, 0.05 adımlarla artır.
Araç 0.6 m civarında **sabit kalana** kadar (ne çıkıyor ne iniyor).
**Çıktı:** `FF_HOVER = <o değer>` → `config.py`.
**Geçme:** ±0.05 komut aralığında araç 10 sn dengede.

### Adım 3 — Derinlik `Kp` süpürme (15 dk)
`ki=0, kd=0, ff=FF_HOVER`. `Kp`: 0.5 → 1.0 → 1.5 → 2.0 → 3.0.
Her değerde 0 → 0.6 m adım.
**Aranan:** hafif salınım başlangıcı → `Kp_kritik`. **Seçim:** `Kp = 0.6 · Kp_kritik`.
**Geçme:** aşım < 25 cm, sürekli salınım yok.

### Adım 4 — Derinlik `Kd` (10 dk)
`Kd`: 0.2 → 0.4 → 0.6 → 1.0. Aşımı söndür.
**Geçme:** aşım < 15 cm, yerleşme < 6 s (H3).

### Adım 5 — Derinlik `Ki` (10 dk)
`Ki`: 0.05 → 0.10 → 0.20. Kalıcı hatayı sil.
**Geçme:** 20 sn'de ±5 cm RMS (H2), kalıcı sapma < 3 cm, sarkma/salınım yok.
> ⚠️ Ki artırırken aşım geri gelirse Ki'yi düşür, Kd'yi artırma — windup işaretidir.

### Adım 6 — Heading **iç döngü** (ω) (15 dk)
Derinlik kilitliyken `ω_hedef = 20 °/s` sabit ver; `Kp_rate` süpür (0.010 → 0.030),
sonra `Ki_rate` (0.005 → 0.020).
**Geçme:** ölçülen ω, hedefin ±3 °/s içinde, salınımsız, 1 sn içinde oturuyor.

### Adım 7 — Heading **dış döngü** (15 dk)
`Kp_pos` süpür (0.8 → 1.2 → 2.0). Yerinde 90° dönüş adımı.
**Geçme:** H6 — süre < 6 s, aşım < 8°, ±2° yerleşme.

### Adım 8 — Düz segment + **hız kalibrasyonu** (15 dk)
`cruise_throttle = 0.35`, 16 sn düz git. Havuz kenarından mesafeyi ölç.
**Çıktılar:** `v = mesafe / 16` (m/s) → §4.4 tablosundan daire `ω`'sını seç.
**Geçme:** H5 — heading ±3° RMS; iz gözle **düz**.

### Adım 9 — Daire (15 dk)
Adım 8'den gelen `ω` ve `v` ile. Çapı gözle/kenar referansıyla ölç.
**Geçme:** H8 — çap ≥ 1.2 m, jiroskop toplamı 360° ± 10°, çıkışta heading `h0+180 ± 5°`.

### Adım 10 — Tam prova (kalan süre, en az 3 tekrar)
Tam görev dizisi, e-stop gösterimi dahil.
**Geçme:** H9 — ardışık **3 denemede** bitiş 1×1 m alan içinde.

---

## 8. Riskler ve önlemler

| Risk | Belirti | Önlem |
|---|---|---|
| IMU kalibrasyonu havuzda tekrar bozulur (metal/kablo manyetik etkisi) | heading yavaşça sürükleniyor | Kalibrasyon **görev konfigürasyonunda** (tüm kablolar bağlı, batarya takılı) yapılacak |
| Manyetometre havuz demir donatısından etkileniyor | heading sıçramalı | `HEADING_FILTER_ALPHA` 0.98 → 0.995 (jiroskopa güven); görev 2 dk sürüyor, jiroskop sürüklenmesi tolere edilir |
| Batarya düşerken hız düşüyor | son segment kısa kalıyor | Voltaj loglanacak; provalar aynı şarj seviyesinde tekrarlanacak |
| Derinlik sensörü havuzda gürültülü | `heave` titriyor | `ḋ` LPF `tau` 0.3 → 0.5; OSR'yi 2048'e çıkar |
| Havuz süresi yetmiyor | — | Öncelik: derinlik tutma > heading tutma > dönüş > daire. Daire en son ayarlanır; şartname minimumu 1 m çap, pay bırakıldı |
| Yeni thread yapısı kilitlenir | döngü donar | `RovState` sadece `threading.Lock` + kopya okuma; watchdog: 0.5 sn veri gelmezse `thr.stop()` |

---

## 9. Genişletilmiş log şeması

Ayarlamanın tamamı bu CSV'den yapılacak — eksik sütun = havuzda kaybedilen deneme.

```
t, dt, state,
depth, depth_target, depth_rate, depth_p, depth_i, depth_d, depth_ff, depth_sat,
heading, heading_target, heading_err, yaw_rate, yaw_rate_target, yaw_p, yaw_i, yaw_sat,
roll, pitch,
surge, yaw, heave, roll_cmd, pitch_cmd,
m_HL, m_HR, m_VFL, m_VFR, m_VRL, m_VRR,
batt_v, note
```

Ek olarak `tools/analyze_log.py`: bir log dosyasından H1–H9 kriterlerini otomatik
hesaplayıp "GEÇTİ/KALDI" tablosu basar. Havuz kenarında her denemeden sonra çalıştırılır.

---

## 10. Sıradaki adım

Onay verirsen **K1 → K2 → K3** sırasıyla kod değişikliklerine başlıyorum;
§5'teki 12 kalemi yukarıdaki zaman planına göre uygularım.
Havuza girmeden önce §7 protokolünü tek sayfalık yazdırılabilir bir kontrol listesi
olarak da çıkarabilirim.

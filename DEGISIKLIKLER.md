# Yapılan Değişiklikler — Her Satırın Gerekçesi

> Bu doküman `PID_BASIT_ANLATIM.md`'deki her soruna karşılık **hangi dosyada ne yaptığımı**,
> **neden öyle yaptığımı** ve **nasıl doğruladığımı** anlatır.
> Sonunda ayrıca: sorun listesinde olmayıp yine de yazdığım şeyler ve
> **bilerek değiştirmediğim** şeyler.

---

## 0. Havuza girmeden çalıştırılacak 3 komut

```bash
cd rov

# 1) SORUN 1 — IMU kalibrasyonu (araç DÜZ ve HAREKETSİZ)
python3 calibrate_imu.py

# 2) SORUN 5 — motor yönleri (pervaneler sökülü ya da araç suda)
python3 verify_directions.py

# 3) SORUN 2 — döngü frekansı gerçekten düzeldi mi?
python3 main.py --check-loop
```

Üçü de geçmeden havuza girme. Sonra:

```bash
python3 pid_tune.py       # havuzda canlı ayar konsolu
python3 tools/analyze_log.py   # her denemeden sonra GEÇTİ/KALDI raporu
```

---

# BÖLÜM 1 — Sorunlar ve çözümleri

## SORUN 1 — Geçersiz IMU kalibrasyonu

**Nerede:** `calibrate_imu.py`, `sensors/imu.py`

**Ne yaptım**

`calibrate_accel()` içine üç fiziksel geçerlilik kontrolü koydum:

| Kontrol | Eşik | Ne yakalar |
|---|---|---|
| `\|bias\| > 0.3 g` | ivmeölçer ±2 g aralığında | Doyuma girmiş / saçma değer (mevcut 2.0 g) |
| ölçüm std > 0.05 g | — | Kalibrasyon sırasında araç hareket ediyordu |
| toplam ivme 0.85–1.15 g dışı | 1 g olmalı | Araç düz durmuyor veya sensör doyumda |

Herhangi biri tetiklenirse fonksiyon `None` döner ve `main()` **config.py'ye hiçbir şey yazmaz.**
`calibrate_gyro()` de aynı şekilde sertleştirildi (hareket varsa veya bias >10 dps ise geçersiz).

Ayrıca `sensors/imu.py` **import anında** config'teki değerleri kontrol edip büyük bir uyarı basıyor.
Test ederken bunu gördüm — tam istediğim gibi çalışıyor:

```
[IMU KALİBRASYON UYARISI] ACCEL_BIAS=(2.0, -2.0, -0.296) FİZİKSEL OLARAK GEÇERSİZ.
    SONUCU: araç düz dururken program 'yan yatmışım' sanıyor, roll/pitch
    PID'leri boşuna çalışıyor VE pusula da bozuluyor.
    YAP: python3 calibrate_imu.py
```

**Neden bu yaklaşım**

Değeri kodda "düzeltmek" yanlış olurdu — doğru değeri ancak sensör ölçebilir.
Yapılması gereken, **bir daha bozuk değerin sessizce config'e girmesini engellemek.**
Bu hata iki hafta boyunca fark edilmedi çünkü hiçbir yerde alarm çalmıyordu.

**Nasıl doğrulanır:** kalibrasyondan sonra `python3 test_imu.py` — 60 saniyede roll/pitch sürüklenmesi < 2° olmalı.

---

## SORUN 2 — Döngü 50 Hz yerine 7.3 Hz

**Nerede:** `sensors/depth.py`, `sensors/state.py` *(yeni)*, `utils/looptimer.py` *(yeni)*, `main.py`, `hal/i2c_lock.py` *(yeni)*, `utils/logger.py`, `comms/web_server.py`

Bu tek soruna dört ayrı cephede saldırdım, çünkü tek bir düzeltme yetmezdi.

### 2a) Sensör hassasiyeti — `sensors/depth.py`

OSR artık `config.DEPTH_OSR` ile ayarlanıyor, varsayılan **1024**.

| OSR | Bekleme | Çözünürlük |
|---|---|---|
| 8192 (eski) | 20 ms × 2 = **40 ms** | ~0.02 mm |
| 1024 (yeni) | 3 ms × 2 = **6 ms** | **~2 mm** |

Toleransımız 5 cm; 2 mm fazlasıyla yeter. **6.7 kat hızlanma, bedava.**

### 2b) Sensörler ayrı thread'e — `sensors/state.py` (yeni dosya)

```
IMU thread'i      (100 Hz)  ─┐
                             ├─▶  RovState (kilitli ortak hafıza)  ─▶  kontrol döngüsü (50 Hz)
Derinlik thread'i ( 20 Hz)  ─┘                                          hiç beklemez
```

Kontrol döngüsü `snapshot()` ile **tutarlı bir kopya** alır. Neden kopya: thread'ler sürekli
yazıyor; döngü ortasında heading güncellenirse aynı adımda yarısı eski yarısı yeni veriyle
hesap yapmış olurduk.

Derinlik thread'i ayrıca **dikey hızı** hesaplayıp filtreliyor — PID'in D terimi bunu kullanıyor,
böylece PID sayısal türev almak zorunda kalmıyor.

### 2c) Deadline zamanlayıcı — `utils/looptimer.py` (yeni dosya)

Eski kod her döngü sonunda `time.sleep(1/LOOP_HZ)` yapıyordu. Bu **hesaplama süresini hesaba katmaz**:

```
gerçek periyot = hesaplama_süresi + 1/LOOP_HZ
```

Doğrusu deadline mantığı: `sonraki = önceki + periyot; uyu(sonraki - şimdi)`.
`LoopTimer` ayrıca gerçek frekansı ölçüyor ve hedefin altına düşerse uyarı basıyor.

> İnce ayrıntı: ESC arm (`time.sleep(2.0)`) gibi tek seferlik bloklamalar "en kötü adım"
> istatistiğini bozmasın diye ayrı sayılıyor.

### 2d) Tek okuma — `control/stabilizer.py`, `utils/logger.py`, `comms/web_server.py`

Aynı sensör döngü başına **3 kez** okunuyordu (stabilizer + görev kodu + logger).
Artık `Stabilizer.sample()` döngü başına bir kez örnekliyor; `depth_error()`, logger ve web
arayüzü hep bu önbelleği okuyor. Web arayüzü de artık kendi thread'inden I2C'ye dokunmuyor.

### 2e) I2C kilidi — `hal/i2c_lock.py` (yeni dosya)

Artık üç thread aynı I2C hattını kullanıyor (IMU, derinlik, motor sürücü).
Tek ortak `RLock` ile veri yolu işlemleri korunuyor.

**Kritik detay:** kilit **sadece veri yolu işlemlerini** sarıyor, sensörün dönüşüm beklemesini
**sarmıyor**. Yoksa derinlik sensörü beklerken IMU thread'i de dururdu ve SORUN 2'yi geri getirirdik.

**Ölçülen sonuç (simülasyon):** `50.0 Hz`, en kötü adım `25 ms`. `--check-loop` gerçek donanımda aynısını ölçer.

---

## SORUN 3 — PWM ölü bandı küçük komutları yutuyor

**Nerede:** `control/mixer.py`

```python
DEADBAND_N = PWM_DEADBAND_US / PWM_RANGE_US       # 30/470 = 0.0638

def deadband_compensate(u, eps=0.01):
    if abs(u) < eps: return 0.0                    # gerçekten dur (titreme yok)
    mag = DEADBAND_N + (1.0 - DEADBAND_N) * min(1.0, abs(u))
    return mag if u > 0 else -mag
```

| Komut | Eski (motora giden) | Yeni |
|---|---|---|
| 0.00 | 0 | 0 |
| 0.01 | **0** (yutuldu) | 0.073 |
| 0.05 | **0** (yutuldu) | 0.111 |
| 0.50 | 0.50 | 0.532 |
| 1.00 | 1.00 | 1.000 |

**Yerleşim kararı:** telafi, grup normalizasyonundan **sonra**, `MOTOR_DIRECTION`'dan **önce** uygulanıyor.
Önce normalize edilmeli ki motorlar arası oran korunsun; sonra her motor ölü bandın üstüne taşınır.

**Kabul edilen takas:** telafi motor bazında uygulandığı için diferansiyel fark bir miktar küçülüyor
(0.100 → 0.094). Uçurumdan (sıfıra düşme) çok daha iyi. `config.DEADBAND_COMPENSATION = False` ile
havuzda tek satırda kapatılabilir.

**Doğrulama (birim testi):** sıfır tam sıfır kalıyor, küçük komut ölü bandın üstüne çıkıyor,
tam gaz hâlâ 1.0 (taşma yok), simetrik, monoton artan. ✅

---

## SORUN 4 — Derinlik kontrolünün üç eksiği

**Nerede:** `sensors/depth.py`, `control/pid.py`, `control/stabilizer.py`, `config.py`

### 4a) Yukarı çıkışı göremiyordu

`max(0.0, ...)` kırpması `read_depth_m()`'den **kaldırıldı** — artık işaretli değer dönüyor.
Ekranda göstermek için ayrı `read_depth_m_display()` var.

Yüzeye çıkmak eleme sebebi; PID'in ne kadar yukarı fırladığını görememesi kabul edilemez.

### 4b) İleri besleme (feed-forward) yok

`PID.update(..., ff=...)` girişi eklendi. Yeni formül:

```
motor = FF_HOVER + Kp·hata + Ki·∫hata − Kd·dikey_hız
```

`config.FF_HOVER` şu an **0.0** — çünkü **havuzda ölçülmesi gerekiyor**.
`pid_tune.py`'de `hover <değer>` komutu tam bunun için: PID kapalı, sabit dikey gaz;
araç ne çıkıyor ne iniyorsa o değer `FF_HOVER`'dır.

> `tools/analyze_log.py` her raporda `FF_HOVER = 0 → İLERİ BESLEME HİÇ ÖLÇÜLMEMİŞ` uyarısı basıyor,
> unutulmasın diye.

### 4c) Windup

Detayı SORUN 7'de (aynı kod). Kısaca: çıkış tavana değince I birikimi duruyor.

---

## SORUN 5 — Motor yön tablosu güvenilmez

**Nerede:** `verify_directions.py` *(yeni dosya)*

`MOTOR_DIRECTION` değerlerini **kodda değiştirmedim** — çünkü doğru değeri ancak fiziksel ölçüm verir.
Bunun yerine ölçümü yapan aracı yazdım:

**Aşama A — motorlar tek tek.** Her motora ham +%25 komut gönderilir (`MOTOR_DIRECTION` uygulanmadan)
ve net bir soru sorulur:

- yatay motorlar: "+ komut aracı **ileri** itmeli"
- dikey motorlar: "+ komut aracı **aşağı** itmeli"

Cevaba göre işaret belirlenir. Bu tam olarak mixer'in konvansiyonu, yani sonuç doğrudan
`MOTOR_DIRECTION`'a karşılık gelir.

**Aşama B — eksen doğrulaması.** Bulunan tabloyla mixer çalıştırılıp ileri / sağa dönüş / dalış
test edilir. Bu, tablonun doğru olduğunu **teyit** eder ve kanal karışıklığını yakalar.

**Aşama C — config.py'ye yazma** (önce `.bak` yedeği).

Aşama B başarısızsa config yazılmıyor ve olası sebepler ekrana basılıyor
(kanal karışıklığı → `kanal_test.py`).

---

## SORUN 6 — Daire sayacı gürültüyü tur sanıyor

**Nerede:** `missions/video_demo.py`, `config.py`

```python
# ESKİ:  self._circle_acc += abs(yaw_rate) * dt
# YENİ:
if not self._circle_done and abs(w) > GYRO_NOISE_DPS:   # 1.0 dps eşiği
    self._circle_acc += w * dt                          # İŞARETLİ
```

Üç ayrı düzeltme:

1. **`abs()` kaldırıldı** — işaretli toplama. Ters yöne sapma artık sayacı geri alıyor.
2. **Gürültü eşiği** (`GYRO_NOISE_DPS = 1.0`) — eşik altı okumalar sayılmıyor.
3. **Duruma girişte sıfırlama** — eski kodda sayaç sadece `__init__`'te sıfırlanıyordu.

Ek olarak: hedefe ulaşınca sayaç **donduruluyor** (`_circle_done`). Yoksa `h0+180`'e kilitlenme
dönüşü de sayaca ekleniyor ve log 530° gösteriyordu — kabul kriteri anlamsızlaşıyordu.

**Ölçülen fark (birim testi, 40 sn duran araç, 0.5 dps RMS gürültü):**

| Yöntem | Sonuç |
|---|---|
| Eski `abs()` | **+15.9°** sahte dönüş |
| Yeni işaretli + eşik | **−0.0°** |

---

## SORUN 7 — PID sınıfının yapısal eksikleri

**Nerede:** `control/pid.py` — dosya **sıfırdan yazıldı**

| # | Eksik | Çözüm |
|---|---|---|
| 7a | D, hata üzerinden → setpoint kick | D artık **ölçüm** üzerinden (`meas_rate` / `measurement`) |
| 7b | D filtresi örnek sayısına bağlı | Zaman sabitli: `a = dt/(d_tau+dt)` |
| 7c/4c | Anti-windup çıkış doygunluğuna bakmıyor | **Back-calculation** (aşağıda) |
| 7e | Ki integral birikiminin içinde | Ki **çıkışta**; `i_limit` artık çıkış biriminde |
| 7f | İç terimler görünmüyor | `self.last` → P/I/D/FF/sat ayrı ayrı, CSV'ye yazılıyor |
| — | FF girişi yok | `update(..., ff=...)` |
| — | Ölü bölge yok | `deadzone` (roll/pitch için) |

### Anti-windup — burada bir hata bulup düzelttim

İlk yazdığım versiyon "çıkış doygunsa bu adımı hiç işleme" diyordu (koşullu integrasyon).
Birim testi yazınca **çalışmadığını gördüm**:

> Tek bir adımda I'nın büyüme miktarı kalan boşluktan büyükse, adımın **tamamı** reddediliyor.
> Sonuç: I hiçbir zaman **bir adım bile** büyüyemiyor ve kalıcı hata asla silinmiyor.
> Ölçülen: `kp=0.05, hata=10, ki=1.0, dt=0.1` → I sonsuza kadar 0 kaldı.

Doğrusu **back-calculation**: I'yı, çıktıyı **tam olarak sınıra getiren** değere kadar bırak.

```python
i_üst = ( out_limit - (ff + P + D)) / ki
i_alt = (-out_limit - (ff + P + D)) / ki
i_yeni = clamp(i_try, min(i_alt,i_üst), max(i_alt,i_üst))

# tek istisna: kırpma güncelleme YÖNÜNÜ tersine çevirdiyse (P tek başına
# sınırı aşmışsa) I'yı ters yöne sürüklemek yerine dondur
if (i_try > self._i and i_yeni < self._i) or (i_try < self._i and i_yeni > self._i):
    i_yeni = self._i
```

**Doğrulama:** 30 sn sürekli doygunluk sonrası

| | I tepe değeri | Hedefe gelince |
|---|---|---|
| Eski (koşulsuz + kırpma) | **5.000** (tavan) | 60+ sn hâlâ tam güçte itiyor |
| Yeni (back-calculation) | **0.500** (çıkışı tam 1.0 yapan değer) | aşım yok |

Ve ayrı bir testte sabit −0.3 bozucu yük altında kalıcı hatayı tam olarak sildi (ölçüm 1.000 / hedef 1.000). ✅

---

## SORUN 8 — Aynı sensör döngüde 3 kez okunuyor

SORUN 2d'de çözüldü (tek örnekleme + önbellek).

---

# BÖLÜM 2 — Sorun listesinde olmayan, ama yazdığım şeyler

Bunlar teşhis sırasında çıkmayan ama tasarımın doğru çalışması için gerekli olan parçalar.

## Y1 — Kaskad yön kontrolü (`control/cascade.py`, yeni dosya)

**Neden:** Eski tek-PID yapısı "90 derece hata var → şu kadar gaz" diyordu. Ama "şu kadar gaz"ın
aracı saniyede kaç derece döndürdüğünü kimse bilmiyor — batarya doluyken hızlı, boşken yavaş döner.
Havuzda bulunan katsayı yarışmada aynı davranmaz.

```
DIŞ KATMAN:  açı hatası → istenen dönüş hızı (°/s), ÜST SINIRLI
İÇ KATMAN:   istenen dönüş hızı → motor komutu (jiroskop geri beslemeli)
```

Dört kazanç:

1. Dönüş hızı **doğrudan** sınırlanıyor → aşım büyük ölçüde bitiyor.
2. Jiroskop **ölçüm** olarak kullanılıyor — türev almaya gerek yok.
3. Batarya/akıntı farkını iç döngü emiyor; dış döngü hep aynı davranıyor.
4. **Daire görevi aynı iç döngüyü kullanıyor** (`update_rate`) — ayrı kod yok.

Üç mod: `cruise` (düz seyir, düşük yetki), `turn` (yerinde dönüş, tam yetki), `circle` (daire).

## Y2 — Dairenin çapı artık hesaplanıyor

**Neden:** Eski kod daireye sabit bir **yaw komutu** veriyordu; çap bataryaya, sürüklenmeye,
motor sıcaklığına göre değişiyordu. Şartnamedeki 1 m şartını tutturmak şansa kalıyordu.

Yeni yöntem sabit **dönüş hızı hedefi** (kapalı çevrim):

```
çap = 2 × ileri_hız / dönüş_hızı
```

| Çap | İleri hız | Gereken dönüş hızı | Tur süresi |
|---|---|---|---|
| 1.2 m | 0.25 m/s | 23.9 °/s | 15.1 s |
| 1.2 m | 0.20 m/s | 19.1 °/s | 18.8 s |
| 1.5 m | 0.25 m/s | 19.1 °/s | 18.8 s |

İleri hız havuzda bir kez ölçülüp (protokol Adım 8) tablodan dönüş hızı seçilecek.

## Y3 — Watchdog (`main.py`)

Bir sensör thread'i takılırsa kontrol döngüsü **eski veriyle uçmaya devam ederdi** — tehlikeli.
Artık `SENSOR_STALE_S` (0.5 s) kadar taze veri gelmezse motorlar nötre çekilip görev iptal ediliyor.

## Y4 — Manyetometre reddi (`sensors/imu.py`)

Havuz kenarındaki demir donatı veya motor akımı pusulayı bozabilir. Manyetik alan şiddeti
makul aralığın (5–200 µT) dışındaysa o adımda pusula düzeltmesi **atlanıyor**, jiroskopla devam
ediliyor. Görev 2 dakika sürdüğü için jiroskop sürüklenmesi tolere edilebilir; ani pusula sıçraması
edilemez.

## Y5 — Simülatöre pozitif yüzerlik (`sim/simulator.py`)

Eski simülasyon **nötr yüzerlikte** bir araç modelliyordu. Bu, yeni tasarımın en önemli parçasını
test edilemez hâle getiriyordu: nötr yüzerlikte `ff = 0` zaten doğru cevap olur, hiçbir şey öğrenilmez.

Eklenenler: `BUOYANCY_MS = 0.06` (motorlar kapalıyken yükselme hızı → beklenen `FF_HOVER ≈ 0.12`)
ve `YAW_BIAS_DPS = 1.5` (motor asimetrisi — heading Ki'sini test etmek için).

## Y6 — Canlı ayar konsolu (`pid_tune.py`, yeni dosya)

Eski `pid_test_cal.py` sensörü kendi döngüsünde bloklayarak okuyordu (SORUN 2) ve katsayı
değiştirirken I birikimini sıfırlamıyordu — sonuçları yanıltıcıydı.

Yeni konsol katsayıyı **çalışırken** değiştiriyor, adım testi yapıyor ve sonucu **otomatik ölçüyor**:

```
  ADIM YANITI ANALİZİ  (depth, 0.120 -> 0.600 m)
  Aşım (overshoot) :  20.4 %   (0.098 m)
  Yerleşme (%5)    : OLUŞMADI  (kalıcı hata bandı aşıyor: band ±0.024 m)
  Kalıcı hata      : +0.060 m
  Son 5 sn RMS     : 0.010 m
  ÖNERİ: Kalıcı hata var (>3 cm): Ki'yi artır ya da FF'i düzelt.
```

**Öneri satırları özellikle var** — havuz kenarında düşünmeye vakit olmuyor.

## Y7 — Log analiz aracı (`tools/analyze_log.py`, yeni dosya)

Her denemeden sonra H1–H9 kabul kriterlerini otomatik ölçüp GEÇTİ/KALDI tablosu basıyor,
ayrıca PID tanısı yapıyor (doygunluk oranı, I tepe değeri, FF ölçülmüş mü).

## Y8 — Simülasyon uçtan uca testi (`tools/sim_test.py`)

Tam görevi simülasyonda koşturuyor. **Bu test iki gerçek kusur yakaladı** — aşağıda.

## Y9 — Genişletilmiş log şeması (`utils/logger.py`)

Eski log "motor neden %70 gaz verdi, P'den mi I'dan mı geldi" sorusunu cevaplayamıyordu.
Yeni sütunlar: `dt, hz, depth_p, depth_i, depth_d, depth_ff, depth_sat, yaw_p, yaw_i, yaw_sat,
yaw_rate_target, m_H_L … m_V_RR, imu_hz, depth_hz`.

---

# BÖLÜM 3 — Simülasyonun yakaladığı iki gerçek kusur

Bunlar teşhis aşamasında **görünmüyordu**; ancak kodu çalıştırınca ortaya çıktı.

## K1 — İç döngü `i_limit` çok düşüktü

**Belirti:** Dönüş hızı hedefi 30 °/s iken kontrolcü 25 °/s'de dengelendi.

**Sebep:** `out_limit` değil, `i_limit = 0.25` idi.

> **Hız döngüsünde kalıcı komutun asıl kaynağı I terimidir.** P sadece *hataya* tepki verir;
> hedefe yaklaştıkça P küçülür ve sıfıra gider. Yani "30 °/s'de sabit dönmek" için gereken
> kalıcı gazı I taşır. I 0.25'te tavan yapınca hedef hıza hiç ulaşılamıyordu
> (P=0.10 + I=0.25 tavan = 0.35 komut).

Bu daire çapını da hedeflediğimiz değil **ulaşılabilen** değer yapardı — tam da kaçınmak istediğimiz durum.

**Düzeltme:** `HEADING_RATE i_limit: 0.25 → 0.55`.
**Kural:** hız döngüsünde `i_limit ≈ out_limit`.

## K2 — Simülasyon saati gerçek zamandan yavaş akıyordu

**Belirti:** Daire, jiroskop toplamına göre 370° dönmüşken heading sadece 289° değişmişti.

**Sebep:** `main.py`'deki simülasyon döngüsü fiziği her adımda **sabit** 0.01 s ilerletiyordu,
ama `time.sleep(0.01)` + işlem yükü yüzünden gerçekte ~0.0125 s geçiyordu.
Simülasyon zamanı gerçek zamanın **~0.8 katı** akıyordu.

Bu bir kontrol hatası değil ama **tüm süre/hız/daire çapı ölçümlerini sessizce %25 yanıltıyordu** —
yani simülasyondan çıkardığımız her sayı yanlış olurdu.

**Düzeltme:** gerçek geçen süre kullanılıyor (`min(0.05, now - prev)`).

**Ek bulgu (aynı testten):** daire `cruise` modunun `out_limit = 0.35` yetkisiyle çalışıyordu ve
iç döngü doyuyordu → ayrı bir `circle` modu eklendi (`w_max 45 °/s, out_limit 0.55`).

---

# BÖLÜM 4 — Bilerek DEĞİŞTİRMEDİĞİM şeyler

Bir şeyi değiştirmemek de bir karardır. Gerekçeleri:

| Ne | Neden dokunmadım |
|---|---|
| **`MOTOR_DIRECTION` değerleri** | Doğru değeri ancak fiziksel ölçüm verir. Tahmin etmek, yanlış olması hâlinde pozitif geri besleme demek. Bunun yerine ölçüm aracını yazdım (`verify_directions.py`). |
| **`FF_HOVER`** | Aracın kaldırma kuvvetine bağlı; havuzda ölçülmeli. 0.0 bırakıldı ve analiz aracı her raporda hatırlatıyor. |
| **MS5837 2. derece sıcaklık kompanzasyonu** | Havuz sıcaklığında (~25 °C) etkisi ihmal edilebilir ve **suya girmeden doğrulayamayız**. Çalışan ama doğrulanamayacak bir şeyi değiştirmek gereksiz risk. |
| **`PWM_DEADBAND_US = 30`** | Ölü bandın kendisi *doğru* — ESC titremesini ve bip sesini engelliyor. Sorun ölü bant değil, telafisinin olmamasıydı. Telafiyi mixer'e ekledim, ölü bandı bıraktım. |
| **`PWM_NEUTRAL_US = 1470`, `PCA9685_REF_CLOCK_HZ`** | Bunlar ölçülerek kalibre edilmiş ve `thrusters.py`'de sınır kontrolü var. Doğrulayamadan dokunmam. |
| **`missions/line_follow.py`, `missions/nav_mission.py`** | Bu görevler kapsam dışıydı (video gösterimi seçildi). `Stabilizer` API'sini **geriye uyumlu** tuttum, ikisi de değişiklik olmadan çalışıyor. |
| **`comms/web_server.py`** | Sadece bloklayan derinlik okuması kaldırıldı. Arayüzün geri kalanına dokunulmadı. |
| **`auto_pid.py`, `pid_test_cal.py`** | Silmedim; başına "artık kullanılmıyor, yerine `pid_tune.py`" uyarısı koydum. `auto_pid.py`'nin röle/Ziegler-Nichols yöntemi `MOTOR_DIRECTION` doğrulanmadan ve döngü 7.3 Hz iken anlamsız Ku/Tu üretir. |
| **`config.py` yedekleri** | Her otomatik yazımdan önce `.bak` alınıyor. Ayrıca bu çalışma öncesi hâli `config.py.bak_pidrewrite` olarak duruyor. |

---

# BÖLÜM 5 — Dosya dosya özet

| Dosya | Durum | Sorun |
|---|---|---|
| `control/pid.py` | **sıfırdan yazıldı** | 4b, 4c, 7a–7f |
| `control/cascade.py` | **yeni** | Y1 |
| `control/mixer.py` | ölü bant telafisi | 3 |
| `control/stabilizer.py` | **sıfırdan yazıldı** | 2, 4, 7, Y1 |
| `sensors/state.py` | **yeni** | 2, 8 |
| `sensors/depth.py` | **sıfırdan yazıldı** | 2a, 4a |
| `sensors/imu.py` | **sıfırdan yazıldı** | 1, 2e, Y4 |
| `hal/i2c_lock.py` | **yeni** | 2e |
| `hal/i2c.py` | kilit eklendi | 2e |
| `utils/looptimer.py` | **yeni** | 2c |
| `utils/logger.py` | **sıfırdan yazıldı** | 2d, 7f, Y9 |
| `missions/video_demo.py` | **sıfırdan yazıldı** | 6, 8, Y1, Y2 |
| `main.py` | **sıfırdan yazıldı** | 2, Y3, K2 |
| `config.py` | PID bölümü yeniden tasarlandı | tümü |
| `calibrate_imu.py` | doğrulama eklendi | 1 |
| `sim/simulator.py` | yüzerlik + asimetri | Y5 |
| `verify_directions.py` | **yeni** | 5 |
| `pid_tune.py` | **yeni** | Y6 |
| `tools/analyze_log.py` | **yeni** | Y7 |
| `tools/sim_test.py` | **yeni** | Y8 |
| `comms/web_server.py` | bloklayan okuma kaldırıldı | 2d |
| `auto_pid.py`, `pid_test_cal.py` | "kullanılmıyor" işareti | — |

---

# BÖLÜM 6 — Simülasyon sonucu (mevcut durum)

```
  [GEÇTİ] Kontrol döngüsü >= 30 Hz                    ölçülen  49.8   eşik 30.0 Hz
  [GEÇTİ] Derinlik tutma <= 5 cm RMS                  ölçülen  0.043  eşik 0.05 m
  [GEÇTİ] Düz seyirde yön <= 3 deg RMS                ölçülen  1.79   eşik 3.0 deg
  [GEÇTİ] Derinlik aşımı <= 15 cm                     ölçülen  0.014  eşik 0.15 m
  [GEÇTİ] Yüzeye çıkmama (derinlik >= 0.25 m)         ölçülen  0.465  eşik 0.25 m
  [GEÇTİ] 90 derece dönüş <= 6 s (TURN1)              ölçülen  4.48   eşik 6.0 s
  [GEÇTİ] 90 derece dönüş <= 6 s (TURN2)              ölçülen  4.36   eşik 6.0 s
  [GEÇTİ] Roll/pitch <= 8 deg                         ölçülen  0.0    eşik 8.0 deg
  [GEÇTİ] Daire sayacı hedefe ulaştı (±10 deg)        ölçülen  370.6  eşik 10.0 deg
  SONUÇ: TÜM KRİTERLER GEÇTİ
```

**Bunun anlamı ve anlamı olmayanı:**

- ✅ Mantık doğru: durum makinesi, kaskad kontrol, anti-windup, daire sayacı, döngü hızı.
- ❌ Katsayılar doğru **değil** — simülasyon fiziği kaba bir yaklaşım. `Kp = 1.5` sim'de iyi
  çalışıyor olması gerçek araçta iyi çalışacağı anlamına gelmez.
- ❌ `FF_HOVER` hâlâ ölçülmedi.
- ❌ `MOTOR_DIRECTION` hâlâ doğrulanmadı.

Bunlar zaten havuz protokolünün işi (`PID_TASARIM_PLANI.md` §7).

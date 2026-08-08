# EGE ROV — Yer İstasyonu (Web GCS) Kullanım Kılavuzu

> Arayüzdeki **her** öğenin ne olduğu, neyi gösterdiği ve ne işe yaradığı.
> Sonda havuz iş akışı ve sorun giderme tablosu var.

---

## 0. Nasıl açılır

### Araçla (Jetson üzerinde)

```bash
cd rov
python3 main.py                    # video demo görevi
python3 main.py --mission line     # Görev 1: hat takibi
python3 main.py --mission nav      # Görev 2: navigasyon
python3 main.py --sim              # donanım yok, fizik simülasyonu
```

Tarayıcı: **http://192.168.1.10:8000/** (Jetson'ın kendisinde `http://localhost:8000/`)

### Araçsız (masaüstünde önizleme / öğrenme)

```bash
cd rov
python3 -m comms.web_server --demo        # --port 8001 ile port değiştirilebilir
```

Tarayıcı: **http://localhost:8000/**

Hiçbir harici paket gerekmez (OpenCV/numpy dahil). Kamera görüntüsü olmaz, geri kalan
her şey gerçek: basit bir araç fizik modelini **gerçek** `Stabilizer` ve PID nesneleri
sürer. PID katsayısını değiştirdiğin an cevabın nasıl değiştiğini görürsün.

> Demodaki fizik kaba bir yaklaşımdır. Orada bulduğun katsayı havuzda aynı çıkmaz.
> Demonun faydası **arayüzü ve iş akışını** havuza girmeden öğrenmendir.

---

## 1. Ekran düzeni

```
┌──────────────────────────────────────────────────────────────────────────┐
│ ÜST BAR   logo │ rozetler (bağlantı, arm, e-stop, sensör, mod, durum, Hz) │
│                                            │ ARM VEHICLE │ EMERGENCY ABORT │
├───────────────────┬─────────────────────┬────────────────────────────────┤
│ SOL               │ ORTA                │ SAĞ                            │
│ • FPV video       │ • HUD / PFD         │ • KONTROL MODU  ← en önemli    │
│ • AR göstergeler  │ • Telemetri kartları│ • Sekmeler:                    │
│ • Manuel sürüş    │ • Görev 2 sensörleri│   HEDEF │ PID │ SAĞLIK │ GÖREV │
│   tuş yardımı     │ • Motor itki monitörü│ • Canlı log konsolu           │
└───────────────────┴─────────────────────┴────────────────────────────────┘
```

Telemetri **20 Hz** yenilenir (saniyede 20 kez).

---

## 2. Üst bar

### 2.1 Rozetler

| Rozet | Yeşil nokta | Kırmızı / sönük | Ne demek |
|---|---|---|---|
| **BAĞLANTI** | `CANLI BAĞLANTI (20Hz)` | `BAĞLANTI KESİLDİ` | Tarayıcı ↔ Jetson HTTP bağlantısı. Kesikse ağ/tether sorunu ya da program durdu. |
| **ARM** | `ARMED` | `DISARMED` | Motorlar komut kabul ediyor mu. DISARMED iken `thr.command()` hiçbir şey yapmaz. |
| **E-STOP** | `E-STOP NORMAL` | `E-STOP AKTİF!` | Donanımsal acil durdurma hattı (GPIO 27). Aktifse görev iptal edilir. |
| **SENSÖR** | `SENSÖR TAZE` | `SENSÖR BAYAT!` | Watchdog. Bayat olursa **görev otomatik iptal olur ve motorlar nötre çekilir.** |
| **MOD** | — | — | Aktif kontrol modu (AUTO/HOLD/HOVER/RATE/TELEOP). Bkz. §5. |
| **DURUM** | — | — | Görevin durum makinesindeki adı (COUNTDOWN, DIVE, CRUISE, TURN1, CIRCLE...) ya da mod adı. |
| **Hz** | yeşil = ≥30 Hz | sönük = <30 Hz | Kontrol döngüsünün **gerçek** frekansı. H1 kabul kriteri: ≥30 Hz. |

### 2.2 ARM VEHICLE

ESC'lere 2 saniye boyunca nötr sinyal gönderip motorları hazırlar (ESC bip sesi
duyarsın). Basınca `DISARM VEHICLE`e döner.

**ARM edilmeden hiçbir motor dönmez.** Görev başlatmak da ARM ister.

### 2.3 EMERGENCY ABORT

Tek tuşla:
1. Tüm motorlar nötre (`thrusters.stop()`),
2. Aktif görev iptal (`mission.abort()`),
3. Teleop eksenleri temizlenir.

Panik butonudur. Havuzda parmağın buranın üstünde dursun.

---

## 3. Sol sütun — FPV ve manuel sürüş

### 3.1 Canlı FPV kanalı

MJPEG akışı (`/video_feed`), 15 FPS, JPEG kalite 60 (`config.py`).
Kamera yoksa ya da OpenCV kurulu değilse "KAMERA GÖRÜNTÜSÜ ARANIYOR..." yazar —
**bu arayüzün geri kalanını etkilemez.**

> Not: Eskiden video akışı başladığında tüm arayüz donuyordu (sunucu tek thread'liydi).
> Artık video kendi thread'inde akıyor; telemetri etkilenmiyor.

### 3.2 AR Göstergeler (onay kutusu)

Görüntünün üstüne bindirilen nişangâh + derinlik (sol alt) + heading (üst orta).
Kapatınca ham görüntüyü görürsün.

### 3.3 Manuel sürüş (klavye)

| Tuş | Eksen | Etki |
|---|---|---|
| `W` / `S` | surge | İleri / geri |
| `A` / `D` | yaw | Sola / sağa dönüş |
| `I` / `K` | heave | Yüksel / dal |
| `SPACE` | — | Tüm eksenler nötr |

Her tuş **0.25** birim ekler (basılı tutmak değeri artırmaz, kombinasyon yapar:
`W`+`D` = ileri + sağa).

Altındaki satır anlık eksen değerlerini gösterir.

**Önemli davranışlar:**
- Bir input kutusuna yazarken tuşlar teleop'a gitmez (yanlışlıkla motor komutu yok).
- Sekme arkaya alınırsa (`blur`) tüm tuşlar bırakılmış sayılır — **motor komutu asılı kalmaz.**
- Teleop komut gönderildiğinde konsola satır yazılmaz (20 Hz'de konsolu doldurmasın diye).

---

## 4. Orta sütun — HUD ve telemetri

### 4.1 HUD / PFD (Primary Flight Display)

Uçak kokpitindeki yapay ufuk göstergesinin denizaltı versiyonu.

| Öğe | Renk | Anlamı |
|---|---|---|
| Üst yarı (mavi) / alt yarı (kahve) | — | Yapay ufuk. Araç yattıkça (**roll**) çizgi döner. |
| Ufuk çizgisi | camgöbeği | 0° referans |
| Merdiven çizgileri (−30°…+30°) | beyaz | **Pitch** kademeleri, 10°'de bir |
| Sabit uçak sembolü (kanatlar + nokta) | sarı | Aracın kendisi — hep merkezde durur |
| Üstteki pusula şeridi | beyaz, N/E/S/W harfleri | **Heading**. Sarı üçgen = mevcut yön |
| Pusula şeridindeki **eflatun** üçgen | eflatun | **Hedef heading** ("bug"). Sarı ile eflatun çakışınca yön hedefindesin. |
| Sağdaki dikey şerit | camgöbeği, 0–10 m | **Derinlik merdiveni**. Sarı ok = mevcut derinlik |
| Derinlik şeridindeki **eflatun** ok | eflatun | **Hedef derinlik** |

Pratik okuma: *eflatun işaretler nereye gitmek istediğini, sarı işaretler nerede
olduğunu gösterir. İkisi çakıştığında hedeftesin.*

### 4.2 Telemetri kartları

| Kart | Büyük değer | Alt satır | Nereden geliyor |
|---|---|---|---|
| **DERİNLİK** | ölçülen derinlik (m) | hedef derinlik | MS5837, `stabilizer.depth_m` |
| **HEADING** | pusula açısı (°) | hedef yön | IMU füzyonu, `stabilizer.heading_deg` |
| **PITCH** | yunuslama (°) | roll (yatış) | IMU |
| **SİSTEM BASINCI** | mbar | su sıcaklığı (°C) | MS5837 |
| **DİKEY HIZ** | m/s (+ = dalıyor) | derinlik hatası (m) | `depth_rate_mps` — PID'in D terimi bunu kullanır |
| **DÖNÜŞ HIZI** | °/s | aktif yön modu | Jiroskop `yaw_rate` |

> Hedef satırında **`—`** görüyorsan o eksende hedef **yok** demektir (PID o ekseni
> tutmuyor). Sayı varsa PID aktif olarak o değeri tutmaya çalışıyor.

**DİKEY HIZ neden önemli:** dalış/çıkış hızını gösterir. Derinlik PID'i sayısal türev
almak yerine bu ölçümü kullanır (gürültü çok daha az). Değer sürekli salınıyorsa
Kd fazla ya da `d_tau` küçük demektir.

### 4.3 Görev 2 sensörleri

Sadece GPS/sonar bağlıyken görünür (`--mission nav`).

- **GPS**: `FIX VAR` / `FIX YOK`, altında enlem-boylam.
- **SONAR MESAFE**: Ping Sonar'ın ölçtüğü mesafe (mm). Şamandıra tespitinde kullanılır.

### 4.4 Motor itki dağılımı

6 motor için üç bilgi:

1. **Çubuk** — komutun büyüklüğü (0–100%).
   - **Camgöbeği** = pozitif yön, **kırmızı** = negatif yön.
2. **Yüzde** — işaretli komut değeri (−100…+100).
3. **µs** — ESC'ye **gerçekten giden** PWM darbesi.

| Motor | Konum |
|---|---|
| `V_FL` / `V_FR` | Dikey ön-sol / ön-sağ |
| `V_RL` / `V_RR` | Dikey arka-sol / arka-sağ |
| `H_L` / `H_R` | Yatay sol / sağ |

**µs değerini neden gösteriyoruz:** normalize komut (%) ile gerçek darbe aynı şey
değildir. Nötr **1470 µs**, aralık **1000–1940 µs**, ölü bant **±30 µs**. Komut
%2 gösterip µs'nin 1470'te (nötr) takılı kalması "ölü banda düştü" demektir —
sadece yüzdeye bakarak bunu fark edemezsin. Aynı şekilde 1940'ta sabitlenmişse
motor doymuştur.

---

## 5. KONTROL MODU — arayüzün en önemli kutusu

**Hedef belirlemenin çalışması buna bağlıdır.**

Görevlerin `step()` metodu her çağrılışta kendi hedefini yazar
(`set_targets(depth_m=...)`). Ana döngü 50 Hz döndüğü için, AUTO modunda senin
verdiğin hedef **20 milisaniye** içinde silinir. Mod anahtarı hedefin *sahibini*
belirler.

| Mod | Hedefin sahibi | Görev çalışıyor mu | Ne zaman kullanılır |
|---|---|---|---|
| **AUTO** | Görev | ✅ Evet | Yarışma. Görev kendi derinlik/yön planını uygular. |
| **HOLD** | **Sen** | ⏸ Duraklatıldı | Hedef verip PID'in tutmasını istediğinde. Adım testleri, PID ayarı. |
| **HOVER** | — (derinlik PID **kapalı**) | ⏸ Duraklatıldı | `FF_HOVER` ölçümü. Sabit dikey gaz verilir. |
| **RATE** | Sen (dönüş hızı) | ⏸ Duraklatıldı | Daire görevi ayarı. Sabit °/s hedefi. |
| **TELEOP** | Sen (eksenler) | ⏸ Duraklatıldı | Doğrudan sürüş. PID devrede değil. |

Kutunun altındaki satır seçili modun ne yaptığını açıklar; sağ üstteki küçük etiket
"hedef sahibi: görev / sen" der.

**AUTO'dayken hedef verirsen** arayüz sessizce yutmaz, konsola şunu yazar:

> `Hata: AUTO modunda hedef görev tarafından eziliyor. Önce HOLD moduna geç.`

**AUTO → HOLD geçişinde** aracın **o anki** derinliği ve yönü hedef yapılır. Yani
HOLD'a basınca araç olduğu yerde kalır — eski görev hedefine doğru fırlamaz.

> ⚠️ HOLD modunda görev durur ama **motorlar durmaz** — PID hedefi tutmaya devam
> eder. Motorları gerçekten durdurmak için `DISARM` ya da `EMERGENCY ABORT`.

---

## 6. Sekme: HEDEF

### 6.1 Derinlik hedefi

- **Kutu + UYGULA**: mutlak hedef (metre).
- **−50cm / −10cm / +10cm / +50cm**: mevcut **ölçülen** derinliğe göre bağıl hedef.
  Havuzda en çok kullanacağın butonlar — "biraz daha aşağı" demenin en hızlı yolu.
- Alt satır: `şu an: X m → hedef: Y m`
- **BIRAK**: hedefi kaldırır (`None`) ve derinlik PID'inin I birikimini sıfırlar.
  Araç artık derinlik tutmaz.

Hedef **0–10 m** arasına kırpılır. Kendi sınırını koymak istersen `config.py`'a
`MAX_DEPTH_M = 3.0` gibi bir satır ekle — sunucu varsa onu kullanır (şu an tanımlı
değil, 10 m varsayılanı geçerli).

Her hedef değişimi **adım kaydını otomatik başlatır** (bkz. §7.5).

### 6.2 Yön (heading) hedefi

Aynı mantık: mutlak kutu, bağıl butonlar (`−90° −10° +10° +90°`), BIRAK.
Değer 0–360 arasına normalize edilir.

**Yön modu** (açılır liste) — kaskad denetleyicinin yetkisini belirler:

| Mod | `w_max` (maks. dönüş hızı) | `out_limit` (motor yetkisi) | Ne için |
|---|---|---|---|
| `cruise` | 15 °/s | 0.35 | Düz seyir. Yavaş ve nazik; rotayı bozmaz. |
| `turn` | 30 °/s | 0.60 | Yerinde 90° dönüş. Hızlı. |
| `circle` | 45 °/s | 0.55 | Daire görevi. Tam yetki. |

> Neden üç mod: düz giderken aracı saniyede 30° döndürmek rotayı bozar. Dairede ise
> düşük yetki iç döngüyü doyurur ve çap "hedeflenen" değil "ulaşılabilen" değere göre
> oluşur — tam da kaçınmak istediğimiz durum.

### 6.3 İleri gaz / dönüş hızı

- **Surge kaydırağı** (−1…+1): HOLD ve RATE modlarında ileri gaz. Kaydırağı
  bıraktığında komut gider (sürüklerken sürekli istek atmaz).
- **Dönüş °/s + UYGULA**: RATE modunda dönüş hızı hedefi (±90 ile sınırlı).

**Daire çapı formülü:** `çap = 2 × ileri_hız / dönüş_hızı`
Örnek: 0.25 m/s ile 24 °/s → yaklaşık 1.2 m çap. Şartname minimumu 1.0 m.

### 6.4 Derinlik kalibrasyonu

- **Hover kaydırağı** (−0.6…+0.6): HOVER modunda sabit dikey gaz.
  **Araç ne çıkıyor ne iniyorsa o değer `FF_HOVER`'dır.** Havuzda ilk ölçülmesi
  gereken şeylerden biri (`config.FF_HOVER` şu an 0.0 = ölçülmedi).
- **FF + UYGULA**: bulduğun asılı kalma gücünü derinlik PID'ine ileri besleme
  olarak verir. PID'in üstüne eklenen sabit itkidir; I teriminin bu işi yapmak
  zorunda kalmamasını sağlar (daha hızlı oturma, daha az kalıcı hata).
- **YÜZEY REFERANSINI SIFIRLA**: araç yüzeydeyken basıncı sıfır referans alır.
  **Her havuz seansında, araç suya girmeden ya da tam yüzeydeyken yapılmalı.**
  Yapılmazsa tüm derinlik ölçümleri kayar.

### 6.5 Güç sınırı ⚠️

- **Thrust** (0.05–1.00): tüm motor çıkışını ölçekler. **İlk havuz testlerinde
  0.3–0.5 arası tut.** Yanlış motor yönü ya da agresif katsayı varsa hasarı sınırlar.
- **Slew** (0.2–10 birim/sn): motor komutunun saniyede ne kadar değişebileceği.
  Düşük = yumuşak, ESC ve güç hattını korur. Yüksek = daha çevik ama sert.

Bu ikisi `config.THRUST_LIMIT` ve `config.SLEW_RATE`'i **canlı** değiştirir —
program yeniden başlatılmaz. (Kalıcı olması için `config.py`'a elle yazman gerekir.)

---

## 7. Sekme: PID

### 7.1 Denetleyici seçimi

| Seçenek | Tip | Notlar |
|---|---|---|
| DERİNLİK | düz PID | `kp=1.5, ki=0.15, kd=0.6` (başlangıç) |
| HEADING | **kaskad** | Dış P (açı→hız) + iç PI (hız→motor). Ekstra alanlar açılır. |
| ROLL | düz PID | `deadzone=2°` — 2°'nin altında hiç karışmaz |
| PITCH | düz PID | aynı |

Sağ üstteki rozet: **senkron** (kutular cihazdakiyle aynı) / **gönderilmedi**
(değiştirdin ama GÜNCELLE'ye basmadın) / **bağlantı yok**.

### 7.2 Canlı hata/çıkış grafiği

| Çizgi | Renk | Ne |
|---|---|---|
| hata | sarı | hedef − ölçülen |
| çıkış | camgöbeği | PID'in ürettiği motor komutu |
| `DOYGUN` yazısı | kırmızı | Çıkış limitine dayandı — PID daha fazlasını isteyip veremiyor |

İki seri **kendi ölçeğinde** çizilir (hata metre, çıkış birimsiz olduğu için).
Şekle bak, mutlak yüksekliğe değil.

**Ne aramalısın:** hata sıfıra düzgün inip orada kalmalı. Sürekli salınım → Kd
fazla ya da Kp fazla. Sıfıra hiç inmiyor → Ki eksik ya da FF eksik. `DOYGUN`
sürekli yanıyorsa PID'in yetkisi yetmiyor (`out_limit`) ya da hedef fiziksel
olarak ulaşılamaz.

### 7.3 P / I / D terim çubukları

Her terimin çıkışa **katkısı**. Orta çizgi sıfır, sağ pozitif (camgöbeği), sol
negatif (kırmızı). `Σ` satırı toplam çıkıştır; doyduğunda kırmızı çerçeve alır.
Ölçek `out_limit`e göredir.

Bu, PID ayarında en öğretici gösterge: *hangi terim işi yapıyor?*

- P hep büyük, I sıfıra yakın → kalıcı hata kalabilir, Ki artır.
- I sürekli tavanda → windup; `i_limit` ya da FF sorunu.
- D sürekli zıplıyor → gürültü; `d_tau` büyüt.

Altındaki satır: sayısal `Hata` ve `Çıkış`. Heading seçiliyse ayrıca
`ω hedef / ölçülen / mod` — kaskadın dış katmanının istediği dönüş hızı, jiroskobun
ölçtüğü hız ve aktif mod.

### 7.4 Kazanç kutuları

- `Kp`, `Ki`, `Kd` — her PID için.
- Heading seçiliyse ek satır: `Kp_pos` (dış katman açı→hız kazancı),
  `w_max` (maks. dönüş hızı), `i_lim` (I birikim sınırı).

**Enter** ya da **GÜNCELLE** ile gönderilir. Boş bırakılan kutu mevcut değeri **ezmez**.

**I SIFIRLA**: I birikimini temizler. Katsayı değiştirdikten sonra eski birikim yeni
katsayının sonucunu maskeler — bu yüzden GÜNCELLE zaten otomatik sıfırlar. Buton,
ayar yapmadan sadece birikimi atmak istediğinde işe yarar.

> Kutular sayfa açılışında **cihazdaki gerçek değerlerle** dolar (`/api/pid`).
> Yazarken üzerine canlı veri yazılmaz.

### 7.5 Adım cevabı testi

PID ayarının asıl aracı. Sisteme ani bir hedef değişimi ("adım") verip cevabı ölçer.

1. Mod **HOLD** olmalı.
2. **Hedef** kutusuna değer yaz → **ADIM VER**.
   (Ya da HEDEF sekmesinden hedef ver — kayıt her iki durumda da otomatik başlar.)
3. Başlıktaki **KAYIT** etiketi kırmızı yanıp söner — kayıt sürüyor (maks. 45 sn).
4. Araç oturunca **SONUCU ANALİZ ET**.

Grafik: **camgöbeği = ölçülen**, **yeşil kesik = hedef**. İkisi aynı ölçekte
çizilir, böylece aşımı gözle görürsün.

**Analiz sonuçları:**

| Değer | Anlamı | İyi |
|---|---|---|
| **Aşım** | Hedefi ne kadar aştı (% ve mutlak) | <%10 yeşil, >%25 kırmızı |
| **Yerleşme %5** | Hedefin ±%5 bandına girip bir daha çıkmadığı an | <6 sn yeşil, "oluşmadı" kırmızı |
| **Kalıcı hata** | Son 5 saniyenin ortalaması − hedef | <0.05 yeşil |
| **Gürültü RMS** | Son 5 saniyenin standart sapması | küçük olsun |

Altında **"ne yapmalı"** önerileri çıkar (derinlik için örnek):

- Aşım >15 cm → *Kd'yi artır, sonra Kp'yi biraz düşür.*
- Yavaş oturuyor → *Kp'yi artır ya da FF eksik.*
- Kalıcı hata >3 cm → *Ki'yi artır ya da FF'i düzelt.*
- Titreşim → *Kd'yi düşür ya da d_tau'yu büyüt.*

> Bu analizin matematiği `pid_tune.py` konsolundakiyle **aynıdır**
> (`control/operator.py::StepRecorder`). Konsol ve web aynı sonucu verir.

---

## 8. Sekme: SAĞLIK

### 8.1 Sensör sağlığı

| Satır | Beklenen | Kırmızıysa |
|---|---|---|
| IMU frekans | ~100 Hz | <40 Hz kırmızı, <80 sarı. I2C hattı tıkalı ya da CPU dolu. |
| IMU veri yaşı | birkaç ms | 500 ms'yi geçerse watchdog görevi keser |
| IMU hata | 0 | Artıyorsa I2C okuma hatası — kablo/adres sorunu |
| Derinlik frekans | ~20 Hz | <8 Hz kırmızı. MS5837 OSR ayarı ya da I2C çakışması. |
| Derinlik veri yaşı | birkaç ms | aynı eşik |
| Derinlik hata | 0 | aynı |

Altındaki kutu: **WATCHDOG: VERİ TAZE** (yeşil) ya da **VERİ BAYAT — GÖREV İPTAL
EDİLİR!** (kırmızı).

**Neden bu panel var:** watchdog sensör verisi bayatladığında görevi iptal edip
motorları nötre çeker. Eskiden bu sessizce oluyordu; operatör *neden* iptal
olduğunu göremiyordu. Artık hangi sensörün düştüğünü anında görürsün.

### 8.2 Kontrol döngüsü

| Satır | Anlamı |
|---|---|
| **Ölçülen** | Ana döngünün gerçek frekansı (hedef 50 Hz) |
| **Hedef** | `config.LOOP_HZ` |
| **En kötü adım** | Şimdiye kadarki en uzun döngü adımı (ms) |
| **Takılma** | 1 saniyeyi aşan duraklama sayısı — **0 olmalı** |
| **Toplam adım** | Döngünün kaç kez döndüğü |
| **Çalışma süresi** | Program ne kadardır açık |

Alttaki kutu: **H1 KABUL KRİTERİ GEÇTİ/KALDI** (eşik 30 Hz).

Düşükse: `config.DEPTH_OSR` düşür, `LOG_EVERY_N` artır, kamera/GCS'yi kapat.

### 8.3 Ham sensör

Jiroskop üç ekseni (°/s), yüzey referans basıncı (mbar), anlık basınç, sıcaklık.
Kalibrasyon doğrulaması ve arıza teşhisi için.

---

## 9. Sekme: GÖREV

### 9.1 Otonom görev

Üç buton görevi **gerçekten başlatır** (eskiden sadece konsola yazı yazıyorlardı):

- **GÖSTERİM VİDEOSU GÖREVİ** — dalış, düz seyir, 90° dönüşler, daire
- **GÖREV 1: HAT TAKİBİ & BORU HİZALAMA**
- **GÖREV 2: OTONOM NAVİGASYON**

Şartlar: **ARM edilmiş olmalı**. Görev başlatınca mod otomatik **AUTO**'ya geçer.
`MISSION.start_delay_s` (10 sn) geri sayım vardır.

**GÖREVİ DURDUR**: görevi iptal eder ve **HOLD** moduna geçer — araç olduğu yerde
kalır, motorlar durmaz.

> Hangi görevlerin başlatılabileceği o an hangi `main.py` komutuyla çalıştığına
> bağlıdır (kamera gerektiren görev, kamera açılmamışsa listede olmaz).

### 9.2 Görev iç durumu

Görevin kendi sayaçları — hangi görevin çalıştığına göre değişir:

| Alan | Anlamı |
|---|---|
| `state` | Durum makinesindeki adım (DIVE, CRUISE, TURN1, CIRCLE...) |
| `yuzey_ihlali` | Yüzeye çıkma sayısı — **H4 kriteri: 0 olmalı** |
| `daire_aci` | Dairede biriken açı (hedef ~370°) |
| `tarama_aci` | Görev 2 tarama açısı |
| `yorunge_aci` | Görev 2 yörünge açısı |

### 9.3 Mini ROV vinç

- **VİNÇ BIRAK / VİNÇ ÇEK** — CLS3860MED servo, ayrı thread'de çalışır.
- **MINI ROV GERİ GELDİ BİLDİR** — Görev 1'de mini ROV'un döndüğünü bildirir,
  görev bir sonraki adıma geçer. (Aktif görev bunu desteklemiyorsa hata verir.)

---

## 10. Canlı log konsolu

Gönderdiğin her komutun sonucu buraya düşer.

| Renk | Anlamı |
|---|---|
| Camgöbeği | Bilgi / başarılı komut |
| Sarı | Uyarı (geçersiz giriş, desteklenmeyen işlem) |
| Kırmızı | Hata (sunucu reddetti, ağ hatası) |

Son 200 satır tutulur. Teleop komutları buraya yazılmaz.

Açılışta HTML'de bulunamayan arayüz elemanı varsa burada uyarı çıkar
(tarayıcı konsolunda listesi olur).

---

## 11. Havuz iş akışı — önerilen sıra

```
1.  Araç suda, yüzeyde, hareketsiz
    └─ SAĞLIK sekmesi: IMU ~100 Hz, derinlik ~20 Hz, watchdog yeşil mi?
    └─ Döngü ≥30 Hz mi?

2.  HEDEF sekmesi → YÜZEY REFERANSINI SIFIRLA
    └─ Derinlik kartı 0.00 m göstermeli

3.  Güç sınırını kıs: Thrust = 0.4

4.  ARM VEHICLE

5.  FF_HOVER ölç:
    └─ Mod: HOVER
    └─ Hover kaydırağını yavaşça artır
    └─ Araç ne çıkıyor ne iniyorsa → o değeri not et
    └─ FF kutusuna yaz → UYGULA

6.  Derinlik PID ayarı:
    └─ Mod: HOLD
    └─ PID sekmesi → DERİNLİK seç
    └─ Hedef 1.0 → ADIM VER → oturmasını bekle → SONUCU ANALİZ ET
    └─ Öneriye göre Kp/Ki/Kd değiştir → GÜNCELLE → tekrar adım ver
    └─ Aşım <15 cm, yerleşme <6 sn, kalıcı hata <3 cm olana kadar

7.  Yön PID ayarı:
    └─ Yön modu: turn
    └─ PID sekmesi → HEADING seç
    └─ HEDEF sekmesi → +90° → ADIM VER → analiz
    └─ Dış katman kp_pos ve iç döngü Kp/Ki ile oyna

8.  Daire ayarı:
    └─ Yön modu: circle, Mod: RATE
    └─ Surge 0.25, Dönüş 24 °/s
    └─ P/I/D çubuklarında Σ DOYGUN olmamalı
    └─ Çapı ölç, formülle doğrula

9.  Güç sınırını 1.0'a çıkar, görev denemesi:
    └─ GÖREV sekmesi → görev başlat
    └─ Mod AUTO'ya geçer

10. Her denemeden sonra:
    python3 tools/analyze_log.py <log dosyası>
```

**Bulduğun katsayıları `config.py`'a elle işlemeyi unutma** — arayüzden yapılan
değişiklikler program kapanınca kaybolur.

---

## 12. Sorun giderme

| Belirti | Muhtemel sebep | Ne yapmalı |
|---|---|---|
| `BAĞLANTI KESİLDİ` | Program durdu, ağ koptu | Jetson'da program çalışıyor mu, tether takılı mı |
| Hedef veriyorum değişmiyor | Mod **AUTO** | **HOLD**'a geç (konsolda uyarı yazar) |
| Motorlar hiç dönmüyor | DISARMED | ARM VEHICLE |
| Görev başlamıyor | ARM edilmemiş | Önce ARM |
| `SENSÖR BAYAT!` | I2C hattı / sensör thread'i düştü | SAĞLIK sekmesinde hangisi kırmızı bak |
| Döngü <30 Hz | CPU dolu, sensör bloklıyor | `DEPTH_OSR` düşür, `LOG_EVERY_N` artır |
| Kamera yok | OpenCV kurulu değil / kamera bağlı değil | Arayüzün geri kalanı çalışır, sorun değil |
| Derinlik yanlış | Yüzey referansı sıfırlanmamış | YÜZEY REFERANSINI SIFIRLA |
| Araç titriyor | Kd fazla | Kd düşür, `d_tau` büyüt |
| Hedefe ulaşmıyor | Ki ya da FF eksik | Analiz önerisine bak |
| `DOYGUN` sürekli yanıyor | PID yetkisi yetmiyor | `out_limit` artır ya da hedef ulaşılamaz |
| Motor % var ama µs nötr | Ölü banda düştü | `PWM_DEADBAND_US` / `DEADBAND_COMPENSATION` |

---

## 13. API referansı (ileri seviye)

Arayüzün kullandığı uç noktalar — kendi script'inden de çağırabilirsin.

| Uç nokta | Metod | Ne döner |
|---|---|---|
| `/api/telemetry` | GET | Tüm canlı telemetri (JSON) |
| `/api/pid` | GET | Tüm PID kazançları + son adım terimleri |
| `/api/step` | GET | Adım cevabı eğrisi (seyreltilmiş) |
| `/api/config` | GET | Modlar, görevler, sabitler |
| `/api/command` | POST | Komutlar (aşağıda) |
| `/video_feed` | GET | MJPEG akışı |

**Komutlar** (`POST /api/command`, gövde JSON):

```
{"cmd":"arm"} / {"cmd":"disarm"} / {"cmd":"abort"}
{"cmd":"set_mode","mode":"HOLD"}
{"cmd":"set_target","depth":1.5}
{"cmd":"set_target","heading_rel":90}
{"cmd":"clear_target","axis":"depth"}
{"cmd":"set_pid","pid_name":"depth","kp":1.5,"ki":0.15,"kd":0.6}
{"cmd":"reset_pid","pid_name":"depth"}
{"cmd":"set_rate","rate":24}      {"cmd":"set_surge","surge":0.25}
{"cmd":"set_hover","hover":0.2}   {"cmd":"set_ff","ff":0.18}
{"cmd":"heading_mode","mode":"circle"}
{"cmd":"zero_depth"}
{"cmd":"set_limits","thrust_limit":0.4,"slew_rate":3.0}
{"cmd":"step_start","kind":"depth"} / {"cmd":"step_stop"} / {"cmd":"analyze"}
{"cmd":"mission_start","mission":"video"} / {"cmd":"mission_stop"}
{"cmd":"teleop","surge":0.2,"yaw":0,"heave":0} / {"cmd":"teleop_off"}
{"cmd":"winch_deploy"} / {"cmd":"winch_retract"} / {"cmd":"minrov_back"}
```

Örnek:

```bash
curl -s localhost:8000/api/telemetry | python3 -m json.tool
curl -s -X POST localhost:8000/api/command \
     -H 'Content-Type: application/json' \
     -d '{"cmd":"set_mode","mode":"HOLD"}'
```

---

## 14. İlgili dosyalar

| Dosya | Ne var |
|---|---|
| `rov/gcs/index.html` | Arayüz iskeleti |
| `rov/gcs/js/app.js` | Tüm arayüz mantığı, panel modülleri |
| `rov/gcs/js/hud.js` | HUD/PFD canvas çizimi |
| `rov/gcs/css/gcs.css` | Görsel stil |
| `rov/comms/web_server.py` | HTTP sunucu, telemetri, komut API, demo modu |
| `rov/control/operator.py` | Kontrol modları + adım cevabı analizi |
| `rov/main.py` | Ana kontrol döngüsü |
| `rov/config.py` | Tüm katsayılar ve limitler |
| `DEGISIKLIKLER.md` §7 | Bu arayüzde ne değişti ve neden |

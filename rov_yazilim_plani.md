# EGE ROV — 20 Günlük Yazılım Sprint Planı
Hedef: Otonom video gösterimi görevi (yarışmaya katılım ön koşulu)

## Görev Tanımı (şartname 2.4.3.3, İleri Kategori)
Araç bataryalı, tamamen otonom, dış müdahalesiz, tamamı su altında:

1. Min. 15 sn düz ileri gidiş
2. Sağa 90° dönüş
3. Min. 15 sn düz gidiş
4. Min. 1 m çapında daire — en az 1 tam tur
5. Min. 15 sn düz gidiş
6. Sağa 90° dönüş
7. Min. 15 sn düz gidiş
8. Aracın **tamamı** 1×1 m başlangıç alanı içinde bitirmeli

Ek zorunluluklar: acil durdurma butonu videoda gösterilecek (tüm motorlar durmalı), video kesintisiz 1–5 dk, 720p+, su yüzeyine hiç çıkılmayacak, güvenlik halatı serbest.

## Kritik Gözlemler
- Rota kapalı bir dikdörtgen: kenar süreleri eşit olursa (15'er sn, sabit hız) araç teorik olarak başa döner. En büyük hata kaynağı: dönüşlerdeki sapma ve akıntı/sürüklenme. → **heading PID şart**, süre tutmak yetmez.
- Daire köşede çizildiği için rotayı bozmaz (aynı noktadan devam ediliyor) — daire sonunda **daire öncesi heading'den 90° sağa** dönmüş olarak çıkmak yeterli. En temiz yöntem: daireyi tam 360° jiroskopla sayıp bitişte hedef heading'e kilitlenmek.
- 1×1 m alana "aracın tamamı" sığmalı → araç ~60–90 cm ise hata payı santimetrelerle ölçülür. **Süreleri 15 sn'de tutmak (fazla uzatmamak) sürüklenme hatasını azaltır.**
- Derinlik sabit tutulmalı (yüzeye çıkma = eleme). Derinlik PID + güvenli hedef derinlik (örn. 0.5–1 m).
- Havuzda çekilebilir → test ve çekim aynı yerde yapılabilir, deneme sayısı sınırsız. Bu büyük avantaj.

## Gerekli Yazılım (sadece bu görev için)
Kameraya, hat takibine, yer istasyonuna **gerek yok**. Minimum sistem:

```
main.py               # başlat butonu/komutu → görev, acil durdurmayı izle
config.py             # PWM kanalları, PID katsayıları, süreler, hedef derinlik
hal/thrusters.py      # ESC PWM sürücü, arm/disarm, nötr, failsafe
control/mixer.py      # [ileri, dönüş, dalış, roll, pitch] → 6 motor
control/pid.py        # PID sınıfı
sensors/imu.py        # heading + açısal hız (pusulalı IMU şart!)
sensors/depth.py      # basınç → derinlik
control/stabilizer.py # derinlik PID + heading PID + roll/pitch dengeleme
missions/video_demo.py# durum makinesi (aşağıda)
utils/logger.py       # her testin kaydı (ayar için hayati)
```

### video_demo.py durum makinesi
```
BEKLE → (başlat) → DAL (hedef derinliğe in, heading₀ kaydet)
→ DÜZ1 (15 sn, heading₀) → DÖN1 (heading₀+90°)
→ DÜZ2 (15 sn) → DAİRE (jiroskopla 360° say, sabit dönüş+ileri, çıkışta heading₀+180°)
→ DÜZ3 (15 sn) → DÖN2 (heading₀+270°)
→ DÜZ4 (15 sn) → DUR (motorlar nötr, yüzeye çıkma opsiyonel değil—alanda su altında dur)
```
Her düz segment: heading PID + derinlik PID + sabit ileri gaz.
Her dönüş: ileri gaz sıfır, heading hedefe kilitlen (tolerans ±5°), sonra bekle-sabitle.

## 20 Günlük Takvim

| Gün | İş | Çıktı |
|---|---|---|
| 1–2 | Donanım netleştirme + HAL: ESC sürme (PCA9685 vb.), arm/nötr/limitler, acil durdurma testi | Motorlar komutla dönüyor |
| 3–4 | `mixer.py` + `pid.py` + `config.py` (masaüstünde birim testli) | Eksen komutları → 6 motor doğru karışıyor |
| 5–6 | Sensörler: IMU heading okuma + kalibrasyon, derinlik sensörü okuma | Stabil heading ve derinlik verisi |
| 7–9 | `stabilizer.py`: önce derinlik PID, sonra heading PID — **ilk havuz testi (gün 8–9)** | Araç sabit derinlikte heading tutuyor |
| 10–12 | PID ayarı (havuzda), roll/pitch dengeleme, loglama | Düzgün 15 sn düz gidiş |
| 13–15 | `video_demo.py` durum makinesi + kuru/havuz testleri | Tam dizi otonom dönüyor |
| 16–18 | Tam prova: başlangıç alanına dönüş hassasiyeti için süre/hız ince ayarı, acil durdurma gösterimi provası | Ardışık 3 başarılı tam tur |
| 19–20 | Video çekimi (birkaç deneme) + yükleme | Teslim ✔ |

**Yedek pay:** gün 16–18'deki provalar aynı zamanda tampon. Havuz erişimi hangi günler mümkünse testler ona göre kaydırılmalı — kodun havuzsuz yazılabilen kısımları (1–6) öne yığıldı.

## Riskler ve Önlemler
- **IMU yoksa/pusulasız ise bu görev güvenilir yapılamaz.** İlk iş IMU'yu netleştirmek (öneri: BNO055/BNO085 — kalibreli heading verir). Jiroskop-entegrasyonu kısa görevde iş görür ama pusulalı olan çok daha sağlam.
- **ESC sürme yöntemi belirsiz** (Jetson donanımsal PWM sınırlı) → PCA9685 (I2C, 16 kanal) en hızlı çözüm; gün 1'de karar.
- Başlangıç alanına dönememe → süreleri minimumda tut, düz segment hızlarını eşitle, gerekirse son segmentte süreyi logdan kalibre et (gidilen mesafe ≈ hız×süre simetrisi).
- Batarya zorunlu (kablo yok) → görev başlatma araç kapalıyken ayarlanmalı: ör. güç verilince 10 sn geri sayım + LED, ya da su geçirmez buton/manyetik anahtar.

## Sonrası (video tesliminden sonra, yarışma parkurları)
1. `comms/teleop.py` — tether üzerinden manuel sürüş (Görev 1'in Mini ROV kısmı için)
2. `missions/line_follow.py` — kamera + OpenCV otonom hat takibi (40 puan)
3. `missions/autonomous_nav.py` — şamandıra/koordinat görevi (ölü hesap + kamera teyidi)

# PID ve ROV Kontrolü — Sıfırdan, Basit Anlatım

> Bu doküman `PID_TASARIM_PLANI.md` içindeki her şeyi teknik terim kullanmadan anlatır.
> Hiçbir şey atlanmadı, sadece sadeleştirildi.

---

# BÖLÜM 1 — PID nedir?

## 1.1 Duş metaforu

Duşa giriyorsun. Su soğuk. Musluğu sıcağa doğru çeviriyorsun. Ne kadar çevireceksin?

- **Çok az çevirirsen:** su ısınması saatler sürer.
- **Çok fazla çevirirsen:** su kaynar, geri çevirirsin, bu sefer buz olur, tekrar çevirirsin...
  Sürekli sıcak-soğuk gidip gelirsin. Buna **salınım** denir.
- **Doğru miktarda çevirirsen:** su hızlıca istediğin sıcaklığa gelir ve orada kalır.

PID tam olarak bunu yapan bir formüldür. "Şu an neredeyim, nerede olmak istiyorum,
o halde motoru ne kadar çalıştırayım?" sorusunun cevabıdır.

Bizim durumda:
- **İstediğim:** 0.6 metre derinlik
- **Şu an:** 0.3 metre derinlik
- **Hata:** 0.3 metre (daha derine inmem lazım)
- **PID'in cevabı:** "dikey motorları %45 güçle aşağı çalıştır"

## 1.2 Üç harf, üç farklı soru

PID = **P** + **I** + **D**. Üçü aynı anda çalışır, toplanır, sonuç motora gider.

### P — "Şu an ne kadar uzaktayım?"

En basit olan. Hata ne kadar büyükse motoru o kadar çok çalıştır.

```
P çıktısı = Kp × hata
```

`Kp` senin ayarladığın sayı. Büyük Kp = agresif tepki, küçük Kp = uysal tepki.

- **Kp küçükse:** araç hedefe çok yavaş yaklaşır, hatta hiç ulaşamaz.
- **Kp büyükse:** araç hedefi geçer, geri döner, tekrar geçer → salınır.

**Tek başına P'nin sorunu:** Aracımız suda yüzüyor (batmıyor). 0.6 metrede kalmak için
motorların **sürekli** aşağı itmesi lazım. Ama P diyor ki: "hata sıfırsa motor sıfır."
Motor durunca araç yükselir, hata oluşur, motor çalışır, araç iner, hata sıfırlanır, motor durur...
Sonuç: araç asla tam 0.6'da durmaz, hep biraz yukarıda takılır. Buna **kalıcı hata** denir.

### I — "Ne kadar zamandır uzaktayım?"

I, hatayı **biriktirir**. Hata küçük ama uzun süredir devam ediyorsa, I yavaş yavaş büyür
ve motoru daha çok çalıştırır. Yukarıdaki "hep 5 cm yukarıda takılma" sorununu I çözer.

```
I çıktısı = Ki × (hataların zaman içindeki toplamı)
```

- **Ki küçükse:** kalıcı hata uzun süre kalır.
- **Ki büyükse:** araç hedefi aşar ve salınır (I geç tepki verir, geç de bırakır).

### D — "Ne kadar hızlı yaklaşıyorum?" (fren)

D, hatanın **değişim hızına** bakar. Araç hedefe çok hızlı geliyorsa D "yavaşla, çarpacaksın"
der ve motoru ters yönde kısar.

Arabayı park ederken duvara yaklaşırken frene basman gibi. Duvara olan mesafeye değil,
**yaklaşma hızına** göre fren yaparsın.

- **Kd yoksa:** araç hedefi aşar (overshoot).
- **Kd büyükse:** araç aşırı temkinli olur, ayrıca sensör gürültüsünü büyütür → motorlar titrer.

### Özet

| Terim | Neye bakar | Ne işe yarar | Fazlası ne yapar |
|---|---|---|---|
| **P** | Şu anki hata | Hedefe götürür | Salınım |
| **I** | Geçmişteki hata birikimi | Kalıcı hatayı siler | Aşım + gecikmeli salınım |
| **D** | Hatanın değişim hızı | Fren, aşımı engeller | Titreme (gürültüye duyarlı) |

**Ayar sırası her zaman:** önce P (hareket etsin), sonra D (aşmasın), en son I (tam otursun).
Bu yüzden havuz protokolünde Adım 3=Kp, Adım 4=Kd, Adım 5=Ki sırası var.

---

# BÖLÜM 2 — Bizim aracımız neyi kontrol ediyor?

Aracın 6 motoru var:
- **2 yatay motor** (`H_L` sol, `H_R` sağ) → ileri gitmek ve sağa/sola dönmek
- **4 dikey motor** (`V_FL`, `V_FR`, `V_RL`, `V_RR`) → dalmak, çıkmak, dengede durmak

Bunlardan **3 ayrı PID** çalışıyor (aslında 4, ama roll ve pitch aynı mantık):

| Kontrol | Soru | Hangi sensör | Hangi motorlar |
|---|---|---|---|
| **Derinlik** | Kaç metre derindeyim? | Basınç sensörü (MS5837) | 4 dikey |
| **Yön (heading)** | Pusulada kaç dereceyim? | IMU / pusula (MPU-9250) | 2 yatay (biri ileri biri geri = dönüş) |
| **Roll / Pitch** | Yan yatmış mıyım? Burnum yukarıda mı? | IMU (ivmeölçer) | 4 dikey (biri fazla biri az) |

### "mixer" ne iş yapıyor?

PID'ler motor değil, **istek** üretir: "ileri git", "sağa dön", "aşağı in".
`mixer.py` bu istekleri 6 motorun her birinin ne kadar döneceğine çevirir.

```
"ileri git"  →  H_L ve H_R ikisi de ileri
"sağa dön"   →  H_L ileri, H_R geri
"aşağı in"   →  4 dikey motor da aşağı
"sağa yat"   →  soldaki dikeyler fazla, sağdakiler az
```

Bunlar aynı anda gelir ve toplanır. Örneğin "ileri git + hafif sağa dön" istendiğinde
`H_L = 0.35 + 0.1 = 0.45`, `H_R = 0.35 - 0.1 = 0.25` olur.

---

# BÖLÜM 3 — Sensörler ne yapıyor, nasıl bozuluyor?

## 3.1 Basınç sensörü (derinlik)

Su ne kadar derinse üstündeki su o kadar ağırdır, basınç o kadar yüksektir.
Sensör basıncı ölçer, formülle metreye çevirir. Basit ve güvenilir.

**Önemli:** Suya girmeden önce "yüzeydeki basınç" kaydedilir (`zero_at_surface`).
Derinlik = şimdiki basınç − yüzey basıncı. Bu referans yanlışsa tüm derinlik ölçümü kayar.

## 3.2 IMU (yön, eğim)

IMU içinde 3 ayrı sensör var, üçü de kusurlu, bu yüzden birleştiriliyor:

**a) Jiroskop** — "şu an saniyede kaç derece dönüyorum?" der.
- ✅ Çok hızlı ve hassas, ani dönüşleri mükemmel yakalar.
- ❌ **Sürüklenir.** Açıyı bulmak için dönüş hızını sürekli toplarsın. Ufacık bir ölçüm
  hatası bile toplana toplana büyür. 1 dakika sonra "hiç dönmedim" derken 20° dönmüş sanır.

**b) İvmeölçer** — yerçekimini hisseder, "hangi taraf aşağı?" der → yan yatma açısı.
- ✅ Sürüklenmez, uzun vadede hep doğru.
- ❌ Yavaş ve gürültülü; araç hızlanınca yerçekimini ivmeden ayırt edemez.

**c) Manyetometre (pusula)** — dünyanın manyetik alanını ölçer → kuzey nerede.
- ✅ Sürüklenmez.
- ❌ Gürültülü, metalden/motordan etkilenir.

**Tamamlayıcı filtre** bunları birleştirir:
> "Kısa vadede jiroskopa güven (hızlı ve pürüzsüz), uzun vadede pusulaya güven (sürüklenmez).
> Jiroskobun toplayarak biriktirdiği hatayı yavaşça pusulaya doğru düzelt."

Koddaki `HEADING_FILTER_ALPHA = 0.98` bu güven oranı: %98 jiroskop, %2 pusula.

## 3.3 Kalibrasyon nedir, neden şart?

Hiçbir sensör mükemmel değildir. Hareketsiz duran bir jiroskop "0" yerine "0.4" der.
Buna **bias** (sapma) denir. Kalibrasyon = "araç kesinlikle hareketsizken sensör ne diyor?"
diye ölçüp, o değeri her ölçümden çıkarmak.

- `GYRO_BIAS` → araç hareketsizken jiroskobun okuduğu sahte dönüş
- `ACCEL_BIAS` → araç düz dururken ivmeölçerin sahte eğimi
- `MAG_OFFSET/SCALE` → pusulanın metal parçalardan etkilenme düzeltmesi

---

# BÖLÜM 4 — Bulduğum sorunlar, tek tek

## SORUN 1 — Kalibrasyon değeri fiziksel olarak imkânsız 🔴

```python
ACCEL_BIAS = (2.0, -2.0, -0.296)
```

**Basitçe:** İvmeölçer sensörümüz en fazla **2g** ölçebiliyor (g = yerçekimi birimi).
Sensörün **düzeltme değeri**, ölçebildiği tüm aralık kadar. Bu, bir cetvele
"her ölçümünden 30 cm çıkar" demek gibi — 30 cm'lik cetvelde.

**Nasıl olmuş:** Kalibrasyon yapılırken araç hareketsiz değildi, sallanıyordu. Sensör
doyuma girdi (ölçebileceği maksimuma yapıştı) ve o değer "sapma" diye kaydedildi.

**Sonucu:** Araç dümdüz dururken program "57 derece yan yatmışım" sanıyor.
Bunu hesapladım ve **gerçek testin logunda roll tam olarak −57.06°'ye oturmuş.**
Virgülden sonraki basamağa kadar aynı. Yani tahmin değil, kesin.

**Bu bir sorun zinciri başlatıyor:**

1. Roll/pitch PID'leri "57 derece yatmışım, düzeltmeliyim!" diye 4 dikey motoru
   sürekli birbirine karşı çalıştırıyor. Boşuna güç harcanıyor.
2. Pusula hesabı, aracın eğimini bilmek zorunda (yan yatmış bir pusula yanlış okur —
   buna **eğim telafisi** denir). Eğim bilgisi sahte olduğu için **pusula da sahte oluyor.**
   Logda heading 10 saniyede 154° → 98° kaydı; sahte açı yerine oturdukça kayma durdu.

**Yani:** roll/pitch bozukluğu ve heading kayması **ayrı iki arıza değil.**
İkisi de bu tek kalibrasyon hatasından geliyor. İyi haber — tek düzeltme ikisini birden çözüyor.

**Çözüm:** `calibrate_imu.py`'yi araç **kımıldamadan, düz zeminde** tekrar çalıştır.
Ayrıca scripte bir kontrol ekliyorum: "hesaplanan sapma 0.3g'den büyükse kaydetme, hata ver."
Böylece bir daha bozuk değer config'e yazılamaz.

---

## SORUN 2 — Program saniyede 50 kez değil, 7 kez karar veriyor 🔴

**Basitçe:** Araba kullanırken yola ne sıklıkla bakarsın? Saniyede birkaç kez.
Peki 7 saniyede bir baksan? Şeritte kalabilir misin?

Programın "yola bakma" hızına **kontrol döngü frekansı** denir. Hedefimiz saniyede
50 kez (50 Hz). Loglardan ölçtüm: gerçek donanımda **saniyede 7.3 kez.**

**Neden:** Basınç sensörünü okumak 40 milisaniye sürüyor — çünkü sensörden en yüksek
hassasiyet isteniyor ve o hassasiyet için sensörün "düşünme" süresi uzun.
Bu 40 ms boyunca **program tamamen duruyor**, başka hiçbir şey yapmıyor.

Daha kötüsü: bu okuma **her döngüde 3 ayrı yerden** yapılıyor:
1. Stabilizer PID hesabı için okuyor
2. Görev kodu "hedef derinliğe geldik mi?" diye tekrar okuyor
3. Log kaydedici yazmak için bir kez daha okuyor

3 × 40 ms = 120 ms bekleme. Ölçülen 136 ms ile birebir uyuyor.

**Bu neden çok kötü:**
- **D terimi anlamsızlaşıyor.** D "ne kadar hızlı değişiyorum" der; 7 Hz'de ölçüm arası
  o kadar uzun ki değişim hızını doğru hesaplayamıyor.
- **Gecikme.** Araç hedefi geçtikten 136 ms sonra haberin oluyor. Bu sürede araç yol almış oluyor.
  Gecikme, PID'de **her zaman salınıma** yol açar — aynı Kp değeri hızlı döngüde stabilken
  yavaş döngüde salınır.
- **Düzensiz aralık.** Bazen 88 ms, bazen 138 ms. PID matematiği düzenli aralık varsayar.

**Çözüm — 3 parça:**

1. **Sensörden daha az hassasiyet iste.** Şu an 8192 kat hassasiyet isteniyor (40 ms).
   1024 kat istersek 2.3 ms sürüyor ve çözünürlük **2 milimetre** oluyor.
   Bizim toleransımız 5 santimetre — 2 mm fazlasıyla yeter. **17 kat hızlanma, bedava.**

2. **Sensör okumayı ayrı bir "çalışana" ver (thread).**
   Şu an: aşçı hem yemek yapıyor hem sürekli fırına bakmaya gidiyor, o sırada tezgah duruyor.
   Yeni: bir kişi sürekli fırına bakıp tahtaya son sıcaklığı yazıyor; aşçı sadece tahtaya bakıyor.
   Kontrol döngüsü **hiç beklemiyor**, en son yazılan değeri okuyup devam ediyor.

3. **Döngüde tek okuma.** Sensör bir kez okunur, değeri saklanır, ihtiyacı olan herkes
   (PID, görev kodu, log) o saklanan değeri kullanır.

---

## SORUN 3 — Motorların "boşluğu" küçük komutları yutuyor 🟠

**Basitçe:** Eski arabaların direksiyonunda boşluk olur — birkaç santim çevirirsin,
tekerlek kımıldamaz, sonra birden tutar. Bizim motorlarda da öyle bir boşluk var.

Kodda `PWM_DEADBAND_US = 30` var. Bu kasıtlı konmuş, iyi bir sebeple: motorlar sıfıra çok
yakın komutlarda titrer ve bip sesi çıkarır. O yüzden "çok küçük komutları sıfır say" deniyor.

**Ama hesabı yapılmamış.** Toplam komut aralığımız 470 birim, boşluk 30 birim.
Yani **komutun %6.4'ünden küçük her şey sıfıra düşüyor.**

Şimdi bunu PID katsayılarıyla birleştirelim:

| Eksen | Kp | İtki üretmek için gereken **en az** hata |
|---|---|---|
| Yön (heading) | 0.02 | **3.2 derece** |
| Roll | 0.01 | **6.4 derece** |
| Pitch | 0.01 | **6.4 derece** |

**Ne demek bu:** Araç 5 derece yan yatmışsa, roll PID'i "düzelt" komutu üretiyor
ama komut boşluğun içinde kaldığı için **motorlara hiçbir şey gitmiyor.**
Roll/pitch kontrolü pratikte **hiç çalışmıyor.** 6.4 dereceyi geçince de aniden itki
başlıyor — araç sarsıntılı şekilde bir o yana bir bu yana gidiyor.

Yön kontrolünde de son 3.2 derece "ölü". Dönüş tam oturmuyor, hep biraz kayık kalıyor.

**Çözüm — boşluk telafisi:** Komut sıfırdan farklıysa, motora göndermeden önce
komutu boşluğun hemen üstüne çıkar.

```
Komut 0 ise           → motora 0 gönder (gerçekten dur)
Komut 0.01 ise        → motora 0.074 gönder (boşluğu atla, hafif it)
Komut 0.50 ise        → motora 0.53 gönder
Komut 1.00 ise        → motora 1.00 gönder
```

Böylece "en küçük komut bile bir etki yaratır" garantisi olur ve komut ile itki arasındaki
ilişki **düz bir çizgi** haline gelir. PID'in varsaydığı da tam olarak budur.

---

## SORUN 4 — Derinlik kontrolünün 3 ayrı eksiği 🟠

### 4a) Yukarı çıkışı göremiyor

Koddaki satır:
```python
return max(0.0, ...)   # "negatifse sıfır yaz"
```

Araç referans yüzeyin üstüne çıkarsa derinlik negatif olur. Bu satır negatifi 0 yapıyor.

**Sorun:** Araç 0.6 metreyi aşıp yukarı fırladığında program "0 metredeyim" diyor ve
orada donuyor. Ne kadar yükseldiğini **göremiyor**, dolayısıyla ne kadar sert düzeltmesi
gerektiğini de bilemiyor. Yüzeye çıkmak bizde **eleme sebebi** — bu körlük kabul edilemez.

**Çözüm:** Kontrol için gerçek (negatif olabilen) değer kullanılacak.
Ekranda göstermek için istersen sıfırla, ama PID gerçeği görsün.

### 4b) İleri besleme (feed-forward) yok

**Basitçe:** Aracımız su üstüne çıkmak ister (kasıtlı olarak — arıza olursa yüzsün diye).
Yani 0.6 metrede **durmak** için bile motorların sürekli aşağı itmesi lazım.

Şu an bu sürekli itkiyi **sadece I terimi** sağlayabiliyor. Ama I yavaş birikir ve
üst sınırı var. Sonuç: araç önce yükselir, I birikir, sonra düzelir — her seferinde.

**İleri besleme çözümü:** Bunu zaten biliyoruz, ölçebiliriz! Havuzda 3 dakikada
"araç hangi motor gücünde ne çıkıyor ne iniyor?" diye ölçeriz, o sayıyı `FF_hover`
olarak yazarız. Sonra formül şöyle olur:

```
motor komutu = FF_hover (bilinen sabit itki)  +  PID (sadece artık hatayı düzelt)
```

PID'in işi kolaylaşır, tepki hızlanır, I terimi sadece küçük düzeltmeler yapar.

> **Analoji:** Bisikletle yokuşta duruyorsun. Her seferinde "geriye kaydım, pedal çevireyim"
> demek yerine, en baştan yokuşu tutacak kadar sabit güç uygularsın; pedal sadece
> ince ayar yapar. FF budur.

### 4c) Windup (birikim taşması)

**Basitçe:** Arabanın çamura saplandığını düşün. Gaza sonuna kadar basıyorsun,
araba kımıldamıyor. Sen "daha da bas" diye baskıyı artırmaya devam ediyorsun.
Sonra araba birden kurtuluyor — ve fırlıyor.

I terimi tam olarak bunu yapar: motor zaten **sonuna kadar açıkken** hata devam ettiği
sürece I birikmeye devam eder. Sonra araç hedefe gelir, ama I o kadar şişmiştir ki
motor hâlâ tam güçte iter → araç hedefi fena halde aşar.

Logda tam bunu görüyorum: `heave = 1.0` (tam güç) **10 saniye boyunca** sabit.

**Çözüm — koşullu integrasyon:** "Motor zaten sonuna kadar açıksa ve hata hâlâ aynı
yöndeyse, I'yi biriktirme — nasılsa daha fazla veremiyorum." Bir satırlık kural,
aşımın büyük kısmını çözer.

---

## SORUN 5 — Motor yön tablosu güvenilmez 🔴

`config.py` içinde her motor için `+1` veya `-1` yazan bir tablo var:
"bu motor ters bağlanmış, komutu ters çevir."

Şu an **hepsi −1**. Ve `config.py`'nin kendi yorumu bunu itiraf ediyor:
bu ölçüm, ayrı bir PWM frekans hatası varken yapılmış. O sırada **her motor her testte
geri dönüyordu**, o yüzden hepsine −1 yazılmış. Frekans hatası düzeltildi,
ama tablo düzeltilmedi.

**Bu neden bu kadar kritik:** PID'in tüm mantığı şuna dayanır:
> "Çok yukarıdayım → aşağı it → aşağı inerim → hata azalır."

İşaret tersse:
> "Çok yukarıdayım → aşağı it komutu veriyorum ama motor **yukarı** itiyor →
> daha da yukarı çıkıyorum → hata büyüyor → daha sert itiyorum → daha da yukarı..."

Buna **pozitif geri besleme** denir ve sistem saniyeler içinde patlar.
**Hiçbir Kp/Ki/Kd değeri bunu düzeltemez.** Bu yüzden PID'e dokunmadan önce
kuru ortamda her motor tek tek elle doğrulanacak.

**Nasıl doğrulanacak:**
1. Her motoru tek tek çalıştır, pervanenin hangi yöne su ittiğini elle hisset/gözle.
2. Sonra eksen testi: "aşağı in" komutu ver → 4 dikey motorun hepsi suyu **yukarı**
   atmalı (ki araç aşağı gitsin). "İleri git" → 2 yatay motor suyu geriye atmalı.

---

## SORUN 6 — Daire sayacı gürültüyü tur sanıyor 🟠

Daire görevinde "tam 360 derece döndüm mü?" sorusunu jiroskopla cevaplıyoruz:
her an dönüş hızını okuyup topluyoruz.

Koddaki satır:
```python
self._circle_acc += abs(yaw_rate) * dt
```

`abs()` = "mutlak değer", yani eksi işaretini atıp hep artı yapıyor.

**Sorun:** Jiroskop hiç dönmeyen bir araçta bile gürültü yüzünden bir an +0.5, bir an −0.5 okur.
Normalde bunlar birbirini götürür, toplam sıfır kalır. Ama `abs()` eksileri de artı yaptığı için
**gürültü toplanmaya başlıyor.** ~0.5 derece/sn gürültü, 40 saniyelik dairede
**~20 derece sahte dönüş** üretir.

Daha kötüsü: araç yanlışlıkla ters yöne saparsa, sayaç yine **ileri** gider.
Yani ters dönüş bile "tur tamamlandı" sayılır.

**Çözüm:** `abs()` kaldırılacak (işaretli toplama) + 1 derece/sn altındaki okumalar
gürültü sayılıp yok sayılacak. Ayrıca daireye her girişte sayaç sıfırlanacak.

---

## SORUN 7 — PID kodunun kendisindeki 6 eksik 🟡

Bunlar tek tek küçük ama birikince ayar yapmayı imkânsızlaştırıyor.

### 7a) "Setpoint kick" — hedef değişince ani gaz

D terimi hatanın değişim hızına bakıyor. Sen hedefi 0° iken aniden 90° yapınca,
hata bir anda 90 derece "sıçrıyor". D bunu "araç inanılmaz hızlı hareket ediyor!"
diye yorumluyor ve devasa bir fren komutu üretiyor. Araç sarsılıyor.

Ama araç hareket etmedi — **sen hedefi değiştirdin.**

**Çözüm:** D, hataya değil **ölçüme** baksın. Ölçüm (aracın gerçek açısı) sıçramaz,
sürekli değişir. Hedef değişince D hiçbir şey hissetmez. Bizim her dönüşte hedef
değiştiğimiz için bu düzeltme kritik.

### 7b) D filtresi düzensiz aralıkta bozuluyor

D terimi gürültülü olduğu için yumuşatılıyor (filtre). Ama mevcut filtre
"her ölçümde şu kadar yumuşat" diyor — geçen **süreye** bakmıyor.
Döngü bazen 88 ms bazen 138 ms sürdüğü için filtre her seferinde farklı davranıyor.

**Çözüm:** Filtreyi süreye bağla. "0.15 saniyelik yumuşatma" desin, kaç ölçüm geçtiğine değil.

### 7c) Anti-windup gerçek doyuma bakmıyor
(SORUN 4c'de anlatıldı — I, motor tam güçteyken bile birikiyor.)

### 7d) İleri besleme girişi yok
(SORUN 4b — PID sınıfına `ff` parametresi eklenecek.)

### 7e) Ki'nin yeri yanlış

Kodda `Ki`, biriktirme sırasında uygulanıyor. Havuzda `Ki`'yi canlı değiştirdiğinde
**geçmişte biriken kısım eski Ki ile hesaplanmış olarak kalıyor.** Ayarlama sırasında
kafa karıştırıcı sonuçlar üretir. Ki çıkışta uygulanmalı.

### 7f) İç değerler görünmüyor

Şu an PID sadece toplam sonucu veriyor. Ayarlarken "bu komut P'den mi geldi, I'dan mı,
D'den mi?" sorusunu cevaplayamıyoruz. Kör ayar yapıyoruz.

**Çözüm:** P, I, D, FF ve "doydu mu" bilgisi ayrı ayrı loglanacak. Havuzda 3 saatimiz
var, kör deneme yapacak vaktimiz yok.

---

## SORUN 8 — Aynı sensör döngüde 3 kez okunuyor
(SORUN 2'de anlatıldı — bir kez okunup saklanacak.)

---

# BÖLÜM 5 — Yeni tasarımda neyi neden değiştiriyorum?

## 5.1 Derinlik: "bilinen yükü baştan taşı"

**Eski:**
```
motor = P + I + D
```
**Yeni:**
```
motor = FF_hover + P + I + D(dikey hıza göre)
```

- `FF_hover`: havuzda ölçülen "asılı kalma" gücü. PID'in yükünü alır.
- D artık "hatanın değişimi"ne değil **aracın dikey hızına** bakıyor.
  Fiziksel karşılığı net: **hızlı düşüyorsan fren yap.** Hedef değişse bile sıçramaz.

## 5.2 Yön: tek PID yerine iç içe iki PID (kaskad)

**Eski yapı:** "90 derece hata var → şu kadar gaz ver." Ama şu kadar gazın aracı
saniyede kaç derece döndürdüğünü kimse bilmiyor. Batarya doluyken hızlı, boşken yavaş
döner. Ayar tekrarlanabilir olmuyor.

**Yeni yapı — iki katman:**

```
DIŞ KATMAN (yavaş, "nereye"):
   "90 derece sağa dönmem lazım"
   → "o halde saniyede 30 derece hızla dön"   ← ve bu hıza ÜST SINIR koyabiliyorum

İÇ KATMAN (hızlı, "ne kadar"):
   "hedef 30 derece/sn, jiroskop 22 derece/sn diyor"
   → "gazı biraz artır"
```

**Neden çok daha iyi:**

1. **Dönüş hızını doğrudan sınırlayabiliyorum.** "En fazla 30 derece/sn" diyorum.
   Araç asla kontrolsüz hızlanamaz → hedefi aşma sorunu büyük ölçüde biter.
2. **Jiroskop doğrudan ölçüm olarak kullanılıyor.** Dönüş hızını zaten sensörden
   **ölçüyoruz** — hesaplamaya, türev almaya, gürültüyle uğraşmaya gerek yok.
3. **Batarya/motor farklarını iç katman emiyor.** Dış katman hep aynı davranır.
4. **Daire görevinde aynı iç katman kullanılıyor.** "Saniyede 24 derece dön" deyip
   bırakıyoruz. Ayrı bir kod yazmaya gerek yok.

> **Analoji:** Arabada "şu kavşakta dön" (dış katman) ile "hız sabitleyici 50 km/s"
> (iç katman) gibi. Hız sabitleyici yokuşu, rüzgârı kendisi halleder;
> sen sadece nereye gideceğini söylersin.

## 5.3 Dairenin çapını matematikle belirlemek

Şu an daire için sabit bir dönüş **komutu** veriliyor → çap her seferinde farklı çıkıyor,
şartnamenin 1 metre şartını tutturmak şansa kalıyor.

Yeni yöntemde basit bir formül var:

```
çap = 2 × (ileri hız) / (dönüş hızı)
```

Yani ileri ne kadar hızlı gidersen ve ne kadar yavaş dönersen daire o kadar büyük olur.

| İstenen çap | İleri hız | Gereken dönüş hızı | Tur süresi |
|---|---|---|---|
| 1.2 m | 0.25 m/s | 23.9 °/s | 15.1 s |
| 1.2 m | 0.20 m/s | 19.1 °/s | 18.8 s |
| 1.5 m | 0.25 m/s | 19.1 °/s | 18.8 s |

Havuzda ileri hızı bir kez ölçeceğiz (16 saniye düz git, mesafeyi ölç, böl).
Sonra tablodan dönüş hızını seçeceğiz. Çap **hesapla** garanti edilmiş olacak.
1 metre şart, biz 1.2 hedefliyoruz — %20 emniyet payı.

## 5.4 Roll/Pitch: önce mekanik, sonra yazılım

Roll/pitch PID'ini agresifleştirmek yanlış çözüm. Araç zaten dengeliyse PID'e iş kalmaz.

**Havuzda ilk 10 dakika:** motorlar kapalı, araç suya bırakılır, nasıl durduğuna bakılır.
Yan yatıyorsa **ağırlık/köpük** ile düzeltilir. Hedef: motorlar kapalıyken ±3° içinde durmalı.

Sonra PID **düşük yetkiyle** devreye girer, boşluk telafisiyle, ve 2 derecenin altında
hiç karışmaz (gereksiz titremeyi önlemek için).

---

# BÖLÜM 6 — Havuzda neden bu sırayla ilerliyoruz?

Ayar yaparken **aynı anda iki şey değiştirirsen hiçbir şey öğrenemezsin.**
Araç kötü davrandı — Kp yüzünden mi, Ki yüzünden mi, dengesizlikten mi? Bilemezsin.

O yüzden protokol katman katman ilerliyor; her katman altındakinin sağlam olmasına dayanıyor:

| Sıra | Adım | Neden burada? |
|---|---|---|
| 0 | Güvenlik + acil durdurma | Kontrolü kaybedersen aracı kurtarabilmelisin |
| 1 | **Mekanik denge** (motorlar kapalı) | Araç dengesizse hiçbir PID bunu düzeltemez |
| 2 | **FF_hover ölçümü** | Derinlik PID'i bu sayı olmadan doğru ayarlanamaz |
| 3 | Derinlik **Kp** | Önce hareket etsin |
| 4 | Derinlik **Kd** | Sonra aşmasın |
| 5 | Derinlik **Ki** | En son tam otursun |
| 6 | Yön **iç katman** (dönüş hızı) | Dış katman iç katmanın üstünde çalışıyor |
| 7 | Yön **dış katman** (açı) | Ancak iç katman sağlamsa ayarlanabilir |
| 8 | **Düz gidiş + hız ölçümü** | Daire hesabı bu hıza ihtiyaç duyuyor |
| 9 | **Daire** | Adım 8'in çıktısıyla hesaplanıyor |
| 10 | **Tam prova ×3** | Her şey birlikte çalışıyor mu? |

**Neden derinlik önce, yön sonra?**
Derinlik bozuksa araç sürekli iniyor-çıkıyor demektir. İnip çıkarken aracın burnu
yukarı/aşağı gidiyor, bu da pusulanın eğim telafisini zorluyor → yön ölçümü bozuluyor.
Yani **yön kontrolü derinlik kontrolüne bağımlı.** Tersi değil. O yüzden sıra bu.

**Neden süre sıkıntısında daire en sona bırakılıyor?**
Şartnamedeki 1 metre şartı için %20 pay bıraktık. Daire biraz kaba olsa da geçer.
Ama derinlik kontrolü bozulursa araç yüzeye çıkar ve **doğrudan elenir.**

---

# BÖLÜM 7 — Havuza kadar 3 saat: ne yapılacak?

| Süre | İş | Neden |
|---|---|---|
| **0:00–0:20** | IMU'yu yeniden kalibre et | SORUN 1 — her şeyin kökü |
| **0:20–0:50** | Döngüyü hızlandır (sensör thread'i + hassasiyet düşür) | SORUN 2 — PID'in çalışması için şart |
| **0:50–1:10** | 6 motorun yönünü elle doğrula | SORUN 5 — yanlışsa araç kontrolden çıkar |
| **1:10–2:10** | PID kodunu yeniden yaz + boşluk telafisi + kaskad yön | SORUN 3, 4, 7 |
| **2:10–2:30** | Daire sayacını düzelt + detaylı loglama ekle | SORUN 6, 7f |
| **2:30–2:50** | Havuz için canlı ayar konsolu yaz | Havuzda kodu durdurup düzenlemeye vakit yok |
| **2:50–3:00** | Çanta, e-stop testi, protokol çıktısı | — |

**Zaman daralırsa öncelik:**
`IMU kalibrasyonu` → `motor yönleri` → `döngü hızı` → `boşluk telafisi` → `ayar konsolu` → `kaskad`

İlk üçü olmadan **hiçbir şey ilerlemez.** Kaskad yön kontrolü olmadan mevcut PID ile
idare edilebilir (daha zor ayarlanır ama çalışır).

---

# BÖLÜM 8 — Tek cümlelik özet

> Mevcut PID katsayıları kötü olduğu için değil, **PID'in gördüğü bilgiler yanlış
> ve motorlara gönderdiği komutlar eksik ulaştığı için** araç düzgün çalışmıyor.
> Önce sensörü, döngü hızını ve motor yönlerini düzelteceğiz; sonra PID'i
> ölçülebilir ve tekrarlanabilir hale getireceğiz; havuzda da katman katman,
> her adımda tek bir şey değiştirerek ayarlayacağız.

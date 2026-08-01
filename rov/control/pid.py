"""
PID denetleyici — SIFIRDAN YENIDEN YAZILDI.

Bu dosya PID_BASIT_ANLATIM.md'deki SORUN 4 ve SORUN 7'yi cozer.

Eski surumdeki problemler ve buradaki karsiliklari:

  7a) "Setpoint kick": D terimi HATA uzerinden turev aliyordu. Hedef 0'dan
      90'a atlayinca hata da atliyordu, D bunu "arac inanilmaz hizli
      hareket ediyor" sanip devasa bir fren komutu uretiyordu.
      COZUM: D artik OLCUM uzerinden calisiyor (`meas_rate` ya da
      `measurement`). Hedef degisince D hicbir sey hissetmez.

  7b) D filtresi ornek sayisina bagliydi; dongu suresi 88-138 ms arasi
      oynadigi icin her adimda farkli kesim frekansi uyguluyordu.
      COZUM: zaman sabitli filtre  a = dt / (d_tau + dt).

  4c) Windup: motor zaten tam guctayken (doygun) I birikmeye devam
      ediyordu; sonra arac hedefi fena halde asiyordu.
      COZUM: kosullu integrasyon — cikis doygunsa VE hata doygunlugu
      derinlestiriyorsa I commit EDILMEZ.

  4b) Ileri besleme (feed-forward) girisi yoktu; aracin pozitif kaldirma
      kuvvetini tasimak tamamen I'nin sirtindaydi (yavas + windup).
      COZUM: `ff` girisi. out = ff + P + I + D.

  7e) Ki, integral birikiminin ICINDE uygulaniyordu. Havuzda Ki'yi canli
      degistirince gecmis birikim eski kazancta kaliyor, sonuc kafa
      karistirici oluyordu.
      COZUM: I ham hata-zaman toplami olarak tutulur, Ki CIKISTA carpilir.
      i_limit artik CIKIS BIRIMINDE (0..1 motor komutu) — yorumlamasi kolay.

  7f) Sadece toplam cikis goruluyordu; "bu komut P'den mi I'dan mi geldi"
      sorusunu cevaplayamiyorduk, yani havuzda kor ayar yapiyorduk.
      COZUM: `self.last` sozlugunde P/I/D/FF/doygunluk ayri ayri tutulur,
      logger bunlari CSV'ye yazar.

  3)  Olu bant: cok kucuk komutlar motora hic ulasmiyordu. Bu PID'in
      degil mixer'in isi — bkz. control/mixer.py. Burada sadece
      `deadzone` var: "su acinin altinda hic karisma" (roll/pitch icin).
"""
import time


def clamp(v, lo, hi):
    """v degerini [lo, hi] araligina kirpar."""
    return lo if v < lo else (hi if v > hi else v)


def angle_error_deg(target, current):
    """Iki heading arasindaki EN KISA acisal fark (-180..+180 derece).

    Neden gerekli: 350 derece ile 10 derece arasindaki fark 340 degil, 20'dir.
    Duz cikarma yapilsa arac uzun yoldan donmeye calisirdi.
    Pozitif sonuc = saga (saat yonu) donulmeli.
    """
    return (target - current + 180.0) % 360.0 - 180.0


class PID:
    def __init__(self, kp, ki, kd, out_limit=1.0, i_limit=0.5,
                 d_tau=0.15, deadzone=0.0, ff=0.0, name=""):
        """
        kp, ki, kd : PID katsayilari
        out_limit  : cikis siniri (+-). Motor komutu icin 1.0 = tam guc.
        i_limit    : I teriminin CIKIS BIRIMINDEKI ust siniri.
                     Ornek: i_limit=0.4 -> I tek basina en fazla %40 gaz verebilir.
                     (Eski surumde bu deger ham birikim birimindeydi, anlasilmazdi.)
        d_tau      : D teriminin alcak geciren filtre ZAMAN SABITI (saniye).
                     Buyuk = daha yumusak ama daha gec. 0.15 s iyi bir baslangic.
        deadzone   : |hata| bu degerin altindaysa PID hic karismaz (sadece ff doner).
                     Roll/pitch icin kullanilir; gereksiz titremeyi onler.
        ff         : sabit ileri besleme (feed-forward). Derinlik icin
                     "asili kalma gucu" (FF_HOVER) buraya girer.
        name       : loglarda gorunecek isim (tanilama icin).
        """
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.out_limit = float(out_limit)
        self.i_limit = float(i_limit)
        self.d_tau = float(d_tau)
        self.deadzone = float(deadzone)
        self.ff = float(ff)
        self.name = name
        self.reset()

    # ------------------------------------------------------------------ durum
    def reset(self):
        """Ic durumu sifirlar. Hedef degistiginde ya da motorlar durdurulup
        tekrar baslatildiginda cagrilmali — aksi halde eski I birikimi ve
        eski D gecmisi yeni duruma sicrar."""
        self._i = 0.0            # ham hata-zaman toplami (Ki HARIC)
        self._prev_meas = None   # sayisal turev icin onceki olcum
        self._prev_err = None    # (yedek yol) onceki hata
        self._d_filt = 0.0       # filtrelenmis turev
        self._prev_t = None
        self.last = dict(p=0.0, i=0.0, d=0.0, ff=0.0, out=0.0,
                         err=0.0, dt=0.0, sat=0)

    # ------------------------------------------------------- canli ayar (havuz)
    def get_params(self):
        """Mevcut katsayilar (web arayuzu ve pid_tune.py icin)."""
        return {"kp": self.kp, "ki": self.ki, "kd": self.kd, "ff": self.ff,
                "out_limit": self.out_limit, "i_limit": self.i_limit,
                "d_tau": self.d_tau, "deadzone": self.deadzone}

    def set_params(self, kp=None, ki=None, kd=None, ff=None,
                   out_limit=None, i_limit=None, d_tau=None, deadzone=None,
                   reset=True):
        """Katsayilari canli gunceller.

        reset=True (varsayilan): I birikimi temizlenir. Havuzda katsayi
        degistirirken bu SART — yoksa eski katsayiyla birikmis I yeni
        katsayinin sonucunu maskeler ve hangi degisikligin ne yaptigini
        anlayamazsin.
        """
        if kp is not None:        self.kp = float(kp)
        if ki is not None:        self.ki = float(ki)
        if kd is not None:        self.kd = float(kd)
        if ff is not None:        self.ff = float(ff)
        if out_limit is not None: self.out_limit = float(out_limit)
        if i_limit is not None:   self.i_limit = float(i_limit)
        if d_tau is not None:     self.d_tau = float(d_tau)
        if deadzone is not None:  self.deadzone = float(deadzone)
        if reset:
            self.reset()

    # -------------------------------------------------------------- ana hesap
    def update(self, error, meas_rate=None, measurement=None, ff=None, now=None):
        """Bir kontrol adimi hesaplar ve motor komutunu (-out_limit..+out_limit) dondurur.

        error      : hedef - olculen  (aci icin angle_error_deg ile hesaplanmali)
        meas_rate  : olcumun DEGISIM HIZI, dogrudan sensorden. EN IYI YOL.
                     Ornek: heading icin jiroskopun yaw_rate'i, derinlik icin
                     hesaplanmis dikey hiz. Turev almaya gerek kalmaz, gurultu olmaz.
        measurement: meas_rate yoksa, olcumun kendisi verilir; turevi burada
                     sayisal olarak alinir (yine de hata uzerinden almaktan iyi).
        ff         : bu adima ozel ileri besleme (verilmezse self.ff kullanilir)
        now        : zaman (test icin disaridan verilebilir)
        """
        now = time.monotonic() if now is None else now
        if self._prev_t is None:
            dt = 0.0
        else:
            # dt'yi kirp: ilk adim ve donma sonrasi devasa dt'ler I ve D'yi patlatir
            dt = clamp(now - self._prev_t, 0.0, 0.2)
        self._prev_t = now

        ff_val = self.ff if ff is None else float(ff)

        # --- OLU BOLGE: kucuk hatalarda hic karisma -------------------------
        # Roll/pitch icin: arac 2 derece yatmissa duzeltmeye calismak
        # gereksiz titreme uretir. Sadece ileri besleme gecer.
        if self.deadzone > 0.0 and abs(error) < self.deadzone:
            self._d_filt *= 0.9          # turev hafizasini yavasca bosalt
            out = clamp(ff_val, -self.out_limit, self.out_limit)
            self.last = dict(p=0.0, i=0.0, d=0.0, ff=ff_val, out=out,
                             err=error, dt=dt, sat=0)
            return out

        # --- D TERIMI: her zaman OLCUM uzerinden ---------------------------
        # Matematiksel not: hata = hedef - olcum. Hedef sabitken
        #   d(hata)/dt = -d(olcum)/dt
        # Bu yuzden olcum hizini dogrudan kullanip basina eksi koyuyoruz.
        # Kazanci: hedef degistiginde turev SICRAMAZ (setpoint kick yok).
        if meas_rate is not None:
            raw_d = -float(meas_rate)
        elif measurement is not None:
            if self._prev_meas is None or dt <= 0.0:
                raw_d = 0.0
            else:
                raw_d = -(measurement - self._prev_meas) / dt
            self._prev_meas = measurement
        else:
            # yedek yol (eski davranis) — mumkunse kullanma
            if self._prev_err is None or dt <= 0.0:
                raw_d = 0.0
            else:
                raw_d = (error - self._prev_err) / dt
        self._prev_err = error

        # zaman sabitli 1. derece alcak geciren filtre
        # a = dt/(tau+dt): dt buyudukce filtre daha cok "yeni" degeri alir,
        # boylece degisken dongu suresinde bile ayni yumusatma davranisi olur.
        if dt > 0.0:
            a = dt / (self.d_tau + dt)
            self._d_filt += a * (raw_d - self._d_filt)
        d_term = self.kd * self._d_filt

        p_term = self.kp * error

        # --- I TERIMI: back-calculation tabanli anti-windup -----------------
        #
        # AMAC: I terimi, cikis tavana DEGENE KADAR birikebilsin; tavana
        # degdikten sonra DURSUN. Ne fazla ne eksik.
        #
        # NEDEN "hepsi ya da hicbiri" DONDURMA YETMIYOR (birim testiyle bulundu):
        #   Basit yontem "cikis doygunsa bu adimi hic isleme" der. Ama tek bir
        #   adimda I'nin buyume miktari (error*dt*ki) kalan bosluktan buyukse,
        #   adimin TAMAMI reddedilir. Sonuc: I hicbir zaman BIR ADIM BILE
        #   buyuyemez ve kalici hata asla silinmez.
        #   Olculdu: kp=0.05, hata=10, ki=1.0, dt=0.1 -> I sonsuza kadar 0 kaldi.
        #
        # DOGRUSU: I'yi, ciktiyi TAM OLARAK sinira getiren degere kadar birak.
        #   i_izin_ust = (out_limit  - ff - P - D) / ki
        #   i_izin_alt = (-out_limit - ff - P - D) / ki
        # I bu araliga kirpilir. Boylece "tavana degene kadar bir, sonra dur".
        #
        # TEK ISTISNA: bu kirpma, I'yi guncelleme YONUNUN TERSINE iterse
        # (ornegin P tek basina siniri asmissa) I'yi ters yone surukleyip
        # sonradan yetersiz tepki (undershoot) yaratir. O durumda I dondurulur.
        if self.ki <= 0.0:
            self._i = 0.0
            i_term = 0.0
            out_raw = ff_val + p_term + i_term + d_term
        else:
            i_try = self._i + error * dt
            # I'nin cikis birimindeki katkisini i_limit ile sinirla
            i_cap = self.i_limit / self.ki
            i_try = clamp(i_try, -i_cap, i_cap)

            # ciktiyi tam sinira getiren I degerleri
            taban = ff_val + p_term + d_term
            i_ust = (self.out_limit - taban) / self.ki
            i_alt = (-self.out_limit - taban) / self.ki
            lo, hi = (i_alt, i_ust) if i_alt <= i_ust else (i_ust, i_alt)
            i_yeni = clamp(i_try, lo, hi)

            # kirpma guncelleme yonunu TERSINE cevirdiyse: dondur
            if (i_try > self._i and i_yeni < self._i) or \
               (i_try < self._i and i_yeni > self._i):
                i_yeni = self._i

            self._i = i_yeni
            i_term = self.ki * self._i
            out_raw = ff_val + p_term + i_term + d_term

        out = clamp(out_raw, -self.out_limit, self.out_limit)

        # havuz ayari icin telemetri (logger bunlari CSV'ye yazar)
        self.last = dict(p=p_term, i=i_term, d=d_term, ff=ff_val, out=out,
                         err=error, dt=dt, sat=int(out != out_raw))
        return out

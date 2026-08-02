"""
EGE ROV - Merkezi yapilandirma.
Cihaz uzerinde optimizasyon yaparken SADECE bu dosyayi degistirmen yeterli.

TUM parametreler burada: motor PWM, PID, gorev sureleri, sensor adresleri,
vizyon esikleri, haberlesme portlari, GPS, sonar, vinc servo, e-stop GPIO.
"""

# ---------------------------------------------------------------- genel
LOOP_HZ = 50                 # ana kontrol dongusu frekansi (Hz)
SIM_MODE = False              # True: donanim yok, simulasyonla calis. Cihazda False yap.

# ---------------------------------------------------------------- motorlar
# PWM kanal atamalari (PCA9685 kanal no ya da kullandigin surucunun kanali)
# Yerlesim: 4 dikey (V), 2 yatay (H)
#   V_FL: on-sol dikey   V_FR: on-sag dikey
#   V_RL: arka-sol dikey V_RR: arka-sag dikey
#   H_L : sol yatay      H_R : sag yatay
MOTOR_CHANNELS = {
    "V_FL": 0,
    "V_FR": 3,
    "V_RL": 1,
    "V_RR": 5,
    "H_L":  2,
    "H_R":  4,
}

# PWM sinyal parametreleri (her iki yone simetrik — tek merkezden yonetilir)
PCA9685_REF_CLOCK_HZ = 29_040_000 # PCA9685 dahili saat frekansi (58.08 Hz olcumune gore 29.04 MHz olarak kalibre edildi)
                                  # DIKKAT: PCA9685(...) cagrisina reference_clock_speed=PCA9685_REF_CLOCK_HZ
                                  # VERILMEZSE kart 50Hz yerine 58.1Hz'de calisir ve TUM darbeler %14 kisalir
                                  # (1500us -> ESC 1291us gorur = geri yon). Her yeni scriptte bunu gec!
FREQ_HZ         = 50          # ESC PWM tasiyici frekansi (Hz) - tum backendler bunu kullanir

# ESC'nin FIZIKSEL sinirlari. Bunlar donanim gercegi, ayar degil - degistirmeyin.
# calibrate_escs.py ESC EEPROM'una 1000-2000us araligini yaziyor.
# Hicbir kod bu araligin disina darbe gonderemez (thrusters.py zorunlu kilar).
ESC_ABS_MIN_US  = 1000        # ESC'nin anladigi en kisa darbe
ESC_ABS_MAX_US  = 2000        # ESC'nin anladigi en uzun darbe

PWM_NEUTRAL_US  = 1470        # ESC notr sinyali (mikrosaniye)
# Notrden her iki yone esit sapma. TOPLAM genislik DEGIL, tek yondeki pay!
# 1470 notr icin: asagi pay 1470-1000=470, yukari pay 2000-1470=530.
# Simetrik kalmak icin kucuk olani secilir -> 470 => 1000..1940us.
PWM_RANGE_US    = 470
PWM_MIN_US      = PWM_NEUTRAL_US - PWM_RANGE_US   # 1000 us (otomatik hesap)
PWM_MAX_US      = PWM_NEUTRAL_US + PWM_RANGE_US   # 1940 us (otomatik hesap)
PWM_DEADBAND_US = 30          # notr etrafinda olu bant (titremeyi ve bip sesini engeller)
THRUST_LIMIT    = 1.0         # motor guc siniri 0..1 (baslangicta dusuk tut!)
SLEW_RATE       = 2.0         # birim/sn - motor komutu degisim hizi siniri (ani gaz onler)

# Pervane yonu duzeltmeleri: ters donen motor icin -1 yaz.
# Bu degerler tercih olarak belirlenmistir (kasitli secim).
# Mixer ciktisi: pozitif heave = ROV dalar, pozitif surge = ileri.
MOTOR_DIRECTION = {
    "V_FL": -1, "V_FR": -1, "V_RL": -1, "V_RR": -1,
    "H_L": -1, "H_R": -1,
}

# ---------------------------------------------------------------- PID katsayilari
# ============================================================================
# BU BOLUM YENIDEN TASARLANDI — bkz. PID_TASARIM_PLANI.md ve PID_BASIT_ANLATIM.md
# ============================================================================
# Anlamlar (control/pid.py):
#   kp/ki/kd  : klasik PID katsayilari
#   out_limit : cikis siniri (1.0 = tam gaz)
#   i_limit   : I teriminin CIKIS BIRIMINDEKI ust siniri.
#               0.4 -> I tek basina en fazla %40 gaz verebilir.
#               (Eskiden ham birikim birimindeydi, kimse yorumlayamiyordu.)
#   d_tau     : D teriminin filtre ZAMAN SABITI (sn). Buyuk = yumusak ama gec.
#   deadzone  : |hata| bunun altindaysa PID hic karismaz (titremeyi onler)
#   ff        : ileri besleme (feed-forward)

# --- DERINLIK -----------------------------------------------------------
# ff = FF_HOVER: aracin 'asili kalma' gucu. Arac POZITIF KALDIRMA kuvvetli
# (arıza halinde yuzsun diye) — yani sabit derinlikte DURMAK bile surekli
# asagi itki ister. Eskiden bunu tek basina I terimi tasiyordu: yavas ve
# windup uretiyordu. Simdi bilinen kismi ff veriyor, PID sadece artigi duzeltiyor.
#
# !!! FF_HOVER HAVUZDA OLCULECEK !!!  (havuz protokolu Adim 2)
#     python3 pid_tune.py  ->  'hover'  komutu
FF_HOVER = 0.0               # 0.0 = olculmedi. Beklenen aralik 0.15 .. 0.40

PID_DEPTH = dict(
    kp=1.5, ki=0.15, kd=0.6,
    out_limit=1.0,      # dikey eksende tam yetki
    i_limit=0.40,       # I tek basina en fazla %40 gaz
    d_tau=0.30,         # dikey hiz olcumu gurultulu -> biraz daha yumusak
    deadzone=0.0,       # derinlikte olu bolge YOK (5 cm bile onemli)
    ff=FF_HOVER,
)

# --- YON (KASKAD) -------------------------------------------------------
# Tek PID yerine ic ice iki katman (control/cascade.py):
#   DIS  : aci hatasi  -> istenen donus hizi (derece/sn), UST SINIRLI
#   IC   : istenen donus hizi -> motor komutu (jiroskop geri beslemeli)
# Kazanc: donus hizi dogrudan sinirlanir (asim ↓), jiroskop OLCUM olarak
# kullanilir (turev gurultusu yok), batarya/akinti farkini ic dongu emer.
HEADING_POS = dict(
    kp=1.2,             # 1 derece hata -> 1.2 derece/sn donus istegi
)
HEADING_RATE = dict(
    kp=0.020,           # 30 dps hata -> 0.60 komut (tam yetki). Mantikli baslangic.
    ki=0.010,           # akinti/motor asimetrisinden kalan kalici hatayi siler
    kd=0.0,             # ic dongude D genelde gerekmez; gerekirse havuzda ekle
    # i_limit NEDEN out_limit KADAR BUYUK:
    #   HIZ dongusunde kalici komutun ASIL kaynagi I terimidir. P sadece
    #   HATAYA tepki verir; hedefe yaklastikca P kuculur ve sifira gider.
    #   Yani "30 dps'de sabit donmek" icin gereken kalici gazi I tasir.
    #   Simulasyonda i_limit=0.25 iken kontrolcu 30 dps hedefine ULASAMADI,
    #   25 dps'de dengelendi (P=0.10 + I=0.25 tavan = 0.35 komut).
    #   Bu, daire capini da hedefledigimiz degil ULASILABILEN deger yapar.
    #   Kural: hiz dongusunde i_limit ~ out_limit.
    i_limit=0.55,
    d_tau=0.10,
)
# Iki calisma modu — duz seyirde araci sert dondurmek rotayi bozar.
HEADING_MODES = dict(
    cruise=dict(w_max_dps=15.0, out_limit=0.35),   # duz segmentler
    turn=dict(w_max_dps=30.0, out_limit=0.60),     # yerinde 90 derece donus
    circle=dict(w_max_dps=45.0, out_limit=0.55),   # daire (sabit donus hizi)
)
# NEDEN AYRI 'circle' MODU:
#   Simulasyon testinde daire, 'cruise' modunun out_limit=0.35 yetkisiyle
#   calisiyordu. Istenen donus hizi 60 dps iken ancak ~24 dps uretilebildi;
#   yani ic dongu DOYGUNDU ve daire capi hedeflenen degil ULASILABILEN
#   degere gore olustu — tam da kacinmak istedigimiz durum.
#   Daire yetkisi ayri tanimlanir; ic dongu doymadan hedef hizi tutabilmeli.
#   HAVUZDA KONTROL: log'da yaw_sat sutunu daire boyunca 0 kalmali.

# --- ROLL / PITCH -------------------------------------------------------
# NOT: Once MEKANIK TRIM. Arac motorlar kapaliyken +-3 derece icinde durmali.
# PID mekanik dengesizligi duzeltemez; sadece kalan kucuk sapmayi toplar.
# deadzone=2.0: 2 derecenin altinda hic karisma (gereksiz titreme yok).
# D terimi jiroskoptan geliyor (turev yok).
PID_ROLL  = dict(kp=0.020, ki=0.0, kd=0.010, out_limit=0.25,
                 i_limit=0.05, d_tau=0.10, deadzone=2.0, ff=0.0)
PID_PITCH = dict(kp=0.020, ki=0.0, kd=0.010, out_limit=0.25,
                 i_limit=0.05, d_tau=0.10, deadzone=2.0, ff=0.0)

# --- ESKI SURUM UYUMU ---------------------------------------------------
# auto_pid.py gibi eski scriptler bunu import ediyor. Yon kontrolu artik
# kaskad; bu deger KULLANILMIYOR, sadece import hatasi olmasin diye duruyor.
PID_HEADING = dict(kp=0.02, ki=0.0, kd=0.008, out_limit=0.6, i_limit=0.2)

# ---------------------------------------------------------------- kontrol dongusu
# SORUN 2: gercek donanimda dongu 50 Hz yerine 7.3 Hz'de calisiyordu
# (derinlik sensoru her okumada 40 ms bloklyordu ve dongu basina 3 kez okunuyordu).
# Cozum: sensorler ayri thread'lerde (sensors/state.py), kontrol dongusu
# sadece hafizadan okuyor.
IMU_THREAD_HZ    = 100.0     # IMU okuma frekansi (kendi thread'inde)
DEPTH_THREAD_HZ  = 20.0      # derinlik okuma frekansi (kendi thread'inde)
DEPTH_RATE_TAU   = 0.30      # derinlik HIZI filtresi zaman sabiti (sn)
SENSOR_STALE_S   = 0.5       # bu surede veri gelmezse watchdog motorlari keser
DEPTH_OSR        = 1024      # MS5837 hassasiyeti. 1024 -> ~2 mm, 3 ms bekleme.
                             # Eskiden 8192 idi -> 20 ms bekleme x2 = 40 ms.
LOOP_WARN_HZ     = 25.0      # dongu bunun altina duserse ekrana uyari bas

# ---------------------------------------------------------------- olu bant telafisi
# SORUN 3: PWM_DEADBAND_US=30 / PWM_RANGE_US=470 = 0.0638
# Komutun %6.4'unden kucugu motora HIC ulasmiyordu -> roll/pitch PID'i
# 6.4 derecenin altinda hic calismiyordu. Telafi control/mixer.py'de.
DEADBAND_COMPENSATION = True   # havuzda sorun cikarsa False yapip test et
DEADBAND_EPS = 0.01            # bunun altindaki komut GERCEKTEN sifir

# ---------------------------------------------------------------- jiroskop gurultusu
# SORUN 6: daire sayaci abs(yaw_rate) ile topluyordu; jiroskop gurultusu
# bile birikip 40 saniyede ~20 derece sahte donus uretiyordu.
# Artik ISARETLI toplama + bu esigin altini yoksayma var.
GYRO_NOISE_DPS = 1.0

# ---------------------------------------------------------------- gorev (video gosterimi)
MISSION = dict(
    target_depth_m   = 0.6,   # hedef derinlik (yuzeye cikmak YASAK, cok derin de gerekmez)
    dive_timeout_s   = 30.0,  # dalis icin max sure (10->30: yuzey kuvvetini yenmek zaman alir)
    dive_power       = 1.0,   # dalis fazinda tam guc: 1.0 = motorlarin max itisi
                              # ROV boyutuna gore 0.8..1.0 arasi dene
    depth_tol_m      = 0.15,  # derinlik "tamam" toleransi
    straight_time_s  = 16.0,  # min 15 sn sart -> pay birak
    cruise_throttle  = 0.35,  # duz gidis ileri gaz (0..1) - hiz kalibrasyonuyla ayarlanacak
    turn_tol_deg     = 5.0,   # donus tamamlandi toleransi
    turn_settle_s    = 1.5,   # donus sonrasi sabitlenme suresi
    turn_timeout_s   = 15.0,
    # --- DAIRE: artik sabit KOMUT degil, sabit DONUS HIZI hedefi ---
    # Eski yontemde sabit yaw komutu veriliyordu; cap bataryaya, suruklenmeye,
    # motor sicakligina gore degisiyordu (tekrarlanabilir degil).
    # Yeni yontem kapali cevrim:   cap = 2 * ileri_hiz / donus_hizi
    #   D=1.2 m, v=0.25 m/s -> w = 23.9 dps, tur 15.1 s
    #   D=1.2 m, v=0.20 m/s -> w = 19.1 dps, tur 18.8 s
    #   D=1.5 m, v=0.25 m/s -> w = 19.1 dps, tur 18.8 s
    # v HAVUZDA olculecek (protokol Adim 8), sonra w bu formulle secilecek.
    circle_diameter_m   = 1.2,   # hedef cap (sartname min 1.0 m, %20 pay)
    circle_yaw_rate_dps = 24.0,  # hedef donus hizi (derece/sn) - Adim 8 sonrasi guncelle
    circle_throttle     = 0.25,  # daire: ileri gaz
    circle_deg          = 370.0, # tam turdan biraz fazla (jiroskop toplami)
    circle_yaw_rate     = 0.35,  # (ESKI - kullanilmiyor, geriye uyum icin)
    start_delay_s    = 10.0,  # baslat komutundan gorev baslangicina geri sayim
)

# ---------------------------------------------------------------- sensorler (KTR donanimi)
I2C_BUS = 8                  # Tüm cihazlar (0x0C, 0x40, 0x68, 0x76) Bus 8 üzerinde bulundu (Pin 3/5)
IMU_ADDR = 0x68              # MPU-9250
MAG_ADDR = 0x0C              # AK8963 (MPU-9250 icindeki manyetometre)
DEPTH_ADDR = 0x76            # MS5837-30BA
FLUID_DENSITY = 997         # kg/m3 (deniz ~1025, havuz/tatli su 997)
SURFACE_PRESSURE_MBAR = 1010.0  # kalibrasyon ile olculdu (calibrate_depth.py)
                             # None: gorev basinda zero_at_surface() ile olculur

# IMU fuzyon + kalibrasyon (cihaz uzerinde olculecek)
USE_MAGNETOMETER = True  # Pusula modulu devrede (heading icin kullanilir)
HEADING_FILTER_ALPHA = 0.995  # Sensor verisi daha yumusak aksin diye artirildi
ROLL_PITCH_FILTER_ALPHA = 0.995 # Sensor verisi daha yumusak aksin diye artirildi
MAG_OFFSET = (0.0, 0.0, 0.0)         # manyetometreyi bosverdik simdilik
MAG_SCALE  = (1.0, 1.0, 1.0)         # manyetometreyi bosverdik simdilik
GYRO_BIAS  = (5.677, 0.82, -0.015)   # kalibrasyon ile olculdu
ACCEL_BIAS = (0.0, 0.0, 0.0)         # ROV hareketsizken calibrate_imu.py ile yeniden olc
ACCEL_SCALE= (1.0, 1.0, 1.0)         # 6-noktali kalibrasyon ile olculur
MOUNT_ROLL_DEG = 0.0
MOUNT_PITCH_DEG = 0.0

# ---------------------------------------------------------------- loglama
LOG_DIR = "logs"
LOG_EVERY_N = 5              # her N dongude bir satir logla (50Hz/5 = 10Hz kayit)

# ---------------------------------------------------------------- e-stop GPIO
# Jetson 40-pin header Pin 13 = BCM 27 (Hall sensoru #2 - acil durdurma)
ESOP_GPIO_BCM = 27           # BCM pin numarasi (i2cdetect ile degistirilebilir)
ESTOP_BOUNCE_MS = 100        # debounce suresi (ms)

# ---------------------------------------------------------------- haberlesme
COMMS_PORT = 9000            # TCP JSON komut portu (Jetson'da server, laptopta client)
VIDEO_PORT = 9001            # UDP JPEG video portu
GCS_IP    = "192.168.1.100" # Yer istasyonu laptop IP'si
ROV_IP    = "192.168.1.10"  # Jetson statik IP'si
VIDEO_QUALITY = 60           # JPEG kalitesi 0-100
VIDEO_FPS    = 15            # video gonderme hedef FPS

# GCS Web Arayuzu Portlari (http://192.168.1.10:8000)
GCS_WEB_PORT  = 8000         # HTTP Web Arayuz Portu
GCS_WS_PORT   = 8080         # Canli Telemetri / WebSocket Portu
JOYSTICK_PORT = 12345        # PS3 Joystick TCP Sunucu Portu

# ---------------------------------------------------------------- kamera
# CSI: Jetson Orin Nano / Xavier NX uzerinde RPi Camera v2.1
# USB: Fallback / test icin USB webcam
CSI_CAMERA   = True          # True: CSI (GStreamer), False: USB (/dev/video0)
CAM_WIDTH    = 1280
CAM_HEIGHT   = 720
CAM_FPS      = 30
CAM_SENSOR_ID = 0            # CSI sensor ID (genellikle 0)

# ---------------------------------------------------------------- goruntu isleme (vizyon)
# TEKNOFEST Hat Takibi: Kırmızı şerit (dış bant) içerisinde Siyah çizgi (iç merkez)
LINE_TRACK_MODE = "RED_BLACK"  # "RED_BLACK": Kırmızı bant içi siyah çizgi, "SINGLE_HSV": tek aralık

# Kırmızı şerit HSV aralıkları (Kırmızı H açısı 0-12 ve 168-180 arasında iki parçadır)
RED_HSV_LOW1  = (0,   90,  50)
RED_HSV_HIGH1 = (12,  255, 255)
RED_HSV_LOW2  = (168, 90,  50)
RED_HSV_HIGH2 = (180, 255, 255)

# Siyah iç çizgi HSV aralığı (düşük V parlaklığı)
BLACK_HSV_LOW  = (0,   0,   0)
BLACK_HSV_HIGH = (180, 255, 90)

# Tekli HSV yedek mod (varsayılan beyaz/genel hat için)
LINE_HSV_LOW  = (0,   0,  170)   # (H_min, S_min, V_min)
LINE_HSV_HIGH = (180, 70, 255)   # (H_max, S_max, V_max)

LINE_MIN_AREA = 2000              # px² — kucuk gurultuyu yoksay
LINE_BLUR_K   = 7                 # Gaussian blur kernel boyutu (tek sayi)

# Boru hizalama (pipe aligner)
PIPE_MIN_RADIUS  = 35             # px — minimum boru yaricapi (Hough)
PIPE_ALIGN_TOL   = 25             # px — hizalama toleransi
PIPE_DEPTH_DELTA = 0.15           # m  — boru hizalamada derinlik ayari

# Vizyon PID: piksel hatasi -> yaw komutu
VISION_KP = 0.003            # ileri (surge) sabitken heading duzeltmesi
VISION_MAX_YAW = 0.4         # max yaw komutu (0..1)

# ---------------------------------------------------------------- GPS
GPS_SERIAL = "/dev/ttyTHS1" # Quectel L80-R UART portu (Jetson 40-pin Pin8/10)
GPS_BAUD   = 9600            # L80-R varsayilan baud
GPS_TIMEOUT_S = 30.0         # bu sure icinde fix alinamazsa uyari

# ---------------------------------------------------------------- Ping Sonar
SONAR_SERIAL = "/dev/ttyUSB0"  # Blue Robotics Ping Sonar USB-UART adaptoru
SONAR_BAUD   = 115200
SONAR_BUOY_RANGE_MIN_MM  = 200    # yanlis okuma filtreleme: min mesafe
SONAR_BUOY_RANGE_MAX_MM  = 5000   # max algilama mesafesi
SONAR_CONFIDENCE_MIN     = 80     # 0-100, dusuk guvenilirlik = yoksay

# ---------------------------------------------------------------- vinc servo (Mini ROV)
# CLS3860MED 60kg servo — PCA9685 CH6
WINCH_CHANNEL     = 6
WINCH_NEUTRAL_US  = 1500    # servo orta konum / bekle
WINCH_DEPLOY_US   = 1900    # Mini ROV birak (ipin geri virmesi)
WINCH_RETRACT_US  = 1100    # Mini ROV cek
WINCH_DEPLOY_S    = 4.0     # birakmak icin vinc surmesi gereken sure (sn)
WINCH_RETRACT_S   = 6.0     # geri cekmek icin sure (sn)

# ---------------------------------------------------------------- gorev 1 parametreleri (hat takibi)
LINE_CRUISE_SURGE    = 0.25  # hat takibinde ileri gaz (yavas — kontrol icin)
LINE_APPROACH_SURGE  = 0.12  # boru girisine yaklasirken
LINE_TARGET_DEPTH    = 0.5   # hat takibi derinligi (m)
LINE_PIPE_DEPTH      = 0.4   # boru girisine hizalanirken derinlik (m)

# ---------------------------------------------------------------- gorev 2 parametreleri (nav)
NAV_TARGET_DEPTH     = 1.0   # navigasyon derinligi (m)
NAV_CRUISE_SURGE     = 0.30  # navigasyonda ileri gaz
BUOY_ORBIT_RADIUS_M  = 1.0   # samandira orbit yaricapi (m) — min 1m cap = 0.5m r
BUOY_ORBIT_SPEED     = 0.20  # orbit surge hizi
BUOY_ORBIT_YAW       = 0.25  # orbit yaw hizi
BUOY_ORBIT_DEG       = 370.0 # tam turdan biraz fazla (jiroskop toplami)


DIVE_MAX= DIVE_MAX = dict(
    target_depth_m = 1.5,   # hedef derinlik (m)
    duration_s     = 60.0,  # toplam test suresi (sn) — sure dolunca motorlar notre cekilir
    dive_power     = 1.0,   # dalis fazi gucu (1.0 = PWM tavani, verebildigi tum guc)
    hold_kp        = 2.0,   # hedefe ulasinca derinligi tutan P katsayisi
    start_delay_s  = 5.0,   # baslamadan once geri sayim (sn)
)
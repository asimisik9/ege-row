"""
EGE ROV - Merkezi yapilandirma.
Cihaz uzerinde optimizasyon yaparken SADECE bu dosyayi degistirmen yeterli.

TUM parametreler burada: motor PWM, PID, gorev sureleri, sensor adresleri,
vizyon esikleri, haberlesme portlari, GPS, sonar, vinc servo, e-stop GPIO.
"""

# ---------------------------------------------------------------- genel
LOOP_HZ = 50                 # ana kontrol dongusu frekansi (Hz)
SIM_MODE = True              # True: donanim yok, simulasyonla calis. Cihazda False yap.

# ---------------------------------------------------------------- motorlar
# PWM kanal atamalari (PCA9685 kanal no ya da kullandigin surucunun kanali)
# Yerlesim: 4 dikey (V), 2 yatay (H)
#   V_FL: on-sol dikey   V_FR: on-sag dikey
#   V_RL: arka-sol dikey V_RR: arka-sag dikey
#   H_L : sol yatay      H_R : sag yatay
MOTOR_CHANNELS = {
    "V_FL": 0,
    "V_RL": 1,
    "H_L":  2,
    "V_FR": 3,
    "V_RR": 4,
    "H_R":  5,
}

# PWM sinyal parametreleri (her iki yone simetrik — tek merkezden yonetilir)
PCA9685_REF_CLOCK_HZ = 25_000_000 # PCA9685 dahili saat frekansi (Hz). Sapma varsa calibrate_escs.py ile ayarlanir.
PWM_NEUTRAL_US  = 1500        # ESC standart notr sinyali (mikrosaniye)
PWM_RANGE_US    = 400         # Notrden her iki yone esit PWM sapma miktari (+/- 400us -> 1100..1900us)
PWM_MIN_US      = PWM_NEUTRAL_US - PWM_RANGE_US   # 1100 us (otomatik hesap)
PWM_MAX_US      = PWM_NEUTRAL_US + PWM_RANGE_US   # 1900 us (otomatik hesap)
PWM_DEADBAND_US = 45          # notr etrafinda olu bant (titremeyi ve bip sesini engeller)
THRUST_LIMIT    = 0.8         # motor guc siniri 0..1 (baslangicta dusuk tut!)
SLEW_RATE       = 2.0         # birim/sn - motor komutu degisim hizi siniri (ani gaz onler)

# Pervane yonu duzeltmeleri: ters donen motor icin -1 yaz.
MOTOR_DIRECTION = {
    "V_FL": 1, "V_FR": 1, "V_RL": 1, "V_RR": 1,
    "H_L": 1, "H_R": 1,
}

# ---------------------------------------------------------------- PID katsayilari
# Cihaz verisiyle optimize edilecek ana degerler bunlar.
PID_DEPTH   = dict(kp=2.0,  ki=0.1,  kd=0.8,  out_limit=1.0, i_limit=0.3)
PID_HEADING = dict(kp=0.02, ki=0.0,  kd=0.008, out_limit=0.6, i_limit=0.2)  # giris: derece
PID_ROLL    = dict(kp=0.01, ki=0.0,  kd=0.004, out_limit=0.3, i_limit=0.1)
PID_PITCH   = dict(kp=0.01, ki=0.0,  kd=0.004, out_limit=0.3, i_limit=0.1)

# ---------------------------------------------------------------- gorev (video gosterimi)
MISSION = dict(
    target_depth_m   = 0.6,   # hedef derinlik (yuzeye cikmak YASAK, cok derin de gerekmez)
    dive_timeout_s   = 10.0,  # dalis icin max sure
    depth_tol_m      = 0.15,  # derinlik "tamam" toleransi
    straight_time_s  = 16.0,  # min 15 sn sart -> pay birak
    cruise_throttle  = 0.35,  # duz gidis ileri gaz (0..1) - hiz kalibrasyonuyla ayarlanacak
    turn_tol_deg     = 5.0,   # donus tamamlandi toleransi
    turn_settle_s    = 1.5,   # donus sonrasi sabitlenme suresi
    turn_timeout_s   = 15.0,
    circle_yaw_rate  = 0.35,  # daire: donus komutu (0..1)
    circle_throttle  = 0.25,  # daire: ileri gaz -> yaricapi bu oran belirler (~>1 m cap olmali)
    circle_deg       = 370.0, # tam turdan biraz fazla (jiroskop toplami)
    start_delay_s    = 10.0,  # baslat komutundan gorev baslangicina geri sayim
)

# ---------------------------------------------------------------- sensorler (KTR donanimi)
I2C_BUS = 7                  # Jetson Orin Nano: pin 3/5 -> genelde bus 7 (i2cdetect ile dogrula!)
IMU_ADDR = 0x68              # MPU-9250
MAG_ADDR = 0x0C              # AK8963 (MPU-9250 icindeki manyetometre)
DEPTH_ADDR = 0x76            # MS5837-30BA
FLUID_DENSITY = 1025         # kg/m3 (deniz ~1025, havuz/tatli su 997)

# IMU fuzyon + kalibrasyon (cihaz uzerinde olculecek)
HEADING_FILTER_ALPHA = 0.98  # jiroskop agirligi (0..1), kalani manyetometre
MAG_OFFSET = (0.0, 0.0, 0.0) # hard-iron ofsetleri (kalibrasyonla bulunacak)
MAG_SCALE  = (1.0, 1.0, 1.0) # soft-iron olcekleri
GYRO_BIAS  = (0.0, 0.0, 0.0) # durgun halde olculen sapma (derece/sn)

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

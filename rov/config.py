"""
EGE ROV - Merkezi yapilandirma.
Cihaz uzerinde optimizasyon yaparken SADECE bu dosyayi degistirmen yeterli.
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
    "V_FR": 1,
    "V_RL": 2,
    "V_RR": 3,
    "H_L":  4,
    "H_R":  5,
}

PWM_NEUTRAL_US = 1500        # ESC notr sinyali (mikrosaniye)
PWM_MIN_US = 1100
PWM_MAX_US = 1900
PWM_DEADBAND_US = 25         # notr etrafinda olu bant
THRUST_LIMIT = 0.8           # motor guc siniri 0..1 (baslangicta dusuk tut!)
SLEW_RATE = 2.0              # birim/sn - motor komutu degisim hizi siniri (ani gaz onler)

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

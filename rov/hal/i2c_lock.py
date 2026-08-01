"""
Tum I2C erisimleri icin TEK ortak kilit.

NEDEN VAR (SORUN 2 -> thread mimarisine gecis):
  Artik uc ayri thread ayni I2C hattini kullaniyor:
    - IMU thread'i      (100 Hz, 0x68 + 0x0C)
    - Derinlik thread'i ( 20 Hz, 0x76)
    - Kontrol dongusu   ( 50 Hz, 0x40 PCA9685 motor yazimi)

  Linux i2c-dev katmani TEK bir islemi (transaction) atomik yapar, ama
  ust uste binen okuma/yazma sirasi yine de bozulabilir (ozellikle
  "register sec + oku" gibi iki adimli erisimlerde).

  Bu kilit SADECE gercek veri yolu islemlerini sarar; sensorun donusum
  bekleme suresini (time.sleep) ASLA sarmaz. Aksi halde derinlik sensoru
  beklerken IMU thread'i de bloke olurdu ve SORUN 2'yi geri getirirdik.

Kullanim:
    from hal.i2c_lock import I2C_LOCK
    with I2C_LOCK:
        bus.write_byte(addr, cmd)      # sadece islemin kendisi
    time.sleep(0.003)                  # bekleme kilit DISINDA
    with I2C_LOCK:
        data = bus.read_i2c_block_data(addr, 0x00, 3)
"""
import threading

# RLock: ayni thread ic ice girerse kilitlenme olmasin.
I2C_LOCK = threading.RLock()

"""
Blue Robotics Ping Sonar surucu.

USB-UART adaptoru uzerinden "Ping Protocol" mesajlari okur.
Mesafe (mm) ve guven degeri (0-100) dondurur.

Dokumantasyon: https://docs.bluerobotics.com/ping-protocol/

Kullanim:
    sonar = PingSonar()
    sonar.start()                  # arka plan okuma thread'i
    m = sonar.measurement          # (distance_mm, confidence) | None
    sonar.stop()

Protokol ozeti:
  Baslangic: 0x42 0x52 (BR)
  Uzunluk: 2 byte LE
  ID: 2 byte LE
  Kaynak/Hedef: her biri 1 byte
  Veri: n byte
  Checksum: 2 byte LE

Tek mesaj ID = 1300 (distance_simple) kullanilir.
"""
import struct
import threading
import time
from typing import Optional, Tuple

import serial

from config import SONAR_SERIAL, SONAR_BAUD, SONAR_BUOY_RANGE_MIN_MM, \
                   SONAR_BUOY_RANGE_MAX_MM, SONAR_CONFIDENCE_MIN

# Ping Protocol sabitleri
_START1 = 0x42
_START2 = 0x52
_MSG_DISTANCE_SIMPLE = 1300
_MSG_REQUEST = 6


class PingSonar:
    def __init__(self):
        self._meas: Optional[Tuple[int, int]] = None   # (distance_mm, confidence)
        self._lock = threading.Lock()
        self._thread = None
        self._running = False
        self._port = None

    # ---------------------------------------------------------------- public
    @property
    def measurement(self) -> Optional[Tuple[int, int]]:
        """(distance_mm, confidence 0-100) | None."""
        with self._lock:
            return self._meas

    def distance_mm(self) -> Optional[int]:
        """Sadece mesafeyi dondurur, guven filtresi uygular."""
        m = self.measurement
        if m is None:
            return None
        dist, conf = m
        if conf < SONAR_CONFIDENCE_MIN:
            return None
        if not (SONAR_BUOY_RANGE_MIN_MM <= dist <= SONAR_BUOY_RANGE_MAX_MM):
            return None
        return dist

    def start(self):
        """Okuma thread'ini baslat."""
        try:
            self._port = serial.Serial(SONAR_SERIAL, SONAR_BAUD, timeout=1.0)
        except serial.SerialException as e:
            print(f"[SONAR] Port acilamadi ({SONAR_SERIAL}): {e}")
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"[SONAR] {SONAR_SERIAL}@{SONAR_BAUD} baslatildi.")

    def stop(self):
        self._running = False
        if self._port:
            self._port.close()

    # ---------------------------------------------------------------- private
    def _loop(self):
        """Surekli mesaj iste + oku."""
        while self._running:
            try:
                self._request()
                msg = self._read_message()
                if msg is not None:
                    with self._lock:
                        self._meas = msg
                time.sleep(0.1)  # 10 Hz
            except Exception as e:
                print(f"[SONAR] Okuma hatasi: {e}")
                time.sleep(0.5)

    def _request(self):
        """Ping'e distance_simple mesaj iste."""
        payload = struct.pack("<HH", _MSG_DISTANCE_SIMPLE, 0)
        self._send(_MSG_REQUEST, payload)

    def _send(self, msg_id: int, payload: bytes):
        length = len(payload)
        header = struct.pack("<BBHHBB", _START1, _START2,
                             length, msg_id, 0, 0)
        data = header + payload
        checksum = sum(data) & 0xFFFF
        packet = data + struct.pack("<H", checksum)
        self._port.write(packet)

    def _read_message(self) -> Optional[Tuple[int, int]]:
        """Bir mesaj oku, (distance_mm, confidence) ya da None dondur."""
        # Baslangic baytlarini bul
        b = self._port.read(1)
        while b and b[0] != _START1:
            b = self._port.read(1)
        if not b:
            return None
        b2 = self._port.read(1)
        if not b2 or b2[0] != _START2:
            return None

        header = self._port.read(6)
        if len(header) < 6:
            return None
        length, msg_id, src, dst = struct.unpack("<HHBB", header)

        payload = self._port.read(length)
        if len(payload) < length:
            return None
        self._port.read(2)  # checksum (basit surucude dogrulamiyoruz)

        if msg_id == _MSG_DISTANCE_SIMPLE and length >= 5:
            distance_mm = struct.unpack("<I", payload[:4])[0]
            confidence  = payload[4]
            return (distance_mm, confidence)
        return None

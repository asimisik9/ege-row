"""
Yer istasyonu (laptop) istemcisi.

Kullanim (laptopta):
    python3 -m comms.client          # ROV_IP = config.ROV_IP
    python3 -m comms.client 192.168.1.10

Klavye komutlari:
  s  → durum sor (state, heading, depth)
  a  → abort (tum motorlar dur)
  i/k/j/l/u/o → teleop (ileri/geri/sol/sag/yukari/asagi)
  [space]  → teleop sifirla (nötr)
  t        → teleop modundan cik (goreve don)
  q / Ctrl+C → cikarken abort gonder
"""
import json
import socket
import sys
import time
import threading

try:
    import msvcrt  # Windows
    def _getch():
        return msvcrt.getwch()
except ImportError:
    import tty, termios
    def _getch():
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

from config import ROV_IP, COMMS_PORT

TELEOP_STEP = 0.15   # bir tusa basmada eksen degisim miktari


class CommsClient:
    def __init__(self, host=None):
        self.host = host or ROV_IP
        self._sock = None
        self._axes = {"surge": 0.0, "yaw": 0.0, "heave": 0.0,
                      "roll": 0.0, "pitch": 0.0}
        self._teleop_mode = False
        self._running = False

    # ---------------------------------------------------------------- public
    def connect(self):
        """Jetson TCP sunucusuna baglan."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(5.0)
        self._sock.connect((self.host, COMMS_PORT))
        self._sock.settimeout(None)
        self._running = True
        print(f"Baglandi: {self.host}:{COMMS_PORT}")

    def run(self):
        """Ana klavye dongusu."""
        print("\n=== EGE ROV Yer Istasyonu ===")
        print("s=durum  a=abort  i/k/j/l/u/o=teleop  [space]=notr  t=teleop_off  q=cikis\n")
        while self._running:
            ch = _getch()
            if ch in ("q", "\x03"):
                self._send({"cmd": "abort"})
                break
            elif ch == "s":
                r = self._send({"cmd": "state"})
                if r:
                    print(f"  State={r.get('state')}  H={r.get('heading')}°"
                          f"  D={r.get('depth')}m  R={r.get('roll')}°  P={r.get('pitch')}°")
            elif ch == "a":
                self._send({"cmd": "abort"})
                print("  ABORT gonderildi.")
            elif ch == "t":
                self._send({"cmd": "teleop_off"})
                self._teleop_mode = False
                self._axes = {k: 0.0 for k in self._axes}
                print("  Gorev kontrolune donuldu.")
            elif ch == " ":
                self._axes = {k: 0.0 for k in self._axes}
                self._send_teleop()
            elif ch in "ikjluo":
                self._teleop_mode = True
                mapping = {
                    "i": ("surge",  TELEOP_STEP),
                    "k": ("surge", -TELEOP_STEP),
                    "j": ("yaw",   -TELEOP_STEP),
                    "l": ("yaw",    TELEOP_STEP),
                    "u": ("heave",  TELEOP_STEP),
                    "o": ("heave", -TELEOP_STEP),
                }
                ax, delta = mapping[ch]
                self._axes[ax] = max(-1.0, min(1.0, self._axes[ax] + delta))
                self._send_teleop()
                print(f"  Teleop: {self._axes}")

    def disconnect(self):
        self._running = False
        if self._sock:
            self._sock.close()

    # ---------------------------------------------------------------- private
    def _send(self, msg):
        """JSON satiri gonder, yaniti al."""
        try:
            self._sock.sendall((json.dumps(msg) + "\n").encode())
            data = b""
            while not data.endswith(b"\n"):
                chunk = self._sock.recv(1024)
                if not chunk:
                    return None
                data += chunk
            return json.loads(data.decode().strip())
        except Exception as e:
            print(f"  [HATA] {e}")
            return None

    def _send_teleop(self):
        self._send({"cmd": "teleop", **self._axes})


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else None
    client = CommsClient(host)
    try:
        client.connect()
        client.run()
    except ConnectionRefusedError:
        print(f"Baglanti reddedildi: {client.host}:{COMMS_PORT}. Jetson calistirildi mi?")
    except KeyboardInterrupt:
        pass
    finally:
        client.disconnect()
        print("Baglanti kapatildi.")


if __name__ == "__main__":
    main()

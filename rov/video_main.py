import sys
import subprocess
import os

print("=" * 60)
print(" DIKKAT: video_main.py ARTIK KULLANILMIYOR (DEPRECATED)!")
print(" Yeni SensorHub mimarisi (threadli hizli I2C) icin tum gorevler")
print(" artik main.py icinde birlestirildi.")
print(" Otomatik olarak 'python3 rov/main.py' baslatiliyor...")
print("=" * 60)
print()

# Script'in oldugu dizini bul ve main.py'yi calistir
rov_dir = os.path.dirname(os.path.abspath(__file__))
main_py = os.path.join(rov_dir, "main.py")

sys.exit(subprocess.call([sys.executable, main_py] + sys.argv[1:]))

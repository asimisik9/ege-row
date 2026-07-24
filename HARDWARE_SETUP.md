# EGE ROV — Hardware Setup & Wiring Guide (`ege-row`)

> **Motor Layout: 4 Vertical (V_FL, V_FR, V_RL, V_RR) + 2 Horizontal (H_L, H_R)**  
> This guide covers every physical connection needed to go from bare hardware to a running ROV, in the exact order you should perform the steps.

---

## Part 1 — System Overview

```
Battery Pack (22.2V 6S4P, ASPİLSAN)
      │
      ├─► Power Distribution Board (Custom PCB)
      │     ├─► 12V ──► XL4016 Step-Down ──► Jetson Xavier NX
      │     ├─► 5V  ──► PCA9685 V+ rail (servo/signal power)
      │     ├─► 3.3V ──► Sensor bus
      │     ├─► 22.2V ──► 6× BLU 30A ESCs ──► 6× Degz Robotics M1 Motors
      │     └─► Hardware Leak Cutoff (direct, no software)
      │
      └─► Jetson Xavier NX (I2C Bus 7 on 40-pin header)
            ├─► I2C ──► PCA9685 PWM Driver (0x40)
            │               ├─ CH0: V_FL (Front-Left Vertical)
            │               ├─ CH1: V_RL (Rear-Left Vertical)
            │               ├─ CH2: H_L  (Left Horizontal)
            │               ├─ CH3: V_FR (Front-Right Vertical)
            │               ├─ CH4: V_RR (Rear-Right Vertical)
            │               └─ CH5: H_R  (Right Horizontal)
            ├─► I2C ──► MPU-9250 IMU (0x68)
            │               └─► AK8963 Magnetometer (0x0C, via bypass)
            ├─► I2C ──► MS5837-30BA Depth Sensor (0x76)
            └─► Ethernet ──► Tether ──► Laptop
```

**Motor Layout (Top View):**
```
            FRONT
    V_FL ───────── V_FR
      |               |
  H_L ◄──── ROV ────► H_R
      |               |
    V_RL ───────── V_RR
            REAR
```

---

## Part 2 — Jetson Xavier NX 40-Pin Header

> Pin 1 is marked by a triangle/arrow on the board near the connector.
> **On the Xavier NX, the I2C bus exposed on pins 3/5 is typically Bus 8.**
> Verify first with `sudo i2cdetect -l`. Update `I2C_BUS` in `config.py` accordingly.

```
Pin 1   ── 3.3V Power  ── Sensor VCC rail (MPU-9250, MS5837)
Pin 2   ── 5V Power    ── (spare)
Pin 3   ── I2C SDA     ── Shared I2C data (PCA9685, MPU-9250, MS5837)
Pin 4   ── 5V Power    ── PCA9685 VCC
Pin 5   ── I2C SCL     ── Shared I2C clock
Pin 6   ── GND         ── Common ground
Pin 9   ── GND         ── Common ground
Pin 14  ── GND         ── Common ground
Pin 17  ── 3.3V Power  ── (spare / pull-ups)
Pin 25  ── GND         ── Common ground
```

> [!IMPORTANT]
> The Xavier NX **does not have hardware PWM pins** that work reliably for ESC driving at 50Hz. This is why we use the PCA9685 over I2C. Do not attempt to drive ESC signal lines directly from GPIO pins.

---

## Part 3 — PCA9685 PWM Driver (I2C address: 0x40)

### Wiring to Jetson

| PCA9685 Pin | Connect To | Notes |
|-------------|-----------|-------|
| VCC | Jetson Pin 4 (5V) | Logic power for the chip |
| GND | Jetson Pin 6 (GND) | Common ground |
| SDA | Jetson Pin 3 (I2C SDA) | Data line |
| SCL | Jetson Pin 5 (I2C SCL) | Clock line |
| OE | GND | Pull LOW to always-enable outputs |
| V+ | PDB 5V rail | Servo/ESC signal power rail |

> [!NOTE]
> The PCA9685 breakout board has built-in 10kΩ pull-up resistors on SDA/SCL. This is sufficient for 3 devices on the same bus (PCA9685 + MPU-9250 + MS5837). No extra pull-ups needed.

### Channel Assignments (matches `config.py`)

| PCA9685 Channel | Code Name | Physical Motor | Function |
|-----------------|-----------|---------------|---------|
| **CH 0** | `V_FL` | Front-Left | **Vertical** (depth/roll/pitch) |
| **CH 1** | `V_RL` | Rear-Left | **Vertical** (depth/roll/pitch) |
| **CH 2** | `H_L` | Left side | **Horizontal** (surge/yaw) |
| **CH 3** | `V_FR` | Front-Right | **Vertical** (depth/roll/pitch) |
| **CH 4** | `V_RR` | Rear-Right | **Vertical** (depth/roll/pitch) |
| **CH 5** | `H_R` | Right side | **Horizontal** (surge/yaw) |

### Connecting ESC Signal Wires to PCA9685

Each ESC signal cable has 3 wires. Wire them to the PCA9685 output pins like this:

```
PCA9685 Output Pin Block (3 pins per channel):
  [GND] [VCC] [SIG]

ESC Signal Cable:
  Black (GND) ──► PCA9685 GND pin
  Red   (VCC) ──► DO NOT CONNECT (ESC is self-powered from battery)
  White/Yellow ─► PCA9685 SIG pin
```

> [!WARNING]
> Only connect the **Signal** and **GND** wires from the ESC signal cable to the PCA9685. Never connect the ESC's red VCC wire to the PCA9685 V+ — this would try to back-power the PCA9685 from the ESC's internal BEC which can cause conflicts or damage.

---

## Part 4 — BLU 30A ESCs (×6)

### Per-ESC Wiring

| ESC Wire | Connect To |
|----------|-----------|
| Thick Red (+) | PDB main 22.2V output |
| Thick Black (−) | PDB GND |
| 3× Phase wires | Motor's 3× phase wires (any order first, swap 2 to reverse direction) |
| Thin Signal (white/yellow) | PCA9685 SIG pin on assigned channel |
| Thin GND (black) | PCA9685 GND pin on same channel |

### ESC Arming Sequence
The BLU 30A ESC requires a **1750µs neutral signal** before it responds to any thrust commands. The `Thrusters.arm()` method in `hal/thrusters.py` sends 1750µs for **2 seconds** on startup.

**Listen for the beep sequence on power-up:**
1. Plug in battery → ESC plays startup tones
2. Once script calls `arm()` → ESC sends a double-beep confirming it's armed
3. Only after that beep will the ESC respond to thrust commands

### Motor Direction: How to Determine `MOTOR_DIRECTION` in `config.py`

**Before water, on dry land:**
```bash
cd ege-row/rov
python3 main.py --test-motors
```

This spins each motor for 2 seconds at low power (0.15 throttle). For each motor:
- If it spins the **correct direction** → leave `MOTOR_DIRECTION` as `1`
- If it spins the **wrong direction** → either:
  - **Hardware fix (recommended):** Swap any 2 of the 3 phase wires on that motor/ESC
  - **Software fix:** Set that motor's direction to `-1` in `config.py`

**Correct spin directions for your layout:**
```
Top view:
   V_FL (CCW) ┌─────┐ (CW) V_FR
              │     │
   H_L (→)  ◄─  ROV  ─►  H_R (←)   (H_L pushes water right, H_R pushes left for forward)
              │     │
   V_RL (CW) └─────┘ (CCW) V_RR

Vertical motors should all push DOWN for heave+ (diving).
If any vertical motor blows air UP, it's reversed — swap two phase wires.
```

---

## Part 5 — MPU-9250 IMU (I2C address: 0x68)

The `ege-row` driver communicates with **both** the MPU-9250 (accel/gyro at 0x68) AND its internal **AK8963 magnetometer** (at 0x0C via I2C bypass mode). This is set up automatically by `sensors/imu.py`.

### Wiring

| MPU-9250 Pin | Connect To | Notes |
|--------------|-----------|-------|
| VCC | Jetson Pin 1 (3.3V) | **3.3V ONLY — not 5V, will damage chip** |
| GND | Jetson Pin 6 (GND) | |
| SDA | Jetson Pin 3 (I2C SDA) | Shared bus |
| SCL | Jetson Pin 5 (I2C SCL) | Shared bus |
| AD0 | GND | Sets address to 0x68. If HIGH → 0x69 (conflicts with MS5837 at 0x76, no issue) |
| INT | Not connected | |
| NCS | 3.3V or leave floating | Disables SPI mode |

> [!IMPORTANT]
> The AK8963 magnetometer inside the MPU-9250 is accessed by putting the MPU in **I2C bypass mode** (done by `imu.py` in `__init__`). You do NOT need to wire AK8963 separately — it sits on the same 4 wires. The driver sends `INT_PIN_CFG = 0x02` to open the bypass, then addresses the magnetometer at **0x0C** directly on your I2C bus.

### Verifying MPU-9250 is visible
After wiring, with Jetson powered:
```bash
# Replace 7 with your actual I2C bus number
sudo i2cdetect -y 7
```
Expected output — you should see **0x0c** (magnetometer) AND **0x68** (MPU):
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- 0c -- -- --
...
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: -- -- -- -- -- -- 76 --
```
> If you only see 0x68 but not 0x0c: the bypass hasn't been written yet (normal until first `Mpu9250()` init). Run the script and check again. If 0x0c never appears, the MPU-9250 module's bypass pin may be hardwired off — check your module's schematic.

---

## Part 6 — MS5837-30BA Depth Sensor (I2C address: 0x76)

### Wiring

| MS5837 Pin | Connect To | Notes |
|------------|-----------|-------|
| VCC | Jetson Pin 1 (3.3V) | **3.3V ONLY** |
| GND | Jetson Pin 6 (GND) | |
| SDA | Jetson Pin 3 (I2C SDA) | Shared bus |
| SCL | Jetson Pin 5 (I2C SCL) | Shared bus |

> [!NOTE]
> The MS5837 has **no built-in pull-up resistors**. Rely on the PCA9685 module's pull-ups. If the sensor doesn't appear on `i2cdetect`, add a 4.7kΩ resistor from SDA to 3.3V and another from SCL to 3.3V.

### Surface Zeroing
The `ege-row` code calls `depth.zero_at_surface()` at mission start, which reads the current pressure as the atmospheric reference. This means **the sensor must be powered and stable at the surface** before the mission starts — it auto-calls this during `mission.start()`.

---

## Part 7 — Power Wiring Summary

```
Battery (+22.2V) ──► 60A Main Fuse ──► PDB
                                         │
                          ┌──────────────┼────────────────┐
                          ▼              ▼                 ▼
                    XL4016 Buck      6× ESC (+)        LED Driver
                    → 12V → Jetson   (direct 22.2V)
                                         │
                                    6× Motor Phases
```

> [!CAUTION]
> The ESCs draw up to **30A each** at full thrust. With 6 ESCs, peak draw is ~180A. Ensure all high-current wiring uses **appropriate gauge wire** (minimum 12AWG for ESC power runs, 10AWG or thicker for main battery leads). Thin wire = heat = fire risk in the enclosed watertight housing.

> [!WARNING]
> **Never connect/disconnect the battery with thrusters in the water unless you know motors are stopped.** The BLU 30A ESCs initialize with neutral, but any transient on power-up can cause a brief motor pulse. Always handle the ROV clear of people and propellers when connecting the battery.

---

## Part 8 — Software Setup on the Jetson

### Step 1: Install Dependencies
```bash
# On the Jetson (SSH in over ethernet, or directly)
pip3 install smbus2 adafruit-circuitpython-pca9685 adafruit-blinka
```

### Step 2: Verify I2C Bus Number
```bash
sudo i2cdetect -l
# Look for: i2c-7  i2c  ...  or i2c-8
# Then scan whichever bus your devices are wired to:
sudo i2cdetect -y 7   # Try 7, then 8
```
You should see: **0x0c** (AK8963 after first run), **0x40** (PCA9685), **0x68** (MPU-9250), **0x76** (MS5837)

### Step 3: Update `config.py`
```bash
cd ege-row/rov
nano config.py
```
Make these two changes:
```python
SIM_MODE = False   # ← Change True to False for real hardware
I2C_BUS  = 7      # ← Set to whichever bus i2cdetect found your devices on (7 or 8)
FLUID_DENSITY = 997   # ← 997 for pool/freshwater, 1025 for seawater
```

### Step 4: Run IMU Calibration
This step is **mandatory before the first real mission**. It measures your gyro drift and compass hard/soft-iron errors and writes them into `config.py` automatically.

```bash
cd ege-row/rov
python3 calibrate_imu.py
```

**Follow the on-screen prompts:**
1. Place ROV perfectly still on a flat surface → press Enter → wait 5 seconds (gyro bias)
2. Pick up ROV and **slowly rotate it in all directions** (figure-8 motion, tumble it forward/backward, left/right) for 15 seconds (magnetometer calibration)
3. Results are printed and `config.py` is updated automatically. Old values backed up to `config.py.bak`.

### Step 5: Test Motors (Dry Land, Away from People!)
```bash
python3 main.py --test-motors
```
Each motor spins for 2 seconds at 15% throttle in sequence: V_FL → V_FR → V_RL → V_RR → H_L → H_R.

**What to check:**
- All 6 motors spin ✓
- Each vertical motor (V_*) pushes **downward** (correct for diving) ✓
- H_L pushes water to the **right** (ROV moves left → correct for H_L = left motor) ✓
- H_R pushes water to the **left** (correct for H_R = right motor) ✓

**If a motor is reversed:** Swap any 2 of its 3 motor phase wires, OR set `-1` in `MOTOR_DIRECTION` in `config.py`.

### Step 6: Test Simulator on Laptop First (Recommended)
Before pool, verify the entire video demo mission logic runs correctly:
```bash
# On your laptop (not the Jetson), SIM_MODE=True (default)
cd ege-row/rov
python3 run_sim.py
```
This runs the full mission in software simulation with approximate ROV physics. The output shows the XY trajectory — you should see a rectangle path with a circle at one corner.

---

## Part 9 — First Pool Test Checklist

Run through this list before every water entry:

```
PRE-DIVE
[ ] All 6 ESC signal wires verified plugged into correct PCA9685 channels
[ ] Motor direction confirmed correct during dry land test
[ ] All watertight housing O-rings clean, seated, and housing fully closed
[ ] config.py: SIM_MODE = False
[ ] config.py: I2C_BUS correct (verified with i2cdetect)
[ ] config.py: FLUID_DENSITY = 997 (pool) or 1025 (sea)
[ ] IMU calibration done (GYRO_BIAS / MAG_OFFSET / MAG_SCALE set)
[ ] Ethernet connection to Jetson verified (ping 192.168.1.10)
[ ] Safety tether (50m) attached to ROV
[ ] Mission ABORT: mıknatıs hazır (magnetic emergency switch within reach)

LAUNCH SEQUENCE
1. Place ROV at water surface
2. Connect battery BEFORE submerging (Jetson boots, ESCs arm)
3. SSH into Jetson: ssh user@192.168.1.10
4. cd ege-row/rov && python3 main.py
5. Terminal shows "gorev baslatiliyor..." and 10-second countdown
6. Lower ROV into water
7. Mission starts autonomously at T+10s (start_delay_s in config.py)

EMERGENCY ABORT
- Physical: Approach ROV with magnet → triggers magnetic switch → all power cut
- Software: Press Ctrl+C in SSH terminal → mission.abort() runs → motors stop
```

---

## Part 10 — Ethernet Connection to Jetson

The Jetson communicates over the tether via its built-in Ethernet port.

**Configure static IP on the Jetson (run once):**
```bash
sudo nmcli connection modify "Wired connection 1" \
    ipv4.method manual \
    ipv4.addresses 192.168.1.10/24
sudo nmcli connection up "Wired connection 1"
```

**Configure your laptop:**
- Set laptop Ethernet to static: `192.168.1.100 / 255.255.255.0`

**Test connectivity:**
```bash
ping 192.168.1.10     # From laptop
ssh user@192.168.1.10 # SSH into Jetson
```

---

## Part 11 — I2C Bus Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| No devices on `i2cdetect` | Wrong bus number | Try `-y 7`, `-y 8`, `-y 1` |
| 0x68 visible but 0x0c not | Normal until first `Mpu9250()` init | Run any Python script that creates `Mpu9250()` |
| 0x40 not visible | PCA9685 OE pin floating HIGH | Pull OE to GND |
| `OSError: [Errno 121] Remote I/O error` | Weak pull-ups or too many devices | Add external 4.7kΩ pull-up per line |
| `FileNotFoundError: /dev/i2c-7` | I2C not enabled in kernel | Run `sudo modprobe i2c-dev` or add to `/etc/modules` |

> [!TIP]
> After SSH, you can install `i2c-tools` with `sudo apt install i2c-tools` to get the `i2cdetect` command if it's not already available.

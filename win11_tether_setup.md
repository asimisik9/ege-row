# Windows 11 & NVIDIA Jetson Direct Tether Setup Guide
**For Underwater ROVs, Autonomous Robotics, and Surface Ground Control Stations**

This guide provides a step-by-step, bulletproof configuration to establish a reliable, high-speed direct Ethernet connection between a **Windows 11 Ground Control Station (GCS)** PC and an **NVIDIA Jetson** embedded computer (subsea/field unit).

It eliminates common connection issues caused by Windows network behaviors (APIPA IP overrides, adapter power-saving link drops, MediaSense resetting interface bindings, ICS disabling itself on reboot) and configures **Passwordless SSH** and **SSH Keepalives** for seamless autonomous AI agent execution and uninterrupted remote control.

---

## Network Architecture & Specifications

| Parameter | Surface Station (Windows 11 PC) | Remote Unit (NVIDIA Jetson) |
| :--- | :--- | :--- |
| **Interface Role** | Gateway & Internet Share Master | Tether Node / Robot Client |
| **IPv4 Address** | `192.168.137.1` | `192.168.137.2` |
| **Subnet Mask** | `255.255.255.0` (`/24`) | `255.255.255.0` (`/24`) |
| **Default Gateway** | *(Leave Blank)* | `192.168.137.1` |
| **DNS Servers** | *(Handled by Wi-Fi/System)* | `8.8.8.8`, `1.1.1.1` |

---

## Part 1: Windows 11 Host Configuration

### Step 1: Assign Static IP to Ethernet Adapter

1. Press `Win + R`, type `ncpa.cpl`, and hit **Enter** to open Network Connections.
2. Right-click your physical **Ethernet Adapter** and select **Properties**.
3. Select **Internet Protocol Version 4 (TCP/IPv4)** and click **Properties**.
4. Configure the following settings:
   * Select **Use the following IP address**:
     * **IP address:** `192.168.137.1`
     * **Subnet mask:** `255.255.255.0`
     * **Default gateway:** *(Leave empty)*
   * Select **Use the following DNS server addresses**:
     * **Preferred DNS server:** `8.8.8.8`
     * **Alternate DNS server:** `1.1.1.1`
5. Click **OK** and then **Close**.

---

### Step 2: Enable Internet Connection Sharing (ICS)

Passes Wi-Fi internet access down the tether cable to the Jetson for remote code updates (`git pull`, `apt update`).

1. In `ncpa.cpl`, right-click your active **Wi-Fi Adapter** and select **Properties**.
2. Navigate to the **Sharing** tab.
3. Check **"Allow other network users to connect through this computer's Internet connection"**.
4. If a drop-down menu titled *Home networking connection* appears, select your **Ethernet Adapter**. *(Note: If only two adapters exist on the system, Windows hides this drop-down and auto-selects the Ethernet adapter).*
5. Click **OK**.

---

### Step 3: Disable Adapter Power Saving & Energy Efficiency

Prevent Windows from putting the physical Ethernet chip into sleep mode when idle or when signal flickers occur over long umbilical cables.

1. Press `Win + X` and select **Device Manager**.
2. Expand **Network adapters**, right-click your Ethernet controller (e.g., *Realtek PCIe GbE* or *Intel Ethernet*) $\rightarrow$ **Properties**.
3. Go to the **Power Management** tab:
   * **Uncheck** *"Allow the computer to turn off this device to save power"*.
4. Go to the **Advanced** tab and set the following to **Disabled** (Turkish translations included for reference):
   * **Energy Efficient Ethernet / Advanced EEE** (*Enerji Tasarruflu Ethernet*) $\rightarrow$ `Disabled` (`Devre Dışı`)
   * **Green Ethernet** (*Yeşil Ethernet*) $\rightarrow$ `Disabled` (`Devre Dışı`)
   * **Power Saving Mode / Auto Disable Gigabit** (*Power Saving Mode / Gigabit'i Otomatik Devre Dışı Bırak*) $\rightarrow$ `Disabled` (`Devre Dışı`)
   * **ARP Offload / NS Offload** (*ARP Yük Boşaltma / NS Yük Boşaltma*) $\rightarrow$ `Disabled` (`Devre Dışı`)
5. Keep **IPv4 Checksum Offload** (*IPv4 Yük Boşaltmayı Sağla*) set to **Enabled** (`Etkin`).

---

### Step 4: System Registry & TCP/IP Stack Hardening

Run Windows PowerShell as Administrator to lock down adapter behavior:

```powershell
# 1. Disable APIPA auto-configuration system-wide
New-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Services\Tcpip\Parameters" -Name "IPAutoconfigurationEnabled" -Value 0 -PropertyType DWORD -Force

# 2. Disable MediaSense (prevents Windows from resetting IP bindings on link drop)
New-ItemProperty -Path "HKLM:\SYSTEM\CurrentControlSet\Services\Tcpip\Parameters" -Name "DisableDHCPMediaSense" -Value 1 -PropertyType DWORD -Force

# 3. Force Internet Connection Sharing (ICS) to persist across reboots and Wi-Fi disconnects
New-ItemProperty -Path "HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\SharedAccess" -Name "EnableRebootPersistConnection" -Value 1 -PropertyType DWORD -Force

# 4. Set Internet Connection Sharing service to start automatically
Set-Service -Name "SharedAccess" -StartupType Automatic
Restart-Service -Name "SharedAccess"

# 5. Disable Duplicate Address Detection (DAD) Transmits on Ethernet Interface
# (Replace '12' with your adapter's actual ifIndex from Get-NetIPInterface -AddressFamily IPv4)
Set-NetIPInterface -InterfaceIndex 12 -AddressFamily IPv4 -DadTransmits 0 -PolicyStore PersistentStore
```

*Reboot the Windows PC after applying registry settings.*

---

## Part 2: NVIDIA Jetson Configuration (ALREADY DONE)

Configure the Jetson to hold its static IP (`192.168.137.2`) and route traffic to `192.168.137.1` as default gateway.

### Method A: Via NetworkManager CLI (`nmcli`) — *Recommended*

```bash
# 1. Locate connection name (usually "Wired connection 1" or "eth0")
nmcli connection show

# 2. Configure Static IP, Gateway, and DNS
sudo nmcli connection modify "Wired connection 1" ipv4.addresses "192.168.137.2/24"
sudo nmcli connection modify "Wired connection 1" ipv4.gateway "192.168.137.1"
sudo nmcli connection modify "Wired connection 1" ipv4.dns "8.8.8.8, 1.1.1.1"
sudo nmcli connection modify "Wired connection 1" ipv4.method "manual"

# 3. Bring connection up
sudo nmcli connection up "Wired connection 1"
```

---

### Method B: Via Netplan (`/etc/netplan/01-netcfg.yaml`)

```yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.137.2/24
      routes:
        - to: default
          via: 192.168.137.1
      nameservers:
        addresses: [8.8.8.8, 1.1.1.1]
```
Apply with: `sudo netplan apply`

---

### Hardware Permissions & Sudo Configuration (Optional!)

To ensure Python/ROS scripts and local AI agents can interact with hardware without pausing for passwords:

1. **Grant Non-Sudo Hardware Access:**
   ```bash
   sudo usermod -aG dialout,i2c,gpio,video,docker sungur
   ```
2. **(Optional) Passwordless Sudo for Fully Autonomous Agent Execution:**
   ```bash
   sudo visudo -f /etc/sudoers.d/sungur-nopasswd
   ```
   Add the line:
   ```text
   sungur ALL=(ALL) NOPASSWD: ALL
   ```

---

## Part 3: SSH Key-Based Authentication & Keepalive Configuration(Recovery+Quality of Life)

This section enables **instant passwordless login** and **fast subsea failure detection** so that both humans and automated AI agents (in IDEs like Antigravity or VS Code) can execute remote scripts cleanly without hanging.

### Step 1: Create SSH Keepalive & Host Alias Config File

Windows user SSH settings are stored at `C:\Users\<YourUsername>\.ssh\config`. 

Run this command in **Windows PowerShell** to automatically create the folder and configuration file:

```powershell
New-Item -ItemType Directory -Force -Path "$env:USERPROFILE\.ssh"

@'
Host sungur
    HostName 192.168.137.2
    User sungur
    ServerAliveInterval 2
    ServerAliveCountMax 5

Host jetson
    HostName 192.168.137.2
    User sungur
    ServerAliveInterval 2
    ServerAliveCountMax 5
'@ | Out-File -Encoding ascii -FilePath "$env:USERPROFILE\.ssh\config" -Append
```

> **What this does:**
> * **`HostAlias` (`sungur` / `jetson`):** Allows you to connect by simply typing `ssh sungur` or `ssh jetson` instead of typing the full IP/user combination every time.
> * **`ServerAliveInterval 2` & `ServerAliveCountMax 5`:** Sends a keepalive heartbeat pulse every 2 seconds down the tether cable. If the tether flickers or drops for >10 seconds, SSH terminates cleanly instead of freezing indefinitely.

---

### Step 2: Generate SSH Key Pair on Windows

In **PowerShell** on your Windows PC, generate an ED25519 SSH key pair:

```powershell
ssh-keygen -t ed25519
```
*Press **Enter** for all prompts to accept the default path (`$env:USERPROFILE\.ssh\id_ed25519`) and leave the passphrase empty.*

---

### Step 3: Copy Public Key to Jetson (Passwordless Login Setup)

Run this PowerShell command to transfer your PC's public key to the Jetson's `authorized_keys` file:

```powershell
type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh sungur@192.168.137.2 "mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys"
```
*(Enter your Jetson user password **one last time** when prompted).*

> **Note:** This step does **NOT** change your Jetson's password. It simply registers a secure "VIP key" for your Windows PC. Other devices will still be prompted for a password as normal.

---

### Step 4: Verify Passwordless Connection

Test the new passwordless setup from PowerShell:

```powershell
ssh sungur
```

If configured correctly, you will instantly log into the Jetson terminal without being prompted for a password!

---

## Part 4: Local AI Agent & IDE Workflow (Antigravity / VS Code)

With Passwordless SSH and Keepalives configured, your local AI IDE agent (e.g. Antigravity running on Windows) can edit code locally, push files down the tether cable, execute subsea test scripts, and fix errors completely autonomously.

### Recommended Project Rule (`.antigravityrules` or System Prompt)

Add this instruction block to your workspace root so the local AI knows how to test on the Jetson:

```markdown
### Execution & Testing Rules for Remote Jetson Node
- The target robot runs on a remote NVIDIA Jetson at IP `192.168.137.2` (SSH alias: `sungur`).
- All code editing is performed locally on Windows.
- To test code changes on the Jetson:
  1. Transfer modified files over the tether using SCP:
     `scp -r .\src\* sungur:~/rov_code/src/`
  2. Execute the test script remotely via SSH:
     `ssh sungur "python3 ~/rov_code/src/test_motors.py"`
  3. Read the terminal output (stdout/stderr) returned by the SSH command to verify or debug.
```

---

## Part 5: Pre-Dive Sanity Check Routine

Perform this 4-step check before every subsea deployment:

### 1. Check Topside Network Adapter
In Windows Terminal (`cmd` or PowerShell):
```cmd
ipconfig
```
*Verify Ethernet adapter reports **IPv4 Address: `192.168.137.1`**.*

### 2. Ping the Subsea Jetson
```cmd
ping 192.168.137.2
```

### 3. Passwordless SSH Test
```cmd
ssh sungur
```
*Should open the Jetson terminal instantly without asking for a password.*

### 4. Verify Subsea Internet Access
Inside the Jetson terminal:
```bash
ping -c 2 google.com
```

---

## Part 6: Subsea & Physical Hardware Troubleshooting

If software settings are verified but dropouts occur during field operations, inspect physical hardware factors:

1. **Signal Attenuation over Long Tethers (>50 meters):**
   * Standard Ethernet limits are 100 meters. For long umbilical cables, force the Windows adapter speed to **100 Mbps Full Duplex** under *Device Manager $\rightarrow$ Advanced $\rightarrow$ Speed & Duplex* for higher noise tolerance.
2. **Motor & Thruster EMI Noise:**
   * High-current BLDC motor lines running parallel to data cables inject electromagnetic interference. Use **Shielded Twisted Pair (STP / SFTP)** Ethernet cable and ground the shield at the surface enclosure.
3. **Connector Corrosion & Moisture:**
   * Apply dielectric grease to dry-mate connectors and inspect O-ring seals on subsea penetrators.
4. **Thruster Safety Watchdog:**
   * Ensure your robot control scripts implement a heartbeat watchdog (e.g., disarming thrusters if no valid control packet is received for >2.0 seconds).
5. **Clean Shutdowns:**
   * Always shut down the Jetson properly (`sudo shutdown -h now`) before cutting battery power to protect the SD card from corruption.
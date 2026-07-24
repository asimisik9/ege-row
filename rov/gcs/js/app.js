/* ==========================================================================
   EGE ROV — Ground Control Station (GCS) Main Application Controller
   ========================================================================== */

document.addEventListener('DOMContentLoaded', () => {
  const hud = new HUDController('hud-canvas');
  let teleopActive = false;

  // DOM Elements
  const badgeConn = document.getElementById('badge-conn');
  const badgeArm = document.getElementById('badge-arm');
  const badgeEstop = document.getElementById('badge-estop');
  const badgeMission = document.getElementById('badge-mission');

  const btnArm = document.getElementById('btn-arm');
  const btnAbort = document.getElementById('btn-abort');

  const valDepth = document.getElementById('val-depth');
  const valTargetDepth = document.getElementById('val-target-depth');
  const valHeading = document.getElementById('val-heading');
  const valTargetHeading = document.getElementById('val-target-heading');
  const valPitch = document.getElementById('val-pitch');
  const valRoll = document.getElementById('val-roll');
  const valPressure = document.getElementById('val-pressure');
  const valTemp = document.getElementById('val-temp');

  const arDepth = document.getElementById('ar-depth');
  const arHeading = document.getElementById('ar-heading');

  const consoleBox = document.getElementById('console-box');

  // Thruster Bar Elements
  const thrusters = {
    V_FL: { fill: document.getElementById('bar-vfl'), lbl: document.getElementById('lbl-vfl') },
    V_FR: { fill: document.getElementById('bar-vfr'), lbl: document.getElementById('lbl-vfr') },
    V_RL: { fill: document.getElementById('bar-vrl'), lbl: document.getElementById('lbl-vrl') },
    V_RR: { fill: document.getElementById('bar-vrr'), lbl: document.getElementById('lbl-vrr') },
    H_L:  { fill: document.getElementById('bar-hl'),  lbl: document.getElementById('lbl-hl') },
    H_R:  { fill: document.getElementById('bar-hr'),  lbl: document.getElementById('lbl-hr') },
  };

  // ── 1. Canlı Telemetri Döngüsü (20Hz Poll) ──────────────────────────────
  async function pollTelemetry() {
    try {
      const res = await fetch('/api/telemetry');
      if (!res.ok) throw new Error('HTTP ' + res.status);
      const data = await res.json();
      
      updateUI(data);
      hud.update(data);

      badgeConn.className = 'badge online';
      badgeConn.querySelector('.lbl').textContent = 'CANLI BAĞLANTI (20Hz)';
    } catch (e) {
      badgeConn.className = 'badge';
      badgeConn.querySelector('.lbl').textContent = 'BAĞLANTI KESİLDİ';
    } finally {
      setTimeout(pollTelemetry, 50); // 20Hz
    }
  }

  function updateUI(data) {
    if (!data) return;

    // Badges
    if (data.armed) {
      badgeArm.className = 'badge online';
      badgeArm.querySelector('.lbl').textContent = 'ARMED';
      btnArm.textContent = 'DISARM VEHICLE';
      btnArm.className = 'btn btn-arm armed';
    } else {
      badgeArm.className = 'badge';
      badgeArm.querySelector('.lbl').textContent = 'DISARMED';
      btnArm.textContent = 'ARM VEHICLE';
      btnArm.className = 'btn btn-arm';
    }

    if (data.estop) {
      badgeEstop.className = 'badge online';
      badgeEstop.querySelector('.lbl').textContent = 'E-STOP AKTİF!';
      badgeEstop.querySelector('.dot').style.background = '#ff0055';
    } else {
      badgeEstop.className = 'badge';
      badgeEstop.querySelector('.lbl').textContent = 'E-STOP NORMAL';
    }

    badgeMission.textContent = 'DURUM: ' + (data.state || 'IDLE');

    // Values
    if (data.depth !== undefined) {
      valDepth.innerHTML = `${data.depth.toFixed(2)} <small>m</small>`;
      valTargetDepth.textContent = `Hedef: ${data.target_depth.toFixed(2)}m`;
      arDepth.textContent = data.depth.toFixed(2);
    }

    if (data.heading !== undefined) {
      valHeading.innerHTML = `${data.heading.toFixed(1)} <small>°</small>`;
      valTargetHeading.textContent = `Hedef: ${data.target_heading.toFixed(1)}°`;
      arHeading.textContent = Math.round(data.heading).toString().padStart(3, '0');
    }

    if (data.pitch !== undefined) valPitch.innerHTML = `${data.pitch.toFixed(1)} <small>°</small>`;
    if (data.roll !== undefined) valRoll.textContent = data.roll.toFixed(1);
    if (data.pressure_mbar !== undefined) valPressure.innerHTML = `${Math.round(data.pressure_mbar)} <small>mbar</small>`;
    if (data.temp_c !== undefined) valTemp.textContent = data.temp_c.toFixed(1);

    // Thrusters
    if (data.thrusters) {
      for (const [key, val] of Object.entries(data.thrusters)) {
        if (thrusters[key]) {
          const pct = Math.abs(val) * 100;
          thrusters[key].fill.style.width = pct + '%';
          thrusters[key].lbl.textContent = `%${Math.round(pct * (val < 0 ? -1 : 1))}`;
          thrusters[key].fill.style.background = val < 0 ? '#ff0055' : '#00f3ff';
        }
      }
    }
  }

  // ── 2. Komut Gönderme Yardımcısı ─────────────────────────────────────────
  async function sendCommand(cmd, payload = {}) {
    try {
      const res = await fetch('/api/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ cmd, ...payload })
      });
      const ret = await res.json();
      if (ret.ok) {
        logConsole(ret.message || `${cmd} başarılı`, 'info');
      } else {
        logConsole(`Hata: ${ret.error}`, 'err');
      }
    } catch (e) {
      logConsole(`Ağ hatası: ${e.message}`, 'err');
    }
  }

  // ── 3. Buton Event Bağlantıları ──────────────────────────────────────────
  btnArm.addEventListener('click', () => {
    const isArmed = btnArm.classList.contains('armed');
    sendCommand(isArmed ? 'disarm' : 'arm');
  });

  btnAbort.addEventListener('click', () => {
    sendCommand('abort');
  });

  document.getElementById('btn-winch-deploy').addEventListener('click', () => sendCommand('winch_deploy'));
  document.getElementById('btn-winch-retract').addEventListener('click', () => sendCommand('winch_retract'));
  document.getElementById('btn-minrov_back').addEventListener('click', () => sendCommand('minrov_back'));

  // Görev Butonları
  document.getElementById('btn-m-video').addEventListener('click', () => {
    logConsole('Video gösterimi görevi seçildi', 'info');
  });
  document.getElementById('btn-m-line').addEventListener('click', () => {
    logConsole('Görev 1 (Hat Takibi) seçildi', 'info');
  });
  document.getElementById('btn-m-nav').addEventListener('click', () => {
    logConsole('Görev 2 (Navigasyon) seçildi', 'info');
  });

  // PID Tuner
  document.getElementById('btn-update-pid').addEventListener('click', () => {
    const target = document.getElementById('sel-pid-target').value;
    const kp = parseFloat(document.getElementById('pid-kp').value);
    const ki = parseFloat(document.getElementById('pid-ki').value);
    const kd = parseFloat(document.getElementById('pid-kd').value);
    sendCommand('set_pid', { pid_name: target, kp, ki, kd });
  });

  // AR Overlay Toggle
  document.getElementById('chk-ar-overlay').addEventListener('change', (e) => {
    document.getElementById('ar-overlay').style.display = e.target.checked ? 'block' : 'none';
  });

  // ── 4. Klavye Teleop Sürüş Mantığı ───────────────────────────────────────
  const activeKeys = {};
  window.addEventListener('keydown', (e) => {
    if (['INPUT', 'SELECT'].includes(e.target.tagName)) return;
    activeKeys[e.key.toLowerCase()] = true;
    updateKeyboardTeleop();
  });

  window.addEventListener('keyup', (e) => {
    activeKeys[e.key.toLowerCase()] = false;
    updateKeyboardTeleop();
  });

  function updateKeyboardTeleop() {
    let surge = 0.0, yaw = 0.0, heave = 0.0;
    const step = 0.25;

    if (activeKeys['w']) surge += step;
    if (activeKeys['s']) surge -= step;
    if (activeKeys['a']) yaw -= step;
    if (activeKeys['d']) yaw += step;
    if (activeKeys['i']) heave -= step; // Yüksel
    if (activeKeys['k']) heave += step; // Dal

    if (activeKeys[' ']) {
      surge = 0.0; yaw = 0.0; heave = 0.0;
    }

    if (surge !== 0 || yaw !== 0 || heave !== 0) {
      teleopActive = true;
      document.getElementById('teleop-status').textContent = `KLAVYE TELEOP: AKTİF (Surge=${surge.toFixed(2)}, Yaw=${yaw.toFixed(2)}, Heave=${heave.toFixed(2)})`;
      sendCommand('teleop', { surge, yaw, heave });
    } else if (teleopActive) {
      teleopActive = false;
      document.getElementById('teleop-status').textContent = 'KLAVYE TELEOP: DEVRE DIŞI';
      sendCommand('teleop_off');
    }
  }

  // ── 5. Konsol Log Yardımcısı ─────────────────────────────────────────────
  function logConsole(msg, type = 'info') {
    const timeStr = new Date().toLocaleTimeString();
    const div = document.createElement('div');
    div.className = `log-line ${type}`;
    div.textContent = `[${timeStr}] ${msg}`;
    consoleBox.appendChild(div);
    consoleBox.scrollTop = consoleBox.scrollHeight;
  }

  // Akışı Başlat
  pollTelemetry();
  logConsole('Yer istasyonu sistemi hazır. http://192.168.1.10:8000/', 'info');
});

/* ==========================================================================
   EGE ROV — Ground Control Station (GCS) Main Application Controller

   NOT (hata gecmisi): Bu dosyada tek bir yazim hatasi ('btn-minrov_back',
   dogrusu 'btn-minrov-back') null uzerinde addEventListener cagirdigi icin
   DOMContentLoaded isleyicisinin TAMAMI o satirda cokuyordu. Sonuc: PID
   paneli baglanmiyor, klavye teleop calismiyor ve pollTelemetry() hic
   baslamiyordu — yani sayfa tamamen olu goruntusu veriyordu.

   Bu tur tek noktadan cokmeyi bir daha yasamamak icin asagida $() ve on()
   yardimcilari var: eksik bir element artik konsola uyari yazar ve gerisi
   calismaya devam eder.
   ========================================================================== */

document.addEventListener('DOMContentLoaded', () => {

  // ── 0. Guvenli DOM yardimcilari ─────────────────────────────────────────
  const missing = [];

  function $(id) {
    const el = document.getElementById(id);
    if (!el) missing.push(id);
    return el;
  }

  /** Element yoksa sessizce atla — tek bir typo tum arayuzu oldurmesin. */
  function on(id, event, handler) {
    const el = $(id);
    if (el) el.addEventListener(event, handler);
    return el;
  }

  const hud = new HUDController('hud-canvas');
  let teleopActive = false;

  // DOM Elements
  const badgeConn = $('badge-conn');
  const badgeArm = $('badge-arm');
  const badgeEstop = $('badge-estop');
  const badgeMission = $('badge-mission');

  const btnArm = $('btn-arm');

  const valDepth = $('val-depth');
  const valTargetDepth = $('val-target-depth');
  const valHeading = $('val-heading');
  const valTargetHeading = $('val-target-heading');
  const valPitch = $('val-pitch');
  const valRoll = $('val-roll');
  const valPressure = $('val-pressure');
  const valTemp = $('val-temp');

  const arDepth = $('ar-depth');
  const arHeading = $('ar-heading');

  const consoleBox = $('console-box');

  // Thruster Bar Elements
  const thrusters = {
    V_FL: { fill: $('bar-vfl'), lbl: $('lbl-vfl') },
    V_FR: { fill: $('bar-vfr'), lbl: $('lbl-vfr') },
    V_RL: { fill: $('bar-vrl'), lbl: $('lbl-vrl') },
    V_RR: { fill: $('bar-vrr'), lbl: $('lbl-vrr') },
    H_L:  { fill: $('bar-hl'),  lbl: $('lbl-hl') },
    H_R:  { fill: $('bar-hr'),  lbl: $('lbl-hr') },
  };

  const num = (v) => (typeof v === 'number' && isFinite(v));

  // ── 1. Canlı Telemetri Döngüsü (20Hz Poll) ──────────────────────────────
  let connected = false;

  async function pollTelemetry() {
    try {
      const res = await fetch('/api/telemetry', { cache: 'no-store' });
      if (!res.ok) throw new Error('HTTP ' + res.status);
      const data = await res.json();

      updateUI(data);
      hud.update(data);
      pidMonitor.update(data);

      if (!connected) {
        connected = true;
        logConsole('Telemetri akışı kuruldu.', 'info');
        pidMonitor.loadGains();   // acilista kutulari cihazdaki degerle doldur
      }
      badgeConn.className = 'badge online';
      badgeConn.querySelector('.lbl').textContent = 'CANLI BAĞLANTI (20Hz)';
    } catch (e) {
      connected = false;
      badgeConn.className = 'badge';
      badgeConn.querySelector('.lbl').textContent = 'BAĞLANTI KESİLDİ';
      pidMonitor.setSync('offline', 'bağlantı yok');
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

    // Values — her alan ayri ayri korunuyor: gorev baslamadan once
    // telemetride sadece armed/estop olabilir, sayisal alanlar gelmez.
    if (num(data.depth)) {
      valDepth.innerHTML = `${data.depth.toFixed(2)} <small>m</small>`;
      arDepth.textContent = data.depth.toFixed(2);
    }
    if (num(data.target_depth)) {
      valTargetDepth.textContent = `Hedef: ${data.target_depth.toFixed(2)}m`;
    }
    if (num(data.heading)) {
      valHeading.innerHTML = `${data.heading.toFixed(1)} <small>°</small>`;
      arHeading.textContent = Math.round(data.heading).toString().padStart(3, '0');
    }
    if (num(data.target_heading)) {
      valTargetHeading.textContent = `Hedef: ${data.target_heading.toFixed(1)}°`;
    }
    if (num(data.pitch)) valPitch.innerHTML = `${data.pitch.toFixed(1)} <small>°</small>`;
    if (num(data.roll)) valRoll.textContent = data.roll.toFixed(1);
    if (num(data.pressure_mbar)) valPressure.innerHTML = `${Math.round(data.pressure_mbar)} <small>mbar</small>`;
    if (num(data.temp_c)) valTemp.textContent = data.temp_c.toFixed(1);

    // Thrusters
    if (data.thrusters) {
      for (const [key, val] of Object.entries(data.thrusters)) {
        const t = thrusters[key];
        if (!t || !t.fill || !num(val)) continue;
        const pct = Math.min(100, Math.abs(val) * 100);
        t.fill.style.width = pct + '%';
        t.lbl.textContent = `%${Math.round(val * 100)}`;
        t.fill.style.background = val < 0 ? '#ff0055' : '#00f3ff';
      }
    }
  }

  // ── 2. Komut Gönderme Yardımcısı ─────────────────────────────────────────
  async function sendCommand(cmd, payload = {}, quiet = false) {
    try {
      const res = await fetch('/api/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ cmd, ...payload })
      });
      const ret = await res.json();
      if (ret.ok) {
        if (!quiet) logConsole(ret.message || `${cmd} başarılı`, 'info');
      } else {
        logConsole(`Hata: ${ret.error}`, 'err');
      }
      return ret;
    } catch (e) {
      if (!quiet) logConsole(`Ağ hatası: ${e.message}`, 'err');
      return { ok: false, error: e.message };
    }
  }

  // ── 3. CANLI PID İZLEME & AYAR ───────────────────────────────────────────
  const pidMonitor = (() => {
    const sel = $('sel-pid-target');
    const canvas = $('pid-chart');
    const ctx = canvas ? canvas.getContext('2d') : null;
    const syncBadge = $('pid-sync');
    const cascadeRow = $('pid-cascade-row');
    const satLegend = $('lg-sat');
    const outErr = $('pid-err');
    const outOut = $('pid-out');
    const outExtra = $('pid-extra');
    const setpointInput = $('pid-setpoint');

    // gain alani -> input elemani
    const fields = {
      kp: $('pid-kp'), ki: $('pid-ki'), kd: $('pid-kd'),
      kp_pos: $('pid-kp-pos'), w_max: $('pid-wmax'), i_limit: $('pid-ilimit'),
    };

    const terms = {
      p: $('term-p'), i: $('term-i'), d: $('term-d'), out: $('term-out'),
    };

    // Her PID icin ayri gecmis tut: sekme degistirince grafik karismaz.
    const HISTORY = 160;
    const history = {};
    let editing = false;      // kullanici kutuya dokundu mu (canli yazma dursun)
    let lastGains = {};

    function current() { return sel ? sel.value : 'depth'; }

    function hist(name) {
      if (!history[name]) history[name] = { err: [], out: [] };
      return history[name];
    }

    function setSync(cls, text) {
      if (!syncBadge) return;
      syncBadge.className = 'pid-sync' + (cls ? ' ' + cls : '');
      syncBadge.textContent = text;
    }

    /** Cihazdaki GERCEK kazanclari oku ve kutulara yaz. */
    async function loadGains() {
      try {
        const r = await fetch('/api/pid', { cache: 'no-store' });
        const j = await r.json();
        if (!j.ok) return;
        applyGains(j.pid[current()]);
      } catch (e) { /* baglanti yoksa sessiz gec */ }
    }

    function applyGains(snap) {
      if (!snap || !snap.gains) return;
      lastGains = snap.gains;
      const isCascade = ('kp_pos' in snap.gains);
      if (cascadeRow) cascadeRow.style.display = isCascade ? 'grid' : 'none';

      if (editing) return;   // kullanici yazarken uzerine yazma
      for (const [k, el] of Object.entries(fields)) {
        if (!el) continue;
        const v = snap.gains[k];
        el.value = (v === undefined || v === null) ? '' : v;
        el.classList.remove('dirty');
      }
      setSync('', 'senkron');
    }

    /** Her telemetri karesinde cagrilir: grafik + terim cubuklari. */
    function update(data) {
      const name = current();
      const snap = data && data.pid ? data.pid[name] : null;
      if (!snap) return;

      if (!editing) applyGains(snap);
      else if (cascadeRow) cascadeRow.style.display = ('kp_pos' in (snap.gains || {})) ? 'grid' : 'none';

      const h = hist(name);
      h.err.push(snap.err || 0);
      h.out.push(snap.out || 0);
      if (h.err.length > HISTORY) { h.err.shift(); h.out.shift(); }

      // P/I/D katki cubuklari — cikis limitine gore olceklenir
      const limit = Math.max(0.001, Math.abs(lastGains.out_limit || 1.0));
      for (const key of ['p', 'i', 'd', 'out']) {
        const row = terms[key];
        if (!row) continue;
        const v = snap[key] || 0;
        const frac = Math.min(1, Math.abs(v) / limit) * 50;  // yarim genislik %
        row.querySelector('.pos').style.width = v > 0 ? frac + '%' : '0%';
        row.querySelector('.neg').style.width = v < 0 ? frac + '%' : '0%';
        row.querySelector('.tv').textContent = v.toFixed(3);
        row.classList.toggle('saturated', key === 'out' && !!snap.sat);
      }

      if (outErr) outErr.textContent = (snap.err || 0).toFixed(3);
      if (outOut) outOut.textContent = (snap.out || 0).toFixed(3);
      if (satLegend) satLegend.classList.toggle('on', !!snap.sat);

      if (outExtra) {
        outExtra.textContent = (snap.w_target !== undefined)
          ? `ω hedef: ${snap.w_target.toFixed(1)}°/s · ölçülen: ${snap.w_meas.toFixed(1)}°/s · mod: ${snap.mode || '-'}`
          : '';
      }

      draw(h);
    }

    /** Basit strip-chart. Harici kutuphane YOK — Jetson internetsiz calisir. */
    function draw(h) {
      if (!ctx) return;
      const W = canvas.width, H = canvas.height;
      ctx.clearRect(0, 0, W, H);

      // izgara
      ctx.strokeStyle = 'rgba(255,255,255,0.07)';
      ctx.lineWidth = 1;
      for (let i = 1; i < 4; i++) {
        const y = (H / 4) * i;
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
      }

      const series = [
        { data: h.err, color: '#ffcc00' },
        { data: h.out, color: '#00f3ff' },
      ];

      // Iki seri de kendi olceginde cizilir; yoksa kucuk cikis duz cizgi olur.
      for (const s of series) {
        if (s.data.length < 2) continue;
        let max = 0;
        for (const v of s.data) max = Math.max(max, Math.abs(v));
        max = max < 1e-6 ? 1 : max * 1.15;

        ctx.strokeStyle = s.color;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        s.data.forEach((v, i) => {
          const x = (i / (HISTORY - 1)) * W;
          const y = H / 2 - (v / max) * (H / 2 - 4);
          i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
        });
        ctx.stroke();
      }

      // sifir cizgisi
      ctx.strokeStyle = 'rgba(255,255,255,0.25)';
      ctx.beginPath(); ctx.moveTo(0, H / 2); ctx.lineTo(W, H / 2); ctx.stroke();
    }

    // Kullanici kutuya dokundugu an canli yazmayi durdur
    for (const el of Object.values(fields)) {
      if (!el) continue;
      el.addEventListener('input', () => {
        editing = true;
        el.classList.add('dirty');
        setSync('dirty', 'gönderilmedi');
      });
      // Enter = gonder
      el.addEventListener('keydown', (e) => {
        if (e.key === 'Enter') push();
      });
    }

    // Kaskad alanlari (kp_pos/w_max/i_limit) sadece heading icin gonderilir.
    // NOT: burada offsetParent ile "gorunur mu" bakmak YANLIS — duzen
    // hesaplanmamissa (ya da panel scroll disindaysa) null doner ve kazanclar
    // sessizce gonderilmez. Alanin hangi PID'e ait oldugunu acikca biliyoruz.
    const CASCADE_FIELDS = new Set(['kp_pos', 'w_max', 'i_limit']);

    async function push() {
      const name = current();
      const isCascade = ('kp_pos' in (lastGains || {}));
      const payload = { pid_name: name };
      let any = false;
      for (const [k, el] of Object.entries(fields)) {
        if (!el) continue;
        if (CASCADE_FIELDS.has(k) && !isCascade) continue;  // bu PID'de yok
        const v = parseFloat(el.value);
        if (isFinite(v)) { payload[k] = v; any = true; }
      }
      if (!any) { logConsole('Geçerli kazanç girilmedi', 'warn'); return; }

      const ret = await sendCommand('set_pid', payload);
      if (ret.ok) {
        editing = false;
        Object.values(fields).forEach(el => el && el.classList.remove('dirty'));
        setSync('', 'senkron');
      }
    }

    if (sel) {
      sel.addEventListener('change', () => {
        editing = false;
        loadGains();
        // Hedef kutusunu secilen eksene gore anlamlandir
        if (setpointInput) {
          setpointInput.value = current() === 'heading' ? '90' : '1.5';
        }
      });
    }

    on('btn-update-pid', 'click', push);

    on('btn-reset-pid', 'click', () => {
      sendCommand('reset_pid', { pid_name: current() });
      const h = hist(current());
      h.err.length = 0; h.out.length = 0;
    });

    on('btn-set-target', 'click', () => {
      const v = parseFloat(setpointInput ? setpointInput.value : NaN);
      if (!isFinite(v)) { logConsole('Geçersiz hedef', 'warn'); return; }
      const key = current() === 'heading' ? 'heading' : 'depth';
      sendCommand('set_target', { [key]: v });
    });

    return { update, loadGains, setSync };
  })();

  // ── 4. Buton Event Bağlantıları ──────────────────────────────────────────
  on('btn-arm', 'click', () => {
    sendCommand(btnArm.classList.contains('armed') ? 'disarm' : 'arm');
  });

  on('btn-abort', 'click', () => sendCommand('abort'));

  on('btn-winch-deploy', 'click', () => sendCommand('winch_deploy'));
  on('btn-winch-retract', 'click', () => sendCommand('winch_retract'));
  on('btn-minrov-back', 'click', () => sendCommand('minrov_back'));

  // Görev Butonları
  on('btn-m-video', 'click', () => logConsole('Video gösterimi görevi seçildi', 'info'));
  on('btn-m-line', 'click', () => logConsole('Görev 1 (Hat Takibi) seçildi', 'info'));
  on('btn-m-nav', 'click', () => logConsole('Görev 2 (Navigasyon) seçildi', 'info'));

  // AR Overlay Toggle
  on('chk-ar-overlay', 'change', (e) => {
    const ov = document.getElementById('ar-overlay');
    if (ov) ov.style.display = e.target.checked ? 'block' : 'none';
  });

  // ── 5. Klavye Teleop Sürüş Mantığı ───────────────────────────────────────
  const activeKeys = {};
  const teleopStatus = $('teleop-status');

  window.addEventListener('keydown', (e) => {
    if (['INPUT', 'SELECT', 'TEXTAREA'].includes(e.target.tagName)) return;
    activeKeys[e.key.toLowerCase()] = true;
    updateKeyboardTeleop();
  });

  window.addEventListener('keyup', (e) => {
    activeKeys[e.key.toLowerCase()] = false;
    updateKeyboardTeleop();
  });

  // Sekme arkaya alinirsa keyup gelmez ve motor komutu ASILI KALIR.
  window.addEventListener('blur', () => {
    for (const k in activeKeys) activeKeys[k] = false;
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

    if (activeKeys[' ']) { surge = 0.0; yaw = 0.0; heave = 0.0; }

    if (surge !== 0 || yaw !== 0 || heave !== 0) {
      teleopActive = true;
      if (teleopStatus) {
        teleopStatus.textContent =
          `KLAVYE TELEOP: AKTİF (Surge=${surge.toFixed(2)}, Yaw=${yaw.toFixed(2)}, Heave=${heave.toFixed(2)})`;
      }
      sendCommand('teleop', { surge, yaw, heave }, true);
    } else if (teleopActive) {
      teleopActive = false;
      if (teleopStatus) teleopStatus.textContent = 'KLAVYE TELEOP: DEVRE DIŞI';
      sendCommand('teleop_off', {}, true);
    }
  }

  // ── 6. Konsol Log Yardımcısı ─────────────────────────────────────────────
  function logConsole(msg, type = 'info') {
    if (!consoleBox) return;
    const div = document.createElement('div');
    div.className = `log-line ${type}`;
    div.textContent = `[${new Date().toLocaleTimeString()}] ${msg}`;
    consoleBox.appendChild(div);
    while (consoleBox.childElementCount > 200) consoleBox.removeChild(consoleBox.firstChild);
    consoleBox.scrollTop = consoleBox.scrollHeight;
  }

  // ── 7. Akışı Başlat ──────────────────────────────────────────────────────
  if (missing.length) {
    console.warn('[GCS] HTML\'de bulunamayan element id\'leri:', missing);
    logConsole(`UYARI: ${missing.length} arayüz elemanı bulunamadı (konsola bak)`, 'warn');
  }

  pollTelemetry();
  logConsole('Yer istasyonu sistemi hazır.', 'info');
});

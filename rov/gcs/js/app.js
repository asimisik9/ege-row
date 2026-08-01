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
  const badgeMode = $('badge-mode');
  const badgeHealth = $('badge-health');
  const badgeLoop = $('badge-loop');

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
      modePanel.update(data);
      targetPanel.update(data);
      healthPanel.update(data);
      stepPanel.update(data);

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
    if (num(data.roll)) valRoll.innerHTML = `${data.roll.toFixed(1)} <small>°</small>`;
    if (num(data.pressure_mbar)) valPressure.innerHTML = `${Math.round(data.pressure_mbar)} <small>mbar</small>`;
    if (num(data.temp_c)) valTemp.textContent = data.temp_c.toFixed(1);

    // Dikey hiz: dalis/cikis hizini gormek havuzda cok ise yarar
    if (num(data.depth_rate)) {
      const dr = data.depth_rate;
      set('val-depth-rate', `${dr >= 0 ? '+' : ''}${dr.toFixed(2)} <small>m/s</small>`, true);
    }
    if (num(data.depth_error)) set('val-depth-err', data.depth_error.toFixed(3));
    if (num(data.yaw_rate)) set('val-yawrate', `${data.yaw_rate.toFixed(1)} <small>°/s</small>`, true);
    if (data.heading_mode) set('val-hmode', data.heading_mode);

    // Gorev 2 sensorleri — sadece gercekten veri varsa goster
    const navCards = document.getElementById('nav-cards');
    if (navCards) {
      const hasNav = (data.gps !== undefined) || (data.sonar_mm !== undefined);
      navCards.style.display = hasNav ? 'grid' : 'none';
      if (data.gps) {
        set('val-gps', data.gps.fix ? 'FIX VAR' : 'FIX YOK');
        set('val-gps-sub', data.gps.fix
          ? `${data.gps.lat.toFixed(5)}, ${data.gps.lon.toFixed(5)}`
          : 'uydu bekleniyor');
      }
      if (data.sonar_mm !== undefined) {
        set('val-sonar', data.sonar_mm == null ? '— <small>mm</small>'
                                               : `${data.sonar_mm} <small>mm</small>`, true);
      }
    }

    // Thrusters — normalize komut + gercekten ESC'ye giden PWM darbesi
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
    if (data.thruster_us) {
      for (const [key, us] of Object.entries(data.thruster_us)) {
        const el = document.getElementById('us-' + key.toLowerCase().replace('_', ''));
        if (el) el.textContent = (us == null) ? '—' : `${us} µs`;
      }
    }
  }

  /** Kisa yardimci: id'li elemanin metnini/HTML'ini yaz (yoksa sessiz gec). */
  function set(id, value, html) {
    const el = document.getElementById(id);
    if (!el) return;
    if (html) el.innerHTML = value; else el.textContent = value;
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

  // ── 3a. SEKMELER ─────────────────────────────────────────────────────────
  document.querySelectorAll('.tab').forEach(tab => {
    tab.addEventListener('click', () => {
      document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
      document.querySelectorAll('.tab-page').forEach(p => p.classList.remove('active'));
      tab.classList.add('active');
      const page = document.getElementById(tab.dataset.tab);
      if (page) page.classList.add('active');
    });
  });

  // ── 3b. KONTROL MODU ─────────────────────────────────────────────────────
  //
  // Hedef belirlemenin calismamasinin sebebi buydu: gorevin step() metodu
  // 50 Hz'de kendi hedefini yaziyordu. Mod, hedefin SAHIBINI belirler.
  const modePanel = (() => {
    const hint = $('mode-hint');
    const owner = $('mode-owner');
    const HINTS = {
      AUTO:   ['hedef sahibi: görev',
               'AUTO: hedefleri görev yönetir. Kendi hedefini vermek için <b>HOLD</b>\'a geç.'],
      HOLD:   ['hedef sahibi: sen',
               'HOLD: görev duraklatıldı. Verdiğin derinlik/yön hedefini PID tutar.'],
      HOVER:  ['derinlik PID kapalı',
               'HOVER: sabit dikey gaz. Araç ne çıkıp ne iniyorsa o değer <b>FF_HOVER</b>\'dır.'],
      RATE:   ['dönüş hızı hedefi',
               'RATE: sabit dönüş hızı. Daire çapı = 2·ileri hız / dönüş hızı.'],
      TELEOP: ['doğrudan sürüş',
               'TELEOP: WASD/IJKL eksenleri doğrudan motorlara gidiyor. PID devrede değil.'],
    };
    let currentMode = null;

    document.querySelectorAll('.btn-mode').forEach(btn => {
      btn.addEventListener('click', () => sendCommand('set_mode', { mode: btn.dataset.mode }));
    });

    function update(data) {
      const m = data.mode;
      if (!m || m === currentMode) return;
      currentMode = m;
      document.querySelectorAll('.btn-mode').forEach(b =>
        b.classList.toggle('active', b.dataset.mode === m));
      if (badgeMode) badgeMode.textContent = 'MOD: ' + m;
      const h = HINTS[m];
      if (h) {
        if (owner) owner.textContent = h[0];
        if (hint) hint.innerHTML = h[1];
      }
    }
    return { update, get: () => currentMode };
  })();

  // ── 3c. HEDEF PANELİ ─────────────────────────────────────────────────────
  const targetPanel = (() => {
    const sldSurge = $('sld-surge'), sldHover = $('sld-hover');
    const sldThrust = $('sld-thrust'), sldSlew = $('sld-slew');
    const selHmode = $('sel-heading-mode');
    let userTouching = null;    // kaydirak surukleniyorsa telemetriyle ezme

    /** Kaydirak: surukleme bitince komut gonder (her pikselde istek atma). */
    function slider(el, svId, cmd, key, fmt) {
      if (!el) return;
      const sv = document.getElementById(svId);
      const show = () => { if (sv) sv.textContent = fmt(parseFloat(el.value)); };
      el.addEventListener('input', () => { userTouching = el; show(); });
      el.addEventListener('change', () => {
        sendCommand(cmd, { [key]: parseFloat(el.value) });
        setTimeout(() => { if (userTouching === el) userTouching = null; }, 400);
      });
      show();
    }

    slider(sldSurge, 'sv-surge', 'set_surge', 'surge', v => v.toFixed(2));
    slider(sldHover, 'sv-hover', 'set_hover', 'hover', v => v.toFixed(2));
    slider(sldThrust, 'sv-thrust', 'set_limits', 'thrust_limit', v => v.toFixed(2));
    slider(sldSlew, 'sv-slew', 'set_limits', 'slew_rate', v => v.toFixed(1));

    on('btn-tgt-depth', 'click', () => {
      const v = parseFloat(($('tgt-depth') || {}).value);
      if (!isFinite(v)) return logConsole('Geçersiz derinlik', 'warn');
      sendCommand('set_target', { depth: v });
    });
    on('btn-tgt-heading', 'click', () => {
      const v = parseFloat(($('tgt-heading') || {}).value);
      if (!isFinite(v)) return logConsole('Geçersiz yön', 'warn');
      sendCommand('set_target', { heading: v });
    });
    on('btn-tgt-roll', 'click', () => {
      const v = parseFloat(($('tgt-roll') || {}).value);
      if (!isFinite(v)) return logConsole('Geçersiz yatış (roll)', 'warn');
      sendCommand('set_target', { roll: v });
    });

    // Bagil hedefler (+10 cm, -90 derece ...) — havuzda en cok kullanilan
    document.querySelectorAll('[data-depth]').forEach(b =>
      b.addEventListener('click', () =>
        sendCommand('set_target', { depth_rel: parseFloat(b.dataset.depth) })));
    document.querySelectorAll('[data-hdg]').forEach(b =>
      b.addEventListener('click', () =>
        sendCommand('set_target', { heading_rel: parseFloat(b.dataset.hdg) })));
    document.querySelectorAll('[data-roll]').forEach(b =>
      b.addEventListener('click', () =>
        sendCommand('set_target', { roll_rel: parseFloat(b.dataset.roll) })));

    on('btn-clear-depth', 'click', () => sendCommand('clear_target', { axis: 'depth' }));
    on('btn-clear-heading', 'click', () => sendCommand('clear_target', { axis: 'heading' }));
    on('btn-clear-roll', 'click', () => sendCommand('clear_target', { axis: 'roll' }));

    on('btn-tgt-rate', 'click', () => {
      const v = parseFloat(($('tgt-rate') || {}).value);
      if (!isFinite(v)) return logConsole('Geçersiz dönüş hızı', 'warn');
      sendCommand('set_rate', { rate: v });
    });
    on('btn-tgt-ff', 'click', () => {
      const v = parseFloat(($('tgt-ff') || {}).value);
      if (!isFinite(v)) return logConsole('Geçersiz FF', 'warn');
      sendCommand('set_ff', { ff: v });
    });
    on('btn-zero-depth', 'click', () => sendCommand('zero_depth'));

    if (selHmode) {
      selHmode.addEventListener('change', () =>
        sendCommand('heading_mode', { mode: selHmode.value }));
    }

    function update(data) {
      set('live-depth', num(data.depth) ? data.depth.toFixed(2) : '—');
      set('live-heading', num(data.heading) ? data.heading.toFixed(1) : '—');
      set('live-roll', num(data.roll) ? data.roll.toFixed(1) : '—');
      set('live-tgt-depth', data.depth_locked ? data.target_depth.toFixed(2) : '—');
      set('live-tgt-heading', data.heading_locked ? data.target_heading.toFixed(1) : '—');
      set('live-tgt-roll', data.roll_locked ? data.target_roll.toFixed(1) : '—');

      if (selHmode && data.heading_mode && selHmode.value !== data.heading_mode
          && document.activeElement !== selHmode) {
        selHmode.value = data.heading_mode;
      }
      // Cihazdaki gercek degerleri kaydiraklara yansit (kullanici tutmuyorsa)
      syncSlider(sldSurge, 'sv-surge', data.surge, v => v.toFixed(2));
      syncSlider(sldHover, 'sv-hover', data.hover_cmd, v => v.toFixed(2));
      if (data.limits) {
        syncSlider(sldThrust, 'sv-thrust', data.limits.thrust_limit, v => v.toFixed(2));
        syncSlider(sldSlew, 'sv-slew', data.limits.slew_rate, v => v.toFixed(1));
      }
    }

    function syncSlider(el, svId, value, fmt) {
      if (!el || !num(value) || userTouching === el) return;
      if (Math.abs(parseFloat(el.value) - value) < 1e-6) return;
      el.value = value;
      const sv = document.getElementById(svId);
      if (sv) sv.textContent = fmt(value);
    }

    return { update };
  })();

  // ── 3d. SAĞLIK PANELİ ────────────────────────────────────────────────────
  //
  // Watchdog gorevi iptal ediyordu ama operator NEDEN iptal oldugunu
  // goremiyordu. Bu panel sensor thread'lerinin gercek durumunu gosterir.
  const healthPanel = (() => {
    function row(id, text, state, pct) {
      const el = document.getElementById(id);
      if (!el) return;
      el.textContent = text;
      const parent = el.closest('.hrow');
      if (parent) {
        parent.classList.toggle('bad', state === 'bad');
        parent.classList.toggle('warn', state === 'warn');
        const bar = parent.querySelector('i');
        if (bar && pct !== undefined) bar.style.setProperty('--pct', Math.min(100, pct) + '%');
      }
    }

    function update(data) {
      const h = data.health;
      if (h) {
        row('h-imu-hz', `${h.imu_hz.toFixed(0)} Hz`,
            h.imu_hz < 40 ? 'bad' : (h.imu_hz < 80 ? 'warn' : ''), h.imu_hz);
        row('h-imu-age', `${(h.imu_age * 1000).toFixed(0)} ms`,
            h.imu_age > h.stale_s ? 'bad' : '');
        row('h-imu-err', h.imu_errors, h.imu_errors > 0 ? 'warn' : '');
        row('h-dep-hz', `${h.depth_hz.toFixed(0)} Hz`,
            h.depth_hz < 8 ? 'bad' : (h.depth_hz < 15 ? 'warn' : ''), h.depth_hz * 5);
        row('h-dep-age', `${(h.depth_age * 1000).toFixed(0)} ms`,
            h.depth_age > h.stale_s ? 'bad' : '');
        row('h-dep-err', h.depth_errors, h.depth_errors > 0 ? 'warn' : '');

        const wd = document.getElementById('watchdog');
        if (wd) {
          wd.textContent = h.healthy
            ? `WATCHDOG: VERİ TAZE (eşik ${h.stale_s}s)`
            : 'WATCHDOG: VERİ BAYAT — GÖREV İPTAL EDİLİR!';
          wd.className = 'watchdog' + (h.healthy ? '' : ' bad');
        }
        if (badgeHealth) {
          badgeHealth.className = 'badge' + (h.healthy ? ' online' : '');
          badgeHealth.querySelector('.lbl').textContent =
            h.healthy ? 'SENSÖR TAZE' : 'SENSÖR BAYAT!';
        }
      }

      const l = data.loop;
      if (l) {
        const bad = l.hz < 30, warn = l.warn_hz && l.hz < l.warn_hz;
        row('l-hz', `${l.hz.toFixed(1)} Hz`, bad ? 'bad' : (warn ? 'warn' : ''),
            (l.hz / l.target_hz) * 100);
        row('l-target', `${l.target_hz.toFixed(0)} Hz`, '');
        row('l-worst', `${l.worst_dt_ms.toFixed(0)} ms`, l.worst_dt_ms > 100 ? 'warn' : '');
        row('l-stalls', l.stalls, l.stalls > 0 ? 'bad' : '');
        row('l-count', l.count, '');
        const v = document.getElementById('loop-verdict');
        if (v) {
          v.textContent = bad
            ? `H1 KABUL KRİTERİ KALDI (${l.hz.toFixed(1)} Hz < 30 Hz)`
            : `H1 KABUL KRİTERİ GEÇTİ (${l.hz.toFixed(1)} Hz ≥ 30 Hz)`;
          v.className = 'watchdog' + (bad ? ' bad' : '');
        }
        if (badgeLoop) {
          badgeLoop.className = 'badge' + (bad ? '' : ' online');
          badgeLoop.querySelector('.lbl').textContent = `${l.hz.toFixed(0)} Hz`;
        }
      }
      if (num(data.uptime_s)) {
        const s = Math.floor(data.uptime_s);
        row('l-uptime', `${Math.floor(s / 60)}dk ${s % 60}sn`, '');
      }

      if (data.gyro) row('h-gyro', data.gyro.map(g => g.toFixed(1)).join(' / '), '');
      if (num(data.surface_ref_mbar)) row('h-surface', `${data.surface_ref_mbar.toFixed(2)} mbar`, '');
      if (num(data.pressure_mbar)) row('h-press', `${data.pressure_mbar.toFixed(1)} mbar`, '');
      if (num(data.temp_c)) row('h-temp', `${data.temp_c.toFixed(1)} °C`, '');

      // Görev iç durumu
      const mi = document.getElementById('mission-info');
      if (mi && data.mission_info) {
        mi.innerHTML = Object.entries(data.mission_info)
          .map(([k, v]) => `<div class="hrow"><span>${k}</span><b>${v}</b><i></i></div>`)
          .join('');
      }
    }
    return { update };
  })();

  // ── 3e. ADIM CEVABI TESTİ & ANALİZ ───────────────────────────────────────
  const stepPanel = (() => {
    const canvas = $('step-chart');
    const ctx = canvas ? canvas.getContext('2d') : null;
    const recDot = $('rec-dot');
    let series = [];
    let lastFetch = 0;

    on('btn-analyze', 'click', async () => {
      const r = await sendCommand('analyze', {}, true);
      const box = document.getElementById('analysis');
      if (!r.ok) { logConsole(r.error || 'Analiz başarısız', 'warn'); return; }
      if (box) box.style.display = 'flex';

      const u = r.unit || '';
      setAn('an-asim', `${r.asim_pct.toFixed(1)} % (${r.asim_abs} ${u})`,
            r.asim_pct < 10 ? 'good' : (r.asim_pct > 25 ? 'bad' : ''));
      setAn('an-yerlesme', r.yerlesme_s === null ? 'oluşmadı' : `${r.yerlesme_s} s`,
            r.yerlesme_s === null ? 'bad' : (r.yerlesme_s < 6 ? 'good' : ''));
      setAn('an-kalici', `${r.kalici} ${u}`, Math.abs(r.kalici) < 0.05 ? 'good' : 'bad');
      setAn('an-rms', `${r.rms} ${u}`, '');

      const ul = document.getElementById('an-advice');
      if (ul) ul.innerHTML = (r.oneri || []).map(x => `<li>${x}</li>`).join('');
      logConsole(`Analiz: aşım %${r.asim_pct.toFixed(1)}, yerleşme ` +
                 `${r.yerlesme_s === null ? '—' : r.yerlesme_s + 's'}, ` +
                 `kalıcı ${r.kalici}${u}`, 'info');
    });

    function setAn(id, text, cls) {
      const el = document.getElementById(id);
      if (!el) return;
      el.textContent = text;
      el.parentElement.className = 'an' + (cls ? ' ' + cls : '');
    }

    async function update(data) {
      const rec = !!data.step_test;
      if (recDot) recDot.classList.toggle('on', rec);

      // Egriyi 4 Hz'de cek — 20 Hz telemetriyle birlikte cekmeye gerek yok
      const now = Date.now();
      if (now - lastFetch > 250) {
        lastFetch = now;
        try {
          const r = await fetch('/api/step', { cache: 'no-store' });
          const j = await r.json();
          series = j.series || [];
        } catch (e) { /* yoksay */ }
      }
      draw();
    }

    function draw() {
      if (!ctx || !canvas) return;
      const W = canvas.width, H = canvas.height;
      ctx.clearRect(0, 0, W, H);
      ctx.strokeStyle = 'rgba(255,255,255,0.07)';
      ctx.lineWidth = 1;
      for (let i = 1; i < 4; i++) {
        const y = (H / 4) * i;
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
      }
      if (series.length < 2) {
        ctx.fillStyle = 'rgba(255,255,255,0.25)';
        ctx.font = '11px monospace';
        ctx.fillText('adım testi yok — bir hedef ver', 12, H / 2);
        return;
      }

      // Olculen ve hedef AYNI olcekte cizilir; yoksa asim gozle gorulmez.
      let lo = Infinity, hi = -Infinity;
      for (const [, v, tg] of series) {
        lo = Math.min(lo, v, tg); hi = Math.max(hi, v, tg);
      }
      const pad = (hi - lo) * 0.15 || 0.5;
      lo -= pad; hi += pad;
      const tMax = series[series.length - 1][0] || 1;
      const X = t => (t / tMax) * (W - 2) + 1;
      const Y = v => H - ((v - lo) / (hi - lo)) * (H - 6) - 3;

      // hedef (yesil, kesik)
      ctx.strokeStyle = '#00ff66';
      ctx.setLineDash([4, 3]);
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      series.forEach(([t, , tg], i) => i ? ctx.lineTo(X(t), Y(tg)) : ctx.moveTo(X(t), Y(tg)));
      ctx.stroke();
      ctx.setLineDash([]);

      // olculen (cyan)
      ctx.strokeStyle = '#00f3ff';
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      series.forEach(([t, v], i) => i ? ctx.lineTo(X(t), Y(v)) : ctx.moveTo(X(t), Y(v)));
      ctx.stroke();

      ctx.fillStyle = 'rgba(255,255,255,0.35)';
      ctx.font = '9px monospace';
      ctx.fillText(`${tMax.toFixed(1)}s`, W - 30, H - 3);
      ctx.fillText(hi.toFixed(2), 3, 10);
      ctx.fillText(lo.toFixed(2), 3, H - 3);
    }
    return { update };
  })();

  // ── 3f. GÖREV BAŞLAT / DURDUR ────────────────────────────────────────────
  document.querySelectorAll('[data-mission]').forEach(b => {
    b.addEventListener('click', () =>
      sendCommand('mission_start', { mission: b.dataset.mission }));
  });
  on('btn-mission-stop', 'click', () => sendCommand('mission_stop'));

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

    // Adim girdisi ver: hedefi degistirir VE kaydi baslatir.
    // Not: AUTO modunda sunucu bunu reddeder (gorev hedefi eziyor) — hata
    // mesaji konsola duser ve kullaniciya HOLD'a gecmesi soylenir.
    on('btn-set-target', 'click', async () => {
      const v = parseFloat(setpointInput ? setpointInput.value : NaN);
      if (!isFinite(v)) { logConsole('Geçersiz hedef', 'warn'); return; }
      const name = current();
      if (name === 'roll' || name === 'pitch') {
        await sendCommand('step_start', { kind: name });
        logConsole(`${name.toUpperCase()} adım kaydı başladı. Aracı elinizle bozup bırakın (0'a dönüş test edilir).`, 'info');
        return;
      }
      const key = name === 'heading' ? 'heading' : 'depth';
      await sendCommand('set_target', { [key]: v });
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

  // NOT: Görev butonları artık gerçekten görev başlatıyor (bölüm 3f,
  // data-mission). Eskiden sadece konsola yazı yazıyorlardı.

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

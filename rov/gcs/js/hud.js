/* ==========================================================================
   EGE ROV — HTML5 Canvas Primary Flight Display (HUD / PFD) Engine
   ========================================================================== */

class HUDController {
  constructor(canvasId) {
    this.canvas = document.getElementById(canvasId);
    if (!this.canvas) return;
    this.ctx = this.canvas.getContext('2d');
    this.w = this.canvas.width;
    this.h = this.canvas.height;

    this.pitch = 0.0;
    this.roll = 0.0;
    this.heading = 0.0;
    this.targetHeading = 0.0;
    this.depth = 0.0;
    this.targetDepth = 0.0;
  }

  update(telemetry) {
    if (!telemetry) return;
    this.pitch = telemetry.pitch || 0.0;
    this.roll = telemetry.roll || 0.0;
    this.heading = telemetry.heading || 0.0;
    this.targetHeading = telemetry.target_heading || 0.0;
    this.depth = telemetry.depth || 0.0;
    this.targetDepth = telemetry.target_depth || 0.0;
    this.render();
  }

  render() {
    const ctx = this.ctx;
    const w = this.w;
    const h = this.h;
    const cx = w / 2;
    const cy = h / 2;

    ctx.clearRect(0, 0, w, h);

    // ── 1. Sky & Ground Horizon (Artificial Horizon) ───────────────────────
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate((this.roll * Math.PI) / 180);

    const pitchPxPerDeg = 4;
    const pitchOffset = this.pitch * pitchPxPerDeg;

    // Background Clipping Arc (Circle Mask)
    const radius = Math.min(w, h) * 0.42;
    ctx.beginPath();
    ctx.arc(0, 0, radius, 0, Math.PI * 2);
    ctx.clip();

    // Sky (Blue)
    ctx.fillStyle = '#0a2342';
    ctx.fillRect(-w, -h * 2 + pitchOffset, w * 2, h * 2);

    // Ground (Brown/Dark)
    ctx.fillStyle = '#261705';
    ctx.fillRect(-w, pitchOffset, w * 2, h * 2);

    // Horizon Line (White/Cyan)
    ctx.strokeStyle = '#00f3ff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(-w, pitchOffset);
    ctx.lineTo(w, pitchOffset);
    ctx.stroke();

    // Pitch Ladder Lines (-30° to +30°)
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 1;
    ctx.fillStyle = '#ffffff';
    ctx.font = '10px monospace';
    ctx.textAlign = 'center';

    for (let deg = -30; deg <= 30; deg += 10) {
      if (deg === 0) continue;
      const y = pitchOffset - deg * pitchPxPerDeg;
      const lineW = deg % 20 === 0 ? 40 : 20;

      ctx.beginPath();
      ctx.moveTo(-lineW, y);
      ctx.lineTo(lineW, y);
      ctx.stroke();

      ctx.fillText(deg.toString(), lineW + 12, y + 3);
    }

    ctx.restore();

    // ── 2. Fixed Aircraft Symbol (Crosshair) ──────────────────────────────
    ctx.save();
    ctx.strokeStyle = '#ffcc00';
    ctx.lineWidth = 3;

    // Wing Left
    ctx.beginPath();
    ctx.moveTo(cx - 50, cy);
    ctx.lineTo(cx - 20, cy);
    ctx.lineTo(cx - 20, cy + 8);
    ctx.stroke();

    // Wing Right
    ctx.beginPath();
    ctx.moveTo(cx + 50, cy);
    ctx.lineTo(cx + 20, cy);
    ctx.lineTo(cx + 20, cy + 8);
    ctx.stroke();

    // Center dot
    ctx.fillStyle = '#ffcc00';
    ctx.beginPath();
    ctx.arc(cx, cy, 3, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();

    // ── 3. Compass Tape (Top) ──────────────────────────────────────────────
    this.renderCompassTape(ctx, cx, 30);

    // ── 4. Depth Ladder (Right Edge) ───────────────────────────────────────
    this.renderDepthLadder(ctx, w - 25, cy);
  }

  renderCompassTape(ctx, cx, y) {
    const tapeW = 240;
    const tapeH = 24;

    ctx.save();
    ctx.fillStyle = 'rgba(0, 0, 0, 0.6)';
    ctx.fillRect(cx - tapeW / 2, y - tapeH / 2, tapeW, tapeH);
    ctx.strokeStyle = 'rgba(0, 243, 255, 0.4)';
    ctx.strokeRect(cx - tapeW / 2, y - tapeH / 2, tapeW, tapeH);

    ctx.clip();

    const pxPerDeg = 2.5;
    const hdg = this.heading;

    ctx.fillStyle = '#ffffff';
    ctx.font = '10px monospace';
    ctx.textAlign = 'center';

    for (let deg = -180; deg <= 540; deg += 15) {
      const normDeg = (deg + 360) % 360;
      const x = cx + (deg - hdg) * pxPerDeg;

      if (x >= cx - tapeW / 2 && x <= cx + tapeW / 2) {
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.6)';
        ctx.beginPath();
        ctx.moveTo(x, y - 4);
        ctx.lineTo(x, y + 4);
        ctx.stroke();

        let label = normDeg.toString();
        if (normDeg === 0) label = 'N';
        else if (normDeg === 90) label = 'E';
        else if (normDeg === 180) label = 'S';
        else if (normDeg === 270) label = 'W';

        ctx.fillText(label, x, y + 10);
      }
    }

    // Target Heading Bug (Magenta)
    const tgtX = cx + (this.targetHeading - hdg) * pxPerDeg;
    ctx.fillStyle = '#ff00ff';
    ctx.beginPath();
    ctx.moveTo(tgtX - 5, y - 10);
    ctx.lineTo(tgtX + 5, y - 10);
    ctx.lineTo(tgtX, y - 3);
    ctx.closePath();
    ctx.fill();

    // Center Indicator Triangle (Yellow)
    ctx.restore();
    ctx.fillStyle = '#ffcc00';
    ctx.beginPath();
    ctx.moveTo(cx - 6, y - tapeH / 2);
    ctx.lineTo(cx + 6, y - tapeH / 2);
    ctx.lineTo(cx, y - tapeH / 2 + 6);
    ctx.closePath();
    ctx.fill();
  }

  renderDepthLadder(ctx, x, cy) {
    const h = 180;
    ctx.save();
    ctx.fillStyle = 'rgba(0, 0, 0, 0.6)';
    ctx.fillRect(x - 20, cy - h / 2, 40, h);
    ctx.strokeStyle = 'rgba(0, 243, 255, 0.4)';
    ctx.strokeRect(x - 20, cy - h / 2, 40, h);

    const pxPerM = 40;
    const curDepth = this.depth;

    ctx.fillStyle = '#00f3ff';
    ctx.font = '10px monospace';
    ctx.textAlign = 'right';

    for (let d = 0; d <= 10; d += 0.5) {
      const y = cy + (d - curDepth) * pxPerM;
      if (y >= cy - h / 2 && y <= cy + h / 2) {
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.5)';
        ctx.beginPath();
        ctx.moveTo(x - 15, y);
        ctx.lineTo(x - 5, y);
        ctx.stroke();

        if (d % 1 === 0) {
          ctx.fillText(d.toFixed(1), x - 18, y + 3);
        }
      }
    }

    // Target Depth Marker
    const tgtY = cy + (this.targetDepth - curDepth) * pxPerM;
    if (tgtY >= cy - h / 2 && tgtY <= cy + h / 2) {
      ctx.fillStyle = '#ff00ff';
      ctx.beginPath();
      ctx.moveTo(x + 5, tgtY - 4);
      ctx.lineTo(x - 5, tgtY);
      ctx.lineTo(x + 5, tgtY + 4);
      ctx.closePath();
      ctx.fill();
    }

    // Center Pointer
    ctx.restore();
    ctx.fillStyle = '#ffcc00';
    ctx.beginPath();
    ctx.moveTo(x - 20, cy - 5);
    ctx.lineTo(x - 12, cy);
    ctx.lineTo(x - 20, cy + 5);
    ctx.closePath();
    ctx.fill();
  }
}

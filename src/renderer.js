import { CONFIG, clamp } from "./config.js";

function heatColor(v) {
  const x = clamp(v, 0, 1);
  // Blue -> purple -> red.
  const r = Math.floor(25 + 230 * x);
  const g = Math.floor(18 + 60 * (1 - Math.abs(x - 0.55) * 1.8));
  const b = Math.floor(230 - 180 * x);
  return `rgb(${r},${g},${b})`;
}

function roundedRectPath(ctx, x, y, w, h, r) {
  const rr = Math.min(r, w * 0.5, h * 0.5);
  ctx.beginPath();
  ctx.moveTo(x + rr, y);
  ctx.lineTo(x + w - rr, y);
  ctx.quadraticCurveTo(x + w, y, x + w, y + rr);
  ctx.lineTo(x + w, y + h - rr);
  ctx.quadraticCurveTo(x + w, y + h, x + w - rr, y + h);
  ctx.lineTo(x + rr, y + h);
  ctx.quadraticCurveTo(x, y + h, x, y + h - rr);
  ctx.lineTo(x, y + rr);
  ctx.quadraticCurveTo(x, y, x + rr, y);
  ctx.closePath();
}

export class Renderer {
  constructor(worldCanvas, mapCanvas) {
    this.worldCanvas = worldCanvas;
    this.mapCanvas = mapCanvas;
    this.wctx = worldCanvas.getContext("2d");
    this.mctx = mapCanvas.getContext("2d");
    this.pulse = 0;
  }

  drawWorld(env, robot, nav, trail, simTime) {
    const ctx = this.wctx;
    const w = this.worldCanvas.width;
    const h = this.worldCanvas.height;

    ctx.clearRect(0, 0, w, h);
    const bg = ctx.createLinearGradient(0, 0, 0, h);
    bg.addColorStop(0, "#0f1a23");
    bg.addColorStop(1, "#0a1118");
    ctx.fillStyle = bg;
    ctx.fillRect(0, 0, w, h);

    // Debris.
    for (const o of env.obstacles) {
      const g = ctx.createLinearGradient(o.x, o.y, o.x + o.w, o.y + o.h);
      g.addColorStop(0, "#5b3b2f");
      g.addColorStop(1, "#2b1f1b");
      ctx.fillStyle = g;
      ctx.fillRect(o.x, o.y, o.w, o.h);
    }

    // Trail fading.
    for (let i = 1; i < trail.length; i++) {
      const a = i / trail.length;
      ctx.strokeStyle = `rgba(76, 218, 224, ${0.05 + 0.45 * a})`;
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(trail[i - 1].x, trail[i - 1].y);
      ctx.lineTo(trail[i].x, trail[i].y);
      ctx.stroke();
    }

    // Goal marker.
    if (nav.goal) {
      ctx.strokeStyle = "rgba(255, 220, 120, 0.8)";
      ctx.setLineDash([6, 6]);
      ctx.beginPath();
      ctx.arc(nav.goal.x, nav.goal.y, 12, 0, Math.PI * 2);
      ctx.stroke();
      ctx.setLineDash([]);
    }

    // Local planned path (memory-based A*).
    if (nav.path && nav.path.length > 1) {
      ctx.strokeStyle = "rgba(146, 210, 255, 0.55)";
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      const start = nav.path[Math.min(nav.pathIndex, nav.path.length - 1)];
      ctx.moveTo(start.x, start.y);
      for (let i = Math.min(nav.pathIndex + 1, nav.path.length - 1); i < nav.path.length; i++) {
        ctx.lineTo(nav.path[i].x, nav.path[i].y);
      }
      ctx.stroke();
    }

    // Confirmed survivors in green.
    for (const s of env.survivors) {
      if (!s.confirmed) continue;
      ctx.save();
      ctx.shadowColor = "rgba(80, 255, 130, 0.9)";
      ctx.shadowBlur = 14;
      ctx.fillStyle = "#4fff86";
      ctx.beginPath();
      ctx.arc(s.x, s.y, 6, 0, Math.PI * 2);
      ctx.fill();
      ctx.restore();
      ctx.strokeStyle = "rgba(180,255,200,0.95)";
      ctx.lineWidth = 1.8;
      ctx.beginPath();
      ctx.moveTo(s.x - 10, s.y);
      ctx.lineTo(s.x + 10, s.y);
      ctx.moveTo(s.x, s.y - 10);
      ctx.lineTo(s.x, s.y + 10);
      ctx.stroke();
    }

    // Ultrasonic rays.
    const beams = [
      { d: robot.ultra.front, a: robot.theta, c: "rgba(255,215,140,0.45)" },
      { d: robot.ultra.left, a: robot.theta + Math.PI / 4, c: "rgba(140,200,255,0.35)" },
      { d: robot.ultra.right, a: robot.theta - Math.PI / 4, c: "rgba(140,200,255,0.35)" },
    ];
    for (const b of beams) {
      ctx.strokeStyle = b.c;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(robot.x, robot.y);
      ctx.lineTo(robot.x + Math.cos(b.a) * b.d, robot.y + Math.sin(b.a) * b.d);
      ctx.stroke();
    }

    // Radar pulse.
    this.pulse += 0.12;
    const pr = 22 + ((this.pulse + simTime * 1.8) % 34);
    const grad = ctx.createRadialGradient(robot.x, robot.y, 4, robot.x, robot.y, pr);
    grad.addColorStop(0, "rgba(46, 225, 255, 0.25)");
    grad.addColorStop(1, "rgba(46, 225, 255, 0)");
    ctx.fillStyle = grad;
    ctx.beginPath();
    ctx.arc(robot.x, robot.y, pr, 0, Math.PI * 2);
    ctx.fill();

    // Robot body (cyan glow).
    ctx.save();
    ctx.translate(robot.x, robot.y);
    ctx.rotate(robot.theta);
    ctx.shadowColor = "rgba(67, 235, 255, 0.75)";
    ctx.shadowBlur = 16;
    ctx.fillStyle = "#7fe9ff";
    roundedRectPath(ctx, -12, -9, 24, 18, 5);
    ctx.fill();

    ctx.shadowBlur = 0;
    ctx.fillStyle = "#0d1016";
    ctx.fillRect(-14, -10, 4, 8);
    ctx.fillRect(-14, 2, 4, 8);
    ctx.fillRect(10, -10, 4, 8);
    ctx.fillRect(10, 2, 4, 8);

    // Direction arrow.
    ctx.strokeStyle = "#ffe37a";
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(18, 0);
    ctx.stroke();
    ctx.restore();
  }

  drawMap(mapping, clusters, trail, env, simTime = 0) {
    const ctx = this.mctx;
    const w = this.mapCanvas.width;
    const h = this.mapCanvas.height;
    const cw = w / mapping.cols;
    const ch = h / mapping.rows;

    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = "#0b1117";
    ctx.fillRect(0, 0, w, h);
    // Persistent low-level uncertainty haze.
    ctx.fillStyle = "rgba(255, 64, 80, 0.08)";
    ctx.fillRect(0, 0, w, h);

    for (let y = 0; y < mapping.rows; y++) {
      for (let x = 0; x < mapping.cols; x++) {
        const i = mapping.idx(x, y);
        const vx = mapping.visited[i];
        const p = mapping.prob[i];
        const ex = mapping.explored[i];

        if (ex > 0) {
          ctx.fillStyle = "rgba(48, 69, 90, 0.35)";
          ctx.fillRect(x * cw, y * ch, cw, ch);
        }

        if (vx > 0) {
          const a = Math.min(0.32, 0.05 + vx * 0.03);
          ctx.fillStyle = `rgba(88, 108, 124, ${a})`;
          ctx.fillRect(x * cw, y * ch, cw, ch);
        }

        // Full-map, absolute confidence gradient with subtle live noise.
        // Using absolute confidence avoids map drifting visually back to blue.
        const normalized = clamp(p, 0, 1);
        const noise = 0.01 * Math.sin(x * 0.21 + y * 0.17 + simTime * 1.1);
        const displayP = clamp(0.08 + normalized * 0.92 + noise, 0, 1);
        const alpha = 0.18 + 0.48 * displayP;
        ctx.fillStyle = heatColor(displayP);
        ctx.globalAlpha = alpha;
        ctx.fillRect(x * cw, y * ch, cw, ch);
        ctx.globalAlpha = 1;

        if (mapping.obstacle[i]) {
          ctx.fillStyle = "rgba(245, 69, 82, 0.92)";
          ctx.fillRect(x * cw, y * ch, cw, ch);
        }
      }
    }

    // Confirmed survivors in map space.
    for (const s of env.survivors) {
      if (!s.confirmed) continue;
      const mx = (s.x / CONFIG.world.width) * w;
      const my = (s.y / CONFIG.world.height) * h;
      ctx.fillStyle = "rgba(80, 255, 130, 0.95)";
      ctx.beginPath();
      ctx.arc(mx, my, 5, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = "rgba(200,255,220,0.95)";
      ctx.lineWidth = 1.4;
      ctx.stroke();
    }

    // Cluster highlights.
    for (const c of clusters) {
      ctx.strokeStyle = "rgba(255, 238, 132, 0.9)";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc((c.x / CONFIG.world.width) * w, (c.y / CONFIG.world.height) * h, 8 + c.size * 0.2, 0, Math.PI * 2);
      ctx.stroke();
    }

    // Trail on map.
    if (trail.length > 1) {
      ctx.strokeStyle = "rgba(120, 235, 245, 0.75)";
      ctx.lineWidth = 1.8;
      ctx.beginPath();
      for (let i = 0; i < trail.length; i++) {
        const px = (trail[i].x / CONFIG.world.width) * w;
        const py = (trail[i].y / CONFIG.world.height) * h;
        if (i === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      }
      ctx.stroke();
    }
  }
}

import { CONFIG, clamp } from "./config.js";
import { angleToVec, dist } from "./utils.js";

export class MappingSystem {
  constructor() {
    this.cols = Math.floor(CONFIG.world.width / CONFIG.world.mapCell);
    this.rows = Math.floor(CONFIG.world.height / CONFIG.world.mapCell);
    const n = this.cols * this.rows;

    this.visited = new Float32Array(n);
    this.explored = new Uint8Array(n);
    this.obstacle = new Uint8Array(n);
    this.prob = new Float32Array(n);
    this.ambient = new Float32Array(n);
    for (let y = 0; y < this.rows; y++) {
      for (let x = 0; x < this.cols; x++) {
        const i = this.idx(x, y);
        // Non-zero uncertainty prior: map never collapses to zero confidence.
        const n0 = 0.04 * Math.sin(x * 0.21 + y * 0.13) + 0.025 * Math.sin(x * 0.07 - y * 0.17);
        this.ambient[i] = clamp(0.13 + n0, 0.08, 0.22);
        this.prob[i] = this.ambient[i];
      }
    }

    this.blurTimer = 0;
    this.maxProb = 0;
    this.minProb = 0;
    this.coverage = 0;
  }

  idx(cx, cy) {
    return cy * this.cols + cx;
  }

  inBounds(cx, cy) {
    return cx >= 0 && cy >= 0 && cx < this.cols && cy < this.rows;
  }

  worldToCell(x, y) {
    return {
      cx: clamp(Math.floor(x / CONFIG.world.mapCell), 0, this.cols - 1),
      cy: clamp(Math.floor(y / CONFIG.world.mapCell), 0, this.rows - 1),
    };
  }

  cellCenter(cx, cy) {
    const c = CONFIG.world.mapCell;
    return { x: cx * c + c * 0.5, y: cy * c + c * 0.5 };
  }

  markVisited(x, y) {
    const { cx, cy } = this.worldToCell(x, y);
    const i = this.idx(cx, cy);
    this.visited[i] += 1;
    this.explored[i] = 1;
  }

  updateObstacleFromUltrasonic(robot, env) {
    const angles = [
      { d: robot.ultra.front, a: robot.theta },
      { d: robot.ultra.left, a: robot.theta + Math.PI / 4 },
      { d: robot.ultra.right, a: robot.theta - Math.PI / 4 },
    ];

    for (const beam of angles) {
      const ray = env.raycast(robot.x, robot.y, beam.a, beam.d + 4, 2);

      // Persistently mark all free cells seen by ultrasonic beams.
      const steps = Math.max(1, Math.floor(ray.dist / 4));
      for (let s = 0; s <= steps; s++) {
        const t = s / steps;
        const px = robot.x + (ray.x - robot.x) * t;
        const py = robot.y + (ray.y - robot.y) * t;
        const c = this.worldToCell(px, py);
        const i = this.idx(c.cx, c.cy);
        this.explored[i] = 1;
      }

      if (ray.hit) {
        const c = this.worldToCell(ray.x, ray.y);
        const i = this.idx(c.cx, c.cy);
        this.obstacle[i] = 1;
        this.explored[i] = 1;
      }
    }
  }

  updateProbability(robot, dt) {
    // Mean-reverting decay: confidence trends back to uncertain prior, not zero.
    for (let i = 0; i < this.prob.length; i++) {
      this.prob[i] = clamp(this.prob[i] * CONFIG.mapping.decay + this.ambient[i] * (1 - CONFIG.mapping.decay), 0, 1);
    }

    // Sensor noise floor and false returns keep low-level uncertainty alive.
    const rc = this.worldToCell(robot.x, robot.y);
    const rangeCells = Math.ceil(CONFIG.sensors.radar.range / CONFIG.world.mapCell);
    for (let y = rc.cy - rangeCells; y <= rc.cy + rangeCells; y++) {
      for (let x = rc.cx - rangeCells; x <= rc.cx + rangeCells; x++) {
        if (!this.inBounds(x, y)) continue;
        const i = this.idx(x, y);
        const p = this.cellCenter(x, y);
        const d = dist({ x: robot.x, y: robot.y }, p);
        if (d > CONFIG.sensors.radar.range) continue;
        const noise = (Math.sin((x + 1) * 0.31 + (y + 3) * 0.17 + robot.theta * 1.7) + 1) * 0.5;
        const gain = 0.00045 * (0.6 + 0.4 * noise) * (1 - d / CONFIG.sensors.radar.range);
        this.prob[i] = clamp(this.prob[i] + gain * dt * 60, 0, 1);
      }
    }

    if (!robot.radar.detected || robot.radar.signal <= 0.06) return;

    const sensorDir = robot.theta + robot.radar.direction;
    const dirVec = angleToVec(sensorDir);

    for (let y = rc.cy - rangeCells; y <= rc.cy + rangeCells; y++) {
      for (let x = rc.cx - rangeCells; x <= rc.cx + rangeCells; x++) {
        if (!this.inBounds(x, y)) continue;
        const p = this.cellCenter(x, y);
        const d = dist({ x: robot.x, y: robot.y }, p);
        if (d > CONFIG.sensors.radar.range) continue;

        const vec = { x: p.x - robot.x, y: p.y - robot.y };
        const len = Math.max(1e-6, Math.hypot(vec.x, vec.y));
        const directionalWeight = Math.max(0, (dirVec.x * vec.x + dirVec.y * vec.y) / len);
        const gain = CONFIG.mapping.radarGain * robot.radar.signal * directionalWeight;

        const i = this.idx(x, y);
        this.prob[i] = clamp(this.prob[i] + gain * dt * 60, 0, 1);
      }
    }
  }

  suppressConfirmedZones(survivors, dt) {
    const radius = 80;
    const r2 = radius * radius;
    for (const s of survivors) {
      if (!s.confirmed) continue;
      const c = this.worldToCell(s.x, s.y);
      const cellR = Math.ceil(radius / CONFIG.world.mapCell);
      for (let y = c.cy - cellR; y <= c.cy + cellR; y++) {
        for (let x = c.cx - cellR; x <= c.cx + cellR; x++) {
          if (!this.inBounds(x, y)) continue;
          const cc = this.cellCenter(x, y);
          const dx = cc.x - s.x;
          const dy = cc.y - s.y;
          const d2 = dx * dx + dy * dy;
          if (d2 > r2) continue;
          const w = 1 - Math.sqrt(d2) / radius;
          const i = this.idx(x, y);
          // Pull probability back toward ambient around confirmed targets.
          const pull = 0.18 * w * dt * 60;
          this.prob[i] = this.prob[i] * (1 - pull) + this.ambient[i] * pull;
        }
      }
    }
  }

  dampZone(x0, y0, radius = 80, strength = 0.5) {
    const r2 = radius * radius;
    const c = this.worldToCell(x0, y0);
    const cellR = Math.ceil(radius / CONFIG.world.mapCell);
    for (let y = c.cy - cellR; y <= c.cy + cellR; y++) {
      for (let x = c.cx - cellR; x <= c.cx + cellR; x++) {
        if (!this.inBounds(x, y)) continue;
        const cc = this.cellCenter(x, y);
        const dx = cc.x - x0;
        const dy = cc.y - y0;
        const d2 = dx * dx + dy * dy;
        if (d2 > r2) continue;
        const w = 1 - Math.sqrt(d2) / radius;
        const i = this.idx(x, y);
        const pull = clamp(strength * w, 0, 1);
        this.prob[i] = this.prob[i] * (1 - pull) + this.ambient[i] * pull;
      }
    }
  }

  blurProbability() {
    const src = this.prob;
    const tmp = new Float32Array(src.length);
    const dst = new Float32Array(src.length);
    const kernel = [1, 4, 6, 4, 1];
    const norm = 16;

    for (let y = 0; y < this.rows; y++) {
      for (let x = 0; x < this.cols; x++) {
        let acc = 0;
        for (let k = -2; k <= 2; k++) {
          const xx = clamp(x + k, 0, this.cols - 1);
          acc += src[this.idx(xx, y)] * kernel[k + 2];
        }
        tmp[this.idx(x, y)] = acc / norm;
      }
    }

    for (let y = 0; y < this.rows; y++) {
      for (let x = 0; x < this.cols; x++) {
        let acc = 0;
        for (let k = -2; k <= 2; k++) {
          const yy = clamp(y + k, 0, this.rows - 1);
          acc += tmp[this.idx(x, yy)] * kernel[k + 2];
        }
        dst[this.idx(x, y)] = clamp(acc / norm, 0, 1);
      }
    }

    this.prob = dst;
  }

  clusterHighProb() {
    const threshold = CONFIG.mapping.clusterThreshold;
    const seen = new Uint8Array(this.prob.length);
    const clusters = [];

    const neighbors = [
      [1, 0],
      [-1, 0],
      [0, 1],
      [0, -1],
      [1, 1],
      [1, -1],
      [-1, 1],
      [-1, -1],
    ];

    for (let y = 0; y < this.rows; y++) {
      for (let x = 0; x < this.cols; x++) {
        const i0 = this.idx(x, y);
        if (seen[i0] || this.prob[i0] < threshold) continue;

        const q = [{ x, y }];
        seen[i0] = 1;
        let sumX = 0;
        let sumY = 0;
        let sumW = 0;
        let peak = 0;
        let count = 0;

        while (q.length) {
          const c = q.pop();
          const i = this.idx(c.x, c.y);
          const w = this.prob[i];
          const center = this.cellCenter(c.x, c.y);
          sumX += center.x * w;
          sumY += center.y * w;
          sumW += w;
          peak = Math.max(peak, w);
          count++;

          for (const [dx, dy] of neighbors) {
            const nx = c.x + dx;
            const ny = c.y + dy;
            if (!this.inBounds(nx, ny)) continue;
            const ni = this.idx(nx, ny);
            if (seen[ni] || this.prob[ni] < threshold) continue;
            seen[ni] = 1;
            q.push({ x: nx, y: ny });
          }
        }

        if (count < 2 || sumW <= 0) continue;
        clusters.push({
          x: sumX / sumW,
          y: sumY / sumW,
          confidence: peak,
          size: count,
        });
      }
    }

    return clusters.sort((a, b) => b.confidence - a.confidence);
  }

  updateCoverage() {
    let exploredCells = 0;
    for (let i = 0; i < this.explored.length; i++) if (this.explored[i]) exploredCells++;
    this.coverage = exploredCells / this.explored.length;
    this.maxProb = 0;
    this.minProb = 1;
    for (let i = 0; i < this.prob.length; i++) {
      this.maxProb = Math.max(this.maxProb, this.prob[i]);
      this.minProb = Math.min(this.minProb, this.prob[i]);
    }
  }

  step(robot, env, dt) {
    this.markVisited(robot.x, robot.y);
    this.updateObstacleFromUltrasonic(robot, env);
    this.updateProbability(robot, dt);
    this.suppressConfirmedZones(env.survivors, dt);

    this.blurTimer += dt;
    if (this.blurTimer >= CONFIG.mapping.blurInterval) {
      this.blurTimer = 0;
      this.blurProbability();
    }

    this.updateCoverage();
  }
}

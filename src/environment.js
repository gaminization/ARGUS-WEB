import { CONFIG, clamp, randRange } from "./config.js";
import { dist } from "./utils.js";

export class Environment {
  constructor(rng) {
    this.rng = rng;
    this.width = CONFIG.world.width;
    this.height = CONFIG.world.height;
    this.obstacles = [];
    this.survivors = [];
    this.generate();
  }

  generate() {
    // Outer boundaries.
    this.obstacles.push({ x: 0, y: 0, w: this.width, h: 14 });
    this.obstacles.push({ x: 0, y: this.height - 14, w: this.width, h: 14 });
    this.obstacles.push({ x: 0, y: 0, w: 14, h: this.height });
    this.obstacles.push({ x: this.width - 14, y: 0, w: 14, h: this.height });

    // Debris clusters with narrow passages.
    for (let i = 0; i < 36; i++) {
      const cx = randRange(this.rng, 80, this.width - 80);
      const cy = randRange(this.rng, 80, this.height - 80);
      const pieces = 3 + Math.floor(this.rng() * 5);
      for (let p = 0; p < pieces; p++) {
        const w = randRange(this.rng, 18, 62);
        const h = randRange(this.rng, 18, 70);
        const x = clamp(cx + randRange(this.rng, -40, 40) - w / 2, 16, this.width - 16 - w);
        const y = clamp(cy + randRange(this.rng, -40, 40) - h / 2, 16, this.height - 16 - h);
        this.obstacles.push({ x, y, w, h });
      }
    }

    // Carve a few open areas by removing intersecting rubble pieces.
    const clearZones = [
      { x: 130, y: 120, r: 70 },
      { x: 650, y: 610, r: 85 },
      { x: 400, y: 380, r: 110 },
    ];

    this.obstacles = this.obstacles.filter((o) => {
      if (o.w >= this.width - 20 || o.h >= this.height - 20) return true;
      const center = { x: o.x + o.w * 0.5, y: o.y + o.h * 0.5 };
      return !clearZones.some((z) => dist(center, z) < z.r);
    });

    this.spawnSurvivors(6);
  }

  spawnSurvivors(count) {
    let tries = 0;
    while (this.survivors.length < count && tries < 5000) {
      tries++;
      const p = {
        x: randRange(this.rng, 60, this.width - 60),
        y: randRange(this.rng, 60, this.height - 60),
      };
      if (this.collidesCircle(p.x, p.y, 18)) continue;
      if (p.x < 160 && p.y < 180) continue;
      if (this.survivors.some((s) => dist(s, p) < 110)) continue;
      this.survivors.push({
        ...p,
        breathingHz: randRange(this.rng, 0.14, 0.32),
        phase: randRange(this.rng, 0, Math.PI * 2),
        confirmed: false,
      });
    }
  }

  collidesCircle(x, y, radius) {
    for (const o of this.obstacles) {
      const nx = clamp(x, o.x, o.x + o.w);
      const ny = clamp(y, o.y, o.y + o.h);
      if ((x - nx) * (x - nx) + (y - ny) * (y - ny) <= radius * radius) return true;
    }
    return false;
  }

  raycast(x, y, angle, maxRange, step = 2) {
    const dx = Math.cos(angle);
    const dy = Math.sin(angle);
    for (let d = 0; d <= maxRange; d += step) {
      const px = x + dx * d;
      const py = y + dy * d;
      if (px < 0 || py < 0 || px >= this.width || py >= this.height) {
        return { hit: true, dist: d, x: px, y: py };
      }
      if (this.collidesCircle(px, py, 1)) {
        return { hit: true, dist: d, x: px, y: py };
      }
    }
    return {
      hit: false,
      dist: maxRange,
      x: x + Math.cos(angle) * maxRange,
      y: y + Math.sin(angle) * maxRange,
    };
  }
}

const canvas = document.getElementById("simCanvas");
const ctx = canvas.getContext("2d");

const ui = {
  startBtn: document.getElementById("startBtn"),
  pauseBtn: document.getElementById("pauseBtn"),
  resetBtn: document.getElementById("resetBtn"),
  speedSlider: document.getElementById("speedSlider"),
  modeValue: document.getElementById("modeValue"),
  positionValue: document.getElementById("positionValue"),
  headingValue: document.getElementById("headingValue"),
  wheelValue: document.getElementById("wheelValue"),
  ultraValue: document.getElementById("ultraValue"),
  imuValue: document.getElementById("imuValue"),
  gpsValue: document.getElementById("gpsValue"),
  ldValue: document.getElementById("ldValue"),
  scoreValue: document.getElementById("scoreValue"),
  coverageValue: document.getElementById("coverageValue"),
  survivorValue: document.getElementById("survivorValue"),
  collisionValue: document.getElementById("collisionValue"),
  revisitValue: document.getElementById("revisitValue"),
  timeValue: document.getElementById("timeValue"),
};

const GRID_W = 49;
const GRID_H = 31;
const CELL = 20;
const WORLD_W = GRID_W * CELL;
const WORLD_H = GRID_H * CELL;
const SENSOR_MAX = 110;

function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

function normalizeAngle(a) {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

function dist(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

function mulberry32(seed) {
  return function () {
    let t = (seed += 0x6d2b79f5);
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

class PID {
  constructor(kp, ki, kd, outMin = -Infinity, outMax = Infinity) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.outMin = outMin;
    this.outMax = outMax;
    this.integral = 0;
    this.prevError = 0;
  }

  reset() {
    this.integral = 0;
    this.prevError = 0;
  }

  update(error, dt) {
    this.integral += error * dt;
    const derivative = dt > 0 ? (error - this.prevError) / dt : 0;
    this.prevError = error;
    return clamp(this.kp * error + this.ki * this.integral + this.kd * derivative, this.outMin, this.outMax);
  }
}

class Simulation {
  constructor(seed = 20260325) {
    this.random = mulberry32(seed);
    this.running = false;
    this.simSpeed = 1;
    this.time = 0;
    this.lastTs = performance.now();

    this.grid = Array.from({ length: GRID_H }, () => Array(GRID_W).fill(0));
    this.known = Array.from({ length: GRID_H }, () => Array(GRID_W).fill(-1));
    this.explored = Array.from({ length: GRID_H }, () => Array(GRID_W).fill(false));
    this.visited = Array.from({ length: GRID_H }, () => Array(GRID_W).fill(false));
    this.prob = Array.from({ length: GRID_H }, () => Array(GRID_W).fill(0.05));
    this.confirmed = Array.from({ length: GRID_H }, () => Array(GRID_W).fill(false));

    this.survivors = [];
    this.trail = [];

    this.score = 0;
    this.collisions = 0;
    this.revisitViolations = 0;
    this.coverage = 0;
    this.mode = "Idle";

    this.lastPlanTime = 0;
    this.path = [];
    this.pathIndex = 0;

    this.wallPid = new PID(0.035, 0.0008, 0.02, -1.4, 1.4);

    this.robot = {
      x: 2.5 * CELL,
      y: 2.5 * CELL,
      heading: 0,
      radius: 7,
      wheelBase: 16,
      leftVel: 0,
      rightVel: 0,
      maxWheelVel: 44,
      ultrasonic: { front: SENSOR_MAX, left: SENSOR_MAX, right: SENSOR_MAX },
      imu: { yaw: 0, accel: 0 },
      gps: { x: 0, y: 0, timer: 0 },
      encoders: { left: 0, right: 0 },
      ld2410: { strength: 0, detected: false },
    };

    this.generateWorld();
    this.initRobotFreeZone();
    this.spawnSurvivors(7);
    this.exploreAroundRobot();
  }

  reset() {
    const newSeed = Math.floor(Math.random() * 1e9);
    const next = new Simulation(newSeed);
    Object.assign(this, next);
  }

  cellOf(x, y) {
    return {
      cx: clamp(Math.floor(x / CELL), 0, GRID_W - 1),
      cy: clamp(Math.floor(y / CELL), 0, GRID_H - 1),
    };
  }

  cellCenter(cx, cy) {
    return { x: cx * CELL + CELL / 2, y: cy * CELL + CELL / 2 };
  }

  inBounds(cx, cy) {
    return cx >= 0 && cy >= 0 && cx < GRID_W && cy < GRID_H;
  }

  isObstacleAtPixel(x, y) {
    const { cx, cy } = this.cellOf(x, y);
    return this.grid[cy][cx] === 1;
  }

  generateWorld() {
    for (let y = 0; y < GRID_H; y++) {
      for (let x = 0; x < GRID_W; x++) {
        if (y === 0 || y === GRID_H - 1 || x === 0 || x === GRID_W - 1) {
          this.grid[y][x] = 1;
        }
      }
    }

    const rubbleBlobs = 48;
    for (let i = 0; i < rubbleBlobs; i++) {
      const cx = 2 + Math.floor(this.random() * (GRID_W - 4));
      const cy = 2 + Math.floor(this.random() * (GRID_H - 4));
      const rx = 1 + Math.floor(this.random() * 3);
      const ry = 1 + Math.floor(this.random() * 2);
      for (let y = cy - ry; y <= cy + ry; y++) {
        for (let x = cx - rx; x <= cx + rx; x++) {
          if (!this.inBounds(x, y)) continue;
          if (this.random() < 0.74) this.grid[y][x] = 1;
        }
      }
    }

    // Create traversable corridors by carving paths.
    for (let c = 0; c < 8; c++) {
      let x = 1 + Math.floor(this.random() * (GRID_W - 2));
      let y = 1 + Math.floor(this.random() * (GRID_H - 2));
      for (let k = 0; k < 120; k++) {
        if (this.inBounds(x, y)) this.grid[y][x] = 0;
        const dir = Math.floor(this.random() * 4);
        if (dir === 0) x++;
        if (dir === 1) x--;
        if (dir === 2) y++;
        if (dir === 3) y--;
        x = clamp(x, 1, GRID_W - 2);
        y = clamp(y, 1, GRID_H - 2);
      }
    }
  }

  initRobotFreeZone() {
    for (let y = 1; y <= 4; y++) {
      for (let x = 1; x <= 4; x++) {
        this.grid[y][x] = 0;
      }
    }
    const start = this.cellOf(this.robot.x, this.robot.y);
    this.visited[start.cy][start.cx] = true;
    this.known[start.cy][start.cx] = 0;
  }

  spawnSurvivors(count) {
    let attempts = 0;
    while (this.survivors.length < count && attempts < 5000) {
      attempts++;
      const x = 3 + Math.floor(this.random() * (GRID_W - 6));
      const y = 3 + Math.floor(this.random() * (GRID_H - 6));
      if (this.grid[y][x] === 1) continue;
      if (x < 8 && y < 8) continue;
      const p = this.cellCenter(x, y);
      const ok = this.survivors.every((s) => dist(s, p) > CELL * 5);
      if (ok) {
        this.survivors.push({
          x: p.x,
          y: p.y,
          breathingHz: 0.18 + this.random() * 0.15,
          phase: this.random() * Math.PI * 2,
          confirmed: false,
        });
      }
    }
  }

  castRay(originX, originY, angle, maxDist) {
    const step = 2;
    let x = originX;
    let y = originY;
    for (let d = 0; d <= maxDist; d += step) {
      x = originX + Math.cos(angle) * d;
      y = originY + Math.sin(angle) * d;
      if (x <= 0 || y <= 0 || x >= WORLD_W || y >= WORLD_H) {
        return { dist: d, hit: true, x, y };
      }
      if (this.isObstacleAtPixel(x, y)) {
        return { dist: d, hit: true, x, y };
      }
    }
    return {
      dist: maxDist,
      hit: false,
      x: originX + Math.cos(angle) * maxDist,
      y: originY + Math.sin(angle) * maxDist,
    };
  }

  readUltrasonic() {
    const a = this.robot.heading;
    const rays = {
      front: this.castRay(this.robot.x, this.robot.y, a, SENSOR_MAX),
      left: this.castRay(this.robot.x, this.robot.y, a - Math.PI / 2, SENSOR_MAX),
      right: this.castRay(this.robot.x, this.robot.y, a + Math.PI / 2, SENSOR_MAX),
    };
    this.robot.ultrasonic.front = rays.front.dist + (this.random() - 0.5) * 2.5;
    this.robot.ultrasonic.left = rays.left.dist + (this.random() - 0.5) * 2.5;
    this.robot.ultrasonic.right = rays.right.dist + (this.random() - 0.5) * 2.5;
    return rays;
  }

  updateImu(prevV, currentV, dt) {
    const yawNoise = (this.random() - 0.5) * 0.02;
    const accelNoise = (this.random() - 0.5) * 0.8;
    this.robot.imu.yaw = this.robot.heading + yawNoise;
    this.robot.imu.accel = dt > 0 ? (currentV - prevV) / dt + accelNoise : accelNoise;
  }

  updateGps(dt) {
    this.robot.gps.timer += dt;
    if (this.robot.gps.timer >= 0.55) {
      this.robot.gps.timer = 0;
      this.robot.gps.x = this.robot.x + (this.random() - 0.5) * 4.5;
      this.robot.gps.y = this.robot.y + (this.random() - 0.5) * 4.5;
    }
  }

  updateLd2410(dt) {
    const range = 120;
    const sigma = 42;
    let signal = 0;
    for (const s of this.survivors) {
      const d = Math.hypot(this.robot.x - s.x, this.robot.y - s.y);
      if (d < range) {
        const breathing = 0.5 + 0.5 * Math.sin(this.time * 2 * Math.PI * s.breathingHz + s.phase);
        signal += Math.exp(-(d * d) / (2 * sigma * sigma)) * (0.55 + 0.45 * breathing);
      }
    }
    signal += (this.random() - 0.5) * 0.05;
    signal = clamp(signal, 0, 1.2);
    this.robot.ld2410.strength = signal;
    this.robot.ld2410.detected = signal > 0.22;

    this.updateSurvivorProbability(signal, range, sigma, dt);
  }

  updateSurvivorProbability(signal, range, sigma, dt) {
    const { cx, cy } = this.cellOf(this.robot.x, this.robot.y);
    const cells = Math.ceil(range / CELL);

    for (let y = cy - cells; y <= cy + cells; y++) {
      for (let x = cx - cells; x <= cx + cells; x++) {
        if (!this.inBounds(x, y)) continue;
        if (this.known[y][x] === 1) continue;
        const cc = this.cellCenter(x, y);
        const d = Math.hypot(this.robot.x - cc.x, this.robot.y - cc.y);
        if (d > range) continue;

        const expected = Math.exp(-(d * d) / (2 * sigma * sigma));
        const likelihood = Math.exp(-Math.pow(signal - expected, 2) / (2 * 0.22 * 0.22));
        const prior = this.prob[y][x];
        const posterior = clamp(prior + (likelihood - 0.5) * 0.08 * dt * this.simSpeed, 0.01, 0.99);
        this.prob[y][x] = posterior;
      }
    }

    for (const s of this.survivors) {
      if (s.confirmed) continue;
      const sc = this.cellOf(s.x, s.y);
      if (this.prob[sc.cy][sc.cx] > 0.83 && Math.hypot(this.robot.x - s.x, this.robot.y - s.y) < 70) {
        s.confirmed = true;
        this.confirmed[sc.cy][sc.cx] = true;
      }
    }
  }

  integrateOdometry(dt) {
    this.robot.encoders.left += this.robot.leftVel * dt;
    this.robot.encoders.right += this.robot.rightVel * dt;
  }

  exploreAroundRobot() {
    const { cx, cy } = this.cellOf(this.robot.x, this.robot.y);
    for (let y = cy - 1; y <= cy + 1; y++) {
      for (let x = cx - 1; x <= cx + 1; x++) {
        if (!this.inBounds(x, y)) continue;
        this.explored[y][x] = true;
        if (this.known[y][x] === -1) this.known[y][x] = 0;
      }
    }
  }

  updateMapWithRays(rays) {
    const rayList = [rays.front, rays.left, rays.right];
    for (const ray of rayList) {
      const steps = Math.floor(ray.dist / 2);
      for (let i = 0; i < steps; i++) {
        const t = i / Math.max(steps, 1);
        const px = this.robot.x + (ray.x - this.robot.x) * t;
        const py = this.robot.y + (ray.y - this.robot.y) * t;
        const { cx, cy } = this.cellOf(px, py);
        if (!this.inBounds(cx, cy)) continue;
        this.explored[cy][cx] = true;
        this.known[cy][cx] = 0;
      }
      const hit = this.cellOf(ray.x, ray.y);
      if (this.inBounds(hit.cx, hit.cy)) {
        this.explored[hit.cy][hit.cx] = true;
        if (ray.hit) this.known[hit.cy][hit.cx] = 1;
      }
    }
    this.exploreAroundRobot();
  }

  neighbors(cx, cy) {
    const out = [];
    const dirs = [
      [1, 0],
      [-1, 0],
      [0, 1],
      [0, -1],
    ];
    for (const [dx, dy] of dirs) {
      const nx = cx + dx;
      const ny = cy + dy;
      if (this.inBounds(nx, ny)) out.push({ cx: nx, cy: ny });
    }
    return out;
  }

  pickFrontierTarget() {
    let best = null;
    const robotCell = this.cellOf(this.robot.x, this.robot.y);

    for (let y = 1; y < GRID_H - 1; y++) {
      for (let x = 1; x < GRID_W - 1; x++) {
        if (this.known[y][x] !== 0 || this.visited[y][x]) continue;
        const n = this.neighbors(x, y);
        const hasUnknownNeighbor = n.some((k) => this.known[k.cy][k.cx] === -1);
        if (!hasUnknownNeighbor) continue;
        const d = Math.abs(robotCell.cx - x) + Math.abs(robotCell.cy - y);
        if (!best || d < best.d) {
          best = { cx: x, cy: y, d };
        }
      }
    }

    if (best) return best;

    for (let y = 1; y < GRID_H - 1; y++) {
      for (let x = 1; x < GRID_W - 1; x++) {
        if (this.known[y][x] !== 0 || this.visited[y][x]) continue;
        const d = Math.abs(robotCell.cx - x) + Math.abs(robotCell.cy - y);
        if (!best || d < best.d) {
          best = { cx: x, cy: y, d };
        }
      }
    }

    return best;
  }

  planPath() {
    const start = this.cellOf(this.robot.x, this.robot.y);
    const goal = this.pickFrontierTarget();
    if (!goal) {
      this.path = [];
      this.pathIndex = 0;
      return false;
    }

    const open = [start];
    const key = (c) => `${c.cx},${c.cy}`;
    const came = new Map();
    const g = new Map([[key(start), 0]]);
    const f = new Map([[key(start), Math.abs(start.cx - goal.cx) + Math.abs(start.cy - goal.cy)]]);

    while (open.length) {
      let bestIdx = 0;
      for (let i = 1; i < open.length; i++) {
        if ((f.get(key(open[i])) ?? Infinity) < (f.get(key(open[bestIdx])) ?? Infinity)) bestIdx = i;
      }
      const current = open.splice(bestIdx, 1)[0];
      const cKey = key(current);
      if (current.cx === goal.cx && current.cy === goal.cy) {
        const path = [current];
        let back = cKey;
        while (came.has(back)) {
          const p = came.get(back);
          path.push(p);
          back = key(p);
        }
        path.reverse();
        this.path = path;
        this.pathIndex = 1;
        return true;
      }

      for (const n of this.neighbors(current.cx, current.cy)) {
        if (this.known[n.cy][n.cx] !== 0) continue;
        if (this.visited[n.cy][n.cx] && !(n.cx === goal.cx && n.cy === goal.cy)) continue;

        const nKey = key(n);
        const tentative = (g.get(cKey) ?? Infinity) + 1;
        if (tentative < (g.get(nKey) ?? Infinity)) {
          came.set(nKey, current);
          g.set(nKey, tentative);
          const h = Math.abs(n.cx - goal.cx) + Math.abs(n.cy - goal.cy);
          f.set(nKey, tentative + h);
          if (!open.some((o) => o.cx === n.cx && o.cy === n.cy)) open.push(n);
        }
      }
    }

    this.path = [];
    this.pathIndex = 0;
    return false;
  }

  followPathControl(dt) {
    if (!this.path.length || this.pathIndex >= this.path.length) {
      this.mode = "Replanning";
      this.robot.leftVel = 0;
      this.robot.rightVel = 0;
      return;
    }

    const next = this.path[this.pathIndex];
    const wp = this.cellCenter(next.cx, next.cy);
    const dx = wp.x - this.robot.x;
    const dy = wp.y - this.robot.y;
    const distance = Math.hypot(dx, dy);

    if (distance < 3.2) {
      this.pathIndex++;
      return;
    }

    const desiredHeading = Math.atan2(dy, dx);
    const headingErr = normalizeAngle(desiredHeading - this.robot.heading);
    const headingTerm = clamp(headingErr * 1.6, -1.6, 1.6);

    const rightDist = this.robot.ultrasonic.right;
    const wallErr = rightDist < SENSOR_MAX - 2 ? 28 - rightDist : 0;
    const wallTerm = this.wallPid.update(wallErr, dt);

    let steer = headingTerm + wallTerm;
    const front = this.robot.ultrasonic.front;

    let base = 24;
    this.mode = "Path + PID Wall Follow";

    if (front < 14) {
      base = 0;
      steer = -1.8;
      this.mode = "Obstacle Avoidance";
    }

    const lv = clamp(base - steer * 8.5, -this.robot.maxWheelVel, this.robot.maxWheelVel);
    const rv = clamp(base + steer * 8.5, -this.robot.maxWheelVel, this.robot.maxWheelVel);

    this.robot.leftVel = lv;
    this.robot.rightVel = rv;
  }

  tryMove(dt) {
    const prev = { x: this.robot.x, y: this.robot.y };

    const v = (this.robot.leftVel + this.robot.rightVel) / 2;
    const omega = (this.robot.rightVel - this.robot.leftVel) / this.robot.wheelBase;

    const newHeading = this.robot.heading + omega * dt;
    const nx = this.robot.x + Math.cos(newHeading) * v * dt;
    const ny = this.robot.y + Math.sin(newHeading) * v * dt;

    const newCell = this.cellOf(nx, ny);
    const currentCell = this.cellOf(this.robot.x, this.robot.y);

    let collision = false;
    if (nx < CELL || ny < CELL || nx > WORLD_W - CELL || ny > WORLD_H - CELL) collision = true;
    if (this.grid[newCell.cy][newCell.cx] === 1) collision = true;

    if (!collision && !(newCell.cx === currentCell.cx && newCell.cy === currentCell.cy)) {
      if (this.visited[newCell.cy][newCell.cx]) {
        this.revisitViolations += 1;
        this.path = [];
        this.pathIndex = 0;
        collision = true;
      }
    }

    if (collision) {
      this.collisions += 1;
      this.robot.leftVel = -8;
      this.robot.rightVel = 8;
      return { prev, v: 0 };
    }

    this.robot.heading = normalizeAngle(newHeading);
    this.robot.x = nx;
    this.robot.y = ny;

    const c = this.cellOf(this.robot.x, this.robot.y);
    this.visited[c.cy][c.cx] = true;
    this.known[c.cy][c.cx] = 0;
    this.explored[c.cy][c.cx] = true;
    this.trail.push({ x: this.robot.x, y: this.robot.y });
    if (this.trail.length > 1200) this.trail.shift();

    return { prev, v };
  }

  updateMissionScore(dt) {
    let exploredCount = 0;
    let freeCount = 0;
    for (let y = 0; y < GRID_H; y++) {
      for (let x = 0; x < GRID_W; x++) {
        if (this.grid[y][x] === 0) freeCount++;
        if (this.explored[y][x] && this.grid[y][x] === 0) exploredCount++;
      }
    }

    this.coverage = freeCount > 0 ? exploredCount / freeCount : 0;
    const confirmedCount = this.survivors.filter((s) => s.confirmed).length;

    this.score =
      this.coverage * 1000 +
      confirmedCount * 260 -
      this.collisions * 70 -
      this.revisitViolations * 22 -
      this.time * 0.45;

    this.score = Math.max(0, this.score);

    if (this.coverage > 0.98 || this.path.length === 0 && this.time > 12 && confirmedCount === this.survivors.length) {
      this.mode = "Mission Complete";
      this.running = false;
    }

    if (!this.path.length && this.time > 12 && confirmedCount > 0 && dt > 0) {
      const anyTarget = this.pickFrontierTarget();
      if (!anyTarget) {
        this.mode = "No Unvisited Frontier";
        this.running = false;
      }
    }
  }

  updateUI() {
    const c = this.cellOf(this.robot.x, this.robot.y);
    const deg = ((this.robot.heading * 180) / Math.PI + 360) % 360;
    ui.modeValue.textContent = this.mode;
    ui.positionValue.textContent = `(${c.cx}, ${c.cy})`;
    ui.headingValue.textContent = `${deg.toFixed(1)}°`;
    ui.wheelValue.textContent = `L:${this.robot.leftVel.toFixed(1)} R:${this.robot.rightVel.toFixed(1)}`;
    ui.ultraValue.textContent = `F:${this.robot.ultrasonic.front.toFixed(1)} L:${this.robot.ultrasonic.left.toFixed(1)} R:${this.robot.ultrasonic.right.toFixed(1)}`;
    ui.imuValue.textContent = `${this.robot.imu.yaw.toFixed(2)} / ${this.robot.imu.accel.toFixed(2)}`;
    ui.gpsValue.textContent = `${this.robot.gps.x.toFixed(1)}, ${this.robot.gps.y.toFixed(1)}`;
    ui.ldValue.textContent = this.robot.ld2410.detected
      ? `Breathing signature ${this.robot.ld2410.strength.toFixed(2)}`
      : `No target (${this.robot.ld2410.strength.toFixed(2)})`;

    ui.scoreValue.textContent = Math.round(this.score).toString();
    ui.coverageValue.textContent = `${(this.coverage * 100).toFixed(1)}%`;
    ui.survivorValue.textContent = this.survivors.filter((s) => s.confirmed).length.toString();
    ui.collisionValue.textContent = this.collisions.toString();
    ui.revisitValue.textContent = this.revisitViolations.toString();
    ui.timeValue.textContent = `${this.time.toFixed(1)}s`;
  }

  drawGrid() {
    for (let y = 0; y < GRID_H; y++) {
      for (let x = 0; x < GRID_W; x++) {
        const px = x * CELL;
        const py = y * CELL;

        let baseColor = "#19222c";
        if (this.explored[y][x]) baseColor = "#2a3d4f";
        if (this.known[y][x] === 1) baseColor = "#c6423f";
        ctx.fillStyle = baseColor;
        ctx.fillRect(px, py, CELL, CELL);

        if (this.known[y][x] !== 1 && this.explored[y][x]) {
          const p = this.prob[y][x];
          if (p > 0.1) {
            const alpha = clamp((p - 0.1) / 0.9, 0, 1) * 0.65;
            ctx.fillStyle = `rgba(240, 182, 67, ${alpha.toFixed(3)})`;
            ctx.fillRect(px + 2, py + 2, CELL - 4, CELL - 4);
          }
        }

        if (this.confirmed[y][x]) {
          ctx.fillStyle = "#39d67a";
          ctx.beginPath();
          ctx.arc(px + CELL / 2, py + CELL / 2, 5, 0, Math.PI * 2);
          ctx.fill();
        }
      }
    }

    ctx.strokeStyle = "rgba(200, 220, 240, 0.08)";
    ctx.lineWidth = 1;
    for (let x = 0; x <= GRID_W; x++) {
      ctx.beginPath();
      ctx.moveTo(x * CELL + 0.5, 0);
      ctx.lineTo(x * CELL + 0.5, WORLD_H);
      ctx.stroke();
    }
    for (let y = 0; y <= GRID_H; y++) {
      ctx.beginPath();
      ctx.moveTo(0, y * CELL + 0.5);
      ctx.lineTo(WORLD_W, y * CELL + 0.5);
      ctx.stroke();
    }
  }

  drawTrail() {
    if (this.trail.length < 2) return;
    ctx.strokeStyle = "rgba(78, 205, 196, 0.75)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(this.trail[0].x, this.trail[0].y);
    for (let i = 1; i < this.trail.length; i++) {
      ctx.lineTo(this.trail[i].x, this.trail[i].y);
    }
    ctx.stroke();
  }

  drawRobot() {
    const r = this.robot;

    ctx.save();
    ctx.translate(r.x, r.y);
    ctx.rotate(r.heading);

    // Chassis
    const chassis = ctx.createLinearGradient(-10, -10, 10, 10);
    chassis.addColorStop(0, "#ced9e6");
    chassis.addColorStop(1, "#71859a");
    ctx.fillStyle = chassis;
    ctx.beginPath();
    ctx.roundRect(-10, -8, 20, 16, 4);
    ctx.fill();

    // Wheels
    ctx.fillStyle = "#151b21";
    ctx.fillRect(-11, -10, 4, 7);
    ctx.fillRect(-11, 3, 4, 7);
    ctx.fillRect(7, -10, 4, 7);
    ctx.fillRect(7, 3, 4, 7);

    // Sensor head
    ctx.fillStyle = "#4ecdc4";
    ctx.beginPath();
    ctx.arc(8, 0, 3, 0, Math.PI * 2);
    ctx.fill();

    // Forward indicator
    ctx.strokeStyle = "#ffd166";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(15, 0);
    ctx.stroke();

    ctx.restore();

    // Ultrasonic rays visualization
    const rays = [
      { d: r.ultrasonic.front, a: r.heading, c: "rgba(255, 209, 102, 0.35)" },
      { d: r.ultrasonic.left, a: r.heading - Math.PI / 2, c: "rgba(138, 201, 255, 0.28)" },
      { d: r.ultrasonic.right, a: r.heading + Math.PI / 2, c: "rgba(138, 201, 255, 0.28)" },
    ];
    for (const ray of rays) {
      ctx.strokeStyle = ray.c;
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      ctx.moveTo(r.x, r.y);
      ctx.lineTo(r.x + Math.cos(ray.a) * ray.d, r.y + Math.sin(ray.a) * ray.d);
      ctx.stroke();
    }
  }

  drawPath() {
    if (!this.path.length || this.pathIndex >= this.path.length) return;
    ctx.strokeStyle = "rgba(129, 191, 255, 0.7)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    const start = this.cellCenter(this.path[this.pathIndex].cx, this.path[this.pathIndex].cy);
    ctx.moveTo(start.x, start.y);
    for (let i = this.pathIndex + 1; i < this.path.length; i++) {
      const p = this.cellCenter(this.path[i].cx, this.path[i].cy);
      ctx.lineTo(p.x, p.y);
    }
    ctx.stroke();
  }

  render() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    const bgGrad = ctx.createLinearGradient(0, 0, 0, canvas.height);
    bgGrad.addColorStop(0, "#17212b");
    bgGrad.addColorStop(1, "#101820");
    ctx.fillStyle = bgGrad;
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    this.drawGrid();
    this.drawTrail();
    this.drawPath();
    this.drawRobot();
  }

  step(dtRaw) {
    const dt = dtRaw * this.simSpeed;
    this.time += dt;

    const rays = this.readUltrasonic();
    this.updateMapWithRays(rays);
    this.updateLd2410(dt);

    if (this.time - this.lastPlanTime > 0.8 || !this.path.length || this.pathIndex >= this.path.length) {
      this.planPath();
      this.lastPlanTime = this.time;
    }

    this.followPathControl(dt);

    const prevV = (this.robot.leftVel + this.robot.rightVel) / 2;
    const moved = this.tryMove(dt);
    const currentV = moved.v;
    this.integrateOdometry(dt);
    this.updateImu(prevV, currentV, dt);
    this.updateGps(dt);

    this.updateMissionScore(dt);
    this.updateUI();
    this.render();
  }

  loop = (ts) => {
    const deltaMs = clamp(ts - this.lastTs, 0, 40);
    this.lastTs = ts;
    if (this.running) {
      this.step(deltaMs / 1000);
    } else {
      this.updateUI();
      this.render();
    }
    requestAnimationFrame(this.loop);
  };
}

const sim = new Simulation();
requestAnimationFrame((ts) => {
  sim.lastTs = ts;
  sim.loop(ts);
});

ui.startBtn.addEventListener("click", () => {
  sim.running = true;
  sim.mode = "Exploring";
});

ui.pauseBtn.addEventListener("click", () => {
  sim.running = false;
  if (sim.mode !== "Mission Complete") sim.mode = "Paused";
});

ui.resetBtn.addEventListener("click", () => {
  sim.reset();
  sim.mode = "Idle";
});

ui.speedSlider.addEventListener("input", (e) => {
  sim.simSpeed = Number(e.target.value);
});

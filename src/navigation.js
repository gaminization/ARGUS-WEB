import { CONFIG, clamp, normAngle } from "./config.js";
import { angleToVec, dist, vAdd, vLen, vNorm, vScale, vecToAngle } from "./utils.js";

export class Navigation {
  constructor(rng, mapping) {
    this.rng = rng;
    this.mapping = mapping;
    this.goal = null;
    this.goalTimer = 0;
    this.lastForce = { x: 1, y: 0 };
    this.stuckBurst = 0;

    this.path = [];
    this.pathIndex = 0;
    this.pathTimer = 0;
    this.inflation = 2;

    this.escapePhase = 0;
    this.escapeTimer = 0;
    this.escapeTurnSign = 1;
    this.escapeCooldown = 0;

    // Backtracking is forbidden unless this timer is active.
    this.allowBacktrackTimer = 0;
    this.recentCells = [];
    this.recentCellSet = new Set();
    this.recentCellMax = 90;
  }

  cellKey(c) {
    return `${c.cx},${c.cy}`;
  }

  updateRecentCells(robot) {
    const c = this.mapping.worldToCell(robot.x, robot.y);
    const k = this.cellKey(c);
    if (this.recentCellSet.has(k)) return;
    this.recentCells.push(k);
    this.recentCellSet.add(k);
    if (this.recentCells.length > this.recentCellMax) {
      const old = this.recentCells.shift();
      this.recentCellSet.delete(old);
    }
  }

  visitPenaltyAt(x, y) {
    const c = this.mapping.worldToCell(x, y);
    return this.mapping.visited[this.mapping.idx(c.cx, c.cy)] * 0.2;
  }

  chooseFrontierGoal(robot) {
    // Prefer true frontier cells with low local revisit pressure.
    let best = null;
    for (let y = 2; y < this.mapping.rows - 2; y++) {
      for (let x = 2; x < this.mapping.cols - 2; x++) {
        const i = this.mapping.idx(x, y);
        if (!this.mapping.explored[i] || this.mapping.obstacle[i]) continue;
        if (this.recentCellSet.has(`${x},${y}`)) continue;

        const neigh = [
          this.mapping.idx(x + 1, y),
          this.mapping.idx(x - 1, y),
          this.mapping.idx(x, y + 1),
          this.mapping.idx(x, y - 1),
        ];
        const bordersUnknown = neigh.some((n) => this.mapping.explored[n] === 0);
        if (!bordersUnknown) continue;

        // Local revisit density around candidate (3x3).
        let localVisits = 0;
        for (let dy = -1; dy <= 1; dy++) {
          for (let dx = -1; dx <= 1; dx++) {
            localVisits += this.mapping.visited[this.mapping.idx(x + dx, y + dy)];
          }
        }

        const center = this.mapping.cellCenter(x, y);
        const d = dist(robot, center);
        const score = (1 / (1 + d * 0.0085)) * (1 / (1 + localVisits * 0.2));
        if (!best || score > best.score) best = { x: center.x, y: center.y, score };
      }
    }
    return best;
  }

  chooseClusterGoal(robot, clusters) {
    if (!clusters.length) return null;
    let best = null;
    for (const c of clusters) {
      const d = dist(robot, c);
      const distancePenalty = 1 / (1 + d * 0.009);
      const visitPenalty = 1 / (1 + this.visitPenaltyAt(c.x, c.y));
      const score = c.confidence * distancePenalty * visitPenalty;
      if (!best || score > best.score) best = { ...c, score };
    }
    return best;
  }

  isBlocked(cx, cy) {
    if (!this.mapping.inBounds(cx, cy)) return true;
    for (let dy = -this.inflation; dy <= this.inflation; dy++) {
      for (let dx = -this.inflation; dx <= this.inflation; dx++) {
        const nx = cx + dx;
        const ny = cy + dy;
        if (!this.mapping.inBounds(nx, ny)) return true;
        if (this.mapping.obstacle[this.mapping.idx(nx, ny)] > 0) return true;
      }
    }
    return false;
  }

  planPath(robot) {
    if (!this.goal) {
      this.path = [];
      this.pathIndex = 0;
      return false;
    }

    const allowBacktrack = this.allowBacktrackTimer > 0;
    const start = this.mapping.worldToCell(robot.x, robot.y);
    const goalCell = this.mapping.worldToCell(this.goal.x, this.goal.y);

    const goalCandidates = [];
    for (let r = 0; r <= 4; r++) {
      for (let dy = -r; dy <= r; dy++) {
        for (let dx = -r; dx <= r; dx++) {
          const gx = goalCell.cx + dx;
          const gy = goalCell.cy + dy;
          if (!this.mapping.inBounds(gx, gy)) continue;
          const i = this.mapping.idx(gx, gy);
          if (!this.mapping.explored[i] || this.mapping.obstacle[i]) continue;
          if (this.isBlocked(gx, gy)) continue;
          goalCandidates.push({ cx: gx, cy: gy });
        }
      }
      if (goalCandidates.length) break;
    }

    if (!goalCandidates.length || this.isBlocked(start.cx, start.cy)) {
      this.path = [];
      this.pathIndex = 0;
      return false;
    }

    let selectedGoal = goalCandidates[0];
    let bestGoalD = Infinity;
    for (const g of goalCandidates) {
      const d = Math.abs(g.cx - start.cx) + Math.abs(g.cy - start.cy);
      if (d < bestGoalD) {
        bestGoalD = d;
        selectedGoal = g;
      }
    }

    const key = (c) => `${c.cx},${c.cy}`;
    const open = [start];
    const openSet = new Set([key(start)]);
    const came = new Map();
    const gScore = new Map([[key(start), 0]]);

    while (open.length) {
      let bestIdx = 0;
      let bestF = Infinity;
      for (let i = 0; i < open.length; i++) {
        const c = open[i];
        const gc = gScore.get(key(c)) ?? Infinity;
        const h = Math.abs(c.cx - selectedGoal.cx) + Math.abs(c.cy - selectedGoal.cy);
        const f = gc + h;
        if (f < bestF) {
          bestF = f;
          bestIdx = i;
        }
      }

      const current = open.splice(bestIdx, 1)[0];
      openSet.delete(key(current));

      if (current.cx === selectedGoal.cx && current.cy === selectedGoal.cy) {
        const rev = [current];
        let k = key(current);
        while (came.has(k)) {
          const p = came.get(k);
          rev.push(p);
          k = key(p);
        }
        rev.reverse();
        this.path = rev.map((c) => this.mapping.cellCenter(c.cx, c.cy));
        this.pathIndex = Math.min(1, this.path.length - 1);
        return true;
      }

      const nbrs = [
        { cx: current.cx + 1, cy: current.cy },
        { cx: current.cx - 1, cy: current.cy },
        { cx: current.cx, cy: current.cy + 1 },
        { cx: current.cx, cy: current.cy - 1 },
      ];

      for (const n of nbrs) {
        if (!this.mapping.inBounds(n.cx, n.cy)) continue;
        if (this.isBlocked(n.cx, n.cy)) continue;

        const ni = this.mapping.idx(n.cx, n.cy);
        if (!this.mapping.explored[ni]) continue;

        const nk = key(n);
        const visits = this.mapping.visited[ni];
        // Hard anti-revisit: only allow revisits when recovery has been enabled.
        if (!allowBacktrack && (visits > 0.05 || this.recentCellSet.has(nk))) continue;

        const revisitCost = allowBacktrack ? visits * 0.25 : visits * 4.2;
        const tentative = (gScore.get(key(current)) ?? Infinity) + 1 + revisitCost;
        if (tentative < (gScore.get(nk) ?? Infinity)) {
          came.set(nk, current);
          gScore.set(nk, tentative);
          if (!openSet.has(nk)) {
            open.push(n);
            openSet.add(nk);
          }
        }
      }
    }

    this.path = [];
    this.pathIndex = 0;
    return false;
  }

  currentWaypointVector(robot) {
    if (!this.path.length || this.pathIndex >= this.path.length) return null;

    let wp = this.path[this.pathIndex];
    while (wp && dist(robot, wp) < 10 && this.pathIndex < this.path.length - 1) {
      this.pathIndex++;
      wp = this.path[this.pathIndex];
    }

    if (!wp) return null;
    const v = { x: wp.x - robot.x, y: wp.y - robot.y };
    if (vLen(v) < 1e-4) return null;
    return vNorm(v);
  }

  computeRepulsion(robot) {
    const u = robot.ultra;
    const maxR = CONFIG.nav.repulsionRange;
    const beams = [
      { d: u.front, a: robot.theta },
      { d: u.left, a: robot.theta + Math.PI / 4 },
      { d: u.right, a: robot.theta - Math.PI / 4 },
    ];

    let total = { x: 0, y: 0 };
    for (const b of beams) {
      if (b.d > maxR) continue;
      const mag = (1 - b.d / maxR) ** 2;
      const away = angleToVec(b.a + Math.PI);
      total = vAdd(total, vScale(away, mag));
    }
    return total;
  }

  computeRadarVector(robot) {
    if (!robot.radar.detected) return { x: 0, y: 0 };
    const a = robot.theta + robot.radar.direction;
    return vScale(angleToVec(a), robot.radar.signal);
  }

  computeGoalVector(robot) {
    const wp = this.currentWaypointVector(robot);
    if (wp) return wp;

    if (!this.goal) return { x: 0, y: 0 };
    const v = { x: this.goal.x - robot.x, y: this.goal.y - robot.y };
    const l = vLen(v);
    if (l < 10) return { x: 0, y: 0 };
    return vScale(vNorm(v), clamp(l / 120, 0.25, 1));
  }

  potentialField(robot, state) {
    const goal = this.computeGoalVector(robot);
    const radar = this.computeRadarVector(robot);
    const repulsion = this.computeRepulsion(robot);

    const m = vNorm(robot.momentumVec);
    const momentum = vScale(m, 1);

    let force = { x: 0, y: 0 };
    force = vAdd(force, vScale(goal, CONFIG.nav.goalWeight));
    force = vAdd(force, vScale(radar, state === "TRACK" ? CONFIG.nav.radarWeight * 1.4 : CONFIG.nav.radarWeight));

    const repWeight = this.stuckBurst > 0 ? CONFIG.nav.repulsionWeight * 0.45 : CONFIG.nav.repulsionWeight;
    force = vAdd(force, vScale(repulsion, repWeight));
    force = vAdd(force, vScale(momentum, CONFIG.nav.momentumWeight));

    // Do not inject random steering unless recovering/stuck.
    if (this.stuckBurst > 0) {
      force = vAdd(force, {
        x: (this.rng() - 0.5) * CONFIG.nav.randomNoise,
        y: (this.rng() - 0.5) * CONFIG.nav.randomNoise,
      });
    }

    if (vLen(force) < 1e-4) force = this.lastForce;
    this.lastForce = vNorm(force);
    return force;
  }

  updateGoal(robot, state, clusters, dt) {
    this.goalTimer += dt;
    const needRefresh = !this.goal || this.goalTimer > 1.2 || dist(robot, this.goal) < 20;
    if (!needRefresh) return;

    this.goalTimer = 0;
    let goal = null;

    if (state === "TRACK" || state === "CONFIRM") {
      goal = this.chooseClusterGoal(robot, clusters);
    }

    if (!goal) goal = this.chooseFrontierGoal(robot);
    if (!goal) {
      goal = {
        x: clamp(robot.x + (this.rng() - 0.5) * 120, 30, CONFIG.world.width - 30),
        y: clamp(robot.y + (this.rng() - 0.5) * 120, 30, CONFIG.world.height - 30),
      };
    }
    this.goal = goal;
    this.pathTimer = 1e9;
  }

  computeControl(robot, state, dt) {
    const force = this.potentialField(robot, state);
    const targetAngle = vecToAngle(force);
    const err = normAngle(targetAngle - robot.imu.theta);

    const pid = CONFIG.control.pid;
    robot.pidIntegral = clamp(robot.pidIntegral + err * dt, -1.5, 1.5);
    const deriv = (err - robot.pidPrevErr) / Math.max(dt, 1e-6);
    robot.pidPrevErr = err;

    let angular = pid.kp * err + pid.ki * robot.pidIntegral + pid.kd * deriv;

    const front = robot.ultra.front;
    const left = robot.ultra.left;
    const right = robot.ultra.right;

    const near = Math.min(front, left, right);
    if (near < 16) {
      angular += left > right ? 2.2 : -2.2;
    } else if (front < CONFIG.robot.safeDist) {
      angular += left > right ? 1.7 : -1.7;
    }

    const turnFactor = clamp(1 - Math.abs(err) / (Math.PI * 0.8), 0.1, 1);
    const clearanceFactor = clamp((front - 10) / 45, 0.08, 1);
    let linear = CONFIG.robot.baseSpeed * turnFactor * clearanceFactor;
    if (Math.min(left, right) < 18) linear *= 0.7;
    if (front < CONFIG.robot.safeDist) linear *= 0.25;
    if (near < 16) linear *= 0.1;

    if (state === "CONFIRM") {
      linear *= 0.22;
      angular += 0.85;
    }

    return { linear, angular, err };
  }

  step(robot, state, clusters, dt) {
    this.updateRecentCells(robot);

    if (robot.stuckTime > CONFIG.nav.stuckTimeout) {
      this.stuckBurst = 1.3;
      this.goalTimer = 1e9;
      this.pathTimer = 1e9;
      this.allowBacktrackTimer = 1.8;
    }
    this.stuckBurst = Math.max(0, this.stuckBurst - dt);
    this.allowBacktrackTimer = Math.max(0, this.allowBacktrackTimer - dt);

    this.escapeCooldown = Math.max(0, this.escapeCooldown - dt);

    const closeCount = (robot.ultra.front < 30 ? 1 : 0) + (robot.ultra.left < 24 ? 1 : 0) + (robot.ultra.right < 24 ? 1 : 0);
    const boxedTrap = closeCount >= 2 && robot.stuckTime > 0.85;

    if (this.escapePhase === 0 && this.escapeCooldown <= 0 && boxedTrap) {
      this.escapeTurnSign = robot.ultra.right > robot.ultra.left ? -1 : 1;
      this.escapePhase = 1;
      this.escapeTimer = 0.28;
      this.path = [];
      this.pathIndex = 0;
      this.goalTimer = 1e9;
      this.pathTimer = 1e9;
      this.allowBacktrackTimer = 1.5;
    }

    if (this.escapePhase === 1) {
      this.escapeTimer -= dt;
      if (this.escapeTimer <= 0) {
        this.escapePhase = 2;
        this.escapeTimer = 0.48;
      }
      return { linear: -20, angular: 0, err: 0, recovering: true };
    }

    if (this.escapePhase === 2) {
      this.escapeTimer -= dt;
      if (this.escapeTimer <= 0) {
        this.escapePhase = 0;
        this.escapeCooldown = 1.0;
      }
      return { linear: 8, angular: this.escapeTurnSign * 2.0, err: 0, recovering: true };
    }

    this.updateGoal(robot, state, clusters, dt);

    this.pathTimer += dt;
    if (this.pathTimer > 0.7 || !this.path.length || this.pathIndex >= this.path.length) {
      let ok = this.planPath(robot);
      if (!ok && this.allowBacktrackTimer <= 0) {
        // Temporarily permit backtracking only when forward-only plan fails.
        this.allowBacktrackTimer = 1.4;
        ok = this.planPath(robot);
      }
      this.pathTimer = 0;
      if (!ok) this.goalTimer = 1e9;
    }

    const ctrl = this.computeControl(robot, state, dt);

    if (this.stuckBurst > 0) {
      ctrl.linear *= 0.8;
      ctrl.angular += (this.rng() - 0.5) * 0.5;
    }

    ctrl.recovering = false;
    return ctrl;
  }
}

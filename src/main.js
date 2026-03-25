import { CONFIG, clamp } from "./config.js";
import { Environment } from "./environment.js";
import { Logger } from "./logger.js";
import { MappingSystem } from "./mapping.js";
import { Navigation } from "./navigation.js";
import { ReplayBuffer } from "./replay.js";
import { Robot } from "./robot.js";
import { SensorSuite } from "./sensors.js";
import { BehaviorStateMachine } from "./stateMachine.js";
import { Renderer } from "./renderer.js";
import { mulberry32 } from "./utils.js";

class ArgusSim {
  constructor() {
    this.worldCanvas = document.getElementById("worldView");
    this.mapCanvas = document.getElementById("mapView");
    this.logEl = document.getElementById("logOutput");

    this.hud = {
      mode: document.getElementById("hudMode"),
      signal: document.getElementById("hudSignal"),
      maxProb: document.getElementById("hudMaxProb"),
      found: document.getElementById("hudFound"),
      gps: document.getElementById("hudGps"),
      coverage: document.getElementById("hudCoverage"),
      score: document.getElementById("hudScore"),
      time: document.getElementById("hudTime"),
    };

    this.btn = {
      start: document.getElementById("btnStart"),
      pause: document.getElementById("btnPause"),
      reset: document.getElementById("btnReset"),
      replay: document.getElementById("btnReplay"),
      clearLogs: document.getElementById("btnClearLogs"),
      speed: document.getElementById("speedCtrl"),
      scrub: document.getElementById("scrubCtrl"),
    };

    this.resetAll(20260325);
    this.attachEvents();
    requestAnimationFrame((t) => {
      this.lastTs = t;
      this.loop(t);
    });
  }

  resetAll(seed = Math.floor(Math.random() * 1e9)) {
    this.rng = mulberry32(seed);
    this.env = new Environment(this.rng);
    this.robot = new Robot();
    this.sensors = new SensorSuite(this.rng);
    this.mapping = new MappingSystem();
    this.nav = new Navigation(this.rng, this.mapping);
    this.machine = new BehaviorStateMachine();
    this.replay = new ReplayBuffer();
    this.logger = new Logger();
    this.renderer = new Renderer(this.worldCanvas, this.mapCanvas);

    this.simTime = 0;
    this.speed = 1;
    this.liveMode = "LIVE";
    this.paused = true;
    this.trail = [];
    this.confirmed = new Set();
    this.ignoredHotspots = [];
    this.trackWatch = {
      targetKey: null,
      bestDist: Infinity,
      noImprove: 0,
    };
    this.lastLogTimes = { radar: -99, nav: -99, map: -99, ctrl: -99 };

    this.logger.push("SYS", `Session initialized seed=${seed}`, this.simTime);
    this.updateHud();
    this.render();
  }

  attachEvents() {
    this.btn.start.addEventListener("click", () => {
      this.paused = false;
      this.logger.push("SYS", "Mission started", this.simTime);
    });

    this.btn.pause.addEventListener("click", () => {
      this.paused = !this.paused;
      this.logger.push("SYS", this.paused ? "Paused" : "Resumed", this.simTime);
    });

    this.btn.reset.addEventListener("click", () => this.resetAll());

    this.btn.replay.addEventListener("click", () => {
      this.liveMode = this.liveMode === "LIVE" ? "REPLAY" : "LIVE";
      this.logger.push("SYS", `Mode => ${this.liveMode}`, this.simTime);
      if (this.liveMode === "REPLAY") {
        this.paused = true;
        if (this.replay.frames.length) this.replay.setCursor(this.replay.frames.length - 1);
      } else if (this.replay.frames.length) {
        this.restoreFromFrame(this.replay.frames[this.replay.frames.length - 1]);
      }
      this.updateHud();
    });

    this.btn.clearLogs.addEventListener("click", () => {
      this.logger.clear();
      this.logger.push("SYS", "Logs cleared", this.simTime);
      this.flushLogs();
    });

    this.btn.speed.addEventListener("input", (e) => {
      this.speed = Number(e.target.value);
    });

    this.btn.scrub.addEventListener("input", (e) => {
      const t = Number(e.target.value);
      if (!this.replay.frames.length) return;
      const idx = Math.floor((t / 100) * (this.replay.frames.length - 1));
      this.replay.setCursor(idx);
      if (this.liveMode === "REPLAY") this.renderReplayFrame();
    });

    window.addEventListener("keydown", (e) => {
      if (e.code === "Space") {
        e.preventDefault();
        this.paused = !this.paused;
      }
      if (e.key === "r" || e.key === "R") this.resetAll();
      if (e.key === "p" || e.key === "P") {
        this.liveMode = this.liveMode === "LIVE" ? "REPLAY" : "LIVE";
        if (this.liveMode === "REPLAY") {
          this.paused = true;
          if (this.replay.frames.length) this.replay.setCursor(this.replay.frames.length - 1);
        } else if (this.replay.frames.length) {
          this.restoreFromFrame(this.replay.frames[this.replay.frames.length - 1]);
        }
      }
      if (e.key === "ArrowLeft" && this.liveMode === "REPLAY") {
        this.replay.stepCursor(-2);
        this.renderReplayFrame();
      }
      if (e.key === "ArrowRight" && this.liveMode === "REPLAY") {
        this.replay.stepCursor(2);
        this.renderReplayFrame();
      }
      if (e.key === "c" || e.key === "C") {
        this.logger.clear();
      }
      if (e.key === "Escape") {
        this.paused = true;
        this.logger.push("SYS", "Simulation halted (ESC)", this.simTime);
      }
    });
  }

  detectSurvivors(clusters) {
    for (const c of clusters) {
      if (c.confidence < 0.72) continue;
      for (let i = 0; i < this.env.survivors.length; i++) {
        const s = this.env.survivors[i];
        const d = Math.hypot(s.x - c.x, s.y - c.y);
        if (d < 38 && !this.confirmed.has(i)) {
          this.confirmed.add(i);
          s.confirmed = true;
          this.logger.push("RADAR", `Survivor cluster confirmed #${i + 1}`, this.simTime);
        }
      }
    }
  }

  getActiveClusters(allClusters) {
    this.ignoredHotspots = this.ignoredHotspots.filter((z) => z.until > this.simTime);
    if (!allClusters.length) return allClusters;
    const confirmedSurvivors = this.env.survivors.filter((s) => s.confirmed);
    const hasAnyFilters = confirmedSurvivors.length > 0 || this.ignoredHotspots.length > 0;
    if (!hasAnyFilters) return allClusters;

    return allClusters.filter((c) => {
      for (const s of confirmedSurvivors) {
        const d = Math.hypot(c.x - s.x, c.y - s.y);
        if (d < 90) return false;
      }
      for (const z of this.ignoredHotspots) {
        const d = Math.hypot(c.x - z.x, c.y - z.y);
        if (d < z.r) return false;
      }
      return true;
    });
  }

  updateTrackWatch(state, activeClusters, dt) {
    if (state !== "TRACK" && state !== "CONFIRM") {
      this.trackWatch.targetKey = null;
      this.trackWatch.bestDist = Infinity;
      this.trackWatch.noImprove = 0;
      return;
    }

    const best = activeClusters.length ? activeClusters[0] : null;
    if (!best) {
      this.trackWatch.noImprove += dt;
      return;
    }

    const key = `${Math.round(best.x / 18)},${Math.round(best.y / 18)}`;
    const d = Math.hypot(best.x - this.robot.x, best.y - this.robot.y);
    if (this.trackWatch.targetKey !== key) {
      this.trackWatch.targetKey = key;
      this.trackWatch.bestDist = d;
      this.trackWatch.noImprove = 0;
      return;
    }

    if (d < this.trackWatch.bestDist - 3) {
      this.trackWatch.bestDist = d;
      this.trackWatch.noImprove = 0;
      return;
    }

    this.trackWatch.noImprove += dt;
    const trapped = this.trackWatch.noImprove > 3.2 || (this.robot.stuckTime > 1.0 && d < 110);
    if (!trapped) return;

    this.ignoredHotspots.push({
      x: best.x,
      y: best.y,
      r: 88,
      until: this.simTime + 16,
    });
    this.mapping.dampZone(best.x, best.y, 92, 0.55);
    this.machine.setState("EXPLORE");
    this.nav.goalTimer = 1e9;
    this.nav.path = [];
    this.nav.pathIndex = 0;
    this.trackWatch.targetKey = null;
    this.trackWatch.bestDist = Infinity;
    this.trackWatch.noImprove = 0;
    this.logger.push("NAV", "Hotspot trap detected: abandoning target and resuming exploration", this.simTime);
  }

  score() {
    const survivors = this.confirmed.size;
    const coverage = this.mapping.coverage;
    return Math.max(
      0,
      CONFIG.scoring.wSurvivor * survivors - CONFIG.scoring.wTime * this.simTime + CONFIG.scoring.wCoverage * coverage
    );
  }

  updateHud() {
    this.hud.mode.textContent = `${this.liveMode}:${this.machine.state}`;
    this.hud.signal.textContent = this.robot.radar.signal.toFixed(2);
    this.hud.maxProb.textContent = this.mapping.maxProb.toFixed(2);
    this.hud.found.textContent = `${this.confirmed.size}/${this.env.survivors.length}`;
    this.hud.gps.textContent = `${this.robot.gps.x.toFixed(1)}, ${this.robot.gps.y.toFixed(1)}`;
    this.hud.coverage.textContent = `${(this.mapping.coverage * 100).toFixed(1)}%`;
    this.hud.score.textContent = this.score().toFixed(1);
    this.hud.time.textContent = `${this.simTime.toFixed(1)}s`;
  }

  flushLogs() {
    this.logEl.textContent = this.logger.text();
    this.logEl.scrollTop = this.logEl.scrollHeight;
  }

  addPeriodicLogs(clusters, ctrl) {
    if (this.simTime - this.lastLogTimes.nav > 1.4) {
      const g = this.nav.goal ? `${this.nav.goal.x.toFixed(0)},${this.nav.goal.y.toFixed(0)}` : "none";
      this.logger.push("NAV", `state=${this.machine.state} goal=${g}`, this.simTime);
      this.lastLogTimes.nav = this.simTime;
    }
    if (this.simTime - this.lastLogTimes.radar > 1.0) {
      this.logger.push(
        "RADAR",
        `signal=${this.robot.radar.signal.toFixed(2)} dir=${this.robot.radar.direction.toFixed(2)} clusters=${clusters.length}`,
        this.simTime
      );
      this.lastLogTimes.radar = this.simTime;
    }
    if (this.simTime - this.lastLogTimes.map > 2.1) {
      this.logger.push(
        "MAP",
        `coverage=${(this.mapping.coverage * 100).toFixed(1)} max_prob=${this.mapping.maxProb.toFixed(2)}`,
        this.simTime
      );
      this.lastLogTimes.map = this.simTime;
    }
    if (this.simTime - this.lastLogTimes.ctrl > 1.1) {
      this.logger.push(
        "CTRL",
        `v=${ctrl.linear.toFixed(1)} w=${ctrl.angular.toFixed(2)} ultraf=${this.robot.ultra.front.toFixed(1)}`,
        this.simTime
      );
      this.lastLogTimes.ctrl = this.simTime;
    }
  }

  render() {
    const clusters = this.mapping.clusterHighProb();
    this.renderer.drawWorld(this.env, this.robot, this.nav, this.trail, this.simTime);
    this.renderer.drawMap(this.mapping, clusters, this.trail, this.env, this.simTime);
    this.updateHud();
    this.flushLogs();

    if (this.replay.frames.length > 1) {
      const t = (this.replay.cursor / (this.replay.frames.length - 1)) * 100;
      this.btn.scrub.value = t.toFixed(1);
    }
  }

  renderReplayFrame() {
    const f = this.replay.current();
    if (!f) return;

    this.restoreFromFrame(f);

    this.updateHud();
    const clusters = this.mapping.clusterHighProb();
    this.renderer.drawWorld(this.env, this.robot, this.nav, this.trail, this.simTime);
    this.renderer.drawMap(this.mapping, clusters, this.trail, this.env, this.simTime);
  }

  restoreFromFrame(f) {
    this.robot.x = f.robot.x;
    this.robot.y = f.robot.y;
    this.robot.theta = f.robot.theta;
    this.robot.ultra = { ...f.robot.ultra };
    this.robot.radar = { ...f.robot.radar };
    this.robot.gps = { ...f.robot.gps };

    this.mapping.visited.set(f.maps.visited);
    this.mapping.explored.set(f.maps.explored);
    this.mapping.obstacle.set(f.maps.obstacle);
    this.mapping.prob.set(f.maps.prob);
    this.mapping.coverage = f.maps.coverage;
    this.mapping.maxProb = f.maps.maxProb;
    this.trail = f.trail.slice();

    this.confirmed = new Set(f.confirmedIndices || []);
    for (let i = 0; i < this.env.survivors.length; i++) {
      this.env.survivors[i].confirmed = this.confirmed.has(i);
    }
  }

  step(dtRaw) {
    const dt = clamp(dtRaw * this.speed, 0, 0.04);
    this.simTime += dt;

    this.sensors.sampleAll(this.robot, this.env, this.simTime, dt);
    this.mapping.step(this.robot, this.env, dt);

    const clusters = this.mapping.clusterHighProb();
    const activeClusters = this.getActiveClusters(clusters);
    const state = this.machine.update(this.robot, activeClusters, this.liveMode, dt);
    this.updateTrackWatch(state, activeClusters, dt);

    const ctrl = this.nav.step(this.robot, state, activeClusters, dt);

    // Wall-follow behavior shaping (phase 1)
    if (!ctrl.recovering && state === "WALL_FOLLOW") {
      const wallErr = CONFIG.nav.wallDistTarget - this.robot.ultra.left;
      if (this.robot.ultra.left < 16) {
        // Too close to left wall: force right turn and slow crawl.
        ctrl.angular -= 1.4;
        ctrl.linear *= 0.15;
      } else if (this.robot.ultra.left > 78) {
        // Lost wall: reacquire gently without charging forward.
        ctrl.angular += 0.85;
        ctrl.linear *= 0.45;
      } else {
        ctrl.angular += wallErr * 0.03;
        ctrl.linear *= 0.8;
      }

      if (this.robot.ultra.front < CONFIG.robot.safeDist) {
        ctrl.angular += this.robot.ultra.right > this.robot.ultra.left ? -1.2 : 1.2;
        ctrl.linear *= 0.12;
      }
    }

    // Track shaping (phase 3)
    if (!ctrl.recovering && state === "TRACK") {
      const bestCluster = activeClusters.length ? activeClusters[0] : null;
      if (this.robot.radar.detected) {
        ctrl.angular += this.robot.radar.direction * 0.9;
        ctrl.linear *= 0.72;
      } else if (bestCluster && bestCluster.confidence > 0.62) {
        const targetAngle = Math.atan2(bestCluster.y - this.robot.y, bestCluster.x - this.robot.x);
        let err = targetAngle - this.robot.theta;
        while (err > Math.PI) err -= Math.PI * 2;
        while (err < -Math.PI) err += Math.PI * 2;
        ctrl.angular += err * 0.85;
        ctrl.linear *= 0.62;
      }
    }

    this.robot.targetLinear = ctrl.linear;
    this.robot.targetAngular = ctrl.angular;
    this.robot.updatePose(dt, this.env);

    this.robot.momentumVec = {
      x: Math.cos(this.robot.theta) * Math.max(0.2, Math.abs(this.robot.linearVelocity)),
      y: Math.sin(this.robot.theta) * Math.max(0.2, Math.abs(this.robot.linearVelocity)),
    };

    this.detectSurvivors(activeClusters);
    this.trail.push({ x: this.robot.x, y: this.robot.y });
    if (this.trail.length > 900) this.trail.shift();

    this.addPeriodicLogs(activeClusters, ctrl);

    this.replay.record(dt, {
      robot: {
        x: this.robot.x,
        y: this.robot.y,
        theta: this.robot.theta,
        ultra: { ...this.robot.ultra },
        radar: { ...this.robot.radar },
        gps: { ...this.robot.gps },
      },
      maps: {
        visited: this.mapping.visited.slice(),
        explored: this.mapping.explored.slice(),
        obstacle: this.mapping.obstacle.slice(),
        prob: this.mapping.prob.slice(),
        coverage: this.mapping.coverage,
        maxProb: this.mapping.maxProb,
      },
      trail: this.trail.slice(-350),
      confirmedIndices: Array.from(this.confirmed.values()),
    });

    this.render();
  }

  loop(ts) {
    const dt = (ts - this.lastTs) / 1000;
    this.lastTs = ts;

    if (this.liveMode === "REPLAY") {
      if (!this.paused) {
        this.replay.stepCursor(1);
        this.renderReplayFrame();
      }
      this.flushLogs();
      requestAnimationFrame((t) => this.loop(t));
      return;
    }

    if (!this.paused) this.step(dt);
    else this.render();

    requestAnimationFrame((t) => this.loop(t));
  }
}

new ArgusSim();

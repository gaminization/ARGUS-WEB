import { CONFIG, clamp, normAngle } from "./config.js";
import { angleToVec } from "./utils.js";

export class SensorSuite {
  constructor(rng) {
    this.rng = rng;
    this.radarQueue = [];
    this.gpsTimer = 0;
  }

  sampleUltrasonic(robot, env) {
    const cfg = CONFIG.sensors.ultrasonic;
    for (const [name, a] of Object.entries(cfg.angles)) {
      const jitter = (this.rng() - 0.5) * cfg.jitter;
      const ray = env.raycast(robot.x, robot.y, robot.theta + a + jitter, cfg.maxRange, 2);
      robot.ultra[name] = clamp(ray.dist + (this.rng() - 0.5) * cfg.noise, 0, cfg.maxRange);
    }
  }

  sampleRadar(robot, env, simTime) {
    const cfg = CONFIG.sensors.radar;
    let signal = 0;
    let weightedDir = { x: 0, y: 0 };

    for (const s of env.survivors) {
      if (s.confirmed) continue;
      const dx = s.x - robot.x;
      const dy = s.y - robot.y;
      const d = Math.hypot(dx, dy);
      if (d > cfg.range) continue;

      const rel = normAngle(Math.atan2(dy, dx) - robot.theta);
      if (Math.abs(rel) > cfg.fov * 0.5) continue;

      const breath = 0.6 + 0.4 * Math.sin(simTime * 2 * Math.PI * s.breathingHz + s.phase);
      const atten = Math.exp(-(d * d) / (2 * 70 * 70));
      const part = atten * breath;
      signal += part;

      const v = angleToVec(rel + robot.theta);
      weightedDir.x += v.x * part;
      weightedDir.y += v.y * part;
    }

    if (this.rng() < cfg.falsePositiveRate) {
      signal = Math.max(signal, 0.2 + this.rng() * 0.35);
      const fake = robot.theta + (this.rng() - 0.5) * Math.PI;
      weightedDir = angleToVec(fake);
    }

    signal = clamp(signal + (this.rng() - 0.5) * cfg.noiseStrength, 0, 1);
    let direction = 0;
    if (Math.hypot(weightedDir.x, weightedDir.y) > 1e-3) {
      direction = normAngle(Math.atan2(weightedDir.y, weightedDir.x) - robot.theta + (this.rng() - 0.5) * cfg.noiseDir);
    } else {
      direction = (this.rng() - 0.5) * Math.PI;
    }

    this.radarQueue.push({
      t: simTime + cfg.delaySec,
      signal,
      direction,
      detected: signal > 0.14,
    });

    while (this.radarQueue.length > 0 && this.radarQueue[0].t <= simTime) {
      const r = this.radarQueue.shift();
      robot.radar.signal = r.signal;
      robot.radar.direction = r.direction;
      robot.radar.detected = r.detected;
    }
  }

  sampleImu(robot) {
    robot.imu.theta = normAngle(robot.theta + (this.rng() - 0.5) * CONFIG.sensors.imu.thetaNoise);
  }

  sampleGps(robot, dt) {
    this.gpsTimer += dt;
    if (this.gpsTimer < CONFIG.sensors.gps.interval) return;
    this.gpsTimer = 0;
    const n = CONFIG.sensors.gps.noise;
    robot.gps.x = robot.x + (this.rng() - 0.5) * n;
    robot.gps.y = robot.y + (this.rng() - 0.5) * n;
  }

  sampleAll(robot, env, simTime, dt) {
    this.sampleUltrasonic(robot, env);
    this.sampleRadar(robot, env, simTime);
    this.sampleImu(robot);
    this.sampleGps(robot, dt);
  }
}

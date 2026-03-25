import { CONFIG, clamp, normAngle } from "./config.js";

export class Robot {
  constructor() {
    this.x = 90;
    this.y = 90;
    this.theta = 0.2;

    this.linearVelocity = 0;
    this.angularVelocity = 0;

    this.targetLinear = 0;
    this.targetAngular = 0;

    this.pidIntegral = 0;
    this.pidPrevErr = 0;
    this.momentumVec = { x: 1, y: 0 };

    this.ultra = { front: 0, left: 0, right: 0 };
    this.radar = { signal: 0, direction: 0, detected: false };
    this.imu = { theta: this.theta };
    this.gps = { x: this.x, y: this.y };

    this.stuckTime = 0;
    this.lastPos = { x: this.x, y: this.y };
  }

  updatePose(dt, env) {
    const alpha = 1 - CONFIG.control.smoothing;
    this.linearVelocity = this.linearVelocity * CONFIG.control.smoothing + this.targetLinear * alpha;
    this.angularVelocity = this.angularVelocity * CONFIG.control.smoothing + this.targetAngular * alpha;

    this.linearVelocity = clamp(this.linearVelocity, -CONFIG.robot.maxLinear, CONFIG.robot.maxLinear);
    this.angularVelocity = clamp(this.angularVelocity, -CONFIG.robot.maxAngular, CONFIG.robot.maxAngular);

    const nx = this.x + Math.cos(this.theta) * this.linearVelocity * dt;
    const ny = this.y + Math.sin(this.theta) * this.linearVelocity * dt;
    const nt = normAngle(this.theta + this.angularVelocity * dt);

    if (!env.collidesCircle(nx, ny, CONFIG.robot.radius)) {
      this.x = nx;
      this.y = ny;
    } else {
      this.linearVelocity *= 0.2;
      this.angularVelocity += 0.6;
    }

    this.theta = nt;

    const moved = Math.hypot(this.x - this.lastPos.x, this.y - this.lastPos.y);
    if (moved < 0.6) this.stuckTime += dt;
    else this.stuckTime = Math.max(0, this.stuckTime - dt * 0.5);

    this.lastPos.x = this.x;
    this.lastPos.y = this.y;
  }
}

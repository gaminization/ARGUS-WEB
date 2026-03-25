import { CONFIG } from "./config.js";

export class ReplayBuffer {
  constructor() {
    this.frames = [];
    this.accum = 0;
    this.cursor = 0;
  }

  clear() {
    this.frames = [];
    this.accum = 0;
    this.cursor = 0;
  }

  record(dt, snapshot) {
    this.accum += dt;
    if (this.accum < CONFIG.replay.dt) return;
    this.accum = 0;

    this.frames.push(snapshot);
    if (this.frames.length > CONFIG.replay.maxFrames) this.frames.shift();
    this.cursor = this.frames.length - 1;
  }

  setCursor(i) {
    this.cursor = Math.max(0, Math.min(this.frames.length - 1, i));
  }

  stepCursor(delta) {
    this.setCursor(this.cursor + delta);
  }

  current() {
    if (!this.frames.length) return null;
    return this.frames[this.cursor];
  }
}

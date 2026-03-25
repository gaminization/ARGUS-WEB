import { timestamp } from "./utils.js";

export class Logger {
  constructor(limit = 260) {
    this.limit = limit;
    this.lines = [];
  }

  push(tag, msg, simTime) {
    const line = `[${timestamp(simTime)}] [${tag}] ${msg}`;
    this.lines.push(line);
    if (this.lines.length > this.limit) this.lines.shift();
  }

  clear() {
    this.lines = [];
  }

  text() {
    return this.lines.join("\n");
  }
}

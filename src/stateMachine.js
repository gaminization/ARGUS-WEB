export class BehaviorStateMachine {
  constructor() {
    this.state = "EXPLORE";
    this.stateTime = 0;
    this.foundCount = 0;
    this.lowSignalTime = 0;
  }

  setState(next) {
    if (next === this.state) return;
    this.state = next;
    this.stateTime = 0;
    this.lowSignalTime = 0;
  }

  update(robot, clusters, mode, dt) {
    if (mode === "REPLAY") {
      this.setState("REPLAY");
      this.stateTime += dt;
      return this.state;
    }

    this.stateTime += dt;

    const frontClose = robot.ultra.front < 34;
    const strongRadar = robot.radar.signal > 0.48;
    const mediumRadar = robot.radar.signal > 0.26;
    const bestCluster = clusters.length ? clusters[0].confidence : 0;
    const strongProb = bestCluster > 0.66;
    const mediumProb = bestCluster > 0.52;
    if (robot.radar.signal < 0.16) this.lowSignalTime += dt;
    else this.lowSignalTime = Math.max(0, this.lowSignalTime - dt * 0.5);
    const hasCluster = clusters.length > 0;

    if (this.state === "WALL_FOLLOW") {
      if (this.stateTime > 2.8 && !frontClose) this.setState("EXPLORE");
      if ((strongRadar || strongProb) && this.stateTime > 1.2) this.setState("TRACK");
      return this.state;
    }

    if (this.state === "EXPLORE") {
      if (frontClose && this.stateTime > 1.0) this.setState("WALL_FOLLOW");
      if ((strongRadar || strongProb) && this.stateTime > 1.4) this.setState("TRACK");
      return this.state;
    }

    if (this.state === "TRACK") {
      if (this.stateTime > 9.0) this.setState("EXPLORE");
      if ((!robot.radar.detected && !mediumProb) || this.lowSignalTime > 2.8) {
        if (this.stateTime > 2.0) this.setState("EXPLORE");
      }
      if (hasCluster && (mediumRadar || mediumProb) && this.stateTime > 2.0) this.setState("CONFIRM");
      if (frontClose && this.stateTime > 1.5) this.setState("WALL_FOLLOW");
      return this.state;
    }

    if (this.state === "CONFIRM") {
      if (this.stateTime > 3.5) {
        if (robot.radar.signal > 0.52) this.foundCount += 1;
        this.setState(robot.radar.detected ? "TRACK" : "EXPLORE");
      }
      return this.state;
    }

    return this.state;
  }
}

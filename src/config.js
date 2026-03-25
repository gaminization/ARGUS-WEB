export const CONFIG = {
  world: {
    width: 800,
    height: 800,
    mapCell: 8,
  },
  robot: {
    radius: 10,
    wheelBase: 24,
    baseSpeed: 58,
    maxLinear: 78,
    maxAngular: 3.4,
    safeDist: 32,
  },
  sensors: {
    ultrasonic: {
      maxRange: 120,
      jitter: 1.2,
      noise: 2.4,
      angles: {
        front: 0,
        left: Math.PI / 4,
        right: -Math.PI / 4,
      },
    },
    radar: {
      range: 180,
      fov: Math.PI * 0.95,
      delaySec: 0.24,
      noiseStrength: 0.06,
      noiseDir: 0.18,
      falsePositiveRate: 0.012,
    },
    imu: {
      thetaNoise: 0.015,
    },
    gps: {
      interval: 0.5,
      noise: 4.5,
    },
  },
  mapping: {
    radarGain: 0.09,
    decay: 0.999,
    blurInterval: 0.24,
    clusterThreshold: 0.56,
  },
  nav: {
    repulsionRange: 84,
    goalWeight: 1.2,
    radarWeight: 1.0,
    repulsionWeight: 2.3,
    momentumWeight: 0.65,
    randomNoise: 0.08,
    stuckTimeout: 3.2,
    wallDistTarget: 34,
  },
  control: {
    pid: {
      kp: 2.45,
      ki: 0.2,
      kd: 0.42,
    },
    smoothing: 0.78,
  },
  replay: {
    dt: 0.08,
    maxFrames: 14000,
  },
  scoring: {
    wSurvivor: 350,
    wTime: 1.2,
    wCoverage: 180,
  },
};

export function clamp(v, min, max) {
  return Math.max(min, Math.min(max, v));
}

export function normAngle(a) {
  let x = a;
  while (x > Math.PI) x -= Math.PI * 2;
  while (x < -Math.PI) x += Math.PI * 2;
  return x;
}

export function lerp(a, b, t) {
  return a + (b - a) * t;
}

export function randRange(rng, min, max) {
  return min + (max - min) * rng();
}

export function mulberry32(seed) {
  return function () {
    let t = (seed += 0x6d2b79f5);
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export function vAdd(a, b) {
  return { x: a.x + b.x, y: a.y + b.y };
}

export function vScale(v, s) {
  return { x: v.x * s, y: v.y * s };
}

export function vLen(v) {
  return Math.hypot(v.x, v.y);
}

export function vNorm(v) {
  const l = Math.max(1e-9, Math.hypot(v.x, v.y));
  return { x: v.x / l, y: v.y / l };
}

export function angleToVec(a) {
  return { x: Math.cos(a), y: Math.sin(a) };
}

export function vecToAngle(v) {
  return Math.atan2(v.y, v.x);
}

export function dist(a, b) {
  return Math.hypot(a.x - b.x, a.y - b.y);
}

export function timestamp(seconds) {
  const m = Math.floor(seconds / 60);
  const s = (seconds % 60).toFixed(1).padStart(4, "0");
  return `${String(m).padStart(2, "0")}:${s}`;
}

// Lightweight vector / quaternion math used by the physics core.
// Vectors are plain {x,y,z}. Quaternions are {w,x,y,z}.
// World frame is Y-up: +X right, +Y up, +Z forward (Three.js convention).
// Body frame matches at zero attitude. Main rotor thrust is along body +Y.

export const v3 = (x = 0, y = 0, z = 0) => ({ x, y, z });
export const vAdd = (a, b) => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z });
export const vSub = (a, b) => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z });
export const vScale = (a, s) => ({ x: a.x * s, y: a.y * s, z: a.z * s });
export const vDot = (a, b) => a.x * b.x + a.y * b.y + a.z * b.z;
export const vCross = (a, b) => ({
  x: a.y * b.z - a.z * b.y,
  y: a.z * b.x - a.x * b.z,
  z: a.x * b.y - a.y * b.x,
});
export const vLen = (a) => Math.sqrt(vDot(a, a));
export const vNorm = (a) => {
  const L = vLen(a);
  return L > 1e-12 ? vScale(a, 1 / L) : v3();
};
export const vCopy = (a) => ({ x: a.x, y: a.y, z: a.z });
export const vSet = (out, a) => { out.x = a.x; out.y = a.y; out.z = a.z; return out; };

// Quaternion (w, x, y, z) representing a rotation. w=1, x=y=z=0 is identity.
export const qIdent = () => ({ w: 1, x: 0, y: 0, z: 0 });
export const qCopy = (q) => ({ w: q.w, x: q.x, y: q.y, z: q.z });

export const qNorm = (q) => {
  const n = Math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n < 1e-12) return qIdent();
  return { w: q.w / n, x: q.x / n, y: q.y / n, z: q.z / n };
};

export const qMul = (a, b) => ({
  w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
  y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
  z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
});

// Rotate a vector v by quaternion q (q * v * q^-1, expanded form).
export const qRotate = (q, v) => {
  // t = 2 * (q.xyz × v)
  const tx = 2 * (q.y * v.z - q.z * v.y);
  const ty = 2 * (q.z * v.x - q.x * v.z);
  const tz = 2 * (q.x * v.y - q.y * v.x);
  // result = v + q.w * t + q.xyz × t
  return {
    x: v.x + q.w * tx + (q.y * tz - q.z * ty),
    y: v.y + q.w * ty + (q.z * tx - q.x * tz),
    z: v.z + q.w * tz + (q.x * ty - q.y * tx),
  };
};

export const qInverse = (q) => ({ w: q.w, x: -q.x, y: -q.y, z: -q.z });

// Build quaternion from axis-angle.
export const qFromAxisAngle = (axis, angle) => {
  const a = vNorm(axis);
  const s = Math.sin(angle / 2);
  return { w: Math.cos(angle / 2), x: a.x * s, y: a.y * s, z: a.z * s };
};

// Build quaternion from intrinsic Tait-Bryan angles (roll, pitch, yaw).
// Here: roll=rotation about body +Z (forward), pitch=about body +X (right),
// yaw=about world +Y (up). Apply order: yaw, pitch, roll (intrinsic).
export const qFromEuler = (roll, pitch, yaw) => {
  const cy = Math.cos(yaw / 2), sy = Math.sin(yaw / 2);
  const cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2);
  const cr = Math.cos(roll / 2), sr = Math.sin(roll / 2);
  // Y (yaw) * X (pitch) * Z (roll)
  return qMul(qMul(
    { w: cy, x: 0, y: sy, z: 0 },
    { w: cp, x: sp, y: 0, z: 0 }),
    { w: cr, x: 0, y: 0, z: sr }
  );
};

// Recover roll/pitch/yaw from quaternion. Uses the same convention as qFromEuler:
// rotation about body +Z (roll) inside body +X (pitch) inside world +Y (yaw).
export const qToEuler = (q) => {
  // Forward axis (body +Z) in world frame:
  const fwd = qRotate(q, v3(0, 0, 1));
  // Right axis (body +X) in world frame:
  const right = qRotate(q, v3(1, 0, 0));

  // Yaw: heading of forward axis in horizontal plane.
  const yaw = Math.atan2(fwd.x, fwd.z);
  // Pitch: elevation of forward axis above horizon.
  const pitch = Math.asin(Math.max(-1, Math.min(1, fwd.y)));
  // Roll: tilt of right axis around forward axis.
  // Project right onto plane perpendicular to fwd, measure angle from world horizontal.
  const roll = Math.atan2(right.y, Math.cos(pitch) * (right.x * Math.cos(yaw) - right.z * Math.sin(yaw)));
  return { roll, pitch, yaw };
};

// Integrate orientation by angular velocity ω (body frame) over dt.
// dq/dt = 0.5 * q * ω_q,  ω_q = (0, ωx, ωy, ωz)
export const qIntegrate = (q, omegaBody, dt) => {
  const wq = { w: 0, x: omegaBody.x, y: omegaBody.y, z: omegaBody.z };
  const dq = qMul(q, wq);
  const out = {
    w: q.w + 0.5 * dq.w * dt,
    x: q.x + 0.5 * dq.x * dt,
    y: q.y + 0.5 * dq.y * dt,
    z: q.z + 0.5 * dq.z * dt,
  };
  return qNorm(out);
};

// 4x4 matrix operations (row-major). Used by control allocator.
export const mat4Inv = (m) => {
  // General 4x4 inverse via cofactor expansion. Returns null if singular.
  const a = m;
  const inv = new Array(16);
  inv[0]  =  a[5]*a[10]*a[15] - a[5]*a[11]*a[14] - a[9]*a[6]*a[15] + a[9]*a[7]*a[14] + a[13]*a[6]*a[11] - a[13]*a[7]*a[10];
  inv[4]  = -a[4]*a[10]*a[15] + a[4]*a[11]*a[14] + a[8]*a[6]*a[15] - a[8]*a[7]*a[14] - a[12]*a[6]*a[11] + a[12]*a[7]*a[10];
  inv[8]  =  a[4]*a[9]*a[15]  - a[4]*a[11]*a[13] - a[8]*a[5]*a[15] + a[8]*a[7]*a[13] + a[12]*a[5]*a[11] - a[12]*a[7]*a[9];
  inv[12] = -a[4]*a[9]*a[14]  + a[4]*a[10]*a[13] + a[8]*a[5]*a[14] - a[8]*a[6]*a[13] - a[12]*a[5]*a[10] + a[12]*a[6]*a[9];
  inv[1]  = -a[1]*a[10]*a[15] + a[1]*a[11]*a[14] + a[9]*a[2]*a[15] - a[9]*a[3]*a[14] - a[13]*a[2]*a[11] + a[13]*a[3]*a[10];
  inv[5]  =  a[0]*a[10]*a[15] - a[0]*a[11]*a[14] - a[8]*a[2]*a[15] + a[8]*a[3]*a[14] + a[12]*a[2]*a[11] - a[12]*a[3]*a[10];
  inv[9]  = -a[0]*a[9]*a[15]  + a[0]*a[11]*a[13] + a[8]*a[1]*a[15] - a[8]*a[3]*a[13] - a[12]*a[1]*a[11] + a[12]*a[3]*a[9];
  inv[13] =  a[0]*a[9]*a[14]  - a[0]*a[10]*a[13] - a[8]*a[1]*a[14] + a[8]*a[2]*a[13] + a[12]*a[1]*a[10] - a[12]*a[2]*a[9];
  inv[2]  =  a[1]*a[6]*a[15]  - a[1]*a[7]*a[14]  - a[5]*a[2]*a[15] + a[5]*a[3]*a[14] + a[13]*a[2]*a[7]  - a[13]*a[3]*a[6];
  inv[6]  = -a[0]*a[6]*a[15]  + a[0]*a[7]*a[14]  + a[4]*a[2]*a[15] - a[4]*a[3]*a[14] - a[12]*a[2]*a[7]  + a[12]*a[3]*a[6];
  inv[10] =  a[0]*a[5]*a[15]  - a[0]*a[7]*a[13]  - a[4]*a[1]*a[15] + a[4]*a[3]*a[13] + a[12]*a[1]*a[7]  - a[12]*a[3]*a[5];
  inv[14] = -a[0]*a[5]*a[14]  + a[0]*a[6]*a[13]  + a[4]*a[1]*a[14] - a[4]*a[2]*a[13] - a[12]*a[1]*a[6]  + a[12]*a[2]*a[5];
  inv[3]  = -a[1]*a[6]*a[11]  + a[1]*a[7]*a[10]  + a[5]*a[2]*a[11] - a[5]*a[3]*a[10] - a[9]*a[2]*a[7]   + a[9]*a[3]*a[6];
  inv[7]  =  a[0]*a[6]*a[11]  - a[0]*a[7]*a[10]  - a[4]*a[2]*a[11] + a[4]*a[3]*a[10] + a[8]*a[2]*a[7]   - a[8]*a[3]*a[6];
  inv[11] = -a[0]*a[5]*a[11]  + a[0]*a[7]*a[9]   + a[4]*a[1]*a[11] - a[4]*a[3]*a[9]  - a[8]*a[1]*a[7]   + a[8]*a[3]*a[5];
  inv[15] =  a[0]*a[5]*a[10]  - a[0]*a[6]*a[9]   - a[4]*a[1]*a[10] + a[4]*a[2]*a[9]  + a[8]*a[1]*a[6]   - a[8]*a[2]*a[5];

  let det = a[0]*inv[0] + a[1]*inv[4] + a[2]*inv[8] + a[3]*inv[12];
  if (Math.abs(det) < 1e-14) return null;
  det = 1.0 / det;
  for (let i = 0; i < 16; i++) inv[i] *= det;
  return inv;
};

// 4x4 * 4-vector. Vector is [w0, w1, w2, w3].
export const mat4MulVec = (m, v) => [
  m[0]*v[0] + m[1]*v[1] + m[2]*v[2] + m[3]*v[3],
  m[4]*v[0] + m[5]*v[1] + m[6]*v[2] + m[7]*v[3],
  m[8]*v[0] + m[9]*v[1] + m[10]*v[2] + m[11]*v[3],
  m[12]*v[0] + m[13]*v[1] + m[14]*v[2] + m[15]*v[3],
];

export const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));
export const lerp = (a, b, t) => a + (b - a) * t;
export const deg = (r) => r * 180 / Math.PI;
export const rad = (d) => d * Math.PI / 180;

// Wrap an angle (radians) into [-π, π]. Used for shortest-arc heading errors.
export const wrapPi = (a) => {
  const tau = 2 * Math.PI;
  let r = a - tau * Math.floor((a + Math.PI) / tau);
  return r;
};

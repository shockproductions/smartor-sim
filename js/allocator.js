// Control allocation for SMaRTOR.
//
// Two strategies are exposed:
//  - simpleMixer: maps throttle/roll/pitch/yaw channels onto the real vehicle
//                 geometry while keeping familiar pilot-facing stick semantics.
//  - solveAllocator: builds the 4x4 effectiveness matrix that maps motor
//                    commands to {Fy, Mx, My, Mz} in body frame and inverts it
//                    so the controller can request a wrench directly.

import { mat4Inv, mat4MulVec, clamp } from "./mathx.js";

// Map [throttle, roll, pitch, yaw] (each -1..+1, throttle 0..1) to motor cmds.
// The mixer remains channel based, but it reuses the allocator so the neutral
// point matches the real hover trim and the 120-degree tilted outer rotors
// produce the intended axes.
//
// Pilot-facing conventions:
//   roll  + = bank right
//   pitch + = nose up
//   yaw   + = yaw right
export function simpleMixer(thr, roll, pitch, yaw, cfg, positions, directions, cache) {
  const m = cfg.frameMass + cfg.batteryMass + cfg.payloadMass;
  const Fy = m * cfg.gravity * (0.6 + 0.8 * clamp(thr, 0, 1)); // 0.5 stick = hover
  const Mx = -pitch * 2.0;
  const My = yaw * 0.75;
  const Mz = -roll * 2.0;
  return solveAllocator({ Fy, Mx, My, Mz }, cfg, positions, directions, cache);
}

// Build the 4x4 control effectiveness matrix B such that
//   [Fy, Mx, My, Mz]^T = B * [l_c, l0, l1, l2]^T
// where l_x are normalized linear thrust fractions (0..1) and the rotor force
// is f_i = T_max_i * l_i * direction_i.
//
// The solver converts these linear thrust fractions back to motor commands with
// command = fraction^(1/gamma), matching rotor.thrustFromCommand().
export function buildEffectivenessMatrix(cfg, positions, directions) {
  const Tmax = [
    cfg.centralMaxThrust,
    cfg.outerMaxThrust,
    cfg.outerMaxThrust,
    cfg.outerMaxThrust,
  ];
  const kQ = [cfg.centralKq, cfg.outerKq, cfg.outerKq, cfg.outerKq];
  const spin = [cfg.centralSpinSign, cfg.outerSpinSign, cfg.outerSpinSign, cfg.outerSpinSign];

  const B = new Array(16).fill(0);
  for (let j = 0; j < 4; j++) {
    const F = {
      x: Tmax[j] * directions[j].x,
      y: Tmax[j] * directions[j].y,
      z: Tmax[j] * directions[j].z,
    };
    const r = positions[j];
    const Mthr = {
      x: r.y * F.z - r.z * F.y,
      y: r.z * F.x - r.x * F.z,
      z: r.x * F.y - r.y * F.x,
    };
    const reactMag = -spin[j] * kQ[j] * Tmax[j];
    const Mreact = {
      x: directions[j].x * reactMag,
      y: directions[j].y * reactMag,
      z: directions[j].z * reactMag,
    };
    const M = { x: Mthr.x + Mreact.x, y: Mthr.y + Mreact.y, z: Mthr.z + Mreact.z };
    B[0 * 4 + j] = F.y;
    B[1 * 4 + j] = M.x;
    B[2 * 4 + j] = M.y;
    B[3 * 4 + j] = M.z;
  }
  return B;
}

// Solve for motor commands given a desired body-frame wrench [Fy, Mx, My, Mz].
// Returns {c, o:[u0,u1,u2], raw:[l_c,l0,l1,l2]} with commands clamped to [0,1].
// `cache` can be shared across calls; the inverse is recomputed only when
// `cache.dirty` is true or missing.
export function solveAllocator(wrench, cfg, positions, directions, cache) {
  if (!cache.B || cache.dirty) {
    cache.B = buildEffectivenessMatrix(cfg, positions, directions);
    cache.Binv = mat4Inv(cache.B);
    cache.dirty = false;
  }

  let u;
  if (cache.Binv) {
    u = mat4MulVec(cache.Binv, [wrench.Fy, wrench.Mx, wrench.My, wrench.Mz]);
  } else {
    return hoverTrim(cfg, positions, directions);
  }

  const gamma = Math.max(cfg.thrustGamma || 1, 1e-6);
  const toCommand = (linearFraction) => Math.pow(clamp(linearFraction, 0, 1), 1 / gamma);
  return {
    c: toCommand(u[0]),
    o: [toCommand(u[1]), toCommand(u[2]), toCommand(u[3])],
    raw: u,
  };
}

// Compute the hover-trim point: vertical force = m*g, all moments = 0.
// Returns the {c, o, raw} that satisfies the system for the given config.
export function hoverTrim(cfg, positions, directions) {
  const cache = { dirty: true };
  const m = cfg.frameMass + cfg.batteryMass + cfg.payloadMass;
  const wrench = { Fy: m * cfg.gravity, Mx: 0, My: 0, Mz: 0 };
  return solveAllocator(wrench, cfg, positions, directions, cache);
}

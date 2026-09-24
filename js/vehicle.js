// Vehicle definition + state container.
// All distances in meters, masses in kg, forces in N, moments in N·m, angles in radians.

import { v3, qIdent, qCopy, vCopy, rad } from "./mathx.js";

// Default SMaRTOR configuration. Numbers are illustrative starting values
// chosen to be roughly self-consistent for a small-scale prototype.
export function defaultConfig() {
  return {
    // --- Geometry ---
    centralRotorDiameter: 0.60,   // m
    outerRotorDiameter: 0.20,
    outerRotorRadius: 0.45,        // distance from CoM to each outer motor (m)
    outerTiltDeg: 12,              // tangential tilt angle of outer rotors (deg)
    centralOffset: { x: 0, y: 0.05, z: 0 }, // CoM-relative offset of central rotor hub
    cgOffset: { x: 0, y: 0, z: 0 },         // shift CoM from geometric center

    // --- Mass & inertia ---
    frameMass: 1.6,                // kg
    batteryMass: 0.8,
    payloadMass: 0.0,
    // Diagonal inertia tensor (body frame, units kg·m^2). Y is up = yaw axis.
    Ixx: 0.045,
    Iyy: 0.080,
    Izz: 0.045,

    // --- Propulsion ---
    centralMaxThrust: 35.0,        // N at full throttle
    outerMaxThrust: 8.0,           // each
    centralKq: 0.018,              // reaction torque per Newton of thrust (N·m / N)
    outerKq: 0.012,
    centralSpinSign: +1,           // +1 = CCW about +Y (viewed from above)
    outerSpinSign: -1,             // outer rotors spin opposite to balance reaction
    motorTau: 0.06,                // first-order spool-up time constant (s)
    thrustGamma: 1.6,              // thrust ~ command^gamma (mild squared-ish curve)

    // --- Aerodynamics / environment ---
    gravity: 9.80665,
    airDensity: 1.225,             // kg/m^3 (informational, not currently coupled to thrust model)
    linearDragK: 0.25,             // N per (m/s)^2, simple isotropic quadratic drag
    angularDragK: 0.06,            // N·m per (rad/s) on each axis

    // --- Control ---
    controlMode: "stabilize",      // "manual" | "mixer" | "allocator" | "stabilize"
    rateGains: { p: 0.18, i: 0.05, d: 0.004, max: 5.0 },     // per-axis rate PID
    attitudeGains: { p: 6.0, max: 4.0 },                       // attitude->rate
    altitudeGains: { p: 1.5, d: 1.2 },                         // climb-rate->Fz
    posKp: 1.0,                    // altitude error (m) -> climb-rate (m/s)
    headingKp: 2.0,                // heading error (rad) -> yaw rate (rad/s)
    hoverThrottle: 0.55,           // initial guess; allocator overrides at runtime
  };
}

// Initial vehicle state.
export function newState() {
  return {
    pos: v3(0, 1.0, 0),            // start 1 m up
    vel: v3(),
    quat: qIdent(),
    omega: v3(),                   // body-frame angular velocity
    // Per-rotor commanded throttle (0..1) and current (post-lag) throttle.
    cmd: { c: 0, o: [0, 0, 0] },
    act: { c: 0, o: [0, 0, 0] },
    // Cached per-step diagnostics for telemetry/visualisation.
    diag: {
      thrustVec: { c: v3(), o: [v3(), v3(), v3()] },     // body-frame thrust per rotor
      thrustMag: { c: 0, o: [0, 0, 0] },
      reactionTorque: v3(),                               // body frame
      netForceWorld: v3(),
      netMomentBody: v3(),
      power: 0,
      saturationFlags: [false, false, false, false],
    },
    onGround: false,
    time: 0,
  };
}

// Total mass (kg).
export const totalMass = (cfg) => cfg.frameMass + cfg.batteryMass + cfg.payloadMass;

// Position vectors of the four rotors in body frame, relative to CoM.
// Outer rotors lie in body XZ plane (horizontal), spaced 120° starting at +Z (forward).
export function rotorPositions(cfg) {
  const R = cfg.outerRotorRadius;
  const offs = [
    { x: cfg.cgOffset.x * -1 + cfg.centralOffset.x,
      y: cfg.cgOffset.y * -1 + cfg.centralOffset.y,
      z: cfg.cgOffset.z * -1 + cfg.centralOffset.z },
  ];
  for (let i = 0; i < 3; i++) {
    const theta = (i * 2 * Math.PI) / 3; // 0, 120, 240 deg
    offs.push({
      x: -cfg.cgOffset.x + R * Math.sin(theta),
      y: -cfg.cgOffset.y,
      z: -cfg.cgOffset.z + R * Math.cos(theta),
    });
  }
  return offs; // [central, outer0, outer1, outer2]
}

// Unit thrust direction vectors in body frame.
// Central rotor thrusts straight up (+Y body).
// Outer rotors are tilted by α tangentially in CCW sense about +Y.
// At azimuth θ (measured from +Z, around +Y), tangential unit = (cos θ, 0, -sin θ).
// Thrust direction = (sin α · cos θ, cos α, -sin α · sin θ).
export function rotorDirections(cfg) {
  const a = rad(cfg.outerTiltDeg);
  const sa = Math.sin(a), ca = Math.cos(a);
  const dirs = [{ x: 0, y: 1, z: 0 }];
  for (let i = 0; i < 3; i++) {
    const theta = (i * 2 * Math.PI) / 3;
    dirs.push({
      x: sa * Math.cos(theta),
      y: ca,
      z: -sa * Math.sin(theta),
    });
  }
  return dirs;
}

// Rotor azimuth angles (radians, around +Y from +Z axis).
export function rotorAzimuths() {
  return [0, (2 * Math.PI) / 3, (4 * Math.PI) / 3];
}

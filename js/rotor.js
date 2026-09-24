// Rotor model: maps commanded throttle (0..1) to thrust magnitude and
// produces a reaction torque about the rotor spin axis. Includes a
// first-order lag to approximate motor/ESC spool-up.

import { clamp } from "./mathx.js";

// Compute steady-state thrust for a normalized command (0..1).
export function thrustFromCommand(cmd, maxThrust, gamma = 1.6) {
  const u = clamp(cmd, 0, 1);
  return maxThrust * Math.pow(u, gamma);
}

// Advance per-rotor "actual" throttle toward the commanded throttle using
// a first-order lag. tau is the time constant (seconds).
export function stepActuator(actual, cmd, tau, dt) {
  if (tau <= 1e-6) return clamp(cmd, 0, 1);
  const alpha = 1 - Math.exp(-dt / tau);
  return actual + (clamp(cmd, 0, 1) - actual) * alpha;
}

// Reaction torque on the airframe from a rotor (about the rotor's thrust axis).
// kQ is N·m per N of thrust. Sign opposes spin direction.
export function reactionTorqueScalar(thrustN, kQ, spinSign) {
  return -spinSign * kQ * thrustN;
}

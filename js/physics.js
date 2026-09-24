// Rigid body physics step for the SMaRTOR vehicle.
// Integrates: position/velocity in world frame, orientation/angular velocity
// using a body-frame angular velocity and quaternion.

import {
  v3, vAdd, vSub, vScale, vCross, vLen, vDot, vNorm,
  qRotate, qInverse, qIntegrate,
} from "./mathx.js";
import { totalMass, rotorPositions, rotorDirections } from "./vehicle.js";
import { thrustFromCommand, stepActuator, reactionTorqueScalar } from "./rotor.js";

// Compute and apply forces/torques for one substep of length dt.
// Mutates state in place. Returns the same state object.
export function step(state, cfg, dt) {
  // 1) Advance actuator dynamics (motor lag).
  state.act.c = stepActuator(state.act.c, state.cmd.c, cfg.motorTau, dt);
  for (let i = 0; i < 3; i++) {
    state.act.o[i] = stepActuator(state.act.o[i], state.cmd.o[i], cfg.motorTau, dt);
  }

  // 2) Compute thrust per rotor (body frame).
  const positions = rotorPositions(cfg);    // [central, o0, o1, o2]
  const directions = rotorDirections(cfg);

  const thrustMag = [
    thrustFromCommand(state.act.c, cfg.centralMaxThrust, cfg.thrustGamma),
    thrustFromCommand(state.act.o[0], cfg.outerMaxThrust, cfg.thrustGamma),
    thrustFromCommand(state.act.o[1], cfg.outerMaxThrust, cfg.thrustGamma),
    thrustFromCommand(state.act.o[2], cfg.outerMaxThrust, cfg.thrustGamma),
  ];
  // Saturation flags (within 1% of max command).
  state.diag.saturationFlags = [
    state.cmd.c >= 0.99,
    state.cmd.o[0] >= 0.99,
    state.cmd.o[1] >= 0.99,
    state.cmd.o[2] >= 0.99,
  ];

  // 3) Sum body-frame force and moment from all rotors.
  let Fbody = v3();
  let Mbody = v3();
  const thrustVecs = new Array(4);
  for (let i = 0; i < 4; i++) {
    const F = vScale(directions[i], thrustMag[i]);
    thrustVecs[i] = F;
    Fbody = vAdd(Fbody, F);
    Mbody = vAdd(Mbody, vCross(positions[i], F));
  }
  state.diag.thrustVec.c = thrustVecs[0];
  state.diag.thrustVec.o[0] = thrustVecs[1];
  state.diag.thrustVec.o[1] = thrustVecs[2];
  state.diag.thrustVec.o[2] = thrustVecs[3];
  state.diag.thrustMag.c = thrustMag[0];
  state.diag.thrustMag.o[0] = thrustMag[1];
  state.diag.thrustMag.o[1] = thrustMag[2];
  state.diag.thrustMag.o[2] = thrustMag[3];

  // 4) Reaction torques. Each rotor produces a torque about its thrust axis
  // opposing its spin direction. Approximate: τ = -spin * kQ * |F|.
  const tauC = reactionTorqueScalar(thrustMag[0], cfg.centralKq, cfg.centralSpinSign);
  const tauO = [
    reactionTorqueScalar(thrustMag[1], cfg.outerKq, cfg.outerSpinSign),
    reactionTorqueScalar(thrustMag[2], cfg.outerKq, cfg.outerSpinSign),
    reactionTorqueScalar(thrustMag[3], cfg.outerKq, cfg.outerSpinSign),
  ];
  // Apply along each rotor's thrust direction (body frame).
  let reactionM = vScale(directions[0], tauC);
  for (let i = 0; i < 3; i++) {
    reactionM = vAdd(reactionM, vScale(directions[i + 1], tauO[i]));
  }
  Mbody = vAdd(Mbody, reactionM);
  state.diag.reactionTorque = reactionM;

  // 5) Aerodynamic damping (very simple).
  // Linear drag opposite to world-frame velocity, quadratic.
  const speed = vLen(state.vel);
  let dragWorld = v3();
  if (speed > 1e-4) {
    const k = cfg.linearDragK;
    const mag = k * speed * speed;
    dragWorld = vScale(vNorm(state.vel), -mag);
  }
  // Angular drag (body frame, linear in ω).
  const angularDrag = vScale(state.omega, -cfg.angularDragK);
  Mbody = vAdd(Mbody, angularDrag);

  // 6) Transform body force to world, add gravity & drag.
  const Fworld_thrust = qRotate(state.quat, Fbody);
  const m = totalMass(cfg);
  const Fworld = vAdd(
    vAdd(Fworld_thrust, dragWorld),
    v3(0, -m * cfg.gravity, 0)
  );
  state.diag.netForceWorld = Fworld;
  state.diag.netMomentBody = Mbody;

  // 7) Integrate translation (semi-implicit Euler).
  const accel = vScale(Fworld, 1 / m);
  state.vel = vAdd(state.vel, vScale(accel, dt));
  state.pos = vAdd(state.pos, vScale(state.vel, dt));

  // 8) Integrate rotation. ω' = I^-1 (M - ω × Iω). Diagonal inertia.
  const Ix = cfg.Ixx, Iy = cfg.Iyy, Iz = cfg.Izz;
  const w = state.omega;
  const Iw = v3(Ix * w.x, Iy * w.y, Iz * w.z);
  const wxIw = vCross(w, Iw);
  const Mnet = vSub(Mbody, wxIw);
  const omegaDot = v3(Mnet.x / Ix, Mnet.y / Iy, Mnet.z / Iz);
  state.omega = vAdd(state.omega, vScale(omegaDot, dt));
  state.quat = qIntegrate(state.quat, state.omega, dt);

  // 9) Ground constraint: simple stop at y=0 (spring-damper would be smoother
  // but for an overview-level sim, a hard floor is fine).
  if (state.pos.y < 0) {
    state.pos.y = 0;
    if (state.vel.y < 0) state.vel.y = 0;
    // Gentle friction to avoid sliding forever on the ground.
    state.vel.x *= Math.max(0, 1 - 4 * dt);
    state.vel.z *= Math.max(0, 1 - 4 * dt);
    state.onGround = true;
  } else {
    state.onGround = false;
  }

  // 10) Power estimate: thrust * |v_axial| through prop disk is the ideal
  // mechanical power; we approximate with thrust * proportional rotor speed.
  // For a non-blade-element model, use P ≈ T^1.5 / sqrt(2 ρ A) (momentum theory).
  const rho = cfg.airDensity;
  const Ac = Math.PI * (cfg.centralRotorDiameter / 2) ** 2;
  const Ao = Math.PI * (cfg.outerRotorDiameter / 2) ** 2;
  const pw = (T, A) => Math.pow(Math.max(T, 0), 1.5) / Math.sqrt(2 * rho * Math.max(A, 1e-6));
  state.diag.power =
    pw(thrustMag[0], Ac) +
    pw(thrustMag[1], Ao) +
    pw(thrustMag[2], Ao) +
    pw(thrustMag[3], Ao);

  state.time += dt;
  return state;
}

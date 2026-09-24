import { defaultConfig, newState, rotorDirections, rotorPositions, totalMass } from "../js/vehicle.js";
import { buildEffectivenessMatrix, hoverTrim, simpleMixer, solveAllocator } from "../js/allocator.js";
import { computeCommand, newCtrlState, newPilot } from "../js/controller.js";
import { step } from "../js/physics.js";
import { mat4Inv, qFromEuler, qToEuler, rad } from "../js/mathx.js";
import { thrustFromCommand } from "../js/rotor.js";

const DT = 1 / 500;

function assert(cond, message) {
  if (!cond) throw new Error(message);
}

function assertNear(actual, expected, tol, message) {
  assert(Math.abs(actual - expected) <= tol, `${message}: got ${actual}, expected ${expected} +/- ${tol}`);
}

function nonFinitePaths(value, path = "") {
  if (typeof value === "number") return Number.isFinite(value) ? [] : [path];
  if (!value || typeof value !== "object") return [];
  return Object.entries(value).flatMap(([key, child]) => nonFinitePaths(child, path ? `${path}.${key}` : key));
}

function assertValidCommand(cmd, label) {
  const values = [cmd.c, ...cmd.o];
  assert(values.every((u) => Number.isFinite(u) && u >= 0 && u <= 1), `${label} returned invalid motor commands`);
}

function physicsWrenchFromCommand(cfg, positions, directions, cmd) {
  const thrust = [
    thrustFromCommand(cmd.c, cfg.centralMaxThrust, cfg.thrustGamma),
    thrustFromCommand(cmd.o[0], cfg.outerMaxThrust, cfg.thrustGamma),
    thrustFromCommand(cmd.o[1], cfg.outerMaxThrust, cfg.thrustGamma),
    thrustFromCommand(cmd.o[2], cfg.outerMaxThrust, cfg.thrustGamma),
  ];
  const kQ = [cfg.centralKq, cfg.outerKq, cfg.outerKq, cfg.outerKq];
  const spin = [cfg.centralSpinSign, cfg.outerSpinSign, cfg.outerSpinSign, cfg.outerSpinSign];

  let Fy = 0;
  let Mx = 0;
  let My = 0;
  let Mz = 0;
  for (let i = 0; i < 4; i++) {
    const F = {
      x: thrust[i] * directions[i].x,
      y: thrust[i] * directions[i].y,
      z: thrust[i] * directions[i].z,
    };
    const r = positions[i];
    const Mthr = {
      x: r.y * F.z - r.z * F.y,
      y: r.z * F.x - r.x * F.z,
      z: r.x * F.y - r.y * F.x,
    };
    const reactMag = -spin[i] * kQ[i] * thrust[i];
    const Mreact = {
      x: directions[i].x * reactMag,
      y: directions[i].y * reactMag,
      z: directions[i].z * reactMag,
    };
    Fy += F.y;
    Mx += Mthr.x + Mreact.x;
    My += Mthr.y + Mreact.y;
    Mz += Mthr.z + Mreact.z;
  }
  return { Fy, Mx, My, Mz };
}

function runModeStep(mode) {
  const cfg = defaultConfig();
  cfg.controlMode = mode;
  const state = newState();
  const pilot = newPilot();
  const ctrl = newCtrlState();

  if (mode === "waypoint") {
    pilot.mission.waypoints = [{ x: 0, y: 1.5, z: 0, speed: 1.0, wait: 0 }];
    pilot.mission.active = true;
  } else {
    pilot.thr = 0.55;
    pilot.roll = 0.2;
    pilot.pitch = -0.15;
    pilot.yaw = 0.1;
  }

  computeCommand(state, cfg, pilot, ctrl, DT);
  assertValidCommand(state.cmd, mode);
  step(state, cfg, DT);

  const bad = nonFinitePaths(state);
  assert(bad.length === 0, `${mode} produced non-finite state values: ${bad.join(", ")}`);
}

function runMixerNeutralCheck() {
  const cfg = defaultConfig();
  const positions = rotorPositions(cfg);
  const directions = rotorDirections(cfg);
  const cmd = simpleMixer(0.5, 0, 0, 0, cfg, positions, directions, { dirty: true });
  const wrench = physicsWrenchFromCommand(cfg, positions, directions, cmd);
  const weight = totalMass(cfg) * cfg.gravity;

  assertValidCommand(cmd, "mixer neutral");
  assertNear(wrench.Fy, weight, 1e-9, "mixer neutral lift");
  assertNear(wrench.Mx, 0, 1e-9, "mixer neutral pitch moment");
  assertNear(wrench.My, 0, 1e-9, "mixer neutral yaw moment");
  assertNear(wrench.Mz, 0, 1e-9, "mixer neutral roll moment");
}

function runMixerAxisCheck() {
  const cfg = defaultConfig();
  const positions = rotorPositions(cfg);
  const directions = rotorDirections(cfg);

  let cmd = simpleMixer(0.5, 0.1, 0, 0, cfg, positions, directions, { dirty: true });
  let wrench = physicsWrenchFromCommand(cfg, positions, directions, cmd);
  assert(wrench.Mz < -0.1, `mixer +roll should bank right (negative Mz), got ${wrench.Mz}`);
  assert(Math.abs(wrench.Mx) < 0.03, `mixer +roll cross-coupled pitch too much: ${wrench.Mx}`);
  assert(Math.abs(wrench.My) < 0.03, `mixer +roll cross-coupled yaw too much: ${wrench.My}`);

  cmd = simpleMixer(0.5, 0, 0.1, 0, cfg, positions, directions, { dirty: true });
  wrench = physicsWrenchFromCommand(cfg, positions, directions, cmd);
  assert(wrench.Mx < -0.1, `mixer +pitch should nose up (negative Mx), got ${wrench.Mx}`);
  assert(Math.abs(wrench.My) < 0.03, `mixer +pitch cross-coupled yaw too much: ${wrench.My}`);
  assert(Math.abs(wrench.Mz) < 0.03, `mixer +pitch cross-coupled roll too much: ${wrench.Mz}`);

  cmd = simpleMixer(0.5, 0, 0, 0.1, cfg, positions, directions, { dirty: true });
  wrench = physicsWrenchFromCommand(cfg, positions, directions, cmd);
  assert(wrench.My > 0.04, `mixer +yaw should yaw right (positive My), got ${wrench.My}`);
  assert(Math.abs(wrench.Mx) < 0.03, `mixer +yaw cross-coupled pitch too much: ${wrench.Mx}`);
  assert(Math.abs(wrench.Mz) < 0.03, `mixer +yaw cross-coupled roll too much: ${wrench.Mz}`);
}

function runAllocatorAxisCheck() {
  const cfg = defaultConfig();
  cfg.controlMode = "allocator";
  const state = newState();
  const pilot = newPilot();
  const ctrl = newCtrlState();

  pilot.thr = 0.5;
  pilot.roll = 1;
  pilot.pitch = 0;
  pilot.yaw = 0;
  let debug = computeCommand(state, cfg, pilot, ctrl, DT);
  assertNear(debug.wrench.Mx, 0, 1e-12, "roll should not command pitch moment");
  assertNear(debug.wrench.My, 0, 1e-12, "roll should not command yaw moment");
  assertNear(debug.wrench.Mz, -1.2, 1e-12, "roll should bank right with negative Mz");

  pilot.roll = 0;
  pilot.pitch = 1;
  debug = computeCommand(state, cfg, pilot, ctrl, DT);
  assertNear(debug.wrench.Mx, -1.2, 1e-12, "pitch should nose up with negative Mx");
  assertNear(debug.wrench.My, 0, 1e-12, "pitch should not command yaw moment");
  assertNear(debug.wrench.Mz, 0, 1e-12, "pitch should not command roll moment");

  pilot.pitch = 0;
  pilot.yaw = 1;
  debug = computeCommand(state, cfg, pilot, ctrl, DT);
  assertNear(debug.wrench.Mx, 0, 1e-12, "yaw should not command pitch moment");
  assertNear(debug.wrench.My, 0.6, 1e-12, "yaw should command +Y yaw moment");
  assertNear(debug.wrench.Mz, 0, 1e-12, "yaw should not command roll moment");
}

function runStabilizeHover() {
  const cfg = defaultConfig();
  const state = newState();
  const pilot = newPilot();
  const ctrl = newCtrlState();
  const positions = rotorPositions(cfg);
  const directions = rotorDirections(cfg);
  const trim = hoverTrim(cfg, positions, directions);

  state.pos.y = pilot.targetAltitude;
  state.act.c = trim.c;
  state.act.o = [...trim.o];
  state.cmd.c = trim.c;
  state.cmd.o = [...trim.o];

  let maxAbsRoll = 0;
  let maxAbsPitch = 0;
  let maxAbsYawRate = 0;
  for (let i = 0; i < 5000; i++) {
    computeCommand(state, cfg, pilot, ctrl, DT);
    step(state, cfg, DT);
    const euler = qToEuler(state.quat);
    maxAbsRoll = Math.max(maxAbsRoll, Math.abs(euler.roll));
    maxAbsPitch = Math.max(maxAbsPitch, Math.abs(euler.pitch));
    maxAbsYawRate = Math.max(maxAbsYawRate, Math.abs(state.omega.y));
    const bad = nonFinitePaths(state);
    assert(bad.length === 0, `stabilize hover produced non-finite values at step ${i}: ${bad.join(", ")}`);
  }

  assertNear(state.pos.y, pilot.targetAltitude, 0.05, "stabilize hover altitude drift");
  assert(Math.abs(state.vel.y) < 0.05, `stabilize hover vertical speed too high: ${state.vel.y}`);
  assert(maxAbsRoll < 0.01, `stabilize hover roll drift too high: ${maxAbsRoll}`);
  assert(maxAbsPitch < 0.01, `stabilize hover pitch drift too high: ${maxAbsPitch}`);
  assert(maxAbsYawRate < 0.05, `stabilize hover yaw rate too high: ${maxAbsYawRate}`);
  assert(!state.onGround, "stabilize hover touched the ground");
}

function runStabilizeRecovery() {
  const cfg = defaultConfig();
  const state = newState();
  const pilot = newPilot();
  const ctrl = newCtrlState();
  const positions = rotorPositions(cfg);
  const directions = rotorDirections(cfg);
  const trim = hoverTrim(cfg, positions, directions);

  state.pos.y = pilot.targetAltitude;
  state.quat = qFromEuler(rad(8), rad(6), 0);
  state.act.c = trim.c;
  state.act.o = [...trim.o];
  state.cmd.c = trim.c;
  state.cmd.o = [...trim.o];

  const initial = qToEuler(state.quat);
  for (let i = 0; i < 3000; i++) {
    computeCommand(state, cfg, pilot, ctrl, DT);
    step(state, cfg, DT);
    const bad = nonFinitePaths(state);
    assert(bad.length === 0, `stabilize recovery produced non-finite values at step ${i}: ${bad.join(", ")}`);
  }

  const final = qToEuler(state.quat);
  assert(Math.abs(final.roll) < Math.abs(initial.roll) * 0.25, `stabilize roll recovery too weak: ${final.roll}`);
  assert(Math.abs(final.pitch) < Math.abs(initial.pitch) * 0.25, `stabilize pitch recovery too weak: ${final.pitch}`);
  assert(Math.abs(final.roll) < 0.05, `stabilize final roll too large: ${final.roll}`);
  assert(Math.abs(final.pitch) < 0.05, `stabilize final pitch too large: ${final.pitch}`);
  assert(Math.abs(state.pos.y - pilot.targetAltitude) < 0.15, `stabilize recovery altitude drift too high: ${state.pos.y}`);
  assert(!state.onGround, "stabilize recovery touched the ground");
}

function runWaypointMission() {
  const cfg = defaultConfig();
  cfg.controlMode = "waypoint";
  const state = newState();
  const pilot = newPilot();
  const ctrl = newCtrlState();
  const positions = rotorPositions(cfg);
  const directions = rotorDirections(cfg);
  const trim = hoverTrim(cfg, positions, directions);

  state.pos.y = 1.5;
  state.act.c = trim.c;
  state.act.o = [...trim.o];
  state.cmd.c = trim.c;
  state.cmd.o = [...trim.o];

  pilot.mission.active = true;
  pilot.mission.defaultSpeed = 1.2;
  pilot.mission.waypoints = [
    { x: 2, y: 1.5, z: 0, speed: 1.2, wait: 0.2 },
    { x: 2, y: 1.8, z: 2, speed: 1.0, wait: 0.2 },
  ];

  let completed = false;
  for (let i = 0; i < 9000; i++) {
    computeCommand(state, cfg, pilot, ctrl, DT);
    step(state, cfg, DT);
    const bad = nonFinitePaths(state);
    assert(bad.length === 0, `waypoint mission produced non-finite values at step ${i}: ${bad.join(", ")}`);
    if (!pilot.mission.active && pilot.mission.status === "complete") {
      completed = true;
      break;
    }
  }

  assert(completed, "waypoint mission did not complete");
  assert(Math.hypot(state.pos.x - 2, state.pos.z - 2) < 0.45, `waypoint mission final horizontal error too large: ${state.pos.x}, ${state.pos.z}`);
  assert(Math.abs(state.pos.y - 1.8) < 0.2, `waypoint mission final altitude error too large: ${state.pos.y}`);
  assert(pilot.mission.currentIndex >= pilot.mission.waypoints.length - 1, "waypoint mission did not advance through all waypoints");
}

const cfg = defaultConfig();
const positions = rotorPositions(cfg);
const directions = rotorDirections(cfg);
const weight = totalMass(cfg) * cfg.gravity;
const trim = hoverTrim(cfg, positions, directions);

assertValidCommand(trim, "hover trim");
assertNear(physicsWrenchFromCommand(cfg, positions, directions, trim).Fy, weight, 1e-9, "hover trim physics force");
assert(mat4Inv(buildEffectivenessMatrix(cfg, positions, directions)), "effectiveness matrix should be invertible");
assertValidCommand(
  solveAllocator({ Fy: weight, Mx: 0.2, My: -0.1, Mz: 0.05 }, cfg, positions, directions, { dirty: true }),
  "allocator solve",
);

for (const mode of ["manual", "mixer", "allocator", "stabilize", "waypoint"]) runModeStep(mode);
runMixerNeutralCheck();
runMixerAxisCheck();
runAllocatorAxisCheck();
runStabilizeHover();
runStabilizeRecovery();
runWaypointMission();

console.log("sim smoke tests passed");

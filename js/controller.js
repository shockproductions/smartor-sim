// SMaRTOR controllers and pilot input mapping.
//
// Modes:
//  - manual    : pilot commands raw motor throttles via 4 sticks
//  - mixer     : pilot commands [thr, roll, pitch, yaw], geometry-aware mixer
//  - allocator : pilot commands direct body wrench channels
//  - stabilize : pilot commands attitude angles + altitude rate, attitude PID
//                + rate PID + altitude P/D feed the allocator
//  - waypoint  : mission guidance generates attitude/altitude targets and then
//                reuses the stabilize controller underneath

import { clamp, qToEuler, qRotate, v3, wrapPi } from "./mathx.js";
import { simpleMixer, solveAllocator } from "./allocator.js";
import { rotorPositions, rotorDirections, totalMass } from "./vehicle.js";

// Demo mission loaded on startup: climb, fly a 5 m box with a height change,
// then return and settle over the start point. World frame is +Y up.
export function defaultMission() {
  return [
    { x: 0, y: 3,   z: 0, speed: 1.0, wait: 1 },
    { x: 5, y: 3,   z: 0, speed: 1.5, wait: 0 },
    { x: 5, y: 4,   z: 5, speed: 1.5, wait: 1 },
    { x: 0, y: 4,   z: 5, speed: 1.5, wait: 0 },
    { x: 0, y: 1.5, z: 0, speed: 1.0, wait: 1 },
  ];
}

// Pilot-facing conventions used across the sim:
//   roll  + = bank right
//   pitch + = nose up
//   yaw   + = yaw right
export function newPilot() {
  return {
    thr: 0.5,       // 0..1 (stabilize: 0.5 = neutral climb-rate command)
    roll: 0,        // -1..+1
    pitch: 0,
    yaw: 0,
    armed: true,
    holdAltitude: true,
    targetAltitude: 1.5,            // m
    holdHeading: false,
    targetHeading: 0,               // rad
    holdAttitude: false,
    targetRoll: 0,                  // rad (positive = bank right)
    targetPitch: 0,                 // rad (positive = nose up)
    mission: {
      active: false,
      loop: false,
      currentIndex: 0,
      holdTimer: 0,
      status: "idle",
      holdPoint: null,
      lastHeading: 0,
      defaultSpeed: 1.5,
      defaultWait: 0,
      maxTiltDeg: 18,
      maxAccel: 1.8,
      velGain: 1.8,
      waypoints: defaultMission(),
    },
  };
}

// Persistent controller state (PID integrators / previous errors).
export function newCtrlState() {
  return {
    rateIntegrator: { x: 0, y: 0, z: 0 },
    ratePrevError: { x: 0, y: 0, z: 0 },
    altIntegrator: 0,
    altPrev: null,
    cache: { dirty: true },
  };
}

export function markAllocatorDirty(ctrl) {
  ctrl.cache.dirty = true;
}

// Compute motor commands for the current step.
// Mutates state.cmd. Returns desired wrench / target debug data.
export function computeCommand(state, cfg, pilot, ctrl, dt) {
  const positions = rotorPositions(cfg);
  const directions = rotorDirections(cfg);

  let cmd;
  let debug = {};

  switch (cfg.controlMode) {
    case "manual": {
      cmd = {
        c: clamp(pilot.thr, 0, 1),
        o: [
          clamp(pilot.thr + pilot.roll * 0.5 + pilot.pitch * 0.5 - pilot.yaw * 0.4, 0, 1),
          clamp(pilot.thr - pilot.roll * 0.25 + pilot.pitch * 0.25 - pilot.yaw * 0.4, 0, 1),
          clamp(pilot.thr - pilot.roll * 0.25 - pilot.pitch * 0.25 - pilot.yaw * 0.4, 0, 1),
        ],
      };
      break;
    }

    case "mixer": {
      cmd = simpleMixer(pilot.thr, pilot.roll, pilot.pitch, pilot.yaw, cfg, positions, directions, ctrl.cache);
      break;
    }

    case "allocator": {
      const m = totalMass(cfg);
      const Fy = m * cfg.gravity * (0.6 + 0.8 * pilot.thr); // 0 -> 0.6g, 1 -> 1.4g
      const Mx = -pilot.pitch * 1.2;  // body +X
      const My = pilot.yaw * 0.6;     // body +Y
      const Mz = -pilot.roll * 1.2;   // body +Z
      cmd = solveAllocator({ Fy, Mx, My, Mz }, cfg, positions, directions, ctrl.cache);
      debug.wrench = { Fy, Mx, My, Mz };
      break;
    }

    case "waypoint": {
      const guidance = computeWaypointGuidance(state, cfg, pilot, dt);
      const out = computeStabilizeCommand(state, cfg, pilot, ctrl, dt, positions, directions, guidance);
      cmd = out.cmd;
      debug = { ...guidance.debug, ...out.debug };
      break;
    }

    case "stabilize":
    default: {
      const out = computeStabilizeCommand(state, cfg, pilot, ctrl, dt, positions, directions, null);
      cmd = out.cmd;
      debug = out.debug;
      break;
    }
  }

  state.cmd = cmd;
  if (!pilot.armed) {
    state.cmd = { c: 0, o: [0, 0, 0] };
  }
  return debug;
}

function computeStabilizeCommand(state, cfg, pilot, ctrl, dt, positions, directions, guidance) {
  const eul = qToEuler(state.quat);

  const desiredRoll = guidance?.targetRoll ??
    (pilot.holdAttitude ? pilot.targetRoll : pilot.roll * cfg.attitudeGains.max * 0.25);
  const desiredPitch = guidance?.targetPitch ??
    (pilot.holdAttitude ? pilot.targetPitch : pilot.pitch * cfg.attitudeGains.max * 0.25);

  const tgtRollInternal = -desiredRoll;
  const tgtPitchInternal = desiredPitch;

  let tgtYawRate;
  if (guidance?.holdHeading) {
    const headingErr = wrapPi(guidance.targetHeading - eul.yaw);
    tgtYawRate = clamp(cfg.headingKp * headingErr, -cfg.rateGains.max, cfg.rateGains.max);
  } else if (pilot.holdHeading) {
    const headingErr = wrapPi(pilot.targetHeading - eul.yaw);
    tgtYawRate = clamp(cfg.headingKp * headingErr, -cfg.rateGains.max, cfg.rateGains.max);
  } else {
    tgtYawRate = pilot.yaw * cfg.attitudeGains.max;
  }

  // qToEuler pitch increases for nose-up attitude, but body +X rate drives
  // qToEuler pitch negative, so the pitch loop needs the opposite sign.
  const desRateX = clamp(-cfg.attitudeGains.p * (tgtPitchInternal - eul.pitch), -cfg.rateGains.max, cfg.rateGains.max);
  const desRateY = clamp(tgtYawRate, -cfg.rateGains.max, cfg.rateGains.max);
  const desRateZ = clamp(cfg.attitudeGains.p * (tgtRollInternal - eul.roll), -cfg.rateGains.max, cfg.rateGains.max);
  const desRate = v3(desRateX, desRateY, desRateZ);

  const err = {
    x: desRate.x - state.omega.x,
    y: desRate.y - state.omega.y,
    z: desRate.z - state.omega.z,
  };
  ctrl.rateIntegrator.x = clamp(ctrl.rateIntegrator.x + err.x * dt, -2, 2);
  ctrl.rateIntegrator.y = clamp(ctrl.rateIntegrator.y + err.y * dt, -2, 2);
  ctrl.rateIntegrator.z = clamp(ctrl.rateIntegrator.z + err.z * dt, -2, 2);
  const dErr = {
    x: (err.x - ctrl.ratePrevError.x) / Math.max(dt, 1e-6),
    y: (err.y - ctrl.ratePrevError.y) / Math.max(dt, 1e-6),
    z: (err.z - ctrl.ratePrevError.z) / Math.max(dt, 1e-6),
  };
  ctrl.ratePrevError = err;
  const kp = cfg.rateGains.p;
  const ki = cfg.rateGains.i;
  const kd = cfg.rateGains.d;
  const Mx = kp * err.x + ki * ctrl.rateIntegrator.x + kd * dErr.x;
  const My = kp * err.y + ki * ctrl.rateIntegrator.y + kd * dErr.y;
  const Mz = kp * err.z + ki * ctrl.rateIntegrator.z + kd * dErr.z;

  const m = totalMass(cfg);
  let climbCmd;
  if (guidance?.holdAltitude) {
    climbCmd = clamp(cfg.posKp * (guidance.targetAltitude - state.pos.y), -3, 3);
  } else if (pilot.holdAltitude) {
    climbCmd = clamp(cfg.posKp * (pilot.targetAltitude - state.pos.y), -3, 3);
  } else {
    climbCmd = (pilot.thr - 0.5) * 4.0;
  }
  const climbErr = climbCmd - state.vel.y;
  ctrl.altIntegrator = clamp(ctrl.altIntegrator + climbErr * dt, -3, 3);
  const Fy_extra =
    cfg.altitudeGains.p * climbErr +
    cfg.altitudeGains.d * (-state.vel.y) +
    0.5 * ctrl.altIntegrator;
  const upBody = qRotate(state.quat, v3(0, 1, 0));
  const tiltFactor = clamp(upBody.y, 0.3, 1);
  const Fy = m * cfg.gravity / tiltFactor + Fy_extra;

  const cmd = solveAllocator({ Fy, Mx, My, Mz }, cfg, positions, directions, ctrl.cache);
  return {
    cmd,
    debug: {
      wrench: { Fy, Mx, My, Mz },
      desRate,
      climbCmd,
      attErr: { roll: desiredRoll + eul.roll, pitch: desiredPitch - eul.pitch },
      target: guidance ? {
        roll: desiredRoll,
        pitch: desiredPitch,
        altitude: guidance.targetAltitude,
        heading: guidance.targetHeading,
      } : null,
    },
  };
}

function computeWaypointGuidance(state, cfg, pilot, dt) {
  const mission = pilot.mission;
  const eul = qToEuler(state.quat);

  if (!mission.holdPoint && (!mission.active || mission.waypoints.length === 0)) {
    setMissionHold(mission, state, eul.yaw);
  }

  if (!mission.active || mission.waypoints.length === 0) {
    if (!mission.active) mission.status = mission.waypoints.length ? "ready" : "idle";
    if (!mission.waypoints.length) mission.currentIndex = 0;
    return holdPointGuidance(mission.holdPoint);
  }

  mission.holdPoint = null;
  if (mission.status === "armed") mission.lastHeading = eul.yaw;

  let wp = mission.waypoints[mission.currentIndex];
  if (!wp) {
    if (mission.loop && mission.waypoints.length > 0) {
      mission.currentIndex = 0;
      mission.holdTimer = 0;
      wp = mission.waypoints[0];
    } else {
      mission.active = false;
      mission.status = "complete";
      setMissionHold(mission, state, eul.yaw);
      return holdPointGuidance(mission.holdPoint);
    }
  }

  const speed = getWaypointNumber(wp.speed, mission.defaultSpeed);
  const wait = Math.max(0, getWaypointNumber(wp.wait, mission.defaultWait));
  const dx = wp.x - state.pos.x;
  const dy = wp.y - state.pos.y;
  const dz = wp.z - state.pos.z;
  const horizDist = Math.hypot(dx, dz);
  const horizSpeed = Math.hypot(state.vel.x, state.vel.z);
  const arriveRadius = Math.max(0.22, Math.min(0.6, 0.2 + speed * 0.12));
  const verticalTol = 0.18;

  const inWaitWindow =
    horizDist < arriveRadius &&
    Math.abs(dy) < verticalTol &&
    horizSpeed < Math.max(0.25, speed * 0.55);
  if (mission.holdTimer > 0 || inWaitWindow) {
    mission.holdTimer += dt;
    mission.status = wait > 0 ? `wait ${mission.currentIndex + 1}/${mission.waypoints.length}` : `reached ${mission.currentIndex + 1}/${mission.waypoints.length}`;
    if (mission.holdTimer >= wait) {
      mission.currentIndex += 1;
      mission.holdTimer = 0;
      if (mission.currentIndex >= mission.waypoints.length) {
        if (mission.loop && mission.waypoints.length > 0) {
          mission.currentIndex = 0;
        } else {
          mission.active = false;
          mission.status = "complete";
          setMissionHold(mission, state, eul.yaw);
          return holdPointGuidance(mission.holdPoint);
        }
      }
      wp = mission.waypoints[mission.currentIndex];
    }
  } else {
    mission.holdTimer = 0;
    mission.status = `waypoint ${mission.currentIndex + 1}/${mission.waypoints.length}`;
  }

  const tx = wp.x - state.pos.x;
  const ty = wp.y - state.pos.y;
  const tz = wp.z - state.pos.z;
  const dist = Math.hypot(tx, tz);
  const dirX = dist > 1e-6 ? tx / dist : 0;
  const dirZ = dist > 1e-6 ? tz / dist : 0;
  const heading = mission.lastHeading;

  const desiredSpeed = dist > arriveRadius
    ? Math.min(speed, Math.sqrt(Math.max(0, 2 * mission.maxAccel * dist)))
    : 0;
  const desiredVelX = dirX * desiredSpeed;
  const desiredVelZ = dirZ * desiredSpeed;
  const accelX = clamp((desiredVelX - state.vel.x) * mission.velGain, -mission.maxAccel, mission.maxAccel);
  const accelZ = clamp((desiredVelZ - state.vel.z) * mission.velGain, -mission.maxAccel, mission.maxAccel);

  const upWorld = normalize3({ x: accelX / cfg.gravity, y: 1, z: accelZ / cfg.gravity });
  const c = Math.cos(heading);
  const s = Math.sin(heading);
  const localUpX = c * upWorld.x - s * upWorld.z;
  const localUpZ = s * upWorld.x + c * upWorld.z;
  const maxTilt = mission.maxTiltDeg * Math.PI / 180;
  const targetRoll = clamp(Math.asin(clamp(localUpX, -0.55, 0.55)), -maxTilt, maxTilt);
  const targetPitch = clamp(-Math.asin(clamp(localUpZ, -0.55, 0.55)), -maxTilt, maxTilt);

  return {
    holdAttitude: true,
    holdAltitude: true,
    holdHeading: true,
    targetAltitude: wp.y,
    targetHeading: heading,
    targetRoll,
    targetPitch,
    debug: {
      mission: {
        index: mission.currentIndex,
        count: mission.waypoints.length,
        status: mission.status,
        distance: dist,
        altitudeError: ty,
        speed,
        wait,
      },
    },
  };
}

function holdPointGuidance(holdPoint) {
  return {
    holdAttitude: true,
    holdAltitude: true,
    holdHeading: true,
    targetAltitude: holdPoint.y,
    targetHeading: holdPoint.heading,
    targetRoll: 0,
    targetPitch: 0,
    debug: {
      mission: {
        index: 0,
        count: 0,
        status: "hold",
        distance: 0,
        altitudeError: 0,
        speed: 0,
        wait: 0,
      },
    },
  };
}

function setMissionHold(mission, state, heading) {
  mission.holdPoint = {
    x: state.pos.x,
    y: state.pos.y,
    z: state.pos.z,
    heading,
  };
  mission.lastHeading = heading;
  mission.holdTimer = 0;
}

function getWaypointNumber(value, fallback) {
  return Number.isFinite(value) ? value : fallback;
}

function normalize3(v) {
  const L = Math.hypot(v.x, v.y, v.z);
  if (L < 1e-9) return v3(0, 1, 0);
  return v3(v.x / L, v.y / L, v.z / L);
}

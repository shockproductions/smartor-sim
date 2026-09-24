// Entry point. Boots subsystems, owns the simulation loop, and wires
// parameter UI / telemetry / plots to the live state.

import { defaultConfig, newState, totalMass, rotorPositions, rotorDirections } from "./vehicle.js";
import { step } from "./physics.js";
import { newPilot, newCtrlState, computeCommand, markAllocatorDirty } from "./controller.js";
import { hoverTrim } from "./allocator.js";
import { Viz } from "./visualization.js";
import { Plot, PlotRack } from "./plots.js";
import { ParamUI, TelemetryUI, ControlUI } from "./ui.js";
import { qToEuler, deg } from "./mathx.js";

const cfg = defaultConfig();
let state = newState();
const pilot = newPilot();
const ctrl = newCtrlState();

// --- Visualization ---
const viewport = document.getElementById("viewport");
const viz = new Viz(viewport);
viz.rebuildVehicle(cfg);

// --- UI ---
const paramUI = new ParamUI(document.getElementById("params-root"), { cfg, pilot }, {
  onChange: (kind) => {
    if (kind === "allocator") markAllocatorDirty(ctrl);
  },
  onGeometryChange: () => {
    viz.rebuildVehicle(cfg);
    markAllocatorDirty(ctrl);
  },
});
const tlm = new TelemetryUI(document.getElementById("telemetry-root"));
const ctlUI = new ControlUI(document.getElementById("control-root"), pilot, cfg, () => state);

// --- Plots ---
const plotsRoot = document.getElementById("plots-root");
const rack = new PlotRack(plotsRoot);
const plots = {
  inputs: new Plot({ title: "Pilot Inputs", series: [
    { label: "thr" }, { label: "roll" }, { label: "pitch" }, { label: "yaw" },
  ]}),
  motors: new Plot({ title: "Motor Throttles", series: [
    { label: "central" }, { label: "out0" }, { label: "out1" }, { label: "out2" },
  ]}),
  thrust: new Plot({ title: "Per-Rotor Thrust (N)", series: [
    { label: "central" }, { label: "out0" }, { label: "out1" }, { label: "out2" },
  ]}),
  attitude: new Plot({ title: "Attitude (deg)", series: [
    { label: "roll" }, { label: "pitch" }, { label: "yaw" },
  ]}),
  rates: new Plot({ title: "Body Rates (rad/s)", series: [
    { label: "p (X)" }, { label: "q (Y)" }, { label: "r (Z)" },
  ]}),
  forces: new Plot({ title: "Body Moments (N·m)", series: [
    { label: "Mx" }, { label: "My" }, { label: "Mz" },
  ]}),
  altitude: new Plot({ title: "Altitude / Vert.Speed", series: [
    { label: "alt (m)" }, { label: "vy (m/s)" },
  ]}),
  power: new Plot({ title: "Power (W est.)", series: [
    { label: "total" },
  ]}),
};
for (const k of Object.keys(plots)) rack.add(plots[k]);

// --- Top bar runtime controls ---
const btnPlay = document.getElementById("btn-play");
const btnStep = document.getElementById("btn-step");
const btnResetHover = document.getElementById("btn-reset-hover");
const btnResetGround = document.getElementById("btn-reset-ground");
const tsRange = document.getElementById("time-scale");
const tsLabel = document.getElementById("time-scale-val");
const statusLine = document.getElementById("status-line");
const freezeChk = document.getElementById("freeze-plots");
const plotWindow = document.getElementById("plot-window");
const plotWindowVal = document.getElementById("plot-window-val");
let paused = false;
let stepOnce = false;
let timeScale = 1;

btnPlay.addEventListener("click", () => {
  paused = !paused;
  btnPlay.textContent = paused ? "Play" : "Pause";
});
btnStep.addEventListener("click", () => { paused = true; btnPlay.textContent = "Play"; stepOnce = true; });
btnResetHover.addEventListener("click", () => resetTo("hover"));
btnResetGround.addEventListener("click", () => resetTo("ground"));
tsRange.addEventListener("input", () => {
  timeScale = parseFloat(tsRange.value);
  tsLabel.textContent = timeScale.toFixed(2) + "x";
});
freezeChk.addEventListener("change", () => rack.setFrozen(freezeChk.checked));
plotWindow.addEventListener("input", () => {
  rack.setWindow(parseInt(plotWindow.value, 10));
  plotWindowVal.textContent = plotWindow.value + "s";
});
rack.setWindow(10);

// Resizable splitter between 3D viewport and the bottom plots panel.
(() => {
  const splitter = document.getElementById("splitter");
  const app = document.getElementById("app");
  // Initial value matches the CSS fallback.
  const TOP_BAR = 38, GAP = 1, SPLIT_H = 6, MIN_BOTTOM = 60;
  const ctlEl = document.getElementById("control-panel");

  let dragging = false;
  splitter.addEventListener("mousedown", (e) => {
    dragging = true;
    splitter.classList.add("dragging");
    document.body.classList.add("resizing-rows");
    e.preventDefault();
  });
  window.addEventListener("mouseup", () => {
    if (!dragging) return;
    dragging = false;
    splitter.classList.remove("dragging");
    document.body.classList.remove("resizing-rows");
  });
  window.addEventListener("mousemove", (e) => {
    if (!dragging) return;
    const ctlH = ctlEl.getBoundingClientRect().height;
    const winH = window.innerHeight;
    // Bottom panel goes from (mouseY + SPLIT_H) down to (winH - ctlH).
    let h = winH - ctlH - GAP - e.clientY - SPLIT_H;
    // Cap so the 3D viewport keeps at least 120px of vertical space.
    const minViewport = 120;
    const maxBottom = winH - TOP_BAR - GAP - SPLIT_H - GAP - ctlH - GAP - minViewport;
    h = Math.max(MIN_BOTTOM, Math.min(maxBottom, h));
    app.style.setProperty("--bottom-h", h + "px");
    viz.onResize();
  });
  // Touch support (mobile).
  splitter.addEventListener("touchstart", (e) => { dragging = true; e.preventDefault(); }, { passive: false });
  window.addEventListener("touchend", () => { dragging = false; });
  window.addEventListener("touchmove", (e) => {
    if (!dragging || !e.touches[0]) return;
    const winH = window.innerHeight;
    const ctlH = ctlEl.getBoundingClientRect().height;
    let h = winH - ctlH - GAP - e.touches[0].clientY - SPLIT_H;
    h = Math.max(MIN_BOTTOM, Math.min(winH - 220, h));
    app.style.setProperty("--bottom-h", h + "px");
    viz.onResize();
  }, { passive: false });
})();

document.getElementById("cam-mode").addEventListener("change", (e) => viz.setCameraMode(e.target.value));
document.getElementById("show-thrust").addEventListener("change", (e) => viz.setShow({ thrust: e.target.checked }));
document.getElementById("show-net").addEventListener("change", (e) => viz.setShow({ net: e.target.checked }));
document.getElementById("show-omega").addEventListener("change", (e) => viz.setShow({ omega: e.target.checked }));
document.getElementById("show-velocity").addEventListener("change", (e) => viz.setShow({ velocity: e.target.checked }));
document.getElementById("show-axes").addEventListener("change", (e) => viz.setShow({ axes: e.target.checked }));

window.addEventListener("keydown", (e) => {
  if (e.code === "KeyR") resetTo("hover");
});

function resetTo(kind) {
  state = newState();
  ctrl.rateIntegrator = { x: 0, y: 0, z: 0 };
  ctrl.ratePrevError = { x: 0, y: 0, z: 0 };
  ctrl.altIntegrator = 0;
  pilot.mission.active = false;
  pilot.mission.holdPoint = null;
  pilot.mission.status = "idle";
  pilot.mission.holdTimer = 0;
  if (kind === "hover") {
    state.pos.y = 1.5;
    // Pre-load pilot.thr so stabilize mode commands ~0 climb-rate.
    pilot.thr = 0.5;
    // Pre-load actuators with hover trim so we don't see the first transient.
    const positions = rotorPositions(cfg);
    const directions = rotorDirections(cfg);
    const trim = hoverTrim(cfg, positions, directions);
    state.act.c = trim.c; state.act.o = [...trim.o];
    state.cmd = { c: trim.c, o: [...trim.o] };
  } else {
    state.pos.y = 0;
    pilot.thr = 0;
  }
  ctlUI.syncFields();
  rack.clear();
}
resetTo("hover");

// --- HUD ---
const hud = document.getElementById("hud");
function updateHUD(state, cfg) {
  const eul = qToEuler(state.quat);
  const m = totalMass(cfg);
  hud.textContent =
`mode  ${cfg.controlMode.padEnd(10)}  t=${state.time.toFixed(1)}s
pos   x=${state.pos.x.toFixed(2)} y=${state.pos.y.toFixed(2)} z=${state.pos.z.toFixed(2)}
vel   ${state.vel.x.toFixed(2)} ${state.vel.y.toFixed(2)} ${state.vel.z.toFixed(2)}  m/s
att   r=${deg(eul.roll).toFixed(1)}° p=${deg(eul.pitch).toFixed(1)}° y=${deg(eul.yaw).toFixed(1)}°
mass  ${m.toFixed(2)} kg   weight ${(m*cfg.gravity).toFixed(1)} N
thr   c=${state.act.c.toFixed(2)} o=[${state.act.o.map(v=>v.toFixed(2)).join(" ")}]`;
}

// --- Simulation loop ---
const PHYSICS_HZ = 500;
const PHYSICS_DT = 1 / PHYSICS_HZ;
const MAX_SUBSTEPS = 12;       // cap to avoid spiral-of-death after pause
let lastWall = performance.now();
let simAccumulator = 0;

function frame(now) {
  const dtWall = Math.min(0.1, (now - lastWall) / 1000);
  lastWall = now;

  ctlUI.tick(dtWall);

  if (!paused || stepOnce) {
    let advance = paused && stepOnce ? PHYSICS_DT : dtWall * timeScale;
    stepOnce = false;
    simAccumulator += advance;
    let n = 0;
    while (simAccumulator >= PHYSICS_DT && n < MAX_SUBSTEPS) {
      computeCommand(state, cfg, pilot, ctrl, PHYSICS_DT);
      step(state, cfg, PHYSICS_DT);
      simAccumulator -= PHYSICS_DT;
      n++;
      pushSamples(state, cfg);
    }
    // If we've fallen >MAX_SUBSTEPS behind, drop the rest.
    if (simAccumulator > PHYSICS_DT * MAX_SUBSTEPS) simAccumulator = 0;
  }

  viz.update(state, cfg, dtWall);
  updateHUD(state, cfg);
  updateTelemetry();
  rack.render(state.time);
  statusLine.textContent = paused ? "paused" : `running ${timeScale.toFixed(2)}x`;

  requestAnimationFrame(frame);
}

function pushSamples(state, cfg) {
  const t = state.time;
  plots.inputs.push(t, [pilot.thr, pilot.roll, pilot.pitch, pilot.yaw]);
  plots.motors.push(t, [state.act.c, state.act.o[0], state.act.o[1], state.act.o[2]]);
  plots.thrust.push(t, [
    state.diag.thrustMag.c,
    state.diag.thrustMag.o[0],
    state.diag.thrustMag.o[1],
    state.diag.thrustMag.o[2],
  ]);
  const e = qToEuler(state.quat);
  plots.attitude.push(t, [deg(e.roll), deg(e.pitch), deg(e.yaw)]);
  plots.rates.push(t, [state.omega.x, state.omega.y, state.omega.z]);
  plots.forces.push(t, [state.diag.netMomentBody.x, state.diag.netMomentBody.y, state.diag.netMomentBody.z]);
  plots.altitude.push(t, [state.pos.y, state.vel.y]);
  plots.power.push(t, [state.diag.power]);
}

function updateTelemetry() {
  const e = qToEuler(state.quat);
  const m = totalMass(cfg);
  const weight = m * cfg.gravity;
  const Fy_body_avail =
    state.diag.thrustMag.c +
    (state.diag.thrustMag.o[0] + state.diag.thrustMag.o[1] + state.diag.thrustMag.o[2]) *
      Math.cos(cfg.outerTiltDeg * Math.PI / 180);
  const hoverMargin = (Fy_body_avail - weight) / weight;

  // Worst-case tangential yaw authority at full outer throttle.
  const tangentialMax = 3 * cfg.outerMaxThrust * Math.sin(cfg.outerTiltDeg * Math.PI / 180);
  const yawAuthorityMax = tangentialMax * cfg.outerRotorRadius - cfg.centralKq * cfg.centralMaxThrust;

  tlm.setRow("alt", state.pos.y.toFixed(2) + " m");
  tlm.setRow("vert_speed", state.vel.y.toFixed(2) + " m/s");
  tlm.setRow("horiz_speed", Math.hypot(state.vel.x, state.vel.z).toFixed(2) + " m/s");
  tlm.setRow("roll", deg(e.roll).toFixed(1) + "°");
  tlm.setRow("pitch", deg(e.pitch).toFixed(1) + "°");
  tlm.setRow("yaw", deg(e.yaw).toFixed(1) + "°");
  tlm.setRow("p", state.omega.x.toFixed(2));
  tlm.setRow("q", state.omega.y.toFixed(2));
  tlm.setRow("r", state.omega.z.toFixed(2));

  tlm.setRow("thr_c", state.diag.thrustMag.c.toFixed(2) + " N");
  tlm.setRow("thr_o0", state.diag.thrustMag.o[0].toFixed(2) + " N");
  tlm.setRow("thr_o1", state.diag.thrustMag.o[1].toFixed(2) + " N");
  tlm.setRow("thr_o2", state.diag.thrustMag.o[2].toFixed(2) + " N");
  tlm.setRow("cmd_c", state.act.c.toFixed(2));
  tlm.setRow("cmd_o0", state.act.o[0].toFixed(2));
  tlm.setRow("cmd_o1", state.act.o[1].toFixed(2));
  tlm.setRow("cmd_o2", state.act.o[2].toFixed(2));
  tlm.setBar("cmd_c", state.act.c, state.act.c > 0.85);
  tlm.setBar("cmd_o0", state.act.o[0], state.act.o[0] > 0.85);
  tlm.setBar("cmd_o1", state.act.o[1], state.act.o[1] > 0.85);
  tlm.setBar("cmd_o2", state.act.o[2], state.act.o[2] > 0.85);
  tlm.setRow("power", state.diag.power.toFixed(0) + " W");

  tlm.setRow("Fy_body", Fy_body_avail.toFixed(1) + " N");
  tlm.setRow("Mx", state.diag.netMomentBody.x.toFixed(2));
  tlm.setRow("My", state.diag.netMomentBody.y.toFixed(2));
  tlm.setRow("Mz", state.diag.netMomentBody.z.toFixed(2));
  tlm.setRow("hover_margin", (hoverMargin * 100).toFixed(1) + "%");
  tlm.setRow("yaw_authority", yawAuthorityMax.toFixed(2) + " N·m max");

  tlm.setRow("mode", cfg.controlMode);
  const sat = state.diag.saturationFlags;
  tlm.setRow("sat_warn", sat.some(Boolean) ? "SAT " + sat.map((s,i)=> s?["c","o0","o1","o2"][i]:"").filter(Boolean).join(",") : "ok");
}

requestAnimationFrame((t) => { lastWall = t; requestAnimationFrame(frame); });

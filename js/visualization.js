// 3D scene for SMaRTORsim. Builds the vehicle from the live config so geometry
// changes (rotor diameters, radius, tilt) update instantly. Renders debug
// vectors for thrust, net force/moment, ω, and velocity.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { rad } from "./mathx.js";
import { rotorPositions, rotorDirections } from "./vehicle.js";

const COLORS = {
  central: 0x4cc2ff,
  outer: 0x7ee787,
  thrust: 0xffc857,
  netForce: 0xff6b6b,
  netMoment: 0xa472ff,
  omega: 0x62d2a2,
  velocity: 0x4cc2ff,
  body: 0x9aa6c0,
};

export class Viz {
  constructor(container) {
    this.container = container;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0a0e14);
    this.scene.fog = new THREE.Fog(0x0a0e14, 30, 120);

    this.camera = new THREE.PerspectiveCamera(55, 1, 0.05, 1000);
    this.camera.position.set(2.5, 2.0, 3.0);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 1, 0);
    this.controls.enableDamping = true;

    // Lighting.
    const hemi = new THREE.HemisphereLight(0x99c0ff, 0x1a1a25, 0.5);
    this.scene.add(hemi);
    const dir = new THREE.DirectionalLight(0xffffff, 0.9);
    dir.position.set(4, 8, 6);
    this.scene.add(dir);

    // Ground. Recenter a fixed grid/floor footprint under the vehicle so we
    // get effectively endless terrain without allocating more geometry.
    this.groundSnap = 10;
    this.ground = new THREE.Group();
    this.scene.add(this.ground);
    this.grid = new THREE.GridHelper(80, 80, 0x444c5e, 0x222732);
    this.grid.position.y = 0;
    this.ground.add(this.grid);
    this.floor = new THREE.Mesh(
      new THREE.CircleGeometry(40, 96),
      new THREE.MeshBasicMaterial({ color: 0x0a0e14, transparent: true, opacity: 0.6 })
    );
    this.floor.rotation.x = -Math.PI / 2;
    this.floor.position.y = -0.001;
    this.ground.add(this.floor);

    // World axes (small helper at origin).
    this.scene.add(makeAxes(0.6));

    // Vehicle root.
    this.vehicleRoot = new THREE.Group();
    this.scene.add(this.vehicleRoot);

    // Body axes attached to vehicle.
    this.bodyAxes = makeAxes(0.4);
    this.vehicleRoot.add(this.bodyAxes);

    // Will be (re)built when config changes.
    this.rotorMeshes = [];
    this.rotorBlades = [];
    this.armMeshes = [];
    this.frameMesh = null;
    this.thrustArrows = [];
    this.netForceArrow = makeArrow(COLORS.netForce, 1, 0.04);
    this.netMomentArrow = makeArrow(COLORS.netMoment, 1, 0.04);
    this.omegaArrow = makeArrow(COLORS.omega, 1, 0.04);
    this.velocityArrow = makeArrow(COLORS.velocity, 1, 0.04);

    this.scene.add(this.netForceArrow.group);
    this.vehicleRoot.add(this.netMomentArrow.group);
    this.vehicleRoot.add(this.omegaArrow.group);
    this.scene.add(this.velocityArrow.group);

    this.show = {
      thrust: true, net: true, omega: false, velocity: false, axes: true,
    };

    this.cameraMode = "orbit";
    this.lastCfg = null;

    this.onResize = this.onResize.bind(this);
    window.addEventListener("resize", this.onResize);
    this.onResize();
  }

  onResize() {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / Math.max(h, 1);
    this.camera.updateProjectionMatrix();
  }

  setShow(opt) { Object.assign(this.show, opt); }

  setCameraMode(mode) { this.cameraMode = mode; }

  rebuildVehicle(cfg) {
    // Remove old children.
    for (const m of this.rotorMeshes) this.vehicleRoot.remove(m);
    for (const m of this.armMeshes) this.vehicleRoot.remove(m);
    for (const a of this.thrustArrows) this.vehicleRoot.remove(a.group);
    if (this.frameMesh) this.vehicleRoot.remove(this.frameMesh);

    this.rotorMeshes = [];
    this.rotorBlades = [];
    this.armMeshes = [];
    this.thrustArrows = [];

    const positions = rotorPositions(cfg);
    const directions = rotorDirections(cfg);

    // Central frame: a stubby tube under the central rotor.
    const frameGeo = new THREE.CylinderGeometry(0.04, 0.06, 0.18, 16);
    const frameMat = new THREE.MeshStandardMaterial({ color: 0x2a3344, roughness: 0.6, metalness: 0.3 });
    this.frameMesh = new THREE.Mesh(frameGeo, frameMat);
    this.frameMesh.position.set(0, 0, 0);
    this.vehicleRoot.add(this.frameMesh);

    // Arms from CoM to each outer rotor.
    for (let i = 1; i <= 3; i++) {
      const p = positions[i];
      const len = Math.hypot(p.x, p.y, p.z);
      const armGeo = new THREE.CylinderGeometry(0.012, 0.012, len, 12);
      const armMat = new THREE.MeshStandardMaterial({ color: 0x3a4860, roughness: 0.7 });
      const arm = new THREE.Mesh(armGeo, armMat);
      // Cylinder by default is along Y. Rotate to point from origin to p.
      const dir = new THREE.Vector3(p.x, p.y, p.z).normalize();
      const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
      arm.quaternion.copy(q);
      arm.position.set(p.x / 2, p.y / 2, p.z / 2);
      this.armMeshes.push(arm);
      this.vehicleRoot.add(arm);
    }

    // Rotors: a thin disk + two blade strokes that spin.
    for (let i = 0; i < 4; i++) {
      const isCentral = (i === 0);
      const radius = isCentral ? cfg.centralRotorDiameter / 2 : cfg.outerRotorDiameter / 2;
      const color = isCentral ? COLORS.central : COLORS.outer;

      const group = new THREE.Group();
      const p = positions[i];
      group.position.set(p.x, p.y, p.z);

      // Hub disk.
      const hub = new THREE.Mesh(
        new THREE.CylinderGeometry(radius * 0.15, radius * 0.15, 0.02, 16),
        new THREE.MeshStandardMaterial({ color: 0x222933, roughness: 0.5 }),
      );
      group.add(hub);

      // Disc (translucent).
      const disc = new THREE.Mesh(
        new THREE.CircleGeometry(radius, 48),
        new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.10, side: THREE.DoubleSide }),
      );
      disc.rotation.x = -Math.PI / 2;
      group.add(disc);

      // Disc edge ring.
      const ringGeo = new THREE.TorusGeometry(radius, radius * 0.012, 8, 48);
      const ring = new THREE.Mesh(ringGeo, new THREE.MeshBasicMaterial({ color }));
      ring.rotation.x = Math.PI / 2;
      group.add(ring);

      // Blades (just a flat strip across the disc, spun by command).
      const blade = new THREE.Group();
      const bladeMesh = new THREE.Mesh(
        new THREE.BoxGeometry(radius * 1.9, 0.005, radius * 0.08),
        new THREE.MeshStandardMaterial({ color: 0xb8c5da, roughness: 0.4 }),
      );
      blade.add(bladeMesh);
      group.add(blade);
      this.rotorBlades.push(blade);

      // Tilt outer rotors to match thrust direction.
      if (!isCentral) {
        const d = directions[i];
        const dvec = new THREE.Vector3(d.x, d.y, d.z).normalize();
        const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dvec);
        group.quaternion.copy(q);
      }

      this.rotorMeshes.push(group);
      this.vehicleRoot.add(group);

      // Per-rotor thrust arrow, parented to the rotor so it follows tilt.
      const arrow = makeArrow(COLORS.thrust, 0.001, 0.025);
      group.add(arrow.group);
      this.thrustArrows.push(arrow);
    }

    this.lastCfg = cfg;
  }

  // Update vehicle pose, blade spin, and debug arrows.
  update(state, cfg, dt) {
    if (this.lastCfg !== cfg) this.rebuildVehicle(cfg);

    this.vehicleRoot.position.set(state.pos.x, state.pos.y, state.pos.z);
    this.vehicleRoot.quaternion.set(state.quat.x, state.quat.y, state.quat.z, state.quat.w);

    // Blade spin (visual only).
    const baseRpm = 2 * Math.PI * 12; // when act=1, ~12 rev/s
    const central = state.act.c;
    this.rotorBlades[0].rotation.y += central * baseRpm * dt;
    for (let i = 0; i < 3; i++) {
      this.rotorBlades[i + 1].rotation.y += state.act.o[i] * baseRpm * dt * cfg.outerSpinSign * -1; // inverse direction visually
    }

    // Thrust arrows (body frame, in each rotor's local space). Each rotor group
    // has its quaternion already aligned to thrust direction, so a thrust
    // vector along the local +Y maps to the actual thrust. Length scales with
    // current thrust magnitude.
    const showThrust = this.show.thrust;
    for (let i = 0; i < 4; i++) {
      const mag = i === 0 ? state.diag.thrustMag.c : state.diag.thrustMag.o[i - 1];
      const max = i === 0 ? cfg.centralMaxThrust : cfg.outerMaxThrust;
      const length = (mag / Math.max(max, 1e-3)) * 0.6 + 0.001;
      this.thrustArrows[i].group.visible = showThrust;
      setArrow(this.thrustArrows[i], { x: 0, y: 1, z: 0 }, length);
    }

    this.bodyAxes.visible = this.show.axes;

    // Net force in world frame, anchored at vehicle CoM.
    const F = state.diag.netForceWorld;
    const Fmag = Math.hypot(F.x, F.y, F.z);
    this.netForceArrow.group.visible = this.show.net && Fmag > 0.05;
    if (this.netForceArrow.group.visible) {
      this.netForceArrow.group.position.copy(this.vehicleRoot.position);
      const len = Math.min(2.5, Fmag * 0.02);
      setArrow(this.netForceArrow, F, len);
    }

    // Net moment in body frame.
    const M = state.diag.netMomentBody;
    const Mmag = Math.hypot(M.x, M.y, M.z);
    this.netMomentArrow.group.visible = this.show.net && Mmag > 0.01;
    if (this.netMomentArrow.group.visible) {
      const len = Math.min(1.0, Mmag * 0.6);
      setArrow(this.netMomentArrow, M, len);
    }

    // ω (body frame, attached to vehicle).
    const w = state.omega;
    const wmag = Math.hypot(w.x, w.y, w.z);
    this.omegaArrow.group.visible = this.show.omega && wmag > 0.05;
    if (this.omegaArrow.group.visible) setArrow(this.omegaArrow, w, Math.min(1.0, wmag * 0.4));

    // Velocity (world frame).
    const v = state.vel;
    const vmag = Math.hypot(v.x, v.y, v.z);
    this.velocityArrow.group.visible = this.show.velocity && vmag > 0.1;
    if (this.velocityArrow.group.visible) {
      this.velocityArrow.group.position.copy(this.vehicleRoot.position);
      setArrow(this.velocityArrow, v, Math.min(2, vmag * 0.5));
    }

    // Camera follow modes.
    this.updateGround(state);
    this.applyCameraMode(state, dt);

    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  applyCameraMode(state, dt) {
    const target = new THREE.Vector3(state.pos.x, state.pos.y, state.pos.z);
    if (this.cameraMode === "orbit") {
      this.controls.enabled = true;
      this.controls.target.lerp(target, 1 - Math.exp(-dt * 4));
      return;
    }
    if (this.cameraMode === "orbit-chase") {
      this.controls.enabled = true;
      const alpha = 1 - Math.exp(-dt * 5);
      const nextTarget = this.controls.target.clone().lerp(target, alpha);
      const delta = nextTarget.clone().sub(this.controls.target);
      this.controls.target.copy(nextTarget);
      this.camera.position.add(delta);
      return;
    }
    this.controls.enabled = false;

    if (this.cameraMode === "chase") {
      const fwd = new THREE.Vector3(0, 0, 1).applyQuaternion(this.vehicleRoot.quaternion);
      const up = new THREE.Vector3(0, 1, 0);
      const desired = target.clone().add(fwd.multiplyScalar(-3.5)).add(up.multiplyScalar(1.4));
      this.camera.position.lerp(desired, 1 - Math.exp(-dt * 5));
      this.camera.lookAt(target.clone().add(v3ToThree(state.vel, 0.2)));
    } else if (this.cameraMode === "top") {
      this.camera.position.set(state.pos.x, state.pos.y + 6, state.pos.z + 0.001);
      this.camera.lookAt(target);
    } else if (this.cameraMode === "side") {
      this.camera.position.set(state.pos.x + 5, state.pos.y + 0.5, state.pos.z);
      this.camera.lookAt(target);
    } else if (this.cameraMode === "fpv") {
      const fpvOffset = new THREE.Vector3(0, 0.05, 0.05).applyQuaternion(this.vehicleRoot.quaternion);
      this.camera.position.copy(target).add(fpvOffset);
      const lookFwd = new THREE.Vector3(0, 0, 1).applyQuaternion(this.vehicleRoot.quaternion);
      this.camera.lookAt(target.clone().add(lookFwd));
    }
  }

  updateGround(state) {
    const sx = Math.round(state.pos.x / this.groundSnap) * this.groundSnap;
    const sz = Math.round(state.pos.z / this.groundSnap) * this.groundSnap;
    this.ground.position.set(sx, 0, sz);
  }
}

// --- Helpers ---

function makeAxes(len) {
  const g = new THREE.Group();
  const make = (color, dir) => {
    const arrow = makeArrow(color, len, 0.02, true);
    setArrow(arrow, dir, len);
    g.add(arrow.group);
    return arrow;
  };
  make(0xff5d6c, { x: 1, y: 0, z: 0 });
  make(0x6bcb77, { x: 0, y: 1, z: 0 });
  make(0x4d96ff, { x: 0, y: 0, z: 1 });
  return g;
}

function makeArrow(color, length, thickness, slim = false) {
  const group = new THREE.Group();
  const shaftLen = Math.max(length - thickness * 4, 0.001);
  const shaft = new THREE.Mesh(
    new THREE.CylinderGeometry(thickness * (slim ? 0.5 : 1), thickness * (slim ? 0.5 : 1), shaftLen, 8),
    new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.95 })
  );
  shaft.position.y = shaftLen / 2;
  group.add(shaft);
  const head = new THREE.Mesh(
    new THREE.ConeGeometry(thickness * 2.4, thickness * 5.5, 12),
    new THREE.MeshBasicMaterial({ color })
  );
  head.position.y = shaftLen + thickness * 2.7;
  group.add(head);
  return { group, shaft, head, thickness };
}

function setArrow(arrow, dirVec, length) {
  const dir = new THREE.Vector3(dirVec.x, dirVec.y, dirVec.z);
  if (dir.lengthSq() < 1e-12) {
    arrow.group.scale.set(0, 0, 0);
    return;
  }
  dir.normalize();
  const q = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
  arrow.group.quaternion.copy(q);
  // Scale Y axis to set length, leave X/Z at 1.
  const s = Math.max(length, 0.0001);
  arrow.group.scale.set(1, s, 1);
}

function v3ToThree(v, scale = 1) {
  return new THREE.Vector3(v.x * scale, v.y * scale, v.z * scale);
}

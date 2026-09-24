// Builds the parameter editor (left panel), telemetry readout (right panel),
// and pilot/control panel (bottom bar). All controls write directly into the
// live config and pilot objects so changes take effect on the next physics
// substep.

const PARAM_SPEC = [
  {
    section: "Pilot Targets",
    target: "pilot",
    rows: [
      { key: "holdAltitude",   label: "Hold altitude", type: "toggle" },
      { key: "targetAltitude", label: "Target alt",    min: 0, max: 30, step: 0.05, unit: "m" },
      { key: "holdHeading",    label: "Hold heading",  type: "toggle" },
      { key: "targetHeading",  label: "Target hdg",    min: -180, max: 180, step: 1, unit: "deg", scale: "deg" },
      { key: "holdAttitude",   label: "Hold attitude", type: "toggle" },
      { key: "targetRoll",     label: "Target roll",   min: -45, max: 45, step: 0.5, unit: "deg", scale: "deg" },
      { key: "targetPitch",    label: "Target pitch",  min: -45, max: 45, step: 0.5, unit: "deg", scale: "deg" },
    ],
  },
  {
    section: "Geometry",
    rows: [
      { key: "centralRotorDiameter", label: "Center D",   min: 0.2, max: 1.5, step: 0.01, unit: "m" },
      { key: "outerRotorDiameter",   label: "Outer D",    min: 0.05, max: 0.6, step: 0.005, unit: "m" },
      { key: "outerRotorRadius",     label: "Radius R",   min: 0.15, max: 1.5, step: 0.01, unit: "m" },
      { key: "outerTiltDeg",         label: "Tilt",       min: 0, max: 30, step: 0.1, unit: "deg" },
    ],
  },
  {
    section: "Mass / Inertia",
    rows: [
      { key: "frameMass",   label: "Frame",   min: 0.2, max: 5.0, step: 0.01, unit: "kg" },
      { key: "batteryMass", label: "Battery", min: 0.0, max: 3.0, step: 0.01, unit: "kg" },
      { key: "payloadMass", label: "Payload", min: 0.0, max: 3.0, step: 0.01, unit: "kg" },
      { key: "Ixx", label: "Ixx", min: 0.005, max: 0.5, step: 0.001, unit: "kg*m^2" },
      { key: "Iyy", label: "Iyy", min: 0.005, max: 0.5, step: 0.001, unit: "kg*m^2" },
      { key: "Izz", label: "Izz", min: 0.005, max: 0.5, step: 0.001, unit: "kg*m^2" },
    ],
  },
  {
    section: "Propulsion",
    rows: [
      { key: "centralMaxThrust", label: "Center Tmax", min: 5,  max: 120, step: 0.5, unit: "N" },
      { key: "outerMaxThrust",   label: "Outer Tmax",  min: 1,  max: 30,  step: 0.1, unit: "N" },
      { key: "centralKq",        label: "Center kQ",   min: 0,  max: 0.1, step: 0.001, unit: "Nm/N" },
      { key: "outerKq",          label: "Outer kQ",    min: 0,  max: 0.1, step: 0.001, unit: "Nm/N" },
      { key: "motorTau",         label: "Motor tau",   min: 0,  max: 0.4, step: 0.005, unit: "s" },
      { key: "thrustGamma",      label: "Thrust gamma",min: 1,  max: 3,   step: 0.05 },
      { key: "centralSpinSign",  label: "Center spin", type: "select", options: [{v:1,t:"CCW"},{v:-1,t:"CW"}] },
      { key: "outerSpinSign",    label: "Outer spin",  type: "select", options: [{v:1,t:"CCW"},{v:-1,t:"CW"}] },
    ],
  },
  {
    section: "Environment",
    rows: [
      { key: "gravity",      label: "g",          min: 0, max: 25,  step: 0.1, unit: "m/s^2" },
      { key: "airDensity",   label: "rho air",    min: 0.1, max: 1.5, step: 0.01, unit: "kg/m^3" },
      { key: "linearDragK",  label: "Drag (lin)", min: 0,  max: 2,   step: 0.01 },
      { key: "angularDragK", label: "Drag (ang)", min: 0,  max: 0.5, step: 0.001 },
    ],
  },
  {
    section: "Control",
    rows: [
      { key: "controlMode", label: "Mode", type: "select", options: [
        {v: "waypoint", t: "Waypoint"},
        {v: "stabilize", t: "Stabilize"},
        {v: "allocator", t: "Allocator"},
        {v: "mixer", t: "Mixer"},
        {v: "manual", t: "Manual"},
      ]},
      { key: ["rateGains","p"], label: "Rate kP",       min: 0,  max: 0.6, step: 0.005 },
      { key: ["rateGains","i"], label: "Rate kI",       min: 0,  max: 0.3, step: 0.005 },
      { key: ["rateGains","d"], label: "Rate kD",       min: 0,  max: 0.02, step: 0.0005 },
      { key: ["attitudeGains","p"], label: "Att kP",    min: 0,  max: 16,  step: 0.1 },
      { key: ["altitudeGains","p"], label: "Climb kP",  min: 0,  max: 8,   step: 0.05 },
      { key: ["altitudeGains","d"], label: "Climb kD",  min: 0,  max: 8,   step: 0.05 },
      { key: "posKp",     label: "Pos kP (alt)",  min: 0,  max: 4,  step: 0.05 },
      { key: "headingKp", label: "Heading kP",    min: 0,  max: 8,  step: 0.05 },
    ],
  },
];

const ALLOCATOR_KEYS = new Set([
  "centralMaxThrust","outerMaxThrust","centralKq","outerKq",
  "centralSpinSign","outerSpinSign",
  "centralRotorDiameter","outerRotorDiameter","outerRotorRadius","outerTiltDeg",
  "frameMass","batteryMass","payloadMass","gravity",
]);

const GEOMETRY_KEYS = new Set([
  "centralRotorDiameter","outerRotorDiameter","outerRotorRadius","outerTiltDeg",
]);

function getKey(obj, key) {
  if (Array.isArray(key)) return key.reduce((o, k) => o?.[k], obj);
  return obj[key];
}

function setKey(obj, key, value) {
  if (Array.isArray(key)) {
    let o = obj;
    for (let i = 0; i < key.length - 1; i++) o = o[key[i]];
    o[key[key.length - 1]] = value;
  } else {
    obj[key] = value;
  }
}

function flatKey(key) {
  return Array.isArray(key) ? key.join(".") : key;
}

export class ParamUI {
  constructor(root, targets, { onChange, onGeometryChange } = {}) {
    this.root = root;
    this.targets = targets;
    this.onChange = onChange || (() => {});
    this.onGeometryChange = onGeometryChange || (() => {});
    this.fields = {};
    this.build();
  }

  targetFor(section) {
    return this.targets[section.target || "cfg"];
  }

  build() {
    this.root.innerHTML = "";
    for (const section of PARAM_SPEC) {
      const sec = document.createElement("div");
      sec.className = "param-section";
      const head = document.createElement("div");
      head.className = "param-section-header";
      head.textContent = section.section;
      head.addEventListener("click", () => sec.classList.toggle("collapsed"));
      sec.appendChild(head);
      const body = document.createElement("div");
      body.className = "param-section-body";
      sec.appendChild(body);

      const obj = this.targetFor(section);
      for (const row of section.rows) {
        body.appendChild(this.buildRow(row, obj, section.target || "cfg"));
      }
      this.root.appendChild(sec);
    }
  }

  buildRow(row, obj, targetName) {
    const div = document.createElement("div");
    div.className = "param-row";
    const fkey = flatKey(row.key);

    const label = document.createElement("label");
    label.textContent = row.label;
    div.appendChild(label);

    const initial = getKey(obj, row.key);

    if (row.type === "toggle") {
      const cb = document.createElement("input");
      cb.type = "checkbox";
      cb.checked = !!initial;
      cb.addEventListener("change", () => {
        setKey(obj, row.key, cb.checked);
        this.handleChange(row.key, targetName);
      });
      div.style.gridTemplateColumns = "95px 1fr";
      div.appendChild(cb);
      this.fields[targetName + ":" + fkey] = { input: cb, row, target: targetName };
      return div;
    }

    if (row.type === "select") {
      const sel = document.createElement("select");
      for (const opt of row.options) {
        const o = document.createElement("option");
        o.value = String(opt.v);
        o.textContent = opt.t;
        sel.appendChild(o);
      }
      sel.value = String(initial);
      sel.addEventListener("input", () => {
        const orig = row.options.find((o) => String(o.v) === sel.value);
        const v = orig ? orig.v : sel.value;
        setKey(obj, row.key, v);
        this.handleChange(row.key, targetName);
      });
      div.style.gridTemplateColumns = "95px 1fr";
      div.appendChild(sel);
      this.fields[targetName + ":" + fkey] = { input: sel, row, target: targetName };
      return div;
    }

    const isDeg = row.scale === "deg";
    const toUI = (v) => isDeg ? v * 180 / Math.PI : v;
    const fromUI = (v) => isDeg ? v * Math.PI / 180 : v;

    const input = document.createElement("input");
    input.type = "range";
    input.min = row.min;
    input.max = row.max;
    input.step = row.step;
    input.value = String(toUI(initial));

    const valSpan = document.createElement("span");
    valSpan.className = "val";
    const fmt = (uiVal) => {
      const decimals = row.step < 0.01 ? 4 : row.step < 0.1 ? 3 : row.step < 1 ? 2 : 1;
      return Number(uiVal).toFixed(decimals) + (row.unit ? " " + row.unit : "");
    };
    valSpan.textContent = fmt(toUI(initial));

    input.addEventListener("input", () => {
      const uiVal = parseFloat(input.value);
      const stored = fromUI(uiVal);
      setKey(obj, row.key, stored);
      valSpan.textContent = fmt(uiVal);
      this.handleChange(row.key, targetName);
    });

    div.appendChild(input);
    div.appendChild(valSpan);
    this.fields[targetName + ":" + fkey] = { input, val: valSpan, row, fmt, target: targetName, toUI };
    return div;
  }

  handleChange(key, targetName) {
    const flat = flatKey(key);
    if (targetName === "cfg" && !Array.isArray(key) && ALLOCATOR_KEYS.has(key)) this.onChange("allocator");
    if (targetName === "cfg" && !Array.isArray(key) && GEOMETRY_KEYS.has(key)) this.onGeometryChange();
    this.onChange("any", flat);
  }

  refresh() {
    for (const id of Object.keys(this.fields)) {
      const f = this.fields[id];
      const obj = this.targets[f.target];
      const v = getKey(obj, f.row.key);
      if (f.row.type === "toggle") {
        f.input.checked = !!v;
      } else if (f.row.type === "select") {
        f.input.value = String(v);
      } else {
        const uiVal = f.toUI(v);
        f.input.value = String(uiVal);
        if (f.val && f.fmt) f.val.textContent = f.fmt(uiVal);
      }
    }
  }
}

export class TelemetryUI {
  constructor(root) {
    this.root = root;
    this.sections = {};
    this.rows = {};
    this.bars = {};
    this.build();
  }

  build() {
    const sections = [
      { key: "vehicle", title: "Vehicle State", rows: [
        "alt", "vert_speed", "horiz_speed", "roll", "pitch", "yaw", "p", "q", "r",
      ]},
      { key: "rotor", title: "Rotor Outputs", rows: [
        "thr_c","thr_o0","thr_o1","thr_o2","cmd_c","cmd_o0","cmd_o1","cmd_o2","power",
      ]},
      { key: "wrench", title: "Net Forces / Moments", rows: [
        "Fy_body","Mx","My","Mz","hover_margin","yaw_authority",
      ]},
      { key: "ctrl", title: "Controller", rows: [
        "mode","wrench_des","sat_warn",
      ]},
    ];
    for (const s of sections) {
      const sec = document.createElement("div");
      sec.className = "tlm-section";
      const head = document.createElement("div");
      head.className = "tlm-section-header";
      head.textContent = s.title;
      sec.appendChild(head);
      this.sections[s.key] = sec;
      this.root.appendChild(sec);
      for (const r of s.rows) {
        const div = document.createElement("div");
        div.className = "tlm-row";
        const k = document.createElement("span"); k.className = "k"; k.textContent = r;
        const v = document.createElement("span"); v.className = "v"; v.textContent = "--";
        div.appendChild(k); div.appendChild(v);
        sec.appendChild(div);
        this.rows[r] = v;
      }
    }
    for (const k of ["cmd_c","cmd_o0","cmd_o1","cmd_o2"]) {
      const bar = document.createElement("div"); bar.className = "tlm-bar";
      const fill = document.createElement("div"); fill.className = "tlm-bar-fill";
      bar.appendChild(fill);
      this.rows[k].parentElement.after(bar);
      this.bars[k] = fill;
    }
  }

  setRow(key, value) {
    if (!this.rows[key]) return;
    this.rows[key].textContent = value;
  }

  setBar(key, frac, warn = false) {
    const fill = this.bars[key];
    if (!fill) return;
    fill.style.width = Math.max(0, Math.min(100, frac * 100)) + "%";
    fill.className = "tlm-bar-fill" + (frac >= 0.99 ? " bad" : warn ? " warn" : "");
  }
}

export class ControlUI {
  constructor(root, pilot, cfg, getState = null) {
    this.root = root;
    this.pilot = pilot;
    this.cfg = cfg;
    this.getState = getState || (() => null);
    this.build();
    this.installKeyboard();
  }

  build() {
    this.root.innerHTML = "";
    const sliders = [
      { key: "thr",   label: "Throttle", min: 0,  max: 1,  step: 0.005 },
      { key: "roll",  label: "Roll",     min: -1, max: 1,  step: 0.01 },
      { key: "pitch", label: "Pitch",    min: -1, max: 1,  step: 0.01 },
      { key: "yaw",   label: "Yaw",      min: -1, max: 1,  step: 0.01 },
    ];

    const colA = document.createElement("div"); colA.className = "ctl-col";
    const hdr = document.createElement("h4"); hdr.textContent = "Pilot inputs"; colA.appendChild(hdr);
    this.fields = {};
    for (const s of sliders) {
      const row = document.createElement("div"); row.className = "ctl-row";
      const lab = document.createElement("label"); lab.textContent = s.label;
      const inp = document.createElement("input"); inp.type = "range";
      inp.min = s.min; inp.max = s.max; inp.step = s.step;
      inp.value = String(this.pilot[s.key]);
      const val = document.createElement("span"); val.className = "val";
      val.textContent = Number(this.pilot[s.key]).toFixed(2);
      inp.addEventListener("input", () => {
        const v = parseFloat(inp.value);
        this.pilot[s.key] = v;
        val.textContent = v.toFixed(2);
      });
      row.appendChild(lab); row.appendChild(inp); row.appendChild(val);
      colA.appendChild(row);
      this.fields[s.key] = { input: inp, val };
    }
    this.root.appendChild(colA);

    const colB = document.createElement("div"); colB.className = "ctl-col";
    const h2 = document.createElement("h4"); h2.textContent = "Pilot Aux"; colB.appendChild(h2);
    const armRow = document.createElement("div"); armRow.className = "ctl-row";
    const armLab = document.createElement("label"); armLab.textContent = "Armed";
    const armInp = document.createElement("input"); armInp.type = "checkbox"; armInp.checked = this.pilot.armed;
    armInp.addEventListener("change", () => { this.pilot.armed = armInp.checked; });
    armRow.appendChild(armLab); armRow.appendChild(armInp); armRow.appendChild(document.createElement("span"));
    colB.appendChild(armRow);

    const cRow = document.createElement("div"); cRow.className = "ctl-row";
    const cLab = document.createElement("label"); cLab.textContent = "Center";
    const btn = document.createElement("button"); btn.textContent = "Sticks -> 0";
    btn.addEventListener("click", () => {
      this.pilot.roll = 0;
      this.pilot.pitch = 0;
      this.pilot.yaw = 0;
      this.syncFields();
    });
    cRow.appendChild(cLab); cRow.appendChild(btn); cRow.appendChild(document.createElement("span"));
    colB.appendChild(cRow);

    const captureRow = document.createElement("div"); captureRow.className = "ctl-row";
    const captureLab = document.createElement("label"); captureLab.textContent = "Capture";
    const captureBtn = document.createElement("button"); captureBtn.textContent = "Add current";
    captureBtn.addEventListener("click", () => this.addWaypointFromState());
    captureRow.appendChild(captureLab); captureRow.appendChild(captureBtn); captureRow.appendChild(document.createElement("span"));
    colB.appendChild(captureRow);
    this.root.appendChild(colB);

    const colC = document.createElement("div"); colC.className = "ctl-col";
    const h3 = document.createElement("h4"); h3.textContent = "Keys (focus viewport)"; colC.appendChild(h3);
    const keysHelp = document.createElement("div");
    keysHelp.className = "ctl-help";
    keysHelp.innerHTML = `
      <div><span class="kbd">W</span>/<span class="kbd">S</span> throttle +/-</div>
      <div><span class="kbd">A</span>/<span class="kbd">D</span> yaw +/-</div>
      <div><span class="kbd">Up</span>/<span class="kbd">Down</span> pitch +/-</div>
      <div><span class="kbd">Left</span>/<span class="kbd">Right</span> roll +/-</div>
      <div><span class="kbd">Space</span> sticks->0 &nbsp; <span class="kbd">R</span> reset hover</div>
    `;
    colC.appendChild(keysHelp);
    this.root.appendChild(colC);

    const colD = document.createElement("div"); colD.className = "ctl-col ctl-mission";
    const h4 = document.createElement("h4"); h4.textContent = "Waypoint Mission"; colD.appendChild(h4);

    const missionBar = document.createElement("div"); missionBar.className = "mission-toolbar";
    this.missionToggleBtn = document.createElement("button");
    this.missionToggleBtn.addEventListener("click", () => this.toggleMission());
    const addBtn = document.createElement("button"); addBtn.textContent = "Add";
    addBtn.addEventListener("click", () => this.addWaypoint());
    const clearBtn = document.createElement("button"); clearBtn.textContent = "Clear";
    clearBtn.addEventListener("click", () => this.clearWaypoints());
    const loopLabel = document.createElement("label"); loopLabel.className = "mission-loop";
    const loopInp = document.createElement("input"); loopInp.type = "checkbox"; loopInp.checked = this.pilot.mission.loop;
    loopInp.addEventListener("change", () => { this.pilot.mission.loop = loopInp.checked; });
    loopLabel.appendChild(loopInp);
    loopLabel.appendChild(document.createTextNode("Loop"));
    missionBar.appendChild(this.missionToggleBtn);
    missionBar.appendChild(addBtn);
    missionBar.appendChild(clearBtn);
    missionBar.appendChild(loopLabel);
    colD.appendChild(missionBar);

    this.missionStatus = document.createElement("div");
    this.missionStatus.className = "mission-status";
    colD.appendChild(this.missionStatus);

    this.missionList = document.createElement("div");
    this.missionList.className = "mission-list";
    colD.appendChild(this.missionList);
    this.root.appendChild(colD);

    this.renderMission();
    this.syncMissionStatus();
  }

  syncFields() {
    for (const k of ["thr","roll","pitch","yaw"]) {
      const f = this.fields[k];
      if (!f) continue;
      f.input.value = String(this.pilot[k]);
      f.val.textContent = Number(this.pilot[k]).toFixed(2);
    }
    this.syncMissionStatus();
  }

  addWaypoint() {
    const last = this.pilot.mission.waypoints[this.pilot.mission.waypoints.length - 1];
    this.pilot.mission.waypoints.push({
      x: Number.isFinite(last?.x) ? last.x : 0,
      y: Number.isFinite(last?.y) ? last.y : this.pilot.targetAltitude,
      z: Number.isFinite(last?.z) ? last.z : 0,
      speed: last?.speed ?? this.pilot.mission.defaultSpeed,
      wait: last?.wait ?? this.pilot.mission.defaultWait,
    });
    this.renderMission();
  }

  addWaypointFromState() {
    const state = this.getState();
    if (!state) return this.addWaypoint();
    this.pilot.mission.waypoints.push({
      x: round3(state.pos.x),
      y: round3(state.pos.y),
      z: round3(state.pos.z),
      speed: this.pilot.mission.defaultSpeed,
      wait: this.pilot.mission.defaultWait,
    });
    this.renderMission();
  }

  clearWaypoints() {
    this.pilot.mission.waypoints = [];
    this.pilot.mission.currentIndex = 0;
    this.pilot.mission.holdTimer = 0;
    this.pilot.mission.active = false;
    this.pilot.mission.status = "idle";
    this.renderMission();
  }

  toggleMission() {
    const mission = this.pilot.mission;
    if (mission.active) {
      mission.active = false;
      mission.status = "stopped";
    } else if (mission.waypoints.length) {
      mission.active = true;
      mission.currentIndex = Math.min(mission.currentIndex, Math.max(0, mission.waypoints.length - 1));
      mission.holdTimer = 0;
      mission.holdPoint = null;
      mission.status = "armed";
      this.cfg.controlMode = "waypoint";
    }
    this.syncMissionStatus();
  }

  renderMission() {
    const mission = this.pilot.mission;
    this.missionList.innerHTML = "";
    if (!mission.waypoints.length) {
      const empty = document.createElement("div");
      empty.className = "mission-empty";
      empty.textContent = "No waypoints yet";
      this.missionList.appendChild(empty);
      this.syncMissionStatus();
      return;
    }

    mission.waypoints.forEach((wp, index) => {
      const row = document.createElement("div");
      row.className = "mission-row" + (index === mission.currentIndex && mission.active ? " active" : "");

      const head = document.createElement("div");
      head.className = "mission-row-head";
      const title = document.createElement("span");
      title.textContent = `WP ${index + 1}`;
      const delBtn = document.createElement("button");
      delBtn.textContent = "X";
      delBtn.addEventListener("click", () => {
        mission.waypoints.splice(index, 1);
        mission.currentIndex = Math.min(mission.currentIndex, Math.max(0, mission.waypoints.length - 1));
        if (!mission.waypoints.length) {
          mission.active = false;
          mission.status = "idle";
        }
        this.renderMission();
      });
      head.appendChild(title);
      head.appendChild(delBtn);
      row.appendChild(head);

      const fields = [
        { key: "x", label: "X", step: "0.1", optional: false },
        { key: "y", label: "Y", step: "0.1", optional: false },
        { key: "z", label: "Z", step: "0.1", optional: false },
        { key: "speed", label: "V", step: "0.1", optional: true },
        { key: "wait", label: "Wait", step: "0.1", optional: true },
      ];
      for (const field of fields) {
        const wrap = document.createElement("label");
        wrap.className = "mission-field";
        const name = document.createElement("span");
        name.textContent = field.label;
        const input = document.createElement("input");
        input.type = "number";
        input.step = field.step;
        input.value = Number.isFinite(wp[field.key]) ? String(wp[field.key]) : "";
        if (field.optional) input.placeholder = "auto";
        input.addEventListener("input", () => {
          const raw = input.value.trim();
          wp[field.key] = raw === "" && field.optional ? null : parseFloat(raw || "0");
          this.syncMissionStatus();
        });
        wrap.appendChild(name);
        wrap.appendChild(input);
        row.appendChild(wrap);
      }

      this.missionList.appendChild(row);
    });
    this.syncMissionStatus();
  }

  syncMissionStatus() {
    const mission = this.pilot.mission;
    if (this.missionToggleBtn) this.missionToggleBtn.textContent = mission.active ? "Stop" : "Start";
    if (this.missionStatus) {
      const count = mission.waypoints.length;
      const idx = count ? Math.min(mission.currentIndex + 1, count) : 0;
      this.missionStatus.textContent = `mode ${this.cfg.controlMode}  mission ${mission.status}  wp ${idx}/${count}`;
    }
    if (this.missionList) {
      const rows = this.missionList.querySelectorAll(".mission-row");
      rows.forEach((row, index) => row.classList.toggle("active", index === mission.currentIndex && mission.active));
    }
  }

  installKeyboard() {
    const STEP_THR = 0.6;
    const STEP_AXIS = 1.6;
    const held = new Set();
    window.addEventListener("keydown", (e) => {
      held.add(e.code);
      if (e.code === "Space") {
        this.pilot.roll = 0;
        this.pilot.pitch = 0;
        this.pilot.yaw = 0;
        this.syncFields();
        e.preventDefault();
      }
    });
    window.addEventListener("keyup", (e) => held.delete(e.code));
    this.heldKeys = held;
    this.STEP_THR = STEP_THR;
    this.STEP_AXIS = STEP_AXIS;
  }

  tick(dt) {
    const h = this.heldKeys;
    let changed = false;
    if (h.has("KeyW")) { this.pilot.thr = Math.min(1, this.pilot.thr + this.STEP_THR * dt); changed = true; }
    if (h.has("KeyS")) { this.pilot.thr = Math.max(0, this.pilot.thr - this.STEP_THR * dt); changed = true; }
    if (h.has("ArrowUp"))    { this.pilot.pitch = Math.min(1, this.pilot.pitch + this.STEP_AXIS * dt); changed = true; }
    if (h.has("ArrowDown"))  { this.pilot.pitch = Math.max(-1, this.pilot.pitch - this.STEP_AXIS * dt); changed = true; }
    if (h.has("ArrowLeft"))  { this.pilot.roll = Math.max(-1, this.pilot.roll - this.STEP_AXIS * dt); changed = true; }
    if (h.has("ArrowRight")) { this.pilot.roll = Math.min(1, this.pilot.roll + this.STEP_AXIS * dt); changed = true; }
    if (h.has("KeyA")) { this.pilot.yaw = Math.min(1, this.pilot.yaw + this.STEP_AXIS * dt); changed = true; }
    if (h.has("KeyD")) { this.pilot.yaw = Math.max(-1, this.pilot.yaw - this.STEP_AXIS * dt); changed = true; }

    const decay = (val) => {
      const k = 4 * dt;
      if (val > 0) return Math.max(0, val - k);
      if (val < 0) return Math.min(0, val + k);
      return 0;
    };
    if (!h.has("ArrowUp") && !h.has("ArrowDown")) this.pilot.pitch = decay(this.pilot.pitch);
    if (!h.has("ArrowLeft") && !h.has("ArrowRight")) this.pilot.roll = decay(this.pilot.roll);
    if (!h.has("KeyA") && !h.has("KeyD")) this.pilot.yaw = decay(this.pilot.yaw);

    if (changed || this.pilot.pitch !== 0 || this.pilot.roll !== 0 || this.pilot.yaw !== 0) {
      this.syncFields();
    } else {
      this.syncMissionStatus();
    }
  }
}

function round3(v) {
  return Math.round(v * 1000) / 1000;
}

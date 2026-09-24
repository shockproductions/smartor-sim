# SMaRTOR Sim

Browser-based simulation for a SMaRTOR-style multirotor with:

- one central lift rotor
- three outer rotors spaced 120 degrees apart
- outer rotors tilted tangentially to generate yaw counter-torque
- live parameter editing, telemetry, plots, and 3D visualization
- manual, mixer, allocator, stabilize, and waypoint mission modes

The app is intentionally lightweight: it is a static HTML/CSS/JavaScript sim with no build step.

## Try It Online

**https://shockproductions.github.io/smartor-sim/**

Runs in any modern desktop browser. Nothing to install. The site is redeployed by
`.github/workflows/deploy.yml` on every push to `main`, after the smoke tests pass.

## Running Locally

Requirements:

- Python 3
- Node.js 20+ recommended for the test script

Start the local server from the repo root:

```powershell
npm start
```

Then open:

```text
http://127.0.0.1:8000/
```

Why a local server:

- the app uses ES modules
- `index.html` should be served over HTTP instead of opened directly from disk

## Tests

Run the smoke tests with:

```powershell
npm test
```

The smoke test covers:

- hover trim consistency
- mixer axis signs and neutral balance
- allocator axis mapping
- stabilize hover and recovery from tilt
- waypoint mission completion

## What The Sim Does

The sim runs a rigid-body vehicle model at a fixed physics step of 500 Hz inside the browser render loop. The visualization, telemetry, controls, and plots all read from the same live simulation state.

Main runtime features:

- live editing of geometry, mass, propulsion, and controller parameters
- per-rotor thrust and motor command plots
- multiple camera modes including `Orbit Chase`
- moving ground grid that recenters under the vehicle without allocating more geometry
- waypoint mission editor with per-waypoint `X/Y/Z` and optional `speed` and `wait`

## Physics Model

The physics core lives in [js/physics.js](js/physics.js).

State integrated each step:

- world position
- world velocity
- body angular velocity
- body orientation quaternion

Forces and moments included:

- rotor thrust
- rotor reaction torque
- quadratic linear drag
- linear angular damping
- gravity

Integration details:

- translational dynamics use semi-implicit Euler
- rotational dynamics use diagonal inertia and quaternion integration
- actuators are first-order lag elements (`motorTau`)

Ground handling:

- simple hard floor at `y = 0`
- downward velocity is clamped on contact
- light ground friction damps horizontal sliding

This is an engineering-focused overview sim, not a CFD or blade-element model.

## Rotor / Vehicle Model

Vehicle geometry lives in [js/vehicle.js](js/vehicle.js).

Key assumptions:

- world frame is `+Y` up
- body frame matches world frame at zero attitude
- central rotor thrusts along body `+Y`
- three outer rotors are placed at `0`, `120`, and `240` degrees around the body
- outer rotor thrust vectors are tilted tangentially

Rotor model lives in [js/rotor.js](js/rotor.js).

Rotor behavior:

- command is normalized to `0..1`
- thrust is `T = T_max * command^gamma`
- reaction torque is proportional to thrust via `kQ`
- actuator dynamics use a first-order spool model

## Control Modes

Controller logic lives in [js/controller.js](js/controller.js).

Available modes:

- `manual`
  raw stick-style motor command mapping for debugging
- `mixer`
  pilot `thr/roll/pitch/yaw` are mapped through the geometry-aware allocator
- `allocator`
  pilot inputs command body-frame wrench channels directly
- `stabilize`
  attitude + altitude stabilization using outer attitude logic and inner rate PID
- `waypoint`
  mission guidance generates targets and reuses the stabilize stack underneath

Pilot-facing sign conventions:

- positive roll = bank right
- positive pitch = nose up
- positive yaw = yaw right

## Waypoint Mode

Waypoint mode is designed as a simple mission layer above the stabilized controller.

Each waypoint supports:

- `X`
- `Y`
- `Z`
- optional `speed`
- optional `wait`

Mission behavior:

- the vehicle moves toward the current waypoint
- altitude tracks waypoint `Y`
- heading is held stable during translation
- when the vehicle settles inside the arrival window, the wait timer starts
- after wait completes, the next waypoint activates
- the mission can optionally loop

A demo mission is pre-loaded on startup, so pressing `Start` in the mission panel flies it
straight away: climb to 3 m, fly a 5 m box while climbing to 4 m, then return and settle
over the start point (about 28 s). `Demo` restores it after the list has been edited or cleared.
It is defined by `defaultMission()` in [js/controller.js](js/controller.js).

The UI for mission editing lives in [js/ui.js](js/ui.js).

## Visualization

3D rendering lives in [js/visualization.js](js/visualization.js).

Highlights:

- vehicle mesh rebuilds live from geometry parameters
- rotor discs and blade spin are visualized
- thrust, net force, net moment, angular velocity, and velocity vectors can be toggled
- `Orbit`, `Orbit Chase` (default), `Chase`, `Top`, `Side`, and `FPV` camera modes are available
- the ground grid recenters around the vehicle to keep the scene readable over long motion

## UI / App Structure

Entry point:

- [js/main.js](js/main.js)

Important files:

- [index.html](index.html)
  application shell and import map
- [style.css](style.css)
  layout and panel styling
- [js/main.js](js/main.js)
  bootstraps the sim, UI, plots, and runtime loop
- [js/vehicle.js](js/vehicle.js)
  geometry, masses, inertia, and state container
- [js/rotor.js](js/rotor.js)
  rotor thrust and actuator dynamics
- [js/allocator.js](js/allocator.js)
  control effectiveness and motor allocation
- [js/controller.js](js/controller.js)
  control modes and waypoint guidance
- [js/physics.js](js/physics.js)
  rigid-body update step
- [js/visualization.js](js/visualization.js)
  Three.js scene and camera behavior
- [js/ui.js](js/ui.js)
  parameter, telemetry, control, and mission editor UI
- [js/plots.js](js/plots.js)
  lightweight plotting
- [js/mathx.js](js/mathx.js)
  vector, quaternion, and matrix helpers
- [test/sim-smoke.mjs](test/sim-smoke.mjs)
  regression-oriented smoke tests

## Controls

Top bar:

- play / pause
- step
- reset hover
- reset ground
- time scale

Keyboard:

- `W/S` throttle up/down
- `A/D` yaw positive/negative
- arrow keys for pitch and roll
- `Space` centers sticks
- `R` resets to hover

## Notes And Limitations

- The model is intentionally simplified and tuned for interactive use.
- Ground contact is hard-clamped rather than using a suspension/contact solver.
- Aerodynamics are reduced to simple drag terms.
- Waypoint mode is a practical mission layer, not a full navigation stack.
- The app depends on Three.js from the browser import map, so browser runtime is the source of truth for rendering behavior.

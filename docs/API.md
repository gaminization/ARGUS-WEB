# ARGUS-SIM API & Module Reference

This document provides a comprehensive technical reference for the classes, data structures, and utility functions in the `src/` codebase.

> **Intellectual Property Alignment**: This API documents the **original robotics software modules and simulation architecture created by Garv Arora** (under the academic guidance of **Prof. Padma Priya R** at **Vellore Institute of Technology**). These modules provided the reduction to practice upon which **Indian Patent Publication IN202641072249 A1** (*"Autonomous radar-guided survivor detection and navigation system"*, published 19 June 2026) was filed. Each module implements the core subsystems of the patented invention:
> - `src/sensors.js`: Models Radar Sensor (110), Ultrasonic Sensors (120), IMU (130), and GPS module.
> - `src/mapping.js`: Maintains Probabilistic Grid Map (200) with confidence increment and decay (Claims 1, 4, 8).
> - `src/stateMachine.js`: Implements the 3-state patent FSM (Exploration, Confirmation, Logging) mapped to 5 runtime modes (Claims 2, 3, 10).
> - `src/navigation.js` & `src/robot.js`: Executes SLAM-free autonomous wall-following and potential fields (Claims 5, 6).

---

## 1. Module Index

| Module | Primary Export | Description |
| :--- | :--- | :--- |
| [`src/config.js`](file:///home/gaminizer/Projects/ARGUS-SIM/src/config.js) | `CONFIG`, Math utilities | Central simulation parameters, physical constants, and scoring weights. |
| [`src/environment.js`](file:///home/gaminizer/Projects/ARGUS-SIM/src/environment.js) | `Environment` | Procedural rubble arena, boundary walls, and survivor target placement. |
| [`src/robot.js`](file:///home/gaminizer/Projects/ARGUS-SIM/src/robot.js) | `Robot` | Differential drive kinematics (150), chassis platform (100), and stuck detection. |
| [`src/sensors.js`](file:///home/gaminizer/Projects/ARGUS-SIM/src/sensors.js) | `SensorSuite` | Raycast sonar (120), 24GHz FMCW vital radar (110), 6-DoF IMU (130), and GPS. |
| [`src/mapping.js`](file:///home/gaminizer/Projects/ARGUS-SIM/src/mapping.js) | `MappingSystem` | Probabilistic grid map (200), confidence update/decay (Claims 1, 4), and clustering (Claim 8). |
| [`src/navigation.js`](file:///home/gaminizer/Projects/ARGUS-SIM/src/navigation.js) | `NavigationSystem` | Artificial Potential Fields, frontier search, and trap recovery without SLAM (Claim 6). |
| [`src/stateMachine.js`](file:///home/gaminizer/Projects/ARGUS-SIM/src/stateMachine.js) | `BehaviorStateMachine` | Threshold-driven state machine: Explore, Wall-Follow, Track, Confirm, Replay (Claim 2). |
| [`src/renderer.js`](file:///home/gaminizer/Projects/ARGUS-SIM/src/renderer.js) | `Renderer` | Dual-canvas 2D real-time visualizer for world view and 4-tier probability map. |
| [`src/replay.js`](file:///home/gaminizer/Projects/ARGUS-SIM/src/replay.js) | `ReplayBuffer` | Ring-buffer telemetry recorder and deterministic time-scrubber. |
| [`src/logger.js`](file:///home/gaminizer/Projects/ARGUS-SIM/src/logger.js) | `Logger` | Tagged circular log buffer with timestamp formatting. |
| [`src/utils.js`](file:///home/gaminizer/Projects/ARGUS-SIM/src/utils.js) | Vector & PRNG helpers | 2D vector algebra, Mulberry32 PRNG, and spatial distance metrics. |

---

## 2. Detailed Class Specifications

### 2.1. `Environment` (`src/environment.js`)
Handles world generation, spatial queries, and raycasts.

#### Constructor
```javascript
new Environment(rng: () => number)
```
- Initializes boundary walls, generates 36 procedural rubble clusters, carves open clear zones, and spawns 6 survivor entities.

#### Methods
- `generate(): void`
  Regenerates world obstacles and clears zones.
- `spawnSurvivors(count: number): void`
  Spawns unconfirmed survivor targets with randomized breathing frequencies ($0.14\text{--}0.32\text{ Hz}$).
- `collidesCircle(x: number, y: number, radius: number): boolean`
  Returns `true` if a circle at $(x, y)$ intersects any rectangular obstacle.
- `raycast(x: number, y: number, angle: number, maxRange: number, step?: number): RayHit`
  Casts a ray forward. Returns `{ hit: boolean, dist: number, x: number, y: number }`.

---

### 2.2. `Robot` (`src/robot.js`)
Models the physical robot platform and chassis dynamics.

#### Properties
- `x, y`: Current 2D Cartesian position in world space.
- `theta`: Orientation angle in radians ($[-\pi, \pi]$).
- `linearVelocity, angularVelocity`: Current smoothed velocities.
- `ultra`: Ultrasonic readings `{ front, left, right }`.
- `radar`: Radar status `{ signal, direction, detected }`.
- `imu`: IMU orientation `{ theta }`.
- `gps`: Degraded GPS reading `{ x, y }`.
- `stuckTime`: Cumulative duration robot has been stationary ($v < 0.6\text{ px/s}$).

#### Methods
- `updatePose(dt: number, env: Environment): void`
  Applies first-order velocity smoothing, checks circular collision with environment, updates pose, and tracks stationary time.

---

### 2.3. `SensorSuite` (`src/sensors.js`)
Simulates noisy onboard sensor hardware.

#### Methods
- `sampleUltrasonic(robot: Robot, env: Environment): void`
  Fires 3 simulated acoustic beams with angular jitter and distance noise.
- `sampleRadar(robot: Robot, env: Environment, simTime: number): void`
  Computes vital signal attenuation across all survivors, evaluates FOV cutoff, adds noise and false positives, and pushes into a $240\text{ ms}$ delay queue.
- `sampleImu(robot: Robot): void`
  Samples continuous heading with angular jitter noise.
- `sampleGps(robot: Robot, dt: number): void`
  Updates simulated GPS coordinates at $2\text{ Hz}$ interval with high Gaussian variance.
- `sampleAll(robot: Robot, env: Environment, simTime: number, dt: number): void`
  Executes all sensor sampling pipelines for the current physics step.

---

### 2.4. `MappingSystem` (`src/mapping.js`)
Maintains the internal representations of space and uncertainty.

#### Core Data Structures
- `visited`: `Float32Array` of size $100 \times 100$. Tracks visit counts.
- `explored`: `Uint8Array` of size $100 \times 100$. Explored free/obstacle space.
- `obstacle`: `Uint8Array` of size $100 \times 100$. Detected solid boundaries.
- `prob`: `Float32Array` of size $100 \times 100$. Survivor probability distribution.
- `ambient`: `Float32Array` of size $100 \times 100$. Baseline noise floor.

#### Methods
- `markVisited(x: number, y: number): void`
  Increments visited counter and marks cell as explored.
- `updateObstacleFromUltrasonic(robot: Robot, env: Environment): void`
  Rasterizes ultrasonic beams to clear free space and mark hits.
- `updateProbability(robot: Robot, dt: number): void`
  Applies mean-reverting exponential decay and projects radar directional cone onto the probability grid.
- `suppressConfirmedZones(survivors: Array, dt: number): void`
  Applies radial Gaussian suppression around confirmed targets.
- `blurProbability(): void`
  Executes 5-tap separable 2D Gaussian smoothing pass.
- `clusterHighProb(): Array<Cluster>`
  Extracts 8-connected components with $P \ge 0.56$. Returns sorted clusters `{ x, y, confidence, size }`.

---

### 2.5. `NavigationSystem` (`src/navigation.js`)
Autonomous path planner and potential field engine.

#### Methods
- `step(robot: Robot, state: string, clusters: Array, dt: number): ControlCommand`
  Evaluates trapped status, executes escape routines if boxed, updates goals, computes APF vector, and returns `{ linear, angular, err, recovering }`.
- `potentialField(robot: Robot, state: string): Vector2D`
  Computes combined vector sum of goal attraction, radar attraction, obstacle repulsion, and momentum.
- `planPath(robot: Robot): boolean`
  Runs A\* grid pathfinder to current goal with anti-revisit cost weighting.
- `chooseFrontierGoal(robot: Robot): Point2D | null`
  Finds nearest unvisited frontier cell adjacent to explored free space.
- `chooseClusterGoal(robot: Robot, clusters: Array): Point2D | null`
  Selects highest-confidence reachable survivor cluster.

---

### 2.6. `BehaviorStateMachine` (`src/stateMachine.js`)
Hierarchical state machine managing mission phases.

#### States
- `"EXPLORE"`: Systematic coverage search for unmapped frontiers.
- `"WALL_FOLLOW"`: Reactive obstacle boundary tracing in cluttered passageways.
- `"TRACK"`: Directed homing onto radar vital signals or probability hotspots.
- `"CONFIRM"`: Precision slow-speed verification maneuver around victim location.
- `"REPLAY"`: Simulation freeze and historical timeline scrubbing mode.

#### Methods
- `update(robot: Robot, clusters: Array, mode: string, dt: number): string`
  Evaluates transition conditions and returns active state.
- `setState(next: string): void`
  Transitions to new state and resets state timer.

---

### 2.7. `ReplayBuffer` (`src/replay.js`)
Deterministic simulation recording and scrubbing.

#### Methods
- `record(dt: number, snapshot: Snapshot): void`
  Buffers a snapshot if interval threshold ($80\text{ ms}$) is met. Caps buffer at 14,000 frames.
- `setCursor(i: number): void`
  Moves playback cursor to specific frame index.
- `stepCursor(delta: number): void`
  Steps cursor forward or backward.
- `current(): Snapshot | null`
  Retrieves current snapshot at cursor.

---

### 2.8. `Logger` (`src/logger.js`)
Circular log buffer maintaining up to 260 formatted telemetry events.

#### Methods
- `push(tag: string, msg: string, simTime: number): void`
  Appends `[MM:SS.S] [TAG] message` to buffer.
- `clear(): void`
  Purges log buffer.
- `text(): string`
  Returns all buffered lines separated by newlines.

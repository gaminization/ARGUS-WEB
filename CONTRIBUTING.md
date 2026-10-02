# Contributing to ARGUS-SIM

Thank you for your interest in contributing to **ARGUS-SIM**! This repository hosts an autonomous, radar-centric search-and-rescue robot simulation operating in degraded, vision-obscured, and GPS-denied disaster environments.

---

## 🛠️ Development Philosophy

1. **Realistic Sensing**: We prioritize physically grounded sensor modeling over idealized physics. All sensor readings (radar, ultrasonic, IMU, GPS) must incorporate realistic attenuation, beam divergence, latency, noise, and false-positive characteristics.
2. **Zero Global Cheats**: The robot's decision-making stack must never query ground-truth environment geometry or survivor positions. All actions must be synthesized strictly from local onboard perception and persistent internal maps.
3. **Zero Runtime Dependencies**: The simulation core and web interface remain strictly native ES6 JavaScript and HTML5 Canvas with no heavyweight framework bloat.

---

## 🚀 Getting Started

### Prerequisites
- Any modern web browser (Chrome, Firefox, Safari, Edge)
- Python 3.x (for serving static assets) or Node.js / `npx`

### Local Development Setup

1. **Clone the repository:**
   ```bash
   git clone https://github.com/gaminization/ARGUS-SIM.git
   cd ARGUS-SIM
   ```

2. **Launch the development server:**
   ```bash
   # Using Python 3
   python3 -m http.server 8080

   # Or using npx
   npx serve .
   ```

3. **Open the simulation:**
   Navigate to `http://localhost:8080` in your web browser.

4. **Run syntax validation:**
   ```bash
   npm run check
   ```

---

## 📂 Repository Structure

```
ARGUS-SIM/
├── index.html              # Main simulation interface
├── styles.css              # Cyber-tactical UI styling
├── package.json            # Tooling scripts and metadata
├── favicon.svg             # Application vector icon
├── src/
│   ├── config.js           # Physical constants, tuning parameters, weights
│   ├── environment.js      # Procedural rubble world & survivor generators
│   ├── logger.js           # Circular structured log ring buffer
│   ├── main.js             # Simulation lifecycle, input bindings, loop
│   ├── mapping.js          # Persistent occupancy & Bayesian probability grid
│   ├── navigation.js       # Potential-field & A* frontier navigation
│   ├── renderer.js         # Canvas 2D dual-viewport visualizer
│   ├── replay.js           # Deterministic time-series replay scrubber
│   ├── robot.js            # Differential drive kinematics & momentum
│   ├── sensors.js          # mmWave radar, ultrasonic array, IMU, GPS models
│   ├── stateMachine.js     # 5-phase behavioral finite state machine
│   └── utils.js            # Vector math, PRNG, spatial helper functions
└── docs/                   # Detailed technical documentation
    ├── ARCHITECTURE.md     # Algorithmic and mathematical foundations
    ├── HARDWARE_SPEC.md    # Real-world sensor equivalent specifications
    ├── ALGORITHMS.md       # APF, A*, and spatial clustering walkthrough
    └── API.md              # Codebase module reference
```

---

## 📝 Commit & Pull Request Guidelines

- **Commit Messages**: Follow standard conventional commits format:
  - `feat: add adaptive radar thresholding`
  - `fix: prevent path oscillation in narrow corridors`
  - `docs: update potential field equations in ARCHITECTURE.md`
- **Verification**: Ensure that `npm run check` passes with zero syntax errors before submitting a pull request.
- **Testing**: Test robot navigation across multiple seeds to ensure no persistent deadlocks or stuck states.

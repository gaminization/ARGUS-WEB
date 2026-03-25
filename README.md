# ARGUS-X SIM

Demo-ready autonomous rescue robot simulation with hardware-realistic sensing.

## Hardware modeled
- 3x Ultrasonic (front/left/right)
- LD2410C radar (signal + direction + noise/delay/false positives)
- MPU6050 orientation
- GPS noisy position (UI/logging only)
- Differential-drive motors

No LiDAR, vision, global map knowledge, or perfect localization are used.

## Architecture
- `src/environment.js` world/debris/survivor placement
- `src/sensors.js` local sensor simulation
- `src/mapping.js` persistent visited/obstacle/probability maps
- `src/navigation.js` potential-field goal/radar/repulsion/momentum navigation
- `src/stateMachine.js` behavior phases: EXPLORE, WALL_FOLLOW, TRACK, CONFIRM, REPLAY
- `src/replay.js` record/scrub replay buffer
- `src/renderer.js` world + map visualization
- `src/main.js` integration loop, controls, scoring, logging

## Run
```bash
cd /home/garvarora/ARGUS-SIM
python3 -m http.server 8080
```
Open `http://localhost:8080`

## Controls
- `SPACE` pause/resume
- `R` reset
- `P` toggle LIVE/REPLAY
- `← / →` scrub replay
- `C` clear logs
- `ESC` halt

## Mission metrics
- survivors found
- coverage percentage
- elapsed time
- mission score

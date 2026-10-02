# ARGUS-X Hardware Specification & Sim-to-Real Guide

This document specifies the physical hardware architecture, sensor bill of materials (BOM), electrical topology, and sim-to-real translation parameters for deploying the **ARGUS-X** autonomous navigation stack on physical robotic hardware.

---

## 1. System Overview & Physical Design Philosophy

ARGUS-X is engineered for search-and-rescue operations inside disaster zones (collapsed concrete structures, industrial fires, rubble voids, mine tunnels). In such environments:
- **LiDAR** fails due to severe optical backscattering from smoke, steam, and airborne concrete dust.
- **RGB/Depth Cameras** fail due to pitch-black conditions and particulate occlusion.
- **GPS** is completely denied by subterranean voids and heavy reinforced concrete.

To overcome these constraints, ARGUS-X relies on **millimeter-wave (mmWave) FMCW radar** operating at $24\text{ GHz}$, capable of penetrating dust, drywall, and smoke while capturing sub-millimeter chest wall movements associated with human respiration.

---

## 2. Bill of Materials (BOM)

| Component | Model / Part | Interface | Function in ARGUS-X |
| :--- | :--- | :--- | :--- |
| **Primary Controller / SBC** | Espressif ESP32-S3-WROOM-1 OR Raspberry Pi 4 Model B (4GB) | SPI / UART / I2C | Runs navigation state machine, APF planner, and serial sensor bridges. |
| **mmWave FMCW Radar** | Hi-Link HLK-LD2410C ($24\text{ GHz}$) | UART ($256000\text{ baud}$) | Detects micro-motion (respiration) through dust/debris up to $6\text{ m}$. |
| **Ultrasonic Ranging (3x)** | HC-SR04P / RCWL-1601 ($3.3\text{V}$ compatible) | GPIO (Trigger/Echo) | Obstacle boundary detection and local wall-following (front, $\pm 45^\circ$). |
| **6-DoF IMU** | InvenSense MPU-6050 (or ICM-20948) | I2C ($400\text{ kHz}$) | High-rate orientation estimation, yaw tracking, and angular velocity. |
| **Drivetrain Motors (2x)** | 12V DC Metal Gearmotors (30:1, $250\text{ RPM}$) | Quadrature Encoders | Differential-drive locomotion with wheel odometry feedback. |
| **Motor Driver** | Toshiba TB6612FNG Dual H-Bridge | PWM / Digital IO | Efficient low-heat motor velocity and directional control. |
| **Power Supply** | 3S 11.1V $2200\text{ mAh}$ 25C LiPo Battery | XT60 | Provides high-current power for drivetrain and onboard computation. |
| **Voltage Regulation** | 2x LM2596 DC-DC Buck Regulators | Input: 12V, Out: 5V/3A & 3.3V/2A | Clean, decoupled power rails for compute and analog sensor suites. |
| **Chassis Platform** | Heavy-duty Aluminum Tracked or Skid-Steer Chassis | Mechanical | High-clearance navigation across rubble, pipes, and broken masonry. |

---

## 3. Electrical Wiring & Interface Topology

```
+-----------------------------------------------------------------+
|                       11.1V 3S LiPo Battery                     |
+-------------------------------+---------------------------------+
                                |
                +---------------+---------------+
                |                               |
       [Step-Down 5V/3A]               [Step-Down 3.3V/2A]
                |                               |
+---------------+---------------+       +-------+-------+
|  Raspberry Pi 4 / ESP32-S3   |       | Sensor Bus    |
+---------------+---------------+       +-------+-------+
        | UART (GPIO 14/15)                     |
        +---> Hi-Link HLK-LD2410C Radar         |
        | I2C (SDA/SCL)                         |
        +---> InvenSense MPU-6050 IMU <---------+
        | GPIO (Trig/Echo x 3)                  |
        +---> Front Ultrasonic (HC-SR04P) <-----+
        +---> Left 45° Ultrasonic <-------------+
        +---> Right 45° Ultrasonic <------------+
        | PWM / Direction                       |
        +---> TB6612FNG Dual Driver             |
                | Motor Power (12V Rail)        |
                +---> Left Wheel Motor          |
                +---> Right Wheel Motor         |
```

---

## 4. mmWave Radar: Protocol & Vital Signature Decoding

The **HLK-LD2410C** radar operates at $24.00\text{--}24.25\text{ GHz}$ with FMCW modulation. It reports both static targets (breathing micro-movements) and moving targets across 8 discrete distance gates ($0.75\text{ m}$ per gate).

### 4.1. Serial Frame Specification
- **Baud Rate**: $256000\text{ bps}$, 8 data bits, 1 stop bit, no parity.
- **Standard Telemetry Frame Structure**:
  ```
  [0xFD, 0xFC, 0xFB, 0xFA] - Frame Header (4 bytes)
  [Length]                  - Intra-frame payload length (2 bytes, Little-Endian)
  [Target State]            - 0x00: No target, 0x01: Moving, 0x02: Static, 0x03: Both
  [Moving Target Dist]      - 2 bytes (cm)
  [Moving Target Energy]    - 1 byte (0-100)
  [Static Target Dist]      - 2 bytes (cm)  <-- Human Respiration / Micro-motion
  [Static Target Energy]    - 1 byte (0-100) <-- Vital Signal Strength
  [Detection Distance]      - 2 bytes (cm)
  [0x04, 0x03, 0x02, 0x01] - Frame Footer (4 bytes)
  ```

### 4.2. Vital Signal Translation
The simulation's `robot.radar.signal` corresponds directly to normalized static target energy:

$$\text{SimulatedSignal} \approx \frac{\text{StaticTargetEnergy}}{100.0}$$

Static target detection with energy $\ge 35$ persisting over $> 1.2\text{ s}$ satisfies the transition threshold into the `TRACK` and `CONFIRM` states.

---

## 5. Sim-to-Real Calibration & Scaling Matrix

To map simulation coordinates to physical metric units:

| Simulation Parameter | Sim Value | Physical Scale | Real-World Value |
| :--- | :--- | :--- | :--- |
| **World Dimensions** | $800 \times 800\text{ px}$ | $1\text{ px} = 2.5\text{ cm}$ | $20.0 \times 20.0\text{ meters}$ |
| **Map Grid Cell** | $8\text{ px}$ | $1\text{ cell} = 20\text{ cm}$ | $0.2 \times 0.2\text{ meters}$ |
| **Robot Radius** | $10\text{ px}$ | $25\text{ cm}$ diameter | $0.25\text{ m}$ (Compact AGV) |
| **Wheel Base** | $24\text{ px}$ | Track Width | $30\text{ cm}$ |
| **Max Linear Velocity** | $78\text{ px/s}$ | $2.5\text{ cm/px}$ | $1.95\text{ m/s}$ ($7.0\text{ km/h}$) |
| **Max Angular Velocity**| $3.4\text{ rad/s}$ | Direct | $\approx 195^\circ/\text{s}$ |
| **Ultrasonic Max Range**| $120\text{ px}$ | $2.5\text{ cm/px}$ | $3.0\text{ meters}$ |
| **Radar Max Range** | $180\text{ px}$ | $2.5\text{ cm/px}$ | $4.5\text{ meters}$ |
| **Radar Delay Latency** | $240\text{ ms}$ | Direct | Processing + UART latency |

---

## 6. Embedded Firmware Implementation Notes

When compiling the navigation and mapping logic to embedded C++ / MicroPython on ESP32 or ROS 2 on a Raspberry Pi:
1. **Grid Memory Optimization**: The $100 \times 100$ probability grid requires only $40\text{ KB}$ for `prob` (`float32`) and $10\text{ KB}$ for `obstacle` (`uint8`), fitting comfortably in ESP32 SRAM.
2. **Fixed-Point Math**: Vector normalization and potential field arithmetic can be computed using single-precision IEEE 754 floating-point supported in hardware on ESP32-S3.
3. **Sensor Synchronization**: Ultrasonics should be triggered sequentially with a $15\text{ ms}$ inter-ping delay to prevent acoustic cross-talk reflections.

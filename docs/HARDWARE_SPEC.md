# ARGUS-X Hardware Specification & Sim-to-Real Guide

This document specifies the physical hardware architecture, sensor bill of materials (BOM), electrical topology, and sim-to-real translation parameters for deploying the **ARGUS-X** autonomous navigation stack on physical robotic hardware.

> **Intellectual Property Notice**: This hardware specification details the physical robotics platform designed by **Garv Arora** (under the academic guidance of **Prof. Padma Priya R** at **Vellore Institute of Technology**) for the ARGUS platform, patented under **Indian Patent Publication IN202641072249 A1** (*"Autonomous radar-guided survivor detection and navigation system"*), filed on 10 June 2026 and published on 19 June 2026.

---

## 1. System Overview & Physical Design Philosophy (Patent System 10)

ARGUS-X is engineered for search-and-rescue operations inside disaster zones (collapsed concrete structures, industrial fires, rubble voids, mine tunnels). In such environments:
- **LiDAR** fails due to severe optical backscattering from smoke, steam, and airborne concrete dust.
- **RGB/Depth Cameras** fail due to pitch-black conditions and particulate occlusion.
- **GPS** is completely denied by subterranean voids and heavy reinforced concrete.

To overcome these constraints, ARGUS-X relies on **millimeter-wave (mmWave) FMCW radar (`110`)** operating at $24\text{ GHz}$, capable of penetrating non-metallic dust, drywall, and smoke while capturing sub-millimeter chest wall movements associated with human respiration.

Navigation requires **no SLAM, no camera, and no LiDAR (Claim 6)**, instead combining ultrasonic wall-following (`120`) and IMU dead reckoning (`130`) executed locally on a resource-constrained embedded processor (`140`).

---

## 2. Bill of Materials (BOM) & Patent Reference Numerals

| Component & Numeral | Model / Part | Interface | Function & Patent Specifications |
| :--- | :--- | :--- | :--- |
| **Mobile Robot Chassis (`100`)** | Custom Acrylic / Aluminum Robotic Chassis | Mechanical | Structural platform housing sensors, compute, battery, and locomotion. |
| **Radar Sensor (`110`)** | Hi-Link HLK-LD2410C ($24\text{ GHz}$ FMCW) | UART ($256000\text{ baud}$) | Top-mounted for unobstructed FOV. Detects breathing micro-motion through walls/rubble. $P_d \approx 0.85, P_{fa} \approx 0.05$. Static target range $\approx 0.72\text{ m}$, moving $\approx 0.30\text{ m}$. |
| **Ultrasonic Sensors (`120`)** | 3x HC-SR04 / RCWL-1601 ($3.3\text{V}$ compatible) | GPIO (Trigger/Echo) | Obstacle boundary detection and right-hand wall-following (front, left, right). Range $2\text{--}400\text{ cm}$. |
| **Inertial Measurement Unit (`130`)** | InvenSense MPU-6050 (or ICM-20948) | I2C ($400\text{ kHz}$) | Mounted adjacent to radar sensor 110 to synchronize heading with radar sampling during directional sweeps; dead-reckoning navigation. |
| **Processing Unit (`140`)** | Espressif ESP32-S3 Dual-Core ($240\text{ MHz}$) | SPI / UART / I2C | Dual-core processing operating under $200\text{ KB}$ RAM constraints; **100% local edge processing with zero cloud dependency**. |
| **Drive Mechanism (`150`)** | 12V DC Metal Gearmotors + L298N Driver | PWM / Digital IO | Differential-drive locomotion with wheels at lower portion of chassis. |
| **GPS Module** | NEO-6M / NEO-8M GNSS Module | UART ($9600\text{ baud}$) | Outputs NMEA strings (lat, lon, timestamp) logged strictly upon survivor confirmation in Logging State. |
| **Power Source** | 3S 11.1V $2200\text{ mAh}$ Li-ion / LiPo Battery | XT60 | Located beneath mobile robot chassis base for low center of gravity. |
| **Voltage Regulation** | 2x LM2596 DC-DC Buck Regulators | Input: 12V, Out: 5V/3A & 3.3V/2A | Clean, decoupled power rails for compute and analog sensor suites. |

---

## 3. Electrical Wiring & Interface Topology

```
+-----------------------------------------------------------------+
|              Under-Chassis Power: 11.1V 3S LiPo Battery         |
+-------------------------------+---------------------------------+
                                |
                +---------------+---------------+
                |                               |
       [Step-Down 5V/3A]               [Step-Down 3.3V/2A]
                |                               |
+---------------+---------------+       +-------+-------+
| Processing Unit 140 (ESP32)   |       | Sensor Bus    |
+---------------+---------------+       +-------+-------+
        | UART (GPIO 14/15)                     |
        +---> Radar Sensor 110 (HLK-LD2410C)    |
        | I2C (SDA/SCL)                         |
        +---> IMU 130 (MPU-6050 adjacent to 110)|
        | GPIO (Trig/Echo x 3)                  |
        +---> Front Ultrasonic 120 (HC-SR04) <--+
        +---> Left Ultrasonic 120 (HC-SR04) <---+
        +---> Right Ultrasonic 120 (HC-SR04) <--+
        | UART 2 (RX/TX)                        |
        +---> GPS Module (NMEA Logging)         |
        | PWM / Direction                       |
        +---> L298N Motor Driver (Drive 150)    |
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

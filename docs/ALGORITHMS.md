# ARGUS-SIM Algorithmic Foundations

This document provides in-depth mathematical formulations and algorithmic breakdowns of the navigation, perception, and mapping pipelines implemented in the **ARGUS-X** simulation.

> **Intellectual Property Notice**: The algorithms documented herein represent the **original mathematical formulations, confidence decay laws, and navigation techniques developed by Garv Arora** (under the academic guidance of **Prof. Padma Priya R** at **Vellore Institute of Technology**). These algorithms were reduced to practice in this simulation and form the core claims of **Indian Patent Publication IN202641072249 A1** (*"Autonomous radar-guided survivor detection and navigation system"*), published 19 June 2026. Key patented claims derived from this work include:
> - **Claims 1 & 4**: Confidence increment $C = \min(1.0, C + 0.12 \times r)$ and decay $C = \max(0, C - \beta \times \Delta t)$
> - **Claims 2, 3 & 10**: State machine transitions at $C \ge 0.72$ and multi-angle directional sweeps
> - **Claim 6**: SLAM-free, camera-free, LiDAR-free ultrasonic wall-following and IMU dead reckoning
> - **Claim 7**: Micro-motion respiration vital detection through obstacles
> - **Claim 8**: Neighbouring cell grouping and aggregate confidence ranking
> - **Claim 9**: 10 Hz fixed-rate periodic control loop

---

## 1. Artificial Potential Fields (APF)

The motion planner computes continuous steering vectors by integrating four primary potential forces: attractive goal force, attractive radar vital force, repulsive obstacle force, and momentum continuity force.

$$\vec{F}_{\text{net}} = w_g \vec{F}_{\text{goal}} + w_r \vec{F}_{\text{radar}} + w_{\text{rep}} \vec{F}_{\text{rep}} + w_m \vec{F}_{\text{mom}}$$

### 1.1. Goal Attraction Force ($\vec{F}_{\text{goal}}$)
When an active A\* waypoint or frontier target $\vec{p}_g = (x_g, y_g)$ is set:

$$\vec{v}_g = \vec{p}_g - \vec{p}_{\text{robot}}, \quad d_g = \|\vec{v}_g\|$$

$$\vec{F}_{\text{goal}} = \frac{\vec{v}_g}{d_g} \cdot \text{clamp}\left(\frac{d_g}{120}, 0.25, 1.0\right)$$

### 1.2. Obstacle Repulsion Force ($\vec{F}_{\text{rep}}$)
Repulsion is computed from the 3 ultrasonic beams (front $\theta$, left $\theta + \pi/4$, right $\theta - \pi/4$). For each beam $i$ with range reading $d_i$ and angle $\alpha_i$, within maximum repulsive range $R_{\text{rep}} = 84\text{ px}$:

$$\vec{F}_{\text{rep}} = \sum_{i \in \{\text{front, left, right}\}} \left(1 - \frac{d_i}{R_{\text{rep}}}\right)^2 \cdot \begin{bmatrix} \cos(\alpha_i + \pi) \\ \sin(\alpha_i + \pi) \end{bmatrix}, \quad \forall d_i < R_{\text{rep}}$$

The quadratic term ensures gentle deflections at distance and steep repulsive gradients near physical boundaries.

### 1.3. Radar Vital Force ($\vec{F}_{\text{radar}}$)
When the mmWave radar detects human vital signatures (signal $> 0.14$), a directed attraction force pulls the robot toward the signal bearing:

$$\vec{F}_{\text{radar}} = S_{\text{radar}} \cdot \begin{bmatrix} \cos(\theta + \phi_{\text{radar}}) \\ \sin(\theta + \phi_{\text{radar}}) \end{bmatrix}$$

In the `TRACK` state, $w_r$ is amplified by $1.4\times$ to prioritize homing onto victim signatures over broad frontier exploration.

### 1.4. Momentum Continuity Force ($\vec{F}_{\text{mom}}$)
To eliminate high-frequency heading oscillations and prevent abrupt direction reversals in open passages:

$$\vec{F}_{\text{mom}} = \frac{\vec{v}_{\text{linear}}}{\|\vec{v}_{\text{linear}}\|} = \begin{bmatrix} \cos(\theta) \\ \sin(\theta) \end{bmatrix}$$

---

## 2. PID Steering & Speed Modulation

### 2.1. Heading Error & Angle Normalization
The net force vector $\vec{F}_{\text{net}}$ defines the instantaneous target heading:

$$\theta_{\text{target}} = \text{atan2}(F_{y, \text{net}}, F_{x, \text{net}})$$

$$\theta_{\text{err}} = \text{wrap}_{\pi}(\theta_{\text{target}} - \theta_{\text{imu}})$$

where $\text{wrap}_{\pi}(\phi) = \phi - 2\pi \lfloor (\phi + \pi) / (2\pi) \rfloor \in [-\pi, \pi]$.

### 2.2. Closed-Loop PID Form
The angular command $\omega$ is derived using a continuous discrete PID controller:

$$I_t = \text{clamp}\left(I_{t-1} + \theta_{\text{err}} \cdot \Delta t, -1.5, 1.5\right)$$

$$D_t = \frac{\theta_{\text{err}} - \theta_{\text{prev}}}{\max(\Delta t, 10^{-6})}$$

$$\omega = K_p \theta_{\text{err}} + K_i I_t + K_d D_t$$

*Default tuned gains*: $K_p = 2.45$, $K_i = 0.20$, $K_d = 0.42$.

### 2.3. Emergency Proximity Override & Speed Scaling
If clearance falls below safety thresholds, linear speed $v$ is throttled and angular correction is injected:

$$\text{ClearanceFactor} = \text{clamp}\left(\frac{d_{\text{front}} - 10}{45}, 0.08, 1.0\right)$$

$$\text{TurnFactor} = \text{clamp}\left(1 - \frac{|\theta_{\text{err}}|}{0.8\pi}, 0.10, 1.0\right)$$

$$v = V_{\text{base}} \cdot \text{TurnFactor} \cdot \text{ClearanceFactor}$$

When clearance $d_{\text{front}} < 16\text{ px}$:
$$\omega \leftarrow \omega + \begin{cases} +2.2 & \text{if } d_{\text{left}} > d_{\text{right}} \\ -2.2 & \text{otherwise} \end{cases}, \quad v \leftarrow 0.10 \cdot v$$

---

## 3. mmWave FMCW Radar Vital Modeling

The simulation replicates an onboard $24\text{ GHz}$ FMCW radar (HLK-LD2410C equivalent) capable of detecting microscopic chest displacements ($0.5\text{--}5\text{ mm}$) caused by respiration.

### 3.1. Exponential Path Loss & Vital Modulation
For each unconfirmed survivor $j$ at position $\vec{p}_s$ with Euclidean distance $d_j = \|\vec{p}_s - \vec{p}_{\text{robot}}\|$ and relative bearing $\psi_j = \text{wrap}_\pi(\text{atan2}(y_s - y, x_s - x) - \theta)$:

$$B_j(t) = 0.6 + 0.4 \sin(2\pi f_j t + \phi_j), \quad f_j \in [0.14, 0.32]\text{ Hz}$$

$$S_j = \exp\left(-\frac{d_j^2}{2\sigma_{\text{atten}}^2}\right) \cdot B_j(t), \quad \sigma_{\text{atten}} = 70\text{ px}$$

Conditioned on sensor field-of-view:
$$S_j = 0 \quad \text{if } d_j > R_{\text{radar}} \text{ or } |\psi_j| > \frac{\text{FOV}}{2}$$

### 3.2. Bearing Vector Synthesis & Noise Injection
The composite signal and weighted direction vector:

$$\vec{V}_{\text{radar}} = \sum_j S_j \cdot \begin{bmatrix} \cos(\psi_j + \theta) \\ \sin(\psi_j + \theta) \end{bmatrix}$$

$$S_{\text{total}} = \text{clamp}\left(\sum_j S_j + \mathcal{N}(0, \sigma_{\text{sig}}^2), 0, 1\right)$$

$$\phi_{\text{sensor}} = \text{wrap}_\pi\left(\text{atan2}(V_y, V_x) - \theta + \mathcal{N}(0, \sigma_{\text{dir}}^2)\right)$$

### 3.3. Pipeline Transport Delay
Raw radar returns enter a FIFO queue with a physical transport delay $\tau = 240\text{ ms}$, replicating the onboard FFT calculation, chirp processing, and UART serial transmission overhead of physical radar modules.

---

## 4. Probabilistic Confidence Mapping (Patent Claims 1 & 4)

### 4.1. Confidence Increment Formula (Claim 4 & [0069])
The processing unit maintains a 2D grid map (**200**) of confidence scores $C \in [0.0, 1.0]$. When a radar detection is associated with a given cell:

$$C_{t} = \min\left(1.0, \; C_{t-1} + 0.12 \times r\right)$$

where:
- $C$ represents the cell confidence score.
- $r$ represents the reliability factor associated with the radar detection (accounting for signal strength, SNR, and multi-frame consistency).

### 4.2. Time-Based Confidence Decay Formula (Claim 4 & [0070])
In the absence of further radar detections associated with the cell:

$$C_{t} = \max\left(0.0, \; C_{t-1} - \beta \times \Delta t\right)$$

where:
- $\beta$ is the decay coefficient.
- $\Delta t$ is the elapsed time interval since the last update.

This ensures that stale or transient multipath reflections fade away, preventing false positives while allowing sustained respiration signatures to build certainty.

### 4.3. Four-Tier Heatmap Shading Representation Scale ([0071], [0093])
The spatial confidence map is visualized and categorized according to a 4-tier shading hierarchy:

| Confidence Range | Patent Shading Representation | Interpretation |
| :--- | :--- | :--- |
| **$0.00 \le C < 0.25$** | Dotted / Light Shading | Background ambient noise / unverified space |
| **$0.25 \le C < 0.50$** | Cross-Hatched Shading | Tentative / low confidence detection |
| **$0.50 \le C < 0.75$** | Diagonal Line Shading | Moderate confidence candidate target |
| **$0.75 \le C \le 1.00$** | Dark / Solid Black Shading | Confirmed high confidence survivor location |

Cells exhibiting peak probability based on breathing pattern detection are marked with distinct visual indicators (pink target indicators).

### 4.4. Inverse Sensor Raycasting & Occupancy
Ultrasonic range measurements are converted into spatial occupancy via Bresenham-style ray rasterization:
- Cells along the ray up to $d - 4\text{ px}$ are marked as `explored = 1`.
- The cell at ray termination distance $d$ (if hit occurs) is flagged as `obstacle = 1`.

### 4.5. 2D Separable Gaussian Smoothing
Every $\Delta t_{\text{blur}} = 0.24\text{ s}$, a 5-tap 1D separable Gaussian kernel is applied horizontally and vertically:

$$K = \frac{1}{16} [1, 4, 6, 4, 1]$$

This diffuses spatial discretization artifacts and forms smooth confidence contours.

### 4.6. Confirmed Target Suppression
Upon survivor verification, a radial Gaussian damping window ($r = 80\text{ px}$) suppresses the local probability grid back toward ambient levels to prevent orbiting previously logged targets:

$$P(i) \leftarrow P(i) \cdot (1 - \text{pull}) + P_{\text{ambient}}(i) \cdot \text{pull}, \quad \text{pull} = 0.18 \left(1 - \frac{d}{r}\right) \Delta t \cdot 60$$

---

## 5. Candidate Grouping, Multi-Angle Sweeps & Priority Ranking (Patent Claims 2, 8 & 10)

### 5.1. Neighbouring Cell Grouping (Claim 8)
Contiguous regions of high survivor likelihood are extracted via 8-connected flood-fill:
1. Identify all cells where confidence exceeds detection threshold $T_{\text{detection}} = 0.56$.
2. Group adjacent high-confidence cells into candidate survivor clusters.
3. Compute weighted spatial centroid for each cluster:
   $$\bar{x} = \frac{\sum_{k} x_k \cdot C_k}{\sum_{k} C_k}, \quad \bar{y} = \frac{\sum_{k} y_k \cdot C_k}{\sum_{k} C_k}$$

### 5.2. Aggregate Confidence Score & Priority Ranking (Claim 8 & [0091]–[0094])
For each candidate survivor cluster, the system computes an aggregate confidence score:
$$C_{\text{aggregate}} = \frac{1}{|K|} \sum_{k \in K} C_k \quad \text{or} \quad \sum_{k \in K} w_k C_k$$

A prioritized **ranking table** is generated (Rank ID, Candidate ID, Aggregate Score, Position) so rescue teams can prioritize operational entry based on detection certainty.

### 5.3. Directional Sweep & Multi-Angle Triangulation (Claims 2 & 10, [0086]–[0089])
When any cell confidence exceeds the first threshold ($C \ge 0.72$):
1. Robot pauses forward exploration.
2. Performs a directional sweep across discrete angular intervals relative to the current IMU heading:
   $$\Theta_{\text{sweep}} \in \{-90^\circ, -60^\circ, -45^\circ, -30^\circ, -15^\circ, 0^\circ, +15^\circ, +30^\circ, +45^\circ, +60^\circ, +90^\circ, 180^\circ\}$$
   or a full $360^\circ$ rotation.
3. Correlates multi-angle radar returns with IMU heading for triangulation, verifying true respiration micro-motion against single-angle multipath artifacts.
4. If confidence remains above second threshold, logs GPS NMEA coordinates + grid position.

---

## 6. Anti-Revisit Frontier A\* Search

When in `EXPLORE` mode without high-confidence radar clusters, the robot navigates toward frontiers (explored cells adjacent to unmapped territory).

### 6.1. Revisit Cost Function
Standard A\* paths frequently oscillate in dead-ends. ARGUS-X applies a heavy penalty to previously traversed cells:

$$\text{Cost}(u \to v) = 1 + \begin{cases} 4.2 \times \text{Visits}(v) & \text{Normal Exploration} \\ 0.25 \times \text{Visits}(v) & \text{Backtrack Recovery Enabled} \end{cases}$$

Additionally, cells traversed within the last $4.8\text{ seconds}$ are treated as impassable barriers unless backtrack recovery is explicitly activated.

### 6.2. Trapped Escape Sequence
When ultrasonic sensors detect boxed geometry ($(\text{front} < 30) + (\text{left} < 24) + (\text{right} < 24) \ge 2$) and speed remains near zero for $> 0.85\text{ s}$:
1. **Phase 1 (Reverse Crawl)**: $v = -20\text{ px/s}, \omega = 0$ for $0.28\text{ s}$.
2. **Phase 2 (Pivot Clear)**: $v = +8\text{ px/s}, \omega = \pm 2.0\text{ rad/s}$ away from nearest lateral obstacle for $0.48\text{ s}$.
3. Backtrack timer is enabled for $1.5\text{ s}$ to permit exiting tight cul-de-sacs.

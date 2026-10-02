# ARGUS-SIM Algorithmic Foundations

This document provides in-depth mathematical formulations and algorithmic breakdowns of the navigation, perception, and mapping pipelines implemented in the **ARGUS-X** simulation.

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

## 4. Persistent Occupancy & Spatial Heatmap Mapping

### 4.1. Inverse Sensor Raycasting
Ultrasonic range measurements are converted into spatial occupancy via Bresenham-style ray rasterization:
- Cells along the ray up to $d - 4\text{ px}$ are marked as `explored = 1`.
- The cell at ray termination distance $d$ (if hit occurs) is flagged as `obstacle = 1`.

### 4.2. Mean-Reverting Bayesian Probability Density
The continuous survivor probability grid $P(x, y)$ updates each step:

$$P_{t+1}(i) = \text{clamp}\Big(P_t(i) \cdot \lambda_{\text{decay}} + P_{\text{ambient}}(i) \cdot (1 - \lambda_{\text{decay}}), 0, 1\Big)$$

where $\lambda_{\text{decay}} = 0.999$, ensuring that old or spurious radar hits gradually fade toward the ambient uncertainty floor $P_{\text{ambient}} \approx 0.13 \pm 0.05$.

When active radar returns occur:
$$P_{t+1}(i) \leftarrow \text{clamp}\left(P_t(i) + \gamma_{\text{radar}} \cdot S_{\text{total}} \cdot \max\left(0, \hat{u}_{\text{sensor}} \cdot \hat{u}_{\text{cell}}\right) \cdot \Delta t \cdot 60, 0, 1\right)$$

### 4.3. 2D Separable Gaussian Smoothing
Every $\Delta t_{\text{blur}} = 0.24\text{ s}$, a 5-tap 1D separable Gaussian kernel is applied horizontally and vertically:

$$K = \frac{1}{16} [1, 4, 6, 4, 1]$$

This diffuses spatial discretization artifacts and forms coherent probability contours for downstream clustering.

### 4.4. Confirmed Target Suppression
Upon survivor verification, a radial Gaussian damping window ($r = 80\text{ px}$) suppresses the probability grid back to ambient levels:

$$P(i) \leftarrow P(i) \cdot (1 - \text{pull}) + P_{\text{ambient}}(i) \cdot \text{pull}, \quad \text{pull} = 0.18 \left(1 - \frac{d}{r}\right) \Delta t \cdot 60$$

This prevents the robot from getting trapped orbiting previously rescued victims.

---

## 5. Spatial Density Clustering

Contiguous regions of high survivor likelihood are extracted via 8-connected flood-fill:
1. Identify all seed cells where $P(x, y) \ge T_{\text{cluster}} = 0.56$.
2. For each connected component, compute the weighted centroid:
   $$\bar{x} = \frac{\sum_{k} x_k \cdot P_k}{\sum_{k} P_k}, \quad \bar{y} = \frac{\sum_{k} y_k \cdot P_k}{\sum_{k} P_k}$$
3. Assign confidence $C = \max_k(P_k)$ and size $N = \text{cell count}$.
4. Sort candidates descending by confidence. Top clusters become tracking candidates for state transitions.

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

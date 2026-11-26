# 2D Extended Kalman Filter (EKF) for Planar Vehicle Sensor Fusion

## 1. Big Picture & Role in the System

This document specifies the mathematical formulation of the **2D Extended Kalman Filter (`Ekf2D`)** used as the core state estimator in `slam_sensor_fusion` (see [overview.md](../architecture/overview.md) and [sensor_fusion_node.md](../architecture/sensor_fusion_node.md)).

### Intuition
An autonomous vehicle receives complementary but imperfect motion observations:
- **IMU** (400 Hz): crisp yaw rate and linear acceleration, but integrating acceleration drifts quadratically in position.
- **Wheel odometry** (30–100 Hz): strong longitudinal velocity and non-holonomic lateral constraint ($v_y \approx 0$), but vulnerable to wheel slip and heading drift.
- **LiDAR scan matching** (10 Hz) and **Visual Odometry** (30 Hz): accurate geometric motion constraints at lower frequency, but susceptible to occasional scan-matching outliers, motion blur, or temporary dropouts.

The EKF maintains a Gaussian belief $\mathcal{N}(\mathbf{x}_k, \mathbf{P}_k)$ over the vehicle's planar pose, body-frame velocity, and body-frame acceleration. Between measurements it predicts motion forward using nonlinear 2D kinematics; whenever any sensor delivers a measurement, it fuses the innovation weighted by relative uncertainty while rejecting statistical outliers.

### Assumptions & Known Limitations
- **Planar motion**: Validated on the flat Gazebo track (`track.stl`, driving surface at $z = 0.25\,\text{m}$). Pitch, roll, and vertical displacement are assumed small; optional roll/pitch gravity projection removes small suspension-tilt leakage from IMU linear acceleration.
- **Local linearization**: Linearization errors grow if $\Delta t$ or angular velocity $\omega$ is very large. Midpoint heading integration reduces discretization error substantially; large $\Delta t$ steps are subdivided internally.
- **Gaussian unimodal belief**: Multi-modal ambiguities (e.g., global kidnapped-robot relocalization) are outside the scope of a single EKF.
- **No explicit IMU bias states in v1**: Gazebo's IMU plugin has zero configured bias. If real-hardware IMU drift becomes significant, gyro bias $b_\omega$ and accelerometer biases $(b_{a_x}, b_{a_y})$ should be appended to the state vector.

---

## 2. Notation, Units & Coordinate Conventions

All quantities use SI units and follow ROS REP-103 / REP-105:
- **Reference frame** $\mathcal{W}$ (`odom` or `map`): right-handed ENU, $+Z$ up, yaw $\theta$ CCW positive from $+X$.
- **Vehicle body frame** $\mathcal{B}$ (`base_link`): right-handed FLU ($+X$ forward, $+Y$ left, $+Z$ up).

| Symbol | Meaning | Dimension / Type | Units / Frame |
| :--- | :--- | :--- | :--- |
| $\mathbf{x}_k$ | State vector $[x, y, \theta, v_x, v_y, \omega, a_x, a_y]^\top$ | $\mathbb{R}^8$ | Mixed SI (see below) |
| $x, y$ | Position of `base_link` origin in $\mathcal{W}$ | $\mathbb{R}$ | $\text{m}$ / $\mathcal{W}$ |
| $\theta$ | Heading (yaw) of $\mathcal{B}$ in $\mathcal{W}$, normalized to $[-\pi, \pi)$ | $\mathbb{S}^1$ | $\text{rad}$ / $\mathcal{W}$ |
| $v_x, v_y$ | Linear velocity of `base_link` expressed in $\mathcal{B}$ | $\mathbb{R}$ | $\text{m/s}$ / $\mathcal{B}$ |
| $\omega$ | Yaw angular velocity around $+Z_{\mathcal{B}}$ | $\mathbb{R}$ | $\text{rad/s}$ / $\mathcal{B}$ |
| $a_x, a_y$ | Time derivative of body velocity $(\dot{v}_x, \dot{v}_y)$ expressed in $\mathcal{B}$ | $\mathbb{R}$ | $\text{m/s}^2$ / $\mathcal{B}$ |
| $\mathbf{P}_k$ | State error covariance matrix ($\mathbf{P}_k = \mathbf{P}_k^\top \succeq 0$) | $\mathbb{R}^{8 \times 8}$ | Matching $\mathbf{x}_k \mathbf{x}_k^\top$ |
| $\Delta t$ | Prediction time interval ($t_k - t_{k-1} > 0$) | $\mathbb{R}_{>0}$ | $\text{s}$ |
| $\mathbf{F}_k$ | Process model Jacobian $\left.\frac{\partial f}{\partial \mathbf{x}}\right|_{\mathbf{x}_{k-1}}$ | $\mathbb{R}^{8 \times 8}$ | Dimensionless / mixed |
| $\mathbf{Q}(\Delta t)$ | Discrete process noise covariance | $\mathbb{R}^{8 \times 8}$ | Matching $\mathbf{P}_k$ |
| $\mathbf{z}_k$ | Measurement vector from a sensor | $\mathbb{R}^m$ ($m \le 4$) | Sensor units (SI) |
| $h(\mathbf{x})$ | Measurement observation model | $\mathbb{R}^8 \to \mathbb{R}^m$ | Sensor units (SI) |
| $\mathbf{H}_k$ | Measurement Jacobian $\left.\frac{\partial h}{\partial \mathbf{x}}\right|_{\hat{\mathbf{x}}_k^-}$ | $\mathbb{R}^{m \times 8}$ | Mixed SI |
| $\mathbf{R}_k$ | Measurement noise covariance ($\mathbf{R}_k = \mathbf{R}_k^\top \succ 0$) | $\mathbb{R}^{m \times m}$ | Matching $\mathbf{z}_k \mathbf{z}_k^\top$ |
| $\mathbf{y}_k$ | Innovation (measurement residual) $\mathbf{z}_k - h(\hat{\mathbf{x}}_k^-)$ | $\mathbb{R}^m$ | Sensor units (SI) |
| $\mathbf{S}_k$ | Innovation covariance $\mathbf{H}_k \mathbf{P}_k^- \mathbf{H}_k^\top + \mathbf{R}_k$ | $\mathbb{R}^{m \times m}$ | Matching $\mathbf{R}_k$ |
| $\mathbf{K}_k$ | Kalman gain $\mathbf{P}_k^- \mathbf{H}_k^\top \mathbf{S}_k^{-1}$ | $\mathbb{R}^{8 \times m}$ | Mixed SI |
| $d_M^2$ | Squared Mahalanobis distance $\mathbf{y}_k^\top \mathbf{S}_k^{-1} \mathbf{y}_k$ | $\mathbb{R}_{\ge 0}$ | Dimensionless |

---

## 3. Mathematical Formulation & Derivations

### 3.1 Angle Normalization

Planar angles reside on the circle manifold $\mathbb{S}^1$. Define the wrapping operator `wrapAngle`:

$$\mathrm{wrap}(\alpha) = \mathrm{atan2}(\sin\alpha, \cos\alpha) \in (-\pi, \pi] \tag{1}$$

In implementation (`math_utils::wrapAngle`), modular arithmetic with `std::remainder` maps angles strictly into $[-\pi, \pi)$ (with $+\pi \mapsto -\pi$) without trigonometry overhead, and is applied after every heading prediction, heading update, and angular innovation subtraction. *(Derived from first principles.)*

---

### 3.2 Process Model (Prediction Step)

#### Nonlinear Kinematics (Midpoint-Heading Constant-Acceleration Model)
Over a time interval $\Delta t > 0$, define the body-frame displacement under constant body-frame acceleration $(a_x, a_y)$ and the midpoint heading $\bar{\theta}$:

$$\Delta p_x = v_x \Delta t + \frac{1}{2} a_x \Delta t^2, \qquad \Delta p_y = v_y \Delta t + \frac{1}{2} a_y \Delta t^2 \tag{2}$$

$$\bar{\theta} = \theta + \frac{1}{2}\omega \Delta t, \qquad \bar{c} = \cos\bar{\theta}, \qquad \bar{s} = \sin\bar{\theta} \tag{3}$$

The discrete-time state transition $\mathbf{x}^- = f(\mathbf{x}, \Delta t)$ is:

$$\begin{aligned}
x^- &= x + \Delta p_x \bar{c} - \Delta p_y \bar{s} \\
y^- &= y + \Delta p_x \bar{s} + \Delta p_y \bar{c} \\
\theta^- &= \mathrm{wrap}(\theta + \omega \Delta t) \\
v_x^- &= v_x + a_x \Delta t \\
v_y^- &= v_y + a_y \Delta t \\
\omega^- &= \omega \\
a_x^- &= a_x \\
a_y^- &= a_y
\end{aligned} \tag{4}$$

**Why midpoint heading $\bar{\theta}$ instead of Euler $\theta$?**  
Using $\bar{\theta} = \theta + \frac{1}{2}\omega\Delta t$ (second-order Runge–Kutta / midpoint heading) substantially reduces heading integration drift during Ackermann turns compared to first-order Euler, while avoiding the $1/\omega$ division singularity at $\omega = 0$ found in exact circular-arc CTRV equations. *(Derived from first principles; related to the 2D restriction of Moore & Stouch, 2014.)*

#### Analytical Process Jacobian $\mathbf{F} = \frac{\partial f}{\partial \mathbf{x}}$
Differentiating Eq. (4) with respect to $\mathbf{x}$, noting $\frac{\partial \bar{\theta}}{\partial \theta} = 1$ and $\frac{\partial \bar{\theta}}{\partial \omega} = \frac{1}{2}\Delta t$:

Let:
$$d_x = -\Delta p_x \bar{s} - \Delta p_y \bar{c}, \qquad d_y = \Delta p_x \bar{c} - \Delta p_y \bar{s} \tag{5}$$

Then $\mathbf{F} \in \mathbb{R}^{8 \times 8}$ is:

$$\mathbf{F} = \begin{bmatrix}
1 & 0 & d_x & \Delta t\,\bar{c} & -\Delta t\,\bar{s} & \frac{1}{2}\Delta t\,d_x & \frac{1}{2}\Delta t^2\,\bar{c} & -\frac{1}{2}\Delta t^2\,\bar{s} \\
0 & 1 & d_y & \Delta t\,\bar{s} & \Delta t\,\bar{c} & \frac{1}{2}\Delta t\,d_y & \frac{1}{2}\Delta t^2\,\bar{s} & \frac{1}{2}\Delta t^2\,\bar{c} \\
0 & 0 & 1 & 0 & 0 & \Delta t & 0 & 0 \\
0 & 0 & 0 & 1 & 0 & 0 & \Delta t & 0 \\
0 & 0 & 0 & 0 & 1 & 0 & 0 & \Delta t \\
0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix} \tag{6}$$

*(Derived from first principles; verified against numerical central finite differences in `test_ekf_2d.cpp`.)*

#### Process Noise & Covariance Prediction
The predicted covariance is:

$$\mathbf{P}^- = \mathbf{F} \mathbf{P} \mathbf{F}^\top + \mathbf{Q}(\Delta t), \qquad \mathbf{P}^- \leftarrow \frac{1}{2}\left(\mathbf{P}^- + (\mathbf{P}^-)^\top\right) \tag{7}$$

where $\mathbf{Q}(\Delta t) = \mathbf{Q}_{\text{rate}} \cdot \Delta t$ with diagonal continuous-rate spectral density matrix $\mathbf{Q}_{\text{rate}} = \mathrm{diag}(q_x, q_y, q_\theta, q_{v_x}, q_{v_y}, q_\omega, q_{a_x}, q_{a_y}) \succeq 0$.

> **Engineering heuristic**: Using diagonal $\mathbf{Q}_{\text{rate}} \cdot \Delta t$ (following `robot_localization`, Moore & Stouch, 2014) trades the exact Van Loan / continuous white-jerk off-diagonal cross-terms for simple, transparent per-state YAML tuning while ensuring $\mathbf{Q}(\Delta t)$ scales linearly with time step and stays positive semi-definite. Cross-state correlations are still generated naturally through $\mathbf{F}\mathbf{P}\mathbf{F}^\top$.

When $\Delta t$ exceeds `max_substep_dt_sec` (while still $\le \text{max\_dt\_sec}$), prediction is subdivided into uniform substeps to limit linearization error.

---

### 3.3 Measurement Models & Jacobians

#### 3.3.1 2D Pose Update (Absolute Reference Frame)
For a pose measurement observing any non-empty subset of $[x, y, \theta]^\top$ (controlled by `use_position` and `use_yaw`):

$$h_{\text{pose}}(\mathbf{x}) = \mathbf{H}_{\text{pose}}\mathbf{x} \tag{8}$$

where $\mathbf{H}_{\text{pose}}$ selects rows from $\{0, 1, 2\}$ of $\mathbf{I}_8$. The innovation component corresponding to $\theta$ is wrapped via Eq. (1).

#### 3.3.2 2D Body-Frame Twist Update
For a velocity measurement observing any non-empty subset of $[v_x, v_y, \omega]^\top$ expressed in `base_link` (controlled by `use_vx`, `use_vy`, `use_omega`):

$$h_{\text{twist}}(\mathbf{x}) = \mathbf{H}_{\text{twist}}\mathbf{x} \tag{9}$$

where $\mathbf{H}_{\text{twist}}$ selects rows from $\{3, 4, 5\}$ of $\mathbf{I}_8$. For an Ackermann vehicle with negligible lateral tire slip, wheel odometry typically supplies $v_x$ and the non-holonomic constraint $v_y \approx 0$ (with small tuned variance $\sigma_{v_y}^2$).

#### 3.3.3 2D IMU Update with Rotating-Frame Coriolis Physics
Let $\mathbf{v}^{\mathcal{W}} = \mathbf{R}(\theta)\mathbf{v}^{\mathcal{B}}$ be the vehicle's horizontal world-frame velocity. Differentiating in the inertial frame yields the true horizontal acceleration expressed in body frame $\mathcal{B}$:

$$\mathbf{a}_{\text{inertial}}^{\mathcal{B}} = \mathbf{R}(\theta)^\top \dot{\mathbf{v}}^{\mathcal{W}} = \dot{\mathbf{v}}^{\mathcal{B}} + \begin{bmatrix} 0 & -\omega \\ \omega & 0 \end{bmatrix} \mathbf{v}^{\mathcal{B}} = \begin{bmatrix} a_x - \omega v_y \\ a_y + \omega v_x \end{bmatrix} \tag{10}$$

*(Derived from first principles using rotating-frame kinematics.)*

During a steady turn ($\dot{v}_x = \dot{v}_y = 0$, $v_y = 0$), an IMU mounted at `base_link` measures the lateral centripetal acceleration $a_{y,\text{imu}} = +\omega v_x$. Modeling Eq. (10) explicitly prevents centripetal acceleration from corrupting lateral velocity $v_y$ during corners.

Thus, for the full IMU measurement $\mathbf{z}_{\text{imu}} = [\theta_{\text{imu}}, \omega_{\text{imu}}, a_{x,\text{imu}}, a_{y,\text{imu}}]^\top$ (or any enabled subset of yaw, yaw rate, and linear acceleration):

$$h_{\text{imu}}(\mathbf{x}) = \begin{bmatrix} \theta \\ \omega \\ a_x - \omega v_y \\ a_y + \omega v_x \end{bmatrix} \tag{11}$$

with analytical Jacobian $\mathbf{H}_{\text{imu}} \in \mathbb{R}^{4 \times 8}$ (sub-selected by enabled channels):

$$\mathbf{H}_{\text{imu}} = \begin{bmatrix}
0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & -\omega & -v_y & 1 & 0 \\
0 & 0 & 0 & \omega & 0 & v_x & 0 & 1
\end{bmatrix} \tag{12}$$

**Gravity Compensation (ROS Adapter Layer)**: When `compensate_gravity` is enabled and the IMU orientation quaternion is valid, roll ($\phi$) and pitch ($\psi$) are extracted (independent of yaw) and the gravity vector projected into `imu_link` is subtracted from raw specific force before passing $(a_{x,\text{imu}}, a_{y,\text{imu}})$ to `Ekf2D`:
$$\mathbf{a}_{\text{horiz}}^{\mathcal{B}} = \mathbf{f}_{\text{imu}}^{\mathcal{B}} - \mathbf{R}_x(\phi)^\top \mathbf{R}_y(\psi)^\top \begin{bmatrix} 0 \\ 0 \\ g \end{bmatrix}, \qquad g = 9.80665\,\text{m/s}^2 \tag{13}$$
*(Derived from first principles; standard IMU specific-force decomposition.)*

---

### 3.4 Numerically Stable Update Step (LDLT + Mahalanobis Gating + Joseph Form)

Given predicted belief $(\hat{\mathbf{x}}^-, \mathbf{P}^-)$, measurement $\mathbf{z} \in \mathbb{R}^m$, model $h(\cdot)$, Jacobian $\mathbf{H}$, and validated symmetric positive-definite covariance $\mathbf{R} \in \mathbb{R}^{m \times m}$:

1. **Innovation**:
   $$\mathbf{y} = \mathbf{z} - h(\hat{\mathbf{x}}^-), \qquad y_j \leftarrow \mathrm{wrap}(y_j) \text{ for angular indices } j \tag{14}$$
2. **Innovation Covariance**:
   $$\mathbf{S} = \mathbf{H} \mathbf{P}^- \mathbf{H}^\top + \mathbf{R}, \qquad \mathbf{S} \leftarrow \frac{1}{2}(\mathbf{S} + \mathbf{S}^\top) \tag{15}$$
3. **Cholesky Factorization**: Factorize $\mathbf{S} = \mathbf{L}\mathbf{D}\mathbf{L}^\top$ via `Eigen::LDLT` (no explicit matrix inversion). Reject the update if factorization fails or $\min_i D_{ii} \le 10^{-12}$.
4. **Mahalanobis Outlier Gating**:
   $$d_M^2 = \mathbf{y}^\top \mathbf{S}^{-1} \mathbf{y} = \mathbf{y}^\top \left(\mathbf{L}\mathbf{D}\mathbf{L}^\top\right)^{-1} \mathbf{y} \tag{16}$$
   Under the null hypothesis that the measurement is consistent with the Gaussian prior, $d_M^2 \sim \chi^2_m$ (Bar-Shalom et al., 2001). If gating is enabled ($\gamma_m > 0$) and $d_M^2 > \gamma_m$, reject the measurement cleanly without mutating $(\hat{\mathbf{x}}^-, \mathbf{P}^-)$.
   Default $99\%$ $\chi^2$ gates (from the standard $\chi^2$ distribution):
   - $m = 1$: $6.635$, $m = 2$: $9.210$, $m = 3$: $11.345$, $m = 4$: $13.277$.
5. **Kalman Gain (via LDLT solve)**:
   $$\mathbf{K}^\top = \mathbf{S}^{-1}(\mathbf{H}\mathbf{P}^-) \implies \mathbf{K} = \left(\text{ldlt.solve}(\mathbf{H}\mathbf{P}^-)\right)^\top \tag{17}$$
6. **State Update**:
   $$\hat{\mathbf{x}}^+ = \hat{\mathbf{x}}^- + \mathbf{K}\mathbf{y}, \qquad \theta^+ \leftarrow \mathrm{wrap}(\theta^+) \tag{18}$$
7. **Joseph-Form Covariance Update** (Bar-Shalom et al., 2001):
   $$\mathbf{A} = \mathbf{I}_8 - \mathbf{K}\mathbf{H}$$
   $$\mathbf{P}^+ = \mathbf{A}\mathbf{P}^-\mathbf{A}^\top + \mathbf{K}\mathbf{R}\mathbf{K}^\top, \qquad \mathbf{P}^+ \leftarrow \frac{1}{2}\left(\mathbf{P}^+ + (\mathbf{P}^+)^\top\right) \tag{19}$$

**Why Joseph form?** Because both $\mathbf{A}\mathbf{P}^-\mathbf{A}^\top$ and $\mathbf{K}\mathbf{R}\mathbf{K}^\top$ are congruence transformations of positive semi-definite matrices, $\mathbf{P}^+$ remains guaranteed symmetric positive semi-definite even when $\mathbf{R}$ is very small relative to $\mathbf{P}^-$, where the textbook shortcut $(\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}^-$ can lose positive definiteness from subtractive cancellation.

---

### 3.5 Differential Odometry Adaptation (Preventing Multi-Odom Drift Fighting)

When an upstream odometry node (e.g., LiDAR scan matching or VO) outputs an accumulated pose $\mathbf{p}_{k} = [x_k, y_k, \theta_k]^\top$ in a drifting frame at time $t_k$, `DifferentialAdapter` converts consecutive poses $(\mathbf{p}_{k-1}, t_{k-1})$ and $(\mathbf{p}_k, t_k)$ (with $\Delta t = t_k - t_{k-1} \in [\Delta t_{\min}, \Delta t_{\max}]$) into an equivalent body-frame twist:

$$\Delta x_w = x_k - x_{k-1}, \qquad \Delta y_w = y_k - y_{k-1}, \qquad \Delta \theta = \mathrm{wrap}(\theta_k - \theta_{k-1}) \tag{20}$$

Rotating into the sensor's midpoint heading $\bar{\theta}_s = \theta_{k-1} + \frac{1}{2}\Delta\theta$:

$$\begin{bmatrix} \Delta x_b \\ \Delta y_b \end{bmatrix} = \mathbf{R}(\bar{\theta}_s)^\top \begin{bmatrix} \Delta x_w \\ \Delta y_w \end{bmatrix}, \qquad \mathbf{z}_{\text{twist}} = \frac{1}{\Delta t}\begin{bmatrix} \Delta x_b \\ \Delta y_b \\ \Delta \theta \end{bmatrix} \tag{21}$$

Let $\mathbf{T}_{\text{rot}} = \mathrm{blkdiag}(\mathbf{R}(\bar{\theta}_s)^\top, 1)$. For incremental pose uncertainty $\mathbf{R}_{\Delta p}$ (configured as per-step differential noise or propagated from pose covariance $\mathbf{R}_{p}$ via $\mathbf{R}_{\Delta p} \approx \mathbf{R}_p$ when incremental, or $\mathbf{R}_{p,k} + \mathbf{R}_{p,k-1}$ if independent — `DifferentialAdapter` uses an explicit configurable mode/fallback covariance to avoid artificially exploding uncertainty from cumulative drift covariance):

$$\mathbf{R}_{\text{twist}} = \frac{1}{\Delta t^2}\,\mathbf{T}_{\text{rot}}\,\mathbf{R}_{\Delta p}\,\mathbf{T}_{\text{rot}}^\top \tag{22}$$

*(Derived from first principles; covariance scaling follows standard linear error propagation.)*

---

### 3.6 Rigid-Body Extrinsic Transformation (`sensor_link` $\to$ `base_link`)

If a sensor measures pose or twist at a rigid frame $\mathcal{S}$ mounted at offset $\mathbf{r} = [r_x, r_y]^\top$ and planar yaw $\psi_{\mathcal{B}\mathcal{S}}$ relative to `base_link` $\mathcal{B}$ ($\mathbf{T}_{\mathcal{B}\mathcal{S}} \in SE(2)$):

- **Pose**: Given sensor pose $\mathbf{T}_{\mathcal{W}\mathcal{S}}$, the vehicle base pose is:
  $$\mathbf{T}_{\mathcal{W}\mathcal{B}} = \mathbf{T}_{\mathcal{W}\mathcal{S}} \mathbf{T}_{\mathcal{B}\mathcal{S}}^{-1} \tag{23}$$
- **2D Body Twist**: Given sensor linear velocity $\mathbf{v}^{\mathcal{S}}$ and yaw rate $\omega^{\mathcal{S}}$ (same $\omega$ for a rigid body):
  $$\mathbf{v}^{\mathcal{B}} = \mathbf{R}(\psi_{\mathcal{B}\mathcal{S}})\mathbf{v}^{\mathcal{S}} - \begin{bmatrix} -\omega r_y \\ \omega r_x \end{bmatrix}, \qquad \omega^{\mathcal{B}} = \omega^{\mathcal{S}} \tag{24}$$
  with twist covariance rotated by the Jacobian $\mathbf{J}_{\text{ext}} = \begin{bmatrix} \mathbf{R}(\psi_{\mathcal{B}\mathcal{S}}) & \begin{bmatrix} r_y \\ -r_x \end{bmatrix} \\ \mathbf{0}^\top & 1 \end{bmatrix}$:
  $$\mathbf{R}_{\text{twist}}^{\mathcal{B}} = \mathbf{J}_{\text{ext}} \mathbf{R}_{\text{twist}}^{\mathcal{S}} \mathbf{J}_{\text{ext}}^\top \tag{25}$$

*(Derived from first principles using 2D rigid-body kinematics.)*

---

## 4. Algorithm Walkthrough & Complexity

### Predict (`Ekf2D::predict(dt)`)
1. Validate filter is initialized, $\Delta t$ is finite, and $0 \le \Delta t \le \Delta t_{\max}$. Return immediately if $\Delta t = 0$.
2. Partition $\Delta t$ into $N = \lceil \Delta t / \Delta t_{\text{sub,max}} \rceil$ equal substeps $h = \Delta t / N$.
3. For each substep:
   - Compute $\Delta p_x, \Delta p_y, \bar{\theta}$ via Eqs. (2)–(3).
   - Compute analytical Jacobian $\mathbf{F}$ via Eq. (6).
   - Propagate state $\mathbf{x} \leftarrow f(\mathbf{x}, h)$ via Eq. (4) with angle wrapping Eq. (1).
   - Propagate covariance $\mathbf{P} \leftarrow \mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q}_{\text{rate}} h$ and symmetrize via Eq. (7).

### Update (`Ekf2D::applyUpdate(z, z_hat, H, R, angular_mask, gate_threshold)`)
1. Validate filter is initialized, all inputs are finite, dimensions match ($1 \le m \le 4$), and $\mathbf{R}$ is symmetric positive definite.
2. Compute innovation $\mathbf{y} = \mathbf{z} - \hat{\mathbf{z}}$ and wrap angular entries (Eq. 14).
3. Compute $\mathbf{S} = \mathbf{H}\mathbf{P}\mathbf{H}^\top + \mathbf{R}$, symmetrize (Eq. 15), and compute `Eigen::LDLT` factorization.
4. Compute $d_M^2 = \mathbf{y}^\top \text{ldlt.solve}(\mathbf{y})$ (Eq. 16). If gate enabled and $d_M^2 > \gamma_m$, return `kRejectedOutlier`.
5. Solve $\mathbf{K} = (\text{ldlt.solve}(\mathbf{H}\mathbf{P}))^\top$ (Eq. 17).
6. Update $\mathbf{x}^+ = \mathbf{x} + \mathbf{K}\mathbf{y}$, wrap $\theta^+$ (Eq. 18).
7. Update $\mathbf{P}^+$ via Joseph form (Eq. 19) and symmetrize.

### Complexity
- State dimension $n = 8$, measurement dimension $m \in \{1, 2, 3, 4\}$.
- **Time complexity**: $O(n^3 + m^3)$ per step (fixed small sizes; $< 10\,\mu\text{s}$ per update on a modern CPU).
- **Space complexity**: $O(n^2)$ fixed stack/object memory (zero dynamic heap allocations during steady-state prediction; small fixed-size matrices for updates).

---

## 5. Code-to-Math Traceability

| Math Symbol / Equation | Code Location | Identifier / Function | Notes |
| :--- | :--- | :--- | :--- |
| Eq. (1): Angle wrapping | `src/core/math_utils.cpp` | `wrapAngle()` | Maps to $[-\pi, \pi)$ via `std::remainder` |
| Eqs. (2)–(4): Kinematic transition | `src/core/ekf_2d.cpp` | `Ekf2D::predictSubstep()` | Midpoint-heading 2D constant-accel model |
| Eqs. (5)–(6): Process Jacobian $\mathbf{F}$ | `src/core/ekf_2d.cpp` | `Ekf2D::computeProcessJacobian()` | Verified vs. central finite differences |
| Eq. (7): Covariance prediction | `src/core/ekf_2d.cpp` | `Ekf2D::predictSubstep()` | Enforces $\frac{1}{2}(\mathbf{P} + \mathbf{P}^\top)$ |
| Eq. (8): Pose model $\mathbf{H}_{\text{pose}}$ | `src/core/ekf_2d.cpp` | `Ekf2D::updatePose()` | Supports position-only, yaw-only, or full SE(2) |
| Eq. (9): Twist model $\mathbf{H}_{\text{twist}}$ | `src/core/ekf_2d.cpp` | `Ekf2D::updateTwist()` | Supports any non-empty subset of $(v_x, v_y, \omega)$ |
| Eqs. (10)–(12): IMU Coriolis model & $\mathbf{H}_{\text{imu}}$ | `src/core/ekf_2d.cpp` | `Ekf2D::updateImu()`, `computeImuJacobian()` | Explicit $\boldsymbol{\omega} \times \mathbf{v}^{\mathcal{B}}$ terms |
| Eq. (13): Gravity projection removal | `src/ros/message_converters.cpp` | `convertImuMessage()` | Uses roll & pitch only (yaw-invariant) |
| Eqs. (14)–(19): LDLT + gating + Joseph update | `src/core/ekf_2d.cpp` | `Ekf2D::applyUpdate()` | Zero `.inverse()` calls; SPD-preserving |
| Eqs. (20)–(22): Differential pose-to-twist | `src/core/differential_adapter.cpp` | `DifferentialAdapter::process()` | Midpoint-rotated relative displacement |
| Eqs. (23)–(25): 2D extrinsic transforms | `src/ros/message_converters.cpp` | `transformPose2DToBase()`, `transformTwist2DToBase()` | Uses TF2 `base_link <- sensor_frame` |

---

## 6. Sources & References

1. **Thrun, S., Burgard, W., & Fox, D. (2005).** *Probabilistic Robotics*. MIT Press. — Standard Extended Kalman Filter formulation (predict/update linearization and probabilistic robotics foundations).
2. **Bar-Shalom, Y., Rong Li, X., & Kirubarajan, T. (2001).** *Estimation with Applications to Tracking and Navigation: Theory, Algorithms and Software*. John Wiley & Sons. — Joseph stabilized covariance update form and $\chi^2$ Mahalanobis innovation gating.
3. **Moore, T., & Stouch, D. (2014).** *"A Generalized Extended Kalman Filter Implementation for the Robot Operating System."* Proceedings of the 13th International Conference on Intelligent Autonomous Systems (IAS-13). — Architectural reference for body-frame velocity/acceleration EKF state, diagonal continuous-rate process noise, and differential odometry fusion in ROS.
4. **ROS Enhancement Proposals REP-103 & REP-105.** *Standard Units of Measure and Coordinate Conventions* (REP-103) and *Coordinate Frames for Mobile Platforms* (REP-105). Official ROS specifications.
5. **First-Principles Derivations**: Midpoint-heading 2D kinematic Jacobian (Eqs. 2–6), rotating-frame planar Coriolis IMU measurement model (Eqs. 10–12), differential pose-to-twist conversion (Eqs. 20–22), and 2D rigid-body extrinsic lever-arm transform (Eqs. 23–25) are **derived from first principles** in this document.

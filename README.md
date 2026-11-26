# Shell Autonomous Car — SLAM Sensor Fusion (`slam_sensor_fusion`)

ROS 2 Humble sensor fusion module for the Shell Urban Concept autonomous vehicle simulated in Gazebo (`shell_urban_simulator`).

## Architecture Highlights

- **Pure C++20 / Eigen Core (`ekf_core`)**: Zero ROS dependencies. Implements an 8-state 2D Extended Kalman Filter (`[x, y, theta, vx, vy, omega, ax, ay]`) with midpoint-heading kinematics, rotating-frame Coriolis IMU physics, Cholesky (`LDLT`) linear solves, Mahalanobis $\chi^2$ outlier gating, Joseph-form covariance updates, and per-sensor differential odometry adaptation.
- **Thin ROS 2 Humble Wrapper (`slam_sensor_fusion_node`)**: Parameterized via YAML (`config/ekf_params.yaml`) for all topic names, frame IDs, sensor fusion modes, fallback covariances, gating thresholds, and optional TF2 broadcasting (`publish_tf: false` by default to avoid colliding with Gazebo's built-in odometry TF).

## Documentation

- [Engineering & Agent Rules](rules.md)
- [System Architecture Overview](docs/architecture/overview.md)
- [Sensor Fusion Node Architecture](docs/architecture/sensor_fusion_node.md)
- [2D EKF Theory & Math Derivations](docs/theory/ekf_2d.md)
- [Proposed Team Sensor Interface Contract](docs/interfaces/sensor_contract.md)

## Build & Test

### 1. Build Full ROS 2 Humble Workspace (`colcon`)
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

### 2. Standalone Pure-C++ Core Build with ASan + UBSan (Fast, No ROS Required)
```bash
cmake -S src/slam_sensor_fusion -B build_sanitizers \
  -DSLAM_BUILD_ROS=OFF \
  -DSLAM_ENABLE_SANITIZERS=ON \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build_sanitizers -j
ctest --test-dir build_sanitizers --output-on-failure
```

## Running with Gazebo (`shell_urban_simulator`)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch slam_sensor_fusion sensor_fusion.launch.py use_sim_time:=true
```

To customize topics, frames, or sensor fusion modes when integrating teammates' nodes, edit [ekf_params.yaml](src/slam_sensor_fusion/config/ekf_params.yaml) or pass `params_file:=/path/to/custom.yaml`.

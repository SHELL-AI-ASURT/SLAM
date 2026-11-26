# Proposed Sensor Fusion Team Interface Contract (Draft v1)

This document proposes the ROS 2 Humble interface contract between the **Sensor Fusion Node** (`slam_sensor_fusion`) and upstream team nodes. All topic names, frame IDs, fusion modes, and fallback covariances are configurable via `config/ekf_params.yaml`, so adjustments can be made without changing core C++ code.

---

## 1. Coordinate & Time Conventions (Mandatory for All Nodes)

- **Units (REP-103)**: SI only (`m`, `rad`, `s`, `m/s`, `rad/s`, `m/s^2`).
- **Frames (REP-103 / REP-105)**:
  - World/odometry reference frame: `odom` (right-handed ENU, $+Z$ up, yaw CCW positive from $+X$).
  - Vehicle body frame: `base_link` (right-handed FLU: $+X$ forward, $+Y$ left, $+Z$ up).
  - Sensor frames from `shell_urban_simulator/urdf/robot.sdf`: `imu_link`, `velodyne_link`, `zed_link`.
  - **No leading `/` in `frame_id` or `child_frame_id`** (e.g., use `velodyne_link`, not `/velodyne_link`).
- **Timestamps**: Every message **must** set `header.stamp` from the ROS node clock (`use_sim_time: true` in Gazebo), never zero or system wall-clock time.
- **TF Ownership**: Do **not** broadcast `odom -> base_link` or `map -> odom` from individual sensor nodes — only the simulator (during raw testing) or the sensor fusion node (when `publish_tf: true`) should broadcast vehicle TF edges.

---

## 2. Proposed Topics & Message Types

| Upstream Node | Proposed Topic | Message Type | Expected Rate | Recommended Fusion Mode | Required Fields |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **IMU** (raw or preprocessed) | `/imu` | `sensor_msgs/msg/Imu` | 100–400 Hz | `imu` (`yaw_rate` + `linear_accel`; `yaw` optional) | `header.stamp`, `header.frame_id` (`imu_link` or `base_link`), `angular_velocity.z`, `linear_acceleration.x/y/z`, `orientation` (if gravity comp or yaw enabled) |
| **IMU / Wheel Odom Node** | `/wheel_odom` *(sim default: `/odom`)* | `nav_msgs/msg/Odometry` | 30–100 Hz | `twist` | `header.stamp`, `header.frame_id` (`odom`), `child_frame_id` (`base_link`), `twist.twist.linear.x/y`, `twist.twist.angular.z` |
| **LiDAR Scan Matching Node** | `/lidar_odom` | `nav_msgs/msg/Odometry` | 10 Hz | `pose_differential` *(or `pose_absolute` if scan-to-map)* | `header.stamp`, `header.frame_id` (`odom` or `map`), `child_frame_id` (`base_link` or `velodyne_link`), `pose.pose` (and `twist` if available) |
| **Visual Odometry Node** | `/visual_odom` | `nav_msgs/msg/Odometry` | 15–30 Hz | `pose_differential` *(or `twist`)* | `header.stamp`, `header.frame_id` (`odom`), `child_frame_id` (`base_link` or `zed_link`), `pose.pose` (and `twist` if available) |

### Sensor Fusion Node Outputs

| Output Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/slam/odometry/filtered` | `nav_msgs/msg/Odometry` | Fused 2D pose (`odom`) & body twist (`base_link`) with full 6x6 covariances |
| `/slam/path/filtered` | `nav_msgs/msg/Path` | Bounded fused trajectory for RViz2 visualization |
| `/tf` *(optional, `publish_tf: false` by default)* | `tf2_msgs/msg/TFMessage` | `odom -> base_link` transform when enabled |

---

## 3. Covariance Guidelines

- Populate `pose.covariance` / `twist.covariance` (row-major 6x6) and IMU 3x3 covariances with realistic positive diagonal variances whenever possible.
- If a node cannot estimate dynamic covariance yet, leave the covariance matrix **all zeros** (`0.0`); the sensor fusion node detects zero/invalid covariances and cleanly substitutes its YAML fallback variances.
- **Never** publish `NaN`, `Inf`, or negative diagonal variances.
- For LiDAR scan matching and VO, if tracking fails or scan registration diverges, either **skip publishing** that frame or publish a significantly inflated covariance so Mahalanobis gating / weighting protects the fused estimate.

---

## 4. Open Questions to Finalize with Teammates

1. **IMU / Wheel Odom node**: Will this node publish a combined `nav_msgs/Odometry` (already fusing IMU + wheel encoders), or separate `/imu` and `/wheel_odom` topics? *(Important: if it already fuses IMU yaw rate into its twist, we should avoid double-fusing the same raw IMU signal without accounting for correlation.)*
2. **LiDAR Scan Matching**: Does it perform **scan-to-scan** (drifting odometry $\to$ use `pose_differential`) or **scan-to-map** (global map-frame pose $\to$ use `pose_absolute`)?
3. **Output Frame**: Will LiDAR and VO nodes transform their output poses into `base_link` themselves, or publish in `velodyne_link` / `zed_link`? *(Both are supported automatically via TF2 as long as `child_frame_id` is set accurately.)*
4. **TF Ownership in Final Demo**: Should `slam_sensor_fusion` own `odom -> base_link` (requiring disabling Gazebo's `OdometryPublisher` `/tf` bridge), or publish only `/slam/odometry/filtered` alongside Gazebo's ground-truth TF?

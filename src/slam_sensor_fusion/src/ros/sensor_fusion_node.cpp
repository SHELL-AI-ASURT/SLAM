#include "slam_sensor_fusion/ros/sensor_fusion_node.hpp"

#include "slam_sensor_fusion/core/math_utils.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace slam_sensor_fusion::ros_node {

namespace {

constexpr int64_t kThrottlePeriodMs = 2000;

[[nodiscard]] core::StateVector parseStateVectorParam(const std::vector<double>& values,
                                                      const std::string& param_name,
                                                      const bool require_strictly_positive) {
  if (values.size() != static_cast<std::size_t>(core::kStateDim)) {
    throw std::invalid_argument(param_name + " must contain exactly 8 elements.");
  }
  core::StateVector vec;
  for (Eigen::Index i = 0; i < core::kStateDim; ++i) {
    const double v = values[static_cast<std::size_t>(i)];
    if (!std::isfinite(v) || (require_strictly_positive ? (v <= 0.0) : (v < 0.0))) {
      throw std::invalid_argument(param_name + " contains invalid non-finite or out-of-range value.");
    }
    vec(i) = v;
  }
  return vec;
}

[[nodiscard]] Eigen::Vector3d parsePositiveVec3Param(const std::vector<double>& values,
                                                     const std::string& param_name) {
  if (values.size() != 3U) {
    throw std::invalid_argument(param_name + " must contain exactly 3 elements.");
  }
  Eigen::Vector3d vec;
  for (Eigen::Index i = 0; i < 3; ++i) {
    const double v = values[static_cast<std::size_t>(i)];
    if (!std::isfinite(v) || v <= 0.0) {
      throw std::invalid_argument(param_name + " must contain finite strictly positive values.");
    }
    vec(i) = v;
  }
  return vec;
}

[[nodiscard]] Eigen::Vector2d parsePositiveVec2Param(const std::vector<double>& values,
                                                     const std::string& param_name) {
  if (values.size() != 2U) {
    throw std::invalid_argument(param_name + " must contain exactly 2 elements.");
  }
  Eigen::Vector2d vec;
  for (Eigen::Index i = 0; i < 2; ++i) {
    const double v = values[static_cast<std::size_t>(i)];
    if (!std::isfinite(v) || v <= 0.0) {
      throw std::invalid_argument(param_name + " must contain finite strictly positive values.");
    }
    vec(i) = v;
  }
  return vec;
}

}  // namespace

SensorFusionNode::SensorFusionNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("slam_sensor_fusion_node", options) {
  loadAndValidateParameters();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(output_odom_topic_, rclcpp::QoS(20));
  if (publish_path_) {
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(output_path_topic_, rclcpp::QoS(10));
  }

  if (imu_sensor_.enabled) {
    imu_sensor_.subscription = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_sensor_.topic, makeSubscriptionQoS(imu_sensor_.use_sensor_data_qos),
        [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg) { handleImuCallback(msg); });
  }

  auto subscribe_odom = [this](OdomSensorRuntime& sensor) {
    if (!sensor.enabled) {
      return;
    }
    sensor.subscription = this->create_subscription<nav_msgs::msg::Odometry>(
        sensor.topic, makeSubscriptionQoS(sensor.use_sensor_data_qos),
        [this, &sensor](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          handleOdomCallback(msg, sensor);
        });
  };

  subscribe_odom(wheel_odom_sensor_);
  subscribe_odom(lidar_odom_sensor_);
  subscribe_odom(visual_odom_sensor_);

  const auto timer_period =
      std::chrono::duration<double>(status_log_period_sec_);
  health_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
      [this]() { handleHealthTimer(); });

  RCLCPP_INFO(
      this->get_logger(),
      "slam_sensor_fusion_node started: odom_frame='%s', base_frame='%s', publish_tf=%s, "
      "imu='%s'(%s), wheel='%s'(%s), lidar='%s'(%s), visual='%s'(%s)",
      odom_frame_id_.c_str(), base_frame_id_.c_str(), publish_tf_ ? "true" : "false",
      imu_sensor_.topic.c_str(), imu_sensor_.enabled ? "on" : "off",
      wheel_odom_sensor_.topic.c_str(), wheel_odom_sensor_.enabled ? "on" : "off",
      lidar_odom_sensor_.topic.c_str(), lidar_odom_sensor_.enabled ? "on" : "off",
      visual_odom_sensor_.topic.c_str(), visual_odom_sensor_.enabled ? "on" : "off");
}

OdomFusionMode SensorFusionNode::parseFusionMode(const std::string& mode_str,
                                                 const std::string& param_name) {
  if (mode_str == "twist") {
    return OdomFusionMode::kTwist;
  }
  if (mode_str == "pose_absolute") {
    return OdomFusionMode::kPoseAbsolute;
  }
  if (mode_str == "pose_differential") {
    return OdomFusionMode::kPoseDifferential;
  }
  if (mode_str == "pose_and_twist") {
    return OdomFusionMode::kPoseAndTwist;
  }
  throw std::invalid_argument(param_name + " has unsupported fusion_mode '" + mode_str +
                              "'. Expected: twist | pose_absolute | pose_differential | pose_and_twist.");
}

rclcpp::QoS SensorFusionNode::makeSubscriptionQoS(const bool use_sensor_data_qos) const {
  if (use_sensor_data_qos) {
    return rclcpp::SensorDataQoS();
  }
  return rclcpp::QoS(rclcpp::KeepLast(50));
}

void SensorFusionNode::loadAndValidateParameters() {
  odom_frame_id_ = sanitizeFrameId(this->declare_parameter<std::string>("odom_frame_id", "odom"));
  base_frame_id_ =
      sanitizeFrameId(this->declare_parameter<std::string>("base_frame_id", "base_link"));
  output_odom_topic_ =
      this->declare_parameter<std::string>("output_odom_topic", "/slam/odometry/filtered");
  output_path_topic_ =
      this->declare_parameter<std::string>("output_path_topic", "/slam/path/filtered");
  publish_tf_ = this->declare_parameter<bool>("publish_tf", false);
  publish_path_ = this->declare_parameter<bool>("publish_path", true);
  const int64_t max_path_len = this->declare_parameter<int64_t>("max_path_length", 2000);
  path_min_distance_m_ = this->declare_parameter<double>("path_min_distance_m", 0.05);
  tf_lookup_timeout_sec_ = this->declare_parameter<double>("tf_lookup_timeout_sec", 0.02);
  status_log_period_sec_ = this->declare_parameter<double>("status_log_period_sec", 5.0);

  if (odom_frame_id_.empty() || base_frame_id_.empty()) {
    throw std::invalid_argument("odom_frame_id and base_frame_id must be non-empty.");
  }
  if (max_path_len <= 0 || !std::isfinite(path_min_distance_m_) || path_min_distance_m_ < 0.0 ||
      !std::isfinite(tf_lookup_timeout_sec_) || tf_lookup_timeout_sec_ < 0.0 ||
      !std::isfinite(status_log_period_sec_) || status_log_period_sec_ <= 0.0) {
    throw std::invalid_argument("Invalid output/path/timer parameter values.");
  }
  max_path_length_ = static_cast<std::size_t>(max_path_len);

  // Core EKF parameters
  core::EkfConfig ekf_config{};
  ekf_config.min_dt_sec = this->declare_parameter<double>("ekf.min_dt_sec", 1.0e-4);
  ekf_config.max_dt_sec = this->declare_parameter<double>("ekf.max_dt_sec", 1.0);
  ekf_config.max_substep_dt_sec = this->declare_parameter<double>("ekf.max_substep_dt_sec", 0.02);
  out_of_order_tolerance_sec_ =
      this->declare_parameter<double>("ekf.out_of_order_tolerance_sec", 0.05);
  clock_reset_threshold_sec_ =
      this->declare_parameter<double>("ekf.clock_reset_threshold_sec", 1.0);

  if (!std::isfinite(out_of_order_tolerance_sec_) || out_of_order_tolerance_sec_ < 0.0 ||
      !std::isfinite(clock_reset_threshold_sec_) || clock_reset_threshold_sec_ <= 0.0) {
    throw std::invalid_argument("Invalid out_of_order_tolerance_sec or clock_reset_threshold_sec.");
  }

  const auto init_cov_vec = this->declare_parameter<std::vector<double>>(
      "ekf.initial_covariance_diag", {0.25, 0.25, 0.10, 0.50, 0.10, 0.10, 1.00, 1.00});
  const auto q_rate_vec = this->declare_parameter<std::vector<double>>(
      "ekf.process_noise_rate_diag", {0.02, 0.02, 0.01, 0.50, 0.05, 0.10, 4.00, 2.00});

  ekf_config.initial_covariance_diag =
      parseStateVectorParam(init_cov_vec, "ekf.initial_covariance_diag", true);
  ekf_config.process_noise_rate_diag =
      parseStateVectorParam(q_rate_vec, "ekf.process_noise_rate_diag", false);

  ekf_ = std::make_unique<core::Ekf2D>(ekf_config);

  // IMU parameters
  imu_sensor_.name = "imu";
  imu_sensor_.enabled = this->declare_parameter<bool>("imu.enabled", true);
  imu_sensor_.topic = this->declare_parameter<std::string>("imu.topic", "/imu");
  imu_sensor_.use_sensor_data_qos = this->declare_parameter<bool>("imu.use_sensor_data_qos", false);
  imu_sensor_.conversion.use_yaw = this->declare_parameter<bool>("imu.use_yaw", false);
  imu_sensor_.conversion.use_yaw_rate = this->declare_parameter<bool>("imu.use_yaw_rate", true);
  imu_sensor_.conversion.use_linear_accel =
      this->declare_parameter<bool>("imu.use_linear_accel", true);
  imu_sensor_.conversion.compensate_gravity =
      this->declare_parameter<bool>("imu.compensate_gravity", true);
  imu_sensor_.conversion.use_message_covariance =
      this->declare_parameter<bool>("imu.use_message_covariance", true);
  imu_sensor_.conversion.fallback_yaw_variance =
      this->declare_parameter<double>("imu.fallback_yaw_variance", 0.05);
  imu_sensor_.conversion.fallback_yaw_rate_variance =
      this->declare_parameter<double>("imu.fallback_yaw_rate_variance", 0.002);
  imu_sensor_.conversion.fallback_accel_variance = parsePositiveVec2Param(
      this->declare_parameter<std::vector<double>>("imu.fallback_accel_variance", {0.25, 0.25}),
      "imu.fallback_accel_variance");
  imu_sensor_.conversion.mahalanobis_threshold =
      this->declare_parameter<double>("imu.mahalanobis_threshold", 13.277);
  imu_sensor_.timeout_sec = this->declare_parameter<double>("imu.timeout_sec", 0.5);

  if (!std::isfinite(imu_sensor_.conversion.fallback_yaw_variance) ||
      imu_sensor_.conversion.fallback_yaw_variance <= 0.0 ||
      !std::isfinite(imu_sensor_.conversion.fallback_yaw_rate_variance) ||
      imu_sensor_.conversion.fallback_yaw_rate_variance <= 0.0 ||
      !std::isfinite(imu_sensor_.conversion.mahalanobis_threshold) ||
      !std::isfinite(imu_sensor_.timeout_sec) || imu_sensor_.timeout_sec <= 0.0) {
    throw std::invalid_argument("Invalid IMU parameter values.");
  }

  // Odometry sensors
  wheel_odom_sensor_ = declareOdomSensorParams(
      "wheel_odom", "/odom", "twist", {0.05, 0.05, 0.02}, {0.04, 0.01, 0.02}, {0.002, 0.002, 0.001},
      0.5);
  lidar_odom_sensor_ = declareOdomSensorParams(
      "lidar_odom", "/lidar_odom", "pose_differential", {0.04, 0.04, 0.015}, {0.05, 0.05, 0.02},
      {0.004, 0.004, 0.0015}, 1.0);
  visual_odom_sensor_ = declareOdomSensorParams(
      "visual_odom", "/visual_odom", "pose_differential", {0.09, 0.09, 0.03}, {0.08, 0.08, 0.04},
      {0.008, 0.008, 0.003}, 1.0);
}

SensorFusionNode::OdomSensorRuntime SensorFusionNode::declareOdomSensorParams(
    const std::string& prefix,
    const std::string& default_topic,
    const std::string& default_mode,
    const std::vector<double>& default_pose_var,
    const std::vector<double>& default_twist_var,
    const std::vector<double>& default_diff_var,
    const double default_timeout_sec) {
  OdomSensorRuntime sensor{};
  sensor.name = prefix;
  sensor.enabled = this->declare_parameter<bool>(prefix + ".enabled", true);
  sensor.topic = this->declare_parameter<std::string>(prefix + ".topic", default_topic);
  sensor.use_sensor_data_qos =
      this->declare_parameter<bool>(prefix + ".use_sensor_data_qos", false);

  const std::string mode_str =
      this->declare_parameter<std::string>(prefix + ".fusion_mode", default_mode);
  sensor.mode = parseFusionMode(mode_str, prefix + ".fusion_mode");

  sensor.conversion.use_position = this->declare_parameter<bool>(prefix + ".use_position", true);
  sensor.conversion.use_yaw = this->declare_parameter<bool>(prefix + ".use_yaw", true);
  sensor.conversion.use_vx = this->declare_parameter<bool>(prefix + ".use_vx", true);
  sensor.conversion.use_vy = this->declare_parameter<bool>(prefix + ".use_vy", true);
  sensor.conversion.use_omega = this->declare_parameter<bool>(prefix + ".use_omega", true);
  sensor.conversion.use_message_covariance =
      this->declare_parameter<bool>(prefix + ".use_message_covariance", true);

  sensor.conversion.fallback_pose_variance = parsePositiveVec3Param(
      this->declare_parameter<std::vector<double>>(prefix + ".fallback_pose_variance",
                                                   default_pose_var),
      prefix + ".fallback_pose_variance");
  sensor.conversion.fallback_twist_variance = parsePositiveVec3Param(
      this->declare_parameter<std::vector<double>>(prefix + ".fallback_twist_variance",
                                                   default_twist_var),
      prefix + ".fallback_twist_variance");
  const Eigen::Vector3d diff_var = parsePositiveVec3Param(
      this->declare_parameter<std::vector<double>>(prefix + ".differential_step_variance",
                                                   default_diff_var),
      prefix + ".differential_step_variance");

  sensor.conversion.mahalanobis_threshold =
      this->declare_parameter<double>(prefix + ".mahalanobis_threshold", 11.345);
  sensor.timeout_sec = this->declare_parameter<double>(prefix + ".timeout_sec", default_timeout_sec);

  if (!std::isfinite(sensor.conversion.mahalanobis_threshold) ||
      !std::isfinite(sensor.timeout_sec) || sensor.timeout_sec <= 0.0) {
    throw std::invalid_argument("Invalid threshold or timeout parameter for " + prefix);
  }

  core::DifferentialAdapterConfig diff_cfg{};
  diff_cfg.min_dt_sec = ekf_->config().min_dt_sec;
  diff_cfg.max_dt_sec = ekf_->config().max_dt_sec;
  diff_cfg.default_step_covariance = diff_var.asDiagonal();
  diff_cfg.use_measurement_covariance = false;
  diff_cfg.mahalanobis_threshold = sensor.conversion.mahalanobis_threshold;
  sensor.diff_adapter = std::make_unique<core::DifferentialAdapter>(diff_cfg);

  return sensor;
}

std::optional<Extrinsic2D> SensorFusionNode::lookupExtrinsicToBase(
    const std::string& raw_sensor_frame_id,
    const rclcpp::Time& stamp) {
  const std::string sensor_frame = sanitizeFrameId(raw_sensor_frame_id);
  if (sensor_frame.empty() || sensor_frame == base_frame_id_) {
    return Extrinsic2D::identity();
  }

  try {
    const auto timeout = rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_);
    const auto tf_msg = tf_buffer_->lookupTransform(base_frame_id_, sensor_frame, stamp, timeout);
    return extractExtrinsic2D(tf_msg);
  } catch (const tf2::TransformException&) {
    // Retry with latest available static transform (TimePointZero) for sim-time startup races.
    try {
      const auto tf_msg =
          tf_buffer_->lookupTransform(base_frame_id_, sensor_frame, tf2::TimePointZero);
      return extractExtrinsic2D(tf_msg);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), kThrottlePeriodMs,
                           "TF lookup '%s' <- '%s' failed (%s); assuming identity extrinsic.",
                           base_frame_id_.c_str(), sensor_frame.c_str(), ex.what());
      return Extrinsic2D::identity();
    }
  }
}

SensorFusionNode::TimeAdvanceAction SensorFusionNode::advanceFilterToTimestampLocked(
    const double msg_time_sec,
    const std::optional<core::Pose2DMeasurement>& initial_pose_hint) {
  if (!std::isfinite(msg_time_sec)) {
    return TimeAdvanceAction::kDropStale;
  }

  if (!ekf_->isInitialized()) {
    core::StateVector init_mean = core::StateVector::Zero();
    if (initial_pose_hint.has_value()) {
      if (initial_pose_hint->use_position) {
        init_mean(core::toIndex(core::StateIndex::kX)) = initial_pose_hint->x;
        init_mean(core::toIndex(core::StateIndex::kY)) = initial_pose_hint->y;
      }
      if (initial_pose_hint->use_yaw) {
        init_mean(core::toIndex(core::StateIndex::kTheta)) =
            core::wrapAngle(initial_pose_hint->theta);
      }
    }
    const core::StateMatrix init_cov = ekf_->config().initial_covariance_diag.asDiagonal();
    if (!ekf_->initialize(init_mean, init_cov, msg_time_sec)) {
      return TimeAdvanceAction::kDropStale;
    }
    return TimeAdvanceAction::kReady;
  }

  const double dt_sec = msg_time_sec - ekf_->state().timestamp_sec;

  // Detect Gazebo simulation restart (large backward time jump).
  if (dt_sec < -clock_reset_threshold_sec_) {
    ++stats_.total_clock_resets;
    RCLCPP_WARN(this->get_logger(),
                "Detected backward clock jump of %.3f s; resetting EKF and adapters.", dt_sec);
    resetFilterLocked();
    (void)advanceFilterToTimestampLocked(msg_time_sec, initial_pose_hint);
    return TimeAdvanceAction::kResetTriggered;
  }

  // Drop stale messages beyond the out-of-order tolerance window.
  if (dt_sec < -out_of_order_tolerance_sec_) {
    ++stats_.total_dropped_stale;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), kThrottlePeriodMs,
                         "Dropping stale message (dt = %.4f s < -%.4f s).", dt_sec,
                         out_of_order_tolerance_sec_);
    return TimeAdvanceAction::kDropStale;
  }

  // Minor out-of-order message: update at current state without backward prediction.
  if (dt_sec < 0.0) {
    ++stats_.total_out_of_order_updates;
    return TimeAdvanceAction::kReady;
  }

  const core::PredictStatus pred_status = ekf_->predict(dt_sec);
  if (pred_status == core::PredictStatus::kInvalidDt) {
    // Large forward jump (e.g., Gazebo paused and unpaused): re-anchor timestamp cleanly.
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), kThrottlePeriodMs,
                         "Skipping prediction across large forward gap (dt = %.3f s > %.3f s).",
                         dt_sec, ekf_->config().max_dt_sec);
    const core::EkfState cur = ekf_->state();
    (void)ekf_->initialize(cur.mean, cur.covariance, msg_time_sec);
  }
  return TimeAdvanceAction::kReady;
}

void SensorFusionNode::recordUpdateResultLocked(const core::UpdateResult& result,
                                                const std::string& sensor_name) {
  switch (result.status) {
    case core::UpdateStatus::kUpdated:
      ++stats_.total_updates_accepted;
      break;
    case core::UpdateStatus::kRejectedOutlier:
      ++stats_.total_updates_rejected_outlier;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), kThrottlePeriodMs,
                           "Sensor '%s' measurement rejected by Mahalanobis gate (d_M^2 = %.2f).",
                           sensor_name.c_str(), result.mahalanobis_sq);
      break;
    case core::UpdateStatus::kNoActiveChannels:
      break;
    default:
      ++stats_.total_updates_invalid;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), kThrottlePeriodMs,
                           "Sensor '%s' measurement update skipped (status=%d).",
                           sensor_name.c_str(), static_cast<int>(result.status));
      break;
  }
}

void SensorFusionNode::handleImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  if (!msg) {
    return;
  }

  const rclcpp::Time stamp =
      (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
          ? this->get_clock()->now()
          : rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());

  const auto extrinsic = lookupExtrinsicToBase(msg->header.frame_id, stamp);
  if (!extrinsic.has_value()) {
    return;
  }

  const auto converted = convertImuMessage(*msg, imu_sensor_.conversion, *extrinsic);
  if (!converted.has_value()) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.total_updates_invalid;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), kThrottlePeriodMs,
                         "Received invalid/non-finite IMU message; skipping.");
    return;
  }

  if (converted->used_fallback_covariance) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), kThrottlePeriodMs * 5,
                         "IMU message covariance missing/invalid; using YAML fallback covariance.");
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const double msg_time_sec = stamp.seconds();
  imu_sensor_.last_msg_time_sec = msg_time_sec;
  imu_sensor_.timed_out_logged = false;

  const auto action = advanceFilterToTimestampLocked(msg_time_sec, std::nullopt);
  if (action == TimeAdvanceAction::kDropStale) {
    return;
  }

  const core::UpdateResult result = ekf_->updateImu(converted->measurement);
  recordUpdateResultLocked(result, imu_sensor_.name);
  if (result.succeeded()) {
    publishOutputsLocked(stamp);
  }
}

void SensorFusionNode::handleOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg,
                                          OdomSensorRuntime& sensor) {
  if (!msg) {
    return;
  }

  const rclcpp::Time stamp =
      (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
          ? this->get_clock()->now()
          : rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());

  const auto extrinsic = lookupExtrinsicToBase(msg->child_frame_id, stamp);
  if (!extrinsic.has_value()) {
    return;
  }

  std::optional<ConvertedMeasurement<core::Pose2DMeasurement>> converted_pose;
  std::optional<ConvertedMeasurement<core::Twist2DMeasurement>> converted_twist;

  if (sensor.mode == OdomFusionMode::kPoseAbsolute ||
      sensor.mode == OdomFusionMode::kPoseDifferential ||
      sensor.mode == OdomFusionMode::kPoseAndTwist) {
    converted_pose = convertOdomPoseMessage(*msg, sensor.conversion, *extrinsic);
    if (!converted_pose.has_value()) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.total_updates_invalid;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), kThrottlePeriodMs,
                           "Sensor '%s' pose contained non-finite/invalid fields; skipping.",
                           sensor.name.c_str());
      return;
    }
    if (converted_pose->used_fallback_covariance) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), kThrottlePeriodMs * 5,
          "Sensor '%s' pose covariance missing/invalid; using YAML fallback covariance.",
          sensor.name.c_str());
    }
  }

  if (sensor.mode == OdomFusionMode::kTwist || sensor.mode == OdomFusionMode::kPoseAndTwist) {
    converted_twist = convertOdomTwistMessage(*msg, sensor.conversion, *extrinsic);
    if (!converted_twist.has_value()) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.total_updates_invalid;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), kThrottlePeriodMs,
                           "Sensor '%s' twist contained non-finite fields; skipping.",
                           sensor.name.c_str());
      return;
    }
    if (converted_twist->used_fallback_covariance) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), kThrottlePeriodMs * 5,
          "Sensor '%s' twist covariance missing/invalid; using YAML fallback covariance.",
          sensor.name.c_str());
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  const double msg_time_sec = stamp.seconds();
  sensor.last_msg_time_sec = msg_time_sec;
  sensor.timed_out_logged = false;

  std::optional<core::Pose2DMeasurement> init_hint;
  if ((sensor.mode == OdomFusionMode::kPoseAbsolute ||
       sensor.mode == OdomFusionMode::kPoseAndTwist) &&
      converted_pose.has_value()) {
    init_hint = converted_pose->measurement;
  }

  const auto action = advanceFilterToTimestampLocked(msg_time_sec, init_hint);
  if (action == TimeAdvanceAction::kDropStale) {
    return;
  }

  bool any_updated = false;
  switch (sensor.mode) {
    case OdomFusionMode::kTwist: {
      const auto res = ekf_->updateTwist(converted_twist->measurement);
      recordUpdateResultLocked(res, sensor.name);
      any_updated = res.succeeded();
      break;
    }
    case OdomFusionMode::kPoseAbsolute: {
      const auto res = ekf_->updatePose(converted_pose->measurement);
      recordUpdateResultLocked(res, sensor.name);
      any_updated = res.succeeded();
      break;
    }
    case OdomFusionMode::kPoseDifferential: {
      const auto diff = sensor.diff_adapter->process(converted_pose->measurement, msg_time_sec);
      if (diff.hasTwist()) {
        const auto res = ekf_->updateTwist(diff.twist);
        recordUpdateResultLocked(res, sensor.name);
        any_updated = res.succeeded();
      }
      break;
    }
    case OdomFusionMode::kPoseAndTwist: {
      const auto pose_res = ekf_->updatePose(converted_pose->measurement);
      recordUpdateResultLocked(pose_res, sensor.name + "_pose");
      const auto twist_res = ekf_->updateTwist(converted_twist->measurement);
      recordUpdateResultLocked(twist_res, sensor.name + "_twist");
      any_updated = pose_res.succeeded() || twist_res.succeeded();
      break;
    }
  }

  if (any_updated) {
    publishOutputsLocked(stamp);
  }
}

void SensorFusionNode::publishOutputsLocked(const rclcpp::Time& stamp) {
  if (!ekf_->isInitialized()) {
    return;
  }
  const core::EkfState& state = ekf_->state();

  const auto odom_msg = stateToOdometryMessage(state, stamp, odom_frame_id_, base_frame_id_);
  odom_pub_->publish(odom_msg);

  if (publish_tf_ && tf_broadcaster_) {
    tf_broadcaster_->sendTransform(
        stateToTransformMessage(state, stamp, odom_frame_id_, base_frame_id_));
  }

  if (publish_path_ && path_pub_) {
    bool should_append = path_poses_.empty();
    if (!should_append) {
      const auto& last_pos = path_poses_.back().pose.position;
      const double dx = state.x() - last_pos.x;
      const double dy = state.y() - last_pos.y;
      should_append = (dx * dx + dy * dy) >= (path_min_distance_m_ * path_min_distance_m_);
    }

    if (should_append) {
      geometry_msgs::msg::PoseStamped ps{};
      ps.header = odom_msg.header;
      ps.pose = odom_msg.pose.pose;
      path_poses_.push_back(ps);
      while (path_poses_.size() > max_path_length_) {
        path_poses_.pop_front();
      }
    }

    nav_msgs::msg::Path path_msg{};
    path_msg.header.stamp = stamp;
    path_msg.header.frame_id = odom_frame_id_;
    path_msg.poses.assign(path_poses_.begin(), path_poses_.end());
    path_pub_->publish(path_msg);
  }
}

void SensorFusionNode::handleHealthTimer() {
  std::lock_guard<std::mutex> lock(mutex_);
  const double now_sec = this->get_clock()->now().seconds();

  auto check_timeout = [this, now_sec](const std::string& name, const bool enabled,
                                       const std::optional<double>& last_time,
                                       const double timeout_sec, bool& logged_flag) {
    if (!enabled || !last_time.has_value()) {
      return;
    }
    const double age_sec = now_sec - *last_time;
    if (age_sec > timeout_sec && !logged_flag) {
      logged_flag = true;
      RCLCPP_WARN(this->get_logger(),
                  "Sensor '%s' timed out (no data for %.2f s > %.2f s); continuing with remaining "
                  "sensors.",
                  name.c_str(), age_sec, timeout_sec);
    }
  };

  check_timeout(imu_sensor_.name, imu_sensor_.enabled, imu_sensor_.last_msg_time_sec,
                imu_sensor_.timeout_sec, imu_sensor_.timed_out_logged);
  check_timeout(wheel_odom_sensor_.name, wheel_odom_sensor_.enabled,
                wheel_odom_sensor_.last_msg_time_sec, wheel_odom_sensor_.timeout_sec,
                wheel_odom_sensor_.timed_out_logged);
  check_timeout(lidar_odom_sensor_.name, lidar_odom_sensor_.enabled,
                lidar_odom_sensor_.last_msg_time_sec, lidar_odom_sensor_.timeout_sec,
                lidar_odom_sensor_.timed_out_logged);
  check_timeout(visual_odom_sensor_.name, visual_odom_sensor_.enabled,
                visual_odom_sensor_.last_msg_time_sec, visual_odom_sensor_.timeout_sec,
                visual_odom_sensor_.timed_out_logged);

  RCLCPP_INFO(this->get_logger(),
              "EKF status: init=%s, accepted=%lu, outliers=%lu, invalid=%lu, ooo=%lu, stale=%lu",
              ekf_->isInitialized() ? "yes" : "no",
              static_cast<unsigned long>(stats_.total_updates_accepted),
              static_cast<unsigned long>(stats_.total_updates_rejected_outlier),
              static_cast<unsigned long>(stats_.total_updates_invalid),
              static_cast<unsigned long>(stats_.total_out_of_order_updates),
              static_cast<unsigned long>(stats_.total_dropped_stale));
}

core::EkfState SensorFusionNode::currentState() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return ekf_->state();
}

FusionStats SensorFusionNode::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

void SensorFusionNode::resetFilter() {
  std::lock_guard<std::mutex> lock(mutex_);
  resetFilterLocked();
}

void SensorFusionNode::resetFilterLocked() {
  ekf_->reset();
  if (wheel_odom_sensor_.diff_adapter) {
    wheel_odom_sensor_.diff_adapter->reset();
  }
  if (lidar_odom_sensor_.diff_adapter) {
    lidar_odom_sensor_.diff_adapter->reset();
  }
  if (visual_odom_sensor_.diff_adapter) {
    visual_odom_sensor_.diff_adapter->reset();
  }
  path_poses_.clear();
}

}  // namespace slam_sensor_fusion::ros_node

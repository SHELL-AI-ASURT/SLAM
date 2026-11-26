#include "slam_sensor_fusion/core/math_utils.hpp"
#include "slam_sensor_fusion/ros/message_converters.hpp"
#include "slam_sensor_fusion/ros/sensor_fusion_node.hpp"

#include <gtest/gtest.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace slam_sensor_fusion::ros_node {
namespace {

constexpr double kPi = std::numbers::pi_v<double>;
constexpr double kGravity = 9.80665;

geometry_msgs::msg::Quaternion rpyToQuaternion(const double roll,
                                               const double pitch,
                                               const double yaw) {
  const double cr = std::cos(0.5 * roll);
  const double sr = std::sin(0.5 * roll);
  const double cp = std::cos(0.5 * pitch);
  const double sp = std::sin(0.5 * pitch);
  const double cy = std::cos(0.5 * yaw);
  const double sy = std::sin(0.5 * yaw);

  geometry_msgs::msg::Quaternion q{};
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  return q;
}

class RosFixture : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }
  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST(MessageConvertersTest, SanitizeFrameId_StripsLeadingSlashes) {
  EXPECT_EQ(sanitizeFrameId("/velodyne_link"), "velodyne_link");
  EXPECT_EQ(sanitizeFrameId("///base_link"), "base_link");
  EXPECT_EQ(sanitizeFrameId("imu_link"), "imu_link");
  EXPECT_EQ(sanitizeFrameId(""), "");
}

TEST(MessageConvertersTest, ExtractRollPitchYaw_ValidAndDegenerateQuaternions) {
  const auto q = rpyToQuaternion(0.15, -0.22, 1.35);
  const auto rpy = extractRollPitchYaw(q);
  ASSERT_TRUE(rpy.has_value());
  EXPECT_NEAR(rpy->roll, 0.15, 1.0e-12);
  EXPECT_NEAR(rpy->pitch, -0.22, 1.0e-12);
  EXPECT_NEAR(rpy->yaw, 1.35, 1.0e-12);

  geometry_msgs::msg::Quaternion zero_q{};
  zero_q.x = 0.0;
  zero_q.y = 0.0;
  zero_q.z = 0.0;
  zero_q.w = 0.0;
  EXPECT_FALSE(extractRollPitchYaw(zero_q).has_value());

  geometry_msgs::msg::Quaternion nan_q{};
  nan_q.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(extractRollPitchYaw(nan_q).has_value());
}

TEST(MessageConvertersTest, TransformPoseAndTwistToBase_ExactLeverArmKinematics) {
  // Camera mounted 0.85 m in front of base_link (like zed_link in robot.sdf).
  const Extrinsic2D ext{0.85, 0.0, 0.0};

  // Vehicle pivoting in place at base_link with omega = +1.0 rad/s:
  // sensor sees vy_sensor = +omega * tx = +0.85 m/s, vx_sensor = 0.
  core::Twist2DMeasurement twist_sensor{};
  twist_sensor.vx = 0.0;
  twist_sensor.vy = 0.85;
  twist_sensor.omega = 1.0;
  twist_sensor.covariance = Eigen::Matrix3d::Identity() * 0.01;

  const auto twist_base = transformTwist2DToBase(twist_sensor, ext);
  EXPECT_NEAR(twist_base.vx, 0.0, 1.0e-12);
  EXPECT_NEAR(twist_base.vy, 0.0, 1.0e-12);
  EXPECT_NEAR(twist_base.omega, 1.0, 1.0e-12);
  EXPECT_TRUE(core::isSymmetricPositiveDefinite(twist_base.covariance));

  // Pose inverse check: base at (2, 3, pi/2) -> sensor is at (2, 3 + 0.85, pi/2).
  core::Pose2DMeasurement pose_sensor{};
  pose_sensor.x = 2.0;
  pose_sensor.y = 3.85;
  pose_sensor.theta = kPi / 2.0;
  pose_sensor.covariance = Eigen::Matrix3d::Identity() * 0.02;

  const auto pose_base = transformPose2DToBase(pose_sensor, ext);
  EXPECT_NEAR(pose_base.x, 2.0, 1.0e-12);
  EXPECT_NEAR(pose_base.y, 3.0, 1.0e-12);
  EXPECT_NEAR(pose_base.theta, kPi / 2.0, 1.0e-12);
}

TEST(MessageConvertersTest, ConvertImuMessage_GravityCompensationRemovesTiltProjection) {
  const double roll = -0.05;
  const double pitch = 0.08;
  const double yaw = 1.2;

  sensor_msgs::msg::Imu imu{};
  imu.orientation = rpyToQuaternion(roll, pitch, yaw);
  // Stationary sensor measures upward reaction vector rotated into tilted body frame.
  imu.linear_acceleration.x = -kGravity * std::sin(pitch);
  imu.linear_acceleration.y = kGravity * std::sin(roll) * std::cos(pitch);
  imu.linear_acceleration.z = kGravity * std::cos(roll) * std::cos(pitch);
  imu.angular_velocity.z = 0.0;
  // Leave covariances zero to verify YAML fallback triggers.

  ImuConversionOptions opts{};
  opts.use_yaw = true;
  opts.compensate_gravity = true;
  opts.fallback_yaw_variance = 0.07;
  opts.fallback_yaw_rate_variance = 0.003;
  opts.fallback_accel_variance = Eigen::Vector2d(0.3, 0.4);

  const auto converted = convertImuMessage(imu, opts);
  ASSERT_TRUE(converted.has_value());
  EXPECT_TRUE(converted->used_fallback_covariance);
  EXPECT_NEAR(converted->measurement.ax, 0.0, 1.0e-12);
  EXPECT_NEAR(converted->measurement.ay, 0.0, 1.0e-12);
  EXPECT_NEAR(converted->measurement.yaw, yaw, 1.0e-12);
  EXPECT_DOUBLE_EQ(converted->measurement.covariance(0, 0), 0.07);
  EXPECT_DOUBLE_EQ(converted->measurement.covariance(1, 1), 0.003);
  EXPECT_DOUBLE_EQ(converted->measurement.covariance(2, 2), 0.3);
  EXPECT_DOUBLE_EQ(converted->measurement.covariance(3, 3), 0.4);
}

TEST_F(RosFixture, SensorFusionNode_InvalidParameters_ThrowsAtStartup) {
  rclcpp::NodeOptions bad_opts;
  bad_opts.parameter_overrides({rclcpp::Parameter("wheel_odom.fusion_mode", "unsupported_mode")});
  EXPECT_THROW((void)std::make_shared<SensorFusionNode>(bad_opts), std::invalid_argument);
}

TEST_F(RosFixture, SensorFusionNode_MultiSensorPubSub_FusesAndPublishesOdomAndPath) {
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      rclcpp::Parameter("imu.topic", "/test/imu"),
      rclcpp::Parameter("wheel_odom.topic", "/test/wheel_odom"),
      rclcpp::Parameter("wheel_odom.fusion_mode", "twist"),
      rclcpp::Parameter("lidar_odom.topic", "/test/lidar_odom"),
      rclcpp::Parameter("lidar_odom.fusion_mode", "pose_differential"),
      rclcpp::Parameter("visual_odom.topic", "/test/visual_odom"),
      rclcpp::Parameter("visual_odom.fusion_mode", "pose_differential"),
      rclcpp::Parameter("output_odom_topic", "/test/filtered_odom"),
      rclcpp::Parameter("output_path_topic", "/test/filtered_path"),
      rclcpp::Parameter("publish_tf", false),
      rclcpp::Parameter("publish_path", true),
  });

  auto fusion_node = std::make_shared<SensorFusionNode>(opts);
  auto harness_node = std::make_shared<rclcpp::Node>("test_harness_node");

  auto imu_pub = harness_node->create_publisher<sensor_msgs::msg::Imu>("/test/imu", 20);
  auto wheel_pub = harness_node->create_publisher<nav_msgs::msg::Odometry>("/test/wheel_odom", 20);
  auto lidar_pub = harness_node->create_publisher<nav_msgs::msg::Odometry>("/test/lidar_odom", 20);

  std::vector<nav_msgs::msg::Odometry> received_odom;
  std::vector<nav_msgs::msg::Path> received_path;
  auto odom_sub = harness_node->create_subscription<nav_msgs::msg::Odometry>(
      "/test/filtered_odom", 50,
      [&received_odom](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        received_odom.push_back(*msg);
      });
  auto path_sub = harness_node->create_subscription<nav_msgs::msg::Path>(
      "/test/filtered_path", 50,
      [&received_path](const nav_msgs::msg::Path::ConstSharedPtr msg) {
        received_path.push_back(*msg);
      });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(fusion_node);
  exec.add_node(harness_node);

  // Publish 20 steps (dt = 0.05 s, 1.0 s total) of straight motion at vx = 2.0 m/s.
  constexpr double kVx = 2.0;
  constexpr double kDt = 0.05;
  for (int i = 0; i <= 20; ++i) {
    const double t = 10.0 + static_cast<double>(i) * kDt;
    const rclcpp::Time stamp(static_cast<int64_t>(std::llround(t * 1.0e9)));

    sensor_msgs::msg::Imu imu{};
    imu.header.stamp = stamp;
    imu.header.frame_id = "base_link";
    imu.orientation = rpyToQuaternion(0.0, 0.0, 0.0);
    imu.angular_velocity.z = 0.0;
    imu.linear_acceleration.x = 0.0;
    imu.linear_acceleration.y = 0.0;
    imu.linear_acceleration.z = kGravity;
    imu_pub->publish(imu);

    nav_msgs::msg::Odometry wheel{};
    wheel.header.stamp = stamp;
    wheel.header.frame_id = "odom";
    wheel.child_frame_id = "base_link";
    wheel.twist.twist.linear.x = kVx;
    wheel.twist.twist.linear.y = 0.0;
    wheel.twist.twist.angular.z = 0.0;
    wheel_pub->publish(wheel);

    if (i % 2 == 0) {
      nav_msgs::msg::Odometry lidar{};
      lidar.header.stamp = stamp;
      lidar.header.frame_id = "odom";
      lidar.child_frame_id = "base_link";
      lidar.pose.pose.position.x = kVx * (t - 10.0);
      lidar.pose.pose.position.y = 0.0;
      lidar.pose.pose.orientation = rpyToQuaternion(0.0, 0.0, 0.0);
      lidar_pub->publish(lidar);
    }

    for (int spin_iter = 0; spin_iter < 5; ++spin_iter) {
      exec.spin_some(std::chrono::milliseconds(2));
    }
  }

  const auto state = fusion_node->currentState();
  const auto stats = fusion_node->stats();
  ASSERT_TRUE(state.initialized);
  EXPECT_GT(stats.total_updates_accepted, 40U);
  EXPECT_EQ(stats.total_updates_invalid, 0U);
  EXPECT_FALSE(received_odom.empty());
  EXPECT_FALSE(received_path.empty());

  // After 1.0 s at 2.0 m/s (with initial velocity convergence from 0 -> 2 m/s on first step),
  // velocity should closely match 2.0 m/s and x should be ~2.0 m.
  EXPECT_NEAR(state.vx(), kVx, 0.05);
  EXPECT_NEAR(state.vy(), 0.0, 0.05);
  EXPECT_NEAR(state.x(), 2.0, 0.15);
  EXPECT_NEAR(state.y(), 0.0, 0.05);
  EXPECT_EQ(received_odom.back().header.frame_id, "odom");
  EXPECT_EQ(received_odom.back().child_frame_id, "base_link");

  // Verify backward simulation clock reset clears and re-initializes cleanly.
  nav_msgs::msg::Odometry reset_msg{};
  reset_msg.header.stamp = rclcpp::Time(1, 0);
  reset_msg.header.frame_id = "odom";
  reset_msg.child_frame_id = "base_link";
  reset_msg.twist.twist.linear.x = 0.0;
  wheel_pub->publish(reset_msg);
  for (int spin_iter = 0; spin_iter < 5; ++spin_iter) {
    exec.spin_some(std::chrono::milliseconds(2));
  }
  EXPECT_EQ(fusion_node->stats().total_clock_resets, 1U);
  EXPECT_NEAR(fusion_node->currentState().x(), 0.0, 1.0e-6);
}

}  // namespace
}  // namespace slam_sensor_fusion::ros_node

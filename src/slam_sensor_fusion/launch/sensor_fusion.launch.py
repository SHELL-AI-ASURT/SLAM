"""Launch file for the slam_sensor_fusion 2D EKF sensor fusion node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the launch description for slam_sensor_fusion_node."""
    pkg_share = get_package_share_directory("slam_sensor_fusion")
    default_params_file = os.path.join(pkg_share, "config", "ekf_params.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock (/clock) when running with Gazebo.",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to the YAML parameter file for slam_sensor_fusion_node.",
    )

    sensor_fusion_node = Node(
        package="slam_sensor_fusion",
        executable="sensor_fusion_node",
        name="slam_sensor_fusion_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        sensor_fusion_node,
    ])

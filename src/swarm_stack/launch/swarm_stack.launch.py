"""Launch comms_sim_node, mission_manager_node and metrics_logger_node together.

Run this AFTER ardupilot_gz_bringup's multiagent.launch.py is already up and
all UAVs have finished booting and connecting to Gazebo.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("swarm_stack")
    default_poi_path = os.path.join(pkg_share, "config", "pois.yaml")

    uav_names_arg = DeclareLaunchArgument(
        "uav_names", default_value="['drone1','drone2','drone3','drone4','drone5']",
        description="YAML list of UAV namespaces to manage.")
    poi_path_arg = DeclareLaunchArgument(
        "poi_config_path", default_value=default_poi_path,
        description="Path to the PoI YAML config.")

    uav_names = LaunchConfiguration("uav_names")
    poi_config_path = LaunchConfiguration("poi_config_path")

    comms_node = Node(
        package="swarm_stack", executable="comms_sim_node", name="comms_sim_node",
        parameters=[{"uav_names": uav_names}], output="screen",
    )
    mission_node = Node(
        package="swarm_stack", executable="mission_manager_node", name="mission_manager_node",
        parameters=[{"uav_names": uav_names, "poi_config_path": poi_config_path}], output="screen",
    )
    metrics_node = Node(
        package="swarm_stack", executable="metrics_logger_node", name="metrics_logger_node",
        parameters=[{"uav_names": uav_names}], output="screen",
    )

    return LaunchDescription([
        uav_names_arg, poi_path_arg, comms_node, mission_node, metrics_node,
    ])

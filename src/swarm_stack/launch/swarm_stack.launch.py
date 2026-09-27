"""Launch comms_sim_node, mission_manager_node and metrics_logger_node together.

Run this AFTER ardupilot_gz_bringup's multiagent.launch.py is already up and
all UAVs have finished booting and connecting to Gazebo.
"""
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("swarm_stack")
    default_poi_path = os.path.join(pkg_share, "config", "pois.yaml")

    uav_names_arg = DeclareLaunchArgument(
        "uav_names", default_value="[" + ",".join(f"drone{i}" for i in range(1, 11)) + "]",
        description="YAML list of UAV namespaces to manage.")
    poi_path_arg = DeclareLaunchArgument(
        "poi_config_path", default_value=default_poi_path,
        description="Path to the PoI YAML config.")
    map_view_arg = DeclareLaunchArgument(
        "map_view", default_value="false",
        description="Open one RViz top-down mission map with PoIs and relay links.")

    scenario_arg = DeclareLaunchArgument("scenario_config_path",
        default_value=os.path.join(pkg_share, "config", "sample_scenario.yaml"),
        description="Sample constraints; empty string selects the historical small demo.")
    dashboard_arg = DeclareLaunchArgument("dashboard", default_value="false",
        description="Open the Python live swarm dashboard.")
    scenario = LaunchConfiguration("scenario_config_path")
    uav_names = LaunchConfiguration("uav_names")
    poi_config_path = LaunchConfiguration("poi_config_path")

    comms_node = Node(
        package="swarm_stack", executable="comms_sim_node", name="comms_sim_node",
        parameters=[{"uav_names": uav_names, "scenario_config_path": scenario}], output="screen",
    )
    mission_python = os.path.join(os.environ.get("VIRTUAL_ENV", ""), "bin", "python3")
    if not os.path.isfile(mission_python):
        mission_python = sys.executable
    mission_node = Node(
        package="swarm_stack", executable="mission_manager_node", name="mission_manager_node",
        prefix=mission_python, parameters=[{"uav_names": uav_names, "poi_config_path": poi_config_path, "scenario_config_path": scenario}], output="screen",
    )
    metrics_node = Node(
        package="swarm_stack", executable="metrics_logger_node", name="metrics_logger_node",
        parameters=[{"uav_names": uav_names, "scenario_config_path": scenario}], output="screen",
    )

    map_node = Node(
        package="swarm_stack", executable="visualization_node", name="swarm_visualization_node",
        parameters=[{"scenario_config_path": scenario, "uav_names": uav_names}],
        condition=IfCondition(LaunchConfiguration("map_view")), output="screen",
    )
    dashboard_node = Node(
        package="swarm_stack", executable="scenario_viewer", name="swarm_dashboard",
        arguments=["--live", "--scenario", scenario],
        condition=IfCondition(LaunchConfiguration("dashboard")), output="screen",
    )
    map_rviz = Node(
        package="rviz2", executable="rviz2", name="swarm_map_rviz",
        arguments=["-d", os.path.join(pkg_share, "rviz", "swarm_map.rviz")],
        condition=IfCondition(LaunchConfiguration("map_view")), output="screen",
    )

    return LaunchDescription([
        uav_names_arg, poi_path_arg, map_view_arg, scenario_arg, dashboard_arg, comms_node, mission_node,
        metrics_node, map_node, map_rviz, dashboard_node,
    ])

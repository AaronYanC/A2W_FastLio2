"""Run stable JT128 FAST-LIO plus read-only Map Bundle localization."""

import os.path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("a2w_fastlio2_bringup")
    localization_share = get_package_share_directory("a2w_fastlio_localization")

    map_bundle_path = LaunchConfiguration("map_bundle_path")
    jt128_config = LaunchConfiguration("jt128_config")
    localization_config = LaunchConfiguration("localization_config")
    matching_config = LaunchConfiguration("matching_config")
    topics_config = LaunchConfiguration("topics_config")
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    arguments = [
        DeclareLaunchArgument(
            "map_bundle_path",
            description="Absolute path to a verified Map Bundle V1 directory",
        ),
        DeclareLaunchArgument(
            "jt128_config",
            default_value=os.path.join(bringup_share, "config", "jt128.yaml"),
        ),
        DeclareLaunchArgument(
            "localization_config",
            default_value=os.path.join(localization_share, "config", "localization.yaml"),
        ),
        DeclareLaunchArgument(
            "matching_config",
            default_value=os.path.join(localization_share, "config", "map_matching.yaml"),
        ),
        DeclareLaunchArgument(
            "topics_config",
            default_value=os.path.join(localization_share, "config", "localization_topics.yaml"),
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]
    frontend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "jt128_mapping.launch.py")
        ),
        launch_arguments={
            "config_file": jt128_config,
            "save_map": "false",
            "map_file": "",
            "rviz": rviz,
            "use_sim_time": use_sim_time,
        }.items(),
    )
    localization = Node(
        package="a2w_fastlio_localization",
        executable="localization_node",
        parameters=[
            localization_config,
            matching_config,
            topics_config,
            {"map_bundle_path": ParameterValue(map_bundle_path, value_type=str)},
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        output="screen",
    )
    # This profile contains Localization only; Mapping nodes are deliberately absent.
    return LaunchDescription([*arguments, frontend, localization])

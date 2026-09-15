"""Launch the stable JT128 FAST-LIO frontend with Stage 1 keyframe ingestion."""

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
    mapping_share = get_package_share_directory("a2w_fastlio_mapping")

    jt128_config = LaunchConfiguration("jt128_config")
    mapping_config = LaunchConfiguration("mapping_config")
    frontend_map_file = LaunchConfiguration("frontend_map_file")
    save_frontend_pcd = LaunchConfiguration("save_frontend_pcd")
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    arguments = [
        DeclareLaunchArgument(
            "jt128_config",
            default_value=os.path.join(bringup_share, "config", "jt128.yaml"),
            description="Existing Hesai JT128 FAST-LIO parameter file",
        ),
        DeclareLaunchArgument(
            "mapping_config",
            default_value=os.path.join(mapping_share, "config", "mapping.yaml"),
            description="Stage 1 keyframe ingress parameters",
        ),
        DeclareLaunchArgument(
            "frontend_map_file",
            default_value="",
            description="Optional legacy FAST-LIO PCD output path",
        ),
        DeclareLaunchArgument("save_frontend_pcd", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]

    frontend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "jt128_mapping.launch.py")
        ),
        launch_arguments={
            "config_file": jt128_config,
            "map_file": frontend_map_file,
            "save_map": save_frontend_pcd,
            "rviz": rviz,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    ingress = Node(
        package="a2w_fastlio_mapping",
        executable="mapping_ingress_node",
        parameters=[
            mapping_config,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        output="screen",
    )

    return LaunchDescription([*arguments, frontend, ingress])

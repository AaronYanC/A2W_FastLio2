"""Launch Hesai FAST-LIO2 for Unitree A2-W JT128 data."""

import os.path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("a2w_fastlio2_bringup")
    fast_lio_share = get_package_share_directory("fast_lio")

    config_file = LaunchConfiguration("config_file")
    map_file = LaunchConfiguration("map_file")
    rviz = LaunchConfiguration("rviz")
    rviz_cfg = LaunchConfiguration("rviz_cfg")
    save_map = LaunchConfiguration("save_map")
    use_sim_time = LaunchConfiguration("use_sim_time")

    arguments = [
        DeclareLaunchArgument(
            "config_file",
            default_value=os.path.join(bringup_share, "config", "jt128.yaml"),
            description="JT128 FAST-LIO2 parameter file",
        ),
        DeclareLaunchArgument(
            "map_file",
            default_value="",
            description="Absolute PCD output path when map saving is enabled",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_cfg",
            default_value=os.path.join(fast_lio_share, "rviz", "fastlio.rviz"),
        ),
        DeclareLaunchArgument("save_map", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]

    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        parameters=[
            config_file,
            {
                "map_file_path": ParameterValue(map_file, value_type=str),
                "pcd_save.pcd_save_en": ParameterValue(save_map, value_type=bool),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            },
        ],
        output="screen",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([*arguments, fast_lio_node, rviz_node])

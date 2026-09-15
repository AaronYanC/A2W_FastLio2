"""Run the stable JT128 FAST-LIO frontend and the ROS 2 Mapping backend."""

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
    map_share = get_package_share_directory("a2w_fastlio_map")

    jt128_config = LaunchConfiguration("jt128_config")
    keyframe_config = LaunchConfiguration("keyframe_config")
    mapping_topics_config = LaunchConfiguration("mapping_topics_config")
    pose_graph_config = LaunchConfiguration("pose_graph_config")
    scan_context_config = LaunchConfiguration("scan_context_config")
    registration_config = LaunchConfiguration("registration_config")
    loop_validation_config = LaunchConfiguration("loop_validation_config")
    map_config = LaunchConfiguration("map_config")
    bundle_root = LaunchConfiguration("bundle_root")
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    arguments = [
        DeclareLaunchArgument(
            "jt128_config",
            default_value=os.path.join(bringup_share, "config", "jt128.yaml"),
        ),
        DeclareLaunchArgument(
            "keyframe_config",
            default_value=os.path.join(mapping_share, "config", "mapping.yaml"),
        ),
        DeclareLaunchArgument(
            "mapping_topics_config",
            default_value=os.path.join(mapping_share, "config", "mapping_topics.yaml"),
        ),
        DeclareLaunchArgument(
            "pose_graph_config",
            default_value=os.path.join(mapping_share, "config", "pose_graph.yaml"),
        ),
        DeclareLaunchArgument(
            "scan_context_config",
            default_value=os.path.join(mapping_share, "config", "scan_context.yaml"),
        ),
        DeclareLaunchArgument(
            "registration_config",
            default_value=os.path.join(mapping_share, "config", "registration.yaml"),
        ),
        DeclareLaunchArgument(
            "loop_validation_config",
            default_value=os.path.join(mapping_share, "config", "loop_validation.yaml"),
        ),
        DeclareLaunchArgument(
            "map_config",
            default_value=os.path.join(map_share, "config", "map.yaml"),
        ),
        DeclareLaunchArgument(
            "bundle_root",
            default_value="maps",
            description="Writable Map Bundle root; launcher scripts resolve this from the repository",
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
    ingress = Node(
        package="a2w_fastlio_mapping",
        executable="mapping_ingress_node",
        parameters=[
            keyframe_config,
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        output="screen",
    )
    backend = Node(
        package="a2w_fastlio_mapping",
        executable="mapping_backend_node",
        parameters=[
            mapping_topics_config,
            pose_graph_config,
            scan_context_config,
            registration_config,
            loop_validation_config,
            map_config,
            {"bundle_root": ParameterValue(bundle_root, value_type=str)},
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        output="screen",
    )

    # This profile intentionally contains one and only one map->camera_init owner: backend.
    return LaunchDescription([*arguments, frontend, ingress, backend])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    publish_map_to_lio_odom = LaunchConfiguration(
        "publish_map_to_lio_odom"
    )
    lio_tf_topic = LaunchConfiguration("lio_tf_topic")

    package_share = get_package_share_directory(
        "husky_tunnel_bringup"
    )

    params_file = os.path.join(
        package_share,
        "config",
        "lio_sam.yaml",
    )

    rviz_file = os.path.join(
        package_share,
        "config",
        "tunnel_mapping.rviz",
    )

    lio_tf_remappings = [
        ("/tf", lio_tf_topic),
        ("/tf_static", "/a200_0000/tf_static"),
    ]

    robot_tf_remappings = [
        ("/tf", "/a200_0000/tf"),
        ("/tf_static", "/a200_0000/tf_static"),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "publish_map_to_lio_odom",
            default_value="true",
            description=(
                "Publish the static map-to-lio_odom visualization "
                "transform. Disable it when SLAM Toolbox owns that "
                "transform."
            ),
        ),
        DeclareLaunchArgument(
            "lio_tf_topic",
            default_value="/a200_0000/tf",
            description=(
                "Dynamic TF topic used by LIO-SAM. Exploration mode "
                "isolates it to prevent multiple parents for base_link."
            ),
        ),

        # Convert Gazebo's organized VLP-16 cloud into the
        # ring + relative-time layout required by LIO-SAM.
        Node(
            package="husky_tunnel_bringup",
            executable="lio_cloud_adapter.py",
            name="lio_cloud_adapter",
            output="screen",
        ),

        # Genuine LIO-SAM IMU preintegration.
        Node(
            package="lio_sam",
            executable="lio_sam_imuPreintegration",
            name="lio_sam_imuPreintegration",
            parameters=[params_file],
            remappings=lio_tf_remappings,
            output="screen",
        ),

        Node(
            package="lio_sam",
            executable="lio_sam_imageProjection",
            name="lio_sam_imageProjection",
            parameters=[params_file],
            remappings=lio_tf_remappings,
            output="screen",
        ),

        Node(
            package="lio_sam",
            executable="lio_sam_featureExtraction",
            name="lio_sam_featureExtraction",
            parameters=[params_file],
            remappings=lio_tf_remappings,
            output="screen",
        ),

        Node(
            package="lio_sam",
            executable="lio_sam_mapOptimization",
            name="lio_sam_mapOptimization",
            parameters=[params_file],
            remappings=lio_tf_remappings,
            output="screen",
        ),

        # Visualization connection between the mapping frames.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_lio_odom",
            arguments=[
                "0", "0", "0",
                "0", "0", "0",
                "map", "lio_odom",
            ],
            parameters=[{"use_sim_time": True}],
            remappings=robot_tf_remappings,
            condition=IfCondition(publish_map_to_lio_odom),
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="tunnel_lio_sam_rviz",
            arguments=["-d", rviz_file],
            parameters=[{"use_sim_time": True}],
            remappings=robot_tf_remappings,
            output="screen",
        ),
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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

    return LaunchDescription([
        # Remove invalid LiDAR points and add the time field.
        Node(
            package="husky_tunnel_bringup",
            executable="lio_cloud_adapter.py",
            name="lio_cloud_adapter",
            output="screen",
        ),

        # Supply wheel odometry as LIO-SAM's motion initial guess.
        Node(
            package="husky_tunnel_bringup",
            executable="wheel_odom_relay.py",
            name="wheel_odom_relay",
            output="screen",
        ),

        # Connect the mapping and odometry frames.
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
            output="screen",
        ),

        # Stable LIO-SAM mapping nodes.
        Node(
            package="lio_sam",
            executable="lio_sam_imageProjection",
            name="lio_sam_imageProjection",
            parameters=[params_file],
            output="screen",
        ),

        Node(
            package="lio_sam",
            executable="lio_sam_featureExtraction",
            name="lio_sam_featureExtraction",
            parameters=[params_file],
            output="screen",
        ),

        Node(
            package="lio_sam",
            executable="lio_sam_mapOptimization",
            name="lio_sam_mapOptimization",
            parameters=[params_file],
            output="screen",
        ),

        # RViz with the saved project configuration.
        Node(
            package="rviz2",
            executable="rviz2",
            name="tunnel_mapping_rviz",
            arguments=["-d", rviz_file],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
    ])


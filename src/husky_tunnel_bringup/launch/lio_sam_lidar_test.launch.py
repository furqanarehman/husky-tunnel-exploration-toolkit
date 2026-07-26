import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("husky_tunnel_bringup"),
        "config",
        "lio_sam.yaml",
    )

    return LaunchDescription([
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
    ])

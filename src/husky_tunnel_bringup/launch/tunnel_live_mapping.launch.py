import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory(
        "husky_tunnel_bringup"
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                "launch",
                "tunnel_sim.launch.py",
            )
        )
    )

    lio_sam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                "launch",
                "tunnel_lio_sam_full.launch.py",
            )
        ),
        launch_arguments={
            # Keep LIO-SAM's dynamic transforms away from Clearpath's
            # odom -> base_link navigation tree. The static map ->
            # lio_odom connection remains available for 3D-map display.
            "publish_map_to_lio_odom": "true",
            "lio_tf_topic": "/lio_sam/tf",
        }.items(),
    )

    tf_remappings = [
        ("/tf", "/a200_0000/tf"),
        ("/tf_static", "/a200_0000/tf_static"),
    ]

    cloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="tunnel_cloud_to_scan",
        remappings=[
            (
                "cloud_in",
                "/a200_0000/sensors/lidar3d_0/points",
            ),
            ("scan", "/scan"),
            *tf_remappings,
        ],
        parameters=[{
            "use_sim_time": True,
            "target_frame": "base_link",
            "transform_tolerance": 0.10,
            # Exclude most ground and ceiling returns while retaining
            # tunnel walls and navigation obstacles.
            "min_height": 0.10,
            "max_height": 1.50,
            "angle_min": -math.pi,
            "angle_max": math.pi,
            "angle_increment": math.radians(0.4),
            "scan_time": 0.10,
            "range_min": 0.30,
            "range_max": 30.0,
            "use_inf": True,
            "inf_epsilon": 1.0,
        }],
        output="screen",
    )

    slam_params = os.path.join(
        package_share,
        "config",
        "exploration_slam.yaml",
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_params],
        remappings=tf_remappings,
        output="screen",
    )

    return LaunchDescription([
        simulation,
        TimerAction(
            period=12.0,
            actions=[lio_sam, cloud_to_scan],
        ),
        TimerAction(
            period=16.0,
            actions=[slam_toolbox],
        ),
    ])

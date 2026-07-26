#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("husky_tunnel_bringup")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                "launch",
                "tunnel_live_nav2.launch.py",
            )
        )
    )

    backtracking_explorer = Node(
        package="husky_tunnel_bringup",
        executable="tunnel_backtracking_explorer",
        name="tunnel_backtracking_explorer",
        output="screen",
        emulate_tty=True,
        parameters=[
            os.path.join(
                package_share,
                "config",
                "tunnel_backtracking.yaml",
            )
        ],
        remappings=[
            ("/tf", "/a200_0000/tf"),
            ("/tf_static", "/a200_0000/tf_static"),
        ],
    )

    return LaunchDescription([
        base_launch,
        TimerAction(period=20.0, actions=[backtracking_explorer]),
    ])

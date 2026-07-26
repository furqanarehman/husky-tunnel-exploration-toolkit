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

    navigation_and_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                "launch",
                "tunnel_live_nav2.launch.py",
            )
        )
    )

    frontier_explorer = Node(
        package="husky_tunnel_bringup",
        executable="frontier_explorer.py",
        name="frontier_explorer",
        parameters=[{
            "use_sim_time": True,
            "minimum_frontier_cells": 8,
            "minimum_goal_distance": 0.75,
            "maximum_goal_distance": 25.0,
            "goal_standoff": 1.0,
            "goal_clearance": 0.70,
            "goal_search_radius": 1.2,
            "blacklist_radius": 1.2,
            "goal_timeout": 35.0,
            "information_gain_weight": 0.15,
            "distance_score_weight": 1.0,
            "maximum_frontier_samples": 40,
            "free_threshold": 20,
            "dry_run": False,
        }],
        remappings=[
            ("/tf", "/a200_0000/tf"),
            ("/tf_static", "/a200_0000/tf_static"),
        ],
        output="screen",
    )

    return LaunchDescription([
        navigation_and_mapping,
        # Nav2 starts at 22 seconds in the included launch. Give its
        # lifecycle nodes and the first live map time to become ready.
        TimerAction(
            period=32.0,
            actions=[frontier_explorer],
        ),
    ])

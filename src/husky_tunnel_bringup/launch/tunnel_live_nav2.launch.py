import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_share = get_package_share_directory(
        "husky_tunnel_bringup"
    )

    live_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                "launch",
                "tunnel_live_mapping.launch.py",
            )
        )
    )

    nav2_params = os.path.join(
        package_share,
        "config",
        "exploration_nav2.yaml",
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                "launch",
                "tunnel_navigation.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
            "params_file": nav2_params,
            "use_composition": "False",
        }.items(),
    )

    return LaunchDescription([
        live_mapping,
        # Give Gazebo, LIO-SAM, the scan converter, SLAM Toolbox,
        # and the first occupancy grid time to initialize before Nav2.
        TimerAction(
            period=22.0,
            actions=[navigation],
        ),
    ])

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    clearpath_gz = get_package_share_directory("clearpath_gz")
    tunnel_pkg = get_package_share_directory("husky_tunnel_bringup")

    setup_path = os.path.expanduser("~/husky_ws/clearpath")
    tunnel_path = os.path.join(tunnel_pkg, "worlds", "tunnel")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(clearpath_gz, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "world": tunnel_path,
            "setup_path": setup_path,
            "auto_start": "true",
        }.items(),
    )

    husky = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(clearpath_gz, "launch", "robot_spawn.launch.py")
        ),
        launch_arguments={
            "world": "tunnel",
            "setup_path": setup_path,
            "use_sim_time": "true",
            "generate": "true",
            "rviz": "false",
            "x": "1.5",
            "y": "0.0",
            "z": "0.3",
            "yaw": "0.0",
        }.items(),
    )

    return LaunchDescription([gazebo, husky])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [
            FindPackageShare("ros2_grasp_service"),
            "config",
            "grasp_service_config.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="ros2_grasp_service",
                executable="ros2_grasp_service",
                parameters=[config, {"use_sim_time": True}],
                emulate_tty=True,
            )
        ]
    )

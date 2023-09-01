from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_grasp_service',
            executable='ros2_grasp_service',
            emulate_tty=True
        )
    ])
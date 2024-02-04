from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    open_manipulator_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory("open_manipulator_control"), "/launch/control.launch.py"]
        )
    )
    open_manipulator_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("open_manipulator_moveit"),
                "/launch/move_group_with_rviz.launch.py",
            ]
        )
    )

    config = PathJoinSubstitution(
        [
            FindPackageShare("open_manipulator_bringup"),
            "config",
            "grasp_service_config.yaml",
        ]
    )

    grasp_service_node = Node(
        package="ros2_grasp_service",
        executable="ros2_grasp_service",
        parameters=[config, {"use_sim_time": True}],
        emulate_tty=True,
    )

    return LaunchDescription([open_manipulator_control, open_manipulator_moveit, grasp_service_node])

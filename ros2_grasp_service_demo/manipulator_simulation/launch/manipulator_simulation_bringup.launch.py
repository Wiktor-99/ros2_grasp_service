import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    manipulator_simulation = get_package_share_directory("manipulator_simulation")

    manipulator_spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "manipulator", "-topic", "robot_description"],
        output="screen",
        emulate_tty=True,
    )

    open_manipulator_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("open_manipulator_bringup"),
                "/launch/open_manipulator_bringup.launch.py",
            ]
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="world",
                default_value=os.path.join(manipulator_simulation, "worlds", "default_world.world"),
                description="Full path to the world model file to load",
            ),
            DeclareLaunchArgument(name="gui", default_value="true", description='Set to "false" to run headless.'),
            DeclareLaunchArgument(
                name="server",
                default_value="true",
                description='Set to "false" not to run gzserver.',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([manipulator_simulation, "/launch/gazebo.launch.py"])
            ),
            manipulator_spawner,
            open_manipulator_bringup,
        ]
    )

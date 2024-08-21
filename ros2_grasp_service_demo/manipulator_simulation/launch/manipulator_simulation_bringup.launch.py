import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    open_manipulator_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("open_manipulator_bringup"),
                "/launch/open_manipulator_bringup.launch.py",
            ]
        )
    )

    world_argument = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            get_package_share_directory("manipulator_simulation"), "worlds", "default_world.sdf"
        ),
        description="Robot controller to start.",
    )

    gazebo = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"),
        launch_arguments=[
            (
                "gz_args",
                ["-r -v 4 ", LaunchConfiguration("world"), " --physics-engine gz-physics-bullet-featherstone-plugin"],
            )
        ],
    )

    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        arguments=["/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock"],
        output="screen",
    )

    gazebo_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_manipulator",
        arguments=["-name", "manipulator", "-topic", "robot_description"],
        output="screen",
    )

    return LaunchDescription(
        [
            world_argument,
            gazebo,
            ign_bridge,
            gazebo_spawn_robot,
            open_manipulator_bringup,
        ]
    )

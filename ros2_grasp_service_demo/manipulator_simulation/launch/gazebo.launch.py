import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition


def generate_launch_description():
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(get_package_share_directory("manipulator_simulation"), "models")
    manipulator_simulation_package = get_package_share_directory("manipulator_simulation")
    gazebo_ros_package_dir = get_package_share_directory("gazebo_ros")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="world",
                default_value="default_world.world",
                description="Name of the world model file to load in ariadna_simulation_bringup/worlds",
            ),
            DeclareLaunchArgument(name="gui", default_value="true", description='Set to "false" to run headless.'),
            DeclareLaunchArgument(
                name="verbose",
                default_value="false",
                description='Set to "true" to run verbose logging.',
            ),
            DeclareLaunchArgument(
                name="server",
                default_value="true",
                description='Set to "false" not to run gzserver.',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(gazebo_ros_package_dir, "launch", "gzserver.launch.py")]),
                launch_arguments={
                    "world": PathJoinSubstitution(
                        [manipulator_simulation_package, "worlds", LaunchConfiguration("world")]
                    ),
                    "verbose": LaunchConfiguration("verbose"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("server")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(gazebo_ros_package_dir, "launch", "gzclient.launch.py")]),
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
        ]
    )

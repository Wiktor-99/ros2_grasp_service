import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    use_open_manipulator = DeclareLaunchArgument(
        "use_open_manipulator",
        default_value="False",
        description="Launch simulation with open manipulator.",
    )

    use_6dof_manipulator = DeclareLaunchArgument(
        "use_6dof_manipulator",
        default_value="False",
        description="Launch simulation with custom 6DoF manipulator.",
    )
    manipulator_simulation = get_package_share_directory("manipulator_simulation")

    manipulator_spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "manipulator", "-topic", "robot_description"],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(
            PythonExpression(
                [
                    LaunchConfiguration("use_open_manipulator"),
                    " or ",
                    LaunchConfiguration("use_6dof_manipulator"),
                ]
            )
        ),
    )

    open_manipulator_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("open_manipulator_bringup"),
                "/launch/open_manipulator_bringup.launch.py",
            ]
        ),
        condition=IfCondition(LaunchConfiguration("use_open_manipulator")),
    )
    manipulator_6dof_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("manipulator_6dof_bringup"),
                "/launch/manipulator_6dof_bringup.launch.py",
            ]
        ),
        condition=IfCondition(LaunchConfiguration("use_6dof_manipulator")),
    )

    return LaunchDescription(
        [
            use_open_manipulator,
            use_6dof_manipulator,
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
            manipulator_6dof_bringup,
        ]
    )

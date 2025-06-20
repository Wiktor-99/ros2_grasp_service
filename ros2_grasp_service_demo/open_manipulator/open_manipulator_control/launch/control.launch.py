import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory("open_manipulator_description"),
        "urdf",
        "open_manipulator_robot.xacro",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": xacro.process_file(xacro_file).toxml()},
        ],
        emulate_tty=True,
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("open_manipulator_control"),
            "config",
            "controllers.yaml",
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--param-file",
            robot_controllers,
        ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_action_controller",
            "--param-file",
            robot_controllers,
        ],
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            joint_state_broadcaster_spawner,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[joint_trajectory_controller_spawner, gripper_controller_spawner],
                )
            ),
        ]
    )

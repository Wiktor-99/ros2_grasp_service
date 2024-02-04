import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParameter
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("open_manipulator_moveit"),
            "config",
            "moveit.rviz",
        ]
    )

    moveit_config = (
        MoveItConfigsBuilder("open_manipulator", package_name="open_manipulator_moveit")
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("open_manipulator_description"),
                "urdf",
                "open_manipulator_robot.xacro",
            )
        )
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value="true"),
            move_group_node,
            rviz_node,
        ]
    )
